//! Drive `BinaryClient` against a scripted Xi on an in-memory stream.
//!
//! Device identity frames (status, device info) are synthesized with a
//! made-up serial. Per-shot frames follow the Xi's per-shot format and
//! carry no device identity.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io;
use std::rc::Rc;
use std::sync::LazyLock;

use super::*;
use crate::frame::{FrameSplitter, RawFrame};
use crate::protocol::camera::CamConfig;
use crate::protocol::config::{MODE_FULL_OUTDOOR, MODE_LIMITED_FLIGHT};
use crate::seq::XiShotType;

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// Made-up device serial used in every synthesized identity frame.
const FAKE_SERIAL: [u8; 4] = [0x01, 0x23, 0x45, 0x67];

/// Xi DSP status (29 bytes, version byte 0x1C).
fn xi_dsp_status() -> Vec<u8> {
    let mut p = vec![0u8; 29];
    p[0] = 0x1C;
    p[2..6].copy_from_slice(&FAKE_SERIAL);
    p
}

/// Xi AVR status (25 bytes).
fn xi_avr_status() -> Vec<u8> {
    let mut p = vec![0u8; 25];
    p[0] = 0x18;
    p[4..8].copy_from_slice(&FAKE_SERIAL);
    p
}

/// Xi DSP device info (99 bytes, model slot at the end).
fn xi_dsp_dev_info() -> Vec<u8> {
    let mut p = vec![0u8; 99];
    p[0] = 0x62;
    p[14..18].copy_from_slice(&FAKE_SERIAL);
    p[83..97].copy_from_slice(b"0.38 FS Xi    ");
    p
}

/// Xi AVR device info (75 bytes, Mevo slot layout).
fn xi_avr_dev_info() -> Vec<u8> {
    let mut p = vec![0u8; 75];
    p[0] = 0x4A;
    p[5..9].copy_from_slice(&FAKE_SERIAL);
    p[27..41].copy_from_slice(b"3.18    FSCOPE");
    p[43..54].copy_from_slice(b"Jan  1 2020");
    p[59..67].copy_from_slice(b"12:00:00");
    p
}

/// One Xi shot as the device pushes it: TRIGGER, E8, D4, ED, EF,
/// PROCESSED, IDLE, MODE_ACK. Returns (frames, ED sent in reply to 0x6D).
///
/// D4 back/side 7127/-652 rpm, E8 backspin 7157 rpm.
fn xi_shot() -> (Vec<(u8, Vec<u8>)>, Vec<u8>) {
    let shot = crate::testing::Shot::xi_full_swing();
    let frames = shot
        .frames
        .into_iter()
        .map(|f| (f.type_id, f.payload))
        .collect();
    (frames, shot.club_reply)
}

#[derive(Default)]
struct XiState {
    splitter: FrameSplitter,
    /// Frames the client sent, in order.
    received: Vec<RawFrame>,
    /// Wire bytes waiting to be read by the client.
    outbox: VecDeque<u8>,
    /// ED returned for the next ShotResultReq.
    shot_ed: Option<Vec<u8>>,
    /// Armed by B0 `[01 01]`, disarmed by B0 `[01 00]`.
    armed: bool,
    /// DSP power mode (param 0x01 on the DSP); 0 until written.
    dsp_power_mode: u8,
    /// Frames (dest, type, payload) refused once with a 0x94 instead of
    /// their normal reply.
    refuse: Vec<(BusAddr, u8, Vec<u8>)>,
    /// Frames (dest, type, payload) left unanswered once.
    ignore: Vec<(BusAddr, u8, Vec<u8>)>,
    /// AVR status polls refused (0x94 `[02 30 2A]`) before the first
    /// answer, as after power-on.
    avr_boot_nacks: usize,
    /// Leave the DSP query unanswered instead of refusing it.
    ignore_dsp_query: bool,
    /// The test's time since [`t0`]: the clock the client is polled at.
    elapsed: Duration,
}

impl XiState {
    fn push(&mut self, src: BusAddr, type_id: u8, payload: Vec<u8>) {
        let frame = RawFrame {
            dest: BusAddr::App,
            src,
            type_id,
            payload,
        };
        self.outbox.extend(frame.encode());
    }

    /// Answer one request as the scripted Xi. Nothing addressed to the
    /// PI is answered; the DSP query (0x48) is refused, as the Xi does,
    /// unless `ignore_dsp_query` leaves it to its timeout.
    fn respond(&mut self, f: &RawFrame) {
        let p = &f.payload;
        if let Some(i) = self
            .refuse
            .iter()
            .position(|(d, t, pl)| *d == f.dest && *t == f.type_id && pl == p)
        {
            self.refuse.remove(i);
            self.push(f.dest, 0x94, vec![0x02, f.dest as u8, f.type_id & 0x7F]);
            return;
        }
        if let Some(i) = self
            .ignore
            .iter()
            .position(|(d, t, pl)| *d == f.dest && *t == f.type_id && pl == p)
        {
            self.ignore.remove(i);
            return;
        }
        match (f.dest, f.type_id) {
            (BusAddr::Dsp, 0xAA) => self.push(BusAddr::Dsp, 0xAA, xi_dsp_status()),
            (BusAddr::Avr, 0xAA) if self.avr_boot_nacks > 0 => {
                self.avr_boot_nacks -= 1;
                self.push(BusAddr::Avr, 0x94, vec![0x02, 0x30, 0x2A]);
            }
            (BusAddr::Avr, 0xAA) => self.push(BusAddr::Avr, 0xAA, xi_avr_status()),
            (BusAddr::Dsp, 0x48) if !self.ignore_dsp_query => {
                self.push(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
            }
            (BusAddr::Dsp, 0xBF) => {
                self.push(BusAddr::Dsp, 0x95, vec![0x02, 0x40, 0x3F]);
                if p[3] == 0x01 && p[6] != self.dsp_power_mode {
                    let req = format!("  REQ Power Mode : {} ({}) \0", p[6], self.dsp_power_mode);
                    let new = format!("  NEW Power Mode : {} \0", p[6]);
                    self.push(BusAddr::Dsp, 0xE3, req.into_bytes());
                    self.push(BusAddr::Dsp, 0xE3, new.into_bytes());
                    self.dsp_power_mode = p[6];
                }
            }
            (BusAddr::Dsp, 0x67) => self.push(BusAddr::Dsp, 0xE7, xi_dsp_dev_info()),
            (BusAddr::Avr, 0x67) => self.push(BusAddr::Avr, 0xE7, xi_avr_dev_info()),
            (BusAddr::Dsp, 0xFD) => {
                let mut r = vec![0u8; 34];
                r[0] = 0x21;
                r[1] = p[1];
                self.push(BusAddr::Dsp, 0xFD, r);
            }
            (dest @ (BusAddr::Dsp | BusAddr::Avr), 0x21) => {
                let mut r = vec![0u8; 69];
                r[0] = 0x44;
                self.push(dest, 0xA0, r);
            }
            (BusAddr::Avr, 0x23) => self.push(BusAddr::Avr, 0x94, vec![0x02, 0x30, 0x23]),
            (BusAddr::Avr, 0xBE) => {
                self.push(BusAddr::Avr, 0xBF, vec![0x06, 0x00, 0x00, p[3], 0, 0, 0]);
            }
            (BusAddr::Avr, 0xBF) => self.push(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x3F]),
            (BusAddr::Avr, 0xD0) => self.push(BusAddr::Avr, 0xD1, vec![0u8; 171]),
            (BusAddr::Avr, 0xA5 | 0xA4) => self.push(BusAddr::Avr, f.type_id, p.clone()),
            (BusAddr::Avr, 0xB0) => {
                self.push(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
                if p == &[0x01, 0x01] {
                    self.armed = true;
                    self.push(BusAddr::Avr, 0xE3, b"\n\rARMED Xi\0".to_vec());
                    self.push(BusAddr::Avr, 0xE3, b" INDOOR\0".to_vec());
                } else if self.armed {
                    self.armed = false;
                    self.push(BusAddr::Avr, 0xE3, b"  ARMED CANCELLED\0".to_vec());
                    self.push(BusAddr::Avr, 0xB1, vec![0x02, 0x00, 0x00]);
                }
            }
            (BusAddr::Avr, 0x6D) => {
                if let Some(ed) = self.shot_ed.take() {
                    self.push(BusAddr::Avr, 0xED, ed);
                }
            }
            _ => {}
        }
    }

    fn sent(&self, dest: BusAddr, type_id: u8) -> Vec<&RawFrame> {
        self.received
            .iter()
            .filter(|f| f.dest == dest && f.type_id == type_id)
            .collect()
    }
}

/// In-memory stream backed by a shared `XiState`.
struct XiStream(Rc<RefCell<XiState>>);

impl Read for XiStream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut st = self.0.borrow_mut();
        if st.outbox.is_empty() {
            return Err(io::ErrorKind::WouldBlock.into());
        }
        let n = buf.len().min(st.outbox.len());
        for b in buf.iter_mut().take(n) {
            *b = st.outbox.pop_front().unwrap();
        }
        Ok(n)
    }
}

impl Write for XiStream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut st = self.0.borrow_mut();
        for wire in st.splitter.feed(buf) {
            let frame = RawFrame::parse(&wire).expect("client sent a bad frame");
            st.respond(&frame);
            st.received.push(frame);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// The tests' time origin: the clock is read once, and a test steps time
/// by adding to it.
fn t0() -> Instant {
    static T0: LazyLock<Instant> = LazyLock::new(Instant::now);
    *T0
}

/// The scripted Xi's time: [`t0`] plus what the test has stepped.
fn now(client: &BinaryClient<XiStream>) -> Instant {
    t0() + client.conn.stream().0.borrow().elapsed
}

fn advance(client: &BinaryClient<XiStream>, d: Duration) {
    client.conn.stream().0.borrow_mut().elapsed += d;
}

/// One poll at the scripted Xi's time.
fn poll(client: &mut BinaryClient<XiStream>) -> Option<BinaryEvent> {
    let at = now(client);
    client.poll_at(at).expect("poll failed")
}

/// Poll until `pred` matches an event, collecting every event seen; a
/// poll that brings nothing moves the clock 1 ms (10 s at most).
fn poll_until(
    client: &mut BinaryClient<XiStream>,
    events: &mut Vec<BinaryEvent>,
    pred: impl Fn(&BinaryEvent) -> bool,
) {
    let give_up = now(client) + Duration::from_secs(10);
    while now(client) < give_up {
        match poll(client) {
            Some(event) => {
                let done = pred(&event);
                events.push(event);
                if done {
                    return;
                }
            }
            None => advance(client, Duration::from_millis(1)),
        }
    }
    panic!("event not seen; got {events:?}");
}

#[test]
fn xi_handshake_configure_arm_and_shot() {
    let state = Rc::new(RefCell::new(XiState {
        ignore_dsp_query: true,
        ..XiState::default()
    }));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();

    // Handshake: the DSP query goes unanswered, so this includes the
    // DSP query timeout.
    client.handshake();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    let Some(BinaryEvent::Handshake(h)) = events.pop() else {
        unreachable!()
    };
    assert_eq!(h.model, DeviceModel::Xi);
    assert_eq!(client.model(), DeviceModel::Xi);
    assert!(h.pi.is_none());
    assert!(h.dsp.hw_info.is_none());
    assert_eq!(h.dsp.dev_info.text, "0.38 FS Xi");
    assert!(h.avr.if_cal.is_some());
    assert!(h.avr.avr_config.is_none());

    // Configure + arm. The camera config is a no-op on the Xi.
    client.configure_xi(XiSettings::default());
    client.configure_cam(CamConfig::fusion_preset());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    let configured = events
        .iter()
        .filter(|e| matches!(e, BinaryEvent::Configured))
        .count();
    assert_eq!(configured, 2);
    assert!(client.is_armed());

    // Shot: D4/ED/EF arrive before PROCESSED.
    let (frames, ed) = xi_shot();
    {
        let mut st = state.borrow_mut();
        st.shot_ed = Some(ed);
        for (type_id, payload) in frames {
            st.push(BusAddr::Avr, type_id, payload);
        }
    }
    events.clear();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::ShotComplete(_))
    });
    let data: Vec<&ShotDatum> = events
        .iter()
        .filter_map(|e| match e {
            BinaryEvent::ShotDatum(d) => Some(d),
            _ => None,
        })
        .collect();
    // " INDOOR" text from the arm may still precede the trigger.
    let first = events
        .iter()
        .find(|e| !matches!(e, BinaryEvent::Message(_)))
        .unwrap();
    assert!(matches!(first, BinaryEvent::Trigger));
    assert!(matches!(data[0], ShotDatum::FlightV1(e8) if e8.backspin_rpm == 7157));
    assert!(
        matches!(data[1], ShotDatum::Flight(d4) if d4.backspin_rpm == 7127 && d4.sidespin_rpm == -652)
    );
    assert!(matches!(data[2], ShotDatum::Club(_)));
    assert!(matches!(data[3], ShotDatum::Spin(_)));
    let Some(BinaryEvent::ShotComplete(shot)) = events.last() else {
        unreachable!()
    };
    let flight = shot.flight.as_ref().expect("D4 in shot data");
    assert!((flight.launch_elevation - 21.0).abs() < 0.1);
    assert!(shot.club.is_some());
    assert!(shot.spin.is_some());
    assert!(client.is_armed());

    let st = state.borrow();
    // Nothing is ever sent to the PI.
    assert!(st.received.iter().all(|f| f.dest != BusAddr::Pi));
    // Mevo-only handshake requests are not sent.
    for type_id in [0x23, 0xD2, 0x9B, 0xDE, 0x82, 0x83, 0x90] {
        assert!(
            st.received.iter().all(|f| f.type_id != type_id),
            "unexpected 0x{type_id:02X}"
        );
    }
    // One DSP write: power mode Normal.
    let dsp_bf = st.sent(BusAddr::Dsp, 0xBF);
    assert_eq!(dsp_bf.len(), 1);
    assert_eq!(dsp_bf[0].payload, hex("06000001000001"));
    assert_eq!(st.dsp_power_mode, 1);
    // The Xi configuration: 19 param writes, param 0x02 read
    // after the handshake's 0x0C and 0x0D, one radar cal.
    assert_eq!(st.sent(BusAddr::Avr, 0xBF).len(), 19);
    let reads: Vec<u8> = st
        .sent(BusAddr::Avr, 0xBE)
        .iter()
        .map(|f| f.payload[3])
        .collect();
    assert_eq!(reads, [0x0C, 0x0D, 0x02]);
    let cal = st.sent(BusAddr::Avr, 0xA4);
    assert_eq!(cal.len(), 1);
    assert_eq!(cal[0].payload, [0x06, 0x09, 0x86, 0x00, 0x19, 0x00, 0x00]);
    // Mode 8 first, then detection mode 9 (limited flight) twice.
    let modes: Vec<u8> = st
        .sent(BusAddr::Avr, 0xA5)
        .iter()
        .map(|f| f.payload[2])
        .collect();
    assert_eq!(modes, [0x08, MODE_LIMITED_FLIGHT, MODE_LIMITED_FLIGHT]);
    // Armed once at startup and re-armed after the shot.
    let arms = st
        .sent(BusAddr::Avr, 0xB0)
        .iter()
        .filter(|f| f.payload == [0x01, 0x01])
        .count();
    assert_eq!(arms, 2);
    assert_eq!(st.sent(BusAddr::Avr, 0x6D).len(), 1);
}

/// Frames sent, as "DEST TYPE payload".
fn tx_lines(frames: &[RawFrame]) -> Vec<String> {
    use std::fmt::Write as _;
    frames
        .iter()
        .map(|f| {
            let hex = f.payload.iter().fold(String::new(), |mut s, b| {
                let _ = write!(s, "{b:02x}");
                s
            });
            format!("{} {:02X} {hex}", f.dest, f.type_id)
                .trim_end()
                .to_string()
        })
        .collect()
}

/// The Xi connection: DSP sync ending with the DSP power-mode write
/// right after the DSP config query (0x21) is answered, then AVR sync,
/// the configuration (its first frame is the AVR Altitude write) and the
/// arm.
#[test]
fn xi_connect_sequence_wakes_the_dsp_after_its_config_query() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, BinaryEvent::StepRefused(_)))
    );
    let st = state.borrow();
    let lines = tx_lines(&st.received);
    assert_eq!(
        lines[..18],
        [
            "DSP AA 0101",
            "DSP 48",
            "DSP 67",
            "DSP FD 0100",
            "DSP FD 0108",
            "DSP FD 0109",
            "DSP 21",
            "DSP BF 06000001000001",
            "AVR AA 0101",
            "AVR AA 0101",
            "AVR 67",
            "AVR 67",
            "AVR BE 0300000c",
            "AVR BE 0300000d",
            "AVR 21",
            "AVR D0 020008",
            "AVR BF 06000001000000",
            "AVR A5 020008",
        ]
    );
    // After the handshake the DSP gets only the arm's status poll.
    assert!(
        lines[18..]
            .iter()
            .filter(|l| l.starts_with("DSP "))
            .all(|l| l == "DSP AA 0101")
    );
    assert_eq!(lines.last().unwrap(), "AVR B0 0101");
    assert_eq!(st.dsp_power_mode, 1);
}

/// Run handshake, configure and arm against `state`, returning the
/// events seen.
fn connect_and_arm(state: &Rc<RefCell<XiState>>) -> Vec<BinaryEvent> {
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    assert!(client.is_armed());
    events
}

/// A 0x94 refusing a configuration step, of each command type the Xi
/// configuration sends, is reported as `StepRefused` (with the 0x94's
/// hex) and the configuration carries on to the arm, sending every
/// frame it sends without the refusal.
#[test]
fn xi_config_nack_of_each_step_type_continues_to_the_arm() {
    let baseline = {
        let state = Rc::new(RefCell::new(XiState::default()));
        connect_and_arm(&state);
        tx_lines(&state.borrow().received)
    };
    for (type_id, payload) in [
        (0xA5, "020008"),
        (0xA4, "06098600190000"),
        (0xBE, "03000002"),
        (0xBF, "06000001000000"),
        (0xB0, "0100"),
    ] {
        let state = Rc::new(RefCell::new(XiState {
            refuse: vec![(BusAddr::Avr, type_id, hex(payload))],
            ..XiState::default()
        }));
        let events = connect_and_arm(&state);
        let refused: Vec<&Envelope> = events
            .iter()
            .filter_map(|e| match e {
                BinaryEvent::StepRefused(env) => Some(env),
                _ => None,
            })
            .collect();
        assert_eq!(refused.len(), 1, "0x{type_id:02X}: {events:?}");
        assert_eq!((refused[0].src, refused[0].type_id), (BusAddr::Avr, 0x94));
        assert_eq!(refused[0].raw, [0x02, 0x30, type_id & 0x7F]);
        // Reported before the configuration completes.
        let pos = |pred: fn(&BinaryEvent) -> bool| events.iter().position(pred).unwrap();
        assert!(
            pos(|e| matches!(e, BinaryEvent::StepRefused(_)))
                < pos(|e| matches!(e, BinaryEvent::Configured))
        );
        let st = state.borrow();
        assert!(st.refuse.is_empty(), "0x{type_id:02X} never sent");
        assert_eq!(tx_lines(&st.received), baseline, "0x{type_id:02X}");
    }
}

/// A NACK on the last step still lets the configuration complete:
/// `StepRefused` first, `Configured` on the next poll.
#[test]
fn xi_config_nack_of_the_last_step_completes() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    // Mode change: its last frame is BF param 0x20 = 0.0.
    events.clear();
    state.borrow_mut().refuse = vec![(BusAddr::Avr, 0xBF, hex("080000200000000000"))];
    client.configure_xi(XiSettings::default());
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Configured)
    });
    let n = events.len();
    assert!(
        matches!(&events[n - 2], BinaryEvent::StepRefused(env) if env.raw == [0x02, 0x30, 0x3F])
    );
    assert!(state.borrow().refuse.is_empty());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
}

/// A DSP that refuses the power-mode write does not stop the
/// handshake: `StepRefused` with the DSP's 0x94, then the AVR sync
/// and the arm as normal.
#[test]
fn xi_dsp_power_mode_nack_continues_to_the_arm() {
    let state = Rc::new(RefCell::new(XiState {
        refuse: vec![(BusAddr::Dsp, 0xBF, hex("06000001000001"))],
        ..XiState::default()
    }));
    let events = connect_and_arm(&state);
    let refused: Vec<&Envelope> = events
        .iter()
        .filter_map(|e| match e {
            BinaryEvent::StepRefused(env) => Some(env),
            _ => None,
        })
        .collect();
    assert_eq!(refused.len(), 1);
    assert_eq!((refused[0].src, refused[0].type_id), (BusAddr::Dsp, 0x94));
    assert_eq!(refused[0].raw, [0x02, 0x40, 0x3F]);
    let st = state.borrow();
    assert!(st.refuse.is_empty());
    assert_eq!(st.dsp_power_mode, 0);
    assert!(!st.sent(BusAddr::Avr, 0xB0).is_empty());
}

/// A DSP that never answers the power-mode write does not stop the
/// handshake: after [`seq::DSP_QUERY_TIMEOUT`] it carries on to the AVR
/// sync and the arm, with `power_mode_unanswered` set and no
/// `StepRefused`.
#[test]
fn xi_dsp_power_mode_unanswered_continues_to_the_arm() {
    let state = Rc::new(RefCell::new(XiState {
        ignore: vec![(BusAddr::Dsp, 0xBF, hex("06000001000001"))],
        ..XiState::default()
    }));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    client.configure_xi(XiSettings::default());
    client.arm();
    let start = now(&client);
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    assert!(now(&client) - start >= seq::DSP_QUERY_TIMEOUT);
    let handshake = events
        .iter()
        .find_map(|e| match e {
            BinaryEvent::Handshake(h) => Some(h),
            _ => None,
        })
        .expect("handshake");
    assert!(handshake.dsp.power_mode_unanswered);
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, BinaryEvent::StepRefused(_)))
    );
    let st = state.borrow();
    assert!(st.ignore.is_empty());
    assert_eq!(st.dsp_power_mode, 0);
    assert_eq!(st.sent(BusAddr::Dsp, 0xBF).len(), 1);
    assert!(!st.sent(BusAddr::Avr, 0xB0).is_empty());
}

/// After power-on the Xi's AVR refuses status polls for a few seconds.
/// The handshake re-polls about once a second until it answers, then
/// completes; nothing is reported as a refused step.
#[test]
fn xi_avr_refusing_status_polls_is_re_polled() {
    let state = Rc::new(RefCell::new(XiState {
        avr_boot_nacks: 2,
        ..XiState::default()
    }));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    let start = now(&client);
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    let elapsed = now(&client) - start;
    assert!(elapsed >= 2 * seq::STATUS_REPOLL_INTERVAL, "{elapsed:?}");
    assert!(elapsed < 4 * seq::STATUS_REPOLL_INTERVAL, "{elapsed:?}");
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, BinaryEvent::StepRefused(_)))
    );
    let st = state.borrow();
    let avr: Vec<String> = tx_lines(&st.received)
        .into_iter()
        .filter(|l| l.starts_with("AVR "))
        .collect();
    // Refused, re-polled, refused, re-polled, answered; then the
    // second poll and the rest of the AVR sync.
    assert_eq!(
        avr[..6],
        [
            "AVR AA 0101",
            "AVR AA 0101",
            "AVR AA 0101",
            "AVR AA 0101",
            "AVR 67",
            "AVR 67"
        ]
    );
    assert_eq!(client.model(), DeviceModel::Xi);
}

/// Handshake, configure and arm a client on the scripted Xi.
fn armed_xi() -> (BinaryClient<XiStream>, Rc<RefCell<XiState>>) {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    (client, state)
}

/// Frames sent, as (type, payload).
type Sent = Vec<(u8, Vec<u8>)>;

/// Deliver `xi_shot()` one frame at a time until `ShotComplete`.
/// Returns, for each received frame, its type and what was sent while
/// handling it (status polls left out); and the last frame sent at the
/// moment each E8/D4 datum event was returned.
fn shot_tx_per_frame(
    client: &mut BinaryClient<XiStream>,
    state: &Rc<RefCell<XiState>>,
) -> (Vec<(u8, Sent)>, Sent) {
    let (frames, ed) = xi_shot();
    state.borrow_mut().shot_ed = Some(ed);
    let mut per_frame = Vec::new();
    let mut at_datum = Vec::new();
    let mut complete = false;
    for (type_id, payload) in frames {
        let before = state.borrow().received.len();
        state.borrow_mut().push(BusAddr::Avr, type_id, payload);
        // Each poll handles at most one frame; 50 cover this frame and
        // every reply the scripted device pushes for it.
        for _ in 0..50 {
            match poll(client) {
                Some(BinaryEvent::ShotDatum(ShotDatum::FlightV1(_) | ShotDatum::Flight(_))) => {
                    let st = state.borrow();
                    let last = st.received.last().unwrap();
                    at_datum.push((last.type_id, last.payload.clone()));
                }
                Some(BinaryEvent::ShotComplete(_)) => complete = true,
                _ => {}
            }
        }
        let tx = state.borrow().received[before..]
            .iter()
            .filter(|f| f.type_id != 0xAA)
            .map(|f| (f.type_id, f.payload.clone()))
            .collect();
        per_frame.push((type_id, tx));
    }
    assert!(complete, "no ShotComplete");
    (per_frame, at_datum)
}

/// Every E8 and D4 from the Xi is acked with 0x95 the moment it is
/// received: one ack per frame, sent before the frame's event is
/// returned and before anything else of the post-shot cycle.
#[test]
fn xi_flight_results_are_acked_on_arrival() {
    let (mut client, state) = armed_xi();
    let (per_frame, at_datum) = shot_tx_per_frame(&mut client, &state);
    assert_eq!(
        per_frame,
        [
            (0xE5, vec![]),
            (0xE8, vec![(0x95, vec![0x02, 0x10, 0x68])]),
            (0xD4, vec![(0x95, vec![0x02, 0x10, 0x54])]),
            (0xED, vec![]),
            (0xEF, vec![]),
            (0xE5, vec![(0x69, vec![]), (0x69, vec![])]),
            (0xE5, vec![(0x6D, vec![]), (0xB0, vec![0x01, 0x01])]),
            (0xB1, vec![]),
        ]
    );
    assert_eq!(
        at_datum,
        [
            (0x95, vec![0x02, 0x10, 0x68]),
            (0x95, vec![0x02, 0x10, 0x54])
        ]
    );
    // The acks go to the AVR.
    let st = state.borrow();
    assert_eq!(st.sent(BusAddr::Avr, 0x95).len(), 2);
    assert_eq!(st.received.iter().filter(|f| f.type_id == 0x95).count(), 2);
}

/// The same shot on a Mevo sends no 0x95: only the post-shot cycle
/// (0x69 x2, 0x6D, B0 `[01 01]`). The scripted device is the Xi with
/// the handshake outcome relabeled, so only the model differs.
#[test]
fn mevo_shot_sends_no_result_ack() {
    let (mut client, state) = armed_xi();
    client.device.as_mut().unwrap().model = DeviceModel::Mevo;
    let before = state.borrow().received.len();
    let (per_frame, _) = shot_tx_per_frame(&mut client, &state);
    assert_eq!(
        per_frame,
        [
            (0xE5, vec![]),
            (0xE8, vec![]),
            (0xD4, vec![]),
            (0xED, vec![]),
            (0xEF, vec![]),
            (0xE5, vec![(0x69, vec![]), (0x69, vec![])]),
            (0xE5, vec![(0x6D, vec![]), (0xB0, vec![0x01, 0x01])]),
            (0xB1, vec![]),
        ]
    );
    let st = state.borrow();
    assert!(st.received[before..].iter().all(|f| f.type_id != 0x95));
}

/// Results arriving before PROCESSED (the order both the Mevo+ and the
/// Xi use) are yielded and land in `ShotComplete`, whether the frames
/// arrive one per read or in one burst.
#[test]
fn results_before_processed_are_kept() {
    for burst in [false, true] {
        let state = Rc::new(RefCell::new(XiState::default()));
        let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
        let (frames, ed) = xi_shot();
        {
            let mut st = state.borrow_mut();
            st.shot_ed = Some(ed);
            for (type_id, payload) in frames {
                st.push(BusAddr::Avr, type_id, payload);
            }
        }
        let mut events = Vec::new();
        if burst {
            poll_until(&mut client, &mut events, |e| {
                matches!(e, BinaryEvent::ShotComplete(_))
            });
        } else {
            // One frame per read: drain the outbox frame by frame.
            let wire: Vec<u8> = state.borrow_mut().outbox.drain(..).collect();
            let mut splitter = FrameSplitter::new();
            let mut pending: VecDeque<Vec<u8>> = splitter.feed(&wire).into();
            let mut polls = 0;
            while !matches!(events.last(), Some(BinaryEvent::ShotComplete(_))) {
                polls += 1;
                assert!(polls < 10_000, "no ShotComplete: {events:?}");
                if state.borrow().outbox.is_empty()
                    && let Some(f) = pending.pop_front()
                {
                    state.borrow_mut().outbox.extend(f);
                }
                if let Some(e) = poll(&mut client) {
                    events.push(e);
                }
            }
        }
        let flights = events
            .iter()
            .filter(|e| matches!(e, BinaryEvent::ShotDatum(ShotDatum::Flight(_))))
            .count();
        assert_eq!(flights, 1, "burst={burst}");
        let Some(BinaryEvent::ShotComplete(shot)) = events.last() else {
            unreachable!()
        };
        assert!(shot.flight.is_some(), "burst={burst}");
        assert!(shot.club.is_some(), "burst={burst}");
        assert!(shot.spin.is_some(), "burst={burst}");
    }
}

/// A slow IDLE after PROCESSED (a Mevo in full-outdoor mode takes about
/// 3 s) does not trigger the drain timeout: the re-arm waits for IDLE
/// and the 0x6D result request, even when the results arrived before
/// PROCESSED.
#[test]
fn slow_idle_after_early_results_waits_for_idle() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let (mut frames, ed) = xi_shot();
    // Hold back IDLE and MODE_ACK; everything up to PROCESSED goes now.
    let tail = frames.split_off(frames.len() - 2);
    {
        let mut st = state.borrow_mut();
        st.shot_ed = Some(ed);
        for (type_id, payload) in frames {
            st.push(BusAddr::Avr, type_id, payload);
        }
    }
    let mut events = Vec::new();
    // Past the 3 s drain timeout before IDLE arrives.
    let idle_at = now(&client) + Duration::from_millis(3500);
    while now(&client) < idle_at {
        if let Some(e) = poll(&mut client) {
            events.push(e);
        } else {
            advance(&client, Duration::from_millis(5));
        }
    }
    {
        let st = state.borrow();
        assert!(
            st.sent(BusAddr::Avr, 0xB0).is_empty(),
            "ARM sent before IDLE"
        );
        assert!(st.sent(BusAddr::Avr, 0x6D).is_empty());
    }
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, BinaryEvent::ShotComplete(_)))
    );

    {
        let mut st = state.borrow_mut();
        for (type_id, payload) in tail {
            st.push(BusAddr::Avr, type_id, payload);
        }
    }
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::ShotComplete(_))
    });
    let st = state.borrow();
    let pos = |type_id: u8| {
        st.received
            .iter()
            .position(|f| f.dest == BusAddr::Avr && f.type_id == type_id)
            .unwrap_or_else(|| panic!("0x{type_id:02X} not sent"))
    };
    assert!(pos(0x6D) < pos(0xB0), "ARM sent before the result request");
    let Some(BinaryEvent::ShotComplete(shot)) = events.last() else {
        unreachable!()
    };
    assert!(shot.flight.is_some());
    assert!(shot.club.is_some());
}

/// A selected Xi mode is written as-is in both A5 mode sets after the
/// connect-time mode 8.
#[test]
fn xi_full_outdoor_mode_is_written() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    client.configure_xi(XiSettings {
        mode: MODE_FULL_OUTDOOR,
        ..XiSettings::default()
    });
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    let st = state.borrow();
    let modes: Vec<&[u8]> = st
        .sent(BusAddr::Avr, 0xA5)
        .iter()
        .map(|f| f.payload.as_slice())
        .collect();
    assert_eq!(modes, [[0x02, 0x00, 0x08]; 3]);
}

/// A second Xi configuration on the connection disarms the armed
/// device, sends the mode block only, and is armed again.
#[test]
fn xi_mode_change_disarms_and_sends_the_mode_block() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    let mut events = Vec::new();
    client.handshake();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    let before = state.borrow().received.len();

    events.clear();
    client.configure_xi(XiSettings {
        shot_type: XiShotType::Pitching,
        ..XiSettings::default()
    });
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    assert!(matches!(events[0], BinaryEvent::Disarmed));
    assert!(events.iter().any(|e| matches!(e, BinaryEvent::Configured)));
    assert!(client.is_armed());

    let st = state.borrow();
    let wire: Vec<(u8, Vec<u8>)> = st.received[before..]
        .iter()
        .filter(|f| f.dest == BusAddr::Avr && f.type_id != 0xAA)
        .map(|f| (f.type_id, f.payload.clone()))
        .collect();
    let bf = |id: u8, v: [u8; 3]| (0xBF, vec![0x06, 0x00, 0x00, id, v[0], v[1], v[2]]);
    let zero = (0xBF, vec![0x08, 0x00, 0x00, 0x20, 0, 0, 0, 0, 0]);
    let one = (
        0xBF,
        vec![0x08, 0x00, 0x00, 0x28, 0x00, 0x01, 0x40, 0x00, 0x00],
    );
    let gate = (0xB0, vec![0x01, 0x00]);
    assert_eq!(
        wire,
        [
            gate.clone(),
            bf(0x08, [0, 0, 1]),
            gate.clone(),
            bf(0x16, [0, 0, 2]),
            zero.clone(),
            bf(0x07, [0x00, 0x54, 0x48]),
            (0xA5, vec![0x02, 0x00, MODE_LIMITED_FLIGHT]),
            gate,
            one,
            bf(0x16, [0, 0, 2]),
            zero,
            (0xB0, vec![0x01, 0x01]),
        ]
    );
}

#[test]
fn xi_keepalive_skips_pi() {
    let state = Rc::new(RefCell::new(XiState::default()));
    let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
    client.set_keepalive_interval(Duration::ZERO);
    let mut events = Vec::new();
    client.handshake();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Handshake(_))
    });
    client.configure_xi(XiSettings::default());
    client.arm();
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Armed)
    });
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Keepalive(_))
    });
    poll_until(&mut client, &mut events, |e| {
        matches!(e, BinaryEvent::Keepalive(_))
    });
    assert!(
        state
            .borrow()
            .received
            .iter()
            .all(|f| f.dest != BusAddr::Pi)
    );
}
