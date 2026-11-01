//! Test support: a scripted in-memory FlightScope, result frames recorded
//! from an Xi and payload encoders. Behind the `testing` feature.
//!
//! [`FakeDevice`] is a `Read + Write` stream to hand to
//! [`BinaryConnection::new`]. It serves the frames queued with
//! [`FakeDevice::push`] and [`FakeDevice::push_shot`] and answers the
//! client's requests itself:
//!
//! - status polls (0xAA) to the DSP, the AVR and (on [`FakeDevice::new`])
//!   the camera Pi, with made-up identity bytes, so
//!   [`BinaryClient::arm`](crate::BinaryClient::arm) completes;
//! - the arm (B0 `[01 01]`) with an ack and the `ARMED` text; a config gate
//!   (B0 `[01 00]`) with an ack, and a disarm when armed;
//! - each shot result request (0x6D) with the club result (0xED) of the
//!   oldest shot whose request is still unanswered;
//! - with [`FakeDevice::xi`], also the Xi's handshake and configuration,
//!   with made-up identity frames.
//!
//! [`FakeDevice::new`] answers nothing else of the handshake, so
//! [`BinaryClient::handshake`](crate::BinaryClient::handshake) completes on
//! [`FakeDevice::xi`] only.
//!
//! ```
//! use ironsight::testing::{FakeDevice, Shot};
//! use ironsight::{BinaryClient, BinaryEvent};
//!
//! let device = FakeDevice::new();
//! device.push_shot(&Shot::xi_full_swing());
//! let mut client = BinaryClient::new(device.connection());
//! let completed = loop {
//!     match client.poll().unwrap() {
//!         Some(BinaryEvent::ShotComplete(shot)) => break shot,
//!         Some(_) => {}
//!         None => std::thread::sleep(std::time::Duration::from_millis(1)),
//!     }
//! };
//! assert!(completed.flight.is_some());
//! ```

pub mod fixtures;

use std::cell::RefCell;
use std::collections::VecDeque;
use std::io;
use std::rc::Rc;

use crate::addr::BusAddr;
use crate::conn::BinaryConnection;
use crate::frame::{FrameSplitter, RawFrame};
use crate::protocol::{
    TYPE_CLUB_RESULT, TYPE_FLIGHT_RESULT, TYPE_FLIGHT_RESULT_V1, TYPE_MODE_ACK, TYPE_SHOT_TEXT,
    TYPE_SPIN_RESULT,
};

/// A scripted FlightScope on an in-memory stream.
///
/// Clones share the device: keep one to push frames and read what the
/// client sent while the client owns another.
#[derive(Clone, Default)]
pub struct FakeDevice(Rc<RefCell<State>>);

#[derive(Default)]
struct State {
    splitter: FrameSplitter,
    /// Wire bytes waiting to be read by the client.
    outbox: VecDeque<u8>,
    /// Frames the client sent, in order.
    sent: Vec<RawFrame>,
    /// Frames the device sent, in order.
    pushed: Vec<RawFrame>,
    /// Club results for the shot result requests, one per shot, oldest
    /// first.
    club_replies: VecDeque<Vec<u8>>,
    /// Answer the Xi's handshake and configuration.
    xi: bool,
    /// Armed by B0 `[01 01]`, disarmed by B0 `[01 00]`.
    armed: bool,
    /// DSP power mode (DSP param 0x01); 0 until written.
    dsp_power_mode: u8,
}

impl FakeDevice {
    /// A Mevo+ that answers status polls, the arm and the shot-completion
    /// requests.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// A device that also answers the Xi's handshake and configuration:
    /// status polls, device and product info, configuration queries,
    /// calibration parameters, parameter reads and writes (with the DSP
    /// power-mode texts), mode and radar calibration echoes. It refuses the
    /// DSP query (0x48) and the AVR config query (0x23), and has no camera
    /// Pi. Its status and device info frames carry a made-up serial number.
    #[must_use]
    pub fn xi() -> Self {
        let device = Self::default();
        device.0.borrow_mut().xi = true;
        device
    }

    /// A connection over a clone of this device.
    #[must_use]
    pub fn connection(&self) -> BinaryConnection<FakeDevice> {
        BinaryConnection::new(self.clone())
    }

    /// Queue a frame from `src` to the app.
    pub fn push(&self, src: BusAddr, type_id: u8, payload: impl Into<Vec<u8>>) {
        self.0.borrow_mut().push(src, type_id, payload.into());
    }

    /// Queue `shot`'s frames and answer its shot result request with its
    /// club result. Shots pushed before the client asks are answered in
    /// order: the first request gets the first shot's club result.
    pub fn push_shot(&self, shot: &Shot) {
        let mut st = self.0.borrow_mut();
        st.club_replies.push_back(shot.club_reply.clone());
        for f in &shot.frames {
            st.push(f.src, f.type_id, f.payload.clone());
        }
    }

    /// Every frame the client sent, in order.
    #[must_use]
    pub fn sent(&self) -> Vec<RawFrame> {
        self.0.borrow().sent.clone()
    }

    /// Every frame the device sent (queued and replies), in order.
    #[must_use]
    pub fn pushed(&self) -> Vec<RawFrame> {
        self.0.borrow().pushed.clone()
    }
}

impl State {
    fn push(&mut self, src: BusAddr, type_id: u8, payload: Vec<u8>) {
        let frame = RawFrame {
            dest: BusAddr::App,
            src,
            type_id,
            payload,
        };
        self.outbox.extend(frame.encode());
        self.pushed.push(frame);
    }

    fn respond(&mut self, f: &RawFrame) {
        let p = f.payload.as_slice();
        match (f.dest, f.type_id) {
            (BusAddr::Dsp, 0xAA) => {
                let status = if self.xi {
                    xi_dsp_status()
                } else {
                    mevo_dsp_status()
                };
                self.push(BusAddr::Dsp, 0xAA, status);
            }
            (BusAddr::Avr, 0xAA) => {
                self.push(
                    BusAddr::Avr,
                    0xAA,
                    avr_status([FAKE_SERIAL[0], FAKE_SERIAL[1]]),
                );
            }
            (BusAddr::Pi, 0xAA) if !self.xi => self.push(BusAddr::Pi, 0xAA, pi_status()),
            (BusAddr::Avr, 0xB0) => {
                self.push(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
                if p == [0x01, 0x01] {
                    self.armed = true;
                    self.push(BusAddr::Avr, 0xE3, b"\n\rARMED\0".to_vec());
                } else if self.armed {
                    self.armed = false;
                    self.push(BusAddr::Avr, 0xE3, b"  ARMED CANCELLED\0".to_vec());
                    self.push(BusAddr::Avr, TYPE_MODE_ACK, vec![0x02, 0x00, 0x00]);
                }
            }
            (BusAddr::Avr, 0x6D) => {
                if let Some(ed) = self.club_replies.pop_front() {
                    self.push(BusAddr::Avr, TYPE_CLUB_RESULT, ed);
                }
            }
            _ if self.xi => self.respond_xi(f),
            _ => {}
        }
    }

    /// The Xi's answers to its handshake and configuration.
    fn respond_xi(&mut self, f: &RawFrame) {
        let p = f.payload.as_slice();
        match (f.dest, f.type_id) {
            (dest @ (BusAddr::Dsp | BusAddr::Avr), 0x48 | 0x23) => {
                self.push(dest, 0x94, vec![0x02, dest as u8, f.type_id & 0x7F]);
            }
            (BusAddr::Dsp, 0xBF) => {
                self.push(BusAddr::Dsp, 0x95, vec![0x02, 0x40, 0x3F]);
                if p.len() >= 7 && p[3] == 0x01 && p[6] != self.dsp_power_mode {
                    let req = format!("  REQ Power Mode : {} ({}) \0", p[6], self.dsp_power_mode);
                    let new = format!("  NEW Power Mode : {} \0", p[6]);
                    self.push(BusAddr::Dsp, 0xE3, req.into_bytes());
                    self.push(BusAddr::Dsp, 0xE3, new.into_bytes());
                    self.dsp_power_mode = p[6];
                }
            }
            (BusAddr::Dsp, 0x67) => self.push(BusAddr::Dsp, 0xE7, xi_dsp_dev_info()),
            (BusAddr::Avr, 0x67) => {
                self.push(
                    BusAddr::Avr,
                    0xE7,
                    dev_info(["3.18    FSCOPE", "Jan  1 2020", "12:00:00"]),
                );
            }
            (BusAddr::Dsp, 0xFD) => {
                let mut r = vec![0u8; 34];
                r[0] = 0x21;
                r[1] = p.get(1).copied().unwrap_or(0);
                self.push(BusAddr::Dsp, 0xFD, r);
            }
            (dest @ (BusAddr::Dsp | BusAddr::Avr), 0x21) => {
                let mut r = vec![0u8; 69];
                r[0] = 0x44;
                self.push(dest, 0xA0, r);
            }
            (BusAddr::Avr, 0xBE) => {
                let id = p.get(3).copied().unwrap_or(0);
                self.push(BusAddr::Avr, 0xBF, vec![0x06, 0x00, 0x00, id, 0, 0, 0]);
            }
            (BusAddr::Avr, 0xBF) => self.push(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x3F]),
            (BusAddr::Avr, 0xD0) => self.push(BusAddr::Avr, 0xD1, vec![0u8; 171]),
            (BusAddr::Avr, 0xA5 | 0xA4) => self.push(BusAddr::Avr, f.type_id, p.to_vec()),
            _ => {}
        }
    }
}

impl io::Read for FakeDevice {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut st = self.0.borrow_mut();
        if st.outbox.is_empty() {
            return Err(io::ErrorKind::WouldBlock.into());
        }
        let n = buf.len().min(st.outbox.len());
        for (b, v) in buf.iter_mut().zip(st.outbox.drain(..n)) {
            *b = v;
        }
        Ok(n)
    }
}

impl io::Write for FakeDevice {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut st = self.0.borrow_mut();
        for wire in st.splitter.feed(buf) {
            let frame = RawFrame::parse(&wire)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
            st.respond(&frame);
            st.sent.push(frame);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// One shot as the device sends it, for [`FakeDevice::push_shot`].
///
/// Build one with [`Shot::new`] or a fixture constructor; the fields stay
/// public so a test can reorder, insert or drop frames.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct Shot {
    /// Frames the device sends, in order: the `BALL TRIGGER` text, the
    /// results, then the `PROCESSED` and `IDLE` texts and the mode ack.
    pub frames: Vec<RawFrame>,
    /// Club result (0xED) the device returns for the shot result request
    /// (0x6D).
    pub club_reply: Vec<u8>,
}

impl Shot {
    /// A shot whose `results` (type, payload) come from `results_src`, in
    /// order, between the AVR's `BALL TRIGGER: {trigger_ms} ms` text and its
    /// `PROCESSED` and `IDLE` texts and mode ack. `results_src` is the bus
    /// address stamped on the result frames. Every supported FlightScope
    /// model (Xi, Mevo+, Mevo Gen2) sends them from the AVR
    /// ([`BusAddr::Avr`]); another address lets a test script frames with a
    /// different source. The device answers the shot result request (0x6D)
    /// from the AVR with `club_reply`.
    #[must_use]
    pub fn new(
        trigger_ms: u32,
        results_src: BusAddr,
        results: impl IntoIterator<Item = (u8, Vec<u8>)>,
        club_reply: Vec<u8>,
    ) -> Self {
        let frame = |src, type_id, payload| RawFrame {
            dest: BusAddr::App,
            src,
            type_id,
            payload,
        };
        let avr = |type_id, payload| frame(BusAddr::Avr, type_id, payload);
        let mut frames = vec![avr(
            TYPE_SHOT_TEXT,
            shot_text(&format!("BALL TRIGGER: {trigger_ms} ms")),
        )];
        frames.extend(results.into_iter().map(|(t, p)| frame(results_src, t, p)));
        frames.extend([
            avr(TYPE_SHOT_TEXT, shot_text("PROCESSED")),
            avr(TYPE_SHOT_TEXT, shot_text("IDLE")),
            avr(TYPE_MODE_ACK, vec![0x02, 0x00, 0x00]),
        ]);
        Self { frames, club_reply }
    }

    /// The Xi full swing of [`fixtures::XI_SHOT_D4`]: early result, full
    /// result, club result and spin result.
    #[must_use]
    pub fn xi_full_swing() -> Self {
        Self::new(
            63,
            BusAddr::Avr,
            [
                (TYPE_FLIGHT_RESULT_V1, fixtures::XI_SHOT_E8.to_vec()),
                (TYPE_FLIGHT_RESULT, fixtures::XI_SHOT_D4.to_vec()),
                (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
                (TYPE_SPIN_RESULT, fixtures::XI_SHOT_EF.to_vec()),
            ],
            fixtures::XI_SHOT_ED.to_vec(),
        )
    }

    /// The Xi chip of [`fixtures::XI_CHIP_E8`]: an early result and a club
    /// result, no full result.
    #[must_use]
    pub fn xi_chip() -> Self {
        Self::new(
            123,
            BusAddr::Avr,
            [
                (TYPE_FLIGHT_RESULT_V1, fixtures::XI_CHIP_E8.to_vec()),
                (TYPE_CLUB_RESULT, fixtures::XI_CHIP_ED.to_vec()),
            ],
            fixtures::XI_CHIP_ED.to_vec(),
        )
    }

    /// A shot the Xi triggered on but did not read: an empty early result
    /// and an empty club result ([`fixtures::XI_NO_READ_E8`]).
    #[must_use]
    pub fn xi_no_read() -> Self {
        Self::new(
            187,
            BusAddr::Avr,
            [
                (TYPE_FLIGHT_RESULT_V1, fixtures::XI_NO_READ_E8.to_vec()),
                (TYPE_CLUB_RESULT, fixtures::XI_NO_READ_ED.to_vec()),
            ],
            fixtures::XI_NO_READ_ED.to_vec(),
        )
    }
}

/// Payload of a shot text (0xE5): length byte, the text, a NUL.
///
/// # Panics
///
/// If `text` is longer than 254 bytes.
#[must_use]
pub fn shot_text(text: &str) -> Vec<u8> {
    let len = u8::try_from(text.len() + 1).expect("shot text longer than 254 bytes");
    let mut p = vec![len];
    p.extend_from_slice(text.as_bytes());
    p.push(0);
    p
}

/// Payload of an AVR status (0xAA, 25 bytes, version 0x18, idle) with the
/// given hardware ID bytes and every other byte 0.
#[must_use]
pub fn avr_status(hardware_id: [u8; 2]) -> Vec<u8> {
    let mut p = vec![0u8; 25];
    p[0] = 0x18;
    p[2] = hardware_id[0];
    p[5] = hardware_id[1];
    p
}

/// Payload of an AVR or PI device info (0xE7, 75 bytes) with the given
/// text slots. Each slot holds up to 16 bytes; longer text is cut.
#[must_use]
pub fn dev_info(slots: [&str; 3]) -> Vec<u8> {
    let mut p = vec![0u8; 75];
    p[0] = 0x4A;
    for (i, text) in slots.iter().enumerate() {
        let start = 27 + i * 16;
        let bytes = &text.as_bytes()[..text.len().min(16)];
        p[start..start + bytes.len()].copy_from_slice(bytes);
    }
    p
}

/// Made-up serial number in the Xi's identity frames.
const FAKE_SERIAL: [u8; 4] = [0x01, 0x23, 0x45, 0x67];

/// Mevo+ DSP status (129 bytes, version byte 0x80, idle).
fn mevo_dsp_status() -> Vec<u8> {
    let mut p = vec![0u8; 129];
    p[0] = 0x80;
    p
}

/// Camera Pi status (raw bytes; ironsight keeps them undecoded).
fn pi_status() -> Vec<u8> {
    vec![0x01, 0x00]
}

/// Xi DSP status (29 bytes, version byte 0x1C).
fn xi_dsp_status() -> Vec<u8> {
    let mut p = vec![0u8; 29];
    p[0] = 0x1C;
    p[2..6].copy_from_slice(&FAKE_SERIAL);
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

#[cfg(test)]
mod tests;
