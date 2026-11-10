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
//!   with made-up identity frames;
//! - with [`FakeDevice::mevo_plus`] and [`FakeDevice::mevo_gen2`], also the
//!   Mevo handshake (DSP, AVR and camera Pi), the AVR configuration and the
//!   camera configuration, with made-up identity frames.
//!
//! [`FakeDevice::set_camera_fault`] makes the camera Pi leave a camera
//! configuration step unanswered or refuse it.
//!
//! [`FakeDevice::new`] answers nothing else of the handshake, so
//! [`BinaryClient::handshake`](crate::BinaryClient::handshake) completes on
//! [`FakeDevice::xi`], [`FakeDevice::mevo_plus`] and
//! [`FakeDevice::mevo_gen2`] only.
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
// "FlightScope" and "WiFi" in the docs are names, not code.
#![allow(clippy::doc_markdown)]

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
    /// Answer a Mevo's handshake and configuration, as this generation.
    mevo: Option<MevoGen>,
    /// The camera Pi's WiFi name, for a Mevo.
    ssid: String,
    /// The DSP status a Mevo answers polls with, when set.
    dsp_status: Option<Vec<u8>>,
    /// The camera config the Pi reports, as last written.
    cam_config: Vec<u8>,
    /// Armed by B0 `[01 01]`, disarmed by B0 `[01 00]`.
    armed: bool,
    /// DSP power mode (DSP param 0x01); 0 until written.
    dsp_power_mode: u8,
    /// How the camera Pi mishandles a camera configuration step.
    camera_fault: Option<CameraFault>,
}

/// How a [`FakeDevice`]'s camera Pi mishandles a frame sent to it, by the
/// frame's type: 0x82 the camera config, 0x83 its readback request, 0x81
/// the camera start, 0xBF the parameter write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CameraFault {
    /// Never answer a frame of this type.
    Ignore(u8),
    /// Refuse a frame of this type with a 0x94.
    Refuse(u8),
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

    /// A Mevo+ that also answers the Mevo handshake and configuration: DSP,
    /// AVR and camera Pi device info, product info, configuration queries,
    /// calibration, parameter reads and writes, mode and radar calibration
    /// echoes, the time sync, the camera config, and the Pi's WiFi name
    /// `FS M2-TEST01`. Its DSP status (version 0x80) reports a battery at
    /// 87 %, 31.0 °C and no external power. Every identity value is made up.
    #[must_use]
    pub fn mevo_plus() -> Self {
        Self::mevo(MevoGen::Plus, "FS M2-TEST01")
    }

    /// A Mevo Gen2, answering as [`mevo_plus`](Self::mevo_plus) does, with a
    /// Gen2 DSP status (version 0x46, no decoded battery, temperature or
    /// power) and the WiFi name `FS MG2-TEST01`.
    #[must_use]
    pub fn mevo_gen2() -> Self {
        Self::mevo(MevoGen::Gen2, "FS MG2-TEST01")
    }

    fn mevo(gen_: MevoGen, ssid: &str) -> Self {
        let device = Self::default();
        {
            let mut st = device.0.borrow_mut();
            st.mevo = Some(gen_);
            ssid.clone_into(&mut st.ssid);
            st.cam_config = crate::protocol::camera::CamConfig::standard_preset().encode();
        }
        device
    }

    /// This device with the camera Pi's WiFi name set to `ssid` (empty for
    /// a Pi that reports none).
    #[must_use]
    pub fn with_ssid(self, ssid: &str) -> Self {
        ssid.clone_into(&mut self.0.borrow_mut().ssid);
        self
    }

    /// Answer later DSP status polls of a Mevo with `payload` (see
    /// [`mevo_plus_dsp_status`]).
    pub fn set_dsp_status(&self, payload: impl Into<Vec<u8>>) {
        self.0.borrow_mut().dsp_status = Some(payload.into());
    }

    /// Make the camera Pi mishandle the frames of one type (`None` answers
    /// them again). Applies to the frames the client sends from now on,
    /// the camera warmup's included.
    pub fn set_camera_fault(&self, fault: Option<CameraFault>) {
        self.0.borrow_mut().camera_fault = fault;
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
        if f.dest == BusAddr::Pi {
            match self.camera_fault {
                Some(CameraFault::Ignore(t)) if t == f.type_id => return,
                Some(CameraFault::Refuse(t)) if t == f.type_id => {
                    self.push(BusAddr::Pi, 0x94, vec![0x02, 0x12, f.type_id & 0x7F]);
                    return;
                }
                _ => {}
            }
        }
        match (f.dest, f.type_id) {
            (BusAddr::Dsp, 0xAA) => {
                let status = if self.xi {
                    xi_dsp_status()
                } else if let Some(status) = &self.dsp_status {
                    status.clone()
                } else {
                    match self.mevo {
                        Some(MevoGen::Plus) => mevo_plus_dsp_status(87, 31.0, false),
                        Some(MevoGen::Gen2) => gen2_dsp_status(),
                        None => mevo_dsp_status(),
                    }
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
            _ if self.mevo.is_some() => self.respond_mevo(f),
            _ => {}
        }
    }

    /// A Mevo's answers to its handshake and configuration.
    fn respond_mevo(&mut self, f: &RawFrame) {
        let p = f.payload.as_slice();
        let gen2 = self.mevo == Some(MevoGen::Gen2);
        match (f.dest, f.type_id) {
            (BusAddr::Dsp, 0x48) => {
                let dsp_type = if gen2 { 0xC0 } else { 0x80 };
                self.push(BusAddr::Dsp, 0xC8, vec![0x02, dsp_type, 0x0E]);
            }
            (BusAddr::Dsp, 0x67) => {
                let mut info = vec![0u8; 76];
                info[0] = 0x4B;
                for (i, text) in ["1.00", "TEST-SERIAL-01", "FAKE"].iter().enumerate() {
                    let start = 28 + i * 16;
                    info[start..start + text.len()].copy_from_slice(text.as_bytes());
                }
                self.push(BusAddr::Dsp, 0xE7, info);
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
            (BusAddr::Avr, 0x67) => {
                let version = if gen2 {
                    "4.56    FSCOPE"
                } else {
                    "1.23    FSCOPE"
                };
                let info = dev_info([version, "Feb  3 2024", "12:00:00"]);
                self.push(BusAddr::Avr, 0xE7, info);
            }
            (dest @ (BusAddr::Avr | BusAddr::Pi), 0xBE) => {
                let id = p.get(3).copied().unwrap_or(0);
                self.push(dest, 0xBF, vec![0x06, 0x00, 0x00, id, 0, 0, 0]);
            }
            (dest @ (BusAddr::Avr | BusAddr::Pi), 0xBF) => {
                self.push(dest, 0x95, vec![0x02, dest as u8, 0x3F]);
            }
            (BusAddr::Avr, 0xD2) => self.push(BusAddr::Avr, 0xD3, vec![0u8; 16]),
            (BusAddr::Avr, 0xD0) => self.push(BusAddr::Avr, 0xD1, vec![0u8; 171]),
            (BusAddr::Avr, 0x23) => {
                let mut r = vec![0u8; 17];
                r[0] = 0x10;
                r[1] = if gen2 { 2 } else { 1 };
                self.push(BusAddr::Avr, 0xA2, r);
            }
            (BusAddr::Avr, 0x9B) => self.push(BusAddr::Avr, 0x9B, p.to_vec()),
            (BusAddr::Avr, 0xA5 | 0xA4) => self.push(BusAddr::Avr, f.type_id, p.to_vec()),
            (BusAddr::Pi, 0x67) => {
                let info = dev_info(["2.00", "Jan  1 2021", "12:00:00"]);
                self.push(BusAddr::Pi, 0xE7, info);
            }
            (BusAddr::Pi, 0x83) => {
                let config = self.cam_config.clone();
                self.push(BusAddr::Pi, 0x82, config);
            }
            (BusAddr::Pi, 0x82) => {
                self.cam_config = p.to_vec();
                self.push(BusAddr::Pi, 0x95, vec![0x02, 0x12, 0x02]);
            }
            (BusAddr::Pi, 0x81) => self.push(BusAddr::Pi, 0x95, vec![0x02, 0x12, 0x01]),
            (BusAddr::Pi, 0xDE) => {
                let mut r = vec![0u8; 54];
                r[0] = 0x35;
                if p.get(1) == Some(&0x08) {
                    let ssid = &self.ssid.as_bytes()[..self.ssid.len().min(15)];
                    r[21..21 + ssid.len()].copy_from_slice(ssid);
                    r[37..37 + 9].copy_from_slice(b"TEST-PASS");
                }
                self.push(BusAddr::Pi, 0xDE, r);
            }
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

/// Payload of a Mevo+ DSP status (0xAA, 129 bytes, version 0x80, idle)
/// reporting `battery_pct`, `temp_c` and whether external power is
/// connected; every other byte 0.
#[must_use]
pub fn mevo_plus_dsp_status(battery_pct: u8, temp_c: f64, external_power: bool) -> Vec<u8> {
    let mut p = mevo_dsp_status();
    #[allow(clippy::cast_possible_truncation)] // a test temperature fits
    let temp = (temp_c * 100.0).round() as i16;
    p[40..42].copy_from_slice(&temp.to_be_bytes());
    p[61] = battery_pct;
    p[63] = u8::from(external_power);
    p
}

/// A Mevo generation the fake answers as.
#[derive(Clone, Copy, PartialEq, Eq)]
enum MevoGen {
    Plus,
    Gen2,
}

/// Made-up serial number in the Xi's identity frames.
const FAKE_SERIAL: [u8; 4] = [0x01, 0x23, 0x45, 0x67];

/// Mevo+ DSP status (129 bytes, version byte 0x80, idle).
fn mevo_dsp_status() -> Vec<u8> {
    let mut p = vec![0u8; 129];
    p[0] = 0x80;
    p
}

/// Mevo Gen2 DSP status (71 bytes, version byte 0x46, idle).
fn gen2_dsp_status() -> Vec<u8> {
    let mut p = vec![0u8; 71];
    p[0] = 0x46;
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
