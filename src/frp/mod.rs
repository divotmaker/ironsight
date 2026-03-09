//! FRP device server — bridges ironsight's binary protocol to the
//! [Flight Relay Protocol](https://github.com/flightrelay/spec).
//!
//! Maps [`BinaryEvent`]s from a connected Mevo+/Gen2 to FRP envelopes and
//! streams them to any connected FRP controller over WebSocket (port 5880).
//!
//! Requires the `frp` feature.

mod convert;

use std::collections::HashMap;

use flightrelay::{
    DetectionMode, FrpConnection, FrpEnvelope, FrpEvent, FrpListener, FrpMessage,
    FrpProtocolMessage, ShotKey, SPEC_VERSION,
};

use crate::client::{BinaryEvent, HandshakeOutcome};
use crate::protocol::config;
use crate::seq::ShotDatum;

pub use convert::{ball_flight, club_data};

/// Device name derived from the handshake SSID (e.g. `"FS M2-ABC123"`).
fn device_name(handshake: &HandshakeOutcome) -> String {
    handshake.pi.ssid.clone()
}

/// Map an FRP [`DetectionMode`] to the corresponding ironsight mode constant.
#[must_use]
pub fn detection_mode_to_avr(mode: DetectionMode) -> u8 {
    match mode {
        DetectionMode::Full => config::MODE_INDOOR,
        DetectionMode::Putting => config::MODE_PUTTING,
        DetectionMode::Chipping => config::MODE_CHIPPING,
    }
}

/// An FRP device server backed by an ironsight connection.
///
/// Manages the FRP listener and converts [`BinaryEvent`]s into FRP envelopes.
/// The caller drives both the `BinaryClient` poll loop and this server in the
/// same thread.
pub struct FrpServer {
    listener: FrpListener,
    conn: Option<FrpConnection>,
    device: String,
    shot_number: u32,
    current_key: Option<ShotKey>,
}

impl FrpServer {
    /// Bind the FRP listener on the given address (e.g. `"0.0.0.0:5880"`).
    ///
    /// # Errors
    ///
    /// Returns an error if the TCP listener cannot bind.
    pub fn bind(addr: &str) -> Result<Self, flightrelay::FrpError> {
        let listener = FrpListener::bind(addr, &[SPEC_VERSION])?;
        Ok(Self {
            listener,
            conn: None,
            device: String::new(),
            shot_number: 0,
            current_key: None,
        })
    }

    /// Set the device name (typically from handshake SSID).
    pub fn set_device_name(&mut self, handshake: &HandshakeOutcome) {
        self.device = device_name(handshake);
    }

    /// Accept a controller connection (blocking).
    ///
    /// Replaces any existing connection.
    ///
    /// # Errors
    ///
    /// Returns an error if the WebSocket handshake fails.
    pub fn accept(&mut self) -> Result<(), flightrelay::FrpError> {
        let conn = self.listener.accept()?;
        conn.set_nonblocking(true)?;
        self.conn = Some(conn);
        Ok(())
    }

    /// Try to accept a controller connection (non-blocking).
    ///
    /// Replaces any existing connection if a new one arrives.
    pub fn try_accept(&mut self) -> Result<bool, flightrelay::FrpError> {
        // FrpListener doesn't expose set_nonblocking on the underlying
        // TcpListener, so we set it before accept. This is a bit awkward
        // but keeps the API simple.
        // For now, callers should use accept() in a separate thread or
        // check_controller() for polling incoming commands.
        Ok(false)
    }

    /// Poll for incoming controller commands (non-blocking).
    ///
    /// Returns a [`DetectionMode`] if the controller sent `set_detection_mode`.
    pub fn check_controller(&mut self) -> Option<DetectionMode> {
        let conn = self.conn.as_mut()?;
        match conn.try_recv() {
            Ok(Some(FrpMessage::Protocol(FrpProtocolMessage::SetDetectionMode {
                mode, ..
            }))) => mode,
            Err(flightrelay::FrpError::Closed) => {
                self.conn = None;
                None
            }
            _ => None,
        }
    }

    /// Send a device info envelope with telemetry from the handshake.
    ///
    /// # Errors
    ///
    /// Returns an error if the send fails.
    pub fn send_device_info(
        &mut self,
        handshake: &HandshakeOutcome,
    ) -> Result<(), flightrelay::FrpError> {
        let conn = match self.conn.as_mut() {
            Some(c) => c,
            None => return Ok(()),
        };

        let mut telemetry = HashMap::new();
        telemetry.insert("ready".to_owned(), "true".to_owned());
        telemetry.insert(
            "tilt".to_owned(),
            format!("{:.1}", handshake.avr.status.tilt),
        );
        telemetry.insert(
            "roll".to_owned(),
            format!("{:.1}", handshake.avr.status.roll),
        );

        let model = if handshake.dsp.hw_info.dsp_type == 0xC0 {
            "Mevo Gen2"
        } else {
            "Mevo+"
        };

        let env = FrpEnvelope {
            device: self.device.clone(),
            event: FrpEvent::DeviceTelemetry {
                manufacturer: Some("FlightScope".to_owned()),
                model: Some(model.to_owned()),
                firmware: Some(handshake.dsp.dev_info.text.clone()),
                telemetry: Some(telemetry),
            },
        };

        conn.send_envelope(&env).or_else(|e| {
            if matches!(e, flightrelay::FrpError::Closed) {
                self.conn = None;
                Ok(())
            } else {
                Err(e)
            }
        })
    }

    /// Process a [`BinaryEvent`] and send any resulting FRP envelopes.
    ///
    /// # Errors
    ///
    /// Returns an error if a send fails (other than connection close).
    pub fn handle_event(&mut self, event: &BinaryEvent) -> Result<(), flightrelay::FrpError> {
        let events = match event {
            BinaryEvent::Trigger => {
                self.shot_number += 1;
                let key = ShotKey {
                    shot_id: uuid_v4(),
                    shot_number: self.shot_number,
                };
                self.current_key = Some(key.clone());
                vec![FrpEvent::ShotTrigger { key }]
            }
            BinaryEvent::ShotDatum(datum) => self.datum_to_events(datum),
            BinaryEvent::ShotComplete(data) => {
                let mut events = Vec::new();
                // Send any data not already sent via ShotDatum
                if let Some(ref flight) = data.flight
                    && let Some(ref key) = self.current_key
                {
                    events.push(FrpEvent::BallFlight {
                        key: key.clone(),
                        ball: convert::ball_flight(flight),
                    });
                }
                if let Some(ref club) = data.club
                    && let Some(ref key) = self.current_key
                {
                    events.push(FrpEvent::ClubPath {
                        key: key.clone(),
                        club: convert::club_data(club),
                    });
                }
                if let Some(ref key) = self.current_key.take() {
                    events.push(FrpEvent::ShotFinished { key: key.clone() });
                }
                events
            }
            _ => vec![],
        };

        self.send_events(&events)
    }

    fn datum_to_events(&self, datum: &ShotDatum) -> Vec<FrpEvent> {
        let key = match self.current_key {
            Some(ref k) => k.clone(),
            None => return vec![],
        };

        match datum {
            ShotDatum::Flight(flight) => vec![FrpEvent::BallFlight {
                key,
                ball: convert::ball_flight(flight),
            }],
            ShotDatum::Club(club) => vec![FrpEvent::ClubPath {
                key,
                club: convert::club_data(club),
            }],
            ShotDatum::FlightV1(_) | ShotDatum::Spin(_) => vec![],
        }
    }

    fn send_events(&mut self, events: &[FrpEvent]) -> Result<(), flightrelay::FrpError> {
        let conn = match self.conn.as_mut() {
            Some(c) => c,
            None => return Ok(()),
        };

        for event in events {
            let env = FrpEnvelope {
                device: self.device.clone(),
                event: event.clone(),
            };
            match conn.send_envelope(&env) {
                Ok(()) => {}
                Err(flightrelay::FrpError::Closed) => {
                    self.conn = None;
                    return Ok(());
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
}

/// Generate a UUID v4 string without pulling in the `uuid` crate.
fn uuid_v4() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};

    let t = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let seed = t.as_nanos();

    // xorshift128+ with time-based seed — good enough for shot correlation IDs
    let mut s0 = seed as u64;
    let mut s1 = seed.wrapping_mul(6364136223846793005) as u64;
    if s0 == 0 {
        s0 = 0x1234_5678_9abc_def0;
    }
    if s1 == 0 {
        s1 = 0xfedcba9876543210;
    }

    let mut bytes = [0u8; 16];
    for chunk in bytes.chunks_exact_mut(8) {
        let mut x = s0;
        let y = s1;
        s0 = y;
        x ^= x << 23;
        x ^= x >> 17;
        x ^= y;
        x ^= y >> 26;
        s1 = x;
        let val = s0.wrapping_add(s1);
        chunk.copy_from_slice(&val.to_le_bytes());
    }

    // Set version 4 and variant bits
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;

    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        bytes[0], bytes[1], bytes[2], bytes[3],
        bytes[4], bytes[5],
        bytes[6], bytes[7],
        bytes[8], bytes[9],
        bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15],
    )
}
