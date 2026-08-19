//! FRP device adapter — bridges ironsight's binary protocol to the
//! [Flight Relay Protocol](https://github.com/flightrelay/spec).
//!
//! Maps [`BinaryEvent`]s from a connected Mevo+/Gen2 to FRP envelopes and
//! streams them to an FRP controller. The adapter always plays the FRP
//! [`Role::Device`]; the transport direction is the caller's choice:
//!
//! - [`FrpDevice::serve`] accepts controllers on a local port (default 5880)
//! - [`FrpDevice::bridge`] dials a central controller such as flighthook
//!
//! Either way the device emits shot envelopes and accepts `set_detection_mode`.
//! Connections are established on a background thread so the caller's Mevo poll
//! loop never blocks; a dropped connection is re-established automatically.
//!
//! Requires the `frp` feature.

mod convert;

use std::collections::HashMap;
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::thread;

use flightrelay::{
    DetectionMode, EndpointConfig, FrpConnection, FrpEndpoint, FrpEnvelope, FrpEvent, FrpMessage,
    FrpProtocolMessage, Role, SPEC_VERSION, ShotKey, Transport,
};

use crate::client::{BinaryEvent, HandshakeOutcome};
use crate::protocol::config;
use crate::seq::ShotDatum;

pub use convert::{ball_flight, club_data};

/// Backoff between failed connection attempts.
const RETRY_DELAY: std::time::Duration = std::time::Duration::from_secs(2);

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

/// An FRP device backed by an ironsight connection.
///
/// Converts [`BinaryEvent`]s into FRP envelopes and streams them to the
/// connected controller. The caller drives both the `BinaryClient` poll loop
/// and this adapter in the same thread.
pub struct FrpDevice {
    conn: Option<FrpConnection>,
    /// Signals the acceptor thread to establish a connection.
    request: Sender<()>,
    /// Receives established connections from the acceptor thread.
    incoming: Receiver<FrpConnection>,
    /// True while the acceptor thread is working on a connection.
    pending: bool,
    /// Last telemetry envelope, re-sent to each newly connected controller.
    telemetry: Option<FrpEnvelope>,
    device: String,
    shot_number: u32,
    current_key: Option<ShotKey>,
}

impl FrpDevice {
    /// Accept controllers on `addr` (e.g. `"0.0.0.0:5880"`).
    ///
    /// # Errors
    ///
    /// Returns an error if the TCP listener cannot bind.
    pub fn serve(addr: &str) -> Result<Self, flightrelay::FrpError> {
        Self::spawn(EndpointConfig::new(Role::Device, Transport::listen(addr)))
    }

    /// Dial a central controller at `url` (e.g. `"ws://flighthook:5880/frp"`),
    /// identifying as `name`.
    ///
    /// # Errors
    ///
    /// Returns an error if the endpoint cannot be opened.
    pub fn bridge(url: &str, name: &str) -> Result<Self, flightrelay::FrpError> {
        Self::spawn(
            EndpointConfig::new(Role::Device, Transport::connect(url))
                .with_name(name)
                .with_versions(&[SPEC_VERSION]),
        )
    }

    fn spawn(config: EndpointConfig) -> Result<Self, flightrelay::FrpError> {
        let mut endpoint = FrpEndpoint::open(config)?;
        let (request, request_rx) = mpsc::channel::<()>();
        let (conn_tx, incoming) = mpsc::channel::<FrpConnection>();

        thread::spawn(move || {
            while request_rx.recv().is_ok() {
                // Retry until connected — one request yields one connection.
                loop {
                    match endpoint.establish() {
                        Ok(conn) if conn.set_nonblocking(true).is_ok() => {
                            if conn_tx.send(conn).is_err() {
                                return;
                            }
                            break;
                        }
                        // Back off so a refused dial or rejected handshake
                        // does not spin the thread.
                        _ => thread::sleep(RETRY_DELAY),
                    }
                }
            }
        });

        let mut device = Self {
            conn: None,
            request,
            incoming,
            pending: false,
            telemetry: None,
            device: String::new(),
            shot_number: 0,
            current_key: None,
        };
        device.request_connection();
        Ok(device)
    }

    /// Ask the acceptor thread for a connection, unless one is already pending.
    fn request_connection(&mut self) {
        if !self.pending && self.request.send(()).is_ok() {
            self.pending = true;
        }
    }

    /// Adopt a newly established connection, if one is ready.
    ///
    /// Call once per poll-loop iteration. Re-sends the cached telemetry
    /// envelope to each newly connected controller, as the spec requires.
    ///
    /// Returns `true` when a connection was adopted.
    ///
    /// # Errors
    ///
    /// Returns an error if the telemetry re-send fails.
    pub fn poll_connection(&mut self) -> Result<bool, flightrelay::FrpError> {
        if self.conn.is_some() {
            return Ok(false);
        }
        match self.incoming.try_recv() {
            Ok(conn) => {
                self.pending = false;
                self.conn = Some(conn);
                if let Some(env) = self.telemetry.clone() {
                    self.send_envelope(&env)?;
                }
                Ok(true)
            }
            Err(TryRecvError::Empty) => Ok(false),
            Err(TryRecvError::Disconnected) => Ok(false),
        }
    }

    /// Whether a controller is currently connected.
    #[must_use]
    pub fn is_connected(&self) -> bool {
        self.conn.is_some()
    }

    /// Set the device name (typically from handshake SSID).
    pub fn set_device_name(&mut self, handshake: &HandshakeOutcome) {
        self.device = device_name(handshake);
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
            Err(_) => {
                self.drop_connection();
                None
            }
            _ => None,
        }
    }

    /// Build, cache, and send a device telemetry envelope from the handshake.
    ///
    /// The envelope is cached and re-sent to any controller that connects later.
    ///
    /// # Errors
    ///
    /// Returns an error if the send fails.
    pub fn send_device_info(
        &mut self,
        handshake: &HandshakeOutcome,
    ) -> Result<(), flightrelay::FrpError> {
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

        self.telemetry = Some(env.clone());
        self.send_envelope(&env)
    }

    /// Drop the current connection and ask for a replacement.
    fn drop_connection(&mut self) {
        self.conn = None;
        self.request_connection();
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

    /// Send one envelope, dropping the connection if the peer has gone away.
    fn send_envelope(&mut self, env: &FrpEnvelope) -> Result<(), flightrelay::FrpError> {
        let Some(conn) = self.conn.as_mut() else {
            return Ok(());
        };
        match conn.send_envelope(env) {
            Ok(()) => Ok(()),
            Err(flightrelay::FrpError::Closed) => {
                self.drop_connection();
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    fn send_events(&mut self, events: &[FrpEvent]) -> Result<(), flightrelay::FrpError> {
        for event in events {
            if self.conn.is_none() {
                return Ok(());
            }
            let env = FrpEnvelope {
                device: self.device.clone(),
                event: event.clone(),
            };
            self.send_envelope(&env)?;
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
        bytes[0],
        bytes[1],
        bytes[2],
        bytes[3],
        bytes[4],
        bytes[5],
        bytes[6],
        bytes[7],
        bytes[8],
        bytes[9],
        bytes[10],
        bytes[11],
        bytes[12],
        bytes[13],
        bytes[14],
        bytes[15],
    )
}
