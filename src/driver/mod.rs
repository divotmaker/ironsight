//! The FlightScope [`DeviceDriver`]: Mevo, Mevo+, Mevo Gen2 and Xi.
//!
//! Requires the `driver` feature. [`Mevo`] plugs a FlightScope device into
//! any host of flightrelay's driver contract: flightrelay's `FrpServer` and
//! `run` (the `ironsight-frp` binary is `flightrelay::run::<Mevo>()`), or an
//! application with its own host loop.
//!
//! A session connects over TCP, runs the handshake, then configures and arms
//! the device for the detected model in the detection mode the host has
//! latched. It sends:
//!
//! - telemetry: `ready`, `tilt`, `roll`, and on a Mevo+ `battery_pct`,
//!   `temp_c` and `external_power`; only values that changed;
//! - for each shot: `shot_trigger` (with the host's clock), `launch` and
//!   `flight_result` as soon as the full result arrives, `club_path` once
//!   the shot has a ball result, and `shot_finished` when the device is done.
//!   A launch-only full result (see [`convert::launch_only`]) sends a launch
//!   without spin and no flight result. Without a full result, the early
//!   result makes the shot when [`Config::use_estimated`] is on;
//! - an alert for a shot the device did not read, and for a message it could
//!   not decode (the session carries on; the alert names the error, never
//!   the message's bytes);
//! - `raw_frame`s while the host wants them, once the handshake is done: one
//!   framed message each, `data` the payload, `label` `"<type> <src>><dst>"`
//!   (e.g. `"D4 AVR>APP"`). The handshake's messages, those that can
//!   identify the unit, its network or the host ([`carries_identity`]), and
//!   the device's free text (0xE3) and DSP debug output (0xF0) never become
//!   raw frames; they go to the host's audit log as hex trace lines.
//!
//! The session ends with an error when the device closes the connection, an
//! operation times out, or no message arrives for 10 seconds. The host
//! reconnects with the driver's advised delay.
//!
//! [`MevoSession`] is generic over its stream; [`Mevo::connect`] builds it on
//! a `TcpStream`, and with the `testing` feature `Mevo::connect_with` runs a
//! whole session on `ironsight::testing::FakeDevice`.
// "FlightScope" and "WiFi" in the docs are names, not code.
#![allow(clippy::doc_markdown)]

pub mod convert;

use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use flightrelay::config::{ConfigError, DriverConfig};
use flightrelay::driver::{
    ConnectCtx, DeviceCommand, DeviceDriver, DeviceEvent, DeviceIdentity, DeviceSession,
    DriverInfo, Events, HostFlags, Level, Live, NoAction, RawProfile, ShotIds, Stopped,
};
use flightrelay::types::{ClubData, Launch};
use flightrelay::{DetectionMode, FrpEvent, RawDir, RawFrame, Severity, ShotKey, Source};

use crate::addr::BusAddr;
use crate::client::{BinaryClient, BinaryEvent, HandshakeOutcome};
use crate::config::{CameraMode, Config, SpinSource};
use crate::conn::{BinaryConnection, ConnError};
use crate::protocol::camera::{self, CamConfig};
use crate::protocol::handshake::{DeviceGen, DeviceModel};
use crate::protocol::shot::{FlightResult, FlightResultV1};
use crate::protocol::status::{AvrStatus, DspStatus};
use crate::protocol::{
    Message, TYPE_DSP_DEBUG, TYPE_FLIGHT_RESULT, TYPE_FLIGHT_RESULT_V1, TYPE_TEXT,
    carries_identity, is_bulk_shot_data,
};
use crate::seq::ShotDatum;

/// How long the TCP connect may take.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);
/// No message for this long ends the session.
const STALE_TIMEOUT: Duration = Duration::from_secs(10);
/// Keepalive cadence once armed.
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(3);
/// Sleep between polls of an idle connection.
const IDLE_SLEEP: Duration = Duration::from_millis(20);
/// Frame types besides [`carries_identity`]'s that go only to the audit log:
/// the device's free text (0xE3) and DSP debug output (0xF0).
const AUDIT_ONLY: [u8; 2] = [TYPE_TEXT, TYPE_DSP_DEBUG];

/// Why a FlightScope connect or session ended.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The host stopped.
    #[error("stopped")]
    Stopped,
    /// The TCP connect failed.
    #[error("could not connect to {addr}: {source}")]
    Connect {
        /// The device address.
        addr: SocketAddr,
        /// The connect error.
        #[source]
        source: std::io::Error,
    },
    /// The connection failed: closed by the device, an operation timed out,
    /// or an I/O error. The message leaves out payload bytes
    /// ([`ConnError::describe`]).
    #[error("{}", .0.describe())]
    Connection(#[from] ConnError),
    /// Nothing arrived from the device for 10 seconds.
    #[error("no message from the device for {} s", STALE_TIMEOUT.as_secs())]
    Stale,
    /// The config can't be used: [`connect`](DeviceDriver::connect) found
    /// an address that doesn't parse (a host applied a config without
    /// validating it).
    #[error("{0}")]
    Config(#[from] ConfigError),
}

impl From<Stopped> for Error {
    fn from(_: Stopped) -> Self {
        Self::Stopped
    }
}

/// The FlightScope driver. See the [module docs](self).
#[derive(Debug)]
pub struct Mevo {
    config: Config,
    ids: ShotIds,
    spin_source: Live<SpinSource>,
}

impl DeviceDriver for Mevo {
    type Config = Config;
    type Error = Error;
    type Action = NoAction;
    type Session = MevoSession<TcpStream>;

    /// The TCP connect gives up at once on a stop and the handshake checks
    /// for one between polls; closing sends nothing.
    const STOP_WAIT: Duration = Duration::from_secs(3);

    fn info(_: &Config) -> DriverInfo {
        DriverInfo::new("ironsight", "FlightScope Mevo+ / Xi", Source::Measured)
            .with_manufacturer("FlightScope")
            .with_raw(raw_profile())
    }

    fn new(config: Config) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self {
            spin_source: Live::new(config.spin_source),
            config,
            ids: ShotIds::new(),
        })
    }

    fn config(&self) -> &Config {
        &self.config
    }

    fn connect(&mut self, ctx: &mut ConnectCtx<'_>) -> Result<MevoSession<TcpStream>, Error> {
        let addr = self.config.socket_addr()?;
        ctx.out.phase("connecting");
        ctx.out.log(Level::Info, format!("connecting to {addr}"));
        let stream = ctx
            .connect_tcp(addr, CONNECT_TIMEOUT)?
            .map_err(|source| Error::Connect { addr, source })?;
        let _ = stream.set_nodelay(true);
        stream.set_nonblocking(true).map_err(ConnError::Io)?;
        self.open(ctx, stream, addr)
    }

    fn apply_config(&mut self, new: &Config) {
        self.spin_source.set(new.spin_source);
        self.config = new.clone();
    }
}

impl Mevo {
    /// Run a whole session on `stream` instead of a TCP connection: the
    /// handshake, then [`MevoSession`] as [`connect`](DeviceDriver::connect)
    /// returns it. `stream` must not block on reads (return `WouldBlock` when
    /// it has nothing). The Xi's identity uses the configured address.
    ///
    /// # Errors
    ///
    /// As [`connect`](DeviceDriver::connect), without the TCP connect.
    #[cfg(any(test, feature = "testing"))]
    pub fn connect_with<S: Read + Write>(
        &mut self,
        ctx: &mut ConnectCtx<'_>,
        stream: S,
    ) -> Result<MevoSession<S>, Error> {
        let addr = self.config.socket_addr()?;
        self.open(ctx, stream, addr)
    }

    /// Handshake on `stream` and build the session.
    fn open<S: Read + Write>(
        &mut self,
        ctx: &mut ConnectCtx<'_>,
        stream: S,
        addr: SocketAddr,
    ) -> Result<MevoSession<S>, Error> {
        let mut conn = BinaryConnection::new(stream);
        let wire = Wire::attach(&mut conn);
        let mut client = BinaryClient::new(conn);
        client.set_keepalive_interval(KEEPALIVE_INTERVAL);
        client.handshake();
        ctx.out.phase("handshaking");

        let mut last_event = Instant::now();
        let mut results = LastResults::default();
        let handshake = loop {
            ctx.tick()?;
            let polled = client.poll();
            // Nothing of the handshake leaves the driver as a raw frame.
            if wire.emit(&mut ctx.out, &mut results, false) {
                last_event = Instant::now();
            }
            let idle = match polled {
                Ok(Some(BinaryEvent::Handshake(h))) => break h,
                Ok(Some(BinaryEvent::StepRefused(env))) => {
                    last_event = Instant::now();
                    ctx.out.log(
                        Level::Warn,
                        format!(
                            "device refused a step, continuing: {} 0x{:02X} {}",
                            env.src,
                            env.type_id,
                            hex(&env.raw)
                        ),
                    );
                    false
                }
                Ok(Some(_)) => {
                    last_event = Instant::now();
                    false
                }
                Ok(None) => true,
                Err(e @ (ConnError::Wire(_) | ConnError::Protocol(_))) => {
                    could_not_process(&mut ctx.out, &e);
                    true
                }
                Err(e) => return Err(e.into()),
            };
            if last_event.elapsed() >= STALE_TIMEOUT {
                return Err(Error::Stale);
            }
            if idle {
                std::thread::sleep(IDLE_SLEEP);
            }
        };

        let identity = identity(&handshake, addr);
        let model = handshake.model;
        let mode = ctx.latched().mode.unwrap_or(DetectionMode::Full);
        let mut session = MevoSession {
            client,
            identity,
            model,
            config: self.config.clone(),
            spin_source: self.spin_source.clone(),
            ids: self.ids.clone(),
            wire,
            results,
            mode,
            pending_mode: None,
            shot: None,
            pending_fusion_cam: None,
            session_start: Instant::now(),
            warmup: camera::WARMUP,
            last_event: Instant::now(),
            telemetry: HashMap::new(),
            backlog: Vec::new(),
        };
        session.handshake_done(&handshake, &mut ctx.out);
        Ok(session)
    }
}

/// The raw-frame profile: frames between a shot's trigger and its re-arm,
/// both directions, with the bulk per-shot data charged to a byte budget.
fn raw_profile() -> RawProfile {
    RawProfile::new("flightscope-tcp", 8, 256, 128 * 1024, is_bulk_frame, false)
}

/// Whether a raw frame is bulk per-shot data ([`is_bulk_shot_data`]), from
/// the type at the start of its label.
fn is_bulk_frame(frame: &RawFrame) -> bool {
    frame
        .label
        .as_deref()
        .and_then(|l| l.split(' ').next())
        .and_then(|t| u8::from_str_radix(t, 16).ok())
        .is_some_and(is_bulk_shot_data)
}

/// Who the device is: the camera Pi's WiFi SSID, or the model and address
/// on a model without one (the Xi). The model is `"Mevo+"`, `"Mevo Gen2"`,
/// `"Mevo"` or `"Xi"`; the firmware is the AVR device info's version and
/// build date, `"1.23 (Feb 3 2024)"`. Nothing else of the device info is
/// used: the DSP's carries the serial number. No label is set, so the card
/// text is manufacturer and model for every model (`"FlightScope Xi"`).
fn identity(h: &HandshakeOutcome, addr: SocketAddr) -> DeviceIdentity {
    let device = match &h.pi {
        Some(pi) if !pi.ssid.is_empty() => pi.ssid.clone(),
        _ => format!("{} ({addr})", h.model),
    };
    let model = match (
        h.model,
        h.dsp
            .hw_info
            .as_ref()
            .map(crate::protocol::handshake::DspQueryResp::device_gen),
    ) {
        (DeviceModel::Xi, _) => "Xi",
        (DeviceModel::Mevo, Some(g @ (DeviceGen::MevoPlus | DeviceGen::Gen2))) => g.label(),
        (DeviceModel::Mevo, _) => DeviceModel::Mevo.label(),
    };
    let info = &h.avr.dev_info;
    let firmware = match (info.version(), info.build_date()) {
        (Some(v), Some(d)) => Some(format!("{v} ({d})")),
        (Some(v), None) => Some(v.to_owned()),
        (None, Some(d)) => Some(format!("built {d}")),
        (None, None) => None,
    };
    let identity = DeviceIdentity::new(device)
        .with_manufacturer("FlightScope")
        .with_model(model);
    match firmware {
        Some(f) => identity.with_firmware(f),
        None => identity,
    }
}

/// One frame as the connection's hooks saw it.
struct Frame {
    dir: RawDir,
    type_id: u8,
    src: BusAddr,
    dst: BusAddr,
    payload: Vec<u8>,
    at_ms: Option<u64>,
}

/// The frames the connection's hooks collected since the last drain.
#[derive(Clone, Default)]
struct Wire(Arc<Mutex<Vec<Frame>>>);

impl Wire {
    /// Install the send and receive hooks on `conn`.
    fn attach<S: Read + Write>(conn: &mut BinaryConnection<S>) -> Self {
        let wire = Self::default();
        let tx = wire.clone();
        conn.set_on_send(move |cmd, dest| {
            let frame = cmd.encode(dest);
            tx.push(Frame {
                dir: RawDir::Tx,
                type_id: frame.type_id,
                src: frame.src,
                dst: dest,
                payload: frame.payload,
                at_ms: now_ms(),
            });
        });
        let rx = wire.clone();
        conn.set_on_recv(move |env| {
            rx.push(Frame {
                dir: RawDir::Rx,
                type_id: env.type_id,
                src: env.src,
                dst: BusAddr::App,
                payload: env.raw.clone(),
                at_ms: now_ms(),
            });
        });
        wire
    }

    fn push(&self, frame: Frame) {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push(frame);
    }

    /// Emit the collected frames: as `raw_frame`s while the host wants them
    /// and `raw` allows them (after the handshake), or as audit lines for a
    /// frame that is audit-only ([`audit_only`]) and every frame while `raw`
    /// is false. Keeps the latest full and early result payloads in
    /// `results`. Returns whether a frame from the device was among them.
    fn emit(&self, out: &mut Events<'_>, results: &mut LastResults, raw: bool) -> bool {
        let frames = std::mem::take(&mut *self.0.lock().unwrap_or_else(PoisonError::into_inner));
        let mut received = false;
        for f in frames {
            if f.dir == RawDir::Rx {
                received = true;
                match f.type_id {
                    TYPE_FLIGHT_RESULT => results.d4.clone_from(&f.payload),
                    TYPE_FLIGHT_RESULT_V1 => results.e8.clone_from(&f.payload),
                    _ => {}
                }
            }
            let label = format!("{:02X} {}>{}", f.type_id, f.src, f.dst);
            if !raw || audit_only(f.type_id) {
                out.audit(f.dir, &label, &f.payload);
            } else if out.wants_raw() {
                out.frp(FrpEvent::raw_frame(RawFrame {
                    timestamp: f.at_ms,
                    label: Some(label),
                    ..RawFrame::new(f.dir, 0, f.payload)
                }));
            }
        }
        received
    }
}

/// Whether frames of `type_id` go only to the audit log, never out as raw
/// frames: those that carry identity ([`carries_identity`]), the device's
/// free text (0xE3) and its DSP debug output (0xF0).
fn audit_only(type_id: u8) -> bool {
    carries_identity(type_id) || AUDIT_ONLY.contains(&type_id)
}

/// The latest full (0xD4) and early (0xE8) result payloads, for the log
/// line of a result that is dropped.
#[derive(Default)]
struct LastResults {
    d4: Vec<u8>,
    e8: Vec<u8>,
}

/// The shot in progress.
struct Shot {
    key: ShotKey,
    /// The detection mode at the trigger.
    mode: DetectionMode,
    /// A full result was accepted and its launch sent.
    had_d4: bool,
    /// The latest early result.
    e8: Option<FlightResultV1>,
    /// Club data waiting for the shot's ball result.
    club: Option<ClubData>,
    /// The shot's club data was sent.
    club_sent: bool,
}

/// One connection to a FlightScope device. See the [module docs](self).
///
/// Generic over the stream so a test can run it on an in-memory device; the
/// methods are those of [`DeviceSession`], which `MevoSession<TcpStream>`
/// implements.
pub struct MevoSession<S: Read + Write> {
    client: BinaryClient<S>,
    identity: DeviceIdentity,
    model: DeviceModel,
    config: Config,
    spin_source: Live<SpinSource>,
    ids: ShotIds,
    wire: Wire,
    results: LastResults,
    /// The detection mode the device is set to (or being set to).
    mode: DetectionMode,
    /// A mode change waiting for the device to be idle.
    pending_mode: Option<DetectionMode>,
    shot: Option<Shot>,
    /// The Fusion camera config, applied once the camera has warmed up.
    pending_fusion_cam: Option<CamConfig>,
    session_start: Instant,
    /// How long the camera warms up before the Fusion config
    /// ([`camera::WARMUP`]; tests shorten it).
    warmup: Duration,
    last_event: Instant,
    /// Telemetry as last sent.
    telemetry: HashMap<String, String>,
    /// Events of a command, sent with the next poll.
    backlog: Vec<DeviceEvent>,
}

impl<S: Read + Write> std::fmt::Debug for MevoSession<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MevoSession")
            .field("identity", &self.identity)
            .field("model", &self.model)
            .field("mode", &self.mode)
            .finish_non_exhaustive()
    }
}

impl<S: Read + Write> MevoSession<S> {
    /// The model the handshake detected.
    #[must_use]
    pub fn model(&self) -> DeviceModel {
        self.model
    }

    /// The detection mode the device is set to, or being set to.
    #[must_use]
    pub fn mode(&self) -> DetectionMode {
        self.mode
    }

    /// Who the device is. See [`DeviceSession::identity`].
    #[must_use]
    pub fn identity(&self) -> &DeviceIdentity {
        &self.identity
    }

    /// Drive the device for about `timeout`. See [`DeviceSession::poll`].
    ///
    /// # Errors
    ///
    /// Returns what ended the session.
    pub fn poll(&mut self, timeout: Duration, out: &mut Events<'_>) -> Result<(), Error> {
        let deadline = Instant::now() + timeout;
        self.flush_backlog(out);
        loop {
            self.apply_fusion_cam(out);
            let polled = self.client.poll();
            if self.wire.emit(out, &mut self.results, true) {
                self.last_event = Instant::now();
            }
            let idle = match polled {
                Ok(Some(event)) => {
                    self.last_event = Instant::now();
                    self.on_event(event, out);
                    false
                }
                Ok(None) => true,
                Err(e @ (ConnError::Wire(_) | ConnError::Protocol(_))) => {
                    could_not_process(out, &e);
                    true
                }
                Err(e) => return Err(e.into()),
            };
            // Checked on every pass, so a host polling with a zero timeout
            // sees a dead device too.
            if self.last_event.elapsed() >= STALE_TIMEOUT {
                return Err(Error::Stale);
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            if idle {
                std::thread::sleep(left.min(IDLE_SLEEP));
            }
        }
        Ok(())
    }

    /// Take the mode of a host command. A mode that changes what is sent to
    /// the device reconfigures and re-arms it (at once when it is armed,
    /// else once it is idle); one that does not (Xi chipping and putting)
    /// is only latched. The device has no hand or club setting. See
    /// [`DeviceSession::command`].
    ///
    /// What the change emits (log lines, `ready=false`) goes out with the
    /// next [`poll`](Self::poll).
    ///
    /// # Errors
    ///
    /// Never; the signature is the trait's.
    pub fn command(&mut self, cmd: &DeviceCommand) -> Result<(), Error> {
        let flags = HostFlags::new();
        let mut events = Vec::new();
        {
            let mut sink = |e| events.push(e);
            self.take_command(*cmd, &mut Events::new(&mut sink, &flags));
        }
        self.backlog.extend(events);
        Ok(())
    }

    fn take_command(&mut self, cmd: DeviceCommand, out: &mut Events<'_>) {
        if cmd.handed.is_some() || cmd.club.is_some() {
            out.log(
                Level::Debug,
                format!(
                    "hand {:?} and club {:?} don't apply to the device",
                    cmd.handed, cmd.club
                ),
            );
        }
        let Some(new) = cmd.mode else {
            return;
        };
        if new == self.mode && self.pending_mode.is_none() {
            return;
        }
        let label = self.config.wire_label(self.model, new);
        if !self.config.changes_wire(self.model, self.mode, new) {
            out.log(
                Level::Info,
                format!(
                    "mode change: {} -> {new} -- device stays in {label}",
                    self.mode
                ),
            );
            self.mode = new;
            self.pending_mode = None;
        } else if self.client.is_armed() {
            out.log(
                Level::Info,
                format!("mode change: {} -> {new} wire={label}", self.mode),
            );
            self.mode = new;
            self.pending_mode = None;
            self.queue_configure(true, out);
        } else {
            out.log(
                Level::Info,
                format!("deferred mode change to {new} (operation in progress)"),
            );
            self.pending_mode = Some(new);
        }
    }

    /// Release the device: `ready=false`, then the connection is dropped.
    /// See [`DeviceSession::close`].
    ///
    /// # Errors
    ///
    /// Never; the signature is the trait's.
    pub fn close(mut self, out: &mut Events<'_>) -> Result<(), Error> {
        self.wire.emit(out, &mut self.results, true);
        self.flush_backlog(out);
        out.ready(false);
        Ok(())
    }

    /// Send what a command emitted.
    fn flush_backlog(&mut self, out: &mut Events<'_>) {
        for event in std::mem::take(&mut self.backlog) {
            match event {
                DeviceEvent::Log(level, line) => out.log(level, line),
                event => out.emit(event),
            }
        }
    }

    // -- Connect -------------------------------------------------------------

    /// Report the handshake, then queue the configuration and arm for the
    /// detected model.
    fn handshake_done(&mut self, h: &HandshakeOutcome, out: &mut Events<'_>) {
        let dsp = &h.dsp.status;
        let battery = if dsp.is_decoded() {
            format!("{}%", dsp.battery_percent())
        } else {
            "unknown".to_owned()
        };
        let charging = dsp.is_decoded() && dsp.external_power();
        out.log(
            Level::Info,
            format!(
                "handshake complete: {} firmware {} | battery {battery}{} | tilt {:.1} roll {:.1}",
                self.identity.model.as_deref().unwrap_or("unknown model"),
                self.identity.firmware.as_deref().unwrap_or("unknown"),
                if charging { " (charging)" } else { "" },
                h.avr.status.tilt,
                -h.avr.status.roll,
            ),
        );
        // The whole device info text carries more than the firmware.
        out.log(
            Level::Trace,
            format!("AVR device info: {}", h.avr.dev_info.text),
        );
        let mut pairs = avr_telemetry(&h.avr.status);
        pairs.extend(dsp_telemetry(dsp));
        pairs.push(("ready", "false".to_owned()));
        self.send_telemetry(out, pairs);

        if self.model == DeviceModel::Xi {
            out.log(
                Level::Info,
                format!(
                    "FlightScope Xi: configured for {}; ball type, tee height, track % and camera mode do not apply",
                    self.config.wire_label(self.model, self.mode)
                ),
            );
            self.queue_configure(false, out);
        } else {
            self.pending_fusion_cam = match self.config.camera_mode() {
                CameraMode::Standard => None,
                CameraMode::Fusion => Some(CamConfig::fusion_preset()),
                CameraMode::RawFusion => Some(CamConfig::raw_fusion_preset()),
            };
            out.phase("configuring");
            self.client
                .configure_avr(self.config.mevo_settings(self.mode));
            self.client.configure_cam(CamConfig::warmup_preset());
            self.client.arm();
        }
    }

    // -- Poll ----------------------------------------------------------------

    /// Queue the configuration for the current mode, then an arm. A Mevo+
    /// gets the mode alone on a mode change and its full settings otherwise;
    /// the Xi gets its configuration (the full set on the connection's first
    /// one, the mode set after that).
    fn queue_configure(&mut self, mode_change: bool, out: &mut Events<'_>) {
        out.phase("configuring");
        match self.model {
            DeviceModel::Xi => self.client.configure_xi(self.config.xi_settings(self.mode)),
            DeviceModel::Mevo if mode_change => self
                .client
                .configure_avr(self.config.mevo_mode_settings(self.mode)),
            DeviceModel::Mevo => self
                .client
                .configure_avr(self.config.mevo_settings(self.mode)),
        }
        self.client.arm();
        self.send_telemetry(out, [("ready", "false".to_owned())]);
    }

    /// Apply a mode change that waited for the device to be idle.
    fn apply_pending_mode(&mut self, out: &mut Events<'_>) {
        let Some(target) = self.pending_mode.take() else {
            return;
        };
        let label = self.config.wire_label(self.model, target);
        let changes = self.config.changes_wire(self.model, self.mode, target);
        let mode_change = target != self.mode;
        self.mode = target;
        if !changes {
            out.log(
                Level::Info,
                format!("deferred mode change to {target} -- device stays in {label}"),
            );
            return;
        }
        out.log(
            Level::Info,
            format!("applying deferred mode change to {target} wire={label}"),
        );
        self.queue_configure(mode_change, out);
    }

    /// Apply the Fusion camera config once the camera has warmed up, between
    /// shots.
    fn apply_fusion_cam(&mut self, out: &mut Events<'_>) {
        if self.pending_fusion_cam.is_none()
            || self.shot.is_some()
            || self.session_start.elapsed() < self.warmup
        {
            return;
        }
        if let Some(cam) = self.pending_fusion_cam.take() {
            out.log(
                Level::Info,
                "camera warmup complete -- applying the Fusion config",
            );
            out.phase("configuring");
            self.client.configure_cam(cam);
            self.client.arm();
            self.send_telemetry(out, [("ready", "false".to_owned())]);
        }
    }

    fn on_event(&mut self, event: BinaryEvent, out: &mut Events<'_>) {
        match event {
            BinaryEvent::Handshake(_) => {}
            BinaryEvent::Disarmed => {
                out.log(Level::Info, "device disarmed (preparing for reconfigure)");
            }
            BinaryEvent::Configured => {
                let wire = self.config.wire_label(self.model, self.mode);
                out.log(Level::Info, format!("device configured (mode={wire})"));
            }
            BinaryEvent::Armed => {
                let wire = self.config.wire_label(self.model, self.mode);
                out.log(
                    Level::Info,
                    format!("armed -- mode={wire} -- waiting for shots"),
                );
                out.phase("measuring");
                self.send_telemetry(out, [("ready", "true".to_owned())]);
                self.apply_pending_mode(out);
            }
            BinaryEvent::Trigger => self.on_trigger(out),
            BinaryEvent::ShotDatum(datum) => self.on_datum(datum, out),
            BinaryEvent::ShotComplete(_) => {
                self.finish_shot(out);
                // The client re-arms after a shot and reports the completed
                // shot rather than an `Armed` event.
                self.send_telemetry(out, [("ready", "true".to_owned())]);
                self.apply_pending_mode(out);
            }
            BinaryEvent::Keepalive(status) => {
                let mut pairs = Vec::new();
                if let Some(avr) = &status.avr {
                    pairs.extend(avr_telemetry(avr));
                }
                if let Some(dsp) = &status.dsp {
                    pairs.extend(dsp_telemetry(dsp));
                }
                self.send_telemetry(out, pairs);
            }
            BinaryEvent::StepRefused(env) => {
                out.log(
                    Level::Warn,
                    format!(
                        "device refused a step, continuing: {} 0x{:02X} {}",
                        env.src,
                        env.type_id,
                        hex(&env.raw)
                    ),
                );
            }
            BinaryEvent::Message(env) => match &env.message {
                Message::DspStatus(dsp) => self.send_telemetry(out, dsp_telemetry(dsp)),
                Message::AvrStatus(avr) => self.send_telemetry(out, avr_telemetry(avr)),
                _ => {}
            },
        }
    }

    fn on_trigger(&mut self, out: &mut Events<'_>) {
        // The device triggered again before it finished the open shot: that
        // shot ends here, with what it already sent.
        if let Some(open) = self.shot.take() {
            out.log(
                Level::Debug,
                format!(
                    "shot #{} ended by a new trigger before the device finished it",
                    open.key.shot_number
                ),
            );
            out.shot_finished(open.key);
        }
        let key = self.ids.next_key();
        out.log(
            Level::Debug,
            format!("ball trigger detected -- shot #{}", key.shot_number),
        );
        match now_ms() {
            Some(ms) => out.shot_trigger_at(key.clone(), ms),
            None => out.shot_trigger(key.clone()),
        }
        self.shot = Some(Shot {
            key,
            mode: self.mode,
            had_d4: false,
            e8: None,
            club: None,
            club_sent: false,
        });
        self.send_telemetry(out, [("ready", "false".to_owned())]);
    }

    fn on_datum(&mut self, datum: ShotDatum, out: &mut Events<'_>) {
        let spin_source = self.spin_source.get();
        let model = self.model;
        let Some(shot) = self.shot.as_mut() else {
            return;
        };
        match datum {
            ShotDatum::Flight(d4) if d4.is_model_failure() => {
                // The device's distance model failed (e.g. an outdoor
                // full-flight shot into a screen): treat the shot as having
                // no full result, so the early result can stand in.
                out.log(
                    Level::Warn,
                    format!(
                        "shot #{} D4 dropped: launch speed {} (distance model failed) -- 0x{:02X} {}",
                        shot.key.shot_number,
                        d4.launch_speed,
                        TYPE_FLIGHT_RESULT,
                        hex(&self.results.d4),
                    ),
                );
            }
            // Each lifecycle event goes out once: a later full result of
            // the same shot is not sent.
            ShotDatum::Flight(_) if shot.had_d4 => {}
            ShotDatum::Flight(d4) => {
                shot.had_d4 = true;
                let only = convert::launch_only(&d4);
                let mut launch = convert::launch(&d4);
                if only {
                    launch.backspin_rpm = None;
                    launch.sidespin_rpm = None;
                    launch.total_spin_rpm = None;
                    launch.spin_axis = None;
                }
                let mut label = if only { "none (launch only)" } else { "D4" };
                if model == DeviceModel::Xi {
                    let (spin, l) = xi_spin(spin_source, shot.mode, &d4, shot.e8.as_ref());
                    if let Some(spin) = spin {
                        spin.apply(&mut launch);
                        label = l;
                    } else if !only {
                        label = l;
                    }
                }
                out.log(Level::Info, shot_line(&shot.key, "D4", &launch, &d4, label));
                out.launch(shot.key.clone(), launch);
                if !only {
                    out.flight_result(shot.key.clone(), convert::flight_result(&d4));
                }
                if let Some(club) = shot.club.take() {
                    shot.club_sent = true;
                    out.club_path(shot.key.clone(), club);
                }
            }
            ShotDatum::FlightV1(e8) => shot.e8 = Some(e8),
            ShotDatum::Club(_) if shot.club_sent => {}
            ShotDatum::Club(ed) => {
                let club = convert::club_data(&ed);
                if shot.had_d4 {
                    shot.club_sent = true;
                    out.club_path(shot.key.clone(), club);
                } else {
                    shot.club = Some(club);
                }
            }
            ShotDatum::Spin(_) => {}
        }
    }

    /// Send what the finished shot still sends (the early result's ball, the
    /// club data held back, or an alert), then `shot_finished`.
    fn finish_shot(&mut self, out: &mut Events<'_>) {
        let Some(shot) = self.shot.take() else {
            return;
        };
        let n = shot.key.shot_number;
        let mut club = shot.club;
        if !shot.had_d4 {
            match shot.e8 {
                Some(e8) if e8.is_no_read() => {
                    club = None;
                    out.log(
                        Level::Warn,
                        format!(
                            "shot #{n} not read: E8 ball speed {} and no D4 -- 0x{:02X} {}",
                            e8.ball_velocity,
                            TYPE_FLIGHT_RESULT_V1,
                            hex(&self.results.e8),
                        ),
                    );
                    out.alert(
                        Severity::Warn,
                        format!(
                            "Shot #{n}: {} didn't track the ball (no read)",
                            self.model.label()
                        ),
                    );
                }
                _ if !self.config.use_estimated() => {
                    out.log(
                        Level::Info,
                        format!(
                            "shot #{n} had only E8 (estimated) -- skipped (use_estimated=false)"
                        ),
                    );
                }
                Some(e8) => {
                    let launch = convert::early_launch(&e8);
                    let flight = convert::early_flight_result(&e8);
                    out.log(
                        Level::Info,
                        format!(
                            "shot #{n} (E8): ball={:.1}mph VLA={:.1} HLA={:.1} carry={:.1}yd",
                            launch
                                .launch_speed
                                .map_or(0.0, flightrelay::Velocity::as_mph),
                            launch.launch_elevation.unwrap_or(0.0),
                            launch.launch_azimuth.unwrap_or(0.0),
                            flight
                                .carry_distance
                                .map_or(0.0, flightrelay::Distance::as_yards),
                        ),
                    );
                    out.launch(shot.key.clone(), launch);
                    out.flight_result(shot.key.clone(), flight);
                }
                None => {
                    out.log(
                        Level::Warn,
                        format!("shot #{n} processed but no flight result (D4 or E8)"),
                    );
                    out.alert(
                        Severity::Warn,
                        format!("Shot #{n} triggered but no flight result produced"),
                    );
                }
            }
        }
        if let Some(club) = club {
            out.club_path(shot.key.clone(), club);
        }
        out.shot_finished(shot.key);
    }

    /// Send the telemetry values that changed since they were last sent.
    fn send_telemetry(
        &mut self,
        out: &mut Events<'_>,
        pairs: impl IntoIterator<Item = (&'static str, String)>,
    ) {
        let changed: Vec<(&str, String)> = pairs
            .into_iter()
            .filter(|(k, v)| self.telemetry.get(*k) != Some(v))
            .collect();
        if changed.is_empty() {
            return;
        }
        for (k, v) in &changed {
            self.telemetry.insert((*k).to_owned(), v.clone());
        }
        out.telemetry(changed);
    }
}

impl DeviceSession for MevoSession<TcpStream> {
    type Driver = Mevo;

    fn poll(&mut self, timeout: Duration, out: &mut Events<'_>) -> Result<(), Error> {
        MevoSession::<TcpStream>::poll(self, timeout, out)
    }

    fn command(&mut self, cmd: &DeviceCommand) -> Result<(), Error> {
        MevoSession::<TcpStream>::command(self, cmd)
    }

    fn identity(&self) -> &DeviceIdentity {
        &self.identity
    }

    fn close(self, out: &mut Events<'_>) -> Result<(), Error> {
        MevoSession::<TcpStream>::close(self, out)
    }
}

/// `tilt` and `roll` (degrees; the device reports roll with the opposite
/// sign) from an AVR status.
fn avr_telemetry(avr: &AvrStatus) -> Vec<(&'static str, String)> {
    vec![
        ("tilt", format!("{:.1}", avr.tilt)),
        ("roll", format!("{:.1}", -avr.roll)),
    ]
}

/// `battery_pct`, `temp_c` and `external_power` from a DSP status whose
/// layout is decoded; nothing otherwise.
fn dsp_telemetry(dsp: &DspStatus) -> Vec<(&'static str, String)> {
    if !dsp.is_decoded() {
        return Vec::new();
    }
    vec![
        ("battery_pct", dsp.battery_percent().to_string()),
        ("temp_c", format!("{:.1}", dsp.temperature_c())),
        ("external_power", dsp.external_power().to_string()),
    ]
}

/// Log a message the session could not process and raise an alert; the
/// session carries on. The alert and the warning name the error without its
/// bytes, which can carry identity (a short status or device info frame);
/// the bytes go to the audit log only.
fn could_not_process(out: &mut Events<'_>, e: &ConnError) {
    let text = e.describe();
    out.log(Level::Warn, format!("could not process message: {text}"));
    out.alert(Severity::Warn, format!("Could not process message: {text}"));
    match e {
        ConnError::Wire(w) => {
            if let Some(payload) = w.payload() {
                out.audit(RawDir::Rx, "undecoded", payload);
            }
        }
        e => out.log(Level::Trace, format!("could not process message: {e}")),
    }
}

/// The info log line of a shot's full result.
fn shot_line(key: &ShotKey, kind: &str, launch: &Launch, d4: &FlightResult, spin: &str) -> String {
    format!(
        "shot #{} ({kind}): ball={:.1}mph VLA={:.1} HLA={:.1} carry={:.1}yd back={} side={} total={} spin={spin}",
        key.shot_number,
        launch
            .launch_speed
            .map_or(0.0, flightrelay::Velocity::as_mph),
        launch.launch_elevation.unwrap_or(0.0),
        launch.launch_azimuth.unwrap_or(0.0),
        flightrelay::Distance::Meters(d4.carry_distance).as_yards(),
        rpm(launch.backspin_rpm),
        rpm(launch.sidespin_rpm),
        rpm(launch.total_spin_rpm),
    )
}

fn rpm(v: Option<i32>) -> String {
    v.map_or_else(|| "-".to_owned(), |v| format!("{v}rpm"))
}

/// The spin an Xi shot sends in place of the full result's ([`xi_spin`]).
/// Both are the device's readings: splitting the early-result total by the
/// full result's axis only expresses it as the other spin pair.
#[derive(Debug, Clone, Copy, PartialEq)]
enum XiSpin {
    /// The early-result total and the full result's axis (FRP sign,
    /// degrees), with the backspin and FRP sidespin they split into.
    Components {
        back: i32,
        side: i32,
        total: i32,
        axis: f64,
    },
    /// The early-result total alone: the shot has no axis to split it by.
    Total(i32),
}

impl XiSpin {
    /// Put this spin on `launch` in place of the full result's.
    fn apply(self, launch: &mut Launch) {
        launch.total_spin_rpm = None;
        launch.spin_axis = None;
        match self {
            Self::Components {
                back,
                side,
                total,
                axis,
            } => {
                launch.backspin_rpm = Some(back);
                launch.sidespin_rpm = Some(side);
                launch.total_spin_rpm = Some(total);
                launch.spin_axis = Some(axis);
            }
            Self::Total(total) => {
                launch.backspin_rpm = None;
                launch.sidespin_rpm = None;
                launch.total_spin_rpm = Some(total);
            }
        }
    }
}

/// Xi spin for `source`: the spin to use instead of the full result's, or
/// `None` to keep it, and a label naming the source for the log line.
///
/// `D4` keeps the full result's. `E8` takes the total from the early
/// result's backspin; when the shot has club data (club speed plus attack
/// angle or path) and a full-result backspin, the total is split by the full
/// result's spin axis (clamped to ±90° so backspin never turns negative),
/// otherwise it goes out alone. `E8` keeps the full result's spin when there
/// is no early-result backspin, a missing value. `Auto` is `E8` when `mode`
/// (the mode at the trigger) is full swing, else `D4`.
fn xi_spin(
    source: SpinSource,
    mode: DetectionMode,
    d4: &FlightResult,
    e8: Option<&FlightResultV1>,
) -> (Option<XiSpin>, &'static str) {
    match source {
        SpinSource::D4 => (None, "D4"),
        SpinSource::E8 => xi_spin_e8(d4, e8),
        SpinSource::Auto if mode == DetectionMode::Full => {
            let (spin, label) = xi_spin_e8(d4, e8);
            let label = match label {
                "D4 (no E8 spin)" => "D4 (no E8 spin) (auto)",
                "E8 total, no axis" => "E8 total, no axis (auto)",
                _ => "E8 total, D4 axis (auto)",
            };
            (spin, label)
        }
        SpinSource::Auto => (None, "D4 (auto)"),
    }
}

/// The `E8` rule of [`xi_spin`].
#[allow(clippy::cast_possible_truncation)] // |total| fits an i32, so its parts do
fn xi_spin_e8(d4: &FlightResult, e8: Option<&FlightResultV1>) -> (Option<XiSpin>, &'static str) {
    let Some(e8) = e8.filter(|e8| e8.backspin_rpm > 0) else {
        return (None, "D4 (no E8 spin)");
    };
    let has_club_data =
        d4.clubhead_speed > 0.0 && (d4.club_attack_angle != 0.0 || d4.club_strike_direction != 0.0);
    if !(has_club_data && d4.backspin_rpm > 0) {
        return (Some(XiSpin::Total(e8.backspin_rpm)), "E8 total, no axis");
    }
    let total = f64::from(e8.backspin_rpm);
    let axis = f64::from(d4.sidespin_rpm)
        .atan2(f64::from(d4.backspin_rpm))
        .clamp(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
    let back = (total * axis.cos()).round() as i32;
    let side = -(total * axis.sin()).round() as i32;
    let spin = XiSpin::Components {
        back,
        side,
        total: e8.backspin_rpm,
        // The device's axis is positive to the left; FRP's to the right.
        axis: -axis.to_degrees(),
    };
    (Some(spin), "E8 total, D4 axis")
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .fold(String::with_capacity(bytes.len() * 2), |mut s, b| {
            let _ = write!(s, "{b:02X}");
            s
        })
}

/// The current UTC time in milliseconds since the Unix epoch.
fn now_ms() -> Option<u64> {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .ok()
        .and_then(|d| u64::try_from(d.as_millis()).ok())
}

#[cfg(test)]
mod tests;
