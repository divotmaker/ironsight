//! Non-blocking client for the FlightScope binary protocol (port 5100).
//!
//! Wraps [`BinaryConnection`] and the sequencer state machines from [`seq`]
//! into a single `BinaryClient` with a `poll() -> Option<BinaryEvent>` entry
//! point. Callers enqueue operations (handshake, configure, arm) and the
//! client drives them one at a time, emitting [`BinaryEvent`]s as milestones
//! complete.
//!
//! Keepalives are managed automatically after the first arm — queued as
//! proper operations so they never interleave with other sequencers.
//!
//! The handshake detects the model family ([`DeviceModel`]) from the DSP
//! device info and adapts the rest of the session to it: on the Xi, which has
//! no camera Pi, the PI handshake phase, camera configuration and PI status
//! polls are skipped.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use crate::addr::BusAddr;
use crate::conn::{BinaryConnection, ConnError, Envelope};
use crate::protocol::Message;
use crate::protocol::camera::CamConfig;
use crate::protocol::handshake::DeviceModel;
use crate::protocol::status::{AvrStatus, DspStatus, PiStatus};
use crate::seq::{
    self, Action, ArmSequencer, AvrConfigSequencer, AvrSequencer, AvrSettings, AvrSync,
    CameraConfigSequencer, DisarmSequencer, DspSequencer, DspSync, PiSequencer, PiSync, Sequence,
    ShotData, ShotDatum, ShotSequencer, XiConfigSequencer, XiSettings,
};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Events emitted by [`BinaryClient::poll()`].
#[derive(Debug)]
pub enum BinaryEvent {
    /// Handshake (DSP + AVR sync, plus PI sync on models with a camera)
    /// complete.
    Handshake(HandshakeOutcome),
    /// Device disarmed (explicit disarm before re-configure).
    Disarmed,
    /// AVR config or camera config applied. Camera configuration on a model
    /// without a camera sends nothing and reports `Configured` straight away.
    Configured,
    /// Device armed, ready for shots.
    Armed,
    /// Ball detected (`ShotText` "BALL TRIGGER").
    Trigger,
    /// Individual shot data yielded during the shot lifecycle (between
    /// `Trigger` and `ShotComplete`). React to these for real-time
    /// processing — e.g. start ball flight simulation on `Flight`,
    /// display club data on `Club`.
    ShotDatum(ShotDatum),
    /// Shot processing complete, device re-armed. Carries the full
    /// accumulated [`ShotData`] for convenience. Non-streaming callers
    /// can ignore `ShotDatum` events and use this exclusively.
    ShotComplete(Box<ShotData>),
    /// Keepalive round-trip complete. Contains the latest cached status
    /// from DSP/AVR/PI responses. Useful for staleness detection and
    /// telemetry updates.
    Keepalive(StatusSnapshot),
    /// The device refused (0x94) a step of the Xi configuration or the Xi
    /// DSP power-mode write, and the operation carried on with its next
    /// step. Carries the 0x94 frame: `raw` is `[02 bus type & 0x7F]`, the
    /// refusing bus and the refused command. Worth logging as a warning
    /// with that hex. Emitted before the operation's own completion event.
    StepRefused(Envelope),
    /// A camera configuration ended without the device finishing it: the
    /// device refused a step (0x94), or did not answer one within
    /// [`CAMERA_SETUP_TIMEOUT`]. The session is intact; the client goes on
    /// with the next queued operation, so a queued arm still runs. The
    /// camera is left as the device has it (a Fusion configuration that
    /// did not finish leaves it in standard mode).
    CameraSetupUnfinished {
        /// The step the configuration stopped at, in words.
        step: &'static str,
        /// The configuration was for a Fusion mode
        /// ([`CamConfig::is_fusion`]).
        fusion: bool,
        /// The 0x94 that refused the step; `None` when the device did not
        /// answer in time. Log its hex.
        refused: Option<Envelope>,
    },
    /// Any message not consumed by the active operation.
    Message(Envelope),
}

/// Combined results from the handshake.
#[derive(Debug, Clone)]
pub struct HandshakeOutcome {
    /// Model family, detected from the DSP device info.
    pub model: DeviceModel,
    pub dsp: DspSync,
    pub avr: AvrSync,
    /// PI sync results. `None` on models without a camera Pi (the Xi).
    pub pi: Option<PiSync>,
}

/// Latest cached status from keepalive responses.
#[derive(Debug, Clone, Default)]
pub struct StatusSnapshot {
    pub dsp: Option<DspStatus>,
    pub avr: Option<AvrStatus>,
    pub pi: Option<PiStatus>,
}

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

/// Operation waiting in the FIFO queue.
enum QueuedOp {
    Handshake,
    ConfigureAvr(AvrSettings),
    ConfigureXi(XiSettings),
    ConfigureCam(CamConfig),
    Arm,
    Keepalive,
}

/// Currently executing operation (at most one at a time).
#[allow(clippy::large_enum_variant)] // Handshake carries DspSync+AvrSync as intermediate results
enum ActiveOp {
    Handshake {
        phase: Box<HandshakePhase>,
        dsp: Option<DspSync>,
        avr: Option<AvrSync>,
    },
    Disarm(DisarmSequencer),
    Configure(ConfigurePhase),
    Arm(ArmSequencer),
    Shot(Box<ShotSequencer>),
    Keepalive(KeepaliveSequencer),
}

/// Handshake runs 3 sequencers in series.
enum HandshakePhase {
    Dsp(DspSequencer),
    Avr(AvrSequencer),
    Pi(PiSequencer),
}

/// Single-phase configure operations (AVR, Xi or camera, independently).
enum ConfigurePhase {
    Avr(AvrConfigSequencer),
    Xi(XiConfigSequencer),
    Camera(CameraConfigSequencer),
}

// ---------------------------------------------------------------------------
// KeepaliveSequencer
// ---------------------------------------------------------------------------

/// Sends StatusPolls (DSP + AVR, plus PI on models with a camera) and
/// collects the responses.
struct KeepaliveSequencer {
    got_dsp: bool,
    got_avr: bool,
    got_pi: bool,
}

impl KeepaliveSequencer {
    fn new(model: DeviceModel) -> (Self, Vec<Action>) {
        (
            Self {
                got_dsp: false,
                got_avr: false,
                got_pi: !model.has_camera(),
            },
            seq::keepalive_polls(model),
        )
    }
}

impl Sequence for KeepaliveSequencer {
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
        match (&env.message, env.src) {
            (Message::DspStatus(_), BusAddr::Dsp) => self.got_dsp = true,
            (Message::AvrStatus(_), BusAddr::Avr) => self.got_avr = true,
            (Message::PiStatus(_), BusAddr::Pi) => self.got_pi = true,
            _ => {}
        }
        vec![]
    }

    fn is_complete(&self) -> bool {
        self.got_dsp && self.got_avr && self.got_pi
    }
}

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------

const DEFAULT_OP_TIMEOUT: Duration = Duration::from_secs(30);
const DEFAULT_KEEPALIVE_INTERVAL: Duration = Duration::from_secs(1);
const KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(3);

/// How long a camera configuration may take before the client gives up on
/// it and reports [`BinaryEvent::CameraSetupUnfinished`]. The camera's
/// replies come within a second or two. It is shorter than the driver's
/// 10 s silence limit, so an unfinished setup is reported before a silent
/// device ends the session; the other operations keep the operation
/// timeout, whose expiry ends the session.
pub const CAMERA_SETUP_TIMEOUT: Duration = Duration::from_secs(8);

/// Maximum time between TRIGGER and PROCESSED before we give up on the
/// shot and resume keepalives. Without this, a lost PROCESSED message
/// permanently suppresses keepalives (shot_in_progress stuck true),
/// which prevents staleness detection from firing.
const TRIGGER_TIMEOUT: Duration = Duration::from_secs(30);

// ---------------------------------------------------------------------------
// BinaryClient
// ---------------------------------------------------------------------------

/// Non-blocking client for the FlightScope binary protocol (port 5100).
///
/// Wraps a [`BinaryConnection`] and provides a single [`poll()`](Self::poll)
/// entry point that drives sequencers, manages keepalives, and emits
/// [`BinaryEvent`]s.
///
/// # Usage
///
/// ```no_run
/// use ironsight::client::{BinaryClient, BinaryEvent};
/// use ironsight::BinaryConnection;
/// use ironsight::conn::DEFAULT_ADDR;
///
/// let conn = BinaryConnection::connect(DEFAULT_ADDR)?;
/// let mut client = BinaryClient::from_tcp(conn)?;
/// client.handshake();
///
/// loop {
///     match client.poll()? {
///         Some(BinaryEvent::Handshake(h)) => {
///             println!("Connected to {}", h.model);
///             // client.configure_avr(settings);
///             // client.configure_cam(cam_config);
///         }
///         Some(event) => println!("{event:?}"),
///         None => {}
///     }
/// }
/// # Ok::<(), ironsight::ConnError>(())
/// ```
pub struct BinaryClient<S: Read + Write> {
    conn: BinaryConnection<S>,
    queue: VecDeque<QueuedOp>,
    active: Option<ActiveOp>,
    op_deadline: Option<Instant>,

    // Keepalive state
    keepalive_enabled: bool,
    keepalive_queued: bool,
    /// When the last keepalive ended; `None` until the first poll, which
    /// starts the interval.
    last_keepalive: Option<Instant>,
    keepalive_interval: Duration,

    // Configuration
    op_timeout: Duration,
    camera_timeout: Duration,

    // Cached state
    status: StatusSnapshot,
    device: Option<HandshakeOutcome>,
    armed: bool,
    /// True between `Trigger` and `ShotComplete`. During this window,
    /// pre-PROCESSED messages (E8) are intercepted and yielded as
    /// `ShotDatum` events instead of passing through as `Message`.
    shot_in_progress: bool,
    /// When `shot_in_progress` was set. Used by the trigger watchdog to
    /// detect a missing PROCESSED and resume keepalives.
    trigger_time: Option<Instant>,
    /// Shot data that arrived between TRIGGER and PROCESSED, where the
    /// results (D4/ED/D9/EF) normally arrive. Seeds the shot sequencer on
    /// PROCESSED.
    early_shot: ShotData,
    /// Event produced without sending anything (camera configuration on a
    /// model without a camera). Returned by the next `poll()`.
    ready_event: Option<BinaryEvent>,
    /// The Xi's connect-time configuration block has been sent on this
    /// connection.
    xi_connect_sent: bool,
}

impl<S: Read + Write> BinaryClient<S> {
    /// Create a new client wrapping the given connection.
    ///
    /// The underlying stream **must** be non-blocking (or have a short read
    /// timeout). [`poll()`](Self::poll) relies on `recv()` returning
    /// `Ok(None)` promptly so it can check operation deadlines, fire
    /// keepalives, and yield control back to the caller. A blocking stream
    /// will stall the entire poll loop until a message arrives.
    ///
    /// For `TcpStream`, use [`from_tcp()`](Self::from_tcp) which sets
    /// non-blocking mode automatically.
    pub fn new(conn: BinaryConnection<S>) -> Self {
        Self {
            conn,
            queue: VecDeque::new(),
            active: None,
            op_deadline: None,
            keepalive_enabled: false,
            keepalive_queued: false,
            last_keepalive: None,
            keepalive_interval: DEFAULT_KEEPALIVE_INTERVAL,
            op_timeout: DEFAULT_OP_TIMEOUT,
            camera_timeout: CAMERA_SETUP_TIMEOUT,
            status: StatusSnapshot::default(),
            device: None,
            armed: false,
            shot_in_progress: false,
            trigger_time: None,
            early_shot: ShotData::default(),
            ready_event: None,
            xi_connect_sent: false,
        }
    }

    /// Poll the client for the next event.
    ///
    /// Drives the active operation (if any), processes incoming messages,
    /// and manages keepalive scheduling. Returns `Ok(None)` when no data
    /// is available (non-blocking).
    ///
    /// On the Xi, every 0xE8 and 0xD4 flight result is acked with 0x95 as
    /// soon as it is received.
    ///
    /// # Errors
    ///
    /// Returns `Err` on I/O errors, wire decode errors, disconnection,
    /// or operation timeout (except keepalive timeouts, which are non-fatal).
    pub fn poll(&mut self) -> Result<Option<BinaryEvent>, ConnError> {
        self.poll_at(Instant::now())
    }

    /// [`poll()`](Self::poll) at `now`: every timer of the client (the
    /// operation deadlines, the keepalive interval, the trigger watchdog and
    /// the sequencers' timed steps) compares against `now`, and the client
    /// reads no clock of its own. Pass a time that never goes backward.
    ///
    /// # Errors
    ///
    /// As [`poll()`](Self::poll).
    pub fn poll_at(&mut self, now: Instant) -> Result<Option<BinaryEvent>, ConnError> {
        self.last_keepalive.get_or_insert(now);

        // 0. Event produced by an operation that sent nothing.
        if let Some(event) = self.ready_event.take() {
            return Ok(Some(event));
        }

        // 1. Check operation deadline.
        if let Some(deadline) = self.op_deadline
            && now >= deadline
        {
            if matches!(
                self.active,
                Some(ActiveOp::Configure(ConfigurePhase::Camera(_)))
            ) {
                // A camera setup the device never finishes is non-fatal.
                return Ok(self.give_up_camera());
            }
            if matches!(self.active, Some(ActiveOp::Keepalive(_))) {
                // Keepalive timeout is non-fatal. Update last_keepalive
                // so we don't immediately re-queue another one.
                self.active = None;
                self.op_deadline = None;
                self.keepalive_queued = false;
                self.last_keepalive = Some(now);
            } else {
                return Err(ConnError::Timeout);
            }
        }

        // 2. Start next queued operation if idle.
        if self.active.is_none()
            && let Some(queued) = self.queue.pop_front()
        {
            self.start_op(queued, now)?;
            if let Some(event) = self.ready_event.take() {
                return Ok(Some(event));
            }
        }

        // 3. Handshake timeout checks: optional DSP query and AVR cal steps.
        if let Some(ActiveOp::Handshake { ref mut phase, .. }) = self.active {
            match **phase {
                HandshakePhase::Dsp(ref mut dsp) => {
                    for a in dsp.check_timers(now) {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    // On the Xi a timed-out power-mode write is the last
                    // DSP step.
                    if dsp.is_complete() {
                        return self.advance_phase(now);
                    }
                }
                HandshakePhase::Avr(ref mut avr) => {
                    for a in avr.check_timers(now) {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    // On the Xi a timed-out IF cal is the last AVR step.
                    if avr.is_complete() {
                        return self.advance_phase(now);
                    }
                }
                HandshakePhase::Pi(_) => {}
            }
        }

        // 3b. Shot drain timeout — if IDLE never arrives after shot data,
        //     skip ShotResultReq and proceed directly to ARM.
        if let Some(ActiveOp::Shot(ref mut seq)) = self.active {
            let timeout_actions = seq.check_timers(now);
            for a in timeout_actions {
                seq::send_action(&mut self.conn, a)?;
            }
        }

        // 3c. Trigger watchdog — if TRIGGER fired but PROCESSED never
        //     arrived, the session is bad. Return Timeout to force
        //     reconnect rather than leaving keepalives suppressed.
        if self.shot_in_progress
            && !matches!(self.active, Some(ActiveOp::Shot(_)))
            && self
                .trigger_time
                .is_some_and(|t| now.saturating_duration_since(t) >= TRIGGER_TIMEOUT)
        {
            return Err(ConnError::Timeout);
        }

        // 4. Auto-queue keepalive (suppress while shot is in progress —
        //    BALL TRIGGER has fired but PROCESSED hasn't arrived yet).
        if self.keepalive_enabled
            && !self.keepalive_queued
            && !self.shot_in_progress
            && self
                .last_keepalive
                .is_some_and(|t| now.saturating_duration_since(t) >= self.keepalive_interval)
        {
            self.queue.push_back(QueuedOp::Keepalive);
            self.keepalive_queued = true;
        }

        // 5. Non-blocking recv.
        let env = match self.conn.recv()? {
            Some(env) => env,
            None => return Ok(None),
        };

        // 5b. Xi result ack: answer an E8 or D4 before anything else
        //     handles the frame.
        if let Some(ack) = seq::result_ack(self.model(), &env) {
            seq::send_action(&mut self.conn, ack)?;
        }

        // 6. Update status cache.
        match (&env.message, env.src) {
            (Message::DspStatus(s), BusAddr::Dsp) => self.status.dsp = Some(s.clone()),
            (Message::AvrStatus(s), BusAddr::Avr) => self.status.avr = Some(s.clone()),
            (Message::PiStatus(s), BusAddr::Pi) => self.status.pi = Some(s.clone()),
            _ => {}
        }

        // 7. Feed active operation.
        //
        //    Keepalive gets special handling: only status responses
        //    (DspStatus/AvrStatus/PiStatus) are fed to it. All other
        //    messages skip the keepalive and fall through to shot
        //    auto-detection (step 9), so an operation only consumes
        //    messages of the response type it expects. Without this,
        //    unsolicited shot
        //    messages (BALL TRIGGER, PROCESSED, E8, etc.) that arrive
        //    while a keepalive is in-flight would be silently consumed
        //    and the shot would never be detected.
        if let Some(ref mut active) = self.active {
            let dominated = matches!(active, ActiveOp::Keepalive(_))
                && !matches!(
                    (&env.message, env.src),
                    (Message::DspStatus(_), BusAddr::Dsp)
                        | (Message::AvrStatus(_), BusAddr::Avr)
                        | (Message::PiStatus(_), BusAddr::Pi)
                );
            if !dominated {
                let result = Self::feed_active(active, &env, now, &mut self.conn)?;
                let refused = Self::take_refused(active).map(BinaryEvent::StepRefused);
                let event = match result {
                    FeedResult::Consumed => None,
                    FeedResult::Intermediate(event) => Some(*event),
                    FeedResult::PhaseComplete => self.advance_phase(now)?,
                    FeedResult::Done => self.finish_op(now)?,
                    FeedResult::CameraRefused => self.give_up_camera(),
                };
                let Some(refused) = refused else {
                    return Ok(event);
                };
                // The refusal goes first; the operation's own event follows
                // on the next poll. Neither refusing operation finishes by
                // starting another, so `ready_event` is free here.
                debug_assert!(self.ready_event.is_none());
                self.ready_event = event;
                return Ok(Some(refused));
            }
        }

        // 9. No active op — shot auto-detection.
        if let Message::ShotText(ref st) = env.message {
            if st.is_trigger() {
                self.shot_in_progress = true;
                self.trigger_time = Some(now);
                self.armed = false;
                self.early_shot = ShotData::default();
                return Ok(Some(BinaryEvent::Trigger));
            }
            if st.is_processed() {
                self.trigger_time = None;
                let (seq, actions) = ShotSequencer::with_data(std::mem::take(&mut self.early_shot));
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                // May overwrite an in-flight keepalive that was
                // dominated — reset its queued flag so future
                // keepalives aren't permanently suppressed.
                self.keepalive_queued = false;
                self.active = Some(ActiveOp::Shot(Box::new(seq)));
                self.op_deadline = Some(now + self.op_timeout);
                return Ok(None);
            }
        }

        // 10. Pre-PROCESSED shot data. E8, D4, ED, D9 and EF normally arrive
        //     between TRIGGER and PROCESSED. They are yielded as they arrive
        //     and kept for the shot sequencer.
        if self.shot_in_progress {
            match env.message {
                Message::FlightResultV1(ref e8) => {
                    return Ok(Some(BinaryEvent::ShotDatum(ShotDatum::FlightV1(
                        e8.clone(),
                    ))));
                }
                Message::FlightResult(ref r) => {
                    self.early_shot.flight = Some(r.clone());
                    return Ok(Some(BinaryEvent::ShotDatum(ShotDatum::Flight(r.clone()))));
                }
                Message::ClubResult(ref r) => {
                    self.early_shot.club = Some(r.clone());
                    return Ok(Some(BinaryEvent::ShotDatum(ShotDatum::Club(r.clone()))));
                }
                Message::SpinResult(ref r) => {
                    self.early_shot.spin = Some(r.clone());
                    return Ok(Some(BinaryEvent::ShotDatum(ShotDatum::Spin(r.clone()))));
                }
                Message::SpeedProfile(ref r) => {
                    self.early_shot.speed_profile = Some(r.clone());
                }
                _ => {}
            }
        }

        // 11. Unhandled message passthrough.
        Ok(Some(BinaryEvent::Message(env)))
    }

    // -- Public operation enqueuers -----------------------------------------

    /// Enqueue a handshake: DSP + AVR sync, then PI sync on models with a
    /// camera Pi. The model family is detected during DSP sync.
    pub fn handshake(&mut self) {
        self.queue.push_back(QueuedOp::Handshake);
    }

    /// Enqueue AVR configuration (mode, radar cal, parameters).
    ///
    /// If the device is armed when this operation starts executing, an
    /// explicit disarm is automatically prepended so the MODE_RESET
    /// completes before new config commands are sent.
    /// Emits [`BinaryEvent::Disarmed`] then [`BinaryEvent::Configured`].
    pub fn configure_avr(&mut self, avr: AvrSettings) {
        self.queue.push_back(QueuedOp::ConfigureAvr(avr));
    }

    /// Enqueue the Xi's configuration (see [`XiSettings`]).
    ///
    /// The first Xi configuration on a connection sends the connect-time
    /// block (parameter writes, radar calibration) followed by the mode
    /// block; later ones send the mode block only, for a mode change. If
    /// the device is armed when this operation starts executing, it is
    /// disarmed first. Emits [`BinaryEvent::Disarmed`] (when it disarmed)
    /// then [`BinaryEvent::Configured`]. Arm afterward with
    /// [`arm()`](Self::arm).
    pub fn configure_xi(&mut self, xi: XiSettings) {
        self.queue.push_back(QueuedOp::ConfigureXi(xi));
    }

    /// Enqueue camera configuration.
    ///
    /// An armed device is disarmed first ([`BinaryEvent::Disarmed`]); queue
    /// [`arm`](Self::arm) after it to arm again. Emits
    /// [`BinaryEvent::Configured`] on completion. A configuration the
    /// device refuses or does not finish within [`CAMERA_SETUP_TIMEOUT`]
    /// emits [`BinaryEvent::CameraSetupUnfinished`] instead, and the next
    /// queued operation runs. On a model without a
    /// camera Pi (the Xi) nothing is sent and `Configured` is emitted when the
    /// operation is reached.
    pub fn configure_cam(&mut self, cam: CamConfig) {
        self.queue.push_back(QueuedOp::ConfigureCam(cam));
    }

    /// Enqueue arming the device.
    pub fn arm(&mut self) {
        self.queue.push_back(QueuedOp::Arm);
    }

    // -- Read-only accessors ------------------------------------------------

    /// Latest cached status from keepalive responses.
    #[must_use]
    pub fn status(&self) -> &StatusSnapshot {
        &self.status
    }

    /// Handshake outcome, available after `BinaryEvent::Handshake`.
    #[must_use]
    pub fn device(&self) -> Option<&HandshakeOutcome> {
        self.device.as_ref()
    }

    /// Whether the device is currently armed.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Model family, available after `BinaryEvent::Handshake`. Before the
    /// handshake completes this is [`DeviceModel::Mevo`].
    #[must_use]
    pub fn model(&self) -> DeviceModel {
        self.device.as_ref().map_or(DeviceModel::Mevo, |d| d.model)
    }

    // -- Configuration ------------------------------------------------------

    /// Set the keepalive polling interval (default: 1s).
    pub fn set_keepalive_interval(&mut self, interval: Duration) {
        self.keepalive_interval = interval;
    }

    /// Set the operation timeout (default: 30s). Does not affect keepalives
    /// (which always use a 3s timeout) or camera configuration (see
    /// [`set_camera_setup_timeout`](Self::set_camera_setup_timeout)).
    pub fn set_operation_timeout(&mut self, timeout: Duration) {
        self.op_timeout = timeout;
    }

    /// Set how long a camera configuration may take (default:
    /// [`CAMERA_SETUP_TIMEOUT`]) before the client reports
    /// [`BinaryEvent::CameraSetupUnfinished`] and goes on.
    pub fn set_camera_setup_timeout(&mut self, timeout: Duration) {
        self.camera_timeout = timeout;
    }

    // -- Internal: start an operation ---------------------------------------

    fn start_op(&mut self, op: QueuedOp, now: Instant) -> Result<(), ConnError> {
        let timeout = match &op {
            QueuedOp::Keepalive => KEEPALIVE_TIMEOUT,
            QueuedOp::ConfigureCam(_) => self.camera_timeout,
            _ => self.op_timeout,
        };
        self.op_deadline = Some(now + timeout);

        match op {
            QueuedOp::Handshake => {
                let (seq, actions) = DspSequencer::new();
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Handshake {
                    phase: Box::new(HandshakePhase::Dsp(seq)),
                    dsp: None,
                    avr: None,
                });
            }
            QueuedOp::ConfigureAvr(avr_settings) => {
                if self.armed {
                    // Device is armed — disarm first, then re-queue configure.
                    self.queue.push_front(QueuedOp::ConfigureAvr(avr_settings));
                    self.start_disarm()?;
                } else {
                    let (seq, actions) = AvrConfigSequencer::new(avr_settings);
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.active = Some(ActiveOp::Configure(ConfigurePhase::Avr(seq)));
                }
            }
            QueuedOp::ConfigureXi(xi) => {
                if self.armed {
                    self.queue.push_front(QueuedOp::ConfigureXi(xi));
                    self.start_disarm()?;
                } else {
                    let (seq, actions) = if self.xi_connect_sent {
                        XiConfigSequencer::mode_change(&xi)
                    } else {
                        self.xi_connect_sent = true;
                        XiConfigSequencer::connect(&xi)
                    };
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.active = Some(ActiveOp::Configure(ConfigurePhase::Xi(seq)));
                }
            }
            QueuedOp::ConfigureCam(_) if !self.model().has_camera() => {
                self.op_deadline = None;
                self.ready_event = Some(BinaryEvent::Configured);
            }
            QueuedOp::ConfigureCam(cam_config) if self.armed => {
                self.queue.push_front(QueuedOp::ConfigureCam(cam_config));
                self.start_disarm()?;
            }
            QueuedOp::ConfigureCam(cam_config) => {
                let (cam_seq, actions) = CameraConfigSequencer::new(&cam_config);
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Configure(ConfigurePhase::Camera(cam_seq)));
            }
            QueuedOp::Arm => {
                let (seq, actions) = ArmSequencer::new(self.model());
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Arm(seq));
            }
            QueuedOp::Keepalive => {
                let (seq, actions) = KeepaliveSequencer::new(self.model());
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Keepalive(seq));
            }
        }
        Ok(())
    }

    /// Start an explicit disarm (the configure op that needs it has been
    /// re-queued at the front).
    fn start_disarm(&mut self) -> Result<(), ConnError> {
        let (seq, actions) = DisarmSequencer::new();
        for a in actions {
            seq::send_action(&mut self.conn, a)?;
        }
        self.active = Some(ActiveOp::Disarm(seq));
        Ok(())
    }

    // -- Internal: feed a message to the active operation -------------------

    /// The 0x94 that ended a step of the active operation, if any.
    fn take_refused(active: &mut ActiveOp) -> Option<Envelope> {
        match active {
            ActiveOp::Configure(ConfigurePhase::Xi(seq)) => seq.take_refused(),
            ActiveOp::Handshake { phase, .. } => match &mut **phase {
                HandshakePhase::Dsp(seq) => seq.take_refused(),
                _ => None,
            },
            _ => None,
        }
    }

    /// End the active camera configuration, which the device refused or did
    /// not finish, and report where it stopped. The next queued operation
    /// starts on the next poll.
    fn give_up_camera(&mut self) -> Option<BinaryEvent> {
        let Some(ActiveOp::Configure(ConfigurePhase::Camera(mut seq))) = self.active.take() else {
            return None;
        };
        self.op_deadline = None;
        Some(BinaryEvent::CameraSetupUnfinished {
            step: seq.step_name(),
            fusion: seq.is_fusion(),
            refused: seq.take_refused(),
        })
    }

    fn feed_active(
        active: &mut ActiveOp,
        env: &Envelope,
        now: Instant,
        conn: &mut BinaryConnection<S>,
    ) -> Result<FeedResult, ConnError> {
        match active {
            ActiveOp::Handshake { phase, .. } => {
                let actions = match &mut **phase {
                    HandshakePhase::Dsp(seq) => seq.feed(env, now),
                    HandshakePhase::Avr(seq) => seq.feed(env, now),
                    HandshakePhase::Pi(seq) => seq.feed(env, now),
                };
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                let complete = match &**phase {
                    HandshakePhase::Dsp(seq) => seq.is_complete(),
                    HandshakePhase::Avr(seq) => seq.is_complete(),
                    HandshakePhase::Pi(seq) => seq.is_complete(),
                };
                if complete {
                    Ok(FeedResult::PhaseComplete)
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Disarm(seq) => {
                let actions = seq.feed(env, now);
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                if seq.is_complete() {
                    Ok(FeedResult::Done)
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Configure(config_phase) => {
                let actions = match config_phase {
                    ConfigurePhase::Avr(seq) => seq.feed(env, now),
                    ConfigurePhase::Xi(seq) => seq.feed(env, now),
                    ConfigurePhase::Camera(seq) => seq.feed(env, now),
                };
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                let complete = match config_phase {
                    ConfigurePhase::Avr(seq) => seq.is_complete(),
                    ConfigurePhase::Xi(seq) => seq.is_complete(),
                    ConfigurePhase::Camera(seq) => seq.is_complete(),
                };
                if complete {
                    Ok(FeedResult::Done)
                } else if matches!(config_phase, ConfigurePhase::Camera(seq) if seq.is_refused()) {
                    Ok(FeedResult::CameraRefused)
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Arm(seq) => {
                let actions = seq.feed(env, now);
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                if seq.is_complete() {
                    Ok(FeedResult::Done)
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Shot(seq) => {
                let actions = seq.feed(env, now);
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                if seq.is_complete() {
                    Ok(FeedResult::Done)
                } else if let Some(datum) = seq.take_pending() {
                    Ok(FeedResult::Intermediate(Box::new(BinaryEvent::ShotDatum(
                        datum,
                    ))))
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Keepalive(seq) => {
                let actions = seq.feed(env, now);
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                if seq.is_complete() {
                    Ok(FeedResult::Done)
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
        }
    }

    /// Advance a multi-phase operation (handshake or configure) to its next
    /// phase, or emit the completion event if all phases are done.
    fn advance_phase(&mut self, now: Instant) -> Result<Option<BinaryEvent>, ConnError> {
        let active = self.active.take().expect("advance_phase with no active op");

        match active {
            ActiveOp::Handshake { phase, dsp, avr } => match *phase {
                HandshakePhase::Dsp(seq) => {
                    let dsp_result = seq.into_result();
                    let (avr_seq, actions) = AvrSequencer::new(dsp_result.model);
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.op_deadline = Some(now + self.op_timeout);
                    self.active = Some(ActiveOp::Handshake {
                        phase: Box::new(HandshakePhase::Avr(avr_seq)),
                        dsp: Some(dsp_result),
                        avr,
                    });
                    Ok(None)
                }
                HandshakePhase::Avr(seq) => {
                    let avr_result = seq.into_result();
                    let dsp = dsp.expect("DspSync missing after DSP phase");
                    if !dsp.model.has_camera() {
                        // No camera Pi on the bus: the handshake ends here.
                        let outcome = HandshakeOutcome {
                            model: dsp.model,
                            dsp,
                            avr: avr_result,
                            pi: None,
                        };
                        return Ok(Some(self.complete_handshake(outcome)));
                    }
                    let dsp = Some(dsp);
                    let (pi_seq, actions) = PiSequencer::new();
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.op_deadline = Some(now + self.op_timeout);
                    self.active = Some(ActiveOp::Handshake {
                        phase: Box::new(HandshakePhase::Pi(pi_seq)),
                        dsp,
                        avr: Some(avr_result),
                    });
                    Ok(None)
                }
                HandshakePhase::Pi(seq) => {
                    let pi_result = seq.into_result();
                    let dsp = dsp.expect("DspSync missing after handshake");
                    let outcome = HandshakeOutcome {
                        model: dsp.model,
                        dsp,
                        avr: avr.expect("AvrSync missing after handshake"),
                        pi: Some(pi_result),
                    };
                    Ok(Some(self.complete_handshake(outcome)))
                }
            },
            // Only Handshake is multi-phase now.
            _ => unreachable!("advance_phase called on single-phase op"),
        }
    }

    /// Record a finished handshake and build its event.
    fn complete_handshake(&mut self, outcome: HandshakeOutcome) -> BinaryEvent {
        // Detect if a Mevo was armed from a prior session.
        // DspStatus state 6 = armed. This ensures the next
        // configure_avr will auto-disarm. The Xi's DSP status has a
        // different layout; its configuration sends B0[01 00] before the
        // mode block, which disarms it.
        if outcome.model == DeviceModel::Mevo && outcome.dsp.status.state() == 6 {
            self.armed = true;
        }
        self.device = Some(outcome.clone());
        self.active = None;
        self.op_deadline = None;
        BinaryEvent::Handshake(outcome)
    }

    /// Finish a single-phase operation and emit its completion event.
    fn finish_op(&mut self, now: Instant) -> Result<Option<BinaryEvent>, ConnError> {
        let active = self.active.take().expect("finish_op with no active op");
        self.op_deadline = None;

        match active {
            ActiveOp::Disarm(_) => {
                self.armed = false;
                self.keepalive_enabled = false;
                Ok(Some(BinaryEvent::Disarmed))
            }
            ActiveOp::Configure(_) => Ok(Some(BinaryEvent::Configured)),
            ActiveOp::Arm(_) => {
                self.armed = true;
                self.keepalive_enabled = true;
                self.last_keepalive = Some(now);
                Ok(Some(BinaryEvent::Armed))
            }
            ActiveOp::Shot(seq) => {
                self.armed = true;
                self.shot_in_progress = false;
                self.trigger_time = None;
                self.last_keepalive = Some(now);
                Ok(Some(BinaryEvent::ShotComplete(Box::new(seq.into_result()))))
            }
            ActiveOp::Keepalive(_) => {
                self.keepalive_queued = false;
                self.last_keepalive = Some(now);
                let snapshot = self.status.clone();
                // Start next queued op immediately if available.
                if let Some(queued) = self.queue.pop_front() {
                    self.start_op(queued, now)?;
                }
                Ok(Some(BinaryEvent::Keepalive(snapshot)))
            }
            // Only Handshake is multi-phase (uses advance_phase).
            _ => unreachable!("finish_op called on multi-phase op"),
        }
    }
}

/// Internal signal from feed_active to the poll loop.
enum FeedResult {
    /// Message consumed, nothing to emit.
    Consumed,
    /// Emit this event but keep the active op running.
    Intermediate(Box<BinaryEvent>),
    /// A phase within a multi-phase op finished (advance to next phase).
    PhaseComplete,
    /// The entire operation finished.
    Done,
    /// The device refused a step of the camera configuration.
    CameraRefused,
}

// -- TcpStream convenience --------------------------------------------------

impl BinaryClient<TcpStream> {
    /// Create a client from a TCP connection, setting the stream to
    /// non-blocking mode for use with [`poll()`](Self::poll).
    pub fn from_tcp(conn: BinaryConnection<TcpStream>) -> Result<Self, ConnError> {
        // Best-effort: already set by connect()/connect_timeout(), but the
        // caller may have constructed the connection manually. Failure is
        // harmless (just higher latency on small writes).
        let _ = conn.stream().set_nodelay(true);
        conn.stream().set_nonblocking(true)?;
        Ok(Self::new(conn))
    }
}

#[cfg(test)]
mod tests;
