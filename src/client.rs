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
            seq::keepalive_actions_for(model),
        )
    }
}

impl Sequence for KeepaliveSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
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
    last_keepalive: Instant,
    keepalive_interval: Duration,

    // Configuration
    op_timeout: Duration,

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
            last_keepalive: Instant::now(),
            keepalive_interval: DEFAULT_KEEPALIVE_INTERVAL,
            op_timeout: DEFAULT_OP_TIMEOUT,
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
    /// soon as it is received (see [`seq::result_ack`]).
    ///
    /// # Errors
    ///
    /// Returns `Err` on I/O errors, wire decode errors, disconnection,
    /// or operation timeout (except keepalive timeouts, which are non-fatal).
    pub fn poll(&mut self) -> Result<Option<BinaryEvent>, ConnError> {
        // 0. Event produced by an operation that sent nothing.
        if let Some(event) = self.ready_event.take() {
            return Ok(Some(event));
        }

        // 1. Check operation deadline.
        if let Some(deadline) = self.op_deadline
            && Instant::now() >= deadline
        {
            if matches!(self.active, Some(ActiveOp::Keepalive(_))) {
                // Keepalive timeout is non-fatal. Update last_keepalive
                // so we don't immediately re-queue another one.
                self.active = None;
                self.op_deadline = None;
                self.keepalive_queued = false;
                self.last_keepalive = Instant::now();
            } else {
                return Err(ConnError::Timeout);
            }
        }

        // 2. Start next queued operation if idle.
        if self.active.is_none()
            && let Some(queued) = self.queue.pop_front()
        {
            self.start_op(queued)?;
            if let Some(event) = self.ready_event.take() {
                return Ok(Some(event));
            }
        }

        // 3. Handshake timeout checks: optional DSP query and AVR cal steps.
        if let Some(ActiveOp::Handshake { ref mut phase, .. }) = self.active {
            match **phase {
                HandshakePhase::Dsp(ref mut dsp) => {
                    for a in dsp.check_timeout() {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    // On the Xi a timed-out power-mode write is the last
                    // DSP step.
                    if dsp.is_complete() {
                        return self.advance_phase();
                    }
                }
                HandshakePhase::Avr(ref mut avr) => {
                    for a in avr.check_cal_timeout() {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    // On the Xi a timed-out IF cal is the last AVR step.
                    if avr.is_complete() {
                        return self.advance_phase();
                    }
                }
                HandshakePhase::Pi(_) => {}
            }
        }

        // 3b. Shot drain timeout — if IDLE never arrives after shot data,
        //     skip ShotResultReq and proceed directly to ARM.
        if let Some(ActiveOp::Shot(ref mut seq)) = self.active {
            let timeout_actions = seq.check_drain_timeout();
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
                .is_some_and(|t| t.elapsed() >= TRIGGER_TIMEOUT)
        {
            return Err(ConnError::Timeout);
        }

        // 4. Auto-queue keepalive (suppress while shot is in progress —
        //    BALL TRIGGER has fired but PROCESSED hasn't arrived yet).
        if self.keepalive_enabled
            && !self.keepalive_queued
            && !self.shot_in_progress
            && self.last_keepalive.elapsed() >= self.keepalive_interval
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
        //    auto-detection (step 9). This mirrors FS Golf's native
        //    getResponse() which only consumes messages matching the
        //    expected response type. Without this, unsolicited shot
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
                let result = Self::feed_active(active, &env, &mut self.conn)?;
                let refused = Self::take_refused(active).map(BinaryEvent::StepRefused);
                let event = match result {
                    FeedResult::Consumed => None,
                    FeedResult::Intermediate(event) => Some(*event),
                    FeedResult::PhaseComplete => self.advance_phase()?,
                    FeedResult::Done => self.finish_op()?,
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
                self.trigger_time = Some(Instant::now());
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
                self.op_deadline = Some(Instant::now() + self.op_timeout);
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

    /// Enqueue the Xi's configuration, sent the way FS Golf sends it
    /// (see [`XiConfigSequencer`]).
    ///
    /// The first Xi configuration on a connection sends the connect-time
    /// block (parameter writes, radar calibration) followed by the mode
    /// block; later ones send the mode block only, for a mode change. If
    /// the device is armed when this operation starts executing, it is
    /// disarmed first. Emits [`BinaryEvent::Disarmed`] (when it disarmed)
    /// then [`BinaryEvent::Configured`]. Arm afterwards with
    /// [`arm()`](Self::arm).
    pub fn configure_xi(&mut self, xi: XiSettings) {
        self.queue.push_back(QueuedOp::ConfigureXi(xi));
    }

    /// Enqueue camera configuration.
    ///
    /// Emits [`BinaryEvent::Configured`] on completion. On a model without a
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
    /// (which always use a 3s timeout).
    pub fn set_operation_timeout(&mut self, timeout: Duration) {
        self.op_timeout = timeout;
    }

    // -- Internal: start an operation ---------------------------------------

    fn start_op(&mut self, op: QueuedOp) -> Result<(), ConnError> {
        let timeout = match &op {
            QueuedOp::Keepalive => KEEPALIVE_TIMEOUT,
            _ => self.op_timeout,
        };
        self.op_deadline = Some(Instant::now() + timeout);

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
            QueuedOp::ConfigureCam(cam_config) => {
                let (cam_seq, actions) = CameraConfigSequencer::new(&cam_config);
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Configure(ConfigurePhase::Camera(cam_seq)));
            }
            QueuedOp::Arm => {
                let (seq, actions) = ArmSequencer::new_for(self.model());
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

    fn feed_active(
        active: &mut ActiveOp,
        env: &Envelope,
        conn: &mut BinaryConnection<S>,
    ) -> Result<FeedResult, ConnError> {
        match active {
            ActiveOp::Handshake { phase, .. } => {
                let actions = match &mut **phase {
                    HandshakePhase::Dsp(seq) => seq.feed(env),
                    HandshakePhase::Avr(seq) => seq.feed(env),
                    HandshakePhase::Pi(seq) => seq.feed(env),
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
                let actions = seq.feed(env);
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
                    ConfigurePhase::Avr(seq) => seq.feed(env),
                    ConfigurePhase::Xi(seq) => seq.feed(env),
                    ConfigurePhase::Camera(seq) => seq.feed(env),
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
                } else {
                    Ok(FeedResult::Consumed)
                }
            }
            ActiveOp::Arm(seq) => {
                let actions = seq.feed(env);
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
                let actions = seq.feed(env);
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
                let actions = seq.feed(env);
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
    fn advance_phase(&mut self) -> Result<Option<BinaryEvent>, ConnError> {
        let active = self.active.take().expect("advance_phase with no active op");

        match active {
            ActiveOp::Handshake { phase, dsp, avr } => match *phase {
                HandshakePhase::Dsp(seq) => {
                    let dsp_result = seq.into_result();
                    let (avr_seq, actions) = AvrSequencer::new_for(dsp_result.model);
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.op_deadline = Some(Instant::now() + self.op_timeout);
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
                    self.op_deadline = Some(Instant::now() + self.op_timeout);
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
    fn finish_op(&mut self) -> Result<Option<BinaryEvent>, ConnError> {
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
                self.last_keepalive = Instant::now();
                Ok(Some(BinaryEvent::Armed))
            }
            ActiveOp::Shot(seq) => {
                self.armed = true;
                self.shot_in_progress = false;
                self.trigger_time = None;
                self.last_keepalive = Instant::now();
                Ok(Some(BinaryEvent::ShotComplete(Box::new(seq.into_result()))))
            }
            ActiveOp::Keepalive(_) => {
                self.keepalive_queued = false;
                self.last_keepalive = Instant::now();
                let snapshot = self.status.clone();
                // Start next queued op immediately if available.
                if let Some(queued) = self.queue.pop_front() {
                    self.start_op(queued)?;
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
mod tests {
    //! Drive `BinaryClient` against a scripted Xi on an in-memory stream.
    //!
    //! Device identity frames (status, device info) are synthesised with a
    //! made-up serial. Per-shot frames follow the Xi's per-shot format and
    //! carry no device identity.

    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::io;
    use std::rc::Rc;

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

    /// Made-up device serial used in every synthesised identity frame.
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
        let ed = hex(
            "a62f002040000adb0007db000474fffa3f00028b000f220004f80002240008b50015380003f2fffed00002\
             1f0000040028460000000000030000020009ea000000045a3bfffcf203ec210e3ea8fffcf203ebe9e12e7a\
             000030ff23313d086c000030ff233a2a9ca70015de00f05d3531680015df00f041146dc2ffff02fffa6f00\
             1f47ffff02fffa6f001f470016bb000dd80019820016bb000dd8001982fff3a0001080000102",
        );
        let frames = vec![
            (0xE5, hex("1442414c4c20545249474745523a203633206d7300")),
            (
                0xE8,
                hex(
                    "5d000004000000008a28000ff5014c9f0034ef002f1b005209001fcd000000000000001bf50000\
                     0000005600000001e00f0000003d08e0f5d3a3019570ffe4ea00000017ccdefb5b2eff6929001\
                     2cb00000008bc60fe8a0e003a8afffc10",
                ),
            ),
            (
                0xD4,
                hex(
                    "9c00000400005600000000002700020a008a28001fcd005209014ca2000ff30034e30149f80000\
                     00002a04001bd7fffd740000000003e9fffefbffe76d003c1affce900006f700000000000000000\
                     0000000000000006c96002c8fffc66e004e9500d43600571500975a00197701df710000003d08eb\
                     f5cc2b019bbcffe3fd00000017c8affb5496ff6c7f00126500000008b809fe35a1004bd6fffab0",
                ),
            ),
            (0xED, ed.clone()),
            (
                0xEF,
                hex(
                    "8930040003520008266a000251000643e00001a500052eb000016c0005216300016600054b0800\
                     0164000521e900028800072c0f000264000732e00001b600062ed500027900093329000223000\
                     821cb000220000800000000000000000000000000000000000000000011111bf500741bf50000\
                     000000000000030000051bf503691111000000000000",
                ),
            ),
            (0xE5, hex("0a50524f43455353454400")),
            (0xE5, hex("0549444c4500")),
            (0xB1, vec![0x02, 0x00, 0x00]),
        ];
        (frames, ed)
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
        /// Refuse the DSP query instead of leaving it unanswered.
        nack_dsp_query: bool,
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
        /// PI is answered, and neither is the DSP query (0x48), so the DSP
        /// query timeout path is exercised.
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
                (BusAddr::Dsp, 0x48) if self.nack_dsp_query => {
                    self.push(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
                }
                (BusAddr::Dsp, 0xBF) => {
                    self.push(BusAddr::Dsp, 0x95, vec![0x02, 0x40, 0x3F]);
                    if p[3] == 0x01 && p[6] != self.dsp_power_mode {
                        let req =
                            format!("  REQ Power Mode : {} ({}) \0", p[6], self.dsp_power_mode);
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

    /// Poll until `pred` matches an event, collecting every event seen.
    fn poll_until(
        client: &mut BinaryClient<XiStream>,
        events: &mut Vec<BinaryEvent>,
        pred: impl Fn(&BinaryEvent) -> bool,
    ) {
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            match client.poll().expect("poll failed") {
                Some(event) => {
                    let done = pred(&event);
                    events.push(event);
                    if done {
                        return;
                    }
                }
                None => std::thread::sleep(Duration::from_millis(1)),
            }
        }
        panic!("event not seen; got {events:?}");
    }

    #[test]
    fn xi_handshake_configure_arm_and_shot() {
        let state = Rc::new(RefCell::new(XiState::default()));
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
        // FS Golf's configuration: 19 param writes, param 0x02 read
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
        let state = Rc::new(RefCell::new(XiState {
            nack_dsp_query: true,
            ..XiState::default()
        }));
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
            let state = Rc::new(RefCell::new(XiState {
                nack_dsp_query: true,
                ..XiState::default()
            }));
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
                nack_dsp_query: true,
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
        let state = Rc::new(RefCell::new(XiState {
            nack_dsp_query: true,
            ..XiState::default()
        }));
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
            nack_dsp_query: true,
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
            nack_dsp_query: true,
            ignore: vec![(BusAddr::Dsp, 0xBF, hex("06000001000001"))],
            ..XiState::default()
        }));
        let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
        let mut events = Vec::new();
        client.handshake();
        client.configure_xi(XiSettings::default());
        client.arm();
        let t0 = Instant::now();
        poll_until(&mut client, &mut events, |e| {
            matches!(e, BinaryEvent::Armed)
        });
        assert!(t0.elapsed() >= seq::DSP_QUERY_TIMEOUT);
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
            nack_dsp_query: true,
            avr_boot_nacks: 2,
            ..XiState::default()
        }));
        let mut client = BinaryClient::new(BinaryConnection::new(XiStream(state.clone())));
        let mut events = Vec::new();
        client.handshake();
        let t0 = Instant::now();
        poll_until(&mut client, &mut events, |e| {
            matches!(e, BinaryEvent::Handshake(_))
        });
        let elapsed = t0.elapsed();
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
                match client.poll().unwrap() {
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
    /// the handshake outcome relabelled, so only the model differs.
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
                let deadline = Instant::now() + Duration::from_secs(10);
                while !matches!(events.last(), Some(BinaryEvent::ShotComplete(_))) {
                    assert!(Instant::now() < deadline, "no ShotComplete: {events:?}");
                    if state.borrow().outbox.is_empty()
                        && let Some(f) = pending.pop_front()
                    {
                        state.borrow_mut().outbox.extend(f);
                    }
                    if let Some(e) = client.poll().unwrap() {
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
        let idle_at = Instant::now() + Duration::from_millis(3500);
        while Instant::now() < idle_at {
            if let Some(e) = client.poll().unwrap() {
                events.push(e);
            } else {
                std::thread::sleep(Duration::from_millis(5));
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
}
