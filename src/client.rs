//! Non-blocking client for the Mevo+ binary protocol (port 5100).
//!
//! Wraps [`BinaryConnection`] and the sequencer state machines from [`seq`]
//! into a single `BinaryClient` with a `poll() -> Option<BinaryEvent>` entry
//! point. Callers enqueue operations (handshake, configure, arm) and the
//! client drives them one at a time, emitting [`BinaryEvent`]s as milestones
//! complete.
//!
//! Keepalives are managed automatically after the first arm — queued as
//! proper operations so they never interleave with other sequencers.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use crate::addr::BusAddr;
use crate::conn::{BinaryConnection, ConnError, Envelope};
use crate::protocol::camera::CamConfig;
use crate::protocol::status::{AvrStatus, DspStatus, PiStatus};
use crate::protocol::Message;
use crate::seq::{
    self, Action, ArmSequencer, AvrConfigSequencer, AvrSequencer, AvrSettings, AvrSync,
    CameraConfigSequencer, DisarmSequencer, DspSequencer, DspSync, PiSequencer, PiSync, Sequence,
    ShotData, ShotDatum, ShotSequencer,
};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Events emitted by [`BinaryClient::poll()`].
#[derive(Debug)]
pub enum BinaryEvent {
    /// Three-phase handshake (DSP + AVR + PI sync) complete.
    Handshake(HandshakeOutcome),
    /// Device disarmed (explicit disarm before re-configure).
    Disarmed,
    /// AVR config + camera config applied.
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
    /// Any message not consumed by the active operation.
    Message(Envelope),
}

/// Combined results from the three-phase handshake.
#[derive(Debug, Clone)]
pub struct HandshakeOutcome {
    pub dsp: DspSync,
    pub avr: AvrSync,
    pub pi: PiSync,
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

/// Single-phase configure operations (AVR or camera, independently).
enum ConfigurePhase {
    Avr(AvrConfigSequencer),
    Camera(CameraConfigSequencer),
}

// ---------------------------------------------------------------------------
// KeepaliveSequencer
// ---------------------------------------------------------------------------

/// Sends 3 StatusPolls (DSP + AVR + PI) and collects all 3 responses.
struct KeepaliveSequencer {
    got_dsp: bool,
    got_avr: bool,
    got_pi: bool,
}

impl KeepaliveSequencer {
    fn new() -> (Self, Vec<Action>) {
        (
            Self {
                got_dsp: false,
                got_avr: false,
                got_pi: false,
            },
            seq::keepalive_actions(),
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

// ---------------------------------------------------------------------------
// BinaryClient
// ---------------------------------------------------------------------------

/// Non-blocking client for the Mevo+ binary protocol (port 5100).
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
///             println!("Connected to {}", h.pi.ssid);
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
        }
    }

    /// Poll the client for the next event.
    ///
    /// Drives the active operation (if any), processes incoming messages,
    /// and manages keepalive scheduling. Returns `Ok(None)` when no data
    /// is available (non-blocking).
    ///
    /// # Errors
    ///
    /// Returns `Err` on I/O errors, wire decode errors, disconnection,
    /// or operation timeout (except keepalive timeouts, which are non-fatal).
    pub fn poll(&mut self) -> Result<Option<BinaryEvent>, ConnError> {
        // 1. Check operation deadline.
        if let Some(deadline) = self.op_deadline {
            if Instant::now() >= deadline {
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
        }

        // 2. Start next queued operation if idle.
        if self.active.is_none() {
            if let Some(queued) = self.queue.pop_front() {
                self.start_op(queued)?;
            }
        }

        // 3. AvrSequencer cal timeout check.
        if let Some(ActiveOp::Handshake { ref mut phase, .. }) = self.active {
            if let HandshakePhase::Avr(ref mut avr) = **phase {
                let timeout_actions = avr.check_cal_timeout();
                for a in timeout_actions {
                    seq::send_action(&mut self.conn, a)?;
                }
            }
        }

        // 4. Auto-queue keepalive.
        if self.keepalive_enabled
            && !self.keepalive_queued
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

        // 6. Update status cache.
        match (&env.message, env.src) {
            (Message::DspStatus(s), BusAddr::Dsp) => self.status.dsp = Some(s.clone()),
            (Message::AvrStatus(s), BusAddr::Avr) => self.status.avr = Some(s.clone()),
            (Message::PiStatus(s), BusAddr::Pi) => self.status.pi = Some(s.clone()),
            _ => {}
        }

        // 7. Feed active operation.
        if let Some(ref mut active) = self.active {
            match Self::feed_active(active, &env, &mut self.conn)? {
                FeedResult::Consumed => return Ok(None),
                FeedResult::Intermediate(event) => return Ok(Some(*event)),
                FeedResult::PhaseComplete => return self.advance_phase(),
                FeedResult::Done => return self.finish_op(),
            }
        }

        // 8. No active op — shot auto-detection.
        if let Message::ShotText(ref st) = env.message {
            if st.is_trigger() {
                self.shot_in_progress = true;
                self.armed = false;
                return Ok(Some(BinaryEvent::Trigger));
            }
            if st.is_processed() {
                let (seq, actions) = ShotSequencer::new();
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Shot(Box::new(seq)));
                self.op_deadline = Some(Instant::now() + self.op_timeout);
                return Ok(None);
            }
        }

        // 9. Pre-PROCESSED shot data (E8 arrives between TRIGGER and PROCESSED).
        if self.shot_in_progress {
            if let Message::FlightResultV1(ref e8) = env.message {
                return Ok(Some(BinaryEvent::ShotDatum(ShotDatum::FlightV1(
                    e8.clone(),
                ))));
            }
        }

        // 10. Unhandled message passthrough.
        Ok(Some(BinaryEvent::Message(env)))
    }

    // -- Public operation enqueuers -----------------------------------------

    /// Enqueue a three-phase handshake (DSP + AVR + PI sync).
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

    /// Enqueue camera configuration.
    ///
    /// Emits [`BinaryEvent::Configured`] on completion.
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
                    let (seq, actions) = DisarmSequencer::new();
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.active = Some(ActiveOp::Disarm(seq));
                } else {
                    let (seq, actions) = AvrConfigSequencer::new(avr_settings);
                    for a in actions {
                        seq::send_action(&mut self.conn, a)?;
                    }
                    self.active = Some(ActiveOp::Configure(ConfigurePhase::Avr(seq)));
                }
            }
            QueuedOp::ConfigureCam(cam_config) => {
                let (cam_seq, actions) = CameraConfigSequencer::new(&cam_config);
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Configure(ConfigurePhase::Camera(cam_seq)));
            }
            QueuedOp::Arm => {
                let (seq, actions) = ArmSequencer::new();
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Arm(seq));
            }
            QueuedOp::Keepalive => {
                let (seq, actions) = KeepaliveSequencer::new();
                for a in actions {
                    seq::send_action(&mut self.conn, a)?;
                }
                self.active = Some(ActiveOp::Keepalive(seq));
            }
        }
        Ok(())
    }

    // -- Internal: feed a message to the active operation -------------------

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
                    ConfigurePhase::Camera(seq) => seq.feed(env),
                };
                for a in actions {
                    seq::send_action(conn, a)?;
                }
                let complete = match config_phase {
                    ConfigurePhase::Avr(seq) => seq.is_complete(),
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
                    Ok(FeedResult::Intermediate(Box::new(BinaryEvent::ShotDatum(datum))))
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
                    let (avr_seq, actions) = AvrSequencer::new();
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
                    let outcome = HandshakeOutcome {
                        dsp: dsp.expect("DspSync missing after handshake"),
                        avr: avr.expect("AvrSync missing after handshake"),
                        pi: pi_result,
                    };
                    // Detect if device was armed from a prior session.
                    // DspStatus state 6 = armed. This ensures the next
                    // configure_avr will auto-disarm.
                    if outcome.dsp.status.state() == 6 {
                        self.armed = true;
                    }
                    self.device = Some(outcome.clone());
                    self.active = None;
                    self.op_deadline = None;
                    Ok(Some(BinaryEvent::Handshake(outcome)))
                }
            },
            // Only Handshake is multi-phase now.
            _ => unreachable!("advance_phase called on single-phase op"),
        }
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

