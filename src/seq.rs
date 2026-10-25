//! Handshake sequences, operational routines, and pollable state machines.
//!
//! Two layers:
//!
//! 1. **Sequencers** — pollable state machines ([`Sequence`] trait). Each
//!    sequencer's `feed()` accepts a received [`Envelope`] and returns
//!    [`Action`]s to send. Callers own the event loop.
//!
//! 2. **Blocking functions** — convenience wrappers that call [`drive()`] to
//!    run a sequencer to completion on a blocking [`Connection`] (TcpStream).
//!    These preserve the pre-v0.1 API for callers that don't need non-blocking.
//!
//! All protocol logic lives in the sequencers. The blocking functions are thin
//! wrappers.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::addr::BusAddr;
use crate::conn::{BinaryConnection, ConnError, Connection, Envelope};
use crate::protocol::camera::{CamConfig, CamConfigReq, CamState};
use crate::protocol::config::{
    AvrConfigCmd, AvrConfigResp, ConfigResp, ModeSet, ParamReadReq, ParamValue, RadarCal,
};
use crate::protocol::handshake::{
    CalDataReq, CalDataResp, CalParamReq, CalParamResp, DevInfoResp, DeviceModel, DspQueryResp,
    NetConfigReq, ProdInfoReq, ProdInfoResp, TimeSync,
};
use crate::protocol::shot::{
    ClubPrc, ClubResult, FlightResult, FlightResultV1, PrcData, SpeedProfile, SpinResult,
};
use crate::protocol::status::{AvrStatus, DspStatus, PiStatus, StatusPoll};
use crate::protocol::{Command, Message, TYPE_DSP_QUERY};

/// Per-exchange timeout (2s for blocking wrappers).
pub const TIMEOUT: Duration = Duration::from_secs(2);

/// How long the DSP sequencer waits for the DSP query response (0xC8) before
/// carrying on without it. A response from the Xi's DSP has not been
/// observed, so the handshake does not depend on one.
pub const DSP_QUERY_TIMEOUT: Duration = TIMEOUT;

// ===========================================================================
// Action, Sequence trait, drive(), infrastructure
// ===========================================================================

/// An action returned by a sequencer's `feed()` or `new()`.
#[derive(Debug, Clone)]
pub enum Action {
    /// Send a command to a bus address.
    Send(Command, BusAddr),
}

/// Trait implemented by all sequencers.
///
/// `feed()` accepts a received message and returns actions to send.
/// `is_complete()` indicates the sequence is done. Type-specific `finish()`
/// methods (not in the trait) extract the result.
pub trait Sequence {
    /// Process a received message. Returns actions to send (may be empty).
    fn feed(&mut self, env: &Envelope) -> Vec<Action>;

    /// Is the sequence complete?
    fn is_complete(&self) -> bool;
}

/// Send an action on a connection.
pub fn send_action<S: Read + Write>(
    conn: &mut BinaryConnection<S>,
    action: Action,
) -> Result<(), ConnError> {
    match action {
        Action::Send(cmd, dest) => conn.send(&cmd, dest),
    }
}

/// Run a sequence to completion on a blocking stream.
///
/// Sends initial actions, then loops recv→feed until complete.
/// The stream must have a read timeout set to pace the loop. The
/// `deadline` parameter controls the overall timeout for the sequence.
pub fn drive<S: Read + Write>(
    conn: &mut BinaryConnection<S>,
    seq: &mut impl Sequence,
    actions: Vec<Action>,
    deadline: Instant,
) -> Result<(), ConnError> {
    for a in actions {
        send_action(conn, a)?;
    }
    loop {
        if Instant::now() >= deadline {
            return Err(ConnError::Timeout);
        }
        if let Some(env) = conn.recv()? {
            let actions = seq.feed(&env);
            for a in actions {
                send_action(conn, a)?;
            }
            if seq.is_complete() {
                return Ok(());
            }
        }
    }
}

/// Commands for a keepalive poll on a Mevo (DSP + AVR + PI).
/// Fire-and-forget — responses arrive via recv().
#[must_use]
pub fn keepalive_actions() -> Vec<Action> {
    keepalive_actions_for(DeviceModel::Mevo)
}

/// Commands for a keepalive poll on the given model. The PI is polled only
/// on models that have one.
#[must_use]
pub fn keepalive_actions_for(model: DeviceModel) -> Vec<Action> {
    let mut actions = vec![
        Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Dsp,
        ),
        Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Avr,
        ),
    ];
    if model.has_camera() {
        actions.push(Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: true }),
            BusAddr::Pi,
        ));
    }
    actions
}

// ===========================================================================
// Message filtering helpers
// ===========================================================================

/// Messages that all sequencers skip unconditionally.
fn should_skip(env: &Envelope, expected_src: BusAddr) -> bool {
    env.src != expected_src
        || matches!(
            &env.message,
            Message::Text(_)
                | Message::DspDebug(_)
                | Message::CamState(_)
                | Message::ConfigNack(_)
                | Message::Unknown { .. }
        )
}

/// Like `should_skip` but also skips ModeAck (for send_recv compatibility).
fn should_skip_with_mode_ack(env: &Envelope, expected_src: BusAddr) -> bool {
    should_skip(env, expected_src) || matches!(&env.message, Message::ModeAck(_))
}

/// Skip messages that arrive before a CalData/CalParam response:
/// Text, DspDebug, ConfigAck, ConfigNack, Unknown, and wrong-bus.
fn should_skip_for_cal(env: &Envelope, expected_src: BusAddr) -> bool {
    env.src != expected_src
        || matches!(
            &env.message,
            Message::Text(_)
                | Message::DspDebug(_)
                | Message::ConfigAck(_)
                | Message::ConfigNack(_)
                | Message::Unknown { .. }
        )
}

// ===========================================================================
// Phase 1 — DspSequencer
// ===========================================================================

/// Results from Phase 1 — DSP sync.
#[derive(Debug, Clone)]
pub struct DspSync {
    pub status: DspStatus,
    /// DSP hardware info (0xC8). `None` when the DSP query is rejected or
    /// goes unanswered within [`DSP_QUERY_TIMEOUT`].
    pub hw_info: Option<DspQueryResp>,
    pub dev_info: DevInfoResp,
    pub prod_info: [ProdInfoResp; 3],
    pub config: ConfigResp,
    /// Model family, detected from the DSP device info.
    pub model: DeviceModel,
}

#[derive(Debug)]
enum DspStep {
    WaitStatus,
    WaitDspQuery,
    WaitDevInfo,
    WaitProdInfo0,
    WaitProdInfo1,
    WaitProdInfo2,
    WaitConfig,
    Done,
}

/// Pollable state machine for DSP sync (Phase 1).
///
/// Sequence: StatusPoll → DspQuery (0x48) → DevInfoReq → ProdInfoReq ×3 →
/// ConfigQuery. The DSP query is optional: a ConfigNack for it, or no answer
/// within [`DSP_QUERY_TIMEOUT`] (see [`check_timeout()`](Self::check_timeout)),
/// moves on to DevInfoReq with no hardware info.
pub struct DspSequencer {
    step: DspStep,
    status: Option<DspStatus>,
    hw_info: Option<DspQueryResp>,
    dev_info: Option<DevInfoResp>,
    prod_info: Vec<ProdInfoResp>,
    config: Option<ConfigResp>,
    /// Deadline for the DSP query response.
    query_deadline: Option<Instant>,
}

impl DspSequencer {
    /// Create a new DSP sequencer with initial actions to send.
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        let seq = Self {
            step: DspStep::WaitStatus,
            status: None,
            hw_info: None,
            dev_info: None,
            prod_info: Vec::with_capacity(3),
            config: None,
            query_deadline: None,
        };
        let actions = vec![Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Dsp,
        )];
        (seq, actions)
    }

    /// Extract the result. Only valid after `is_complete()`.
    #[must_use]
    pub fn into_result(self) -> DspSync {
        let mut pi = self.prod_info;
        let dev_info = self.dev_info.unwrap();
        DspSync {
            status: self.status.unwrap(),
            hw_info: self.hw_info,
            model: dev_info.model(),
            dev_info,
            prod_info: [pi.remove(0), pi.remove(0), pi.remove(0)],
            config: self.config.unwrap(),
        }
    }

    /// Check whether the DSP query has gone unanswered for
    /// [`DSP_QUERY_TIMEOUT`]; if so, move on to DevInfoReq.
    /// Call this periodically in the event loop.
    pub fn check_timeout(&mut self) -> Vec<Action> {
        if let Some(deadline) = self.query_deadline
            && Instant::now() >= deadline
        {
            return self.skip_dsp_query();
        }
        vec![]
    }

    /// Give up on the DSP query response and request device info.
    fn skip_dsp_query(&mut self) -> Vec<Action> {
        if !matches!(self.step, DspStep::WaitDspQuery) {
            return vec![];
        }
        self.query_deadline = None;
        self.step = DspStep::WaitDevInfo;
        vec![Action::Send(Command::DevInfoReq, BusAddr::Dsp)]
    }
}

impl Sequence for DspSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        if matches!(self.step, DspStep::WaitDspQuery)
            && env.src == BusAddr::Dsp
            && let Message::ConfigNack(ref nack) = env.message
            && nack.acked_cmd == TYPE_DSP_QUERY & 0x7F
        {
            return self.skip_dsp_query();
        }
        if should_skip_with_mode_ack(env, BusAddr::Dsp) {
            return vec![];
        }
        match self.step {
            DspStep::WaitStatus => {
                if let Message::DspStatus(ref s) = env.message {
                    self.status = Some(s.clone());
                    self.step = DspStep::WaitDspQuery;
                    self.query_deadline = Some(Instant::now() + DSP_QUERY_TIMEOUT);
                    return vec![Action::Send(Command::DspQuery, BusAddr::Dsp)];
                }
                vec![]
            }
            DspStep::WaitDspQuery => {
                if let Message::DspQueryResp(ref r) = env.message {
                    self.hw_info = Some(r.clone());
                    self.query_deadline = None;
                    self.step = DspStep::WaitDevInfo;
                    return vec![Action::Send(Command::DevInfoReq, BusAddr::Dsp)];
                }
                vec![]
            }
            DspStep::WaitDevInfo => {
                if let Message::DevInfoResp(ref r) = env.message {
                    self.dev_info = Some(r.clone());
                    self.step = DspStep::WaitProdInfo0;
                    return vec![Action::Send(
                        Command::ProdInfoReq(ProdInfoReq { sub_query: 0x00 }),
                        BusAddr::Dsp,
                    )];
                }
                vec![]
            }
            DspStep::WaitProdInfo0 => {
                if let Message::ProdInfoResp(ref r) = env.message {
                    self.prod_info.push(r.clone());
                    self.step = DspStep::WaitProdInfo1;
                    return vec![Action::Send(
                        Command::ProdInfoReq(ProdInfoReq { sub_query: 0x08 }),
                        BusAddr::Dsp,
                    )];
                }
                vec![]
            }
            DspStep::WaitProdInfo1 => {
                if let Message::ProdInfoResp(ref r) = env.message {
                    self.prod_info.push(r.clone());
                    self.step = DspStep::WaitProdInfo2;
                    return vec![Action::Send(
                        Command::ProdInfoReq(ProdInfoReq { sub_query: 0x09 }),
                        BusAddr::Dsp,
                    )];
                }
                vec![]
            }
            DspStep::WaitProdInfo2 => {
                if let Message::ProdInfoResp(ref r) = env.message {
                    self.prod_info.push(r.clone());
                    self.step = DspStep::WaitConfig;
                    return vec![Action::Send(Command::ConfigQuery, BusAddr::Dsp)];
                }
                vec![]
            }
            DspStep::WaitConfig => {
                if let Message::ConfigResp(ref r) = env.message {
                    self.config = Some(r.clone());
                    self.step = DspStep::Done;
                }
                vec![]
            }
            DspStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, DspStep::Done)
    }
}

// ===========================================================================
// Phase 2 — AvrSequencer
// ===========================================================================

/// Results from Phase 2 — AVR sync.
#[derive(Debug, Clone)]
pub struct AvrSync {
    pub status: AvrStatus,
    pub dev_info: DevInfoResp,
    pub config: ConfigResp,
    pub factory_cal: Option<CalDataResp>,
    pub if_cal: Option<CalParamResp>,
    /// AVR hardware config (0xA2). `None` on the Xi, which is not queried.
    pub avr_config: Option<AvrConfigResp>,
}

#[derive(Debug)]
enum AvrStep {
    WaitStatus1,
    WaitStatus2,
    WaitDevInfo1,
    WaitDevInfo2,
    WaitParam0C,
    WaitParam0D,
    WaitConfig,
    WaitFactoryCal,
    WaitIfCal,
    WaitAvrConfig,
    WaitParam64,
    WaitTimeSync,
    Done,
}

/// Pollable state machine for AVR sync (Phase 2).
///
/// Mevo sequence: StatusPoll ×2 → DevInfoReq ×2 → ParamRead 0x0C, 0x0D →
/// ConfigQuery → factory cal (0xD2, optional) → IF cal (0xD0, optional) →
/// AvrConfigQuery (0x23) → ParamRead 0x64 → TimeSync.
///
/// Xi sequence ([`new_for(DeviceModel::Xi)`](Self::new_for)): the same up to
/// ConfigQuery, then IF cal (0xD0, optional) and done. The factory cal
/// request, AvrConfigQuery (which the Xi rejects), param 0x64 and TimeSync
/// are not sent.
pub struct AvrSequencer {
    step: AvrStep,
    model: DeviceModel,
    status: Option<AvrStatus>,
    dev_info: Option<DevInfoResp>,
    config: Option<ConfigResp>,
    factory_cal: Option<CalDataResp>,
    if_cal: Option<CalParamResp>,
    avr_config: Option<AvrConfigResp>,
    /// Timeout tracking for optional cal responses.
    cal_deadline: Option<Instant>,
}

impl AvrSequencer {
    /// AVR sync for a Mevo.
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        Self::new_for(DeviceModel::Mevo)
    }

    /// AVR sync for the given model family.
    #[must_use]
    pub fn new_for(model: DeviceModel) -> (Self, Vec<Action>) {
        let seq = Self {
            step: AvrStep::WaitStatus1,
            model,
            status: None,
            dev_info: None,
            config: None,
            factory_cal: None,
            if_cal: None,
            avr_config: None,
            cal_deadline: None,
        };
        let actions = vec![Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Avr,
        )];
        (seq, actions)
    }

    /// Extract the result. Only valid after `is_complete()`.
    #[must_use]
    pub fn into_result(self) -> AvrSync {
        AvrSync {
            status: self.status.unwrap(),
            dev_info: self.dev_info.unwrap(),
            config: self.config.unwrap(),
            factory_cal: self.factory_cal,
            if_cal: self.if_cal,
            avr_config: self.avr_config,
        }
    }

    /// Check if a calibration step has timed out (optional responses).
    /// Call this periodically in the event loop.
    pub fn check_cal_timeout(&mut self) -> Vec<Action> {
        if let Some(deadline) = self.cal_deadline
            && Instant::now() >= deadline
        {
            self.cal_deadline = None;
            return self.advance_past_cal();
        }
        vec![]
    }

    /// Skip past the current optional cal step.
    fn advance_past_cal(&mut self) -> Vec<Action> {
        match self.step {
            AvrStep::WaitFactoryCal => {
                self.step = AvrStep::WaitIfCal;
                self.cal_deadline = Some(Instant::now() + TIMEOUT);
                vec![Action::Send(
                    Command::CalParamReq(CalParamReq),
                    BusAddr::Avr,
                )]
            }
            AvrStep::WaitIfCal => {
                self.cal_deadline = None;
                self.after_if_cal()
            }
            _ => vec![],
        }
    }

    /// Next step once the IF cal exchange is over (answered or timed out).
    fn after_if_cal(&mut self) -> Vec<Action> {
        if self.model == DeviceModel::Xi {
            self.step = AvrStep::Done;
            return vec![];
        }
        self.step = AvrStep::WaitAvrConfig;
        vec![Action::Send(Command::AvrConfigQuery, BusAddr::Avr)]
    }
}

impl Sequence for AvrSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        // Cal steps use a different skip filter (allow ConfigAck through
        // so we can skip it, but don't reject it as unexpected).
        match self.step {
            AvrStep::WaitFactoryCal | AvrStep::WaitIfCal => {
                if should_skip_for_cal(env, BusAddr::Avr) {
                    return vec![];
                }
            }
            _ => {
                if should_skip_with_mode_ack(env, BusAddr::Avr) {
                    return vec![];
                }
            }
        }

        match self.step {
            AvrStep::WaitStatus1 => {
                if let Message::AvrStatus(ref s) = env.message {
                    self.status = Some(s.clone());
                    self.step = AvrStep::WaitStatus2;
                    return vec![Action::Send(
                        Command::StatusPoll(StatusPoll { pi_mode: false }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitStatus2 => {
                if let Message::AvrStatus(ref s) = env.message {
                    self.status = Some(s.clone());
                    self.step = AvrStep::WaitDevInfo1;
                    return vec![Action::Send(Command::DevInfoReq, BusAddr::Avr)];
                }
                vec![]
            }
            AvrStep::WaitDevInfo1 => {
                if let Message::DevInfoResp(ref r) = env.message {
                    self.dev_info = Some(r.clone());
                    self.step = AvrStep::WaitDevInfo2;
                    return vec![Action::Send(Command::DevInfoReq, BusAddr::Avr)];
                }
                vec![]
            }
            AvrStep::WaitDevInfo2 => {
                if let Message::DevInfoResp(ref r) = env.message {
                    self.dev_info = Some(r.clone());
                    self.step = AvrStep::WaitParam0C;
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: 0x0C }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitParam0C => {
                if let Message::ParamValue(_) = env.message {
                    self.step = AvrStep::WaitParam0D;
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: 0x0D }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitParam0D => {
                if let Message::ParamValue(_) = env.message {
                    self.step = AvrStep::WaitConfig;
                    return vec![Action::Send(Command::ConfigQuery, BusAddr::Avr)];
                }
                vec![]
            }
            AvrStep::WaitConfig => {
                if let Message::ConfigResp(ref r) = env.message {
                    self.config = Some(r.clone());
                    self.cal_deadline = Some(Instant::now() + TIMEOUT);
                    if self.model == DeviceModel::Xi {
                        self.step = AvrStep::WaitIfCal;
                        return vec![Action::Send(
                            Command::CalParamReq(CalParamReq),
                            BusAddr::Avr,
                        )];
                    }
                    self.step = AvrStep::WaitFactoryCal;
                    return vec![Action::Send(
                        Command::CalDataReq(CalDataReq {
                            sub_cmd: 0x03,
                            payload: CalDataReq::encode_factory(),
                        }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitFactoryCal => {
                if let Message::CalDataResp(ref r) = env.message {
                    self.factory_cal = Some(r.clone());
                    self.step = AvrStep::WaitIfCal;
                    self.cal_deadline = Some(Instant::now() + TIMEOUT);
                    return vec![Action::Send(
                        Command::CalParamReq(CalParamReq),
                        BusAddr::Avr,
                    )];
                }
                // ConfigAck is expected here (skip it silently)
                vec![]
            }
            AvrStep::WaitIfCal => {
                if let Message::CalParamResp(ref r) = env.message {
                    self.if_cal = Some(r.clone());
                    self.cal_deadline = None;
                    return self.after_if_cal();
                }
                // ConfigAck is expected here (skip it silently)
                vec![]
            }
            AvrStep::WaitAvrConfig => {
                if let Message::AvrConfigResp(ref r) = env.message {
                    self.avr_config = Some(r.clone());
                    self.step = AvrStep::WaitParam64;
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: 0x64 }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitParam64 => {
                if let Message::ParamValue(_) = env.message {
                    let epoch = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_secs() as u32;
                    self.step = AvrStep::WaitTimeSync;
                    return vec![Action::Send(
                        Command::TimeSync(TimeSync {
                            epoch,
                            session: 0x00,
                            tail: [0x00, 0x01],
                        }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrStep::WaitTimeSync => {
                if let Message::TimeSync(_) = env.message {
                    self.step = AvrStep::Done;
                }
                vec![]
            }
            AvrStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, AvrStep::Done)
    }
}

// ===========================================================================
// Phase 3 — PiSequencer
// ===========================================================================

/// Results from Phase 3 — PI sync.
#[derive(Debug, Clone)]
pub struct PiSync {
    pub dev_info: DevInfoResp,
    pub cam_config: CamConfig,
    pub ssid: String,
    pub password: String,
}

#[derive(Debug)]
enum PiStep {
    WaitStatus,
    WaitDevInfo,
    WaitParam0A,
    WaitCamConfig1,
    WaitCamConfig2,
    WaitNetConfig,
    WaitNetConfigPw,
    /// Waiting for batch of ParamReadReq responses.
    WaitParams {
        ids: Vec<u8>,
        idx: usize,
    },
    Done,
}

/// Pollable state machine for PI sync (Phase 3).
pub struct PiSequencer {
    step: PiStep,
    dev_info: Option<DevInfoResp>,
    cam_config: Option<CamConfig>,
    ssid: String,
    password: String,
}

impl PiSequencer {
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        let seq = Self {
            step: PiStep::WaitStatus,
            dev_info: None,
            cam_config: None,
            ssid: String::new(),
            password: String::new(),
        };
        let actions = vec![Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: true }),
            BusAddr::Pi,
        )];
        (seq, actions)
    }

    #[must_use]
    pub fn into_result(self) -> PiSync {
        PiSync {
            dev_info: self.dev_info.unwrap(),
            cam_config: self.cam_config.unwrap(),
            ssid: self.ssid,
            password: self.password,
        }
    }
}

impl Sequence for PiSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        if should_skip_with_mode_ack(env, BusAddr::Pi) {
            return vec![];
        }
        match self.step {
            PiStep::WaitStatus => {
                if let Message::PiStatus(_) = env.message {
                    self.step = PiStep::WaitDevInfo;
                    return vec![Action::Send(Command::DevInfoReq, BusAddr::Pi)];
                }
                vec![]
            }
            PiStep::WaitDevInfo => {
                if let Message::DevInfoResp(ref r) = env.message {
                    self.dev_info = Some(r.clone());
                    self.step = PiStep::WaitParam0A;
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: 0x0A }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitParam0A => {
                if let Message::ParamValue(_) = env.message {
                    self.step = PiStep::WaitCamConfig1;
                    return vec![Action::Send(
                        Command::CamConfigReq(CamConfigReq),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitCamConfig1 => {
                if let Message::CamConfig(_) = env.message {
                    self.step = PiStep::WaitCamConfig2;
                    return vec![Action::Send(
                        Command::CamConfigReq(CamConfigReq),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitCamConfig2 => {
                if let Message::CamConfig(ref c) = env.message {
                    self.cam_config = Some(c.clone());
                    self.step = PiStep::WaitNetConfig;
                    return vec![Action::Send(
                        Command::NetConfigReq(NetConfigReq {
                            query_password: false,
                        }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitNetConfig => {
                if let Message::NetConfigResp(_) = env.message {
                    self.step = PiStep::WaitNetConfigPw;
                    return vec![Action::Send(
                        Command::NetConfigReq(NetConfigReq {
                            query_password: true,
                        }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitNetConfigPw => {
                if let Message::NetConfigResp(ref r) = env.message {
                    let mut parts = r.text.splitn(2, '\0');
                    self.ssid = parts.next().unwrap_or("").to_string();
                    self.password = parts.next().unwrap_or("").to_string();
                    // Start the param read batches
                    let ids = vec![0x01, 0x07, 0x08, 0x09, 0x06, 0x0B, 0x03, 0x04, 0x05];
                    self.step = PiStep::WaitParams { ids, idx: 0 };
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: 0x01 }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::WaitParams {
                ref ids,
                ref mut idx,
            } => {
                if let Message::ParamValue(_) = env.message {
                    *idx += 1;
                    if *idx >= ids.len() {
                        self.step = PiStep::Done;
                        return vec![];
                    }
                    let next_id = ids[*idx];
                    return vec![Action::Send(
                        Command::ParamReadReq(ParamReadReq { param_id: next_id }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            PiStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, PiStep::Done)
    }
}

// ===========================================================================
// Phase 4 — AvrConfigSequencer
// ===========================================================================

/// Settings for Phase 4 — AVR configuration.
#[derive(Debug, Clone)]
pub struct AvrSettings {
    /// Detection mode commsIndex (see `config::MODE_*` constants).
    pub mode: u8,
    /// BF parameter writes (ball type, tee height, track%, etc.).
    pub params: Vec<ParamValue>,
    /// Radar calibration values. Optional — skip during mode changes
    /// (the FS Golf app only sends RadarCal on initial configuration
    /// or in the pre-disarm phase, never during the configure phase).
    pub radar_cal: Option<RadarCal>,
}

impl AvrSettings {
    /// Configuration for the Xi in limited-flight mode
    /// ([`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT),
    /// 9). Same as [`xi_with_mode`](Self::xi_with_mode) with that mode.
    #[must_use]
    pub fn xi() -> Self {
        Self::xi_with_mode(crate::protocol::config::MODE_LIMITED_FLIGHT)
    }

    /// Configuration for the Xi in the given detection mode (commsIndex),
    /// with no parameter writes and no radar calibration. The Xi keeps the
    /// parameter values stored on the device.
    ///
    /// Known gap: only the mode is written. When FS Golf selects full-flight
    /// outdoor ([`MODE_FULL_OUTDOOR`](crate::protocol::config::MODE_FULL_OUTDOOR))
    /// it also writes params 0x0028, 0x0007, 0x0025 and 0x0001; those writes
    /// are not sent here, and whether the Xi needs them in that mode is
    /// untested.
    #[must_use]
    pub fn xi_with_mode(mode: u8) -> Self {
        Self {
            mode,
            params: vec![],
            radar_cal: None,
        }
    }
}

#[derive(Debug)]
enum AvrConfigStep {
    /// Wait for leading B0[01 00] config-gate ACK before first param.
    WaitInitGateAck,
    /// Send next param, wait for ConfigAck.
    WaitParamAck {
        param_idx: usize,
    },
    /// Wait for B0 commit ConfigAck after param write.
    WaitParamCommitAck {
        param_idx: usize,
    },
    /// Wait for ModeSet echo.
    WaitModeEcho,
    /// Wait for B0 commit after ModeSet.
    WaitModeCommitAck,
    /// Wait for RadarCal echo.
    WaitRadarCalEcho,
    /// Wait for B0 commit after RadarCal.
    WaitRadarCalCommitAck,
    Done,
}

/// Pollable state machine for AVR configuration (Phase 4).
pub struct AvrConfigSequencer {
    step: AvrConfigStep,
    settings: AvrSettings,
}

impl AvrConfigSequencer {
    #[must_use]
    pub fn new(settings: AvrSettings) -> (Self, Vec<Action>) {
        // Send a leading B0[01 00] config-gate before any params.
        // The FS Golf app always opens the configure phase with this command;
        // after disarm the device needs it to enter config mode before arm
        // will be accepted.
        let seq = Self {
            step: AvrConfigStep::WaitInitGateAck,
            settings,
        };
        let actions = vec![Action::Send(
            Command::AvrConfigCmd(AvrConfigCmd { arm: false }),
            BusAddr::Avr,
        )];
        (seq, actions)
    }
}

impl Sequence for AvrConfigSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        if should_skip_with_mode_ack(env, BusAddr::Avr) {
            return vec![];
        }
        let b0_commit = Command::AvrConfigCmd(AvrConfigCmd { arm: false });

        match self.step {
            AvrConfigStep::WaitInitGateAck => {
                if let Message::ConfigAck(_) = env.message {
                    if self.settings.params.is_empty() {
                        // No params — jump straight to ModeSet.
                        self.step = AvrConfigStep::WaitModeEcho;
                        return vec![Action::Send(
                            Command::ModeSet(ModeSet {
                                mode: self.settings.mode,
                            }),
                            BusAddr::Avr,
                        )];
                    }
                    self.step = AvrConfigStep::WaitParamAck { param_idx: 0 };
                    return vec![Action::Send(
                        Command::ParamValue(self.settings.params[0].clone()),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrConfigStep::WaitParamAck { param_idx } => {
                if let Message::ConfigAck(_) = env.message {
                    self.step = AvrConfigStep::WaitParamCommitAck { param_idx };
                    return vec![Action::Send(b0_commit, BusAddr::Avr)];
                }
                vec![]
            }
            AvrConfigStep::WaitParamCommitAck { param_idx } => {
                if let Message::ConfigAck(_) = env.message {
                    let next_idx = param_idx + 1;
                    if next_idx < self.settings.params.len() {
                        self.step = AvrConfigStep::WaitParamAck {
                            param_idx: next_idx,
                        };
                        return vec![Action::Send(
                            Command::ParamValue(self.settings.params[next_idx].clone()),
                            BusAddr::Avr,
                        )];
                    }
                    // All params done — ModeSet
                    self.step = AvrConfigStep::WaitModeEcho;
                    return vec![Action::Send(
                        Command::ModeSet(ModeSet {
                            mode: self.settings.mode,
                        }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            AvrConfigStep::WaitModeEcho => {
                if let Message::ModeSet(_) = env.message {
                    self.step = AvrConfigStep::WaitModeCommitAck;
                    return vec![Action::Send(b0_commit, BusAddr::Avr)];
                }
                vec![]
            }
            AvrConfigStep::WaitModeCommitAck => {
                if let Message::ConfigAck(_) = env.message {
                    if let Some(ref cal) = self.settings.radar_cal {
                        self.step = AvrConfigStep::WaitRadarCalEcho;
                        return vec![Action::Send(Command::RadarCal(cal.clone()), BusAddr::Avr)];
                    }
                    // No RadarCal — skip to done (mode changes don't need it).
                    self.step = AvrConfigStep::Done;
                }
                vec![]
            }
            AvrConfigStep::WaitRadarCalEcho => {
                if let Message::RadarCal(_) = env.message {
                    self.step = AvrConfigStep::WaitRadarCalCommitAck;
                    return vec![Action::Send(b0_commit, BusAddr::Avr)];
                }
                vec![]
            }
            AvrConfigStep::WaitRadarCalCommitAck => {
                if let Message::ConfigAck(_) = env.message {
                    self.step = AvrConfigStep::Done;
                }
                vec![]
            }
            AvrConfigStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, AvrConfigStep::Done)
    }
}

// ===========================================================================
// Phase 5 — CameraConfigSequencer
// ===========================================================================

#[derive(Debug)]
enum CamConfigStep {
    WaitConfigAck,
    WaitReadback,
    WaitCamStateAck,
    WaitParamAck,
    Done,
}

/// Pollable state machine for camera configuration (Phase 5).
pub struct CameraConfigSequencer {
    step: CamConfigStep,
}

impl CameraConfigSequencer {
    #[must_use]
    pub fn new(config: &CamConfig) -> (Self, Vec<Action>) {
        let seq = Self {
            step: CamConfigStep::WaitConfigAck,
        };
        let actions = vec![Action::Send(
            Command::CamConfig(config.clone()),
            BusAddr::Pi,
        )];
        (seq, actions)
    }
}

impl Sequence for CameraConfigSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        if should_skip_with_mode_ack(env, BusAddr::Pi) {
            return vec![];
        }
        match self.step {
            CamConfigStep::WaitConfigAck => {
                if let Message::ConfigAck(_) = env.message {
                    self.step = CamConfigStep::WaitReadback;
                    return vec![Action::Send(
                        Command::CamConfigReq(CamConfigReq),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            CamConfigStep::WaitReadback => {
                if let Message::CamConfig(_) = env.message {
                    self.step = CamConfigStep::WaitCamStateAck;
                    return vec![Action::Send(
                        Command::CamState(CamState { state: 0x01 }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            CamConfigStep::WaitCamStateAck => {
                if let Message::ConfigAck(_) = env.message {
                    self.step = CamConfigStep::WaitParamAck;
                    return vec![Action::Send(
                        Command::ParamValue(ParamValue {
                            param_id: 0x02,
                            value: crate::protocol::config::ParamData::Int24(10),
                        }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            CamConfigStep::WaitParamAck => {
                if let Message::ConfigAck(_) = env.message {
                    self.step = CamConfigStep::Done;
                }
                vec![]
            }
            CamConfigStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, CamConfigStep::Done)
    }
}

// ===========================================================================
// Phase 5.5 — DisarmSequencer
// ===========================================================================

#[derive(Debug)]
enum DisarmStep {
    /// Wait for ConfigAck from the B0[01 00] (arm=false) command.
    WaitAck,
    /// Wait for the device to fully process the disarm: either
    /// ModeAck (0xB1 MODE_RESET) or Text("ARMED CANCELLED").
    WaitModeReset,
    /// Wait for device to report idle state ("System State 5").
    /// The device emits this 0.5-3ms after "ARMED CANCELLED" — it signals
    /// the disarm transition is fully complete.
    WaitStateIdle,
    Done,
}

/// Pollable state machine for explicit disarm before a mode change.
///
/// Sends `AvrConfigCmd(arm=false)` and waits for ConfigAck + ModeAck/ARMED
/// CANCELLED + "System State 5". Must run before `AvrConfigSequencer` on
/// re-configure while the device is armed, so the device is fully idle
/// before new config commands arrive.
pub struct DisarmSequencer {
    step: DisarmStep,
}

impl DisarmSequencer {
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        let seq = Self {
            step: DisarmStep::WaitAck,
        };
        let actions = vec![Action::Send(
            Command::AvrConfigCmd(AvrConfigCmd { arm: false }),
            BusAddr::Avr,
        )];
        (seq, actions)
    }
}

impl Sequence for DisarmSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        match self.step {
            DisarmStep::WaitAck => {
                if env.src != BusAddr::Avr {
                    return vec![];
                }
                // Skip noise (same as other sequencers but NOT ModeAck)
                if matches!(
                    &env.message,
                    Message::Text(_)
                        | Message::DspDebug(_)
                        | Message::CamState(_)
                        | Message::Unknown { .. }
                ) {
                    return vec![];
                }
                if let Message::ConfigAck(_) = env.message {
                    self.step = DisarmStep::WaitModeReset;
                }
                vec![]
            }
            DisarmStep::WaitModeReset => {
                // Accept MODE_RESET (ModeAck 0xB1) or "ARMED CANCELLED" text
                // from any bus — the device sends these asynchronously.
                if let Message::ModeAck(_) = env.message {
                    self.step = DisarmStep::WaitStateIdle;
                    return vec![];
                }
                if let Message::Text(ref t) = env.message
                    && t.text.contains("ARMED CANCELLED")
                {
                    self.step = DisarmStep::WaitStateIdle;
                }
                vec![]
            }
            DisarmStep::WaitStateIdle => {
                // Wait for the DSP to report idle state. This arrives 0.5-3ms
                // after "ARMED CANCELLED" and signals the device has fully
                // completed the disarm transition.
                if let Message::Text(ref t) = env.message
                    && t.text.contains("System State")
                {
                    self.step = DisarmStep::Done;
                }
                // Also accept ModeAck as completion — on some firmware
                // versions MODE_RESET arrives after ARMED CANCELLED and
                // after System State, so it serves as a final "done" signal.
                if let Message::ModeAck(_) = env.message {
                    self.step = DisarmStep::Done;
                }
                vec![]
            }
            DisarmStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, DisarmStep::Done)
    }
}

// ===========================================================================
// Phase 6 — ArmSequencer
// ===========================================================================

#[derive(Debug)]
enum ArmStep {
    WaitDspStatus,
    WaitAvrStatus,
    WaitArmAck,
    /// After ConfigAck, wait for both PiStatus and "ARMED" text.
    /// On older firmware PiStatus arrives first; on BM17.04 (Jan 2026)
    /// the "ARMED DetectionMode=N" text arrives before PiStatus.
    WaitPiAndArmed {
        got_pi: bool,
        got_armed: bool,
    },
    Done,
}

/// Pollable state machine for the ARM phase (Phase 6).
///
/// Sequence (from pcap):
///   1. StatusPoll → DSP  → wait DspStatus
///   2. StatusPoll → AVR  → wait AvrStatus
///   3. AvrConfigCmd(arm=true) → AVR → wait ConfigAck
///   4. StatusPoll → PI   → wait PiStatus + wait "ARMED" text (any order)
///
/// After ConfigAck, the sequencer waits for both PiStatus and "ARMED"
/// text to arrive. These arrive in different orders depending on firmware
/// version: older firmware sends PiStatus first, BM17.04 (Jan 2026)
/// sends the "ARMED DetectionMode=N" text before PiStatus.
///
/// On ConfigNack (0x94), the sequencer restarts from step 1. The device
/// may reject ARM for up to ~30 seconds after a full configuration
/// (params + RadarCal). The FS Golf app uses different param IDs and
/// gets zero ConfigNack on ARM. Mode-only configuration (no params,
/// no RadarCal) still produces ConfigNack retries but typically fewer.
/// The overall operation timeout (set by the caller) limits retries.
///
/// On a model without a camera Pi ([`new_for(DeviceModel::Xi)`](Self::new_for))
/// step 4 skips the PI poll and waits for the "ARMED" text alone.
pub struct ArmSequencer {
    step: ArmStep,
    retries: u16,
    /// Poll the PI after the arm ConfigAck and wait for its status.
    wait_pi: bool,
}

impl ArmSequencer {
    /// Arm a Mevo.
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        Self::new_for(DeviceModel::Mevo)
    }

    /// Arm the given model family.
    #[must_use]
    pub fn new_for(model: DeviceModel) -> (Self, Vec<Action>) {
        let seq = Self {
            step: ArmStep::WaitDspStatus,
            retries: 0,
            wait_pi: model.has_camera(),
        };
        let actions = vec![Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Dsp,
        )];
        (seq, actions)
    }

    /// Number of ConfigNack retries so far.
    #[must_use]
    pub fn retries(&self) -> u16 {
        self.retries
    }
}

impl Sequence for ArmSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        match self.step {
            ArmStep::WaitDspStatus => {
                // Accept DspStatus from DSP (skip noise from other buses)
                if env.src != BusAddr::Dsp {
                    return vec![];
                }
                if matches!(
                    &env.message,
                    Message::Text(_)
                        | Message::DspDebug(_)
                        | Message::CamState(_)
                        | Message::ConfigNack(_)
                        | Message::ModeAck(_)
                        | Message::Unknown { .. }
                ) {
                    return vec![];
                }
                if let Message::DspStatus(_) = env.message {
                    // Poll AVR status before arming. The device requires
                    // this step to transition the AVR into a state that
                    // accepts the arm command (confirmed via pcap).
                    self.step = ArmStep::WaitAvrStatus;
                    return vec![Action::Send(
                        Command::StatusPoll(StatusPoll { pi_mode: false }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            ArmStep::WaitAvrStatus => {
                if should_skip_with_mode_ack(env, BusAddr::Avr) {
                    return vec![];
                }
                if let Message::AvrStatus(_) = env.message {
                    self.step = ArmStep::WaitArmAck;
                    return vec![Action::Send(
                        Command::AvrConfigCmd(AvrConfigCmd { arm: true }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            ArmStep::WaitArmAck => {
                // Don't use should_skip here — we need to see ConfigNack.
                if env.src != BusAddr::Avr {
                    return vec![];
                }
                if matches!(
                    &env.message,
                    Message::Text(_)
                        | Message::DspDebug(_)
                        | Message::CamState(_)
                        | Message::ModeAck(_)
                        | Message::Unknown { .. }
                ) {
                    return vec![];
                }
                if let Message::ConfigNack(_) = env.message {
                    // Device rejected ARM — retry from the top.
                    // Retries may continue for up to ~30s after full
                    // configuration. Use retries() to monitor progress.
                    self.retries += 1;
                    self.step = ArmStep::WaitDspStatus;
                    return vec![Action::Send(
                        Command::StatusPoll(StatusPoll { pi_mode: false }),
                        BusAddr::Dsp,
                    )];
                }
                if let Message::ConfigAck(_) = env.message {
                    if !self.wait_pi {
                        self.step = ArmStep::WaitPiAndArmed {
                            got_pi: true,
                            got_armed: false,
                        };
                        return vec![];
                    }
                    self.step = ArmStep::WaitPiAndArmed {
                        got_pi: false,
                        got_armed: false,
                    };
                    return vec![Action::Send(
                        Command::StatusPoll(StatusPoll { pi_mode: true }),
                        BusAddr::Pi,
                    )];
                }
                vec![]
            }
            ArmStep::WaitPiAndArmed {
                ref mut got_pi,
                ref mut got_armed,
            } => {
                // Collect both PiStatus and "ARMED" text in any order.
                // Firmware BM17.04 sends ARMED text before PiStatus;
                // older firmware sends PiStatus first.
                if let Message::PiStatus(_) = env.message {
                    *got_pi = true;
                }
                if let Message::Text(ref text) = env.message
                    && text.text.contains("ARMED")
                    && !text.text.contains("CANCELLED")
                {
                    *got_armed = true;
                }
                if *got_pi && *got_armed {
                    self.step = ArmStep::Done;
                }
                vec![]
            }
            ArmStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, ArmStep::Done)
    }
}

// ===========================================================================
// ShotSequencer
// ===========================================================================

/// Shot data collected over one shot.
///
/// The results (D4/ED/D9/EF) normally arrive between "BALL TRIGGER" and
/// "PROCESSED"; late ones may still arrive after "PROCESSED", before "IDLE".
/// This struct captures them from either window.
#[derive(Debug, Clone, Default)]
pub struct ShotData {
    /// Primary ball flight result (0xD4).
    pub flight: Option<FlightResult>,
    /// Club head measurements (0xED).
    pub club: Option<ClubResult>,
    /// Spin measurement data (0xEF).
    pub spin: Option<SpinResult>,
    /// Club speed profile (0xD9).
    pub speed_profile: Option<SpeedProfile>,
    /// Ball radar tracking points (0xEC), one per page.
    pub prc: Vec<PrcData>,
    /// Club radar tracking points (0xEE), one per page.
    pub club_prc: Vec<ClubPrc>,
}

/// A piece of shot data yielded during the shot lifecycle, between
/// `BinaryEvent::Trigger` and `BinaryEvent::ShotComplete`.
///
/// Each variant wraps the corresponding protocol type. Only the four
/// data types relevant for real-time shot processing are included;
/// diagnostic data (`SpeedProfile`, `PrcData`, `ClubPrc`) is available
/// in the accumulated [`ShotData`] via [`ShotSequencer::into_result()`].
#[derive(Debug, Clone)]
pub enum ShotDatum {
    /// Primary ball flight (0xD4). Normally arrives before PROCESSED, after
    /// the E8.
    Flight(FlightResult),
    /// Partial/early ball flight (0xE8). Arrives before PROCESSED.
    FlightV1(FlightResultV1),
    /// Club head measurements (0xED). Normally arrives before PROCESSED.
    Club(ClubResult),
    /// Spin data (0xEF). Normally arrives before PROCESSED.
    Spin(SpinResult),
}

#[derive(Debug)]
enum ShotStep {
    /// Draining shot data until IDLE.
    Draining,
    /// Waiting for ClubResult after ShotResultReq.
    WaitingForClubResult,
    /// Waiting for ConfigAck after B0 ARM.
    WaitingForArmAck,
    /// Waiting for "ARMED" text.
    WaitingForArmed,
    Done,
}

/// Duration to wait for IDLE after receiving flight data during drain.
/// If the device doesn't send IDLE within this window (firmware bug on
/// some Gen1 Mevo+ units), proceed directly to ARM without the redundant
/// ShotResultReq — ClubResult already arrives during the drain phase.
const DRAIN_TIMEOUT: Duration = Duration::from_secs(3);

/// Pollable state machine for post-shot handling.
///
/// Created after receiving E5 "PROCESSED". Drives: ack → drain to IDLE →
/// ShotResultReq → ARM → wait ARMED.
///
/// The drain phase collects late shot data (D4/ED/EF/etc.) and passively absorbs
/// ModeAck (0xB1) and ConfigResp (0xA0) messages that arrive as part of the
/// device's natural shot-completion flow. Neither message gates progression —
/// Gen1 sends both after a ConfigQuery (0x21), but Gen2 firmware skips the
/// ConfigQuery/ConfigResp exchange entirely. By not requiring either message,
/// the sequencer works identically across firmware generations.
pub struct ShotSequencer {
    step: ShotStep,
    data: ShotData,
    /// Latest shot data message to yield to the caller. Set during drain
    /// when D4/ED/EF arrive; taken by `BinaryClient` after each `feed()`.
    pending: Option<ShotDatum>,
    /// Deadline for drain phase. Set when FlightResult (D4) arrives.
    /// If IDLE hasn't arrived by this time, skip directly to ARM.
    drain_deadline: Option<Instant>,
}

impl ShotSequencer {
    /// Create a new shot sequencer. Returns initial actions (ShotDataAck ×2).
    #[must_use]
    pub fn new() -> (Self, Vec<Action>) {
        let seq = Self {
            step: ShotStep::Draining,
            data: ShotData::default(),
            pending: None,
            drain_deadline: None,
        };
        let actions = vec![
            Action::Send(Command::ShotDataAck, BusAddr::Avr),
            Action::Send(Command::ShotDataAck, BusAddr::Avr),
        ];
        (seq, actions)
    }

    /// Create a shot sequencer seeded with shot data that arrived before
    /// "PROCESSED" (where D4/ED/EF normally arrive). Same initial actions
    /// as [`new()`](Self::new). The seed does not start the drain timeout;
    /// only a flight result arriving during drain does, because a Mevo in
    /// full-outdoor mode can take about 3 s from "PROCESSED" to IDLE.
    #[must_use]
    pub fn with_data(data: ShotData) -> (Self, Vec<Action>) {
        let (mut seq, actions) = Self::new();
        seq.data = data;
        (seq, actions)
    }

    /// Extract the accumulated shot data. Only valid after `is_complete()`.
    #[must_use]
    pub fn into_result(self) -> ShotData {
        self.data
    }

    /// Borrow the shot data accumulated so far (e.g. to read flight/club
    /// results during draining, before the sequence is complete).
    pub fn data(&self) -> &ShotData {
        &self.data
    }

    /// Take the pending shot datum (if any). Called by `BinaryClient` after
    /// each `feed()` to yield individual data messages as they arrive.
    pub fn take_pending(&mut self) -> Option<ShotDatum> {
        self.pending.take()
    }

    /// Check if the drain phase has timed out waiting for IDLE.
    ///
    /// Some Gen1 Mevo+ firmware intermittently omits the IDLE message
    /// after shot processing, leaving the sequencer stuck in Draining.
    /// When the drain deadline expires, skip the redundant ShotResultReq
    /// (ClubResult already accumulated during drain) and proceed directly
    /// to ARM.
    pub fn check_drain_timeout(&mut self) -> Vec<Action> {
        if !matches!(self.step, ShotStep::Draining) {
            return vec![];
        }
        if let Some(deadline) = self.drain_deadline
            && Instant::now() >= deadline
        {
            self.step = ShotStep::WaitingForArmAck;
            return vec![Action::Send(
                Command::AvrConfigCmd(AvrConfigCmd { arm: true }),
                BusAddr::Avr,
            )];
        }
        vec![]
    }
}

impl Sequence for ShotSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        match self.step {
            ShotStep::Draining => {
                match &env.message {
                    Message::ShotText(st) if st.is_idle() => {
                        // After IDLE, go straight to requesting final results.
                        // Gen1 FS Golf sends 0x21 ConfigQuery here and waits
                        // for B1 ModeAck + A0 ConfigResp before proceeding,
                        // but Gen2 firmware skips that exchange entirely. The
                        // ConfigQuery is informational (re-reads radar params
                        // we already have) and not required for re-arming.
                        // ModeAck arrives naturally during the device's
                        // shot-completion flow and is absorbed passively.
                        self.step = ShotStep::WaitingForClubResult;
                        return vec![Action::Send(Command::ShotResultReq, BusAddr::Avr)];
                    }
                    Message::FlightResult(r) => {
                        self.data.flight = Some(r.clone());
                        self.pending = Some(ShotDatum::Flight(r.clone()));
                        if self.drain_deadline.is_none() {
                            self.drain_deadline = Some(Instant::now() + DRAIN_TIMEOUT);
                        }
                    }
                    Message::ClubResult(r) => {
                        self.data.club = Some(r.clone());
                        self.pending = Some(ShotDatum::Club(r.clone()));
                    }
                    Message::SpinResult(r) => {
                        self.data.spin = Some(r.clone());
                        self.pending = Some(ShotDatum::Spin(r.clone()));
                    }
                    Message::SpeedProfile(r) => self.data.speed_profile = Some(r.clone()),
                    Message::PrcData(r) => self.data.prc.push(r.clone()),
                    Message::ClubPrc(r) => self.data.club_prc.push(r.clone()),
                    _ => {}
                }
                vec![]
            }
            ShotStep::WaitingForClubResult => {
                if let Message::ClubResult(_) = env.message {
                    self.step = ShotStep::WaitingForArmAck;
                    return vec![Action::Send(
                        Command::AvrConfigCmd(AvrConfigCmd { arm: true }),
                        BusAddr::Avr,
                    )];
                }
                vec![]
            }
            ShotStep::WaitingForArmAck => {
                if should_skip_with_mode_ack(env, BusAddr::Avr) {
                    return vec![];
                }
                if let Message::ConfigAck(_) = env.message {
                    self.step = ShotStep::WaitingForArmed;
                }
                vec![]
            }
            ShotStep::WaitingForArmed => {
                if let Message::Text(ref text) = env.message
                    && text.text.contains("ARMED")
                    && !text.text.contains("CANCELLED")
                {
                    self.step = ShotStep::Done;
                }
                vec![]
            }
            ShotStep::Done => vec![],
        }
    }

    fn is_complete(&self) -> bool {
        matches!(self.step, ShotStep::Done)
    }
}

// ===========================================================================
// Blocking convenience wrappers (preserve pre-v0.1 API)
// ===========================================================================

/// Receive the next message, silently skipping 0xE3 Text debug logs.
///
/// Uses deadline tracking so Text messages don't extend the timeout.
pub fn recv_msg(conn: &mut Connection, timeout: Duration) -> Result<Envelope, ConnError> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout);
        }
        let env = conn.recv_timeout(remaining)?;
        if matches!(env.message, Message::Text(_) | Message::DspDebug(_)) {
            continue;
        }
        return Ok(env);
    }
}

/// Send a command to `dest`, then receive the next non-Text response from that bus.
pub fn send_recv(
    conn: &mut Connection,
    cmd: &Command,
    dest: BusAddr,
    timeout: Duration,
) -> Result<Envelope, ConnError> {
    conn.send(cmd, dest)?;
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout);
        }
        let env = conn.recv_timeout(remaining)?;
        if env.src != dest {
            continue;
        }
        match &env.message {
            Message::Text(_)
            | Message::DspDebug(_)
            | Message::CamState(_)
            | Message::ModeAck(_)
            | Message::ConfigNack(_)
            | Message::Unknown { .. } => continue,
            _ => {}
        }
        return Ok(env);
    }
}

/// Destructure an Envelope's message or return ConnError::Protocol.
macro_rules! expect {
    ($env:expr, $pat:path) => {
        match $env.message {
            $pat(inner) => Ok(inner),
            _ => {
                let hex: String = $env
                    .raw
                    .iter()
                    .map(|b| format!("{b:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ");
                Err(ConnError::Protocol(format!(
                    "expected {}, got UnknownMessage(\"0x{:02X}\", [{}])",
                    stringify!($pat),
                    $env.type_id,
                    hex,
                )))
            }
        }
    };
}

/// Phase 1: Query DSP (blocking wrapper).
pub fn sync_dsp(conn: &mut Connection) -> Result<DspSync, ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = DspSequencer::new();
    let deadline = Instant::now() + Duration::from_secs(30);

    // DspSequencer has an optional DSP query step that can time out
    for a in actions {
        send_action(conn, a)?;
    }
    loop {
        if Instant::now() >= deadline {
            return Err(ConnError::Timeout);
        }
        for a in seq.check_timeout() {
            send_action(conn, a)?;
        }
        if let Some(env) = conn.recv()? {
            for a in seq.feed(&env) {
                send_action(conn, a)?;
            }
            if seq.is_complete() {
                return Ok(seq.into_result());
            }
        }
    }
}

/// Phase 2: Query AVR on a Mevo (blocking wrapper).
pub fn sync_avr(conn: &mut Connection) -> Result<AvrSync, ConnError> {
    sync_avr_for(conn, DeviceModel::Mevo)
}

/// Phase 2: Query AVR on the given model family (blocking wrapper).
pub fn sync_avr_for(conn: &mut Connection, model: DeviceModel) -> Result<AvrSync, ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = AvrSequencer::new_for(model);
    let deadline = Instant::now() + Duration::from_secs(30);

    // AvrSequencer has optional cal steps that can timeout
    for a in actions {
        send_action(conn, a)?;
    }
    loop {
        if Instant::now() >= deadline {
            return Err(ConnError::Timeout);
        }
        // Check cal timeouts
        let timeout_actions = seq.check_cal_timeout();
        for a in timeout_actions {
            send_action(conn, a)?;
        }
        if seq.is_complete() {
            return Ok(seq.into_result());
        }
        if let Some(env) = conn.recv()? {
            let actions = seq.feed(&env);
            for a in actions {
                send_action(conn, a)?;
            }
            if seq.is_complete() {
                return Ok(seq.into_result());
            }
        }
    }
}

/// Phase 3: Query PI (blocking wrapper).
pub fn sync_pi(conn: &mut Connection) -> Result<PiSync, ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = PiSequencer::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    drive(conn, &mut seq, actions, deadline)?;
    Ok(seq.into_result())
}

/// Phase 4: Write AVR parameters, set detection mode, radar calibration (blocking wrapper).
pub fn configure_avr(conn: &mut Connection, settings: &AvrSettings) -> Result<(), ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = AvrConfigSequencer::new(settings.clone());
    let deadline = Instant::now() + Duration::from_secs(30);
    drive(conn, &mut seq, actions, deadline)
}

/// Phase 5: Push camera config, start camera, set PI keepalive param (blocking wrapper).
pub fn configure_camera(conn: &mut Connection, config: &CamConfig) -> Result<(), ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = CameraConfigSequencer::new(config);
    let deadline = Instant::now() + Duration::from_secs(30);
    drive(conn, &mut seq, actions, deadline)
}

/// Phase 6: Final status checks, arm a Mevo, wait for "ARMED" (blocking wrapper).
pub fn arm(conn: &mut Connection) -> Result<(), ConnError> {
    arm_for(conn, DeviceModel::Mevo)
}

/// Phase 6: Final status checks, arm the given model family, wait for "ARMED"
/// (blocking wrapper).
pub fn arm_for(conn: &mut Connection, model: DeviceModel) -> Result<(), ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    let (mut seq, actions) = ArmSequencer::new_for(model);
    let deadline = Instant::now() + Duration::from_secs(30);
    drive(conn, &mut seq, actions, deadline)
}

/// Status from all three nodes, collected during a keepalive poll.
#[derive(Debug, Clone)]
pub struct KeepaliveStatus {
    pub dsp: DspStatus,
    pub avr: AvrStatus,
    pub pi: PiStatus,
}

/// Poll DSP → AVR → PI sequentially on a Mevo (blocking wrapper).
///
/// This always polls the PI, which the Xi does not have, so the blocking
/// API cannot drive an Xi end to end; use [`BinaryClient`](crate::BinaryClient)
/// (or [`keepalive_actions_for`]) for the Xi.
pub fn keepalive(conn: &mut Connection) -> Result<KeepaliveStatus, ConnError> {
    let t = TIMEOUT;

    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: false }),
        BusAddr::Dsp,
        t,
    )?;
    let dsp = expect!(env, Message::DspStatus)?;

    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: false }),
        BusAddr::Avr,
        t,
    )?;
    let avr = expect!(env, Message::AvrStatus)?;

    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: true }),
        BusAddr::Pi,
        t,
    )?;
    let pi = expect!(env, Message::PiStatus)?;

    Ok(KeepaliveStatus { dsp, avr, pi })
}

/// Handle the entire post-shot cycle (blocking wrapper).
///
/// Called after receiving E5 "PROCESSED". Drives: ack → drain to IDLE →
/// ConfigQuery → ShotResultReq → ARM → wait ARMED.
pub fn complete_shot(conn: &mut Connection, log: impl Fn(&str)) -> Result<ShotData, ConnError> {
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(100)))?;
    log("waiting for IDLE...");
    let (mut seq, actions) = ShotSequencer::new();
    let deadline = Instant::now() + Duration::from_secs(30);
    drive(conn, &mut seq, actions, deadline)?;
    log("RE-ARMED");
    Ok(seq.into_result())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::RawFrame;

    /// Build a device → APP envelope.
    fn env(src: BusAddr, type_id: u8, payload: Vec<u8>) -> Envelope {
        let frame = RawFrame {
            dest: BusAddr::App,
            src,
            type_id,
            payload,
        };
        Envelope {
            src,
            type_id,
            raw: frame.payload.clone(),
            message: Message::decode(&frame).unwrap(),
        }
    }

    fn sent(actions: &[Action]) -> Vec<(BusAddr, u8)> {
        actions
            .iter()
            .map(|Action::Send(cmd, dest)| (*dest, cmd.encode(*dest).type_id))
            .collect()
    }

    fn dsp_status() -> Envelope {
        let mut p = vec![0u8; 29];
        p[0] = 0x1C;
        env(BusAddr::Dsp, 0xAA, p)
    }

    #[test]
    fn dsp_query_nack_moves_on() {
        let (mut seq, _) = DspSequencer::new();
        assert_eq!(sent(&seq.feed(&dsp_status())), [(BusAddr::Dsp, 0x48)]);
        let nack = env(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
        assert_eq!(sent(&seq.feed(&nack)), [(BusAddr::Dsp, 0x67)]);
    }

    #[test]
    fn dsp_query_timeout_moves_on() {
        let (mut seq, _) = DspSequencer::new();
        seq.feed(&dsp_status());
        assert!(seq.check_timeout().is_empty());
        seq.query_deadline = Some(Instant::now());
        assert_eq!(sent(&seq.check_timeout()), [(BusAddr::Dsp, 0x67)]);
        // Only once.
        assert!(seq.check_timeout().is_empty());
    }

    #[test]
    fn xi_avr_sync_ends_after_if_cal() {
        let (mut seq, actions) = AvrSequencer::new_for(DeviceModel::Xi);
        assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
        let mut status = vec![0u8; 25];
        status[0] = 0x18;
        let status = env(BusAddr::Avr, 0xAA, status);
        let dev_info = env(BusAddr::Avr, 0xE7, vec![0u8; 75]);
        let param = |id| env(BusAddr::Avr, 0xBF, vec![0x06, 0x00, 0x00, id, 0, 0, 0]);
        let mut config = vec![0u8; 69];
        config[0] = 0x44;

        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0xAA)]);
        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0x67)]);
        assert_eq!(sent(&seq.feed(&dev_info)), [(BusAddr::Avr, 0x67)]);
        assert_eq!(sent(&seq.feed(&dev_info)), [(BusAddr::Avr, 0xBE)]);
        assert_eq!(sent(&seq.feed(&param(0x0C))), [(BusAddr::Avr, 0xBE)]);
        assert_eq!(sent(&seq.feed(&param(0x0D))), [(BusAddr::Avr, 0x21)]);
        // ConfigResp → IF cal request, no factory cal.
        let config = env(BusAddr::Avr, 0xA0, config);
        assert_eq!(sent(&seq.feed(&config)), [(BusAddr::Avr, 0xD0)]);
        let if_cal = env(BusAddr::Avr, 0xD1, vec![0u8; 171]);
        assert!(seq.feed(&if_cal).is_empty());
        assert!(seq.is_complete());
        let sync = seq.into_result();
        assert!(sync.factory_cal.is_none());
        assert!(sync.if_cal.is_some());
        assert!(sync.avr_config.is_none());
    }

    #[test]
    fn xi_avr_sync_if_cal_timeout_completes() {
        let (mut seq, _) = AvrSequencer::new_for(DeviceModel::Xi);
        seq.step = AvrStep::WaitIfCal;
        seq.status = Some(AvrStatus::decode(&[0u8; 25]).unwrap());
        seq.cal_deadline = Some(Instant::now());
        assert!(seq.check_cal_timeout().is_empty());
        assert!(seq.is_complete());
    }

    #[test]
    fn xi_arm_waits_for_armed_text_only() {
        let (mut seq, actions) = ArmSequencer::new_for(DeviceModel::Xi);
        assert_eq!(sent(&actions), [(BusAddr::Dsp, 0xAA)]);
        assert_eq!(sent(&seq.feed(&dsp_status())), [(BusAddr::Avr, 0xAA)]);
        let avr_status = env(BusAddr::Avr, 0xAA, vec![0u8; 25]);
        assert_eq!(sent(&seq.feed(&avr_status)), [(BusAddr::Avr, 0xB0)]);
        let ack = env(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
        assert!(seq.feed(&ack).is_empty());
        assert!(!seq.is_complete());
        let armed = env(BusAddr::Avr, 0xE3, b"\n\rARMED Xi\0".to_vec());
        seq.feed(&armed);
        assert!(seq.is_complete());
    }

    #[test]
    fn mevo_arm_polls_pi() {
        let (mut seq, _) = ArmSequencer::new();
        seq.feed(&dsp_status());
        seq.feed(&env(BusAddr::Avr, 0xAA, vec![0u8; 25]));
        let ack = env(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
        assert_eq!(sent(&seq.feed(&ack)), [(BusAddr::Pi, 0xAA)]);
        seq.feed(&env(BusAddr::Avr, 0xE3, b"\n\rARMED\0".to_vec()));
        assert!(!seq.is_complete());
        seq.feed(&env(BusAddr::Pi, 0xAA, vec![0u8; 17]));
        assert!(seq.is_complete());
    }

    #[test]
    fn xi_settings_mode_only() {
        use crate::protocol::config::{MODE_FULL_OUTDOOR, MODE_LIMITED_FLIGHT};
        assert_eq!(MODE_LIMITED_FLIGHT, 9);
        assert_eq!(MODE_FULL_OUTDOOR, 8);
        let s = AvrSettings::xi();
        assert_eq!(s.mode, MODE_LIMITED_FLIGHT);
        assert!(s.params.is_empty());
        assert!(s.radar_cal.is_none());
        let s = AvrSettings::xi_with_mode(MODE_FULL_OUTDOOR);
        assert_eq!(s.mode, MODE_FULL_OUTDOOR);
        assert!(s.params.is_empty());
        assert!(s.radar_cal.is_none());
    }

    #[test]
    fn xi_avr_config_writes_only_the_mode() {
        use crate::protocol::config::MODE_FULL_OUTDOOR;
        let (mut seq, actions) =
            AvrConfigSequencer::new(AvrSettings::xi_with_mode(MODE_FULL_OUTDOOR));
        let mut all = actions;
        let ack = env(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
        let mut guard = 0;
        while !seq.is_complete() {
            guard += 1;
            assert!(guard < 10, "config sequence did not finish: {all:?}");
            let reply = match all.last() {
                Some(Action::Send(Command::ModeSet(m), _)) => {
                    env(BusAddr::Avr, 0xA5, vec![0x02, 0x00, m.mode])
                }
                _ => ack.clone(),
            };
            all.extend(seq.feed(&reply));
        }
        let wire: Vec<(u8, Vec<u8>)> = all
            .iter()
            .map(|Action::Send(cmd, dest)| {
                let f = cmd.encode(*dest);
                (f.type_id, f.payload)
            })
            .collect();
        assert_eq!(
            wire,
            [
                (0xB0, vec![0x01, 0x00]),
                (0xA5, vec![0x02, 0x00, 0x08]),
                (0xB0, vec![0x01, 0x00]),
            ]
        );
    }

    #[test]
    fn keepalive_polls_pi_only_with_camera() {
        assert_eq!(
            sent(&keepalive_actions()),
            [
                (BusAddr::Dsp, 0xAA),
                (BusAddr::Avr, 0xAA),
                (BusAddr::Pi, 0xAA)
            ]
        );
        assert_eq!(
            sent(&keepalive_actions_for(DeviceModel::Xi)),
            [(BusAddr::Dsp, 0xAA), (BusAddr::Avr, 0xAA)]
        );
    }
}
