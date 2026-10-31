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
    AvrConfigCmd, AvrConfigResp, ConfigResp, ModeSet, ParamData, ParamReadReq, ParamValue, RadarCal,
};
use crate::protocol::handshake::{
    CalDataReq, CalDataResp, CalParamReq, CalParamResp, DevInfoResp, DeviceModel, DspQueryResp,
    NetConfigReq, ProdInfoReq, ProdInfoResp, TimeSync,
};
use crate::protocol::shot::{
    ClubPrc, ClubResult, FlightResult, FlightResultV1, PrcData, SpeedProfile, SpinResult,
};
use crate::protocol::status::{AvrStatus, ConfigAck, DspStatus, PiStatus, StatusPoll};
use crate::protocol::{
    Command, Message, TYPE_DSP_QUERY, TYPE_FLIGHT_RESULT, TYPE_FLIGHT_RESULT_V1, TYPE_PARAM_VALUE,
    TYPE_STATUS,
};

/// Per-exchange timeout (2s for blocking wrappers).
pub const TIMEOUT: Duration = Duration::from_secs(2);

/// How long the DSP sequencer waits for the DSP query response (0xC8) before
/// carrying on without it. A response from the Xi's DSP has not been
/// observed, so the handshake does not depend on one.
pub const DSP_QUERY_TIMEOUT: Duration = TIMEOUT;

/// How long the AVR sequencer waits before re-sending a status poll the
/// AVR refused (0x94). FS Golf re-polls at about this interval while the
/// Xi's AVR starts up.
pub const STATUS_REPOLL_INTERVAL: Duration = Duration::from_secs(1);

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

/// The app's ack of an Xi flight result.
///
/// The Xi expects the app to answer every flight result from its AVR with
/// a 0x95 ack, sent as soon as the frame is received: `[02 10 68]` for an
/// 0xE8 and `[02 10 54]` for a 0xD4. Returns that ack (to the AVR) when
/// `model` is the Xi and `env` is one of those results, and `None`
/// otherwise; the Mevo+ and Gen2 are sent no result acks.
///
/// [`BinaryClient`](crate::BinaryClient) sends it for every frame it
/// receives. Call this when driving the sequencers directly.
#[must_use]
pub fn result_ack(model: DeviceModel, env: &Envelope) -> Option<Action> {
    let is_result = matches!(env.type_id, TYPE_FLIGHT_RESULT_V1 | TYPE_FLIGHT_RESULT);
    (model == DeviceModel::Xi && env.src == BusAddr::Avr && is_result).then(|| {
        Action::Send(
            Command::ConfigAck(ConfigAck::for_result(env.type_id)),
            BusAddr::Avr,
        )
    })
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
    /// Xi only: `true` when the DSP did not answer the power-mode write
    /// within [`DSP_QUERY_TIMEOUT`] and the sync carried on without a
    /// reply. Always `false` on the Mevo+ and Gen2, which are not sent it.
    pub power_mode_unanswered: bool,
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
    /// Xi only: wait for the 0x95 (or 0x94) of the power-mode write, at
    /// most [`DSP_QUERY_TIMEOUT`].
    WaitPowerMode,
    Done,
}

/// Param 0x01 on the Xi's DSP, the power mode: 1 = Normal, 4 = Standby.
/// (Param 0x01 on the AVR is a different parameter, the altitude.)
const XI_DSP_POWER_MODE_ID: u8 = 0x01;
/// DSP power mode Normal (1), written at every Xi connection. A DSP left
/// in Standby (4) wakes up; one already in Normal is unchanged.
const XI_DSP_POWER_MODE_NORMAL: i32 = 1;

/// Pollable state machine for DSP sync (Phase 1).
///
/// Sequence: StatusPoll → DspQuery (0x48) → DevInfoReq → ProdInfoReq ×3 →
/// ConfigQuery. The DSP query is optional: a ConfigNack for it, or no answer
/// within [`DSP_QUERY_TIMEOUT`] (see [`check_timeout()`](Self::check_timeout)),
/// moves on to DevInfoReq with no hardware info.
///
/// On the Xi the sequence ends with a power-mode write to the DSP, BF
/// `[06 00 00 01 00 00 01]` (param 0x01 = 1, Normal), which wakes a DSP left
/// in Standby. It completes on the DSP's 0x95, or on a 0x94 refusing it;
/// the refusal is kept for [`take_refused()`](Self::take_refused). With no
/// reply within [`DSP_QUERY_TIMEOUT`] it completes too, and
/// [`DspSync::power_mode_unanswered`] is set. The Mevo+ and Gen2 are not
/// sent it.
pub struct DspSequencer {
    step: DspStep,
    status: Option<DspStatus>,
    hw_info: Option<DspQueryResp>,
    dev_info: Option<DevInfoResp>,
    prod_info: Vec<ProdInfoResp>,
    config: Option<ConfigResp>,
    /// Deadline for the reply to the DSP query or the power-mode write.
    deadline: Option<Instant>,
    /// 0x94 that ended the power-mode write, not yet taken.
    refused: Option<Box<Envelope>>,
    /// The power-mode write went unanswered.
    power_mode_unanswered: bool,
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
            deadline: None,
            refused: None,
            power_mode_unanswered: false,
        };
        let actions = vec![Action::Send(
            Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Dsp,
        )];
        (seq, actions)
    }

    /// The 0x94 with which the DSP refused the Xi power-mode write, once.
    /// The sequence carries on without it; log the frame's hex.
    #[must_use]
    pub fn take_refused(&mut self) -> Option<Envelope> {
        self.refused.take().map(|env| *env)
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
            power_mode_unanswered: self.power_mode_unanswered,
        }
    }

    /// Check whether the DSP query or the Xi power-mode write has gone
    /// unanswered for [`DSP_QUERY_TIMEOUT`]. A timed-out query moves on to
    /// DevInfoReq; a timed-out power-mode write completes the sequence
    /// (check [`is_complete()`](Sequence::is_complete) afterwards).
    /// Call this periodically in the event loop.
    pub fn check_timeout(&mut self) -> Vec<Action> {
        let Some(deadline) = self.deadline else {
            return vec![];
        };
        if Instant::now() < deadline {
            return vec![];
        }
        match self.step {
            DspStep::WaitDspQuery => self.skip_dsp_query(),
            DspStep::WaitPowerMode => {
                self.deadline = None;
                self.power_mode_unanswered = true;
                self.step = DspStep::Done;
                vec![]
            }
            _ => vec![],
        }
    }

    /// Give up on the DSP query response and request device info.
    fn skip_dsp_query(&mut self) -> Vec<Action> {
        if !matches!(self.step, DspStep::WaitDspQuery) {
            return vec![];
        }
        self.deadline = None;
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
        if matches!(self.step, DspStep::WaitPowerMode) && env.src == BusAddr::Dsp {
            match env.message {
                Message::ConfigAck(ref a) if a.acked_cmd == TYPE_PARAM_VALUE & 0x7F => {
                    self.deadline = None;
                    self.step = DspStep::Done;
                }
                Message::ConfigNack(ref a) if a.acked_cmd == TYPE_PARAM_VALUE & 0x7F => {
                    self.deadline = None;
                    self.refused = Some(Box::new(env.clone()));
                    self.step = DspStep::Done;
                }
                _ => {}
            }
            return vec![];
        }
        if should_skip_with_mode_ack(env, BusAddr::Dsp) {
            return vec![];
        }
        match self.step {
            DspStep::WaitStatus => {
                if let Message::DspStatus(ref s) = env.message {
                    self.status = Some(s.clone());
                    self.step = DspStep::WaitDspQuery;
                    self.deadline = Some(Instant::now() + DSP_QUERY_TIMEOUT);
                    return vec![Action::Send(Command::DspQuery, BusAddr::Dsp)];
                }
                vec![]
            }
            DspStep::WaitDspQuery => {
                if let Message::DspQueryResp(ref r) = env.message {
                    self.hw_info = Some(r.clone());
                    self.deadline = None;
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
                    if self
                        .dev_info
                        .as_ref()
                        .is_some_and(|d| d.model() == DeviceModel::Xi)
                    {
                        self.step = DspStep::WaitPowerMode;
                        self.deadline = Some(Instant::now() + DSP_QUERY_TIMEOUT);
                        return vec![Action::Send(
                            Command::ParamValue(ParamValue {
                                param_id: XI_DSP_POWER_MODE_ID,
                                value: ParamData::Int24(XI_DSP_POWER_MODE_NORMAL),
                            }),
                            BusAddr::Dsp,
                        )];
                    }
                    self.step = DspStep::Done;
                }
                vec![]
            }
            DspStep::WaitPowerMode | DspStep::Done => vec![],
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
///
/// An AVR that is still starting up refuses the status poll with a 0x94
/// (`[02 30 2A]`); the Xi does so for several seconds after power-on. The
/// poll is then sent again [`STATUS_REPOLL_INTERVAL`] later, from
/// [`check_cal_timeout()`](Self::check_cal_timeout), once per refusal,
/// until the AVR answers. The caller's overall deadline bounds the wait.
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
    /// When to re-send a refused status poll.
    repoll_at: Option<Instant>,
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
            repoll_at: None,
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

    /// Check the timed steps: re-send a refused status poll once
    /// [`STATUS_REPOLL_INTERVAL`] has passed, and move past an optional cal
    /// step that has gone unanswered. Call this periodically in the event
    /// loop.
    pub fn check_cal_timeout(&mut self) -> Vec<Action> {
        if let Some(at) = self.repoll_at
            && Instant::now() >= at
        {
            self.repoll_at = None;
            return vec![Action::Send(
                Command::StatusPoll(StatusPoll { pi_mode: false }),
                BusAddr::Avr,
            )];
        }
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
        if matches!(self.step, AvrStep::WaitStatus1 | AvrStep::WaitStatus2)
            && env.src == BusAddr::Avr
            && let Message::ConfigNack(ref nack) = env.message
            && nack.acked_cmd == TYPE_STATUS & 0x7F
        {
            self.repoll_at
                .get_or_insert_with(|| Instant::now() + STATUS_REPOLL_INTERVAL);
            return vec![];
        }
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
                    self.repoll_at = None;
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
                    self.repoll_at = None;
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
    /// Mode-only configuration for the Xi in limited-flight mode
    /// ([`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT),
    /// 9). Same as [`xi_with_mode`](Self::xi_with_mode) with that mode.
    #[deprecated(
        note = "sends the detection mode only; use BinaryClient::configure_xi with XiSettings"
    )]
    #[must_use]
    pub fn xi() -> Self {
        #[allow(deprecated)]
        Self::xi_with_mode(crate::protocol::config::MODE_LIMITED_FLIGHT)
    }

    /// Mode-only configuration for the Xi in the given detection mode
    /// (commsIndex). [`AvrConfigSequencer`] sends B0 `[01 00]`, A5
    /// `[02 00 mode]` and B0 `[01 00]`: no parameter writes and no radar
    /// calibration, so the Xi runs on whatever values it holds.
    ///
    /// [`BinaryClient::configure_xi`](crate::BinaryClient::configure_xi)
    /// sends the full configuration FS Golf sends; see [`XiSettings`].
    #[deprecated(
        note = "sends the detection mode only; use BinaryClient::configure_xi with XiSettings"
    )]
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
// Phase 4 (Xi) — XiConfigSequencer
// ===========================================================================

/// Session type of an Xi configuration, written as param 0x16 (`ClubType`).
///
/// FS Golf writes 1 for its "Full swing" sessions and 2 for "Pitching"
/// (chips and pitches). It pairs Pitching with detection mode 9
/// ([`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT)).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum XiShotType {
    /// Full swing: param 0x16 = 1.
    #[default]
    FullSwing,
    /// Pitching: param 0x16 = 2.
    Pitching,
}

impl XiShotType {
    /// Value written to param 0x16 (`ClubType`).
    #[must_use]
    pub fn param_value(self) -> i32 {
        match self {
            Self::FullSwing => 1,
            Self::Pitching => 2,
        }
    }
}

/// Configuration for the Xi, sent the way FS Golf sends it.
///
/// [`XiConfigSequencer`] (and
/// [`BinaryClient::configure_xi`](crate::BinaryClient::configure_xi)) turn
/// this into FS Golf's frames: a connect-time block once per connection,
/// then a mode block for every mode change. See [`XiConfigSequencer`] for
/// the exact bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct XiSettings {
    /// Detection mode commsIndex:
    /// [`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT)
    /// (9) for nets and simulator screens, or
    /// [`MODE_FULL_OUTDOOR`](crate::protocol::config::MODE_FULL_OUTDOOR) (8).
    pub mode: u8,
    /// Full swing or pitching (param 0x16, `ClubType`).
    pub shot_type: XiShotType,
    /// Distance from the Xi to the ball and surface height (0xA4).
    pub radar_cal: RadarCal,
}

impl Default for XiSettings {
    /// Full swing in limited-flight mode, with FS Golf's default radar
    /// calibration: 2438 mm (8 ft) to the ball, 25 mm (1 in) surface height.
    fn default() -> Self {
        Self {
            mode: crate::protocol::config::MODE_LIMITED_FLIGHT,
            shot_type: XiShotType::FullSwing,
            radar_cal: RadarCal {
                range_mm: 2438,
                height_mm: 25,
            },
        }
    }
}

/// Reply that completes one step of the Xi configuration. A 0x94
/// `CONFIG_NACK` of the step's command type also completes any step.
#[derive(Debug, Clone, Copy)]
enum XiReply {
    /// 0x95 `CONFIG_ACK` of the step's command type.
    Ack,
    /// A5 echo.
    ModeEcho,
    /// A4 echo.
    RadarCalEcho,
    /// 0xBF read response for this param.
    Param(u8),
}

// Xi parameters (0xBF writes), with FS Golf's names where it has one, and
// the values ironsight writes. Every value is the one FS Golf writes.

/// Param 0x01, Altitude.
const XI_ALTITUDE_ID: u8 = 0x01;
/// Altitude: 0.
const XI_ALTITUDE: i32 = 0;
/// Param 0x02 (no FS Golf name). The connect-time block reads it before
/// writing it.
const XI_PARAM_02_ID: u8 = 0x02;
/// Param 0x02: 3.
const XI_PARAM_02: i32 = 3;
/// Param 0x07, `ProcessFlags` (a bitfield).
const XI_PROCESS_FLAGS_ID: u8 = 0x07;
/// `ProcessFlags`, first write of the connect-time block: 0x140B
/// (bits 0, 1, 3, 10, 12).
const XI_PROCESS_FLAGS_INITIAL: i32 = 0x140B;
/// `ProcessFlags`, every later write: 0x5448 (bits 3, 6, 10, 12, 14).
const XI_PROCESS_FLAGS: i32 = 0x5448;
/// Param 0x08, `TriggerOptions`.
const XI_TRIGGER_OPTIONS_ID: u8 = 0x08;
/// `TriggerOptions`: 1, the standard trigger. FS Golf writes 5 for its slow
/// swing option, which ironsight does not offer.
const XI_TRIGGER_OPTIONS: i32 = 1;
/// Param 0x14 (no FS Golf name).
const XI_PARAM_14_ID: u8 = 0x14;
/// Param 0x14: 2.
const XI_PARAM_14: i32 = 2;
/// Param 0x16, `ClubType`: the session type, from [`XiShotType`].
const XI_CLUB_TYPE_ID: u8 = 0x16;
/// Param 0x20 (no FS Golf name), written after every `ClubType` write.
const XI_PARAM_20_ID: u8 = 0x20;
/// Param 0x20: 0.0.
const XI_PARAM_20: f64 = 0.0;
/// Param 0x25, `OperationFlags`.
const XI_OPERATION_FLAGS_ID: u8 = 0x25;
/// `OperationFlags`: 0, no flags set.
const XI_OPERATION_FLAGS: i32 = 0;
/// Param 0x27, `OutdoorMinimumTrackPercentage`.
const XI_OUTDOOR_MIN_TRACK_PCT_ID: u8 = 0x27;
/// `OutdoorMinimumTrackPercentage`: 30 % (FLOAT40 `00 05 78 00 00`).
const XI_OUTDOOR_MIN_TRACK_PCT: f64 = 30.0;
/// Param 0x28, `IndoorModelDragFraction`.
const XI_INDOOR_DRAG_FRACTION_ID: u8 = 0x28;
/// `IndoorModelDragFraction`: 1.0 (FLOAT40 `00 01 40 00 00`).
const XI_INDOOR_DRAG_FRACTION: f64 = 1.0;

fn xi_int(param_id: u8, value: i32) -> (Command, XiReply) {
    (
        Command::ParamValue(ParamValue {
            param_id,
            value: ParamData::Int24(value),
        }),
        XiReply::Ack,
    )
}

fn xi_float(param_id: u8, value: f64) -> (Command, XiReply) {
    (
        Command::ParamValue(ParamValue {
            param_id,
            value: ParamData::Float40(value),
        }),
        XiReply::Ack,
    )
}

fn xi_mode(mode: u8) -> (Command, XiReply) {
    (Command::ModeSet(ModeSet { mode }), XiReply::ModeEcho)
}

fn xi_gate() -> (Command, XiReply) {
    (
        Command::AvrConfigCmd(AvrConfigCmd { arm: false }),
        XiReply::Ack,
    )
}

/// Pollable state machine for the Xi's configuration (Phase 4).
///
/// Sends FS Golf's frames one at a time, each after the reply to the one
/// before: 0x95 for a 0xBF write or B0, the echo for A5 and A4, the 0xBF
/// value for the 0xBE read. A 0x94 refusing the step's command (its
/// `acked_cmd` is the command type & 0x7F) also ends any step, so the
/// device refusing a write, a mode set, the radar calibration or the read
/// does not stall the sequence: it carries on with the next frame. The
/// refusal is kept for [`take_refused()`](Self::take_refused). A step that
/// gets no reply at all waits for the caller's timeout.
///
/// [`connect`](Self::connect) sends the connect-time block, then the mode
/// block. `CC` is the [`XiShotType`] value, `MM` the mode, `RR RR` and `HH`
/// the radar calibration's range and height in mm:
///
/// ```text
/// BF 06 00 00 01 00 00 00         Altitude = 0
/// A5 02 00 08                     mode 8
/// BE 03 00 00 02                  read param 0x02
/// A4 06 RR RR 00 HH 00 00         radar calibration
/// BF 06 00 00 08 00 00 01         TriggerOptions = 1
/// BF 06 00 00 07 00 14 0B         ProcessFlags = 0x140B
/// BF 06 00 00 16 00 00 CC         ClubType = CC
/// BF 06 00 00 25 00 00 00         OperationFlags = 0
/// BF 06 00 00 07 00 54 48         ProcessFlags = 0x5448
/// BF 06 00 00 02 00 00 03         param 0x02 = 3
/// BF 06 00 00 14 00 00 02         param 0x14 = 2
/// BF 08 00 00 27 00 05 78 00 00   OutdoorMinimumTrackPercentage = 30.0
/// BF 08 00 00 28 00 01 40 00 00   IndoorModelDragFraction = 1.0
/// BF 06 00 00 07 00 54 48         ProcessFlags = 0x5448
/// A5 02 00 MM                     mode MM
/// B0 01 00
/// BF 08 00 00 28 00 01 40 00 00   IndoorModelDragFraction = 1.0
/// ```
///
/// [`mode_change`](Self::mode_change) sends the mode block alone:
///
/// ```text
/// BF 06 00 00 08 00 00 01         TriggerOptions = 1
/// B0 01 00
/// BF 06 00 00 16 00 00 CC         ClubType = CC
/// BF 08 00 00 20 00 00 00 00 00   param 0x20 = 0.0
/// BF 06 00 00 07 00 54 48         ProcessFlags = 0x5448
/// A5 02 00 MM                     mode MM
/// B0 01 00
/// BF 08 00 00 28 00 01 40 00 00   IndoorModelDragFraction = 1.0
/// BF 06 00 00 16 00 00 CC         ClubType = CC
/// BF 08 00 00 20 00 00 00 00 00   param 0x20 = 0.0
/// ```
///
/// Arm afterwards with [`ArmSequencer`]. The mode block expects a disarmed
/// device; [`BinaryClient`](crate::BinaryClient) disarms first when needed.
pub struct XiConfigSequencer {
    steps: Vec<(Command, XiReply)>,
    /// Index of the step waiting for its reply.
    current: usize,
    /// 0x94 that ended a step, not yet taken.
    refused: Option<Box<Envelope>>,
}

impl XiConfigSequencer {
    /// Connect-time block followed by the mode block.
    #[must_use]
    pub fn connect(settings: &XiSettings) -> (Self, Vec<Action>) {
        let club = settings.shot_type.param_value();
        let mut steps = vec![
            xi_int(XI_ALTITUDE_ID, XI_ALTITUDE),
            xi_mode(crate::protocol::config::MODE_FULL_OUTDOOR),
            (
                Command::ParamReadReq(ParamReadReq {
                    param_id: XI_PARAM_02_ID,
                }),
                XiReply::Param(XI_PARAM_02_ID),
            ),
            (
                Command::RadarCal(settings.radar_cal.clone()),
                XiReply::RadarCalEcho,
            ),
            xi_int(XI_TRIGGER_OPTIONS_ID, XI_TRIGGER_OPTIONS),
            xi_int(XI_PROCESS_FLAGS_ID, XI_PROCESS_FLAGS_INITIAL),
            xi_int(XI_CLUB_TYPE_ID, club),
            xi_int(XI_OPERATION_FLAGS_ID, XI_OPERATION_FLAGS),
            xi_int(XI_PROCESS_FLAGS_ID, XI_PROCESS_FLAGS),
            xi_int(XI_PARAM_02_ID, XI_PARAM_02),
            xi_int(XI_PARAM_14_ID, XI_PARAM_14),
            xi_float(XI_OUTDOOR_MIN_TRACK_PCT_ID, XI_OUTDOOR_MIN_TRACK_PCT),
            xi_float(XI_INDOOR_DRAG_FRACTION_ID, XI_INDOOR_DRAG_FRACTION),
            xi_int(XI_PROCESS_FLAGS_ID, XI_PROCESS_FLAGS),
            xi_mode(settings.mode),
            xi_gate(),
            xi_float(XI_INDOOR_DRAG_FRACTION_ID, XI_INDOOR_DRAG_FRACTION),
        ];
        steps.extend(Self::mode_steps(settings));
        Self::start(steps)
    }

    /// Mode block only, for a mode change on a connection that has had the
    /// connect-time block.
    #[must_use]
    pub fn mode_change(settings: &XiSettings) -> (Self, Vec<Action>) {
        Self::start(Self::mode_steps(settings))
    }

    fn mode_steps(settings: &XiSettings) -> Vec<(Command, XiReply)> {
        let club = settings.shot_type.param_value();
        vec![
            xi_int(XI_TRIGGER_OPTIONS_ID, XI_TRIGGER_OPTIONS),
            xi_gate(),
            xi_int(XI_CLUB_TYPE_ID, club),
            xi_float(XI_PARAM_20_ID, XI_PARAM_20),
            xi_int(XI_PROCESS_FLAGS_ID, XI_PROCESS_FLAGS),
            xi_mode(settings.mode),
            xi_gate(),
            xi_float(XI_INDOOR_DRAG_FRACTION_ID, XI_INDOOR_DRAG_FRACTION),
            xi_int(XI_CLUB_TYPE_ID, club),
            xi_float(XI_PARAM_20_ID, XI_PARAM_20),
        ]
    }

    fn start(steps: Vec<(Command, XiReply)>) -> (Self, Vec<Action>) {
        let seq = Self {
            steps,
            current: 0,
            refused: None,
        };
        let actions = seq.send_current();
        (seq, actions)
    }

    /// The latest 0x94 with which the device refused a step, once. The
    /// sequence carried on with the next step; log the frame's hex.
    #[must_use]
    pub fn take_refused(&mut self) -> Option<Envelope> {
        self.refused.take().map(|env| *env)
    }

    fn send_current(&self) -> Vec<Action> {
        self.steps
            .get(self.current)
            .map(|(cmd, _)| vec![Action::Send(cmd.clone(), BusAddr::Avr)])
            .unwrap_or_default()
    }
}

impl Sequence for XiConfigSequencer {
    fn feed(&mut self, env: &Envelope) -> Vec<Action> {
        let Some((cmd, reply)) = self.steps.get(self.current) else {
            return vec![];
        };
        if env.src != BusAddr::Avr {
            return vec![];
        }
        let step_type = cmd.encode(BusAddr::Avr).type_id & 0x7F;
        let done = match (*reply, &env.message) {
            (_, Message::ConfigNack(a)) if a.acked_cmd == step_type => {
                self.refused = Some(Box::new(env.clone()));
                true
            }
            (XiReply::Ack, Message::ConfigAck(a)) => a.acked_cmd == step_type,
            (XiReply::ModeEcho, Message::ModeSet(_))
            | (XiReply::RadarCalEcho, Message::RadarCal(_)) => true,
            (XiReply::Param(id), Message::ParamValue(p)) => p.param_id == id,
            _ => false,
        };
        if !done {
            return vec![];
        }
        self.current += 1;
        self.send_current()
    }

    fn is_complete(&self) -> bool {
        self.current >= self.steps.len()
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
        seq.deadline = Some(Instant::now());
        assert_eq!(sent(&seq.check_timeout()), [(BusAddr::Dsp, 0x67)]);
        // Only once.
        assert!(seq.check_timeout().is_empty());
    }

    /// DSP device info naming the model `text`.
    fn dsp_dev_info(text: &[u8; 14]) -> Envelope {
        let mut p = vec![0u8; 99];
        p[0] = 0x62;
        p[83..97].copy_from_slice(text);
        env(BusAddr::Dsp, 0xE7, p)
    }

    /// Run a DSP sync up to and including the config query's reply (0xA0);
    /// returns what that reply produced.
    fn dsp_sync_to_config(seq: &mut DspSequencer, model: &[u8; 14]) -> Vec<Action> {
        assert_eq!(sent(&seq.feed(&dsp_status())), [(BusAddr::Dsp, 0x48)]);
        let nack = env(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
        assert_eq!(sent(&seq.feed(&nack)), [(BusAddr::Dsp, 0x67)]);
        assert_eq!(
            sent(&seq.feed(&dsp_dev_info(model))),
            [(BusAddr::Dsp, 0xFD)]
        );
        let prod = |sub| {
            let mut p = vec![0u8; 34];
            p[0] = 0x21;
            p[1] = sub;
            env(BusAddr::Dsp, 0xFD, p)
        };
        assert_eq!(sent(&seq.feed(&prod(0))), [(BusAddr::Dsp, 0xFD)]);
        assert_eq!(sent(&seq.feed(&prod(8))), [(BusAddr::Dsp, 0xFD)]);
        assert_eq!(sent(&seq.feed(&prod(9))), [(BusAddr::Dsp, 0x21)]);
        let mut config = vec![0u8; 69];
        config[0] = 0x44;
        seq.feed(&env(BusAddr::Dsp, 0xA0, config))
    }

    /// On the Xi the DSP sync ends with the power-mode write (param 0x01 =
    /// 1, Normal) to the DSP, sent after the config query's reply and
    /// completed by the DSP's 0x95.
    #[test]
    fn xi_dsp_sync_wakes_the_dsp() {
        let (mut seq, _) = DspSequencer::new();
        let actions = dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
        assert_eq!(actions.len(), 1);
        assert_eq!(
            wire(&actions[0]),
            (BusAddr::Dsp, 0xBF, hex("06000001000001"))
        );
        assert!(!seq.is_complete());
        // The power-mode texts, and an AVR ack, do not complete it.
        for other in [
            env(BusAddr::Dsp, 0xE3, b"  REQ Power Mode : 1 (4) \0".to_vec()),
            env(BusAddr::Dsp, 0xE3, b"  NEW Power Mode : 1 \0".to_vec()),
            env(BusAddr::Avr, 0x95, hex("02303f")),
            env(BusAddr::Dsp, 0x95, hex("024030")),
        ] {
            assert!(seq.feed(&other).is_empty());
            assert!(!seq.is_complete());
        }
        assert!(seq.feed(&env(BusAddr::Dsp, 0x95, hex("02403f"))).is_empty());
        assert!(seq.is_complete());
        assert!(seq.take_refused().is_none());
        let sync = seq.into_result();
        assert_eq!(sync.model, DeviceModel::Xi);
        assert!(!sync.power_mode_unanswered);
    }

    /// A 0x94 refusing the power-mode write ends the DSP sync too, and is
    /// kept for the caller once.
    #[test]
    fn xi_dsp_power_mode_nack_completes() {
        let (mut seq, _) = DspSequencer::new();
        dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
        assert!(seq.feed(&env(BusAddr::Dsp, 0x94, hex("02403f"))).is_empty());
        assert!(seq.is_complete());
        let refused = seq.take_refused().expect("refusal kept");
        assert_eq!((refused.src, refused.type_id), (BusAddr::Dsp, 0x94));
        assert_eq!(refused.raw, hex("02403f"));
        assert!(seq.take_refused().is_none());
    }

    /// With no reply to the power-mode write within [`DSP_QUERY_TIMEOUT`]
    /// the DSP sync completes without one, flagged in the result.
    #[test]
    fn xi_dsp_power_mode_timeout_completes() {
        let (mut seq, _) = DspSequencer::new();
        dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
        assert!(seq.check_timeout().is_empty());
        assert!(!seq.is_complete());
        seq.deadline = Some(Instant::now());
        assert!(seq.check_timeout().is_empty());
        assert!(seq.is_complete());
        assert!(seq.take_refused().is_none());
        let sync = seq.into_result();
        assert!(sync.power_mode_unanswered);
        assert_eq!(sync.model, DeviceModel::Xi);
    }

    /// A Mevo's DSP sync ends on the config query's reply, with nothing
    /// sent after it.
    #[test]
    fn mevo_dsp_sync_sends_no_power_mode() {
        let (mut seq, _) = DspSequencer::new();
        let actions = dsp_sync_to_config(&mut seq, b"2.10 FS M2    ");
        assert!(actions.is_empty());
        assert!(seq.is_complete());
        assert_eq!(seq.into_result().model, DeviceModel::Mevo);
    }

    /// A refused status poll (0x94 `[02 30 2A]`, a starting AVR) is sent
    /// again after `STATUS_REPOLL_INTERVAL`, once per refusal, until the AVR
    /// answers.
    #[test]
    fn avr_status_nack_repolls() {
        let (mut seq, actions) = AvrSequencer::new_for(DeviceModel::Xi);
        assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
        let nack = env(BusAddr::Avr, 0x94, hex("02302a"));
        let mut status = vec![0u8; 25];
        status[0] = 0x18;
        let status = env(BusAddr::Avr, 0xAA, status);

        for _ in 0..2 {
            assert!(seq.feed(&nack).is_empty());
            // Not before the interval.
            assert!(seq.check_cal_timeout().is_empty());
            let at = seq.repoll_at.expect("re-poll scheduled");
            let wait = at.saturating_duration_since(Instant::now());
            assert!(wait + Duration::from_millis(100) > STATUS_REPOLL_INTERVAL);
            seq.repoll_at = Some(Instant::now());
            assert_eq!(sent(&seq.check_cal_timeout()), [(BusAddr::Avr, 0xAA)]);
            // Once per refusal.
            assert!(seq.check_cal_timeout().is_empty());
        }
        // A NACK of another command is not a refused poll.
        assert!(seq.feed(&env(BusAddr::Avr, 0x94, hex("023023"))).is_empty());
        assert!(seq.repoll_at.is_none());
        // The AVR answers: the handshake goes on as normal.
        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0xAA)]);
        assert!(seq.feed(&nack).is_empty());
        seq.repoll_at = Some(Instant::now());
        assert_eq!(sent(&seq.check_cal_timeout()), [(BusAddr::Avr, 0xAA)]);
        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0x67)]);
        // Past the status polls a refused poll schedules nothing.
        assert!(seq.feed(&nack).is_empty());
        assert!(seq.repoll_at.is_none());
    }

    /// Without a refusal the AVR sync sends exactly what it always has.
    #[test]
    fn mevo_avr_sync_without_nack_is_unchanged() {
        let (mut seq, actions) = AvrSequencer::new();
        assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
        let mut status = vec![0u8; 25];
        status[0] = 0x18;
        let status = env(BusAddr::Avr, 0xAA, status);
        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0xAA)]);
        assert!(seq.check_cal_timeout().is_empty());
        assert_eq!(sent(&seq.feed(&status)), [(BusAddr::Avr, 0x67)]);
        assert!(seq.check_cal_timeout().is_empty());
        assert!(seq.repoll_at.is_none());
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
    #[allow(deprecated)]
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
    #[allow(deprecated)]
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

    fn hex(s: &str) -> Vec<u8> {
        let s: String = s.split_whitespace().collect();
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// The Xi's reply to one configuration frame, as the device sends it.
    fn xi_reply(type_id: u8, payload: &[u8]) -> Envelope {
        match type_id {
            0xBF => env(BusAddr::Avr, 0x95, hex("02303f")),
            0xB0 => env(BusAddr::Avr, 0x95, hex("023030")),
            0xA5 | 0xA4 => env(BusAddr::Avr, type_id, payload.to_vec()),
            0xBE => env(BusAddr::Avr, 0xBF, hex("06000002000001")),
            _ => panic!("unexpected 0x{type_id:02X}"),
        }
    }

    /// Run an Xi configuration against the scripted device, with unrelated
    /// traffic (texts, a DSP ack, a mode ack) before every reply. Returns
    /// every frame sent as (type, payload hex).
    fn run_xi(seq: &mut XiConfigSequencer, actions: Vec<Action>) -> Vec<(u8, String)> {
        use std::fmt::Write as _;
        let noise = [
            env(BusAddr::Avr, 0xE3, b"  ARMED CANCELLED\0".to_vec()),
            env(BusAddr::Dsp, 0x95, hex("02403f")),
            env(BusAddr::Avr, 0xB1, hex("020000")),
        ];
        let mut wire = Vec::new();
        let mut pending = actions;
        while let Some(Action::Send(cmd, dest)) = pending.pop() {
            assert!(pending.is_empty(), "more than one frame in flight");
            assert_eq!(dest, BusAddr::Avr);
            let f = cmd.encode(dest);
            wire.push((
                f.type_id,
                f.payload.iter().fold(String::new(), |mut s, b| {
                    let _ = write!(s, "{b:02x}");
                    s
                }),
            ));
            assert!(wire.len() < 64, "config sequence did not finish");
            for n in &noise {
                assert!(seq.feed(n).is_empty());
            }
            assert!(!seq.is_complete());
            pending = seq.feed(&xi_reply(f.type_id, &f.payload));
        }
        assert!(seq.is_complete());
        wire
    }

    fn frames(lines: &[&str]) -> Vec<(u8, String)> {
        lines
            .iter()
            .map(|l| {
                let (t, p) = l.split_once(' ').unwrap();
                (u8::from_str_radix(t, 16).unwrap(), p.replace(' ', ""))
            })
            .collect()
    }

    /// FS Golf's connect-time block for a session in `mode` with club type
    /// `club`, radar cal 2438 mm / 25 mm.
    fn fs_golf_connect(mode: &str, club: &str) -> Vec<(u8, String)> {
        let club = format!("BF 06000016 0000{club}");
        let mode = format!("A5 0200{mode}");
        frames(&[
            "BF 06000001 000000",
            "A5 020008",
            "BE 03000002",
            "A4 06 0986 00 19 0000",
            "BF 06000008 000001",
            "BF 06000007 00140b",
            &club,
            "BF 06000025 000000",
            "BF 06000007 005448",
            "BF 06000002 000003",
            "BF 06000014 000002",
            "BF 08000027 0005780000",
            "BF 08000028 0001400000",
            "BF 06000007 005448",
            &mode,
            "B0 0100",
            "BF 08000028 0001400000",
        ])
    }

    /// FS Golf's mode block for `mode` and club type `club`.
    fn fs_golf_mode(mode: &str, club: &str) -> Vec<(u8, String)> {
        let club = format!("BF 06000016 0000{club}");
        let mode = format!("A5 0200{mode}");
        frames(&[
            "BF 06000008 000001",
            "B0 0100",
            &club,
            "BF 08000020 0000000000",
            "BF 06000007 005448",
            &mode,
            "B0 0100",
            "BF 08000028 0001400000",
            &club,
            "BF 08000020 0000000000",
        ])
    }

    fn xi(mode: u8, shot_type: XiShotType) -> XiSettings {
        XiSettings {
            mode,
            shot_type,
            ..XiSettings::default()
        }
    }

    #[test]
    fn xi_settings_default_is_fs_golf_full_swing() {
        let s = XiSettings::default();
        assert_eq!(s.mode, 9);
        assert_eq!(s.shot_type, XiShotType::FullSwing);
        assert_eq!(s.radar_cal.range_mm, 2438);
        assert_eq!(s.radar_cal.height_mm, 25);
        assert_eq!(XiShotType::FullSwing.param_value(), 1);
        assert_eq!(XiShotType::Pitching.param_value(), 2);
    }

    #[test]
    fn xi_connect_full_swing_matches_fs_golf() {
        let (mut seq, actions) = XiConfigSequencer::connect(&XiSettings::default());
        let mut expected = fs_golf_connect("09", "01");
        expected.extend(fs_golf_mode("09", "01"));
        assert_eq!(run_xi(&mut seq, actions), expected);
    }

    #[test]
    fn xi_connect_pitching_matches_fs_golf() {
        use crate::protocol::config::MODE_LIMITED_FLIGHT;
        let settings = xi(MODE_LIMITED_FLIGHT, XiShotType::Pitching);
        let (mut seq, actions) = XiConfigSequencer::connect(&settings);
        let mut expected = fs_golf_connect("09", "02");
        expected.extend(fs_golf_mode("09", "02"));
        assert_eq!(run_xi(&mut seq, actions), expected);
    }

    #[test]
    fn xi_connect_full_outdoor_changes_only_the_mode() {
        use crate::protocol::config::MODE_FULL_OUTDOOR;
        let settings = xi(MODE_FULL_OUTDOOR, XiShotType::FullSwing);
        let (mut seq, actions) = XiConfigSequencer::connect(&settings);
        let mut expected = fs_golf_connect("08", "01");
        expected.extend(fs_golf_mode("08", "01"));
        assert_eq!(run_xi(&mut seq, actions), expected);
    }

    #[test]
    fn xi_mode_change_matches_fs_golf() {
        for (shot_type, club) in [(XiShotType::FullSwing, "01"), (XiShotType::Pitching, "02")] {
            let (mut seq, actions) = XiConfigSequencer::mode_change(&xi(9, shot_type));
            assert_eq!(run_xi(&mut seq, actions), fs_golf_mode("09", club));
        }
    }

    #[test]
    fn xi_radar_cal_comes_from_settings() {
        let settings = XiSettings {
            radar_cal: RadarCal {
                range_mm: 3048,
                height_mm: 0,
            },
            ..XiSettings::default()
        };
        let (mut seq, actions) = XiConfigSequencer::connect(&settings);
        let wire = run_xi(&mut seq, actions);
        let cal: Vec<&(u8, String)> = wire.iter().filter(|(t, _)| *t == 0xA4).collect();
        assert_eq!(cal, [&(0xA4, "060be800000000".to_string())]);
        let (mut seq, actions) = XiConfigSequencer::mode_change(&settings);
        assert!(run_xi(&mut seq, actions).iter().all(|(t, _)| *t != 0xA4));
    }

    #[test]
    fn xi_config_waits_for_the_matching_reply() {
        let (mut seq, _) = XiConfigSequencer::mode_change(&XiSettings::default());
        // Step 1 is a BF write: a B0 ack does not complete it.
        assert!(seq.feed(&env(BusAddr::Avr, 0x95, hex("023030"))).is_empty());
        let next = seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f")));
        assert_eq!(sent(&next), [(BusAddr::Avr, 0xB0)]);
        // Step 2 is B0: a BF ack does not complete it, a rejection does.
        assert!(seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f"))).is_empty());
        let next = seq.feed(&env(BusAddr::Avr, 0x94, hex("023030")));
        assert_eq!(sent(&next), [(BusAddr::Avr, 0xBF)]);
        assert!(!seq.is_complete());
    }

    /// For each step type, a 0x94 refusing that type's first frame ends
    /// the step: the sequence sends exactly the frames it sends without the
    /// refusal, finishes, and keeps the 0x94 for the caller.
    #[test]
    fn xi_config_nack_continues_every_step_type() {
        let mut expected = fs_golf_connect("09", "01");
        expected.extend(fs_golf_mode("09", "01"));
        for refused_type in [0xA5, 0xA4, 0xBE, 0xBF, 0xB0] {
            let (mut seq, mut pending) = XiConfigSequencer::connect(&XiSettings::default());
            let mut wire = Vec::new();
            let mut refusals = Vec::new();
            let mut armed_nack = true;
            while let Some(Action::Send(cmd, dest)) = pending.pop() {
                assert!(pending.is_empty());
                let f = cmd.encode(dest);
                wire.push((f.type_id, hex_of(&f.payload)));
                assert!(wire.len() < 64, "0x{refused_type:02X}: did not finish");
                let reply = if armed_nack && f.type_id == refused_type {
                    armed_nack = false;
                    env(BusAddr::Avr, 0x94, vec![0x02, 0x30, refused_type & 0x7F])
                } else {
                    xi_reply(f.type_id, &f.payload)
                };
                pending = seq.feed(&reply);
                refusals.extend(seq.take_refused());
            }
            assert!(seq.is_complete(), "0x{refused_type:02X}");
            assert_eq!(wire, expected, "0x{refused_type:02X}");
            assert_eq!(refusals.len(), 1, "0x{refused_type:02X}");
            assert_eq!(refusals[0].type_id, 0x94);
            assert_eq!(refusals[0].raw, [0x02, 0x30, refused_type & 0x7F]);
        }
    }

    /// A 0x94 for a different command type does not end a step.
    #[test]
    fn xi_config_ignores_a_nack_of_another_type() {
        let (mut seq, _) = XiConfigSequencer::connect(&XiSettings::default());
        // Step 1 is the BF Altitude write.
        for other in ["023030", "023025", "023023", "02302a"] {
            assert!(seq.feed(&env(BusAddr::Avr, 0x94, hex(other))).is_empty());
        }
        // A DSP refusal of a BF does not end an AVR step.
        assert!(seq.feed(&env(BusAddr::Dsp, 0x94, hex("02403f"))).is_empty());
        assert!(seq.take_refused().is_none());
        let next = seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f")));
        assert_eq!(sent(&next), [(BusAddr::Avr, 0xA5)]);
    }

    fn hex_of(b: &[u8]) -> String {
        use std::fmt::Write as _;
        b.iter().fold(String::new(), |mut s, x| {
            let _ = write!(s, "{x:02x}");
            s
        })
    }

    #[test]
    fn xi_param_floats_encode_as_fs_golf_writes_them() {
        use crate::codec::{read_float40, write_float40};
        for (value, wire) in [
            (XI_OUTDOOR_MIN_TRACK_PCT, hex("0005780000")),
            (XI_INDOOR_DRAG_FRACTION, hex("0001400000")),
            (XI_PARAM_20, hex("0000000000")),
        ] {
            let mut buf = Vec::new();
            write_float40(&mut buf, value);
            assert_eq!(buf, wire);
            assert!((read_float40(&wire, 0).unwrap() - value).abs() < 1e-12);
        }
        assert!((read_float40(&hex("0005780000"), 0).unwrap() - 30.0).abs() < 1e-12);
    }

    /// Encode an action as (dest, type, payload).
    fn wire(action: &Action) -> (BusAddr, u8, Vec<u8>) {
        let Action::Send(cmd, dest) = action;
        let f = cmd.encode(*dest);
        (f.dest, f.type_id, f.payload)
    }

    #[test]
    fn xi_flight_results_are_acked() {
        let e8 = env(BusAddr::Avr, 0xE8, vec![0u8; 94]);
        let d4 = env(BusAddr::Avr, 0xD4, vec![0u8; 157]);
        let ack = result_ack(DeviceModel::Xi, &e8).expect("E8 ack");
        assert_eq!(wire(&ack), (BusAddr::Avr, 0x95, vec![0x02, 0x10, 0x68]));
        let ack = result_ack(DeviceModel::Xi, &d4).expect("D4 ack");
        assert_eq!(wire(&ack), (BusAddr::Avr, 0x95, vec![0x02, 0x10, 0x54]));
        let Action::Send(cmd, dest) = &ack;
        assert_eq!(cmd.debug_hex(*dest), "APP→AVR 0x95 3B | 021054");
    }

    #[test]
    fn only_xi_flight_results_are_acked() {
        let e8 = env(BusAddr::Avr, 0xE8, vec![0u8; 94]);
        let d4 = env(BusAddr::Avr, 0xD4, vec![0u8; 157]);
        // The Mevo+ and Gen2 get no result acks.
        assert!(result_ack(DeviceModel::Mevo, &e8).is_none());
        assert!(result_ack(DeviceModel::Mevo, &d4).is_none());
        // Other results and acks are not acked on the Xi.
        for (type_id, len) in [(0xED, 167), (0xEF, 138), (0xD9, 152), (0x95, 3)] {
            let other = env(BusAddr::Avr, type_id, vec![0x02; len]);
            assert!(
                result_ack(DeviceModel::Xi, &other).is_none(),
                "0x{type_id:02X}"
            );
        }
        let text = env(BusAddr::Avr, 0xE5, b"PROCESSED\0".to_vec());
        assert!(result_ack(DeviceModel::Xi, &text).is_none());
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
