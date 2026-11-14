//! What the handshake, configuration and shot sequences take and produce:
//! the device settings ([`MevoSettings`], [`AvrSettings`], [`XiSettings`]),
//! the handshake's results ([`DspSync`], [`AvrSync`], [`PiSync`]) and the
//! shot data ([`ShotDatum`], [`ShotData`]).
//!
//! [`BinaryClient`](crate::BinaryClient) runs the sequences. Each one is a
//! crate-private state machine: `feed()` takes a received [`Envelope`] and
//! the time it is handled and returns the commands to send, and
//! `check_timers(now)` runs the timed steps due at `now`, so a sequence
//! reads no clock.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::addr::BusAddr;
use crate::conn::{BinaryConnection, ConnError, Envelope};
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
use crate::protocol::status::{AvrStatus, ConfigAck, DspStatus, StatusPoll};
use crate::protocol::{
    Command, Message, TYPE_DSP_QUERY, TYPE_FLIGHT_RESULT, TYPE_FLIGHT_RESULT_V1, TYPE_PARAM_VALUE,
    TYPE_STATUS,
};

/// Per-exchange timeout.
pub(crate) const TIMEOUT: Duration = Duration::from_secs(2);

/// How long the DSP sequencer waits for the DSP query response (0xC8) before
/// carrying on without it. The Xi's DSP may not answer it, so the handshake
/// does not depend on one.
pub(crate) const DSP_QUERY_TIMEOUT: Duration = TIMEOUT;

/// How long the AVR sequencer waits before re-sending a status poll the
/// AVR refused (0x94). The Xi's AVR refuses status polls for several
/// seconds after power-on.
pub(crate) const STATUS_REPOLL_INTERVAL: Duration = Duration::from_secs(1);

// ===========================================================================
// Action, Sequence trait, infrastructure
// ===========================================================================

/// An action returned by a sequencer's `feed()` or `new()`.
#[derive(Debug, Clone)]
pub(crate) enum Action {
    /// Send a command to a bus address.
    Send(Command, BusAddr),
}

/// Trait implemented by all sequencers.
///
/// `feed()` accepts a received message and returns actions to send.
/// `is_complete()` indicates the sequence is done. Type-specific `finish()`
/// methods (not in the trait) extract the result.
pub(crate) trait Sequence {
    /// Process a message received at `now`. Returns actions to send (may be
    /// empty). A sequencer that times a step starts the timer from `now`.
    fn feed(&mut self, env: &Envelope, now: Instant) -> Vec<Action>;

    /// Is the sequence complete?
    fn is_complete(&self) -> bool;
}

/// Send an action on a connection.
pub(crate) fn send_action<S: Read + Write>(
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
pub(crate) fn result_ack(model: DeviceModel, env: &Envelope) -> Option<Action> {
    let is_result = matches!(env.type_id, TYPE_FLIGHT_RESULT_V1 | TYPE_FLIGHT_RESULT);
    (model == DeviceModel::Xi && env.src == BusAddr::Avr && is_result).then(|| {
        Action::Send(
            Command::ConfigAck(ConfigAck::for_result(env.type_id)),
            BusAddr::Avr,
        )
    })
}

/// Commands for a keepalive poll on the given model. The PI is polled only
/// on models that have one.
#[must_use]
pub(crate) fn keepalive_polls(model: DeviceModel) -> Vec<Action> {
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

/// Like `should_skip` but also skips ModeAck .
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
#[non_exhaustive]
pub struct DspSync {
    pub status: DspStatus,
    /// DSP hardware info (0xC8). `None` when the DSP query is rejected or
    /// goes unanswered within 2 seconds.
    pub hw_info: Option<DspQueryResp>,
    pub dev_info: DevInfoResp,
    pub prod_info: [ProdInfoResp; 3],
    pub config: ConfigResp,
    /// Model family, detected from the DSP device info.
    pub model: DeviceModel,
    /// Xi only: `true` when the DSP did not answer the power-mode write
    /// within 2 seconds and the sync carried on without a
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
/// within [`DSP_QUERY_TIMEOUT`] (see [`check_timers()`](Self::check_timers)),
/// moves on to DevInfoReq with no hardware info.
///
/// On the Xi the sequence ends with a power-mode write to the DSP, BF
/// `[06 00 00 01 00 00 01]` (param 0x01 = 1, Normal), which wakes a DSP left
/// in Standby. It completes on the DSP's 0x95, or on a 0x94 refusing it;
/// the refusal is kept for [`take_refused()`](Self::take_refused). With no
/// reply within [`DSP_QUERY_TIMEOUT`] it completes too, and
/// [`DspSync::power_mode_unanswered`] is set. The Mevo+ and Gen2 are not
/// sent it.
pub(crate) struct DspSequencer {
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
    /// (check [`is_complete()`](Sequence::is_complete) afterward).
    /// Call this periodically in the event loop.
    pub fn check_timers(&mut self, now: Instant) -> Vec<Action> {
        let Some(deadline) = self.deadline else {
            return vec![];
        };
        if now < deadline {
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
    fn feed(&mut self, env: &Envelope, now: Instant) -> Vec<Action> {
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
                    self.deadline = Some(now + DSP_QUERY_TIMEOUT);
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
                        self.deadline = Some(now + DSP_QUERY_TIMEOUT);
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
#[non_exhaustive]
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
/// Xi sequence ([`new(DeviceModel::Xi)`](Self::new)): the same up to
/// ConfigQuery, then IF cal (0xD0, optional) and done. The factory cal
/// request, AvrConfigQuery (which the Xi rejects), param 0x64 and TimeSync
/// are not sent.
///
/// An AVR that is still starting up refuses the status poll with a 0x94
/// (`[02 30 2A]`); the Xi does so for several seconds after power-on. The
/// poll is then sent again [`STATUS_REPOLL_INTERVAL`] later, from
/// [`check_timers()`](Self::check_timers), once per refusal,
/// until the AVR answers. The caller's overall deadline bounds the wait.
pub(crate) struct AvrSequencer {
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
    /// AVR sync for the given model family.
    #[must_use]
    pub fn new(model: DeviceModel) -> (Self, Vec<Action>) {
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
    pub fn check_timers(&mut self, now: Instant) -> Vec<Action> {
        if let Some(at) = self.repoll_at
            && now >= at
        {
            self.repoll_at = None;
            return vec![Action::Send(
                Command::StatusPoll(StatusPoll { pi_mode: false }),
                BusAddr::Avr,
            )];
        }
        if let Some(deadline) = self.cal_deadline
            && now >= deadline
        {
            self.cal_deadline = None;
            return self.advance_past_cal(now);
        }
        vec![]
    }

    /// Skip past the current optional cal step.
    fn advance_past_cal(&mut self, now: Instant) -> Vec<Action> {
        match self.step {
            AvrStep::WaitFactoryCal => {
                self.step = AvrStep::WaitIfCal;
                self.cal_deadline = Some(now + TIMEOUT);
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
    fn feed(&mut self, env: &Envelope, now: Instant) -> Vec<Action> {
        if matches!(self.step, AvrStep::WaitStatus1 | AvrStep::WaitStatus2)
            && env.src == BusAddr::Avr
            && let Message::ConfigNack(ref nack) = env.message
            && nack.acked_cmd == TYPE_STATUS & 0x7F
        {
            self.repoll_at
                .get_or_insert_with(|| now + STATUS_REPOLL_INTERVAL);
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
                    self.cal_deadline = Some(now + TIMEOUT);
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
                    self.cal_deadline = Some(now + TIMEOUT);
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
#[non_exhaustive]
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
pub(crate) struct PiSequencer {
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
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
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
///
/// Build a Mevo+ / Mevo Gen2 configuration with [`MevoSettings`] (full
/// settings) or [`AvrSettings::mode_only`] (a mode change).
#[derive(Debug, Clone)]
pub struct AvrSettings {
    /// Detection mode commsIndex (see `config::MODE_*` constants).
    pub mode: u8,
    /// BF parameter writes (ball type, tee height, track%, etc.).
    pub params: Vec<ParamValue>,
    /// Radar calibration values. Optional: a mode change sends none.
    pub radar_cal: Option<RadarCal>,
}

impl AvrSettings {
    /// The detection mode alone, for a mode change on a Mevo+ or Mevo Gen2
    /// that already has its parameters and radar calibration.
    /// [`BinaryClient::configure_avr`](crate::BinaryClient::configure_avr)
    /// sends B0 `[01 00]`, A5 `[02 00 mode]` and B0 `[01 00]`.
    ///
    /// After a full parameter set (see [`MevoSettings`]) the Mevo+ can
    /// refuse the arm (0x94) for up to about 30 seconds before it accepts
    /// it. After the mode alone it re-arms much sooner, though a few
    /// refusals can still occur; the arm retries them.
    #[must_use]
    pub fn mode_only(mode: u8) -> Self {
        Self {
            mode,
            params: vec![],
            radar_cal: None,
        }
    }
}

/// Param 0x06: ball type (INT24).
const MEVO_BALL_TYPE_ID: u8 = 0x06;
/// Param 0x0F: written with the minimum track percentage, as a fraction
/// (FLOAT40).
const MEVO_TRACK_FRACTION_ID: u8 = 0x0F;
/// Param 0x26: tee height in meters (FLOAT40).
const MEVO_TEE_HEIGHT_ID: u8 = 0x26;

/// Full configuration for a Mevo+ or Mevo Gen2: detection mode, ball type,
/// minimum track percentage, tee height and radar calibration.
///
/// Build one with [`MevoSettings::new`]; the fields stay public for
/// reading and adjusting.
///
/// Converts into [`AvrSettings`].
/// [`BinaryClient::configure_avr`](crate::BinaryClient::configure_avr)
/// then sends B0 `[01 00]`, then each parameter write in this order, the
/// mode and the radar calibration, each followed by B0 `[01 00]`:
///
/// | Param | Field | Encoding |
/// | ----- | ----- | -------- |
/// | 0x06 | `ball_type` | INT24 |
/// | 0x0F | `track_fraction` | FLOAT40 |
/// | 0x26 | `tee_height_m` | FLOAT40 |
///
/// For a mode change, send [`AvrSettings::mode_only`] instead: the device
/// re-arms sooner after the mode alone.
///
/// ```
/// use ironsight::protocol::config::{MODE_LIMITED_FLIGHT, RadarCal};
/// use ironsight::{AvrSettings, MevoSettings};
///
/// let settings: AvrSettings = MevoSettings::new(
///     MODE_LIMITED_FLIGHT,
///     1,
///     0.8,
///     0.0381,
///     RadarCal { range_mm: 2438, height_mm: 0 },
/// )
/// .into();
/// assert_eq!(settings.params.len(), 3);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub struct MevoSettings {
    /// Detection mode commsIndex (see `config::MODE_*` constants).
    pub mode: u8,
    /// Ball type, param 0x06: 0 = standard ball, 1 = RCT ball.
    pub ball_type: u8,
    /// Minimum outdoor track percentage as a fraction (0.8 = 80 %),
    /// written to param 0x0F. The device's exact use of param 0x0F is not
    /// yet confirmed; ironsight writes this value there unchanged.
    pub track_fraction: f64,
    /// Tee height in meters, param 0x26.
    pub tee_height_m: f64,
    /// Distance from the device to the ball and height of the hitting
    /// surface (0xA4).
    pub radar_cal: RadarCal,
}

impl MevoSettings {
    /// A full configuration: detection mode (`config::MODE_*`), ball type
    /// (0 = standard, 1 = RCT), minimum track percentage as a fraction,
    /// tee height in meters and radar calibration.
    #[must_use]
    pub fn new(
        mode: u8,
        ball_type: u8,
        track_fraction: f64,
        tee_height_m: f64,
        radar_cal: RadarCal,
    ) -> Self {
        Self {
            mode,
            ball_type,
            track_fraction,
            tee_height_m,
            radar_cal,
        }
    }

    /// The parameter writes, in the order they are sent.
    #[must_use]
    pub fn params(&self) -> Vec<ParamValue> {
        vec![
            ParamValue {
                param_id: MEVO_BALL_TYPE_ID,
                value: ParamData::Int24(i32::from(self.ball_type)),
            },
            ParamValue {
                param_id: MEVO_TRACK_FRACTION_ID,
                value: ParamData::Float40(self.track_fraction),
            },
            ParamValue {
                param_id: MEVO_TEE_HEIGHT_ID,
                value: ParamData::Float40(self.tee_height_m),
            },
        ]
    }
}

impl From<MevoSettings> for AvrSettings {
    fn from(settings: MevoSettings) -> Self {
        Self {
            mode: settings.mode,
            params: settings.params(),
            radar_cal: Some(settings.radar_cal),
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
pub(crate) struct AvrConfigSequencer {
    step: AvrConfigStep,
    settings: AvrSettings,
}

impl AvrConfigSequencer {
    #[must_use]
    pub fn new(settings: AvrSettings) -> (Self, Vec<Action>) {
        // Send a leading B0[01 00] config-gate before any params: after a
        // disarm the device needs it to enter config mode before it accepts
        // the arm.
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
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
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

/// Shot type of an Xi configuration, written as param 0x16 (club type).
///
/// Full swing is for full shots; pitching is for chips and pitches and
/// always runs in detection mode 9
/// ([`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT));
/// see [`XiSettings::pitching`].
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
    /// Value written to param 0x16 (club type).
    #[must_use]
    pub fn param_value(self) -> i32 {
        match self {
            Self::FullSwing => 1,
            Self::Pitching => 2,
        }
    }
}

/// Configuration for the Xi: detection mode, shot type and radar
/// calibration.
///
/// [`BinaryClient::configure_xi`](crate::BinaryClient::configure_xi) sends
/// the Xi its full parameter set, radar calibration and mode on the
/// connection's first configuration (the connect-time block), then the mode
/// alone on a change (the mode block).
///
/// Build one with [`full_swing`](Self::full_swing) or
/// [`pitching`](Self::pitching).
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub struct XiSettings {
    /// Detection mode commsIndex:
    /// [`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT)
    /// (9) for nets and simulator screens, or
    /// [`MODE_FULL_OUTDOOR`](crate::protocol::config::MODE_FULL_OUTDOOR) (8).
    pub mode: u8,
    /// Full swing or pitching (param 0x16, club type).
    pub shot_type: XiShotType,
    /// Distance from the Xi to the ball and surface height (0xA4).
    pub radar_cal: RadarCal,
}

impl XiSettings {
    /// A full-swing session in detection mode `mode`:
    /// [`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT)
    /// (9) for nets and simulator screens, or
    /// [`MODE_FULL_OUTDOOR`](crate::protocol::config::MODE_FULL_OUTDOOR) (8)
    /// for full flight outdoors. Any other mode is sent as given.
    #[must_use]
    pub fn full_swing(mode: u8, radar_cal: RadarCal) -> Self {
        Self {
            mode,
            shot_type: XiShotType::FullSwing,
            radar_cal,
        }
    }

    /// A pitching session, for chips and pitches. Pitching always runs in
    /// limited-flight mode
    /// ([`MODE_LIMITED_FLIGHT`](crate::protocol::config::MODE_LIMITED_FLIGHT),
    /// 9).
    #[must_use]
    pub fn pitching(radar_cal: RadarCal) -> Self {
        Self {
            mode: crate::protocol::config::MODE_LIMITED_FLIGHT,
            shot_type: XiShotType::Pitching,
            radar_cal,
        }
    }
}

impl Default for XiSettings {
    /// Full swing in limited-flight mode with the radar calibration
    /// 2438 mm (8 ft) to the ball and a 25 mm (1 in) surface height.
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

// Xi parameters (0xBF writes) and the values ironsight writes.

/// Param 0x01, Altitude.
const XI_ALTITUDE_ID: u8 = 0x01;
/// Altitude: 0.
const XI_ALTITUDE: i32 = 0;
/// Param 0x02. The connect-time block reads it before writing it.
const XI_PARAM_02_ID: u8 = 0x02;
/// Param 0x02: 3.
const XI_PARAM_02: i32 = 3;
/// Param 0x07, processing flags (a bitfield).
const XI_PROCESS_FLAGS_ID: u8 = 0x07;
/// Processing flags, first write of the connect-time block: 0x140B
/// (bits 0, 1, 3, 10, 12).
const XI_PROCESS_FLAGS_INITIAL: i32 = 0x140B;
/// Processing flags, every later write: 0x5448 (bits 3, 6, 10, 12, 14).
const XI_PROCESS_FLAGS: i32 = 0x5448;
/// Param 0x08, trigger options.
const XI_TRIGGER_OPTIONS_ID: u8 = 0x08;
/// Trigger options: 1, the standard trigger.
const XI_TRIGGER_OPTIONS: i32 = 1;
/// Param 0x14.
const XI_PARAM_14_ID: u8 = 0x14;
/// Param 0x14: 2.
const XI_PARAM_14: i32 = 2;
/// Param 0x16, club type: the shot type, from [`XiShotType`].
const XI_CLUB_TYPE_ID: u8 = 0x16;
/// Param 0x20, written after every club type write.
const XI_PARAM_20_ID: u8 = 0x20;
/// Param 0x20: 0.0.
const XI_PARAM_20: f64 = 0.0;
/// Param 0x25, operation flags.
const XI_OPERATION_FLAGS_ID: u8 = 0x25;
/// Operation flags: 0, no flags set.
const XI_OPERATION_FLAGS: i32 = 0;
/// Param 0x27, minimum outdoor track percentage.
const XI_OUTDOOR_MIN_TRACK_PCT_ID: u8 = 0x27;
/// Minimum outdoor track percentage: 30 % (FLOAT40 `00 05 78 00 00`).
const XI_OUTDOOR_MIN_TRACK_PCT: f64 = 30.0;
/// Param 0x28, indoor model drag fraction.
const XI_INDOOR_DRAG_FRACTION_ID: u8 = 0x28;
/// Indoor model drag fraction: 1.0 (FLOAT40 `00 01 40 00 00`).
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
/// Sends the frames one at a time, each after the reply to the one
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
/// BF 06 00 00 01 00 00 00         altitude = 0
/// A5 02 00 08                     mode 8
/// BE 03 00 00 02                  read param 0x02
/// A4 06 RR RR 00 HH 00 00         radar calibration
/// BF 06 00 00 08 00 00 01         trigger options = 1
/// BF 06 00 00 07 00 14 0B         processing flags = 0x140B
/// BF 06 00 00 16 00 00 CC         club type = CC
/// BF 06 00 00 25 00 00 00         operation flags = 0
/// BF 06 00 00 07 00 54 48         processing flags = 0x5448
/// BF 06 00 00 02 00 00 03         param 0x02 = 3
/// BF 06 00 00 14 00 00 02         param 0x14 = 2
/// BF 08 00 00 27 00 05 78 00 00   min. outdoor track % = 30.0
/// BF 08 00 00 28 00 01 40 00 00   indoor drag fraction = 1.0
/// BF 06 00 00 07 00 54 48         processing flags = 0x5448
/// A5 02 00 MM                     mode MM
/// B0 01 00
/// BF 08 00 00 28 00 01 40 00 00   indoor drag fraction = 1.0
/// ```
///
/// [`mode_change`](Self::mode_change) sends the mode block alone:
///
/// ```text
/// BF 06 00 00 08 00 00 01         trigger options = 1
/// B0 01 00
/// BF 06 00 00 16 00 00 CC         club type = CC
/// BF 08 00 00 20 00 00 00 00 00   param 0x20 = 0.0
/// BF 06 00 00 07 00 54 48         processing flags = 0x5448
/// A5 02 00 MM                     mode MM
/// B0 01 00
/// BF 08 00 00 28 00 01 40 00 00   indoor drag fraction = 1.0
/// BF 06 00 00 16 00 00 CC         club type = CC
/// BF 08 00 00 20 00 00 00 00 00   param 0x20 = 0.0
/// ```
///
/// Arm afterward with [`ArmSequencer`]. The mode block expects a disarmed
/// device; [`BinaryClient`](crate::BinaryClient) disarms first when needed.
pub(crate) struct XiConfigSequencer {
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
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
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
///
/// Sends the camera config, reads it back, starts the camera and writes the
/// PI keepalive parameter, each after the reply to the one before. A 0x94
/// refusing the step's command ends the sequence at that step
/// ([`is_refused()`](Self::is_refused)); a step that gets no reply at all
/// waits for the caller's timeout. Either way
/// [`step_name()`](Self::step_name) names the step the sequence stopped at.
pub(crate) struct CameraConfigSequencer {
    step: CamConfigStep,
    /// The type of the command the current step waits on a reply to,
    /// without the reply bit (what a 0x94 carries as `acked_cmd`).
    sent_type: u8,
    fusion: bool,
    /// The device refused (0x94) the current step; nothing more is sent.
    stopped: bool,
    /// 0x94 that refused the current step, not yet taken.
    refused: Option<Box<Envelope>>,
}

impl CameraConfigSequencer {
    #[must_use]
    pub fn new(config: &CamConfig) -> (Self, Vec<Action>) {
        let command = Command::CamConfig(config.clone());
        let seq = Self {
            step: CamConfigStep::WaitConfigAck,
            sent_type: Self::type_of(&command),
            fusion: config.is_fusion(),
            stopped: false,
            refused: None,
        };
        (seq, vec![Action::Send(command, BusAddr::Pi)])
    }

    fn type_of(command: &Command) -> u8 {
        command.encode(BusAddr::Pi).type_id & 0x7F
    }

    /// Move to `step`, which waits on a reply to `command`.
    fn send(&mut self, step: CamConfigStep, command: Command) -> Vec<Action> {
        self.step = step;
        self.sent_type = Self::type_of(&command);
        vec![Action::Send(command, BusAddr::Pi)]
    }

    /// Whether the configuration is a Fusion mode
    /// ([`CamConfig::is_fusion`]).
    #[must_use]
    pub fn is_fusion(&self) -> bool {
        self.fusion
    }

    /// The step the sequence is waiting at (or stopped at), in words:
    /// `"camera config ack"`, `"camera config readback"`,
    /// `"camera start ack"`, `"camera parameter ack"` or `"done"`.
    #[must_use]
    pub fn step_name(&self) -> &'static str {
        match self.step {
            CamConfigStep::WaitConfigAck => "camera config ack",
            CamConfigStep::WaitReadback => "camera config readback",
            CamConfigStep::WaitCamStateAck => "camera start ack",
            CamConfigStep::WaitParamAck => "camera parameter ack",
            CamConfigStep::Done => "done",
        }
    }

    /// Whether the device refused a step with a 0x94. The sequence stopped
    /// there.
    #[must_use]
    pub fn is_refused(&self) -> bool {
        self.stopped
    }

    /// The 0x94 with which the device refused a step, once.
    #[must_use]
    pub fn take_refused(&mut self) -> Option<Envelope> {
        self.refused.take().map(|env| *env)
    }
}

impl Sequence for CameraConfigSequencer {
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
        if let Message::ConfigNack(nack) = &env.message
            && env.src == BusAddr::Pi
            && nack.acked_cmd == self.sent_type
            && !self.stopped
            && !self.is_complete()
        {
            self.refused = Some(Box::new(env.clone()));
            self.stopped = true;
            return vec![];
        }
        if self.stopped || should_skip_with_mode_ack(env, BusAddr::Pi) {
            return vec![];
        }
        match self.step {
            CamConfigStep::WaitConfigAck => {
                if let Message::ConfigAck(_) = env.message {
                    return self.send(
                        CamConfigStep::WaitReadback,
                        Command::CamConfigReq(CamConfigReq),
                    );
                }
                vec![]
            }
            CamConfigStep::WaitReadback => {
                if let Message::CamConfig(_) = env.message {
                    return self.send(
                        CamConfigStep::WaitCamStateAck,
                        Command::CamState(CamState { state: 0x01 }),
                    );
                }
                vec![]
            }
            CamConfigStep::WaitCamStateAck => {
                if let Message::ConfigAck(_) = env.message {
                    return self.send(
                        CamConfigStep::WaitParamAck,
                        Command::ParamValue(ParamValue {
                            param_id: 0x02,
                            value: crate::protocol::config::ParamData::Int24(10),
                        }),
                    );
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
pub(crate) struct DisarmSequencer {
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
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
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
/// Sequence:
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
/// (params + RadarCal). Mode-only configuration (no params, no RadarCal)
/// can still produce ConfigNack retries, but typically fewer.
/// The overall operation timeout (set by the caller) limits retries.
///
/// On a model without a camera Pi ([`new(DeviceModel::Xi)`](Self::new))
/// step 4 skips the PI poll and waits for the "ARMED" text alone.
pub(crate) struct ArmSequencer {
    step: ArmStep,
    retries: u16,
    /// Poll the PI after the arm ConfigAck and wait for its status.
    wait_pi: bool,
}

impl ArmSequencer {
    /// Arm the given model family.
    #[must_use]
    pub fn new(model: DeviceModel) -> (Self, Vec<Action>) {
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
}

impl Sequence for ArmSequencer {
    fn feed(&mut self, env: &Envelope, _now: Instant) -> Vec<Action> {
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
                    // accepts the arm command.
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
#[non_exhaustive]
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
/// in the accumulated [`ShotData`] of `BinaryEvent::ShotComplete`.
#[derive(Debug, Clone)]
#[non_exhaustive]
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
pub(crate) struct ShotSequencer {
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
    pub fn check_timers(&mut self, now: Instant) -> Vec<Action> {
        if !matches!(self.step, ShotStep::Draining) {
            return vec![];
        }
        if let Some(deadline) = self.drain_deadline
            && now >= deadline
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
    fn feed(&mut self, env: &Envelope, now: Instant) -> Vec<Action> {
        match self.step {
            ShotStep::Draining => {
                match &env.message {
                    Message::ShotText(st) if st.is_idle() => {
                        // After IDLE, go straight to requesting final results.
                        // No 0x21 ConfigQuery here: it only re-reads radar
                        // params we already have, Gen2 firmware does not take
                        // part in that exchange, and re-arming does not need
                        // it.
                        // ModeAck arrives naturally during the device's
                        // shot-completion flow and is absorbed passively.
                        self.step = ShotStep::WaitingForClubResult;
                        return vec![Action::Send(Command::ShotResultReq, BusAddr::Avr)];
                    }
                    Message::FlightResult(r) => {
                        self.data.flight = Some(r.clone());
                        self.pending = Some(ShotDatum::Flight(r.clone()));
                        if self.drain_deadline.is_none() {
                            self.drain_deadline = Some(now + DRAIN_TIMEOUT);
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

#[cfg(test)]
mod tests;
