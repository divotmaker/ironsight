//! Handshake sequence helpers and operational routines.
//!
//! Free functions that drive multi-step request/response exchanges over a
//! [`Connection`]. Two layers:
//!
//! 1. **Core helpers** — [`recv_msg`] (recv + skip 0xE3 Text) and [`send_recv`]
//!    (send + recv_msg). These handle the "Text messages can arrive at any time" rule.
//! 2. **Sequence functions** — one per SEQUENCE.md phase, plus operational helpers
//!    for keepalive, shot-ack, and re-arm.

use std::time::{Duration, Instant};

use crate::addr::BusAddr;
use crate::conn::{ConnError, Connection, Envelope};
use crate::protocol::camera::{CamConfig, CamConfigReq, CamState};
use crate::protocol::config::{
    AvrConfigCmd, AvrConfigResp, ConfigResp, ModeSet, ParamReadReq, ParamValue, RadarCal,
};
use crate::protocol::handshake::{
    CalDataReq, CalDataResp, CalParamReq, CalParamResp, DevInfoResp, DspQueryResp, NetConfigReq,
    ProdInfoReq, ProdInfoResp, TimeSync,
};
use crate::protocol::status::{AvrStatus, DspStatus, PiStatus, StatusPoll};
use crate::protocol::{Command, Message};

/// Per-exchange timeout (2s for all operations).
pub const TIMEOUT: Duration = Duration::from_secs(2);

// ---------------------------------------------------------------------------
// Internal: expect! macro
// ---------------------------------------------------------------------------

/// Destructure an [`Envelope`]'s message into a specific [`Message`] variant,
/// or return `ConnError::Protocol`.
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

// ---------------------------------------------------------------------------
// Core helpers
// ---------------------------------------------------------------------------

/// Receive the next message, silently skipping 0xE3 Text debug logs.
///
/// Uses deadline tracking so Text messages don't extend the timeout.
pub fn recv_msg(conn: &mut Connection, timeout: Duration) -> Result<Envelope, ConnError> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout { timeout });
        }
        let env = conn.recv_timeout(remaining)?;
        if matches!(env.message, Message::Text(_) | Message::DspDebug(_)) {
            continue;
        }
        return Ok(env);
    }
}

/// Send a command to `dest`, then receive the next non-Text response from that bus.
///
/// Messages from other bus sources (e.g. late PI responses during AVR config)
/// are silently consumed so they don't cause protocol mismatches.
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
            return Err(ConnError::Timeout { timeout });
        }
        let env = conn.recv_timeout(remaining)?;
        // Skip wrong-bus messages.
        if env.src != dest {
            continue;
        }
        // Skip: Text/DspDebug logs, unsolicited CamState/ModeAck/ConfigNack, Unknown.
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

/// Receive the next message matching `extract`, skipping Text, ConfigAck,
/// Unknown, and messages from unexpected buses.
///
/// Some commands (e.g. CalDataReq, CalParamReq) may produce an intermediate
/// ConfigAck before the actual response. This helper consumes those silently.
fn recv_skip_ack<T>(
    conn: &mut Connection,
    from: BusAddr,
    timeout: Duration,
    extract: impl Fn(Message) -> Option<T>,
) -> Result<T, ConnError> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout { timeout });
        }
        let env = conn.recv_timeout(remaining)?;
        if matches!(
            env.message,
            Message::Text(_) | Message::DspDebug(_) | Message::ConfigAck(_) | Message::ConfigNack(_) | Message::Unknown { .. }
        ) || env.src != from
        {
            continue;
        }
        return extract(env.message)
            .ok_or_else(|| ConnError::Protocol("unexpected message".into()));
    }
}

/// Receive messages until `pred` returns true, skipping everything else.
/// Returns `Ok(())` on match, `Err(Timeout)` if deadline expires.
fn drain_until(
    conn: &mut Connection,
    timeout: Duration,
    pred: impl Fn(&Message) -> bool,
) -> Result<(), ConnError> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout { timeout });
        }
        match conn.recv_timeout(remaining) {
            Ok(env) if pred(&env.message) => return Ok(()),
            Ok(_) => continue,
            Err(ConnError::Timeout { .. }) => return Err(ConnError::Timeout { timeout }),
            Err(e) => return Err(e),
        }
    }
}

// ---------------------------------------------------------------------------
// Phase 1 — DSP Sync (SEQUENCE.md §2.1)
// ---------------------------------------------------------------------------

/// Results from Phase 1 — DSP sync.
#[derive(Debug, Clone)]
pub struct DspSync {
    pub status: DspStatus,
    pub hw_info: DspQueryResp,
    pub dev_info: DevInfoResp,
    pub prod_info: [ProdInfoResp; 3],
    pub config: ConfigResp,
}

/// Phase 1: Query DSP for status, hardware info, device info, product info,
/// and radar configuration.
pub fn sync_dsp(conn: &mut Connection) -> Result<DspSync, ConnError> {
    let t = TIMEOUT;

    // StatusPoll → DSP
    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: false }),
        BusAddr::Dsp,
        t,
    )?;
    let status = expect!(env, Message::DspStatus)?;

    // DspQuery → DSP
    let env = send_recv(conn, &Command::DspQuery, BusAddr::Dsp, t)?;
    let hw_info = expect!(env, Message::DspQueryResp)?;

    // DevInfoReq → DSP
    let env = send_recv(conn, &Command::DevInfoReq, BusAddr::Dsp, t)?;
    let dev_info = expect!(env, Message::DevInfoResp)?;

    // ProdInfoReq ×3 (sub-queries 0x00, 0x08, 0x09)
    let mut prod_info_vec = Vec::with_capacity(3);
    for sub in [0x00, 0x08, 0x09] {
        let env = send_recv(
            conn,
            &Command::ProdInfoReq(ProdInfoReq { sub_query: sub }),
            BusAddr::Dsp,
            t,
        )?;
        prod_info_vec.push(expect!(env, Message::ProdInfoResp)?);
    }

    // ConfigQuery → DSP
    let env = send_recv(conn, &Command::ConfigQuery, BusAddr::Dsp, t)?;
    let config = expect!(env, Message::ConfigResp)?;

    Ok(DspSync {
        status,
        hw_info,
        dev_info,
        prod_info: [
            prod_info_vec.remove(0),
            prod_info_vec.remove(0),
            prod_info_vec.remove(0),
        ],
        config,
    })
}

// ---------------------------------------------------------------------------
// Phase 2 — AVR Sync (SEQUENCE.md §2.2)
// ---------------------------------------------------------------------------

/// Results from Phase 2 — AVR sync.
#[derive(Debug, Clone)]
pub struct AvrSync {
    pub status: AvrStatus,
    pub dev_info: DevInfoResp,
    pub config: ConfigResp,
    pub factory_cal: Option<CalDataResp>,
    pub if_cal: Option<CalParamResp>,
    pub avr_config: AvrConfigResp,
}

/// Phase 2: Query AVR for status, device info, parameters, radar config,
/// factory calibration, IF calibration, AVR config, and time sync.
pub fn sync_avr(conn: &mut Connection) -> Result<AvrSync, ConnError> {
    let t = TIMEOUT;

    // StatusPoll ×2 → AVR (keep last)
    let mut status = None;
    for _ in 0..2 {
        let env = send_recv(
            conn,
            &Command::StatusPoll(StatusPoll { pi_mode: false }),
            BusAddr::Avr,
            t,
        )?;
        status = Some(expect!(env, Message::AvrStatus)?);
    }
    let status = status.unwrap();

    // DevInfoReq ×2 → AVR (keep last)
    let mut dev_info = None;
    for _ in 0..2 {
        let env = send_recv(conn, &Command::DevInfoReq, BusAddr::Avr, t)?;
        dev_info = Some(expect!(env, Message::DevInfoResp)?);
    }
    let dev_info = dev_info.unwrap();

    // ParamReadReq ×2 → AVR (HW/FW version, consume BF responses)
    for param_id in [0x0C, 0x0D] {
        let env = send_recv(
            conn,
            &Command::ParamReadReq(ParamReadReq { param_id }),
            BusAddr::Avr,
            t,
        )?;
        // Consume BF response
        let _ = expect!(env, Message::ParamValue)?;
    }

    // ConfigQuery → AVR
    let env = send_recv(conn, &Command::ConfigQuery, BusAddr::Avr, t)?;
    let config = expect!(env, Message::ConfigResp)?;

    // CalDataReq (factory, sub-cmd 0x03) → AVR
    // Device may respond with ConfigAck before CalDataResp.
    conn.send(
        &Command::CalDataReq(CalDataReq {
            sub_cmd: 0x03,
            payload: CalDataReq::encode_factory(),
        }),
        BusAddr::Avr,
    )?;
    let factory_cal = match recv_skip_ack(conn, BusAddr::Avr, t, |m| match m {
        Message::CalDataResp(inner) => Some(inner),
        _ => None,
    }) {
        Ok(resp) => Some(resp),
        Err(ConnError::Timeout { .. }) => None,
        Err(e) => return Err(e),
    };

    // CalParamReq → AVR
    // Device responds with Text + ConfigAck + CalParamResp (confirmed from pcap).
    // May respond with ConfigNack(0x94) if device is in a stale state; CalParamResp
    // is constant factory data and not used downstream, so treat as optional.
    conn.send(&Command::CalParamReq(CalParamReq), BusAddr::Avr)?;
    let if_cal = match recv_skip_ack(conn, BusAddr::Avr, t, |m| match m {
        Message::CalParamResp(inner) => Some(inner),
        _ => None,
    }) {
        Ok(resp) => Some(resp),
        Err(ConnError::Timeout { .. }) => None,
        Err(e) => return Err(e),
    };

    // AvrConfigQuery → AVR
    let env = send_recv(conn, &Command::AvrConfigQuery, BusAddr::Avr, t)?;
    let avr_config = expect!(env, Message::AvrConfigResp)?;

    // Final ParamReadReq → AVR (consume response)
    let env = send_recv(
        conn,
        &Command::ParamReadReq(ParamReadReq { param_id: 0x64 }),
        BusAddr::Avr,
        t,
    )?;
    let _ = expect!(env, Message::ParamValue)?;

    // TimeSync → AVR
    let epoch = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs() as u32;
    let env = send_recv(
        conn,
        &Command::TimeSync(TimeSync {
            epoch,
            session: 0x00,
            tail: [0x00, 0x01],
        }),
        BusAddr::Avr,
        t,
    )?;
    // Consume echo
    let _ = expect!(env, Message::TimeSync)?;

    Ok(AvrSync {
        status,
        dev_info,
        config,
        factory_cal,
        if_cal,
        avr_config,
    })
}

// ---------------------------------------------------------------------------
// Phase 3 — PI Sync (SEQUENCE.md §2.3)
// ---------------------------------------------------------------------------

/// Results from Phase 3 — PI sync.
#[derive(Debug, Clone)]
pub struct PiSync {
    pub dev_info: DevInfoResp,
    pub cam_config: CamConfig,
    pub ssid: String,
    pub password: String,
}

/// Phase 3: Query PI for status, device info, parameters, camera config,
/// and network credentials.
///
/// Skips sensor activation (0x90) and WiFi scan (0x87) per §7.1.
pub fn sync_pi(conn: &mut Connection) -> Result<PiSync, ConnError> {
    let t = TIMEOUT;

    // StatusPoll(pi=true) → PI
    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: true }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::PiStatus)?;

    // DevInfoReq → PI
    let env = send_recv(conn, &Command::DevInfoReq, BusAddr::Pi, t)?;
    let dev_info = expect!(env, Message::DevInfoResp)?;

    // ParamReadReq 0x0A → PI (first capability flag, sent before CamConfig)
    let env = send_recv(
        conn,
        &Command::ParamReadReq(ParamReadReq { param_id: 0x0A }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::ParamValue)?;

    // CamConfigReq ×2 → PI (keep last)
    let mut cam_config = None;
    for _ in 0..2 {
        let env = send_recv(
            conn,
            &Command::CamConfigReq(CamConfigReq),
            BusAddr::Pi,
            t,
        )?;
        cam_config = Some(expect!(env, Message::CamConfig)?);
    }
    let cam_config = cam_config.unwrap();

    // NetConfigReq (SSID) → PI — response has IP/mask but empty text slots
    let env = send_recv(
        conn,
        &Command::NetConfigReq(NetConfigReq {
            query_password: false,
        }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::NetConfigResp)?;

    // NetConfigReq (password) → PI — response has both SSID and password
    let env = send_recv(
        conn,
        &Command::NetConfigReq(NetConfigReq {
            query_password: true,
        }),
        BusAddr::Pi,
        t,
    )?;
    let pw_resp = expect!(env, Message::NetConfigResp)?;
    let mut parts = pw_resp.text.splitn(2, '\0');
    let ssid = parts.next().unwrap_or("").to_string();
    let password = parts.next().unwrap_or("").to_string();

    // ParamReadReq → PI (capability flags, split into pcap-sized batches)
    for param_id in [0x01, 0x07, 0x08, 0x09] {
        let env = send_recv(
            conn,
            &Command::ParamReadReq(ParamReadReq { param_id }),
            BusAddr::Pi,
            t,
        )?;
        let _ = expect!(env, Message::ParamValue)?;
    }
    // Second batch (pcap has WiFi scan + sensor activation between these)
    for param_id in [0x06, 0x0B, 0x03, 0x04, 0x05] {
        let env = send_recv(
            conn,
            &Command::ParamReadReq(ParamReadReq { param_id }),
            BusAddr::Pi,
            t,
        )?;
        let _ = expect!(env, Message::ParamValue)?;
    }

    Ok(PiSync {
        dev_info,
        cam_config,
        ssid,
        password,
    })
}

// ---------------------------------------------------------------------------
// Phase 4 — Post-Sync AVR Configuration (SEQUENCE.md §2.4)
// ---------------------------------------------------------------------------

/// Settings for Phase 4 — AVR configuration.
#[derive(Debug, Clone)]
pub struct AvrSettings {
    /// Detection mode commsIndex (see `config::MODE_*` constants).
    pub mode: u8,
    /// BF parameter writes (ball type, tee height, track%, etc.).
    pub params: Vec<ParamValue>,
    /// Radar calibration values.
    pub radar_cal: RadarCal,
}

/// Phase 4: Write AVR parameters, set detection mode, radar calibration, arm config.
///
/// Every command is followed by B0 `[01 00]` config commit → ConfigAck,
/// matching the pattern observed in pcap and the working Python script.
pub fn configure_avr(conn: &mut Connection, settings: &AvrSettings) -> Result<(), ConnError> {
    let t = TIMEOUT;
    let b0_commit = Command::AvrConfigCmd(AvrConfigCmd { arm: false });

    // BF param writes, each followed by B0 commit
    for param in &settings.params {
        let env = send_recv(
            conn,
            &Command::ParamValue(param.clone()),
            BusAddr::Avr,
            t,
        )?;
        let _ = expect!(env, Message::ConfigAck)?;
        let env = send_recv(conn, &b0_commit, BusAddr::Avr, t)?;
        let _ = expect!(env, Message::ConfigAck)?;
    }

    // A5 mode set → echo, then B0 commit
    let env = send_recv(
        conn,
        &Command::ModeSet(ModeSet {
            mode: settings.mode,
        }),
        BusAddr::Avr,
        t,
    )?;
    let _ = expect!(env, Message::ModeSet)?;
    let env = send_recv(conn, &b0_commit, BusAddr::Avr, t)?;
    let _ = expect!(env, Message::ConfigAck)?;

    // A4 radar cal → echo, then B0 commit
    let env = send_recv(
        conn,
        &Command::RadarCal(settings.radar_cal.clone()),
        BusAddr::Avr,
        t,
    )?;
    let _ = expect!(env, Message::RadarCal)?;
    let env = send_recv(conn, &b0_commit, BusAddr::Avr, t)?;
    let _ = expect!(env, Message::ConfigAck)?;

    Ok(())
}

// ---------------------------------------------------------------------------
// Phase 5 — Camera Configuration (SEQUENCE.md §2.5)
// ---------------------------------------------------------------------------

/// Phase 5: Push camera config, start camera, set PI keepalive param.
pub fn configure_camera(conn: &mut Connection, config: &CamConfig) -> Result<(), ConnError> {
    let t = TIMEOUT;

    // Push camera config → 0x95 ack
    let env = send_recv(
        conn,
        &Command::CamConfig(config.clone()),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::ConfigAck)?;

    // CamConfigReq → readback (consume)
    let env = send_recv(
        conn,
        &Command::CamConfigReq(CamConfigReq),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::CamConfig)?;

    // CamState start → PI responds with ConfigAck (+ CamState echo, skipped by send_recv).
    let env = send_recv(
        conn,
        &Command::CamState(CamState { state: 0x01 }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::ConfigAck)?;

    // BF param write (0x02 = 10, PI keepalive interval) → 0x95 ack
    let env = send_recv(
        conn,
        &Command::ParamValue(ParamValue {
            param_id: 0x02,
            value: crate::protocol::config::ParamData::Int24(10),
        }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::ConfigAck)?;

    Ok(())
}

// ---------------------------------------------------------------------------
// Phase 6 — ARM (SEQUENCE.md §2.6)
// ---------------------------------------------------------------------------

/// Phase 6: Final status checks, arm the radar, wait for "ARMED" text.
pub fn arm(conn: &mut Connection) -> Result<(), ConnError> {
    let t = TIMEOUT;

    // Final DSP status poll
    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: false }),
        BusAddr::Dsp,
        t,
    )?;
    let _ = expect!(env, Message::DspStatus)?;

    // B0 [01 01] ARM → 0x95 ack
    let env = send_recv(
        conn,
        &Command::AvrConfigCmd(AvrConfigCmd { arm: true }),
        BusAddr::Avr,
        t,
    )?;
    let _ = expect!(env, Message::ConfigAck)?;

    // PI status poll
    let env = send_recv(
        conn,
        &Command::StatusPoll(StatusPoll { pi_mode: true }),
        BusAddr::Pi,
        t,
    )?;
    let _ = expect!(env, Message::PiStatus)?;

    // Wait for "ARMED" text
    wait_for_armed(conn, TIMEOUT)
}

// ---------------------------------------------------------------------------
// Operational helpers
// ---------------------------------------------------------------------------

/// Status from all three nodes, collected during a keepalive poll.
#[derive(Debug, Clone)]
pub struct KeepaliveStatus {
    pub dsp: DspStatus,
    pub avr: AvrStatus,
    pub pi: PiStatus,
}

/// Poll DSP → AVR → PI sequentially. Returns all three statuses.
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

/// Handle the entire post-shot cycle: ack → drain → rearm → ARMED.
///
/// Called after receiving E5 "PROCESSED". Drives the device through:
///   1. ShotDataAck ×2 (best-effort, don't require E9 responses)
///   2. Drain all remaining shot data until E5 "IDLE" arrives
///   3. ConfigQuery → consume B1 + A0
///   4. ShotResultReq → consume duplicate ED
///   5. B0 ARM → wait for "ARMED"
///
/// All intermediate messages (PrcData, ClubPrc, TrackingStatus, etc.) are
/// silently consumed. The `log` callback is invoked for notable events.
pub fn complete_shot(
    conn: &mut Connection,
    log: impl Fn(&str),
) -> Result<(), ConnError> {
    let t = TIMEOUT;

    // 1. Send ShotDataAck ×2 (best-effort).
    //    Don't block waiting for E9 — just fire and let the drain phase
    //    consume whatever comes back.
    for _ in 0..2 {
        conn.send(&Command::ShotDataAck, BusAddr::Avr)?;
    }

    // 2. Drain until we see "IDLE".
    //    Everything else (E9, PRC, ClubPrc, Text, etc.) is consumed.
    log("waiting for IDLE...");
    loop {
        // Use a generous per-message timeout — the device is processing.
        let env = conn.recv_timeout(t)?;
        if let Message::ShotText(ref st) = env.message {
            if st.is_idle() {
                log("IDLE");
                break;
            }
        }
    }

    // 3. ConfigQuery → AVR: expect B1 ModeAck + A0 ConfigResp (either order).
    conn.send(&Command::ConfigQuery, BusAddr::Avr)?;
    {
        let deadline = Instant::now() + t;
        let mut got_mode_ack = false;
        let mut got_config_resp = false;
        while !got_mode_ack || !got_config_resp {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break; // Best-effort — proceed to arm.
            }
            match conn.recv_timeout(remaining) {
                Ok(env) => match env.message {
                    Message::ModeAck(_) => got_mode_ack = true,
                    Message::ConfigResp(_) => got_config_resp = true,
                    _ => continue,
                },
                Err(ConnError::Timeout { .. }) => break,
                Err(e) => return Err(e),
            }
        }
    }

    // 4. ShotResultReq → AVR: triggers duplicate ED (byte-identical).
    conn.send(&Command::ShotResultReq, BusAddr::Avr)?;
    let _ = drain_until(conn, t, |m| matches!(m, Message::ClubResult(_)));

    // 5. ARM.
    let env = send_recv(
        conn,
        &Command::AvrConfigCmd(AvrConfigCmd { arm: true }),
        BusAddr::Avr,
        t,
    )?;
    let _ = expect!(env, Message::ConfigAck)?;

    wait_for_armed(conn, t)?;
    log("RE-ARMED");
    Ok(())
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/// Wait for an E3 text message containing "ARMED" (but not "CANCELLED").
///
/// Consumes all messages until the condition is met or timeout expires.
/// Text messages are not skipped here — we need to inspect them.
fn wait_for_armed(conn: &mut Connection, timeout: Duration) -> Result<(), ConnError> {
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout { timeout });
        }
        let env = conn.recv_timeout(remaining)?;
        if let Message::Text(ref text) = env.message
            && text.text.contains("ARMED")
            && !text.text.contains("CANCELLED")
        {
            return Ok(());
        }
        // Consume other messages (E3 logs, status, etc.)
    }
}
