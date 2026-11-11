//! Message types and decode/encode dispatch.
//!
//! - [`Command`] — messages we send to the Mevo+ (APP → device)
//! - [`Message`] — messages the Mevo+ sends to us (device → APP)
//!
//! Bidirectional message types (e.g., `ModeSet`, `RadarCal`, `CamState`)
//! appear as variants in both enums, wrapping the same struct.
//!
//! ## Versioned Enum Pattern
//!
//! Message types that vary across device generations or firmware versions use a
//! **versioned enum** for forwards/backwards compatibility. The pattern:
//!
//! 1. Top-level type is an **enum** (e.g., `DspStatus`), not a struct.
//! 2. Each wire format gets a **variant struct** (e.g., `DspStatus80`, `DspStatus46`).
//! 3. `decode()` dispatches on the version discriminator (typically `payload[0]`).
//! 4. **Helper methods on the enum** delegate to variants, so callers never match
//!    on variants directly. Unknown variants return defaults (0, false, 0.0).
//! 5. Unknown versions store raw payload bytes with hex `Debug` output.
//!
//! Reference: [`status::DspStatus`].

pub mod ack;
pub mod camera;
pub mod config;
pub mod handshake;
pub mod shot;
pub mod status;

use crate::addr::BusAddr;
use crate::error::Result;
use crate::frame::RawFrame;

// ---------------------------------------------------------------------------
// Type ID constants
// ---------------------------------------------------------------------------

// Requests (APP sends)
pub const TYPE_DSP_QUERY: u8 = 0x48;
pub const TYPE_CONFIG_QUERY: u8 = 0x21;
pub const TYPE_AVR_CONFIG_QUERY: u8 = 0x23;
pub const TYPE_DEV_INFO_REQ: u8 = 0x67;
pub const TYPE_SHOT_DATA_ACK: u8 = 0x69;
pub const TYPE_SHOT_RESULT_REQ: u8 = 0x6D;
pub const TYPE_CAM_STATE: u8 = 0x81;
pub const TYPE_CAM_CONFIG: u8 = 0x82;
pub const TYPE_CAM_CONFIG_REQ: u8 = 0x83;
pub const TYPE_WIFI_SCAN: u8 = 0x87;
pub const TYPE_SENSOR_ACT: u8 = 0x90;
pub const TYPE_TIME_SYNC: u8 = 0x9B;
pub const TYPE_RADAR_CAL: u8 = 0xA4;
pub const TYPE_MODE_SET: u8 = 0xA5;
pub const TYPE_STATUS: u8 = 0xAA;
pub const TYPE_AVR_CONFIG_CMD: u8 = 0xB0;
pub const TYPE_PARAM_READ_REQ: u8 = 0xBE;
pub const TYPE_PARAM_VALUE: u8 = 0xBF;
pub const TYPE_CAL_PARAM_REQ: u8 = 0xD0;
pub const TYPE_CAL_DATA_REQ: u8 = 0xD2;
pub const TYPE_NET_CONFIG: u8 = 0xDE;
pub const TYPE_PROD_INFO: u8 = 0xFD;

// Responses (device sends)
pub const TYPE_CAM_IMAGE_AVAIL: u8 = 0x84;
pub const TYPE_SENSOR_ACT_RESP: u8 = 0x89;
pub const TYPE_CONFIG_NACK: u8 = 0x94;
pub const TYPE_CONFIG_ACK: u8 = 0x95;
pub const TYPE_CONFIG_RESP: u8 = 0xA0;
pub const TYPE_AVR_CONFIG_RESP: u8 = 0xA2;
pub const TYPE_MODE_ACK: u8 = 0xB1;
pub const TYPE_DSP_QUERY_RESP: u8 = 0xC8;
pub const TYPE_CAL_PARAM_RESP: u8 = 0xD1;
pub const TYPE_CAL_DATA_RESP: u8 = 0xD3;
pub const TYPE_FLIGHT_RESULT: u8 = 0xD4;
pub const TYPE_SPEED_PROFILE: u8 = 0xD9;
pub const TYPE_TEXT: u8 = 0xE3;
pub const TYPE_SHOT_TEXT: u8 = 0xE5;
pub const TYPE_DEV_INFO_RESP: u8 = 0xE7;
pub const TYPE_FLIGHT_RESULT_V1: u8 = 0xE8;
pub const TYPE_TRACKING_STATUS: u8 = 0xE9;
pub const TYPE_PRC_DATA: u8 = 0xEC;
pub const TYPE_CLUB_RESULT: u8 = 0xED;
pub const TYPE_CLUB_PRC: u8 = 0xEE;
pub const TYPE_SPIN_RESULT: u8 = 0xEF;
pub const TYPE_DSP_DEBUG: u8 = 0xF0;

// ---------------------------------------------------------------------------
// Message classification
// ---------------------------------------------------------------------------

/// Whether frames of this type can identify the unit, its owner's network or
/// the host, in either direction. Keep such frames out of anything that is
/// shared, such as logs attached to a public bug report.
///
/// | Type | Message | Why |
/// | ---- | ------- | --- |
/// | 0xAA | status (request and response) | hardware and application IDs; also the keepalive |
/// | 0x67 / 0xE7 | device info | the DSP's text carries the serial number |
/// | 0xFD | product info | serial number |
/// | 0x48 / 0xC8 | DSP query | hardware information |
/// | 0xDE | network config | the device's SSID and password |
/// | 0x87 | WiFi scan | names of nearby networks |
/// | 0xD0 / 0xD1, 0xD2 / 0xD3 | calibration parameters and data | factory calibration, unique to the unit |
/// | 0x90 / 0x89 | sensor activation | a per-device certificate |
/// | 0x9B | time sync | the host's clock |
/// | 0x23 / 0xA2 | AVR config query | unit configuration, stored undecoded |
///
/// Shot results, shot texts, configuration writes and their acks are not on
/// the list.
#[must_use]
pub fn carries_identity(type_id: u8) -> bool {
    matches!(
        type_id,
        TYPE_STATUS
            | TYPE_DEV_INFO_REQ
            | TYPE_DEV_INFO_RESP
            | TYPE_PROD_INFO
            | TYPE_DSP_QUERY
            | TYPE_DSP_QUERY_RESP
            | TYPE_NET_CONFIG
            | TYPE_WIFI_SCAN
            | TYPE_CAL_PARAM_REQ
            | TYPE_CAL_PARAM_RESP
            | TYPE_CAL_DATA_REQ
            | TYPE_CAL_DATA_RESP
            | TYPE_SENSOR_ACT
            | TYPE_SENSOR_ACT_RESP
            | TYPE_TIME_SYNC
            | TYPE_AVR_CONFIG_QUERY
            | TYPE_AVR_CONFIG_RESP
    )
}

/// Whether this type is bulk per-shot data: radar tracking points
/// ([`TYPE_PRC_DATA`]), club tracking points ([`TYPE_CLUB_PRC`]) and the
/// speed profile ([`TYPE_SPEED_PROFILE`]). They follow a shot in pages and
/// add up to far more bytes than its results.
#[must_use]
pub fn is_bulk_shot_data(type_id: u8) -> bool {
    matches!(type_id, TYPE_PRC_DATA | TYPE_CLUB_PRC | TYPE_SPEED_PROFILE)
}

// ---------------------------------------------------------------------------
// Command — messages we send to the Mevo+
// ---------------------------------------------------------------------------

/// A message we send to the Mevo+ (APP → DSP/AVR/PI).
#[derive(Debug, Clone)]
pub enum Command {
    // -- Status --
    StatusPoll(status::StatusPoll),

    // -- Configuration --
    ModeSet(config::ModeSet),
    AvrConfigCmd(config::AvrConfigCmd),
    ParamReadReq(config::ParamReadReq),
    ParamValue(config::ParamValue),
    RadarCal(config::RadarCal),

    // -- Handshake --
    DspQuery,
    ConfigQuery,
    AvrConfigQuery,
    DevInfoReq,
    ProdInfoReq(handshake::ProdInfoReq),
    NetConfigReq(handshake::NetConfigReq),
    CalParamReq(handshake::CalParamReq),
    CalDataReq(handshake::CalDataReq),
    TimeSync(handshake::TimeSync),

    // -- Camera --
    CamState(camera::CamState),
    CamConfig(camera::CamConfig),
    CamConfigReq(camera::CamConfigReq),
    SensorAct(camera::SensorAct),

    // -- Shot ack (empty payloads) --
    ShotDataAck,
    ShotResultReq,

    // -- Result ack (the Xi's flight results) --
    ConfigAck(status::ConfigAck),
}

impl Command {
    /// Format as a hex debug line: `"APP→DSP 0xAA 2B | 01 01"`.
    pub fn debug_hex(&self, dest: BusAddr) -> String {
        let frame = self.encode(dest);
        let mut s = format!(
            "APP→{} 0x{:02X} {}B",
            dest,
            frame.type_id,
            frame.payload.len(),
        );
        if !frame.payload.is_empty() {
            s.push_str(" | ");
            let limit = 20;
            for b in frame.payload.iter().take(limit) {
                s.push_str(&format!("{b:02X}"));
            }
            if frame.payload.len() > limit {
                s.push_str("...");
            }
        }
        s
    }

    /// Encode into a `RawFrame` ready for wire transmission.
    pub fn encode(&self, dest: BusAddr) -> RawFrame {
        let src = BusAddr::App;
        let (type_id, payload) = match self {
            Command::StatusPoll(m) => (TYPE_STATUS, m.encode()),
            Command::ModeSet(m) => (TYPE_MODE_SET, m.encode()),
            Command::AvrConfigCmd(m) => (TYPE_AVR_CONFIG_CMD, m.encode()),
            Command::ParamReadReq(m) => (TYPE_PARAM_READ_REQ, m.encode()),
            Command::ParamValue(m) => (TYPE_PARAM_VALUE, m.encode()),
            Command::RadarCal(m) => (TYPE_RADAR_CAL, m.encode()),
            Command::DspQuery => (TYPE_DSP_QUERY, vec![]),
            Command::ConfigQuery => (TYPE_CONFIG_QUERY, vec![]),
            Command::AvrConfigQuery => (TYPE_AVR_CONFIG_QUERY, vec![]),
            Command::DevInfoReq => (TYPE_DEV_INFO_REQ, vec![]),
            Command::ProdInfoReq(m) => (TYPE_PROD_INFO, m.encode()),
            Command::NetConfigReq(m) => (TYPE_NET_CONFIG, m.encode()),
            Command::CalParamReq(m) => (TYPE_CAL_PARAM_REQ, m.encode()),
            Command::CalDataReq(m) => (TYPE_CAL_DATA_REQ, m.payload.clone()),
            Command::TimeSync(m) => (TYPE_TIME_SYNC, m.encode()),
            Command::CamState(m) => (TYPE_CAM_STATE, m.encode()),
            Command::CamConfig(m) => (TYPE_CAM_CONFIG, m.encode()),
            Command::CamConfigReq(_) => (TYPE_CAM_CONFIG_REQ, vec![0x02, 0x01, 0x05]),
            Command::SensorAct(m) => (TYPE_SENSOR_ACT, m.encode()),
            Command::ShotDataAck => (TYPE_SHOT_DATA_ACK, vec![]),
            Command::ShotResultReq => (TYPE_SHOT_RESULT_REQ, vec![]),
            Command::ConfigAck(m) => (TYPE_CONFIG_ACK, m.encode()),
        };

        RawFrame {
            dest,
            src,
            type_id,
            payload,
        }
    }
}

// ---------------------------------------------------------------------------
// Message — messages the Mevo+ sends to us
// ---------------------------------------------------------------------------

/// A message the Mevo+ sends to us (DSP/AVR/PI → APP).
///
/// Covers both solicited responses (handshake, config) and unsolicited
/// device pushes (shot results, state notifications).
#[derive(Debug, Clone)]
pub enum Message {
    // -- Status & keepalive --
    AvrStatus(status::AvrStatus),
    DspStatus(status::DspStatus),
    PiStatus(status::PiStatus),
    ConfigAck(status::ConfigAck),
    ConfigNack(status::ConfigAck),
    ModeAck(status::ModeAck),
    Text(status::Text),

    // -- Configuration (echoes/responses) --
    ModeSet(config::ModeSet),
    ParamValue(config::ParamValue),
    RadarCal(config::RadarCal),
    ConfigResp(config::ConfigResp),
    AvrConfigResp(config::AvrConfigResp),

    // -- Handshake responses --
    DspQueryResp(handshake::DspQueryResp),
    DevInfoResp(handshake::DevInfoResp),
    ProdInfoResp(handshake::ProdInfoResp),
    NetConfigResp(handshake::NetConfigResp),
    CalParamResp(handshake::CalParamResp),
    CalDataResp(handshake::CalDataResp),
    TimeSync(handshake::TimeSync),

    // -- Camera responses --
    CamState(camera::CamState),
    CamConfig(camera::CamConfig),
    CamImageAvail(camera::CamImageAvail),
    SensorActResp(camera::SensorActResp),
    WifiScan {
        payload: Vec<u8>,
    },

    // -- Shot results --
    FlightResult(shot::FlightResult),
    FlightResultV1(shot::FlightResultV1),
    ClubResult(shot::ClubResult),
    SpinResult(shot::SpinResult),
    SpeedProfile(shot::SpeedProfile),
    TrackingStatus(shot::TrackingStatus),
    PrcData(shot::PrcData),
    ClubPrc(shot::ClubPrc),
    ShotText(shot::ShotText),

    /// Gen2 DSP debug output (VT100 terminal text). Log and ignore.
    DspDebug(Vec<u8>),

    // -- Forward compat --
    Unknown {
        type_id: u8,
        src: BusAddr,
        payload: Vec<u8>,
    },
}

impl Message {
    /// Decode a `RawFrame` into a typed `Message`.
    pub fn decode(frame: &RawFrame) -> Result<Self> {
        let p = &frame.payload;
        match frame.type_id {
            // -- Status --
            TYPE_STATUS => match frame.src {
                BusAddr::Avr => Ok(Message::AvrStatus(status::AvrStatus::decode(p)?)),
                BusAddr::Dsp => Ok(Message::DspStatus(status::DspStatus::decode(p)?)),
                BusAddr::Pi => Ok(Message::PiStatus(status::PiStatus::decode(p)?)),
                BusAddr::App => Ok(Message::Unknown {
                    type_id: frame.type_id,
                    src: frame.src,
                    payload: p.to_vec(),
                }),
            },
            TYPE_CONFIG_ACK => Ok(Message::ConfigAck(status::ConfigAck::decode(p)?)),
            TYPE_CONFIG_NACK => Ok(Message::ConfigNack(status::ConfigAck::decode(p)?)),
            TYPE_MODE_ACK => Ok(Message::ModeAck(status::ModeAck::decode(p)?)),
            TYPE_TEXT => Ok(Message::Text(status::Text::decode(p)?)),

            // -- Configuration echoes/responses --
            TYPE_MODE_SET => Ok(Message::ModeSet(config::ModeSet::decode(p)?)),
            TYPE_PARAM_VALUE => Ok(Message::ParamValue(config::ParamValue::decode(p)?)),
            TYPE_RADAR_CAL => Ok(Message::RadarCal(config::RadarCal::decode(p)?)),
            TYPE_CONFIG_RESP => Ok(Message::ConfigResp(config::ConfigResp::decode(p)?)),
            TYPE_AVR_CONFIG_RESP => Ok(Message::AvrConfigResp(config::AvrConfigResp::decode(p)?)),

            // -- Handshake responses --
            TYPE_DSP_QUERY_RESP => Ok(Message::DspQueryResp(handshake::DspQueryResp::decode(p)?)),
            TYPE_DEV_INFO_RESP => Ok(Message::DevInfoResp(handshake::DevInfoResp::decode(p)?)),
            TYPE_PROD_INFO => Ok(Message::ProdInfoResp(handshake::ProdInfoResp::decode(p)?)),
            TYPE_NET_CONFIG => Ok(Message::NetConfigResp(handshake::NetConfigResp::decode(p)?)),
            TYPE_CAL_PARAM_RESP => Ok(Message::CalParamResp(handshake::CalParamResp::decode(p)?)),
            TYPE_CAL_DATA_RESP => Ok(Message::CalDataResp(handshake::CalDataResp::decode(p)?)),
            TYPE_TIME_SYNC => Ok(Message::TimeSync(handshake::TimeSync::decode(p)?)),

            // -- Camera responses --
            TYPE_CAM_STATE => Ok(Message::CamState(camera::CamState::decode(p)?)),
            TYPE_CAM_CONFIG => Ok(Message::CamConfig(camera::CamConfig::decode(p)?)),
            TYPE_CAM_IMAGE_AVAIL => Ok(Message::CamImageAvail(camera::CamImageAvail::decode(p)?)),
            TYPE_SENSOR_ACT_RESP => Ok(Message::SensorActResp(camera::SensorActResp::decode(p)?)),
            TYPE_WIFI_SCAN => Ok(Message::WifiScan {
                payload: p.to_vec(),
            }),

            // -- Shot results --
            TYPE_FLIGHT_RESULT => Ok(Message::FlightResult(shot::FlightResult::decode(p)?)),
            TYPE_FLIGHT_RESULT_V1 => Ok(Message::FlightResultV1(shot::FlightResultV1::decode(p)?)),
            TYPE_CLUB_RESULT => Ok(Message::ClubResult(shot::ClubResult::decode(p)?)),
            TYPE_SPIN_RESULT => Ok(Message::SpinResult(shot::SpinResult::decode(p)?)),
            TYPE_SPEED_PROFILE => Ok(Message::SpeedProfile(shot::SpeedProfile::decode(p)?)),
            TYPE_TRACKING_STATUS => Ok(Message::TrackingStatus(shot::TrackingStatus::decode(p)?)),
            TYPE_PRC_DATA => Ok(Message::PrcData(shot::PrcData::decode(p)?)),
            TYPE_CLUB_PRC => Ok(Message::ClubPrc(shot::ClubPrc::decode(p)?)),
            TYPE_SHOT_TEXT => Ok(Message::ShotText(shot::ShotText::decode(p)?)),
            TYPE_DSP_DEBUG => Ok(Message::DspDebug(p.to_vec())),

            _ => Ok(Message::Unknown {
                type_id: frame.type_id,
                src: frame.src,
                payload: p.to_vec(),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Exactly these 17 types carry identity, and exactly these 3 are bulk
    /// shot data, out of every possible type byte.
    #[test]
    fn identity_and_bulk_types() {
        let identity: Vec<u8> = (0..=255).filter(|&t| carries_identity(t)).collect();
        assert_eq!(
            identity,
            [
                0x23, 0x48, 0x67, 0x87, 0x89, 0x90, 0x9B, 0xA2, 0xAA, 0xC8, 0xD0, 0xD1, 0xD2, 0xD3,
                0xDE, 0xE7, 0xFD,
            ]
        );
        let bulk: Vec<u8> = (0..=255).filter(|&t| is_bulk_shot_data(t)).collect();
        assert_eq!(bulk, [0xD9, 0xEC, 0xEE]);
        // Shot results, texts, configuration and acks are neither.
        for t in [
            TYPE_FLIGHT_RESULT,
            TYPE_FLIGHT_RESULT_V1,
            TYPE_CLUB_RESULT,
            TYPE_SPIN_RESULT,
            TYPE_SHOT_TEXT,
            TYPE_TEXT,
            TYPE_PARAM_VALUE,
            TYPE_CONFIG_ACK,
            TYPE_CONFIG_NACK,
            TYPE_AVR_CONFIG_CMD,
            TYPE_MODE_SET,
            TYPE_RADAR_CAL,
        ] {
            assert!(!carries_identity(t), "0x{t:02X}");
            assert!(!is_bulk_shot_data(t), "0x{t:02X}");
        }
    }
}
