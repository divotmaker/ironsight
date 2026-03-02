//! GVP (Golf Video Processor) JSON protocol client for TCP port 1258.
//!
//! The GVP communicates via null-terminated JSON over TCP. Nine message types
//! are defined. The critical one for face impact fusion is `RESULT`
//! ([`BallTrackerResult`](result::BallTrackerResult)), which carries raw pixel
//! tracking data for the club head and ball.
//!
//! # Message types
//!
//! | Type | Direction | Rust type |
//! |------|-----------|-----------|
//! | `CONFIG_REQUEST` | APP → GVP | [`GvpCommand::ConfigRequest`] |
//! | `CONFIG` | Both | [`GvpMessage::Config`] / [`GvpCommand::Config`] |
//! | `STATUS` | GVP → APP | [`GvpMessage::Status`] |
//! | `TRIGGER` | APP → GVP | [`GvpCommand::Trigger`] |
//! | `LOG` | GVP → APP | [`GvpMessage::Log`] |
//! | `MT_GOLF_EXPECTED_CLUB_TRACK` | APP → GVP | [`GvpCommand::ExpectedClubTrack`] |
//! | `MT_GOLF_EXPECTED_TRACK` | APP → GVP | [`GvpCommand::ExpectedTrack`] |
//! | `RESULT` | GVP → APP | [`GvpMessage::Result`] |
//! | `MT_VIDEO_AVAILABLE` | GVP → APP | [`GvpMessage::VideoAvailable`] |

pub mod config;
pub mod conn;
pub mod log;
pub mod result;
pub mod splitter;
pub mod status;
pub mod track;
pub mod trigger;
pub mod video;

use serde_json::Value;
use thiserror::Error;

pub use config::GvpConfig;
pub use conn::GvpConnection;
pub use result::BallTrackerResult;
pub use splitter::NullSplitter;
pub use status::GvpStatus;

/// Shot GUID type alias. Callers generate UUIDs and send them with TRIGGER;
/// the matching RESULT carries the same GUID.
pub type ShotGuid = String;

// -------------------------------------------------------------------------
// Message type strings
// -------------------------------------------------------------------------

const TYPE_CONFIG_REQUEST: &str = "CONFIG_REQUEST";
const TYPE_CONFIG: &str = "CONFIG";
const TYPE_STATUS: &str = "STATUS";
const TYPE_TRIGGER: &str = "TRIGGER";
const TYPE_LOG: &str = "LOG";
const TYPE_EXPECTED_CLUB_TRACK: &str = "MT_GOLF_EXPECTED_CLUB_TRACK";
const TYPE_EXPECTED_TRACK: &str = "MT_GOLF_EXPECTED_TRACK";
const TYPE_RESULT: &str = "RESULT";
const TYPE_VIDEO_AVAILABLE: &str = "MT_VIDEO_AVAILABLE";

// -------------------------------------------------------------------------
// Protocol versions for commands we send (observed from pcap)
// -------------------------------------------------------------------------

const VERSION_CONFIG_REQUEST: i32 = 1;
const VERSION_CONFIG: i32 = 5;
const VERSION_TRIGGER: i32 = 6;
const VERSION_EXPECTED_TRACK: i32 = 1;

// -------------------------------------------------------------------------
// Error
// -------------------------------------------------------------------------

/// Errors from GVP protocol operations.
#[derive(Debug, Error)]
pub enum GvpError {
    /// TCP I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// JSON parse error.
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    /// No complete message available (stream returned `WouldBlock`/`TimedOut`).
    #[error("recv timed out")]
    Timeout,

    /// TCP stream closed by peer.
    #[error("connection closed by GVP")]
    Disconnected,

    /// Unknown message type string.
    #[error("unknown GVP message type: {msg_type:?}")]
    UnknownType { msg_type: String },

    /// Protocol-level error (missing fields, unexpected structure).
    #[error("GVP protocol error: {0}")]
    Protocol(String),
}

// -------------------------------------------------------------------------
// GvpMessage — messages the GVP sends to us
// -------------------------------------------------------------------------

/// A message received from the GVP (GVP → APP).
#[derive(Debug, Clone)]
pub enum GvpMessage {
    /// Camera configuration (response to CONFIG_REQUEST or echo).
    Config(config::GvpConfig),
    /// Buffer status update.
    Status(status::GvpStatus),
    /// Debug/info log message.
    Log(log::GvpLog),
    /// Ball tracker result with raw pixel tracking data.
    Result(result::BallTrackerResult),
    /// Video file saved notification.
    VideoAvailable(video::VideoAvailable),
    /// Unknown message type (forward compatibility).
    Unknown { msg_type: String, raw: String },
}

impl GvpMessage {
    /// Decode a raw JSON string into a typed `GvpMessage`.
    ///
    /// Parses the `"type"` field to determine the message type, then
    /// deserializes the full JSON into the appropriate struct.
    pub fn decode(json: &str) -> Result<Self, GvpError> {
        let value: Value = serde_json::from_str(json)?;
        let msg_type = value
            .get("type")
            .and_then(Value::as_str)
            .ok_or_else(|| GvpError::Protocol("missing \"type\" field".into()))?;

        match msg_type {
            TYPE_CONFIG => {
                let body: config::GvpConfig = deserialize_body(&value)?;
                Ok(GvpMessage::Config(body))
            }
            TYPE_STATUS => {
                let body: status::GvpStatus = deserialize_body(&value)?;
                Ok(GvpMessage::Status(body))
            }
            TYPE_LOG => {
                let body: log::GvpLog = deserialize_body(&value)?;
                Ok(GvpMessage::Log(body))
            }
            TYPE_RESULT => {
                let body: result::BallTrackerResult = deserialize_body(&value)?;
                Ok(GvpMessage::Result(body))
            }
            TYPE_VIDEO_AVAILABLE => {
                let body: video::VideoAvailable = deserialize_body(&value)?;
                Ok(GvpMessage::VideoAvailable(body))
            }
            // Forward compat: CONFIG_REQUEST, TRIGGER, and track messages are
            // APP→GVP only, but if the GVP ever echoes them, store as Unknown.
            other => Ok(GvpMessage::Unknown {
                msg_type: other.to_owned(),
                raw: json.to_owned(),
            }),
        }
    }
}

// -------------------------------------------------------------------------
// GvpCommand — messages we send to the GVP
// -------------------------------------------------------------------------

/// A message we send to the GVP (APP → GVP).
#[derive(Debug, Clone)]
pub enum GvpCommand {
    /// Request current camera configuration.
    ConfigRequest,
    /// Push updated camera configuration.
    Config(config::GvpConfig),
    /// Notify camera of shot trigger.
    Trigger(trigger::Trigger),
    /// Radar-derived club head trajectory hint.
    ExpectedClubTrack(track::ExpectedTrack),
    /// Radar-derived ball trajectory hint.
    ExpectedTrack(track::ExpectedTrack),
}

impl GvpCommand {
    /// Encode this command as a null-terminated JSON byte string.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let json = match self {
            GvpCommand::ConfigRequest => {
                serde_json::json!({
                    "type": TYPE_CONFIG_REQUEST,
                    "version": VERSION_CONFIG_REQUEST,
                })
            }
            GvpCommand::Config(cfg) => {
                let mut value = serde_json::to_value(cfg).expect("GvpConfig serialization");
                let obj = value.as_object_mut().expect("GvpConfig is an object");
                obj.insert("type".into(), TYPE_CONFIG.into());
                obj.insert("version".into(), VERSION_CONFIG.into());
                value
            }
            GvpCommand::Trigger(trig) => {
                let mut value = serde_json::to_value(trig).expect("Trigger serialization");
                let obj = value.as_object_mut().expect("Trigger is an object");
                obj.insert("type".into(), TYPE_TRIGGER.into());
                obj.insert("version".into(), VERSION_TRIGGER.into());
                value
            }
            GvpCommand::ExpectedClubTrack(track) => {
                let mut value =
                    serde_json::to_value(track).expect("ExpectedTrack serialization");
                let obj = value.as_object_mut().expect("ExpectedTrack is an object");
                obj.insert("type".into(), TYPE_EXPECTED_CLUB_TRACK.into());
                obj.insert("version".into(), VERSION_EXPECTED_TRACK.into());
                value
            }
            GvpCommand::ExpectedTrack(track) => {
                let mut value =
                    serde_json::to_value(track).expect("ExpectedTrack serialization");
                let obj = value.as_object_mut().expect("ExpectedTrack is an object");
                obj.insert("type".into(), TYPE_EXPECTED_TRACK.into());
                obj.insert("version".into(), VERSION_EXPECTED_TRACK.into());
                value
            }
        };

        let mut bytes = serde_json::to_vec(&json).expect("JSON serialization");
        bytes.push(0x00); // null terminator
        bytes
    }

    /// Encode as a pretty-printed null-terminated JSON byte string (for debugging).
    #[must_use]
    pub fn encode_pretty(&self) -> Vec<u8> {
        // Re-encode with pretty printing for debug/logging.
        let compact = self.encode();
        let value: Value =
            serde_json::from_slice(&compact[..compact.len() - 1]).expect("re-parse");
        let mut bytes = serde_json::to_vec_pretty(&value).expect("pretty JSON");
        bytes.push(0x00);
        bytes
    }
}

// -------------------------------------------------------------------------
// Internal helpers
// -------------------------------------------------------------------------

/// Deserialize a body struct from a JSON Value, ignoring the envelope
/// fields (`type`, `version`).
///
/// serde_json's `#[serde(deny_unknown_fields)]` is NOT used on any GVP
/// struct, so the extra `type`/`version` fields are silently ignored
/// during deserialization.
fn deserialize_body<T: serde::de::DeserializeOwned>(value: &Value) -> Result<T, GvpError> {
    serde_json::from_value(value.clone()).map_err(GvpError::Json)
}

// -------------------------------------------------------------------------
// Tests
// -------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_config_request_as_unknown() {
        // CONFIG_REQUEST is APP→GVP, so if received it's Unknown
        let json = r#"{"type":"CONFIG_REQUEST","version":1}"#;
        let msg = GvpMessage::decode(json).unwrap();
        assert!(matches!(msg, GvpMessage::Unknown { .. }));
    }

    #[test]
    fn decode_status() {
        let json = r#"{"type":"STATUS","version":1,"bufferStatus":[{"bufferIndex":0,"status":"IDLE"}]}"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::Status(s) => {
                assert_eq!(s.status(), "IDLE");
                assert!(s.is_idle());
            }
            other => panic!("expected Status, got {other:?}"),
        }
    }

    #[test]
    fn decode_log() {
        let json = r#"{"type":"LOG","version":1,"level":1,"message":"[GVP] test"}"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::Log(l) => {
                assert_eq!(l.level, 1);
                assert_eq!(l.message, "[GVP] test");
            }
            other => panic!("expected Log, got {other:?}"),
        }
    }

    #[test]
    fn decode_video_available() {
        let json = r#"{"type":"MT_VIDEO_AVAILABLE","version":1,"guid":"{abc}","absolutePath":"/home/ftp/video.mp4","relativePath":"/video.mp4"}"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::VideoAvailable(v) => {
                assert_eq!(v.guid, "{abc}");
                assert_eq!(v.absolute_path, "/home/ftp/video.mp4");
            }
            other => panic!("expected VideoAvailable, got {other:?}"),
        }
    }

    #[test]
    fn decode_missing_type_field() {
        let json = r#"{"version":1,"level":1}"#;
        let err = GvpMessage::decode(json).unwrap_err();
        assert!(matches!(err, GvpError::Protocol(_)));
    }

    #[test]
    fn decode_unknown_type() {
        let json = r#"{"type":"FUTURE_TYPE","version":99}"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::Unknown { msg_type, .. } => {
                assert_eq!(msg_type, "FUTURE_TYPE");
            }
            other => panic!("expected Unknown, got {other:?}"),
        }
    }

    #[test]
    fn encode_config_request_round_trip() {
        let cmd = GvpCommand::ConfigRequest;
        let bytes = cmd.encode();
        assert_eq!(bytes.last(), Some(&0x00));

        let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
        let value: Value = serde_json::from_str(json_str).unwrap();
        assert_eq!(value["type"], "CONFIG_REQUEST");
        assert_eq!(value["version"], 1);
    }

    #[test]
    fn encode_trigger_round_trip() {
        let trig = trigger::Trigger::new("{test-guid}".into(), 1771985855.931274);
        let cmd = GvpCommand::Trigger(trig);
        let bytes = cmd.encode();
        assert_eq!(bytes.last(), Some(&0x00));

        let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
        let value: Value = serde_json::from_str(json_str).unwrap();
        assert_eq!(value["type"], "TRIGGER");
        assert_eq!(value["version"], 6);
        assert_eq!(value["guid"], "{test-guid}");
        assert_eq!(value["skipTracking"], false);
    }

    #[test]
    fn encode_config_has_type_and_version() {
        let cfg = config::GvpConfig::fusion();
        let cmd = GvpCommand::Config(cfg);
        let bytes = cmd.encode();

        let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
        let value: Value = serde_json::from_str(json_str).unwrap();
        assert_eq!(value["type"], "CONFIG");
        assert_eq!(value["version"], 5);
        // fusion() sends 640x480 (APP→GVP metadata)
        assert_eq!(value["cameraConfiguration"]["ROI_width"], 640);
        assert_eq!(value["cameraConfiguration"]["ROI_height"], 480);
    }

    #[test]
    fn encode_expected_track() {
        let track = track::ExpectedTrack {
            guid: "{test}".into(),
            duration: 0.1,
            start_time: 0.014,
            poly_u: [308.79, 0.013, -0.119, -15.205, 293.455],
            poly_v: [237.32, 0.007, 0.300, -8.510, 28.224],
            poly_radius: [1.0, 16.332, 2.713, 0.0, 0.0],
        };
        let cmd = GvpCommand::ExpectedClubTrack(track);
        let bytes = cmd.encode();

        let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
        let value: Value = serde_json::from_str(json_str).unwrap();
        assert_eq!(value["type"], "MT_GOLF_EXPECTED_CLUB_TRACK");
        assert_eq!(value["version"], 1);
        assert_eq!(value["polyU"].as_array().unwrap().len(), 5);
    }

    #[test]
    fn config_constructors() {
        // fusion() sends 640x480 (APP→GVP metadata, not capture resolution)
        let cfg = config::GvpConfig::fusion();
        assert_eq!(cfg.camera_configuration.roi_width, 640);
        assert_eq!(cfg.camera_configuration.roi_height, 480);
        assert_eq!(cfg.buffer_configuration.buffer_size_pre_trigger, 0);

        // standard() sends 0x0
        let std = config::GvpConfig::standard();
        assert_eq!(std.camera_configuration.roi_width, 0);
        assert_eq!(std.camera_configuration.roi_height, 0);

        // is_fusion() checks GVP-reported resolution (1640x1232), not APP-sent
        assert!(!cfg.is_fusion());
        assert!(!std.is_fusion());
    }

    #[test]
    fn decode_config_message() {
        let json = r#"{
            "type": "CONFIG",
            "version": 5,
            "bufferConfiguration": {"bufferSizePreTrigger": 10, "bufferSizePostTrigger": 200},
            "cameraCalibration": {"cx": 0, "cy": 0, "fx": 0, "fy": 0, "width": 0, "height": 0, "position": [0,0,0], "rotation": [0,0,0], "distCoeffs": [0,0,0,0,0,0,0,0]},
            "cameraConfiguration": {"ROI_x": 0, "ROI_y": 0, "ROI_width": 1640, "ROI_height": 1232, "ROI_maxWidth": 0, "ROI_maxHeight": 0, "isFreeRun": true, "rotationDegCW": 0},
            "livePreviewProcessingConfiguration": {"ROI_center_u": 0, "ROI_center_v": 0, "ROI_width": 320, "ROI_height": 160, "enabled": false, "rotationDegCW": 0},
            "frameNumberInfoEnabled": true,
            "loggingEnabled": true,
            "saveVideosEnabled": true
        }"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::Config(cfg) => {
                assert!(cfg.is_fusion());
                assert_eq!(cfg.buffer_configuration.buffer_size_pre_trigger, 10);
                assert_eq!(cfg.buffer_configuration.buffer_size_post_trigger, 200);
                assert!(cfg.frame_number_info_enabled);
            }
            other => panic!("expected Config, got {other:?}"),
        }
    }

    #[test]
    fn decode_result_minimal() {
        let json = r#"{
            "type": "RESULT",
            "version": 1,
            "guid": "{test-guid}",
            "cameraCalibration": {"cx": 0, "cy": 0, "fx": 0, "fy": 0, "width": 0, "height": 0, "position": [0,0,0], "rotation": [0,0,0], "distCoeffs": [0,0,0,0,0,0,0,0]},
            "tracks": [
                {
                    "trackId": 1,
                    "frameNumber": [13, 15],
                    "timestamp": [1771985894.233, 1771985894.244],
                    "u": [218.20, 248.65],
                    "v": [124.90, 198.90],
                    "radius": [56.15, 55.13],
                    "circularityFactor": [59.80, 53.35],
                    "shutterTime_ms": [1, 1]
                },
                {
                    "trackId": 0,
                    "frameNumber": [23],
                    "timestamp": [1771985894.300],
                    "u": [325.9],
                    "v": [302.9],
                    "radius": [12.9],
                    "circularityFactor": [0.74],
                    "shutterTime_ms": [1]
                }
            ]
        }"#;
        let msg = GvpMessage::decode(json).unwrap();
        match msg {
            GvpMessage::Result(r) => {
                assert_eq!(r.guid, "{test-guid}");
                assert_eq!(r.tracks.len(), 2);

                let club = r.club_track().unwrap();
                assert!(club.is_club());
                assert_eq!(club.len(), 2);

                let ball = r.ball_track().unwrap();
                assert!(ball.is_ball());
                assert_eq!(ball.len(), 1);
            }
            other => panic!("expected Result, got {other:?}"),
        }
    }
}
