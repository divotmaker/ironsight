//! MT_VIDEO_AVAILABLE message type.
//!
//! Sent by GVP after the shot video has been encoded and saved.

use serde::{Deserialize, Serialize};

/// Video available notification (MT_VIDEO_AVAILABLE message body).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct VideoAvailable {
    pub guid: String,
    pub absolute_path: String,
    pub relative_path: String,
}
