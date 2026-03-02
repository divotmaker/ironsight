//! STATUS message type.
//!
//! Buffer status updates sent by GVP during shot processing.
//! Status transitions: IDLE → TRIGGERED → CONVERTING → PROCESSING → SAVING → IDLE

use serde::{Deserialize, Serialize};

/// Individual buffer status entry.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct BufferStatus {
    pub buffer_index: i32,
    pub status: String,
}

/// GVP buffer status (STATUS message body).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GvpStatus {
    pub buffer_status: Vec<BufferStatus>,
}

impl GvpStatus {
    /// The status string of buffer 0, or "UNKNOWN" if no buffers present.
    #[must_use]
    pub fn status(&self) -> &str {
        self.buffer_status
            .first()
            .map_or("UNKNOWN", |b| b.status.as_str())
    }

    /// Whether buffer 0 is idle and ready for the next trigger.
    #[must_use]
    pub fn is_idle(&self) -> bool {
        self.status() == "IDLE"
    }
}
