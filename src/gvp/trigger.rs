//! TRIGGER message type.
//!
//! Sent by APP to GVP when the binary protocol reports a ball trigger
//! (0xE5 on port 5100). Contains a GUID that ties camera data to the shot.

use serde::{Deserialize, Serialize};

/// Shot trigger notification (TRIGGER message body).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Trigger {
    pub guid: String,
    pub epoch_time: f64,
    pub frame_number: i32,
    pub save_path: String,
    pub reprocess_path: String,
    pub skip_tracking: bool,
    pub trigger_time_offset: f64,
}

impl Trigger {
    /// Create a new trigger with the given GUID and epoch time.
    ///
    /// Sets defaults for the remaining fields (frame_number=0, save_path=guid,
    /// empty reprocess_path, skip_tracking=false, trigger_time_offset=0).
    #[must_use]
    pub fn new(guid: String, epoch_time: f64) -> Self {
        Self {
            save_path: guid.clone(),
            guid,
            epoch_time,
            frame_number: 0,
            reprocess_path: String::new(),
            skip_tracking: false,
            trigger_time_offset: 0.0,
        }
    }
}
