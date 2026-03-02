//! LOG message type.
//!
//! Debug/informational messages from the GVP processor.

use serde::{Deserialize, Serialize};

/// GVP log message (LOG message body).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GvpLog {
    pub level: i32,
    pub message: String,
}
