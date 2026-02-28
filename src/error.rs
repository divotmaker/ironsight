use thiserror::Error;

/// Errors arising from wire protocol parsing and encoding.
#[derive(Debug, Error)]
pub enum WireError {
    #[error("frame too short ({len} bytes, minimum 7)")]
    FrameTooShort { len: usize },

    #[error("missing start marker (expected 0xF0, got 0x{got:02X})")]
    MissingStart { got: u8 },

    #[error("missing end marker (expected 0xF1)")]
    MissingEnd,

    #[error("invalid escape sequence 0xFD 0x{code:02X} at offset {offset}")]
    InvalidEscape { code: u8, offset: usize },

    #[error("checksum mismatch (expected 0x{expected:04X}, computed 0x{computed:04X})")]
    ChecksumMismatch { expected: u16, computed: u16 },

    #[error("unknown bus address 0x{addr:02X}")]
    UnknownBusAddr { addr: u8 },

    #[error("payload too short for {msg_type}: need {need} bytes, got {got}{}", format_raw_suffix(raw))]
    PayloadTooShort {
        msg_type: &'static str,
        need: usize,
        got: usize,
        /// Raw unstuffed payload bytes for debug context.
        raw: Vec<u8>,
    },

    #[error("invalid FLOAT40: non-zero exponent with zero mantissa")]
    InvalidFloat40,

    #[error("invalid string payload: {0}")]
    InvalidString(#[from] std::string::FromUtf8Error),

    #[error("unexpected payload length for {msg_type}: expected {expected}, got {got}{}", format_raw_suffix(raw))]
    UnexpectedLength {
        msg_type: &'static str,
        expected: usize,
        got: usize,
        /// Raw unstuffed payload bytes for debug context.
        raw: Vec<u8>,
    },
}

impl WireError {
    /// Create a `PayloadTooShort` error (raw bytes filled in later via `with_raw`).
    pub(crate) fn payload_too_short(msg_type: &'static str, need: usize, got: usize) -> Self {
        Self::PayloadTooShort { msg_type, need, got, raw: Vec::new() }
    }

    /// Create an `UnexpectedLength` error (raw bytes filled in later via `with_raw`).
    pub(crate) fn unexpected_length(msg_type: &'static str, expected: usize, got: usize) -> Self {
        Self::UnexpectedLength { msg_type, expected, got, raw: Vec::new() }
    }

    /// Attach raw payload bytes to decode-phase errors for diagnostics.
    pub fn with_raw(self, payload: &[u8]) -> Self {
        match self {
            Self::PayloadTooShort { msg_type, need, got, .. } => {
                Self::PayloadTooShort { msg_type, need, got, raw: payload.to_vec() }
            }
            Self::UnexpectedLength { msg_type, expected, got, .. } => {
                Self::UnexpectedLength { msg_type, expected, got, raw: payload.to_vec() }
            }
            other => other,
        }
    }
}

/// Format raw bytes as a suffix like " | 9E 00 03 ..." (empty if no bytes).
fn format_raw_suffix(raw: &[u8]) -> String {
    if raw.is_empty() {
        return String::new();
    }
    let limit = 16;
    let hex: String = raw.iter().take(limit).map(|b| format!("{b:02X}")).collect();
    let ellipsis = if raw.len() > limit { "..." } else { "" };
    format!(" | {hex}{ellipsis}")
}

pub type Result<T> = std::result::Result<T, WireError>;
