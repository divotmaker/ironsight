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

    #[error(
        "payload too short for {msg_type}: need {need} bytes, got {got}{}",
        format_raw_suffix(raw)
    )]
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

    #[error(
        "unexpected payload length for {msg_type}: expected {expected}, got {got}{}",
        format_raw_suffix(raw)
    )]
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
        Self::PayloadTooShort {
            msg_type,
            need,
            got,
            raw: Vec::new(),
        }
    }

    /// Create an `UnexpectedLength` error (raw bytes filled in later via `with_raw`).
    pub(crate) fn unexpected_length(msg_type: &'static str, expected: usize, got: usize) -> Self {
        Self::UnexpectedLength {
            msg_type,
            expected,
            got,
            raw: Vec::new(),
        }
    }

    /// Attach raw payload bytes to decode-phase errors for diagnostics.
    pub fn with_raw(self, payload: &[u8]) -> Self {
        match self {
            Self::PayloadTooShort {
                msg_type,
                need,
                got,
                ..
            } => Self::PayloadTooShort {
                msg_type,
                need,
                got,
                raw: payload.to_vec(),
            },
            Self::UnexpectedLength {
                msg_type,
                expected,
                got,
                ..
            } => Self::UnexpectedLength {
                msg_type,
                expected,
                got,
                raw: payload.to_vec(),
            },
            other => other,
        }
    }

    /// The payload bytes this error carries for diagnostics, if any: the
    /// raw payload of a decode error, or the bytes of an invalid string.
    #[must_use]
    pub fn payload(&self) -> Option<&[u8]> {
        match self {
            Self::PayloadTooShort { raw, .. } | Self::UnexpectedLength { raw, .. }
                if !raw.is_empty() =>
            {
                Some(raw)
            }
            Self::InvalidString(e) => Some(e.as_bytes()),
            _ => None,
        }
    }

    /// The error as text without any payload bytes. A payload can carry the
    /// device's identity (serial number, network name), so this is the form
    /// for an alert or a log line that may be shared; the bytes are in
    /// [`payload`](Self::payload).
    #[must_use]
    pub fn describe(&self) -> String {
        match self {
            Self::PayloadTooShort {
                msg_type,
                need,
                got,
                ..
            } => format!("payload too short for {msg_type}: need {need} bytes, got {got}"),
            Self::UnexpectedLength {
                msg_type,
                expected,
                got,
                ..
            } => {
                format!("unexpected payload length for {msg_type}: expected {expected}, got {got}")
            }
            Self::InvalidString(e) => format!("invalid string payload: {}", e.utf8_error()),
            other => other.to_string(),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn describe_leaves_out_the_payload_and_payload_returns_it() {
        let raw = b"\x01SERIAL-1234".to_vec();
        let errors = [
            WireError::payload_too_short("AvrStatus", 25, raw.len()).with_raw(&raw),
            WireError::unexpected_length("DspStatus", 129, raw.len()).with_raw(&raw),
        ];
        for e in &errors {
            assert!(e.to_string().contains("01534552"), "{e}");
            assert!(!e.describe().contains("01534552"), "{}", e.describe());
            assert!(!e.describe().contains("SERIAL"));
            assert_eq!(e.payload(), Some(raw.as_slice()));
        }
        let utf8 = String::from_utf8(vec![b'S', 0xFF]).unwrap_err();
        let e = WireError::InvalidString(utf8);
        assert!(!e.describe().contains("255"), "{}", e.describe());
        assert_eq!(e.payload(), Some(&[b'S', 0xFF][..]));
        let e = WireError::ChecksumMismatch {
            expected: 1,
            computed: 2,
        };
        assert_eq!(e.describe(), e.to_string());
        assert_eq!(e.payload(), None);
        assert_eq!(
            WireError::payload_too_short("AvrStatus", 25, 0).payload(),
            None
        );
    }
}
