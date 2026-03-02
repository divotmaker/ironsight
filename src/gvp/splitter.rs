//! Null-byte frame splitter for the GVP JSON protocol (port 1258).
//!
//! Each JSON message is terminated by `\x00`. Multiple messages may arrive
//! in a single TCP segment. The splitter buffers partial messages across
//! calls, mirroring [`crate::frame::FrameSplitter`] for the binary protocol.

/// Splits a byte stream into individual null-terminated JSON messages.
///
/// Buffers partial data across calls so it can be fed TCP segment boundaries.
pub struct NullSplitter {
    buf: Vec<u8>,
}

impl NullSplitter {
    pub fn new() -> Self {
        Self {
            buf: Vec::with_capacity(4096),
        }
    }

    /// Feed new data and extract any complete messages.
    ///
    /// Returns a vector of UTF-8 strings, each a complete JSON message
    /// (without the trailing null byte). Partial messages are buffered
    /// for the next call.
    pub fn feed(&mut self, data: &[u8]) -> Vec<String> {
        self.buf.extend_from_slice(data);
        let mut messages = Vec::new();

        while let Some(null_pos) = self.buf.iter().position(|&b| b == 0x00) {
            // Extract everything before the null byte as a message.
            let raw = self.buf.drain(..=null_pos).collect::<Vec<u8>>();
            let json_bytes = &raw[..raw.len() - 1]; // strip trailing null

            // Skip empty segments (consecutive nulls or leading null).
            if json_bytes.is_empty() {
                continue;
            }

            match std::str::from_utf8(json_bytes) {
                Ok(s) => {
                    let trimmed = s.trim();
                    if !trimmed.is_empty() {
                        messages.push(trimmed.to_owned());
                    }
                }
                Err(_) => {
                    // Invalid UTF-8 — skip this segment. The GVP protocol
                    // is always valid UTF-8 JSON, so this is a corrupt frame.
                }
            }
        }

        messages
    }
}

impl Default for NullSplitter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_complete_message() {
        let mut splitter = NullSplitter::new();
        let data = b"{\"type\":\"STATUS\",\"version\":1}\x00";
        let msgs = splitter.feed(data);
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], "{\"type\":\"STATUS\",\"version\":1}");
    }

    #[test]
    fn multiple_messages_in_one_segment() {
        let mut splitter = NullSplitter::new();
        let data = b"{\"type\":\"LOG\"}\x00{\"type\":\"STATUS\"}\x00";
        let msgs = splitter.feed(data);
        assert_eq!(msgs.len(), 2);
        assert_eq!(msgs[0], "{\"type\":\"LOG\"}");
        assert_eq!(msgs[1], "{\"type\":\"STATUS\"}");
    }

    #[test]
    fn partial_message_across_feeds() {
        let mut splitter = NullSplitter::new();

        let msgs = splitter.feed(b"{\"type\":");
        assert!(msgs.is_empty());

        let msgs = splitter.feed(b"\"CONFIG\"}\x00");
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], "{\"type\":\"CONFIG\"}");
    }

    #[test]
    fn empty_segments_skipped() {
        let mut splitter = NullSplitter::new();
        let data = b"\x00\x00{\"type\":\"LOG\"}\x00\x00";
        let msgs = splitter.feed(data);
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], "{\"type\":\"LOG\"}");
    }

    #[test]
    fn no_null_terminator_buffers() {
        let mut splitter = NullSplitter::new();
        let msgs = splitter.feed(b"{\"type\":\"RESULT\",\"version\":1");
        assert!(msgs.is_empty());
        // Complete it in next feed
        let msgs = splitter.feed(b"}\x00");
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], "{\"type\":\"RESULT\",\"version\":1}");
    }

    #[test]
    fn whitespace_trimmed() {
        let mut splitter = NullSplitter::new();
        let data = b"  {\"type\":\"LOG\"}  \x00";
        let msgs = splitter.feed(data);
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], "{\"type\":\"LOG\"}");
    }

    #[test]
    fn large_message_spanning_multiple_feeds() {
        let mut splitter = NullSplitter::new();
        // Simulate a ~3200B RESULT message split across 4 TCP segments
        let json = format!(
            "{{\"type\":\"RESULT\",\"version\":1,\"data\":\"{}\"}}",
            "x".repeat(3000)
        );
        let bytes = json.as_bytes();

        let chunk_size = 1000;
        for chunk in bytes.chunks(chunk_size) {
            let msgs = splitter.feed(chunk);
            assert!(msgs.is_empty());
        }
        let msgs = splitter.feed(b"\x00");
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], json);
    }
}
