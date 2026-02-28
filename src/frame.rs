//! Frame parsing, encoding, byte stuffing, and checksum.
//!
//! Wire format:
//! ```text
//! F0 [STUFFED( DEST SRC TYPE PAYLOAD... CS_HI CS_LO )] F1
//! ```

use crate::addr::BusAddr;
use crate::error::{Result, WireError};

const START: u8 = 0xF0;
const END: u8 = 0xF1;
const ESC: u8 = 0xFD;

/// A parsed frame with unstuffed header fields and payload.
#[derive(Debug, Clone)]
pub struct RawFrame {
    pub dest: BusAddr,
    pub src: BusAddr,
    pub type_id: u8,
    pub payload: Vec<u8>,
}

impl RawFrame {
    /// Parse a single complete wire frame (including F0 start and F1 end markers).
    pub fn parse(wire: &[u8]) -> Result<Self> {
        if wire.len() < 7 {
            return Err(WireError::FrameTooShort { len: wire.len() });
        }
        if wire[0] != START {
            return Err(WireError::MissingStart { got: wire[0] });
        }
        if wire[wire.len() - 1] != END {
            return Err(WireError::MissingEnd);
        }

        let interior = &wire[1..wire.len() - 1];

        // Unstuff the interior, tracking wire offsets for each decoded byte.
        let mut unstuffed: Vec<(u8, usize)> = Vec::with_capacity(interior.len());
        let mut i = 0;
        while i < interior.len() {
            if interior[i] == ESC {
                if i + 1 >= interior.len() {
                    return Err(WireError::InvalidEscape {
                        code: 0x00,
                        offset: i,
                    });
                }
                let decoded = match interior[i + 1] {
                    0x01 => 0xF0,
                    0x02 => 0xF1,
                    0x03 => 0xFD,
                    0x04 => 0xFA,
                    other => {
                        return Err(WireError::InvalidEscape {
                            code: other,
                            offset: i,
                        })
                    }
                };
                unstuffed.push((decoded, i));
                i += 2;
            } else {
                unstuffed.push((interior[i], i));
                i += 1;
            }
        }

        // Need at least 5 unstuffed bytes: DEST + SRC + TYPE + CS_HI + CS_LO
        if unstuffed.len() < 5 {
            return Err(WireError::FrameTooShort {
                len: wire.len(),
            });
        }

        // Last 2 unstuffed bytes are the checksum
        let n = unstuffed.len();
        let cs_received = ((unstuffed[n - 2].0 as u16) << 8) | unstuffed[n - 1].0 as u16;

        // Checksum is over the raw (stuffed) wire bytes before the checksum
        let data_end = unstuffed[n - 2].1;
        let cs_computed: u16 = interior[..data_end].iter().map(|&b| b as u16).sum();

        if cs_received != cs_computed {
            return Err(WireError::ChecksumMismatch {
                expected: cs_received,
                computed: cs_computed,
            });
        }

        let dest = BusAddr::from_byte(unstuffed[0].0)?;
        let src = BusAddr::from_byte(unstuffed[1].0)?;
        let type_id = unstuffed[2].0;
        let payload: Vec<u8> = unstuffed[3..n - 2].iter().map(|&(b, _)| b).collect();

        Ok(RawFrame {
            dest,
            src,
            type_id,
            payload,
        })
    }

    /// Encode this frame into a complete wire frame with byte stuffing and checksum.
    pub fn encode(&self) -> Vec<u8> {
        // Build the unstuffed interior: DEST SRC TYPE PAYLOAD
        let mut raw = Vec::with_capacity(3 + self.payload.len());
        raw.push(self.dest.as_byte());
        raw.push(self.src.as_byte());
        raw.push(self.type_id);
        raw.extend_from_slice(&self.payload);

        // Stuff the data portion
        let mut stuffed_data = stuff_bytes(&raw);

        // Compute checksum on the stuffed data bytes
        let cs: u16 = stuffed_data.iter().map(|&b| b as u16).sum();
        let cs_bytes = [(cs >> 8) as u8, (cs & 0xFF) as u8];
        let stuffed_cs = stuff_bytes(&cs_bytes);

        // Build the final wire frame
        let mut wire = Vec::with_capacity(2 + stuffed_data.len() + stuffed_cs.len());
        wire.push(START);
        wire.append(&mut stuffed_data);
        wire.extend_from_slice(&stuffed_cs);
        wire.push(END);
        wire
    }
}

/// Byte-stuff a slice: escape F0, F1, FD, FA.
fn stuff_bytes(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len());
    for &b in data {
        match b {
            0xF0 => {
                out.push(ESC);
                out.push(0x01);
            }
            0xF1 => {
                out.push(ESC);
                out.push(0x02);
            }
            0xFD => {
                out.push(ESC);
                out.push(0x03);
            }
            0xFA => {
                out.push(ESC);
                out.push(0x04);
            }
            _ => out.push(b),
        }
    }
    out
}

/// Splits a byte stream into individual frames. Buffers partial data across
/// calls, so it can be fed TCP segment boundaries.
pub struct FrameSplitter {
    buf: Vec<u8>,
}

impl FrameSplitter {
    pub fn new() -> Self {
        Self {
            buf: Vec::with_capacity(512),
        }
    }

    /// Feed new data and extract any complete frames.
    ///
    /// Returns a vector of raw wire frames (each starting with F0 and ending
    /// with F1). Partial frames are buffered for the next call.
    pub fn feed(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        self.buf.extend_from_slice(data);
        let mut frames = Vec::new();

        loop {
            // Find start marker
            let start = match self.buf.iter().position(|&b| b == START) {
                Some(pos) => pos,
                None => {
                    self.buf.clear();
                    break;
                }
            };

            // Discard any bytes before the start marker
            if start > 0 {
                self.buf.drain(..start);
            }

            // Find end marker (skip the start byte itself)
            let end = match self.buf[1..].iter().position(|&b| b == END) {
                Some(pos) => pos + 1, // adjust for the 1-offset
                None => break,        // incomplete frame
            };

            // Extract the complete frame (start..=end inclusive)
            let frame: Vec<u8> = self.buf[..=end].to_vec();
            self.buf.drain(..=end);
            frames.push(frame);
        }

        frames
    }
}

impl Default for FrameSplitter {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_worked_example() {
        // WIRE.md Section 1.2: STATUS poll APP→DSP, type 0xAA, payload [01 01]
        // Wire: F0 40 10 AA 01 01 00 FC F1
        let wire = [0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1];
        let frame = RawFrame::parse(&wire).unwrap();
        assert_eq!(frame.dest, BusAddr::Dsp);
        assert_eq!(frame.src, BusAddr::App);
        assert_eq!(frame.type_id, 0xAA);
        assert_eq!(frame.payload, vec![0x01, 0x01]);
    }

    #[test]
    fn encode_worked_example() {
        let frame = RawFrame {
            dest: BusAddr::Dsp,
            src: BusAddr::App,
            type_id: 0xAA,
            payload: vec![0x01, 0x01],
        };
        let wire = frame.encode();
        assert_eq!(
            wire,
            vec![0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1]
        );
    }

    #[test]
    fn round_trip() {
        let original = RawFrame {
            dest: BusAddr::Avr,
            src: BusAddr::App,
            type_id: 0xB0,
            payload: vec![0x01, 0x01],
        };
        let wire = original.encode();
        let parsed = RawFrame::parse(&wire).unwrap();
        assert_eq!(parsed.dest, original.dest);
        assert_eq!(parsed.src, original.src);
        assert_eq!(parsed.type_id, original.type_id);
        assert_eq!(parsed.payload, original.payload);
    }

    #[test]
    fn stuff_all_four_escapes() {
        // Payload containing all 4 escaped values
        let frame = RawFrame {
            dest: BusAddr::Dsp,
            src: BusAddr::App,
            type_id: 0x01,
            payload: vec![0xF0, 0xF1, 0xFD, 0xFA],
        };
        let wire = frame.encode();
        // Interior should have escape sequences for all 4 bytes
        assert!(wire.contains(&ESC));
        // Round-trip should recover the original payload
        let parsed = RawFrame::parse(&wire).unwrap();
        assert_eq!(parsed.payload, vec![0xF0, 0xF1, 0xFD, 0xFA]);
    }

    #[test]
    fn empty_payload() {
        let frame = RawFrame {
            dest: BusAddr::Avr,
            src: BusAddr::App,
            type_id: 0x69,
            payload: vec![],
        };
        let wire = frame.encode();
        let parsed = RawFrame::parse(&wire).unwrap();
        assert_eq!(parsed.type_id, 0x69);
        assert!(parsed.payload.is_empty());
    }

    #[test]
    fn bad_checksum() {
        let mut wire = vec![0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1];
        wire[7] = 0x00; // corrupt checksum low byte
        assert!(matches!(
            RawFrame::parse(&wire),
            Err(WireError::ChecksumMismatch { .. })
        ));
    }

    #[test]
    fn too_short() {
        assert!(matches!(
            RawFrame::parse(&[0xF0, 0xF1]),
            Err(WireError::FrameTooShort { .. })
        ));
    }

    #[test]
    fn frame_splitter_basic() {
        let mut splitter = FrameSplitter::new();
        let wire = [0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1];

        // Feed a complete frame
        let frames = splitter.feed(&wire);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], wire);
    }

    #[test]
    fn frame_splitter_partial() {
        let mut splitter = FrameSplitter::new();
        let wire = [0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1];

        // Feed in two chunks
        let frames = splitter.feed(&wire[..5]);
        assert!(frames.is_empty());
        let frames = splitter.feed(&wire[5..]);
        assert_eq!(frames.len(), 1);
    }

    #[test]
    fn frame_splitter_multiple() {
        let mut splitter = FrameSplitter::new();
        let frame1 = [0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1];
        let frame2 = [0xF0, 0x30, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xEC, 0xF1];
        let mut combined = Vec::new();
        combined.extend_from_slice(&frame1);
        combined.extend_from_slice(&frame2);

        let frames = splitter.feed(&combined);
        assert_eq!(frames.len(), 2);
    }

    #[test]
    fn frame_splitter_garbage_prefix() {
        let mut splitter = FrameSplitter::new();
        let mut data = vec![0x00, 0xFF, 0x42]; // garbage
        data.extend_from_slice(&[0xF0, 0x40, 0x10, 0xAA, 0x01, 0x01, 0x00, 0xFC, 0xF1]);
        let frames = splitter.feed(&data);
        assert_eq!(frames.len(), 1);
    }
}
