//! Handshake request/response messages.

use crate::codec;
use crate::error::{Result, WireError};

/// Decode a null-terminated C string from a fixed-width slot.
fn decode_cstr(slot: &[u8]) -> String {
    let end = slot.iter().position(|&b| b == 0).unwrap_or(slot.len());
    String::from_utf8_lossy(&slot[..end]).into_owned()
}

/// Device hardware generation, detected from the C8 `dspType` byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceGen {
    /// Mevo+ (dspType 0x80)
    MevoPlus,
    /// Mevo Gen2 (dspType 0xC0)
    Gen2,
    /// Unknown hardware (stores raw dspType byte)
    Unknown(u8),
}

impl DeviceGen {
    pub fn from_dsp_type(dsp_type: u8) -> Self {
        match dsp_type {
            0x80 => DeviceGen::MevoPlus,
            0xC0 => DeviceGen::Gen2,
            other => DeviceGen::Unknown(other),
        }
    }

    /// Short label for logging and display.
    pub fn label(&self) -> &'static str {
        match self {
            DeviceGen::MevoPlus => "Mevo+",
            DeviceGen::Gen2 => "Mevo Gen2",
            DeviceGen::Unknown(_) => "Unknown",
        }
    }
}

impl std::fmt::Display for DeviceGen {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DeviceGen::MevoPlus => write!(f, "Mevo+ (0x80)"),
            DeviceGen::Gen2 => write!(f, "Mevo Gen2 (0xC0)"),
            DeviceGen::Unknown(v) => write!(f, "Unknown (0x{v:02X})"),
        }
    }
}

/// DSP query response (3 bytes). Type 0xC8.
///
/// `[version, dspType, pcb]` — Mevo+: `[02 80 0E]`, Gen2: `[02 C0 0E]`.
#[derive(Debug, Clone)]
pub struct DspQueryResp {
    pub version: u8,
    /// DSP type (0x80 = Mevo+, 0xC0 = Gen2)
    pub dsp_type: u8,
    /// PCB revision (0x0E = 14)
    pub pcb: u8,
}

impl DspQueryResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 3 {
            return Err(WireError::payload_too_short("DspQueryResp", 3, payload.len()));
        }
        Ok(Self {
            version: payload[0],
            dsp_type: payload[1],
            pcb: payload[2],
        })
    }

    /// Detect device generation from the dspType byte.
    pub fn device_gen(&self) -> DeviceGen {
        DeviceGen::from_dsp_type(self.dsp_type)
    }
}

/// Device info response (75-76 bytes). Type 0xE7.
///
/// Binary header followed by three 16-byte null-terminated text slots.
/// AVR/PI (75B): header = [1..26], slots at 27/43/59.
/// DSP   (76B): header = [1..27], slots at 28/44/60.
/// Slot contents vary by bus:
///   DSP: version, serial, firmware tag
///   AVR: version + model, build date, build time
///   PI:  firmware rev, build date, build time
#[derive(Debug, Clone)]
pub struct DevInfoResp {
    /// Concatenation of all non-empty text slots, separated by spaces.
    pub text: String,
}

impl DevInfoResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        // byte[0] = len(payload) - 1.  DSP: 76B (len=75), AVR/PI: 75B (len=74).
        let slot_start = if payload.len() >= 76 { 28 } else { 27 };
        let mut parts = Vec::new();
        for i in 0..3 {
            let offset = slot_start + i * 16;
            if offset + 16 <= payload.len() {
                let s = decode_cstr(&payload[offset..offset + 16]);
                if !s.is_empty() {
                    parts.push(s);
                }
            }
        }
        Ok(Self {
            text: parts.join(" "),
        })
    }
}

/// Product info request (2 bytes). Type 0xFD (APP→DSP).
///
/// Payload: `[01 XX]` where XX is sub-query (0x00, 0x08, 0x09).
#[derive(Debug, Clone)]
pub struct ProdInfoReq {
    pub sub_query: u8,
}

impl ProdInfoReq {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("ProdInfoReq", 2, payload.len()));
        }
        Ok(Self {
            sub_query: payload[1],
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x01, self.sub_query]
    }
}

/// Product info response (34 bytes ASCII). Type 0xFD (DSP→APP).
///
/// Contains Pi hardware ID, camera model.
#[derive(Debug, Clone)]
pub struct ProdInfoResp {
    pub text: String,
}

impl ProdInfoResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self { text: decode_cstr(payload) })
    }
}

/// Network config request (2 bytes). Type 0xDE (APP→PI).
///
/// `[01 00]` = SSID, `[01 08]` = password.
#[derive(Debug, Clone)]
pub struct NetConfigReq {
    pub query_password: bool,
}

impl NetConfigReq {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("NetConfigReq", 2, payload.len()));
        }
        Ok(Self {
            query_password: payload[1] == 0x08,
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x01, if self.query_password { 0x08 } else { 0x00 }]
    }
}

/// Network config response (54 bytes). Type 0xDE (PI→APP).
///
/// Layout: [0-1] length, [2-5] IP, [6-9] netmask, [10-20] binary,
///         [21-36] 16B SSID slot, [37-52] 16B password slot, [53] flags.
/// The SSID query (sub 0x00) returns IP/mask but empty text slots.
/// The password query (sub 0x08) returns both SSID and password.
#[derive(Debug, Clone)]
pub struct NetConfigResp {
    pub text: String,
}

impl NetConfigResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        // Extract the first non-empty text slot (SSID at 21, password at 37).
        // Callers distinguish SSID vs password by which sub-query was sent.
        let mut parts = Vec::new();
        for offset in [21, 37] {
            if offset + 16 <= payload.len() {
                let s = decode_cstr(&payload[offset..offset + 16]);
                if !s.is_empty() {
                    parts.push(s);
                }
            }
        }
        Ok(Self {
            text: parts.join("\0"),
        })
    }
}

/// IF calibration parameter request (3 bytes). Type 0xD0.
///
/// Always `[02 00 08]`.
#[derive(Debug, Clone)]
pub struct CalParamReq;

impl CalParamReq {
    pub fn decode(_payload: &[u8]) -> Result<Self> {
        Ok(Self)
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x02, 0x00, 0x08]
    }
}

/// IF calibration parameter response (242 bytes). Type 0xD1.
///
/// Contains calibrator name, date, and INT16 gain/offset arrays.
/// Stored as raw payload.
#[derive(Debug, Clone)]
pub struct CalParamResp {
    pub payload: Vec<u8>,
}

impl CalParamResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }
}

/// Calibration data request. Type 0xD2.
///
/// Two sub-commands:
/// - Sub-cmd 0x03 (10B): factory calibration info
/// - Sub-cmd 0x07 (10B): post-shot parameter dump
#[derive(Debug, Clone)]
pub struct CalDataReq {
    pub sub_cmd: u8,
    pub payload: Vec<u8>,
}

impl CalDataReq {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 4 {
            return Err(WireError::payload_too_short("CalDataReq", 4, payload.len()));
        }
        Ok(Self {
            sub_cmd: payload[3],
            payload: payload.to_vec(),
        })
    }

    /// Sub-cmd 0x03: factory cal request (handshake).
    pub fn encode_factory() -> Vec<u8> {
        vec![0x09, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA5]
    }

    /// Sub-cmd 0x07: post-shot parameter dump request.
    pub fn encode_post_shot() -> Vec<u8> {
        vec![0x09, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    }
}

/// Calibration data response. Type 0xD3.
///
/// Variable length, paginated for sub-cmd 0x07. Stored as raw payload.
#[derive(Debug, Clone)]
pub struct CalDataResp {
    pub payload: Vec<u8>,
}

impl CalDataResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }
}

/// Time synchronization (9 bytes). Type 0x9B.
///
/// Format: `[08 00 epoch(4B BE) session_byte dir_hi dir_lo]`
#[derive(Debug, Clone)]
pub struct TimeSync {
    /// Unix epoch timestamp (seconds)
    pub epoch: u32,
    /// Session byte
    pub session: u8,
    /// Direction-specific tail bytes
    pub tail: [u8; 2],
}

impl TimeSync {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 9 {
            return Err(WireError::payload_too_short("TimeSync", 9, payload.len()));
        }
        Ok(Self {
            epoch: codec::read_uint32(payload, 2)?,
            session: payload[6],
            tail: [payload[7], payload[8]],
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut buf = vec![0x08, 0x00];
        codec::write_uint32(&mut buf, self.epoch);
        buf.push(self.session);
        buf.extend_from_slice(&self.tail);
        buf
    }
}
