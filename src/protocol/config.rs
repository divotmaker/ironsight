//! Configuration and mode messages.

use crate::codec;
use crate::error::{Result, WireError};

// ---------------------------------------------------------------------------
// Detection mode constants (commsIndex values for 0xA5)
// ---------------------------------------------------------------------------

pub const MODE_INDOOR: u8 = 1;
pub const MODE_LONG_INDOOR: u8 = 2;
pub const MODE_PUTTING: u8 = 3;
pub const MODE_CLUB_SWING: u8 = 4;
pub const MODE_CHIPPING: u8 = 5;
pub const MODE_SIM_PUTTING: u8 = 6;
pub const MODE_OUTDOOR: u8 = 9;
pub const MODE_RAW_SAMPLING: u8 = 13;
pub const MODE_PUTTING_DEDICATED: u8 = 14;
pub const MODE_CHIP_IN: u8 = 15;
pub const MODE_CHIP_OUT: u8 = 16;

/// Set detection mode (3 bytes). Type 0xA5.
///
/// Payload: `[02 00 XX]` where XX is the commsIndex.
#[derive(Debug, Clone)]
pub struct ModeSet {
    /// commsIndex value (see MODE_* constants)
    pub mode: u8,
}

impl ModeSet {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 3 {
            return Err(WireError::payload_too_short("ModeSet", 3, payload.len()));
        }
        Ok(Self { mode: payload[2] })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x02, 0x00, self.mode]
    }
}

/// AVR configuration control (2 bytes). Type 0xB0.
///
/// - `[01 00]` = config exchange (pre-arm)
/// - `[01 01]` = arm trigger
#[derive(Debug, Clone)]
pub struct AvrConfigCmd {
    /// true = arm trigger, false = config exchange
    pub arm: bool,
}

impl AvrConfigCmd {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("AvrConfigCmd", 2, payload.len()));
        }
        Ok(Self {
            arm: payload[1] == 0x01,
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x01, if self.arm { 0x01 } else { 0x00 }]
    }
}

/// Parameter read request (4 bytes). Type 0xBE.
///
/// Format: `[03 00 00 XX]` where XX is the parameter ID low byte.
#[derive(Debug, Clone)]
pub struct ParamReadReq {
    pub param_id: u8,
}

impl ParamReadReq {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 4 {
            return Err(WireError::payload_too_short("ParamReadReq", 4, payload.len()));
        }
        Ok(Self {
            param_id: payload[3],
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x03, 0x00, 0x00, self.param_id]
    }
}

/// Parameter value — dual-use read response and write command. Type 0xBF.
///
/// INT24 format (7B): `[06 00 00 param_id val_hi val_mid val_lo]`
/// FLOAT40 format (9B): `[08 00 00 param_id exp_hi exp_lo mant_hi mant_mid mant_lo]`
#[derive(Debug, Clone)]
pub struct ParamValue {
    /// Parameter ID
    pub param_id: u8,
    /// Decoded value
    pub value: ParamData,
}

/// The data portion of a parameter value.
#[derive(Debug, Clone)]
pub enum ParamData {
    Int24(i32),
    Float40(f64),
}

impl ParamValue {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.is_empty() {
            return Err(WireError::payload_too_short("ParamValue", 1, 0));
        }
        match payload[0] {
            0x06 => {
                // INT24 format (7 bytes)
                if payload.len() < 7 {
                    return Err(WireError::payload_too_short("ParamValue(INT24)", 7, payload.len()));
                }
                Ok(Self {
                    param_id: payload[3],
                    value: ParamData::Int24(codec::read_int24(payload, 4)?),
                })
            }
            0x08 => {
                // FLOAT40 format (9 bytes)
                if payload.len() < 9 {
                    return Err(WireError::payload_too_short("ParamValue(FLOAT40)", 9, payload.len()));
                }
                Ok(Self {
                    param_id: payload[3],
                    value: ParamData::Float40(codec::read_float40(payload, 4)?),
                })
            }
            other => Err(WireError::unexpected_length("ParamValue", 6, other as usize)),
        }
    }

    pub fn encode(&self) -> Vec<u8> {
        match &self.value {
            ParamData::Int24(val) => {
                let mut buf = vec![0x06, 0x00, 0x00, self.param_id];
                codec::write_int24(&mut buf, *val);
                buf
            }
            ParamData::Float40(val) => {
                let mut buf = vec![0x08, 0x00, 0x00, self.param_id];
                codec::write_float40(&mut buf, *val);
                buf
            }
        }
    }
}

/// Radar calibration (7 bytes). Type 0xA4. Bidirectional.
///
/// Format: `[06 range_hi range_lo 00 height_mm 00 00]`
#[derive(Debug, Clone)]
pub struct RadarCal {
    /// Sensor-to-tee distance (mm)
    pub range_mm: u16,
    /// Surface height: `floor(height_inches * 25.4)` (mm)
    pub height_mm: u8,
}

impl RadarCal {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 7 {
            return Err(WireError::payload_too_short("RadarCal", 7, payload.len()));
        }
        Ok(Self {
            range_mm: codec::read_uint16(payload, 1)?,
            height_mm: payload[4],
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut buf = vec![0x06];
        codec::write_uint16(&mut buf, self.range_mm);
        buf.push(0x00);
        buf.push(self.height_mm);
        buf.push(0x00);
        buf.push(0x00);
        buf
    }
}

/// TParameters radar config response (69 bytes). Type 0xA0.
///
/// 34 INT16 parameters preceded by a 1-byte size field.
#[derive(Debug, Clone)]
pub struct ConfigResp {
    /// 34 radar configuration parameter values
    pub params: [i16; 34],
}

impl ConfigResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 69 {
            return Err(WireError::payload_too_short("ConfigResp", 69, payload.len()));
        }
        let mut params = [0i16; 34];
        for (i, p) in params.iter_mut().enumerate() {
            *p = codec::read_int16(payload, 1 + i * 2)?;
        }
        Ok(Self { params })
    }
}

/// AVR config response (17 bytes). Type 0xA2.
///
/// Contains version, gain factors, and config bytes.
/// Version byte at `payload[1]`: v1 = Mevo+ (Gen1), v2 = Mevo Gen2.
#[derive(Debug, Clone)]
pub struct AvrConfigResp {
    pub payload: Vec<u8>,
}

impl AvrConfigResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }

    /// Wire format version: 1 = Mevo+ (Gen1), 2 = Mevo Gen2.
    pub fn version(&self) -> u8 {
        self.payload.get(1).copied().unwrap_or(0)
    }
}
