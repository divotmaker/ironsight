//! Status, keepalive, and acknowledgment messages.

use crate::codec;
use crate::error::{Result, WireError};

/// STATUS poll sent by APP to all nodes. Type 0xAA.
///
/// Payload: `[01 01]` for DSP/AVR, `[01 03]` for PI.
#[derive(Debug, Clone)]
pub struct StatusPoll {
    pub pi_mode: bool,
}

impl StatusPoll {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("StatusPoll", 2, payload.len()));
        }
        Ok(Self {
            pi_mode: payload[1] == 0x03,
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x01, if self.pi_mode { 0x03 } else { 0x01 }]
    }
}

/// AVR status response (25 bytes, SRC=0x30). Type 0xAA.
#[derive(Debug, Clone)]
pub struct AvrStatus {
    /// Version byte (0x18 = 24)
    pub version: u8,
    /// Radar state: 0=idle, 1=armed, 2=arming, 3=tracking
    pub state: u8,
    /// Hardware ID high byte
    pub hw_id_hi: u8,
    /// Hardware ID low byte
    pub hw_id_lo: u8,
    /// Full application ID
    pub full_app_id: i32,
    /// DSP temperature (degrees C)
    pub temperature: f64,
    /// Tilt angle (degrees)
    pub tilt: f64,
    /// Roll angle (degrees, sign negated vs display)
    pub roll: f64,
}

impl AvrStatus {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 25 {
            return Err(WireError::payload_too_short("AvrStatus", 25, payload.len()));
        }
        Ok(Self {
            version: payload[0],
            state: payload[1],
            hw_id_hi: payload[2],
            hw_id_lo: payload[5],
            full_app_id: codec::read_int24(payload, 8)?,
            temperature: codec::read_float40(payload, 10)?,
            tilt: codec::read_float40(payload, 15)?,
            roll: codec::read_float40(payload, 20)?,
        })
    }
}

/// DSP status response. Type 0xAA, SRC=0x40.
///
/// Two formats keyed by `payload[0]` (version byte):
/// - `0x80` (128): Gen1 Mevo+, 129 bytes. Fully decoded.
/// - `0x46` (70):  Gen2 Mevo,   71 bytes. Field layout differs; raw bytes preserved.
#[derive(Debug, Clone)]
pub enum DspStatus {
    /// Gen1 Mevo+ (version 0x80, 129 bytes). All fields decoded.
    V80(DspStatus80),
    /// Gen2 Mevo (version 0x46, 71 bytes). Raw payload preserved until field mapping is done.
    V46(DspStatus46),
}

impl DspStatus {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("DspStatus", 2, payload.len()));
        }
        match payload[0] {
            0x80 => Ok(DspStatus::V80(DspStatus80::decode(payload)?)),
            _ => Ok(DspStatus::V46(DspStatus46::decode(payload)?)),
        }
    }

    /// State byte (available on both variants).
    pub fn state(&self) -> u8 {
        match self {
            DspStatus::V80(s) => s.state,
            DspStatus::V46(s) => s.state,
        }
    }

    /// Battery percentage (0-100). Returns 0 for Gen2 (field offsets TBD).
    pub fn battery_percent(&self) -> u8 {
        match self {
            DspStatus::V80(s) => s.battery_percent(),
            DspStatus::V46(_) => 0,
        }
    }

    /// External power connected. Returns false for Gen2 (field offsets TBD).
    pub fn external_power(&self) -> bool {
        match self {
            DspStatus::V80(s) => s.external_power,
            DspStatus::V46(_) => false,
        }
    }

    /// Temperature in degrees C. Returns 0.0 for Gen2 (field offsets TBD).
    pub fn temperature_c(&self) -> f64 {
        match self {
            DspStatus::V80(s) => s.temperature_c(),
            DspStatus::V46(_) => 0.0,
        }
    }
}

/// Gen1 Mevo+ DSP status (version 0x80, 129 bytes).
#[derive(Debug, Clone)]
pub struct DspStatus80 {
    /// State byte
    pub state: u8,
    /// USB input voltage (mV, ~4900)
    pub input_voltage_usb: i16,
    /// System voltage (mV, ~3300)
    pub system_voltage: i16,
    /// Battery current (mA)
    pub battery_current: i16,
    /// Temperature / 100 (degrees C)
    pub temperature_raw: i16,
    /// Battery voltage (mV)
    pub battery_voltage: i16,
    /// Battery voltage 2 (mV)
    pub battery_voltage_2: i16,
    /// Power level (high byte = 0-100%)
    pub power_level: i16,
    /// External power connected (boolean)
    pub external_power: bool,
}

impl DspStatus80 {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 64 {
            return Err(WireError::payload_too_short("DspStatus80", 64, payload.len()));
        }
        Ok(Self {
            state: payload[1],
            input_voltage_usb: codec::read_int16(payload, 4)?,
            system_voltage: codec::read_int16(payload, 8)?,
            battery_current: codec::read_int16(payload, 18)?,
            temperature_raw: codec::read_int16(payload, 40)?,
            battery_voltage: codec::read_int16(payload, 53)?,
            battery_voltage_2: codec::read_int16(payload, 57)?,
            power_level: codec::read_int16(payload, 61)?,
            external_power: payload[63] != 0,
        })
    }

    /// Temperature in degrees C.
    pub fn temperature_c(&self) -> f64 {
        f64::from(self.temperature_raw) / 100.0
    }

    /// Battery percentage (0-100).
    pub fn battery_percent(&self) -> u8 {
        (self.power_level >> 8) as u8
    }
}

/// Gen2 Mevo DSP status (version 0x46, 71 bytes). Field layout TBD.
#[derive(Clone)]
pub struct DspStatus46 {
    /// State byte
    pub state: u8,
    /// Version byte (0x46)
    pub version: u8,
    /// Raw payload bytes for future decoding
    pub payload: Vec<u8>,
}

impl DspStatus46 {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            state: payload[1],
            version: payload[0],
            payload: payload.to_vec(),
        })
    }
}

impl std::fmt::Debug for DspStatus46 {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "DspStatus46 {{ version: 0x{:02X}, state: 0x{:02X}, len: {}, payload: ",
            self.version, self.state, self.payload.len(),
        )?;
        for (i, b) in self.payload.iter().enumerate() {
            if i > 0 {
                write!(f, " ")?;
            }
            write!(f, "{b:02X}")?;
        }
        write!(f, " }}")
    }
}

/// PI status response (raw bytes, SRC=0x12). Type 0xAA.
///
/// Not fully decoded; stored as raw payload for forward compatibility.
#[derive(Debug, Clone)]
pub struct PiStatus {
    pub payload: Vec<u8>,
}

impl PiStatus {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }
}

/// Generic command acknowledgment (3 bytes). Type 0x95.
///
/// Format: `[02 bus_addr acked_cmd]`
#[derive(Debug, Clone)]
pub struct ConfigAck {
    /// Responding subsystem (0x30=AVR, 0x12=PI)
    pub bus_addr: u8,
    /// Low 7 bits of the acknowledged command
    pub acked_cmd: u8,
}

impl ConfigAck {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 3 {
            return Err(WireError::payload_too_short("ConfigAck", 3, payload.len()));
        }
        Ok(Self {
            bus_addr: payload[1],
            acked_cmd: payload[2],
        })
    }
}

/// Mode reset acknowledgment (3 bytes). Type 0xB1. Always `[02 00 00]`.
#[derive(Debug, Clone)]
pub struct ModeAck;

impl ModeAck {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 3 {
            return Err(WireError::payload_too_short("ModeAck", 3, payload.len()));
        }
        Ok(Self)
    }
}

/// ASCII debug/log message from device subsystems. Type 0xE3.
#[derive(Debug, Clone)]
pub struct Text {
    pub text: String,
}

impl Text {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        // Strip trailing nulls and control chars
        let end = payload
            .iter()
            .rposition(|&b| b >= 0x20)
            .map_or(0, |p| p + 1);
        // Strip leading control chars
        let start = payload[..end]
            .iter()
            .position(|&b| b >= 0x20)
            .unwrap_or(0);
        Ok(Self {
            text: String::from_utf8_lossy(&payload[start..end]).into_owned(),
        })
    }
}
