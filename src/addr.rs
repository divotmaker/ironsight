use std::fmt;

use crate::error::{Result, WireError};

/// Bus addresses on the Mevo+ internal network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum BusAddr {
    /// Client application (phone, PC, our tool)
    App = 0x10,
    /// Raspberry Pi camera processor
    Pi = 0x12,
    /// AVR microcontroller (radar I/O, battery)
    Avr = 0x30,
    /// Digital signal processor (radar core)
    Dsp = 0x40,
}

impl BusAddr {
    pub fn from_byte(b: u8) -> Result<Self> {
        match b {
            0x10 => Ok(Self::App),
            0x12 => Ok(Self::Pi),
            0x30 => Ok(Self::Avr),
            0x40 => Ok(Self::Dsp),
            _ => Err(WireError::UnknownBusAddr { addr: b }),
        }
    }

    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

impl fmt::Display for BusAddr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::App => write!(f, "APP"),
            Self::Pi => write!(f, "PI"),
            Self::Avr => write!(f, "AVR"),
            Self::Dsp => write!(f, "DSP"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        for addr in [BusAddr::App, BusAddr::Pi, BusAddr::Avr, BusAddr::Dsp] {
            assert_eq!(BusAddr::from_byte(addr.as_byte()).unwrap(), addr);
        }
    }

    #[test]
    fn unknown_addr() {
        assert!(BusAddr::from_byte(0xFF).is_err());
    }
}
