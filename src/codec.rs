//! Field codecs for the Mevo+ wire protocol.
//!
//! All multi-byte integers are big-endian. Signed types use two's complement
//! with sign extension.

use crate::error::{Result, WireError};

// ---------------------------------------------------------------------------
// Read helpers
// ---------------------------------------------------------------------------

/// Read a big-endian signed 16-bit integer.
pub fn read_int16(data: &[u8], offset: usize) -> Result<i16> {
    check_len(data, offset, 2, "INT16")?;
    Ok(i16::from_be_bytes([data[offset], data[offset + 1]]))
}

/// Read a big-endian unsigned 16-bit integer.
pub fn read_uint16(data: &[u8], offset: usize) -> Result<u16> {
    check_len(data, offset, 2, "UINT16")?;
    Ok(u16::from_be_bytes([data[offset], data[offset + 1]]))
}

/// Read a big-endian signed 24-bit integer (sign-extended to i32).
pub fn read_int24(data: &[u8], offset: usize) -> Result<i32> {
    check_len(data, offset, 3, "INT24")?;
    let raw = ((data[offset] as u32) << 16) | ((data[offset + 1] as u32) << 8) | data[offset + 2] as u32;
    // Sign-extend from 24 bits
    Ok(if raw >= 0x80_0000 {
        raw as i32 - 0x100_0000
    } else {
        raw as i32
    })
}

/// Read a big-endian unsigned 24-bit integer.
pub fn read_uint24(data: &[u8], offset: usize) -> Result<u32> {
    check_len(data, offset, 3, "UINT24")?;
    Ok(((data[offset] as u32) << 16) | ((data[offset + 1] as u32) << 8) | data[offset + 2] as u32)
}

/// Read a big-endian signed 32-bit integer.
pub fn read_int32(data: &[u8], offset: usize) -> Result<i32> {
    check_len(data, offset, 4, "INT32")?;
    Ok(i32::from_be_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ]))
}

/// Read a big-endian unsigned 32-bit integer.
pub fn read_uint32(data: &[u8], offset: usize) -> Result<u32> {
    check_len(data, offset, 4, "UINT32")?;
    Ok(u32::from_be_bytes([
        data[offset],
        data[offset + 1],
        data[offset + 2],
        data[offset + 3],
    ]))
}

/// Decode a FLOAT40 (custom 40-bit floating point).
///
/// Wire layout: `[exp_hi, exp_lo, mant_hi, mant_mid, mant_lo]`
///
/// `value = mantissa * 2^(exponent - 23)`
///
/// where exponent is a signed 16-bit integer and mantissa is a signed 24-bit
/// integer. Zero is five zero bytes.
pub fn read_float40(data: &[u8], offset: usize) -> Result<f64> {
    check_len(data, offset, 5, "FLOAT40")?;
    let exp_raw = i16::from_be_bytes([data[offset], data[offset + 1]]);
    let mant_raw = read_int24(data, offset + 2)?;

    if mant_raw == 0 {
        return Ok(0.0);
    }

    // value = mantissa * 2^(exponent - 23)
    Ok(f64::from(mant_raw) * f64::exp2(f64::from(exp_raw) - 23.0))
}

// ---------------------------------------------------------------------------
// Scaled read helpers
// ---------------------------------------------------------------------------

/// Read INT24 and divide by a scale factor.
pub fn read_int24_scaled(data: &[u8], offset: usize, scale: f64) -> Result<f64> {
    Ok(f64::from(read_int24(data, offset)?) / scale)
}

/// Read INT16 and divide by a scale factor.
pub fn read_int16_scaled(data: &[u8], offset: usize, scale: f64) -> Result<f64> {
    Ok(f64::from(read_int16(data, offset)?) / scale)
}

// ---------------------------------------------------------------------------
// Write helpers
// ---------------------------------------------------------------------------

/// Write a big-endian signed 16-bit integer.
pub fn write_int16(buf: &mut Vec<u8>, val: i16) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write a big-endian unsigned 16-bit integer.
pub fn write_uint16(buf: &mut Vec<u8>, val: u16) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write a big-endian signed 24-bit integer (from i32, must fit in 24 bits).
pub fn write_int24(buf: &mut Vec<u8>, val: i32) {
    let bytes = val.to_be_bytes(); // [b3, b2, b1, b0]
    buf.extend_from_slice(&bytes[1..4]); // take low 3 bytes
}

/// Write a big-endian unsigned 24-bit integer.
pub fn write_uint24(buf: &mut Vec<u8>, val: u32) {
    let bytes = val.to_be_bytes();
    buf.extend_from_slice(&bytes[1..4]);
}

/// Write a big-endian signed 32-bit integer.
pub fn write_int32(buf: &mut Vec<u8>, val: i32) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write a big-endian unsigned 32-bit integer.
pub fn write_uint32(buf: &mut Vec<u8>, val: u32) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Encode a FLOAT40 (custom 40-bit floating point).
///
/// Emulates C `frexp()`: decomposes value into `frac * 2^exp` where
/// `frac` is in `[0.5, 1.0)`. We scale `frac` to a signed 23-bit mantissa.
pub fn write_float40(buf: &mut Vec<u8>, value: f64) {
    if value == 0.0 {
        buf.extend_from_slice(&[0, 0, 0, 0, 0]);
        return;
    }

    // Extract from f64 IEEE 754 bits: 1 sign + 11 exponent + 52 mantissa
    let bits = value.to_bits();
    let sign = (bits >> 63) != 0;
    let ieee_exp = ((bits >> 52) & 0x7FF) as i16;
    let ieee_mant = bits & 0x000F_FFFF_FFFF_FFFF;

    // frexp convention: frac in [0.5, 1.0), so frac = (1 + m/2^52) / 2
    // FLOAT40 mantissa = (int)(frac * 2^23) = (2^52 + m) >> 30
    // FLOAT40 exponent = ieee_exp - 1022 (frexp exponent)
    let mant_abs = ((1u64 << 52 | ieee_mant) >> 30) as i32;
    let mantissa = if sign { -mant_abs } else { mant_abs };
    let exponent = ieee_exp - 1022;

    buf.extend_from_slice(&exponent.to_be_bytes());
    write_int24(buf, mantissa);
}

// ---------------------------------------------------------------------------
// Internal
// ---------------------------------------------------------------------------

fn check_len(data: &[u8], offset: usize, need: usize, name: &'static str) -> Result<()> {
    if data.len() < offset + need {
        Err(WireError::payload_too_short(name, offset + need, data.len()))
    } else {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn int16_round_trip() {
        for val in [0i16, 1, -1, i16::MAX, i16::MIN, 0x7FFF, -0x8000] {
            let mut buf = Vec::new();
            write_int16(&mut buf, val);
            assert_eq!(read_int16(&buf, 0).unwrap(), val);
        }
    }

    #[test]
    fn int24_round_trip() {
        for val in [0i32, 1, -1, 0x7F_FFFF, -0x80_0000, 42, -42] {
            let mut buf = Vec::new();
            write_int24(&mut buf, val);
            assert_eq!(read_int24(&buf, 0).unwrap(), val);
        }
    }

    #[test]
    fn int24_sign_extension() {
        // 0xFF_FFFF should sign-extend to -1
        let data = [0xFF, 0xFF, 0xFF];
        assert_eq!(read_int24(&data, 0).unwrap(), -1);

        // 0x80_0000 should sign-extend to -8388608
        let data = [0x80, 0x00, 0x00];
        assert_eq!(read_int24(&data, 0).unwrap(), -0x80_0000);

        // 0x7F_FFFF should be positive
        let data = [0x7F, 0xFF, 0xFF];
        assert_eq!(read_int24(&data, 0).unwrap(), 0x7F_FFFF);
    }

    #[test]
    fn float40_zero() {
        let data = [0x00, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(read_float40(&data, 0).unwrap(), 0.0);

        let mut buf = Vec::new();
        write_float40(&mut buf, 0.0);
        assert_eq!(buf, vec![0, 0, 0, 0, 0]);
    }

    #[test]
    fn float40_one() {
        // 1.0 = 4194304 * 2^(1-23) = 4194304 * 2^-22
        // exp=1, mant=0x400000
        let data = [0x00, 0x01, 0x40, 0x00, 0x00];
        let val = read_float40(&data, 0).unwrap();
        assert!((val - 1.0).abs() < 1e-10, "expected 1.0, got {val}");

        let mut buf = Vec::new();
        write_float40(&mut buf, 1.0);
        assert_eq!(buf, vec![0x00, 0x01, 0x40, 0x00, 0x00]);
    }

    #[test]
    fn float40_12_5() {
        let data = [0x00, 0x04, 0x64, 0x00, 0x00];
        let val = read_float40(&data, 0).unwrap();
        assert!((val - 12.5).abs() < 1e-10, "expected 12.5, got {val}");
    }

    #[test]
    fn float40_negative() {
        // -2.3: exp=2, mant=-4823449 (0xB66667 sign-extended)
        let data = [0x00, 0x02, 0xB6, 0x66, 0x67];
        let val = read_float40(&data, 0).unwrap();
        assert!((val - (-2.3)).abs() < 1e-6, "expected -2.3, got {val}");
    }

    #[test]
    fn float40_100() {
        let data = [0x00, 0x07, 0x64, 0x00, 0x00];
        let val = read_float40(&data, 0).unwrap();
        assert!((val - 100.0).abs() < 1e-10, "expected 100.0, got {val}");
    }

    #[test]
    fn float40_0_0254() {
        // 0.0254: exp=-5 (0xFFFB), mant=6818274 (0x6809E2)
        let data = [0xFF, 0xFB, 0x68, 0x09, 0xE2];
        let val = read_float40(&data, 0).unwrap();
        assert!(
            (val - 0.0254).abs() < 1e-6,
            "expected 0.0254, got {val}"
        );
    }

    #[test]
    fn float40_round_trip() {
        for &val in &[1.0, -1.0, 12.5, 100.0, 0.0254, -2.3, 0.001, 999.999] {
            let mut buf = Vec::new();
            write_float40(&mut buf, val);
            let decoded = read_float40(&buf, 0).unwrap();
            let rel_err = ((decoded - val) / val).abs();
            assert!(
                rel_err < 1e-6,
                "round-trip failed for {val}: got {decoded} (rel err {rel_err})"
            );
        }
    }

    #[test]
    fn uint16_round_trip() {
        for val in [0u16, 1, 0xFFFF, 0x8000] {
            let mut buf = Vec::new();
            write_uint16(&mut buf, val);
            assert_eq!(read_uint16(&buf, 0).unwrap(), val);
        }
    }

    #[test]
    fn int32_round_trip() {
        for val in [0i32, 1, -1, i32::MAX, i32::MIN] {
            let mut buf = Vec::new();
            write_int32(&mut buf, val);
            assert_eq!(read_int32(&buf, 0).unwrap(), val);
        }
    }
}
