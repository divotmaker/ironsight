//! Shot result messages (AVR → APP).

use crate::codec;
use crate::error::{Result, WireError};

// ---------------------------------------------------------------------------
// 0xD4 — FLIGHT_RESULT (158 bytes)
// ---------------------------------------------------------------------------

/// Primary ball flight result. 1 per shot. Type 0xD4.
///
/// Contains 36 linear scalar fields plus a polynomial scale factor and 15
/// polynomial trajectory coefficients (5th-order, x/y/z).
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct FlightResult {
    /// Shot counter
    pub total: i32,
    /// Tracked time (s)
    pub track_time: f64,
    /// Start position [forward, vertical, lateral] (m)
    pub start_position: [f64; 3],
    /// Launch speed (m/s)
    pub launch_speed: f64,
    /// Launch azimuth (deg, neg = right)
    pub launch_azimuth: f64,
    /// Launch elevation / VLA (deg)
    pub launch_elevation: f64,
    /// Carry distance (m)
    pub carry_distance: f64,
    /// Flight time (s)
    pub flight_time: f64,
    /// Maximum height (m)
    pub max_height: f64,
    /// Landing position [forward, vertical, lateral] (m)
    pub landing_position: [f64; 3],
    /// Backspin (RPM)
    pub backspin_rpm: i32,
    /// Sidespin (RPM)
    pub sidespin_rpm: i32,
    /// Riflespin (RPM)
    pub riflespin_rpm: i32,
    /// Landing spin [0, 1, 2] (RPM)
    pub landing_spin_rpm: [i32; 3],
    /// Landing velocity [forward, vertical, lateral] (m/s)
    pub landing_velocity: [f64; 3],
    /// Total distance (m) — not populated by DSP; contains diagnostic values.
    pub total_distance: f64,
    /// Roll distance (m) — not populated by DSP; always zero on wire.
    pub roll_distance: f64,
    /// Final position [forward, vertical, lateral] (m) — not populated by DSP; always (0,0,0).
    pub final_position: [f64; 3],
    /// Club head speed (m/s)
    pub clubhead_speed: f64,
    /// Club strike direction (deg)
    pub club_strike_direction: f64,
    /// Club attack angle (deg)
    pub club_attack_angle: f64,
    /// Club head speed post-impact (m/s)
    pub clubhead_speed_post: f64,
    /// Club swing plane tilt (deg)
    pub club_swing_plane_tilt: f64,
    /// Club swing plane rotation (deg)
    pub club_swing_plane_rotation: f64,
    /// Club effective loft (deg)
    pub club_effective_loft: f64,
    /// Club face angle (deg)
    pub club_face_angle: f64,
    /// Polynomial scale factor (raw integer, divide poly coefficients by this)
    pub poly_scale: i32,
    /// Trajectory polynomial X coefficients [0..4]
    pub poly_x: [f64; 5],
    /// Trajectory polynomial Y coefficients [0..4]
    pub poly_y: [f64; 5],
    /// Trajectory polynomial Z coefficients [0..4]
    pub poly_z: [f64; 5],
}

impl FlightResult {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 157 {
            return Err(WireError::payload_too_short("FlightResult", 157, payload.len()));
        }

        let poly_scale = codec::read_int24(payload, 109)?;
        let ps = if poly_scale == 0 {
            1.0
        } else {
            f64::from(poly_scale)
        };

        let mut poly_x = [0.0; 5];
        let mut poly_y = [0.0; 5];
        let mut poly_z = [0.0; 5];
        for i in 0..5 {
            poly_x[i] = f64::from(codec::read_int24(payload, 112 + i * 3)?) / ps;
            poly_y[i] = f64::from(codec::read_int24(payload, 127 + i * 3)?) / ps;
            poly_z[i] = f64::from(codec::read_int24(payload, 142 + i * 3)?) / ps;
        }

        Ok(Self {
            total: codec::read_int24(payload, 1)?,
            track_time: codec::read_int24_scaled(payload, 4, 1000.0)?,
            start_position: [
                codec::read_int24_scaled(payload, 7, 1000.0)?,
                codec::read_int24_scaled(payload, 10, 1000.0)?,
                codec::read_int24_scaled(payload, 13, 1000.0)?,
            ],
            launch_speed: codec::read_int24_scaled(payload, 16, 1000.0)?,
            launch_azimuth: codec::read_int24_scaled(payload, 19, 1000.0)?,
            launch_elevation: codec::read_int24_scaled(payload, 22, 1000.0)?,
            carry_distance: codec::read_int24_scaled(payload, 25, 1000.0)?,
            flight_time: codec::read_int24_scaled(payload, 28, 1000.0)?,
            max_height: codec::read_int24_scaled(payload, 31, 1000.0)?,
            landing_position: [
                codec::read_int24_scaled(payload, 34, 1000.0)?,
                codec::read_int24_scaled(payload, 37, 1000.0)?,
                codec::read_int24_scaled(payload, 40, 1000.0)?,
            ],
            backspin_rpm: codec::read_int24(payload, 43)?,
            sidespin_rpm: codec::read_int24(payload, 46)?,
            riflespin_rpm: codec::read_int24(payload, 49)?,
            landing_spin_rpm: [
                codec::read_int24(payload, 52)?,
                codec::read_int24(payload, 55)?,
                codec::read_int24(payload, 58)?,
            ],
            landing_velocity: [
                codec::read_int24_scaled(payload, 61, 1000.0)?,
                codec::read_int24_scaled(payload, 64, 1000.0)?,
                codec::read_int24_scaled(payload, 67, 1000.0)?,
            ],
            total_distance: codec::read_int24_scaled(payload, 70, 1000.0)?,
            roll_distance: codec::read_int24_scaled(payload, 73, 1000.0)?,
            final_position: [
                codec::read_int24_scaled(payload, 76, 1000.0)?,
                codec::read_int24_scaled(payload, 79, 1000.0)?,
                codec::read_int24_scaled(payload, 82, 1000.0)?,
            ],
            clubhead_speed: codec::read_int24_scaled(payload, 85, 1000.0)?,
            club_strike_direction: codec::read_int24_scaled(payload, 88, 1000.0)?,
            club_attack_angle: codec::read_int24_scaled(payload, 91, 1000.0)?,
            clubhead_speed_post: codec::read_int24_scaled(payload, 94, 1000.0)?,
            club_swing_plane_tilt: codec::read_int24_scaled(payload, 97, 1000.0)?,
            club_swing_plane_rotation: codec::read_int24_scaled(payload, 100, 1000.0)?,
            club_effective_loft: codec::read_int24_scaled(payload, 103, 1000.0)?,
            club_face_angle: codec::read_int24_scaled(payload, 106, 1000.0)?,
            poly_scale,
            poly_x,
            poly_y,
            poly_z,
        })
    }
}

// ---------------------------------------------------------------------------
// 0xE8 — FLIGHT_RESULT_V1 (94 bytes)
// ---------------------------------------------------------------------------

/// Early/partial flight result. 1 per shot, sent before D4. Type 0xE8.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct FlightResultV1 {
    /// Shot counter
    pub total: i32,
    /// Club velocity (m/s)
    pub club_velocity: f64,
    /// Ball velocity (m/s)
    pub ball_velocity: f64,
    /// Flight time (s)
    pub flight_time: f64,
    /// Distance (m)
    pub distance: f64,
    /// Height (m)
    pub height: f64,
    /// Lateral (m)
    pub lateral: f64,
    /// Elevation / VLA (deg)
    pub elevation: f64,
    /// Azimuth / HLA (deg)
    pub azimuth: f64,
    /// Tracked time (s)
    pub tracked_time: f64,
    /// Drag coefficient (/1000000)
    pub drag: f64,
    /// Backspin (RPM)
    pub backspin_rpm: i32,
    /// Sidespin (RPM)
    pub sidespin_rpm: i32,
    /// Acceleration
    pub acceleration: f64,
    /// Club strike direction (deg)
    pub club_strike_direction: f64,
    /// Polynomial scale factor
    pub poly_scale: i32,
    /// Trajectory polynomial X coefficients [0..4]
    pub poly_x: [f64; 5],
    /// Trajectory polynomial Y coefficients [0..4]
    pub poly_y: [f64; 5],
    /// Trajectory polynomial Z coefficients [0..4]
    pub poly_z: [f64; 5],
}

impl FlightResultV1 {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 94 {
            return Err(WireError::payload_too_short("FlightResultV1", 94, payload.len()));
        }

        let poly_scale = codec::read_int24(payload, 46)?;
        let ps = if poly_scale == 0 {
            1.0
        } else {
            f64::from(poly_scale).max(1.0)
        };

        let mut poly_x = [0.0; 5];
        let mut poly_y = [0.0; 5];
        let mut poly_z = [0.0; 5];
        for i in 0..5 {
            poly_x[i] = f64::from(codec::read_int24(payload, 49 + i * 3)?) / ps;
            poly_y[i] = f64::from(codec::read_int24(payload, 64 + i * 3)?) / ps;
            poly_z[i] = f64::from(codec::read_int24(payload, 79 + i * 3)?) / ps;
        }

        Ok(Self {
            total: codec::read_int24(payload, 1)?,
            club_velocity: codec::read_int24_scaled(payload, 4, 1000.0)?,
            ball_velocity: codec::read_int24_scaled(payload, 7, 1000.0)?,
            flight_time: codec::read_int24_scaled(payload, 10, 1000.0)?,
            distance: codec::read_int24_scaled(payload, 13, 1000.0)?,
            height: codec::read_int24_scaled(payload, 16, 1000.0)?,
            lateral: codec::read_int24_scaled(payload, 19, 1000.0)?,
            elevation: codec::read_int24_scaled(payload, 22, 1000.0)?,
            azimuth: codec::read_int24_scaled(payload, 25, 1000.0)?,
            tracked_time: codec::read_int24_scaled(payload, 28, 1000.0)?,
            drag: codec::read_int24_scaled(payload, 31, 1_000_000.0)?,
            backspin_rpm: codec::read_int24(payload, 34)?,
            sidespin_rpm: codec::read_int24(payload, 37)?,
            acceleration: codec::read_int24_scaled(payload, 40, 1000.0)?,
            club_strike_direction: codec::read_int24_scaled(payload, 43, 1000.0)?,
            poly_scale,
            poly_x,
            poly_y,
            poly_z,
        })
    }
}

// ---------------------------------------------------------------------------
// 0xED — CLUB_RESULT (167-172 bytes)
// ---------------------------------------------------------------------------

/// Club head measurements. Appears 2x per shot (duplicate). Type 0xED.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct ClubResult {
    /// Number of club PRC tracking points
    pub num_club_prc_points: u8,
    /// Flags (raw)
    pub flags: i32,
    /// Pre-impact club speed (m/s)
    pub pre_club_speed: f64,
    /// Post-impact club speed (m/s)
    pub post_club_speed: f64,
    /// Strike direction / path angle (deg)
    pub strike_direction: f64,
    /// Attack angle (deg)
    pub attack_angle: f64,
    /// Face angle (deg)
    pub face_angle: f64,
    /// Dynamic loft (deg)
    pub dynamic_loft: f64,
    /// Smash factor (ratio)
    pub smash_factor: f64,
    /// Dispersion correction
    pub dispersion_correction: f64,
    /// Swing plane horizontal (deg)
    pub swing_plane_horizontal: f64,
    /// Swing plane vertical (deg)
    pub swing_plane_vertical: f64,
    /// Club azimuth (deg)
    pub club_azimuth: f64,
    /// Club elevation (deg)
    pub club_elevation: f64,
    /// Club offset (m)
    pub club_offset: f64,
    /// Club height (m)
    pub club_height: f64,
    /// Polynomial scale factor
    pub poly_scale: i32,
    /// 12 polynomial arrays, each with 3 coefficients
    /// Order: Pre_v, Pst_v, Pre_x, Pst_x, Pre_y, Pst_y, Pre_z, Pst_z,
    ///        Pre_YX, Pst_YX, Pre_ZX, Pst_ZX
    pub poly_coeffs: [[f64; 3]; 12],
    /// Pre-impact time (ms, /100)
    pub pre_impact_time: f64,
    /// Post-impact time (ms, /100)
    pub post_impact_time: f64,
    /// Club-to-ball time (ms, /100)
    pub club_to_ball_time: f64,
}

impl ClubResult {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 167 {
            return Err(WireError::payload_too_short("ClubResult", 167, payload.len()));
        }

        let poly_scale = codec::read_int24(payload, 47)?;
        let ps = if poly_scale == 0 {
            1.0
        } else {
            f64::from(poly_scale)
        };

        let mut poly_coeffs = [[0.0; 3]; 12];
        for (arr, coeffs) in poly_coeffs.iter_mut().enumerate() {
            for (coeff, val) in coeffs.iter_mut().enumerate() {
                let offset = 50 + arr * 9 + coeff * 3;
                *val = f64::from(codec::read_int24(payload, offset)?) / ps;
            }
        }

        // Timing fields start at offset 158
        let has_timing = payload.len() >= 167;

        Ok(Self {
            num_club_prc_points: payload[1],
            flags: codec::read_int24(payload, 2)?,
            pre_club_speed: codec::read_int24_scaled(payload, 5, 100.0)?,
            post_club_speed: codec::read_int24_scaled(payload, 8, 100.0)?,
            strike_direction: codec::read_int24_scaled(payload, 11, 100.0)?,
            attack_angle: codec::read_int24_scaled(payload, 14, 100.0)?,
            face_angle: codec::read_int24_scaled(payload, 17, 100.0)?,
            dynamic_loft: codec::read_int24_scaled(payload, 20, 100.0)?,
            smash_factor: codec::read_int24_scaled(payload, 23, 1000.0)?,
            dispersion_correction: codec::read_int24_scaled(payload, 26, 1000.0)?,
            swing_plane_horizontal: codec::read_int24_scaled(payload, 29, 100.0)?,
            swing_plane_vertical: codec::read_int24_scaled(payload, 32, 100.0)?,
            club_azimuth: codec::read_int24_scaled(payload, 35, 100.0)?,
            club_elevation: codec::read_int24_scaled(payload, 38, 100.0)?,
            club_offset: codec::read_int24_scaled(payload, 41, 1000.0)?,
            club_height: codec::read_int24_scaled(payload, 44, 1000.0)?,
            poly_scale,
            poly_coeffs,
            pre_impact_time: if has_timing {
                codec::read_int24_scaled(payload, 158, 100.0)?
            } else {
                0.0
            },
            post_impact_time: if has_timing {
                codec::read_int24_scaled(payload, 161, 100.0)?
            } else {
                0.0
            },
            club_to_ball_time: if has_timing {
                codec::read_int24_scaled(payload, 164, 100.0)?
            } else {
                0.0
            },
        })
    }
}

// ---------------------------------------------------------------------------
// 0xEF — SPIN_RESULT (138 bytes)
// ---------------------------------------------------------------------------

/// Antenna array element (7 bytes) within SpinResult.
#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct AntennaElement {
    /// Spin estimate (RPM)
    pub spin_rpm: i16,
    /// Signal peak strength
    pub peak: f64,
    /// Signal-to-noise ratio
    pub snr: i16,
}

/// Spin measurement data. 1 per shot. Type 0xEF.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct SpinResult {
    /// Version/length byte (0x89 = 137)
    pub version: u8,
    /// 5 antenna groups x 3 range bins (15 elements)
    pub antenna_data: [[AntennaElement; 3]; 5],
    /// PM spin raw (RPM)
    pub pm_spin_raw: i16,
    /// PM spin final — authoritative total spin (RPM)
    pub pm_spin_final: i16,
    /// PM spin confidence (0-100)
    pub pm_spin_confidence: i16,
    /// Lift spin (RPM)
    pub lift_spin: i16,
    /// Expected spin for validation (RPM)
    pub spin_validate_expected: i16,
    /// Low limit for validation (RPM)
    pub spin_validate_low: i16,
    /// High limit for validation (RPM)
    pub spin_validate_high: i16,
    /// Validation scaling factor
    pub spin_validate_scaling: i16,
    /// Spin algorithm selector
    pub spin_method: u8,
    /// Spin flags (raw 24-bit)
    pub spin_flags: i32,
    /// Launch spin (RPM)
    pub launch_spin: i16,
    /// AM spin (RPM)
    pub am_spin: i16,
    /// PM spin (RPM)
    pub pm_spin: i16,
    /// Spin axis angle (deg, /10)
    pub spin_axis: f64,
    /// AOD spin (RPM)
    pub aod_spin: i16,
    /// PLL spin (RPM)
    pub pll_spin: i16,
}

impl SpinResult {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 138 {
            return Err(WireError::payload_too_short("SpinResult", 138, payload.len()));
        }

        let version = payload[0];

        // Decode 5x3 antenna arrays (105 bytes at [1-105])
        let mut antenna_data = std::array::from_fn::<_, 5, _>(|_| {
            std::array::from_fn::<_, 3, _>(|_| AntennaElement::default())
        });
        for (group, bins) in antenna_data.iter_mut().enumerate() {
            for (bin, elem) in bins.iter_mut().enumerate() {
                let base = 1 + (group * 3 + bin) * 7;
                *elem = AntennaElement {
                    spin_rpm: codec::read_int16(payload, base)?,
                    peak: codec::read_int24_scaled(payload, base + 2, 1000.0)?,
                    snr: codec::read_int16(payload, base + 5)?,
                };
            }
        }

        Ok(Self {
            version,
            antenna_data,
            pm_spin_raw: codec::read_int16(payload, 106)?,
            pm_spin_final: codec::read_int16(payload, 108)?,
            pm_spin_confidence: codec::read_int16(payload, 110)?,
            lift_spin: codec::read_int16(payload, 112)?,
            spin_validate_expected: codec::read_int16(payload, 114)?,
            spin_validate_low: codec::read_int16(payload, 116)?,
            spin_validate_high: codec::read_int16(payload, 118)?,
            spin_validate_scaling: codec::read_int16(payload, 120)?,
            spin_method: payload[122],
            spin_flags: codec::read_int24(payload, 123)?,
            launch_spin: codec::read_int16(payload, 126)?,
            am_spin: codec::read_int16(payload, 128)?,
            pm_spin: codec::read_int16(payload, 130)?,
            spin_axis: codec::read_int16_scaled(payload, 132, 10.0)?,
            aod_spin: codec::read_int16(payload, 134)?,
            pll_spin: codec::read_int16(payload, 136)?,
        })
    }
}

// ---------------------------------------------------------------------------
// 0xD9 — SPEED_PROFILE (172 bytes)
// ---------------------------------------------------------------------------

/// Club head speed profile. 1 per shot. Type 0xD9.
#[derive(Debug, Clone)]
pub struct SpeedProfile {
    /// Flags/version byte (always 0x01)
    pub flags: u8,
    /// Number of pre-impact samples (36-45)
    pub num_pre: u8,
    /// Number of post-impact samples (18-38)
    pub num_post: u8,
    /// Scale factor for speed samples (always 100)
    pub scale_factor: i32,
    /// Time interval between samples (s, ~853us)
    pub time_interval: f64,
    /// Speed samples (m/s, already divided by scale)
    pub speeds: Vec<f64>,
}

impl SpeedProfile {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        // Device sometimes sends a stub (2B) when there's no speed data.
        if payload.len() < 12 {
            return Ok(Self {
                flags: payload.get(1).copied().unwrap_or(0),
                num_pre: 0,
                num_post: 0,
                scale_factor: 0,
                time_interval: 0.0,
                speeds: Vec::new(),
            });
        }

        let flags = payload[1];
        let num_pre = payload[2];
        let num_post = payload[3];
        let scale_factor = codec::read_int24(payload, 4)?;
        let time_interval = codec::read_float40(payload, 7)?;

        let sf = if scale_factor == 0 {
            1.0
        } else {
            f64::from(scale_factor)
        };

        // Read speed samples (INT16 each, starting at offset 12)
        let num_samples = (payload.len() - 12) / 2;
        let mut speeds = Vec::with_capacity(num_samples);
        for i in 0..num_samples {
            let raw = codec::read_int16(payload, 12 + i * 2)?;
            speeds.push(f64::from(raw) / sf);
        }

        Ok(Self {
            flags,
            num_pre,
            num_post,
            scale_factor,
            time_interval,
            speeds,
        })
    }
}

// ---------------------------------------------------------------------------
// 0xE9 — TRACKING_STATUS (82 bytes)
// ---------------------------------------------------------------------------

/// Radar tracking metadata. 5 per shot in 3 phases. Type 0xE9.
#[derive(Debug, Clone)]
pub struct TrackingStatus {
    /// State/flags byte
    pub state: u8,
    /// Flags byte
    pub flags: u8,
    /// Pre-trigger buffer start (sample index in 2^18 circular buffer)
    pub pre_trig_buf_start: u32,
    /// Club impact index (0xFFFFFF if not yet found)
    pub club_impact_idx: u32,
    /// Trigger index (sample index)
    pub trigger_idx: u32,
    /// Radar calibration constant 1
    pub radar_cal1: u32,
    /// Radar calibration constant 2 (= radar_cal1)
    pub radar_cal2: u32,
    /// AVR radar calibration value
    pub radar_cal_avr: u16,
    /// Processing iteration (0-2)
    pub processing_iteration: u8,
    /// Result quality
    pub result_quality: u8,
    /// Detection subtype
    pub detection_subtype: u8,
    /// PRC tracking point count (validated 7/7 shots)
    pub prc_tracking_count: u8,
    /// Radar measurement
    pub radar_measurement: u16,
    /// Trigger flags
    pub trigger_flags: u8,
    /// Event counter
    pub event_counter: u16,
    /// Radar baseline (signed)
    pub radar_baseline: i32,
    /// Post-tracking measurements
    pub track_measure: [i32; 3],
    /// Track measure 4
    pub track_measure4: u16,
}

impl TrackingStatus {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 82 {
            return Err(WireError::payload_too_short("TrackingStatus", 82, payload.len()));
        }

        Ok(Self {
            state: payload[1],
            flags: payload[2],
            pre_trig_buf_start: codec::read_uint24(payload, 22)?,
            club_impact_idx: codec::read_uint24(payload, 25)?,
            trigger_idx: codec::read_uint24(payload, 28)?,
            radar_cal1: codec::read_uint24(payload, 32)?,
            radar_cal2: codec::read_uint24(payload, 35)?,
            radar_cal_avr: codec::read_uint16(payload, 38)?,
            processing_iteration: payload[47],
            result_quality: payload[48],
            detection_subtype: payload[51],
            prc_tracking_count: payload[54],
            radar_measurement: codec::read_uint16(payload, 56)?,
            trigger_flags: payload[59],
            event_counter: codec::read_uint16(payload, 62)?,
            radar_baseline: codec::read_int24(payload, 67)?,
            track_measure: [
                codec::read_int24(payload, 70)?,
                codec::read_int24(payload, 73)?,
                codec::read_int24(payload, 76)?,
            ],
            track_measure4: codec::read_uint16(payload, 80)?,
        })
    }
}

// ---------------------------------------------------------------------------
// 0xEC — PRC_DATA (variable, 60-byte sub-records)
// ---------------------------------------------------------------------------

const PK_SCALE: f64 = 10000.0 / (1u32 << 23) as f64;

/// A single ball radar tracking point (60 bytes).
#[derive(Debug, Clone)]
pub struct PrcPoint {
    pub index: i16,
    pub peak: i16,
    pub snr: i32,
    pub buf_idx: i16,
    pub flags: u8,
    /// Time counter (~26.7us per count)
    pub time: i32,
    /// Refractive index factor (/100000)
    pub n: f64,
    /// Azimuth (deg, /100)
    pub az: f64,
    /// Elevation (deg, /100)
    pub el: f64,
    /// Radial velocity (m/s, /100)
    pub vel: f64,
    /// Distance (m, /1000)
    pub dist: f64,
    pub sync_idx: i32,
    pub sync_buf: i32,
    /// Individual antenna azimuths (deg, /100)
    pub az1: f64,
    pub az2: f64,
    pub az3: f64,
    /// Individual antenna elevations (deg, /100)
    pub el1: f64,
    pub el2: f64,
    /// Peak values (x PK_SCALE)
    pub pk: [f64; 6],
}

/// Raw ball radar tracking data. Type 0xEC.
#[derive(Debug, Clone)]
pub struct PrcData {
    /// Frame sequence number
    pub sequence: i16,
    /// Tracking points
    pub points: Vec<PrcPoint>,
}

impl PrcData {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 4 {
            return Err(WireError::payload_too_short("PrcData", 4, payload.len()));
        }

        let header = payload[0] as usize;
        let sequence = codec::read_int16(payload, 1)?;
        let sub_count = payload[3] as usize;

        // Version 4: stride 60, (header - 3) % 60 == 0
        let stride = if header >= 3 && (header - 3) % 60 == 0 {
            60
        } else {
            // Unsupported version, store as empty
            return Ok(Self {
                sequence,
                points: Vec::new(),
            });
        };

        let mut points = Vec::with_capacity(sub_count);
        for i in 0..sub_count {
            let base = 4 + i * stride;
            if base + stride > payload.len() {
                break;
            }
            let d = &payload[base..];
            let mut pk = [0.0; 6];
            for (j, p) in pk.iter_mut().enumerate() {
                *p = f64::from(codec::read_int24(d, 42 + j * 3)?) * PK_SCALE;
            }
            points.push(PrcPoint {
                index: codec::read_int16(d, 0)?,
                peak: codec::read_int16(d, 2)?,
                snr: codec::read_int24(d, 4)?,
                buf_idx: codec::read_int16(d, 7)?,
                flags: d[9],
                time: codec::read_int24(d, 10)?,
                n: codec::read_int24_scaled(d, 13, 100_000.0)?,
                az: codec::read_int16_scaled(d, 16, 100.0)?,
                el: codec::read_int16_scaled(d, 18, 100.0)?,
                vel: codec::read_int24_scaled(d, 20, 100.0)?,
                dist: codec::read_int24_scaled(d, 23, 1000.0)?,
                sync_idx: codec::read_int24(d, 26)?,
                sync_buf: codec::read_int24(d, 29)?,
                az1: codec::read_int16_scaled(d, 32, 100.0)?,
                az2: codec::read_int16_scaled(d, 34, 100.0)?,
                az3: codec::read_int16_scaled(d, 36, 100.0)?,
                el1: codec::read_int16_scaled(d, 38, 100.0)?,
                el2: codec::read_int16_scaled(d, 40, 100.0)?,
                pk,
            });
        }

        Ok(Self { sequence, points })
    }
}

// ---------------------------------------------------------------------------
// 0xEE — CLUB_PRC (76-byte sub-records, paginated)
// ---------------------------------------------------------------------------

/// A single club head radar tracking point (76 bytes).
#[derive(Debug, Clone)]
pub struct ClubPrcPoint {
    pub index: i16,
    /// Signed offset from trigger (-1600 to +2100)
    pub buf_ofs: i16,
    pub peak: i16,
    pub snr: i32,
    pub buf_idx: i16,
    /// Time counter
    pub time: i32,
    /// Refractive index factor (/100000)
    pub n: f64,
    /// Azimuth (deg, /100)
    pub az: f64,
    /// Elevation (deg, /100)
    pub el: f64,
    /// Velocity (m/s, /100)
    pub vel: f64,
    /// Velocity 2 (m/s, /100)
    pub vel2: f64,
    /// Distance (m, /1000)
    pub dist: f64,
    pub f30: f64,
    pub f33: f64,
    pub version: u8,
    pub f39: i32,
    pub f42: i32,
    /// (/1000)
    pub f45: f64,
    /// Individual antenna azimuths (deg, /100)
    pub az1: f64,
    pub az2: f64,
    pub az3: f64,
    /// Individual antenna elevations (deg, /100)
    pub el1: f64,
    pub el2: f64,
    /// Peak values (x PK_SCALE)
    pub pk: [f64; 6],
}

/// Raw club head radar tracking data. Type 0xEE.
#[derive(Debug, Clone)]
pub struct ClubPrc {
    pub points: Vec<ClubPrcPoint>,
}

impl ClubPrc {
    /// Decode from a response payload. First byte is data_len, rest is sub-records.
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.is_empty() {
            return Ok(Self {
                points: Vec::new(),
            });
        }

        let data_len = payload[0] as usize;
        let num_records = data_len / 76;
        let mut points = Vec::with_capacity(num_records);

        for i in 0..num_records {
            let base = 1 + i * 76;
            if base + 76 > payload.len() {
                break;
            }
            let d = &payload[base..];

            let mut pk = [0.0; 6];
            for (j, p) in pk.iter_mut().enumerate() {
                *p = f64::from(codec::read_int24(d, 58 + j * 3)?) * PK_SCALE;
            }

            points.push(ClubPrcPoint {
                index: codec::read_int16(d, 0)?,
                buf_ofs: codec::read_int16(d, 2)?,
                peak: codec::read_int16(d, 4)?,
                snr: codec::read_int24(d, 6)?,
                buf_idx: codec::read_int16(d, 9)?,
                time: codec::read_int24(d, 11)?,
                n: codec::read_int24_scaled(d, 14, 100_000.0)?,
                az: codec::read_int16_scaled(d, 17, 100.0)?,
                el: codec::read_int16_scaled(d, 19, 100.0)?,
                vel: codec::read_int24_scaled(d, 21, 100.0)?,
                vel2: codec::read_int24_scaled(d, 24, 100.0)?,
                dist: codec::read_int24_scaled(d, 27, 1000.0)?,
                f30: codec::read_int24_scaled(d, 30, 1000.0)?,
                f33: codec::read_int24_scaled(d, 33, 1000.0)?,
                version: d[38],
                f39: codec::read_int24(d, 39)?,
                f42: codec::read_int24(d, 42)?,
                f45: codec::read_int24_scaled(d, 45, 1000.0)?,
                az1: codec::read_int16_scaled(d, 48, 100.0)?,
                az2: codec::read_int16_scaled(d, 50, 100.0)?,
                az3: codec::read_int16_scaled(d, 52, 100.0)?,
                el1: codec::read_int16_scaled(d, 54, 100.0)?,
                el2: codec::read_int16_scaled(d, 56, 100.0)?,
                pk,
            });
        }

        Ok(Self { points })
    }

    /// Encode a CLUB_PRC page request (APP→AVR, 77 bytes).
    /// `start_index` is the first record index to fetch (increments by 3).
    pub fn encode_request(start_index: u16) -> Vec<u8> {
        let mut buf = vec![0x4C]; // stride
        codec::write_uint16(&mut buf, start_index);
        // Pad to 77 bytes
        buf.resize(77, 0);
        buf
    }
}

// ---------------------------------------------------------------------------
// 0xE5 — SHOT_TEXT (variable)
// ---------------------------------------------------------------------------

/// Shot processing state text. Type 0xE5.
///
/// ASCII messages: "BALL TRIGGER", "Clubimpact", "PROCESSED", "IDLE".
#[derive(Debug, Clone)]
pub struct ShotText {
    pub text: String,
}

impl ShotText {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        // Strip leading/trailing control characters and nulls
        let end = payload
            .iter()
            .rposition(|&b| b >= 0x20)
            .map_or(0, |p| p + 1);
        let start = payload[..end]
            .iter()
            .position(|&b| b >= 0x20)
            .unwrap_or(0);
        Ok(Self {
            text: String::from_utf8_lossy(&payload[start..end]).into_owned(),
        })
    }

    /// Check if this is a "PROCESSED" message.
    pub fn is_processed(&self) -> bool {
        self.text.contains("PROCESSED")
    }

    /// Check if this is an "IDLE" message.
    pub fn is_idle(&self) -> bool {
        self.text.contains("IDLE")
    }

    /// Check if this is a "BALL TRIGGER" message.
    pub fn is_trigger(&self) -> bool {
        self.text.contains("BALL TRIGGER")
    }
}
