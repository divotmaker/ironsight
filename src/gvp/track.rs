//! MT_GOLF_EXPECTED_TRACK and MT_GOLF_EXPECTED_CLUB_TRACK message types.
//!
//! Sent by APP to GVP with radar-derived polynomial trajectory hints.
//! The GVP's SimpleObjectTracker uses these as search hints to find
//! the club head and ball in each camera frame.

use serde::{Deserialize, Serialize};

/// Radar-derived polynomial trajectory hint.
///
/// Used for both `MT_GOLF_EXPECTED_TRACK` (ball) and
/// `MT_GOLF_EXPECTED_CLUB_TRACK` (club). The polynomial
/// `p(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4`
/// predicts expected position in camera pixel coordinates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct ExpectedTrack {
    pub guid: String,
    pub duration: f64,
    pub start_time: f64,
    /// 4th-order polynomial coefficients for u (horizontal pixel).
    #[serde(rename = "polyU")]
    pub poly_u: [f64; 5],
    /// 4th-order polynomial coefficients for v (vertical pixel).
    #[serde(rename = "polyV")]
    pub poly_v: [f64; 5],
    /// 4th-order polynomial coefficients for expected object radius (pixels).
    #[serde(rename = "polyRadius")]
    pub poly_radius: [f64; 5],
}
