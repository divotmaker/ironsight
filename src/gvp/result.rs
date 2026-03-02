//! RESULT message type (BallTrackerResult).
//!
//! Sent by GVP after completing image tracking for both club and ball.
//! Contains raw pixel tracking data from the SimpleObjectTracker (~3200B).
//!
//! Face impact is NOT in this message — it contains only raw pixel
//! coordinates. Face impact location is computed client-side by fusing
//! these camera tracking points with radar data.

use serde::{Deserialize, Serialize};

use super::config::CameraCalibration;

/// A single object track from the camera's SimpleObjectTracker.
///
/// All array fields are parallel arrays indexed by detection frame.
/// For example, `u[i]` and `v[i]` are the pixel coordinates of the
/// object at `timestamp[i]` in camera frame `frame_number[i]`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct Track {
    /// Object identifier: 0=ball, 1=club head, 2-4=reference markers.
    pub track_id: i32,
    /// Camera frame indices where the object was detected.
    pub frame_number: Vec<i32>,
    /// Unix epoch timestamps per frame (fractional seconds).
    pub timestamp: Vec<f64>,
    /// Horizontal pixel coordinates (sub-pixel precision).
    pub u: Vec<f64>,
    /// Vertical pixel coordinates (sub-pixel precision).
    pub v: Vec<f64>,
    /// Detected object radius in pixels.
    pub radius: Vec<f64>,
    /// Shape metric (high = circular, low = elongated).
    pub circularity_factor: Vec<f64>,
    /// Shutter time per frame in milliseconds.
    #[serde(rename = "shutterTime_ms")]
    pub shutter_time_ms: Vec<f64>,
}

impl Track {
    /// Number of detection points in this track.
    #[must_use]
    pub fn len(&self) -> usize {
        self.frame_number.len()
    }

    /// Whether this track has no detection points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frame_number.is_empty()
    }

    /// Whether this is the ball track (track ID 0).
    #[must_use]
    pub fn is_ball(&self) -> bool {
        self.track_id == 0
    }

    /// Whether this is the club head track (track ID 1).
    #[must_use]
    pub fn is_club(&self) -> bool {
        self.track_id == 1
    }
}

/// Ball tracker result (RESULT message body).
///
/// Contains the camera calibration used (all zeros in observed traffic)
/// and a variable number of tracks for detected objects.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct BallTrackerResult {
    pub guid: String,
    pub camera_calibration: CameraCalibration,
    pub tracks: Vec<Track>,
}

impl BallTrackerResult {
    /// Find the ball track (track ID 0), if present.
    #[must_use]
    pub fn ball_track(&self) -> Option<&Track> {
        self.tracks.iter().find(|t| t.is_ball())
    }

    /// Find the club head track (track ID 1), if present.
    #[must_use]
    pub fn club_track(&self) -> Option<&Track> {
        self.tracks.iter().find(|t| t.is_club())
    }
}
