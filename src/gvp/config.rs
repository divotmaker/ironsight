//! CONFIG and CONFIG_REQUEST message types.
//!
//! Bidirectional. GVP sends current config in response to CONFIG_REQUEST;
//! APP sends to push updated settings.

use serde::{Deserialize, Serialize};

/// Camera calibration parameters (intrinsics + extrinsics).
///
/// All zeros in observed traffic — the GVP uses its own stored calibration
/// or derives it from the resolution mode.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CameraCalibration {
    pub cx: f64,
    pub cy: f64,
    pub fx: f64,
    pub fy: f64,
    pub width: i32,
    pub height: i32,
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    #[serde(rename = "distCoeffs")]
    pub dist_coeffs: [f64; 8],
}

impl Default for CameraCalibration {
    fn default() -> Self {
        Self {
            cx: 0.0,
            cy: 0.0,
            fx: 0.0,
            fy: 0.0,
            width: 0,
            height: 0,
            position: [0.0; 3],
            rotation: [0.0; 3],
            dist_coeffs: [0.0; 8],
        }
    }
}

/// Buffer configuration (pre/post trigger frame counts).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct BufferConfiguration {
    pub buffer_size_pre_trigger: i32,
    pub buffer_size_post_trigger: i32,
}

/// Camera ROI and mode configuration.
///
/// Field names use uppercase `ROI_` prefix in the JSON protocol,
/// which doesn't follow camelCase.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CameraConfiguration {
    #[serde(rename = "ROI_x")]
    pub roi_x: i32,
    #[serde(rename = "ROI_y")]
    pub roi_y: i32,
    #[serde(rename = "ROI_width")]
    pub roi_width: i32,
    #[serde(rename = "ROI_height")]
    pub roi_height: i32,
    #[serde(rename = "ROI_maxWidth")]
    pub roi_max_width: i32,
    #[serde(rename = "ROI_maxHeight")]
    pub roi_max_height: i32,
    #[serde(rename = "isFreeRun")]
    pub is_free_run: bool,
    #[serde(rename = "rotationDegCW")]
    pub rotation_deg_cw: i32,
}

/// Live preview processing configuration.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct LivePreviewProcessingConfiguration {
    #[serde(rename = "ROI_center_u")]
    pub roi_center_u: i32,
    #[serde(rename = "ROI_center_v")]
    pub roi_center_v: i32,
    #[serde(rename = "ROI_width")]
    pub roi_width: i32,
    #[serde(rename = "ROI_height")]
    pub roi_height: i32,
    pub enabled: bool,
    #[serde(rename = "rotationDegCW")]
    pub rotation_deg_cw: i32,
}

/// Full GVP camera configuration (CONFIG message body).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct GvpConfig {
    pub buffer_configuration: BufferConfiguration,
    pub camera_calibration: CameraCalibration,
    pub camera_configuration: CameraConfiguration,
    pub live_preview_processing_configuration: LivePreviewProcessingConfiguration,
    pub frame_number_info_enabled: bool,
    pub logging_enabled: bool,
    pub save_videos_enabled: bool,
}

impl GvpConfig {
    /// APP→GVP config for standard (non-Fusion) mode.
    ///
    /// All fields zeroed except metadata flags. The GVP uses its own stored
    /// configuration; the APP-sent CONFIG is metadata/hints, not the actual
    /// capture resolution (which is set via binary 0x82 CAM_CONFIG on port 5100).
    ///
    /// Matches observed five_strikes pcap: ROI 0×0, buffer 0/0.
    #[must_use]
    pub fn standard() -> Self {
        Self {
            buffer_configuration: BufferConfiguration {
                buffer_size_pre_trigger: 0,
                buffer_size_post_trigger: 0,
            },
            camera_calibration: CameraCalibration::default(),
            camera_configuration: CameraConfiguration {
                roi_x: 0,
                roi_y: 0,
                roi_width: 0,
                roi_height: 0,
                roi_max_width: 0,
                roi_max_height: 0,
                is_free_run: true,
                rotation_deg_cw: 0,
            },
            live_preview_processing_configuration: LivePreviewProcessingConfiguration {
                roi_center_u: 0,
                roi_center_v: 0,
                roi_width: 0,
                roi_height: 0,
                enabled: false,
                rotation_deg_cw: 0,
            },
            frame_number_info_enabled: true,
            logging_enabled: true,
            save_videos_enabled: true,
        }
    }

    /// APP→GVP config for Fusion mode (face impact tracking).
    ///
    /// ROI set to 640×480 as observed in the native FS Golf app's
    /// face_impact_shot_id_4 pcap. The actual 1640×1232 capture resolution
    /// is set via binary 0x82 CAM_CONFIG on port 5100, not here.
    #[must_use]
    pub fn fusion() -> Self {
        let mut config = Self::standard();
        config.camera_configuration.roi_width = 640;
        config.camera_configuration.roi_height = 480;
        config
    }

    /// Whether this is a GVP-reported Fusion mode config (1640×1232).
    ///
    /// Only meaningful on configs **received from the GVP** (in response to
    /// CONFIG_REQUEST). APP-sent configs use 640×480 for Fusion mode — the
    /// actual capture resolution is controlled by binary 0x82 CAM_CONFIG.
    #[must_use]
    pub fn is_fusion(&self) -> bool {
        self.camera_configuration.roi_width == 1640
            && self.camera_configuration.roi_height == 1232
    }
}
