//! Camera-related messages (APP ↔ PI).

use crate::codec;
use crate::error::{Result, WireError};

/// Camera start/stop (2 bytes). Type 0x81.
///
/// APP→PI: 0x00=stop, 0x01=start, 0x03=start+streaming.
/// PI→APP: 0x00=off, 0x01=on.
#[derive(Debug, Clone)]
pub struct CamState {
    pub state: u8,
}

impl CamState {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 2 {
            return Err(WireError::payload_too_short("CamState", 2, payload.len()));
        }
        Ok(Self { state: payload[1] })
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x01, self.state]
    }
}

/// Camera configuration (52 bytes, S51 format). Type 0x82.
#[derive(Debug, Clone)]
pub struct CamConfig {
    pub dynamic_config: bool,
    /// Capture width (1024 standard, 1640 Fusion)
    pub resolution_width: i16,
    /// Capture height (768 standard, 1232 Fusion)
    pub resolution_height: i16,
    pub rotation: i16,
    pub ev: i16,
    /// JPEG quality (20 or 80)
    pub quality: u8,
    /// Camera framerate (10 or 20)
    pub framerate: u8,
    /// Streaming framerate (1 or 20)
    pub streaming_framerate: u8,
    /// Pre-trigger ring buffer time (ms)
    pub ringbuffer_pretime_ms: i16,
    /// Post-trigger ring buffer time (ms)
    pub ringbuffer_posttime_ms: i16,
    pub raw_camera_mode: u8,
    pub fusion_camera_mode: bool,
    /// Max raw shutter speed (FLOAT40)
    pub raw_shutter_speed_max: f64,
    pub raw_ev_roi_x: i16,
    pub raw_ev_roi_y: i16,
    pub raw_ev_roi_width: i16,
    pub raw_ev_roi_height: i16,
    pub raw_x_offset: i16,
    pub raw_bin44: bool,
    pub raw_live_preview_write_interval_ms: i16,
    pub raw_y_offset: i16,
    pub buffer_sub_sampling_pre_trigger_div: i16,
    pub buffer_sub_sampling_post_trigger_div: i16,
    /// Sub-sampling switch time offset (FLOAT40)
    pub buffer_sub_sampling_switch_time_offset: f64,
    pub buffer_sub_sampling_total_buffer_size: i16,
    pub buffer_sub_sampling_pre_trigger_buffer_size: i16,
}

impl CamConfig {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.len() < 52 {
            return Err(WireError::payload_too_short("CamConfig", 52, payload.len()));
        }
        Ok(Self {
            dynamic_config: payload[1] != 0,
            resolution_width: codec::read_int16(payload, 2)?,
            resolution_height: codec::read_int16(payload, 4)?,
            rotation: codec::read_int16(payload, 6)?,
            ev: codec::read_int16(payload, 8)?,
            quality: payload[10],
            framerate: payload[11],
            streaming_framerate: payload[12],
            ringbuffer_pretime_ms: codec::read_int16(payload, 13)?,
            ringbuffer_posttime_ms: codec::read_int16(payload, 15)?,
            raw_camera_mode: payload[17],
            fusion_camera_mode: payload[18] != 0,
            raw_shutter_speed_max: codec::read_float40(payload, 19)?,
            raw_ev_roi_x: codec::read_int16(payload, 24)?,
            raw_ev_roi_y: codec::read_int16(payload, 26)?,
            raw_ev_roi_width: codec::read_int16(payload, 28)?,
            raw_ev_roi_height: codec::read_int16(payload, 30)?,
            raw_x_offset: codec::read_int16(payload, 32)?,
            raw_bin44: payload[34] != 0,
            raw_live_preview_write_interval_ms: codec::read_int16(payload, 35)?,
            raw_y_offset: codec::read_int16(payload, 37)?,
            buffer_sub_sampling_pre_trigger_div: codec::read_int16(payload, 39)?,
            buffer_sub_sampling_post_trigger_div: codec::read_int16(payload, 41)?,
            buffer_sub_sampling_switch_time_offset: codec::read_float40(payload, 43)?,
            buffer_sub_sampling_total_buffer_size: codec::read_int16(payload, 48)?,
            buffer_sub_sampling_pre_trigger_buffer_size: codec::read_int16(payload, 50)?,
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(52);
        buf.push(0x33); // S51 size marker
        buf.push(self.dynamic_config as u8);
        codec::write_int16(&mut buf, self.resolution_width);
        codec::write_int16(&mut buf, self.resolution_height);
        codec::write_int16(&mut buf, self.rotation);
        codec::write_int16(&mut buf, self.ev);
        buf.push(self.quality);
        buf.push(self.framerate);
        buf.push(self.streaming_framerate);
        codec::write_int16(&mut buf, self.ringbuffer_pretime_ms);
        codec::write_int16(&mut buf, self.ringbuffer_posttime_ms);
        buf.push(self.raw_camera_mode);
        buf.push(self.fusion_camera_mode as u8);
        codec::write_float40(&mut buf, self.raw_shutter_speed_max);
        codec::write_int16(&mut buf, self.raw_ev_roi_x);
        codec::write_int16(&mut buf, self.raw_ev_roi_y);
        codec::write_int16(&mut buf, self.raw_ev_roi_width);
        codec::write_int16(&mut buf, self.raw_ev_roi_height);
        codec::write_int16(&mut buf, self.raw_x_offset);
        buf.push(self.raw_bin44 as u8);
        codec::write_int16(&mut buf, self.raw_live_preview_write_interval_ms);
        codec::write_int16(&mut buf, self.raw_y_offset);
        codec::write_int16(&mut buf, self.buffer_sub_sampling_pre_trigger_div);
        codec::write_int16(&mut buf, self.buffer_sub_sampling_post_trigger_div);
        codec::write_float40(&mut buf, self.buffer_sub_sampling_switch_time_offset);
        codec::write_int16(&mut buf, self.buffer_sub_sampling_total_buffer_size);
        codec::write_int16(&mut buf, self.buffer_sub_sampling_pre_trigger_buffer_size);
        buf
    }
}

/// Camera config request (3 bytes). Type 0x83.
///
/// Always `[02 01 05]`.
#[derive(Debug, Clone)]
pub struct CamConfigReq;

impl CamConfigReq {
    pub fn decode(_payload: &[u8]) -> Result<Self> {
        Ok(Self)
    }

    pub fn encode(&self) -> Vec<u8> {
        vec![0x02, 0x01, 0x05]
    }
}

/// Per-shot camera image notification (67B long, 2B short). Type 0x84.
#[derive(Debug, Clone)]
pub struct CamImageAvail {
    /// Streaming available (bit 0)
    pub streaming_available: bool,
    /// Fusion data available (bit 0 of fusion_flags)
    pub fusion_available: bool,
    /// Video available (bit 1 of fusion_flags)
    pub video_available: bool,
    /// ISO timestamp for streaming (null-padded, 32 bytes)
    pub streaming_timestamp: Option<String>,
    /// ISO timestamp for fusion (null-padded, 32 bytes)
    pub fusion_timestamp: Option<String>,
}

impl CamImageAvail {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        if payload.is_empty() {
            return Err(WireError::payload_too_short("CamImageAvail", 1, 0));
        }

        if payload[0] == 0x42 {
            // Long form (67 bytes)
            if payload.len() < 67 {
                return Err(WireError::payload_too_short("CamImageAvail(long)", 67, payload.len()));
            }
            let streaming_flags = payload[1];
            let fusion_flags = payload[2];

            let streaming_ts = parse_null_padded_string(&payload[3..35]);
            let fusion_ts = parse_null_padded_string(&payload[35..67]);

            Ok(Self {
                streaming_available: streaming_flags & 1 != 0,
                fusion_available: fusion_flags & 1 != 0,
                video_available: fusion_flags & 2 != 0,
                streaming_timestamp: streaming_ts,
                fusion_timestamp: fusion_ts,
            })
        } else {
            // Short form (2 bytes)
            let flags = if payload.len() > 1 { payload[1] } else { 0 };
            Ok(Self {
                streaming_available: flags & 1 != 0,
                fusion_available: false,
                video_available: false,
                streaming_timestamp: None,
                fusion_timestamp: None,
            })
        }
    }
}

/// Sensor activation data (APP→PI). Type 0x90.
///
/// Client-side licensing — device arms without it.
#[derive(Debug, Clone)]
pub struct SensorAct {
    pub payload: Vec<u8>,
}

impl SensorAct {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }

    pub fn encode(&self) -> Vec<u8> {
        self.payload.clone()
    }
}

/// Sensor activation response (PI→APP). Type 0x89.
///
/// Device certificate (842-char base64).
#[derive(Debug, Clone)]
pub struct SensorActResp {
    pub payload: Vec<u8>,
}

impl SensorActResp {
    pub fn decode(payload: &[u8]) -> Result<Self> {
        Ok(Self {
            payload: payload.to_vec(),
        })
    }
}

/// Parse a null-padded fixed-width string, returning None if empty.
fn parse_null_padded_string(data: &[u8]) -> Option<String> {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    if end == 0 {
        None
    } else {
        Some(String::from_utf8_lossy(&data[..end]).into_owned())
    }
}
