//! Pcap validation: decode all 102 JSON messages extracted from
//! `face_impact_shot_id_4.pcapng` through `GvpMessage::decode()`.
//!
//! The fixture file (`gvp_pcap_messages.json`) was extracted from the pcap
//! using the reassembled TCP stream on port 1258.

#![cfg(feature = "gvp")]

use ironsight::gvp::{GvpMessage, NullSplitter};

// ---------------------------------------------------------------------------
// Pcap fixtures (first message of each type from face_impact_shot_id_4.pcapng)
// ---------------------------------------------------------------------------

const PCAP_TRIGGER: &str = r##"{"epochTime":1771985855.931274,"frameNumber":0,"guid":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","reprocessPath":"","savePath":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","skipTracking":false,"triggerTimeOffset":0,"type":"TRIGGER","version":6}"##;

const PCAP_CONFIG: &str = r##"{"bufferConfiguration":{"bufferSizePostTrigger":0,"bufferSizePreTrigger":0},"cameraCalibration":{"cx":0,"cy":0,"distCoeffs":[0,0,0,0,0,0,0,0],"fx":0,"fy":0,"height":0,"position":[0,0,0],"rotation":[0,0,0],"width":0},"cameraConfiguration":{"ROI_height":480,"ROI_maxHeight":0,"ROI_maxWidth":0,"ROI_width":640,"ROI_x":0,"ROI_y":0,"isFreeRun":true,"rotationDegCW":0},"frameNumberInfoEnabled":true,"livePreviewProcessingConfiguration":{"ROI_center_u":0,"ROI_center_v":0,"ROI_height":0,"ROI_width":0,"enabled":false,"rotationDegCW":0},"loggingEnabled":true,"saveVideosEnabled":true,"type":"CONFIG","version":5}"##;

const PCAP_STATUS: &str = r##"{"bufferStatus":[{"bufferIndex":0,"status":"TRIGGERED"}],"type":"STATUS","version":1}"##;

const PCAP_LOG: &str = r##"{"level":1,"message":"[GVP][2026-02-25 04:17:36.122][173.07/743.766MB][GolfVideoProcessor] Requesting trigger {b39aaf63-edda-42e4-a823-fdcd0d601871} at frame 0","type":"LOG","version":1}"##;

const PCAP_EXPECTED_CLUB_TRACK: &str = r##"{"duration":0.10000000149011612,"guid":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","polyRadius":[1,13.157645225524902,2.713169574737549,0,0],"polyU":[319.7021484375,0.009123682975769043,-0.22010931372642517,-8.884710311889648,181.4336395263672],"polyV":[237.00779724121094,-0.0015800477704033256,0.04636988043785095,2.2090604305267334,-51.26675033569336],"startTime":0.015066666528582573,"type":"MT_GOLF_EXPECTED_CLUB_TRACK","version":1}"##;

const PCAP_EXPECTED_TRACK: &str = r##"{"duration":0.10000000149011612,"guid":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","polyRadius":[1,11.89510726928711,2.713169574737549,0,0],"polyU":[319.49322509765625,-151.2446746826172,701.3250732421875,-2940.84228515625,7344.51806640625],"polyV":[316.2373352050781,-2414.629638671875,10972.69921875,-41789.9921875,88561.140625],"startTime":0.015066666528582573,"type":"MT_GOLF_EXPECTED_TRACK","version":1}"##;

const PCAP_RESULT: &str = r##"{"cameraCalibration":{"cx":0,"cy":0,"distCoeffs":[0,0,0,0,0,0,0,0],"fx":0,"fy":0,"height":0,"position":[0,0,0],"rotation":[0,0,0],"width":0},"guid":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","tracks":[{"circularityFactor":[22.79698944091797,59.222476959228516,52.746917724609375,43.4005126953125,40.884735107421875,35.89910888671875,32.438812255859375,28.917388916015625,29.30267333984375,26.1854248046875,27.1346435546875,27.4405517578125,25.499114990234375,25.784942626953125,23.779754638671875],"frameNumber":[12,14,16,18,19,20,21,22,23,24,25,27,30,33,36],"radius":[43.78057861328125,54.62237548828125,56.3968505859375,53.790496826171875,50.805145263671875,52.68121337890625,50.84654998779297,49.76849365234375,51.211669921875,42.427268981933594,44.12042236328125,46.677520751953125,41.631813049316406,38.551483154296875,35.77196502685547],"shutterTime_ms":[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],"timestamp":[1771985855.895297,1771985855.90636,1771985855.917438,1771985855.9284818,1771985855.9340158,1771985855.939545,1771985855.945097,1771985855.95063,1771985855.95615,1771985855.961684,1771985855.967212,1771985855.978277,1771985855.994866,1771985856.011461,1771985856.028065],"trackId":1,"u":[243.89419555664062,277.89593505859375,296.29791259765625,306.54693603515625,308.6282043457031,311.354736328125,309.05322265625,308.2046813964844,308.65380859375,304.7550048828125,308.66021728515625,311.5762939453125,311.498779296875,311.0279541015625,308.0517883300781],"v":[84.42623901367188,142.55389404296875,211.0748291015625,264.53857421875,285.34564208984375,302.525634765625,312.24676513671875,317.0718994140625,314.4519348144531,311.43511962890625,311.2025146484375,307.023681640625,293.3074951171875,280.91522216796875,267.1562805175781]},{"circularityFactor":[0.7439394593238831,0.22602130472660065,0.36780470609664917,0.27344539761543274,0.21335653960704803,0.7176352739334106,0.4702235162258148,0.29875749349594116,0.8446336984634399,0.6825628280639648],"frameNumber":[23,24,25,26,27,29,31,33,35,37],"radius":[12.936737060546875,12.42425537109375,12.153026580810547,12.050346374511719,10.786643981933594,8.845958709716797,10.261306762695312,10.413684844970703,8.906978607177734,7.775077819824219],"shutterTime_ms":[1,1,1,1,1,1,1,1,1,1],"timestamp":[1771985855.95615,1771985855.961684,1771985855.967212,1771985855.972741,1771985855.978277,1771985855.9893339,1771985856.000401,1771985856.011461,1771985856.022522,1771985856.033581],"trackId":0,"u":[325.9244079589844,326.3190612792969,324.2794494628906,322.9638366699219,321.9942321777344,318.63812255859375,319.4335021972656,317.7243957519531,316.98504638671875,315.0874938964844],"v":[302.9756164550781,289.59783935546875,276.0549621582031,264.12469482421875,253.07778930664062,233.5045623779297,215.47998046875,198.68032836914062,184.11354064941406,170.78111267089844]},{"circularityFactor":[0.897361159324646],"frameNumber":[0],"radius":[0.10269849002361298],"shutterTime_ms":[1],"timestamp":[1771985855.828939],"trackId":3,"u":[0],"v":[0.6216689944267273]},{"circularityFactor":[31],"frameNumber":[0],"radius":[12.189790725708008],"shutterTime_ms":[1],"timestamp":[1771985855.828939],"trackId":2,"u":[327.2470397949219],"v":[312.4698791503906]},{"circularityFactor":[31],"frameNumber":[0],"radius":[1],"shutterTime_ms":[1],"timestamp":[1771985855.931274],"trackId":4,"u":[0],"v":[0]}],"type":"RESULT","version":1}"##;

const PCAP_VIDEO_AVAILABLE: &str = r##"{"absolutePath":"/home/ftp/backup/{b39aaf63-edda-42e4-a823-fdcd0d601871}/video.mp4","guid":"{b39aaf63-edda-42e4-a823-fdcd0d601871}","relativePath":"/{b39aaf63-edda-42e4-a823-fdcd0d601871}/video.mp4","type":"MT_VIDEO_AVAILABLE","version":1}"##;

// ---------------------------------------------------------------------------
// Per-type decode tests
// ---------------------------------------------------------------------------

#[test]
fn pcap_decode_trigger() {
    // TRIGGER is APP→GVP, so it decodes as Unknown (forward compat)
    let msg = GvpMessage::decode(PCAP_TRIGGER).unwrap();
    assert!(matches!(msg, GvpMessage::Unknown { .. }));
}

#[test]
fn pcap_decode_config() {
    let msg = GvpMessage::decode(PCAP_CONFIG).unwrap();
    match msg {
        GvpMessage::Config(cfg) => {
            assert_eq!(cfg.camera_configuration.roi_width, 640);
            assert_eq!(cfg.camera_configuration.roi_height, 480);
            assert!(!cfg.is_fusion());
            assert!(cfg.frame_number_info_enabled);
            assert!(cfg.logging_enabled);
            assert!(cfg.save_videos_enabled);
        }
        other => panic!("expected Config, got {other:?}"),
    }
}

#[test]
fn pcap_decode_status() {
    let msg = GvpMessage::decode(PCAP_STATUS).unwrap();
    match msg {
        GvpMessage::Status(s) => {
            assert_eq!(s.status(), "TRIGGERED");
            assert!(!s.is_idle());
            assert_eq!(s.buffer_status.len(), 1);
            assert_eq!(s.buffer_status[0].buffer_index, 0);
        }
        other => panic!("expected Status, got {other:?}"),
    }
}

#[test]
fn pcap_decode_log() {
    let msg = GvpMessage::decode(PCAP_LOG).unwrap();
    match msg {
        GvpMessage::Log(l) => {
            assert_eq!(l.level, 1);
            assert!(l.message.contains("Requesting trigger"));
            assert!(l.message.contains("{b39aaf63"));
        }
        other => panic!("expected Log, got {other:?}"),
    }
}

#[test]
fn pcap_decode_expected_club_track() {
    // APP→GVP message, decodes as Unknown
    let msg = GvpMessage::decode(PCAP_EXPECTED_CLUB_TRACK).unwrap();
    assert!(matches!(msg, GvpMessage::Unknown { .. }));
}

#[test]
fn pcap_decode_expected_track() {
    // APP→GVP message, decodes as Unknown
    let msg = GvpMessage::decode(PCAP_EXPECTED_TRACK).unwrap();
    assert!(matches!(msg, GvpMessage::Unknown { .. }));
}

#[test]
fn pcap_decode_result() {
    let msg = GvpMessage::decode(PCAP_RESULT).unwrap();
    match msg {
        GvpMessage::Result(r) => {
            assert_eq!(r.guid, "{b39aaf63-edda-42e4-a823-fdcd0d601871}");
            assert_eq!(r.tracks.len(), 5);

            // Club head track (ID 1): 15 points
            let club = r.club_track().expect("club track present");
            assert!(club.is_club());
            assert_eq!(club.len(), 15);
            assert_eq!(club.frame_number[0], 12);
            assert_eq!(club.frame_number[14], 36);
            // Radius range: 35.8 - 56.4
            assert!(club.radius.iter().all(|&r| (35.0..57.0).contains(&r)));

            // Ball track (ID 0): 10 points
            let ball = r.ball_track().expect("ball track present");
            assert!(ball.is_ball());
            assert_eq!(ball.len(), 10);
            assert_eq!(ball.frame_number[0], 23);
            assert_eq!(ball.frame_number[9], 37);
            // Radius range: 7.8 - 12.9
            assert!(ball.radius.iter().all(|&r| (7.0..13.0).contains(&r)));

            // Reference tracks (IDs 2, 3, 4): 1 point each
            let refs: Vec<_> = r.tracks.iter().filter(|t| t.track_id >= 2).collect();
            assert_eq!(refs.len(), 3);
            for t in &refs {
                assert_eq!(t.len(), 1);
            }

            // Camera calibration is all zeros
            assert_eq!(r.camera_calibration.fx, 0.0);
            assert_eq!(r.camera_calibration.fy, 0.0);
            assert_eq!(r.camera_calibration.cx, 0.0);
            assert_eq!(r.camera_calibration.cy, 0.0);
        }
        other => panic!("expected Result, got {other:?}"),
    }
}

#[test]
fn pcap_decode_video_available() {
    let msg = GvpMessage::decode(PCAP_VIDEO_AVAILABLE).unwrap();
    match msg {
        GvpMessage::VideoAvailable(v) => {
            assert_eq!(v.guid, "{b39aaf63-edda-42e4-a823-fdcd0d601871}");
            assert!(v.absolute_path.contains("/home/ftp/backup/"));
            assert!(v.absolute_path.ends_with("/video.mp4"));
            assert!(v.relative_path.starts_with('/'));
        }
        other => panic!("expected VideoAvailable, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Bulk pcap validation: all 102 messages decode without error
// ---------------------------------------------------------------------------

#[test]
fn pcap_decode_all_102_messages() {
    let json_str = include_str!("gvp_pcap_messages.json");
    let messages: Vec<String> = serde_json::from_str(json_str).unwrap();
    assert_eq!(messages.len(), 102, "expected 102 messages from pcap");

    let mut counts = std::collections::HashMap::new();
    for (i, json) in messages.iter().enumerate() {
        let msg = GvpMessage::decode(json)
            .unwrap_or_else(|e| panic!("message {i} failed to decode: {e}\n  json: {json}"));
        let variant = match &msg {
            GvpMessage::Config(_) => "Config",
            GvpMessage::Status(_) => "Status",
            GvpMessage::Log(_) => "Log",
            GvpMessage::Result(_) => "Result",
            GvpMessage::VideoAvailable(_) => "VideoAvailable",
            GvpMessage::Unknown { msg_type, .. } => msg_type.as_str(),
        };
        *counts.entry(variant.to_owned()).or_insert(0) += 1;
    }

    // Expected counts from pcap analysis
    assert_eq!(counts.get("Config"), Some(&2));
    assert_eq!(counts.get("Log"), Some(&80));
    assert_eq!(counts.get("Status"), Some(&10));
    assert_eq!(counts.get("Result"), Some(&2));
    assert_eq!(counts.get("VideoAvailable"), Some(&2));
    // APP→GVP messages decode as Unknown
    assert_eq!(counts.get("TRIGGER"), Some(&2));
    assert_eq!(counts.get("MT_GOLF_EXPECTED_CLUB_TRACK"), Some(&2));
    assert_eq!(counts.get("MT_GOLF_EXPECTED_TRACK"), Some(&2));
}

// ---------------------------------------------------------------------------
// NullSplitter with pcap-realistic data
// ---------------------------------------------------------------------------

#[test]
fn splitter_with_pcap_messages() {
    let mut splitter = NullSplitter::new();

    // Simulate TCP stream: concatenate several messages with null terminators
    let mut stream = Vec::new();
    stream.extend_from_slice(PCAP_STATUS.as_bytes());
    stream.push(0x00);
    stream.extend_from_slice(PCAP_LOG.as_bytes());
    stream.push(0x00);
    stream.extend_from_slice(PCAP_VIDEO_AVAILABLE.as_bytes());
    stream.push(0x00);

    // Feed in one chunk
    let msgs = splitter.feed(&stream);
    assert_eq!(msgs.len(), 3);
    assert_eq!(msgs[0], PCAP_STATUS);
    assert_eq!(msgs[1], PCAP_LOG);
    assert_eq!(msgs[2], PCAP_VIDEO_AVAILABLE);
}

#[test]
fn splitter_result_across_segments() {
    let mut splitter = NullSplitter::new();

    // Split the ~3400B RESULT message across 4 TCP segments
    let mut data = PCAP_RESULT.as_bytes().to_vec();
    data.push(0x00);

    let chunk_size = 1000;
    let chunks: Vec<&[u8]> = data.chunks(chunk_size).collect();
    assert!(chunks.len() >= 3, "should split into 3+ chunks");

    for (i, chunk) in chunks.iter().enumerate() {
        let msgs = splitter.feed(chunk);
        if i < chunks.len() - 1 {
            // Not the last chunk — should buffer
            assert!(msgs.is_empty(), "chunk {i} should not produce a message");
        } else {
            // Last chunk completes the message
            assert_eq!(msgs.len(), 1);
            let msg = GvpMessage::decode(&msgs[0]).unwrap();
            assert!(matches!(msg, GvpMessage::Result(_)));
        }
    }
}

// ---------------------------------------------------------------------------
// CamConfig fusion_preset round-trip
// ---------------------------------------------------------------------------

#[test]
fn cam_config_fusion_preset_encode_decode() {
    use ironsight::protocol::camera::CamConfig;

    let preset = CamConfig::fusion_preset();
    assert!(preset.is_fusion());
    assert_eq!(preset.resolution_width, 1640);
    assert_eq!(preset.resolution_height, 1232);
    assert!(preset.fusion_camera_mode);

    // Encode and decode round-trip
    let encoded = preset.encode();
    assert_eq!(encoded.len(), 52);
    let decoded = CamConfig::decode(&encoded).unwrap();
    assert!(decoded.is_fusion());
    assert_eq!(decoded.resolution_width, 1640);
    assert_eq!(decoded.resolution_height, 1232);
    assert!(decoded.fusion_camera_mode);
    assert_eq!(decoded.quality, 80);
    assert_eq!(decoded.framerate, 20);
}

// ---------------------------------------------------------------------------
// Command encode round-trips
// ---------------------------------------------------------------------------

#[test]
fn config_request_encode_decode_round_trip() {
    use ironsight::gvp::GvpCommand;

    let cmd = GvpCommand::ConfigRequest;
    let bytes = cmd.encode();

    // Null-terminated
    assert_eq!(bytes.last(), Some(&0x00));

    // Valid JSON
    let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
    let msg = GvpMessage::decode(json_str).unwrap();
    // CONFIG_REQUEST is APP→GVP, decodes as Unknown
    match msg {
        GvpMessage::Unknown { msg_type, .. } => assert_eq!(msg_type, "CONFIG_REQUEST"),
        other => panic!("expected Unknown, got {other:?}"),
    }
}

#[test]
fn trigger_encode_decode_round_trip() {
    use ironsight::gvp::{trigger::Trigger, GvpCommand};

    let trig = Trigger::new("{test-guid-123}".into(), 1771985855.931274);
    let cmd = GvpCommand::Trigger(trig);
    let bytes = cmd.encode();

    let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
    // TRIGGER is APP→GVP, decodes as Unknown
    let msg = GvpMessage::decode(json_str).unwrap();
    assert!(matches!(msg, GvpMessage::Unknown { .. }));

    // But we can also deserialize the trigger body directly
    let value: serde_json::Value = serde_json::from_str(json_str).unwrap();
    assert_eq!(value["type"], "TRIGGER");
    assert_eq!(value["version"], 6);
    assert_eq!(value["guid"], "{test-guid-123}");
    assert_eq!(value["skipTracking"], false);
    assert_eq!(value["frameNumber"], 0);
}

#[test]
fn config_encode_decode_round_trip() {
    use ironsight::gvp::{config::GvpConfig, GvpCommand};

    let cfg = GvpConfig::fusion();
    let cmd = GvpCommand::Config(cfg.clone());
    let bytes = cmd.encode();

    let json_str = std::str::from_utf8(&bytes[..bytes.len() - 1]).unwrap();
    let msg = GvpMessage::decode(json_str).unwrap();
    match msg {
        GvpMessage::Config(decoded) => {
            // fusion() sends 640x480 (APP→GVP metadata), not 1640x1232
            assert_eq!(decoded.camera_configuration.roi_width, 640);
            assert_eq!(decoded.camera_configuration.roi_height, 480);
            assert_eq!(
                decoded.camera_configuration.roi_width,
                cfg.camera_configuration.roi_width
            );
            assert_eq!(
                decoded.camera_configuration.roi_height,
                cfg.camera_configuration.roi_height
            );
            assert_eq!(
                decoded.buffer_configuration.buffer_size_pre_trigger,
                cfg.buffer_configuration.buffer_size_pre_trigger
            );
        }
        other => panic!("expected Config, got {other:?}"),
    }
}
