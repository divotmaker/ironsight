//! `FakeDevice` and `Shot` driven through `BinaryConnection` and
//! `BinaryClient`, pinning the frames ironsight sends.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use super::*;
use crate::client::{BinaryClient, BinaryEvent};
use crate::protocol::camera::CamConfig;
use crate::protocol::handshake::DeviceModel;
use crate::protocol::status::AvrStatus;
use crate::protocol::{TYPE_DEV_INFO_RESP, TYPE_STATUS};
use crate::seq::{self, XiSettings};

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// One frame as the connection's hooks see it: direction, source,
/// destination, type, payload hex.
type Seen = (&'static str, BusAddr, BusAddr, u8, String);

/// Record every frame sent and received on `conn`, in order.
fn record(conn: &mut BinaryConnection<FakeDevice>) -> Arc<Mutex<Vec<Seen>>> {
    let seen: Arc<Mutex<Vec<Seen>>> = Arc::default();
    let tx = Arc::clone(&seen);
    conn.set_on_send(move |cmd, dest| {
        let f = cmd.encode(dest);
        tx.lock()
            .unwrap()
            .push(("tx", f.src, dest, f.type_id, hex(&f.payload)));
    });
    let rx = Arc::clone(&seen);
    conn.set_on_recv(move |env| {
        rx.lock()
            .unwrap()
            .push(("rx", env.src, BusAddr::App, env.type_id, hex(&env.raw)));
    });
    seen
}

/// Poll `client` until `pred` matches an event; return every event seen.
fn poll_until(
    client: &mut BinaryClient<FakeDevice>,
    pred: impl Fn(&BinaryEvent) -> bool,
) -> Vec<BinaryEvent> {
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut events = Vec::new();
    while Instant::now() < deadline {
        match client.poll().expect("poll") {
            Some(e) => {
                let done = pred(&e);
                events.push(e);
                if done {
                    return events;
                }
            }
            None => std::thread::sleep(Duration::from_millis(1)),
        }
    }
    panic!("event not seen; got {events:?}");
}

#[test]
fn shot_frames() {
    let shot = Shot::xi_no_read();
    let frames: Vec<(BusAddr, u8, String)> = shot
        .frames
        .iter()
        .map(|f| {
            assert_eq!(f.dest, BusAddr::App);
            (f.src, f.type_id, hex(&f.payload))
        })
        .collect();
    use BusAddr::Avr;
    assert_eq!(
        frames,
        [
            (
                Avr,
                0xE5,
                "1542414c4c20545249474745523a20313837206d7300".into()
            ),
            (Avr, 0xE8, hex(fixtures::XI_NO_READ_E8)),
            (Avr, 0xED, hex(fixtures::XI_NO_READ_ED)),
            (Avr, 0xE5, "0a50524f43455353454400".into()),
            (Avr, 0xE5, "0549444c4500".into()),
            (Avr, 0xB1, "020000".into()),
        ]
    );
    assert_eq!(shot.club_reply, fixtures::XI_NO_READ_ED);
    assert_eq!(
        hex(&Shot::xi_chip().frames[0].payload),
        "1542414c4c20545249474745523a20313233206d7300"
    );
    let types: Vec<u8> = Shot::xi_full_swing()
        .frames
        .iter()
        .map(|f| f.type_id)
        .collect();
    assert_eq!(types, [0xE5, 0xE8, 0xD4, 0xED, 0xEF, 0xE5, 0xE5, 0xB1]);
}

#[test]
fn encoders_decode() {
    let status = avr_status([0x12, 0x34]);
    assert_eq!(status.len(), 25);
    let decoded = AvrStatus::decode(&status).unwrap();
    assert_eq!((decoded.version, decoded.state), (0x18, 0));
    assert_eq!((decoded.hw_id_hi, decoded.hw_id_lo), (0x12, 0x34));

    let info = dev_info(["0.37   M2 TR 03", "Jan 21 2026", "11:29:54"]);
    assert_eq!(info.len(), 75);
    let decoded = crate::protocol::handshake::DevInfoResp::decode(&info).unwrap();
    assert_eq!(decoded.text, "0.37   M2 TR 03 Jan 21 2026 11:29:54");
    assert_eq!(decoded.version(), Some("0.37"));
    // Long slot text is cut to 16 bytes.
    let decoded =
        crate::protocol::handshake::DevInfoResp::decode(&dev_info(["0123456789abcdefXYZ", "", ""]))
            .unwrap();
    assert_eq!(decoded.text, "0123456789abcdef");
}

/// A shot through `BinaryClient` on a Mevo, with identity frames before and
/// during it: every frame in order as the connection's hooks see it, with
/// the post-shot requests (0x69 x2, 0x6D) and the re-arm (B0 `[01 01]`)
/// ironsight sends, and the device's replies.
#[test]
fn shot_sequence_through_the_client() {
    let device = FakeDevice::new();
    let mut shot = Shot::xi_full_swing();
    // A status frame between the early and the full result.
    shot.frames.insert(
        2,
        RawFrame {
            dest: BusAddr::App,
            src: BusAddr::Avr,
            type_id: TYPE_STATUS,
            payload: avr_status([0x12, 0x34]),
        },
    );
    device.push(BusAddr::Avr, TYPE_STATUS, avr_status([0x12, 0x34]));
    device.push(
        BusAddr::Avr,
        TYPE_DEV_INFO_RESP,
        dev_info(["0.37   M2 TR 03", "Jan 21 2026", "SERIAL9"]),
    );
    device.push_shot(&shot);

    let mut conn = device.connection();
    let seen = record(&mut conn);
    let mut client = BinaryClient::new(conn);
    let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::ShotComplete(_)));
    assert!(events.iter().any(|e| matches!(e, BinaryEvent::Trigger)));

    let got: Vec<(&str, u8)> = seen.lock().unwrap().iter().map(|s| (s.0, s.3)).collect();
    assert_eq!(
        got,
        [
            ("rx", 0xAA),
            ("rx", 0xE7),
            ("rx", 0xE5),
            ("rx", 0xE8),
            ("rx", 0xAA),
            ("rx", 0xD4),
            ("rx", 0xED),
            ("rx", 0xEF),
            ("rx", 0xE5),
            ("tx", 0x69),
            ("tx", 0x69),
            ("rx", 0xE5),
            ("tx", 0x6D),
            ("rx", 0xB1),
            ("rx", 0xED),
            ("tx", 0xB0),
            ("rx", 0x95),
            ("rx", 0xE3),
        ]
    );
    let tx: Vec<(BusAddr, BusAddr, u8, String)> = seen
        .lock()
        .unwrap()
        .iter()
        .filter(|s| s.0 == "tx")
        .map(|s| (s.1, s.2, s.3, s.4.clone()))
        .collect();
    use BusAddr::{App, Avr};
    assert_eq!(
        tx,
        [
            (App, Avr, 0x69, String::new()),
            (App, Avr, 0x69, String::new()),
            (App, Avr, 0x6D, String::new()),
            (App, Avr, 0xB0, "0101".into()),
        ]
    );
    let replies: Vec<(u8, String)> = seen.lock().unwrap()[16..]
        .iter()
        .map(|s| (s.3, s.4.clone()))
        .collect();
    assert_eq!(
        replies,
        [(0x95, "023030".into()), (0xE3, hex(b"\n\rARMED\0"))]
    );
    // The device's log matches what the client sent.
    let sent: Vec<u8> = device.sent().iter().map(|f| f.type_id).collect();
    assert_eq!(sent, [0x69, 0x69, 0x6D, 0xB0]);
}

/// The Xi's result acks as sent on the connection: 0x95 `[02 10 68]` for
/// an E8 and `[02 10 54]` for a D4, APP to AVR.
#[test]
fn xi_result_acks() {
    let device = FakeDevice::new();
    device.push(BusAddr::Avr, TYPE_FLIGHT_RESULT_V1, fixtures::XI_CHIP_E8);
    device.push(BusAddr::Avr, TYPE_FLIGHT_RESULT, fixtures::XI_SHOT_D4);
    let mut conn = device.connection();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut acked = 0;
    while acked < 2 {
        assert!(Instant::now() < deadline, "results never arrived");
        if let Some(env) = conn.recv().expect("recv") {
            let ack = seq::result_ack(DeviceModel::Xi, &env).expect("result ack");
            seq::send_action(&mut conn, ack).expect("send");
            acked += 1;
        }
    }
    let sent: Vec<(BusAddr, BusAddr, u8, String)> = device
        .sent()
        .iter()
        .map(|f| (f.src, f.dest, f.type_id, hex(&f.payload)))
        .collect();
    use BusAddr::{App, Avr};
    assert_eq!(
        sent,
        [
            (App, Avr, 0x95, "021068".into()),
            (App, Avr, 0x95, "021054".into()),
        ]
    );
}

/// `FakeDevice::xi` answers the Xi's whole connection: the handshake
/// (ending the DSP sync with the power-mode write, DSP param 0x01 = 1),
/// the configuration and the arm; a shot then gets its result acks.
#[test]
fn xi_connection_through_the_client() {
    let device = FakeDevice::xi();
    let mut client = BinaryClient::new(device.connection());
    client.handshake();
    let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Handshake(_)));
    let Some(BinaryEvent::Handshake(h)) = events.last() else {
        unreachable!()
    };
    assert_eq!(h.model, DeviceModel::Xi);
    assert!(!h.dsp.power_mode_unanswered);

    let line = |f: &RawFrame| format!("{} {:02X} {}", f.dest, f.type_id, hex(&f.payload));
    let sent: Vec<String> = device.sent().iter().map(line).collect();
    assert!(
        sent.contains(&"DSP BF 06000001000001".to_string()),
        "{sent:?}"
    );
    let replies: Vec<(BusAddr, u8, Vec<u8>)> = device
        .pushed()
        .into_iter()
        .filter(|f| f.src == BusAddr::Dsp && matches!(f.type_id, 0x95 | 0xE3))
        .map(|f| (f.src, f.type_id, f.payload))
        .collect();
    assert_eq!(
        replies,
        [
            (BusAddr::Dsp, 0x95, vec![0x02, 0x40, 0x3F]),
            (BusAddr::Dsp, 0xE3, b"  REQ Power Mode : 1 (0) \0".to_vec()),
            (BusAddr::Dsp, 0xE3, b"  NEW Power Mode : 1 \0".to_vec()),
        ]
    );

    client.configure_xi(XiSettings::default());
    client.arm();
    let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, BinaryEvent::StepRefused(_)))
    );
    assert_eq!(
        device.sent().last().map(line).as_deref(),
        Some("AVR B0 0101")
    );

    let before = device.sent().len();
    device.push_shot(&Shot::xi_full_swing());
    poll_until(&mut client, |e| matches!(e, BinaryEvent::ShotComplete(_)));
    let acks: Vec<String> = device.sent()[before..]
        .iter()
        .filter(|f| f.type_id == 0x95)
        .map(line)
        .collect();
    assert_eq!(acks, ["AVR 95 021068", "AVR 95 021054"]);
}

/// `FakeDevice::new` answers the status polls of `BinaryClient::arm`: DSP,
/// AVR, the arm, then the camera Pi.
#[test]
fn new_device_arms_through_the_client() {
    let device = FakeDevice::new();
    let mut client = BinaryClient::new(device.connection());
    client.arm();
    poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
    let sent: Vec<(BusAddr, u8, String)> = device
        .sent()
        .iter()
        .map(|f| (f.dest, f.type_id, hex(&f.payload)))
        .collect();
    use BusAddr::{Avr, Dsp, Pi};
    assert_eq!(
        sent,
        [
            (Dsp, 0xAA, "0101".into()),
            (Avr, 0xAA, "0101".into()),
            (Avr, 0xB0, "0101".into()),
            (Pi, 0xAA, "0103".into()),
        ]
    );
    let replies: Vec<(BusAddr, u8)> = device.pushed().iter().map(|f| (f.src, f.type_id)).collect();
    assert_eq!(
        replies,
        [
            (Dsp, 0xAA),
            (Avr, 0xAA),
            (Avr, 0x95),
            (Avr, 0xE3),
            (Pi, 0xAA)
        ]
    );
}

/// A shot whose result frames carry another bus address than the AVR: the
/// client completes it and sends no result acks.
#[test]
fn results_from_another_bus_complete_the_shot() {
    let shot = Shot::new(
        63,
        BusAddr::Dsp,
        [
            (TYPE_FLIGHT_RESULT_V1, fixtures::XI_SHOT_E8.to_vec()),
            (TYPE_FLIGHT_RESULT, fixtures::XI_SHOT_D4.to_vec()),
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
        ],
        fixtures::XI_SHOT_ED.to_vec(),
    );
    let srcs: Vec<(BusAddr, u8)> = shot.frames.iter().map(|f| (f.src, f.type_id)).collect();
    use BusAddr::{Avr, Dsp};
    assert_eq!(
        srcs,
        [
            (Avr, 0xE5),
            (Dsp, 0xE8),
            (Dsp, 0xD4),
            (Dsp, 0xED),
            (Avr, 0xE5),
            (Avr, 0xE5),
            (Avr, 0xB1),
        ]
    );

    let device = FakeDevice::new();
    device.push_shot(&shot);
    let mut client = BinaryClient::new(device.connection());
    let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::ShotComplete(_)));
    let Some(BinaryEvent::ShotComplete(data)) = events.last() else {
        unreachable!()
    };
    assert!(data.flight.is_some());
    assert!(data.club.is_some());
    let sent: Vec<u8> = device.sent().iter().map(|f| f.type_id).collect();
    assert_eq!(sent, [0x69, 0x69, 0x6D, 0xB0]);
}

/// Shots pushed before the client asks are answered in order: each shot
/// result request (0x6D) gets the next shot's club result, and a request
/// beyond the pushed shots gets none.
#[test]
fn club_replies_follow_the_shots() {
    use std::io::Write;

    let mut device = FakeDevice::new();
    device.push_shot(&Shot::xi_full_swing());
    device.push_shot(&Shot::xi_chip());
    let request = RawFrame {
        dest: BusAddr::Avr,
        src: BusAddr::App,
        type_id: 0x6D,
        payload: vec![],
    }
    .encode();
    let before = device.pushed().len();
    for _ in 0..3 {
        device.write_all(&request).expect("write");
    }
    let replies: Vec<(u8, Vec<u8>)> = device.pushed()[before..]
        .iter()
        .map(|f| (f.type_id, f.payload.clone()))
        .collect();
    assert_eq!(
        replies,
        [
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
            (TYPE_CLUB_RESULT, fixtures::XI_CHIP_ED.to_vec()),
        ]
    );
}

/// The camera steps a fault can hit: the frame type the Pi mishandles, and
/// the step the client reports.
const CAMERA_STEPS: [(u8, &str); 4] = [
    (0x82, "camera config ack"),
    (0x83, "camera config readback"),
    (0x81, "camera start ack"),
    (0xBF, "camera parameter ack"),
];

const SHORT_CAMERA_TIMEOUT: Duration = Duration::from_millis(60);

fn unfinished(events: &[BinaryEvent]) -> Vec<(&'static str, bool, Option<String>)> {
    events
        .iter()
        .filter_map(|e| match e {
            BinaryEvent::CameraSetupUnfinished {
                step,
                fusion,
                refused,
            } => Some((*step, *fusion, refused.as_ref().map(|env| hex(&env.raw)))),
            _ => None,
        })
        .collect()
}

/// A camera setup the Pi never finishes ends after the camera timeout, at
/// the step it stopped at; the queued arm runs and a shot still completes.
#[test]
fn a_camera_setup_the_device_never_finishes_gives_up_and_the_arm_runs() {
    for (frame_type, step) in CAMERA_STEPS {
        for (config, fusion) in [
            (CamConfig::warmup_preset(), false),
            (CamConfig::fusion_preset(), true),
            (CamConfig::raw_fusion_preset(), true),
        ] {
            let device = FakeDevice::mevo_plus();
            device.set_camera_fault(Some(CameraFault::Ignore(frame_type)));
            let mut client = BinaryClient::new(device.connection());
            client.set_camera_setup_timeout(SHORT_CAMERA_TIMEOUT);
            let started = Instant::now();
            client.configure_cam(config);
            client.arm();
            let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
            assert!(
                started.elapsed() >= SHORT_CAMERA_TIMEOUT,
                "0x{frame_type:02X}"
            );
            assert_eq!(
                unfinished(&events),
                [(step, fusion, None)],
                "0x{frame_type:02X}"
            );
            assert!(
                !events.iter().any(|e| matches!(e, BinaryEvent::Configured)),
                "0x{frame_type:02X}"
            );
            assert!(client.is_armed());

            device.push_shot(&Shot::xi_full_swing());
            let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::ShotComplete(_)));
            assert!(unfinished(&events).is_empty());
            assert!(client.is_armed());
        }
    }
}

/// A 0x94 refusing a camera step ends the setup at once, with the frame.
#[test]
fn a_refused_camera_step_gives_up_at_once_and_the_arm_runs() {
    for (frame_type, step) in CAMERA_STEPS {
        let device = FakeDevice::mevo_plus();
        device.set_camera_fault(Some(CameraFault::Refuse(frame_type)));
        let mut client = BinaryClient::new(device.connection());
        client.set_camera_setup_timeout(Duration::from_secs(60));
        client.configure_cam(CamConfig::fusion_preset());
        client.arm();
        let started = Instant::now();
        let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
        assert!(started.elapsed() < Duration::from_secs(5));
        let raw = hex(&[0x02, 0x12, frame_type & 0x7F]);
        assert_eq!(
            unfinished(&events),
            [(step, true, Some(raw))],
            "0x{frame_type:02X}"
        );
        assert!(client.is_armed());
    }
}

/// A refusal of a camera frame the setup did not send, or from another
/// bus, does not end it.
#[test]
fn a_nack_of_another_command_does_not_end_a_camera_setup() {
    let device = FakeDevice::mevo_plus();
    // The Pi refuses a frame the setup never sends.
    device.set_camera_fault(Some(CameraFault::Refuse(0xB0)));
    device.push(BusAddr::Avr, 0x94, vec![0x02, 0x30, 0x02]);
    let mut client = BinaryClient::new(device.connection());
    client.configure_cam(CamConfig::fusion_preset());
    client.arm();
    let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
    assert!(unfinished(&events).is_empty());
    assert!(events.iter().any(|e| matches!(e, BinaryEvent::Configured)));
}

/// A device that finishes the setup reports `Configured`, then arms, with no
/// unfinished event, whatever the camera mode.
#[test]
fn a_camera_setup_the_device_finishes_is_unchanged() {
    for config in [
        CamConfig::standard_preset(),
        CamConfig::warmup_preset(),
        CamConfig::fusion_preset(),
        CamConfig::raw_fusion_preset(),
    ] {
        let device = FakeDevice::mevo_plus();
        let mut client = BinaryClient::new(device.connection());
        client.set_camera_setup_timeout(Duration::from_secs(60));
        client.configure_cam(config);
        client.arm();
        let events = poll_until(&mut client, |e| matches!(e, BinaryEvent::Armed));
        assert!(unfinished(&events).is_empty());
        assert_eq!(
            events
                .iter()
                .filter(|e| matches!(e, BinaryEvent::Configured))
                .count(),
            1
        );
    }
}

/// The camera deadline applies to camera configuration only: another
/// operation that gets no reply still ends in the operation timeout.
#[test]
fn the_camera_deadline_does_not_apply_to_other_operations() {
    let device = FakeDevice::new();
    // `new()` answers nothing of the Xi configuration.
    let mut client = BinaryClient::new(device.connection());
    client.set_camera_setup_timeout(Duration::ZERO);
    client.set_operation_timeout(SHORT_CAMERA_TIMEOUT);
    client.configure_xi(XiSettings::default());
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        match client.poll() {
            Err(crate::ConnError::Timeout) => return,
            Err(e) => panic!("{e}"),
            Ok(_) => std::thread::sleep(Duration::from_millis(1)),
        }
    }
    panic!("no timeout");
}
