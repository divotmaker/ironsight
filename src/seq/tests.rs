use super::*;
use crate::frame::RawFrame;
use std::sync::LazyLock;

/// The tests' time origin: the clock is read once, and a test steps time
/// by adding to it.
fn t0() -> Instant {
    static T0: LazyLock<Instant> = LazyLock::new(Instant::now);
    *T0
}

const MS: Duration = Duration::from_millis(1);

/// Build a device → APP envelope.
fn env(src: BusAddr, type_id: u8, payload: Vec<u8>) -> Envelope {
    let frame = RawFrame {
        dest: BusAddr::App,
        src,
        type_id,
        payload,
    };
    Envelope {
        src,
        type_id,
        raw: frame.payload.clone(),
        message: Message::decode(&frame).unwrap(),
    }
}

fn sent(actions: &[Action]) -> Vec<(BusAddr, u8)> {
    actions
        .iter()
        .map(|Action::Send(cmd, dest)| (*dest, cmd.encode(*dest).type_id))
        .collect()
}

fn dsp_status() -> Envelope {
    let mut p = vec![0u8; 29];
    p[0] = 0x1C;
    env(BusAddr::Dsp, 0xAA, p)
}

#[test]
fn dsp_query_nack_moves_on() {
    let (mut seq, _) = DspSequencer::new();
    assert_eq!(sent(&seq.feed(&dsp_status(), t0())), [(BusAddr::Dsp, 0x48)]);
    let nack = env(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
    assert_eq!(sent(&seq.feed(&nack, t0())), [(BusAddr::Dsp, 0x67)]);
}

#[test]
fn dsp_query_timeout_moves_on() {
    let (mut seq, _) = DspSequencer::new();
    seq.feed(&dsp_status(), t0());
    let due = t0() + DSP_QUERY_TIMEOUT;
    assert!(seq.check_timers(due - MS).is_empty());
    assert_eq!(sent(&seq.check_timers(due)), [(BusAddr::Dsp, 0x67)]);
    // Only once.
    assert!(seq.check_timers(due + MS).is_empty());
}

/// DSP device info naming the model `text`.
fn dsp_dev_info(text: &[u8; 14]) -> Envelope {
    let mut p = vec![0u8; 99];
    p[0] = 0x62;
    p[83..97].copy_from_slice(text);
    env(BusAddr::Dsp, 0xE7, p)
}

/// Run a DSP sync up to and including the config query's reply (0xA0);
/// returns what that reply produced.
fn dsp_sync_to_config(seq: &mut DspSequencer, model: &[u8; 14]) -> Vec<Action> {
    dsp_sync_to_config_from(seq, &dsp_status(), model)
}

/// [`dsp_sync_to_config`], with the DSP's first status `status`.
fn dsp_sync_to_config_from(
    seq: &mut DspSequencer,
    status: &Envelope,
    model: &[u8; 14],
) -> Vec<Action> {
    assert_eq!(sent(&seq.feed(status, t0())), [(BusAddr::Dsp, 0x48)]);
    let nack = env(BusAddr::Dsp, 0x94, vec![0x02, 0x40, 0x48]);
    assert_eq!(sent(&seq.feed(&nack, t0())), [(BusAddr::Dsp, 0x67)]);
    assert_eq!(
        sent(&seq.feed(&dsp_dev_info(model), t0())),
        [(BusAddr::Dsp, 0xFD)]
    );
    let prod = |sub| {
        let mut p = vec![0u8; 34];
        p[0] = 0x21;
        p[1] = sub;
        env(BusAddr::Dsp, 0xFD, p)
    };
    assert_eq!(sent(&seq.feed(&prod(0), t0())), [(BusAddr::Dsp, 0xFD)]);
    assert_eq!(sent(&seq.feed(&prod(8), t0())), [(BusAddr::Dsp, 0xFD)]);
    assert_eq!(sent(&seq.feed(&prod(9), t0())), [(BusAddr::Dsp, 0x21)]);
    let mut config = vec![0u8; 69];
    config[0] = 0x44;
    seq.feed(&env(BusAddr::Dsp, 0xA0, config), t0())
}

/// On the Xi the DSP sync ends with the power-mode write (param 0x01 =
/// 1, Normal) to the DSP, sent after the config query's reply and
/// completed by the DSP's 0x95.
#[test]
fn xi_dsp_sync_wakes_the_dsp() {
    let (mut seq, _) = DspSequencer::new();
    let actions = dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
    assert_eq!(actions.len(), 1);
    assert_eq!(
        wire(&actions[0]),
        (BusAddr::Dsp, 0xBF, hex("06000001000001"))
    );
    assert!(!seq.is_complete());
    // The power-mode texts, and an AVR ack, do not complete it.
    for other in [
        env(BusAddr::Dsp, 0xE3, b"  REQ Power Mode : 1 (4) \0".to_vec()),
        env(BusAddr::Dsp, 0xE3, b"  NEW Power Mode : 1 \0".to_vec()),
        env(BusAddr::Avr, 0x95, hex("02303f")),
        env(BusAddr::Dsp, 0x95, hex("024030")),
    ] {
        assert!(seq.feed(&other, t0()).is_empty());
        assert!(!seq.is_complete());
    }
    assert!(
        seq.feed(&env(BusAddr::Dsp, 0x95, hex("02403f")), t0())
            .is_empty()
    );
    assert!(seq.is_complete());
    assert!(seq.take_refused().is_none());
    let sync = seq.into_result();
    assert_eq!(sync.model, DeviceModel::Xi);
    assert!(!sync.power_mode_unanswered);
}

/// A 0x94 refusing the power-mode write ends the DSP sync too, and is
/// kept for the caller once.
#[test]
fn xi_dsp_power_mode_nack_completes() {
    let (mut seq, _) = DspSequencer::new();
    dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
    assert!(
        seq.feed(&env(BusAddr::Dsp, 0x94, hex("02403f")), t0())
            .is_empty()
    );
    assert!(seq.is_complete());
    let refused = seq.take_refused().expect("refusal kept");
    assert_eq!((refused.src, refused.type_id), (BusAddr::Dsp, 0x94));
    assert_eq!(refused.raw, hex("02403f"));
    assert!(seq.take_refused().is_none());
}

/// With no reply to the power-mode write within [`DSP_QUERY_TIMEOUT`]
/// the DSP sync completes without one, flagged in the result.
#[test]
fn xi_dsp_power_mode_timeout_completes() {
    let (mut seq, _) = DspSequencer::new();
    dsp_sync_to_config(&mut seq, b"0.38 FS Xi    ");
    let due = t0() + DSP_QUERY_TIMEOUT;
    assert!(seq.check_timers(due - MS).is_empty());
    assert!(!seq.is_complete());
    assert!(seq.check_timers(due).is_empty());
    assert!(seq.is_complete());
    assert!(seq.take_refused().is_none());
    let sync = seq.into_result();
    assert!(sync.power_mode_unanswered);
    assert_eq!(sync.model, DeviceModel::Xi);
}

/// An X2 DSP status (version 0x1B) whose last byte, the power mode, is
/// `power`: 1 in Standby, 2 in Normal.
fn x2_dsp_status(power: u8) -> Envelope {
    let mut p = vec![0u8; 28];
    p[0] = 0x1B;
    p[27] = power;
    env(BusAddr::Dsp, 0xAA, p)
}

/// An X2 whose DSP reports Standby is sent the power-mode write (param
/// 0x01 = 1, Normal) after the config query's reply; the DSP's 0x95 ends
/// the sync.
#[test]
fn x2_dsp_sync_wakes_a_dsp_in_standby() {
    let (mut seq, _) = DspSequencer::new();
    let actions = dsp_sync_to_config_from(&mut seq, &x2_dsp_status(1), b"0.31 FS X2    ");
    assert_eq!(actions.len(), 1);
    assert_eq!(
        wire(&actions[0]),
        (BusAddr::Dsp, 0xBF, hex("06000001000001"))
    );
    assert!(!seq.is_complete());
    assert!(
        seq.feed(&env(BusAddr::Dsp, 0x95, hex("02403f")), t0())
            .is_empty()
    );
    assert!(seq.is_complete());
    let sync = seq.into_result();
    assert_eq!(sync.model, DeviceModel::X2);
    assert!(!sync.power_mode_unanswered);
}

/// An X2 whose DSP reports Normal is not sent the power-mode write: its
/// DSP sync ends on the config query's reply.
#[test]
fn x2_dsp_sync_in_normal_sends_no_power_mode() {
    let (mut seq, _) = DspSequencer::new();
    let actions = dsp_sync_to_config_from(&mut seq, &x2_dsp_status(2), b"0.31 FS X2    ");
    assert!(actions.is_empty());
    assert!(seq.is_complete());
    assert_eq!(seq.into_result().model, DeviceModel::X2);
}

/// A Mevo's DSP sync ends on the config query's reply, with nothing
/// sent after it.
#[test]
fn mevo_dsp_sync_sends_no_power_mode() {
    let (mut seq, _) = DspSequencer::new();
    let actions = dsp_sync_to_config(&mut seq, b"2.10 FS M2    ");
    assert!(actions.is_empty());
    assert!(seq.is_complete());
    assert_eq!(seq.into_result().model, DeviceModel::Mevo);
}

/// A status poll the AVR leaves unanswered, or refuses (0x94
/// `[02 30 2A]`, a starting AVR), is sent again every
/// `STATUS_REPOLL_INTERVAL` until the AVR answers.
#[test]
fn avr_status_poll_is_re_sent_until_answered() {
    let (mut seq, actions) = AvrSequencer::new(DeviceModel::Xi, t0());
    assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
    let nack = env(BusAddr::Avr, 0x94, hex("02302a"));
    let mut status = vec![0u8; 25];
    status[0] = 0x18;
    let status = env(BusAddr::Avr, 0xAA, status);

    let mut t = t0();
    // Unanswered, then refused: re-sent once per interval, never before.
    for reply in [None, Some(&nack), None] {
        if let Some(reply) = reply {
            assert!(seq.feed(reply, t + MS).is_empty());
        }
        t += STATUS_REPOLL_INTERVAL;
        assert!(seq.check_timers(t - MS).is_empty());
        assert_eq!(sent(&seq.check_timers(t)), [(BusAddr::Avr, 0xAA)]);
        assert!(seq.check_timers(t).is_empty());
    }
    // The AVR answers: the second poll goes out, re-sent the same way.
    assert_eq!(sent(&seq.feed(&status, t)), [(BusAddr::Avr, 0xAA)]);
    t += STATUS_REPOLL_INTERVAL;
    assert!(seq.check_timers(t - MS).is_empty());
    assert_eq!(sent(&seq.check_timers(t)), [(BusAddr::Avr, 0xAA)]);
    assert_eq!(sent(&seq.feed(&status, t)), [(BusAddr::Avr, 0x67)]);
    // Past the status polls nothing is re-sent.
    assert!(seq.repoll_at.is_none());
    assert!(seq.feed(&nack, t).is_empty());
    assert!(seq.check_timers(t + 10 * STATUS_REPOLL_INTERVAL).is_empty());
}

/// Without a refusal the AVR sync sends exactly what it always has.
#[test]
fn mevo_avr_sync_without_nack_is_unchanged() {
    let (mut seq, actions) = AvrSequencer::new(DeviceModel::Mevo, t0());
    assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
    let mut status = vec![0u8; 25];
    status[0] = 0x18;
    let status = env(BusAddr::Avr, 0xAA, status);
    assert_eq!(sent(&seq.feed(&status, t0())), [(BusAddr::Avr, 0xAA)]);
    assert!(seq.check_timers(t0()).is_empty());
    assert_eq!(sent(&seq.feed(&status, t0())), [(BusAddr::Avr, 0x67)]);
    assert!(seq.check_timers(t0()).is_empty());
    assert!(seq.repoll_at.is_none());
}

#[test]
fn xi_avr_sync_ends_after_if_cal() {
    let (mut seq, actions) = AvrSequencer::new(DeviceModel::Xi, t0());
    assert_eq!(sent(&actions), [(BusAddr::Avr, 0xAA)]);
    let mut status = vec![0u8; 25];
    status[0] = 0x18;
    let status = env(BusAddr::Avr, 0xAA, status);
    let dev_info = env(BusAddr::Avr, 0xE7, vec![0u8; 75]);
    let param = |id| env(BusAddr::Avr, 0xBF, vec![0x06, 0x00, 0x00, id, 0, 0, 0]);
    let mut config = vec![0u8; 69];
    config[0] = 0x44;

    assert_eq!(sent(&seq.feed(&status, t0())), [(BusAddr::Avr, 0xAA)]);
    assert_eq!(sent(&seq.feed(&status, t0())), [(BusAddr::Avr, 0x67)]);
    assert_eq!(sent(&seq.feed(&dev_info, t0())), [(BusAddr::Avr, 0x67)]);
    assert_eq!(sent(&seq.feed(&dev_info, t0())), [(BusAddr::Avr, 0xBE)]);
    assert_eq!(sent(&seq.feed(&param(0x0C), t0())), [(BusAddr::Avr, 0xBE)]);
    assert_eq!(sent(&seq.feed(&param(0x0D), t0())), [(BusAddr::Avr, 0x21)]);
    // ConfigResp → IF cal request, no factory cal.
    let config = env(BusAddr::Avr, 0xA0, config);
    assert_eq!(sent(&seq.feed(&config, t0())), [(BusAddr::Avr, 0xD0)]);
    let if_cal = env(BusAddr::Avr, 0xD1, vec![0u8; 171]);
    assert!(seq.feed(&if_cal, t0()).is_empty());
    assert!(seq.is_complete());
    let sync = seq.into_result();
    assert!(sync.factory_cal.is_none());
    assert!(sync.if_cal.is_some());
    assert!(sync.avr_config.is_none());
}

#[test]
fn xi_avr_sync_if_cal_timeout_completes() {
    let (mut seq, _) = AvrSequencer::new(DeviceModel::Xi, t0());
    seq.step = AvrStep::WaitIfCal;
    seq.status = Some(AvrStatus::decode(&[0u8; 25]).unwrap());
    seq.cal_deadline = Some(t0());
    assert!(seq.check_timers(t0()).is_empty());
    assert!(seq.is_complete());
}

#[test]
fn xi_arm_waits_for_armed_text_only() {
    let (mut seq, actions) = ArmSequencer::new(DeviceModel::Xi);
    assert_eq!(sent(&actions), [(BusAddr::Dsp, 0xAA)]);
    assert_eq!(sent(&seq.feed(&dsp_status(), t0())), [(BusAddr::Avr, 0xAA)]);
    let avr_status = env(BusAddr::Avr, 0xAA, vec![0u8; 25]);
    assert_eq!(sent(&seq.feed(&avr_status, t0())), [(BusAddr::Avr, 0xB0)]);
    let ack = env(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
    assert!(seq.feed(&ack, t0()).is_empty());
    assert!(!seq.is_complete());
    let armed = env(BusAddr::Avr, 0xE3, b"\n\rARMED Xi\0".to_vec());
    seq.feed(&armed, t0());
    assert!(seq.is_complete());
}

#[test]
fn mevo_arm_polls_pi() {
    let (mut seq, _) = ArmSequencer::new(DeviceModel::Mevo);
    seq.feed(&dsp_status(), t0());
    seq.feed(&env(BusAddr::Avr, 0xAA, vec![0u8; 25]), t0());
    let ack = env(BusAddr::Avr, 0x95, vec![0x02, 0x30, 0x30]);
    assert_eq!(sent(&seq.feed(&ack, t0())), [(BusAddr::Pi, 0xAA)]);
    seq.feed(&env(BusAddr::Avr, 0xE3, b"\n\rARMED\0".to_vec()), t0());
    assert!(!seq.is_complete());
    seq.feed(&env(BusAddr::Pi, 0xAA, vec![0u8; 17]), t0());
    assert!(seq.is_complete());
}

fn hex(s: &str) -> Vec<u8> {
    let s: String = s.split_whitespace().collect();
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// The Xi's reply to one configuration frame, as the device sends it.
fn xi_reply(type_id: u8, payload: &[u8]) -> Envelope {
    match type_id {
        0xBF => env(BusAddr::Avr, 0x95, hex("02303f")),
        0xB0 => env(BusAddr::Avr, 0x95, hex("023030")),
        0xA5 | 0xA4 => env(BusAddr::Avr, type_id, payload.to_vec()),
        0xBE => env(BusAddr::Avr, 0xBF, hex("06000002000001")),
        _ => panic!("unexpected 0x{type_id:02X}"),
    }
}

/// Run an Xi configuration against the scripted device, with unrelated
/// traffic (texts, a DSP ack, a mode ack) before every reply. Returns
/// every frame sent as (type, payload hex).
fn run_xi(seq: &mut XiConfigSequencer, actions: Vec<Action>) -> Vec<(u8, String)> {
    use std::fmt::Write as _;
    let noise = [
        env(BusAddr::Avr, 0xE3, b"  ARMED CANCELLED\0".to_vec()),
        env(BusAddr::Dsp, 0x95, hex("02403f")),
        env(BusAddr::Avr, 0xB1, hex("020000")),
    ];
    let mut wire = Vec::new();
    let mut pending = actions;
    while let Some(Action::Send(cmd, dest)) = pending.pop() {
        assert!(pending.is_empty(), "more than one frame in flight");
        assert_eq!(dest, BusAddr::Avr);
        let f = cmd.encode(dest);
        wire.push((
            f.type_id,
            f.payload.iter().fold(String::new(), |mut s, b| {
                let _ = write!(s, "{b:02x}");
                s
            }),
        ));
        assert!(wire.len() < 64, "config sequence did not finish");
        for n in &noise {
            assert!(seq.feed(n, t0()).is_empty());
        }
        assert!(!seq.is_complete());
        pending = seq.feed(&xi_reply(f.type_id, &f.payload), t0());
    }
    assert!(seq.is_complete());
    wire
}

fn frames(lines: &[&str]) -> Vec<(u8, String)> {
    lines
        .iter()
        .map(|l| {
            let (t, p) = l.split_once(' ').unwrap();
            (u8::from_str_radix(t, 16).unwrap(), p.replace(' ', ""))
        })
        .collect()
}

/// The Xi connect-time block for a session in `mode` with club type
/// `club`, radar cal 2438 mm / 25 mm.
fn xi_connect_block(mode: &str, club: &str) -> Vec<(u8, String)> {
    let club = format!("BF 06000016 0000{club}");
    let mode = format!("A5 0200{mode}");
    frames(&[
        "BF 06000001 000000",
        "A5 020008",
        "BE 03000002",
        "A4 06 0986 00 19 0000",
        "BF 06000008 000001",
        "BF 06000007 00140b",
        &club,
        "BF 06000025 000000",
        "BF 06000007 005448",
        "BF 06000002 000003",
        "BF 06000014 000002",
        "BF 08000027 0005780000",
        "BF 08000028 0001400000",
        "BF 06000007 005448",
        &mode,
        "B0 0100",
        "BF 08000028 0001400000",
    ])
}

/// The Xi mode block for `mode` and club type `club`.
fn xi_mode_block(mode: &str, club: &str) -> Vec<(u8, String)> {
    let club = format!("BF 06000016 0000{club}");
    let mode = format!("A5 0200{mode}");
    frames(&[
        "BF 06000008 000001",
        "B0 0100",
        &club,
        "BF 08000020 0000000000",
        "BF 06000007 005448",
        &mode,
        "B0 0100",
        "BF 08000028 0001400000",
        &club,
        "BF 08000020 0000000000",
    ])
}

fn xi(mode: u8, shot_type: XiShotType) -> XiSettings {
    XiSettings {
        mode,
        shot_type,
        ..XiSettings::default()
    }
}

#[test]
fn xi_settings_default_is_full_swing_limited_flight() {
    let s = XiSettings::default();
    assert_eq!(s.mode, 9);
    assert_eq!(s.shot_type, XiShotType::FullSwing);
    assert_eq!(s.radar_cal.range_mm, 2438);
    assert_eq!(s.radar_cal.height_mm, 25);
    assert_eq!(XiShotType::FullSwing.param_value(), 1);
    assert_eq!(XiShotType::Pitching.param_value(), 2);
}

#[test]
fn xi_settings_constructors() {
    use crate::protocol::config::{MODE_FULL_OUTDOOR, MODE_LIMITED_FLIGHT};
    let cal = RadarCal {
        range_mm: 3048,
        height_mm: 25,
    };
    for mode in [MODE_LIMITED_FLIGHT, MODE_FULL_OUTDOOR] {
        assert_eq!(
            XiSettings::full_swing(mode, cal.clone()),
            XiSettings {
                mode,
                shot_type: XiShotType::FullSwing,
                radar_cal: cal.clone(),
            }
        );
    }
    assert_eq!(
        XiSettings::pitching(cal.clone()),
        XiSettings {
            mode: MODE_LIMITED_FLIGHT,
            shot_type: XiShotType::Pitching,
            radar_cal: cal,
        }
    );
    assert_eq!(
        XiSettings::full_swing(MODE_LIMITED_FLIGHT, XiSettings::default().radar_cal),
        XiSettings::default()
    );
}

/// Run an AVR configuration against replies that ack every frame, and
/// return the frames sent as (type, payload hex).
fn run_avr(settings: AvrSettings) -> Vec<(u8, String)> {
    let (mut seq, mut pending) = AvrConfigSequencer::new(settings);
    let mut wire = Vec::new();
    while let Some(Action::Send(cmd, dest)) = pending.pop() {
        assert!(pending.is_empty());
        assert_eq!(dest, BusAddr::Avr);
        let f = cmd.encode(dest);
        wire.push((f.type_id, hex_of(&f.payload)));
        assert!(wire.len() < 32, "did not finish");
        pending = seq.feed(&xi_reply(f.type_id, &f.payload), t0());
    }
    assert!(seq.is_complete());
    wire
}

/// The Mevo+ full configuration: ball type, track fraction and tee
/// height, each followed by B0, then the mode and the radar
/// calibration, each followed by B0.
#[test]
fn mevo_settings_frames() {
    use crate::protocol::config::MODE_LIMITED_FLIGHT;
    let settings = MevoSettings {
        mode: MODE_LIMITED_FLIGHT,
        ball_type: 1,
        track_fraction: 0.8,
        tee_height_m: 0.0381,
        radar_cal: RadarCal {
            range_mm: 2438,
            height_mm: 0,
        },
    };
    assert_eq!(
        run_avr(settings.into()),
        frames(&[
            "B0 0100",
            "BF 06000006 000001",
            "B0 0100",
            "BF 0800000f 0000666666",
            "B0 0100",
            "BF 08000026 fffc4e075f",
            "B0 0100",
            "A5 020009",
            "B0 0100",
            "A4 06 0986 00 00 0000",
            "B0 0100",
        ])
    );
}

#[test]
fn mevo_settings_params_in_order() {
    let settings = MevoSettings {
        mode: 5,
        ball_type: 0,
        track_fraction: 0.6,
        tee_height_m: 0.05,
        radar_cal: RadarCal {
            range_mm: 3048,
            height_mm: 25,
        },
    };
    let avr: AvrSettings = settings.clone().into();
    assert_eq!(avr.mode, 5);
    assert_eq!(avr.radar_cal, Some(settings.radar_cal.clone()));
    let params: Vec<(u8, String)> = avr
        .params
        .iter()
        .map(|p| (p.param_id, format!("{:?}", p.value)))
        .collect();
    assert_eq!(
        params,
        [
            (0x06, "Int24(0)".to_string()),
            (0x0F, "Float40(0.6)".to_string()),
            (0x26, "Float40(0.05)".to_string()),
        ]
    );
}

/// A mode change sends the mode alone between two config gates.
#[test]
fn mode_only_frames() {
    for (mode, hex) in [(3, "020003"), (5, "020005"), (8, "020008"), (9, "020009")] {
        assert_eq!(
            run_avr(AvrSettings::mode_only(mode)),
            frames(&["B0 0100", &format!("A5 {hex}"), "B0 0100"])
        );
    }
}

#[test]
fn xi_connect_full_swing_frames() {
    let (mut seq, actions) = XiConfigSequencer::connect(&XiSettings::default());
    let mut expected = xi_connect_block("09", "01");
    expected.extend(xi_mode_block("09", "01"));
    assert_eq!(run_xi(&mut seq, actions), expected);
}

#[test]
fn xi_connect_pitching_frames() {
    use crate::protocol::config::MODE_LIMITED_FLIGHT;
    let settings = xi(MODE_LIMITED_FLIGHT, XiShotType::Pitching);
    let (mut seq, actions) = XiConfigSequencer::connect(&settings);
    let mut expected = xi_connect_block("09", "02");
    expected.extend(xi_mode_block("09", "02"));
    assert_eq!(run_xi(&mut seq, actions), expected);
}

#[test]
fn xi_connect_full_outdoor_changes_only_the_mode() {
    use crate::protocol::config::MODE_FULL_OUTDOOR;
    let settings = xi(MODE_FULL_OUTDOOR, XiShotType::FullSwing);
    let (mut seq, actions) = XiConfigSequencer::connect(&settings);
    let mut expected = xi_connect_block("08", "01");
    expected.extend(xi_mode_block("08", "01"));
    assert_eq!(run_xi(&mut seq, actions), expected);
}

#[test]
fn xi_mode_change_frames() {
    for (shot_type, club) in [(XiShotType::FullSwing, "01"), (XiShotType::Pitching, "02")] {
        let (mut seq, actions) = XiConfigSequencer::mode_change(&xi(9, shot_type));
        assert_eq!(run_xi(&mut seq, actions), xi_mode_block("09", club));
    }
}

#[test]
fn xi_radar_cal_comes_from_settings() {
    let settings = XiSettings {
        radar_cal: RadarCal {
            range_mm: 3048,
            height_mm: 0,
        },
        ..XiSettings::default()
    };
    let (mut seq, actions) = XiConfigSequencer::connect(&settings);
    let wire = run_xi(&mut seq, actions);
    let cal: Vec<&(u8, String)> = wire.iter().filter(|(t, _)| *t == 0xA4).collect();
    assert_eq!(cal, [&(0xA4, "060be800000000".to_string())]);
    let (mut seq, actions) = XiConfigSequencer::mode_change(&settings);
    assert!(run_xi(&mut seq, actions).iter().all(|(t, _)| *t != 0xA4));
}

#[test]
fn xi_config_waits_for_the_matching_reply() {
    let (mut seq, _) = XiConfigSequencer::mode_change(&XiSettings::default());
    // Step 1 is a BF write: a B0 ack does not complete it.
    assert!(
        seq.feed(&env(BusAddr::Avr, 0x95, hex("023030")), t0())
            .is_empty()
    );
    let next = seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f")), t0());
    assert_eq!(sent(&next), [(BusAddr::Avr, 0xB0)]);
    // Step 2 is B0: a BF ack does not complete it, a rejection does.
    assert!(
        seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f")), t0())
            .is_empty()
    );
    let next = seq.feed(&env(BusAddr::Avr, 0x94, hex("023030")), t0());
    assert_eq!(sent(&next), [(BusAddr::Avr, 0xBF)]);
    assert!(!seq.is_complete());
}

/// For each step type, a 0x94 refusing that type's first frame ends
/// the step: the sequence sends exactly the frames it sends without the
/// refusal, finishes, and keeps the 0x94 for the caller.
#[test]
fn xi_config_nack_continues_every_step_type() {
    let mut expected = xi_connect_block("09", "01");
    expected.extend(xi_mode_block("09", "01"));
    for refused_type in [0xA5, 0xA4, 0xBE, 0xBF, 0xB0] {
        let (mut seq, mut pending) = XiConfigSequencer::connect(&XiSettings::default());
        let mut wire = Vec::new();
        let mut refusals = Vec::new();
        let mut armed_nack = true;
        while let Some(Action::Send(cmd, dest)) = pending.pop() {
            assert!(pending.is_empty());
            let f = cmd.encode(dest);
            wire.push((f.type_id, hex_of(&f.payload)));
            assert!(wire.len() < 64, "0x{refused_type:02X}: did not finish");
            let reply = if armed_nack && f.type_id == refused_type {
                armed_nack = false;
                env(BusAddr::Avr, 0x94, vec![0x02, 0x30, refused_type & 0x7F])
            } else {
                xi_reply(f.type_id, &f.payload)
            };
            pending = seq.feed(&reply, t0());
            refusals.extend(seq.take_refused());
        }
        assert!(seq.is_complete(), "0x{refused_type:02X}");
        assert_eq!(wire, expected, "0x{refused_type:02X}");
        assert_eq!(refusals.len(), 1, "0x{refused_type:02X}");
        assert_eq!(refusals[0].type_id, 0x94);
        assert_eq!(refusals[0].raw, [0x02, 0x30, refused_type & 0x7F]);
    }
}

/// A 0x94 for a different command type does not end a step.
#[test]
fn xi_config_ignores_a_nack_of_another_type() {
    let (mut seq, _) = XiConfigSequencer::connect(&XiSettings::default());
    // Step 1 is the BF Altitude write.
    for other in ["023030", "023025", "023023", "02302a"] {
        assert!(
            seq.feed(&env(BusAddr::Avr, 0x94, hex(other)), t0())
                .is_empty()
        );
    }
    // A DSP refusal of a BF does not end an AVR step.
    assert!(
        seq.feed(&env(BusAddr::Dsp, 0x94, hex("02403f")), t0())
            .is_empty()
    );
    assert!(seq.take_refused().is_none());
    let next = seq.feed(&env(BusAddr::Avr, 0x95, hex("02303f")), t0());
    assert_eq!(sent(&next), [(BusAddr::Avr, 0xA5)]);
}

fn hex_of(b: &[u8]) -> String {
    use std::fmt::Write as _;
    b.iter().fold(String::new(), |mut s, x| {
        let _ = write!(s, "{x:02x}");
        s
    })
}

#[test]
fn xi_param_floats_encode_as_written() {
    use crate::codec::{read_float40, write_float40};
    for (value, wire) in [
        (XI_OUTDOOR_MIN_TRACK_PCT, hex("0005780000")),
        (XI_INDOOR_DRAG_FRACTION, hex("0001400000")),
        (XI_PARAM_20, hex("0000000000")),
    ] {
        let mut buf = Vec::new();
        write_float40(&mut buf, value);
        assert_eq!(buf, wire);
        assert!((read_float40(&wire, 0).unwrap() - value).abs() < 1e-12);
    }
    assert!((read_float40(&hex("0005780000"), 0).unwrap() - 30.0).abs() < 1e-12);
}

/// Encode an action as (dest, type, payload).
fn wire(action: &Action) -> (BusAddr, u8, Vec<u8>) {
    let Action::Send(cmd, dest) = action;
    let f = cmd.encode(*dest);
    (f.dest, f.type_id, f.payload)
}

#[test]
fn xi_flight_results_are_acked() {
    let e8 = env(BusAddr::Avr, 0xE8, vec![0u8; 94]);
    let d4 = env(BusAddr::Avr, 0xD4, vec![0u8; 157]);
    let ack = result_ack(DeviceModel::Xi, &e8).expect("E8 ack");
    assert_eq!(wire(&ack), (BusAddr::Avr, 0x95, vec![0x02, 0x10, 0x68]));
    let ack = result_ack(DeviceModel::Xi, &d4).expect("D4 ack");
    assert_eq!(wire(&ack), (BusAddr::Avr, 0x95, vec![0x02, 0x10, 0x54]));
    let Action::Send(cmd, dest) = &ack;
    assert_eq!(cmd.debug_hex(*dest), "APP→AVR 0x95 3B | 021054");
}

#[test]
fn only_xi_flight_results_are_acked() {
    let e8 = env(BusAddr::Avr, 0xE8, vec![0u8; 94]);
    let d4 = env(BusAddr::Avr, 0xD4, vec![0u8; 157]);
    // The Mevo+ and Gen2 get no result acks.
    assert!(result_ack(DeviceModel::Mevo, &e8).is_none());
    assert!(result_ack(DeviceModel::Mevo, &d4).is_none());
    // Other results and acks are not acked on the Xi.
    for (type_id, len) in [(0xED, 167), (0xEF, 138), (0xD9, 152), (0x95, 3)] {
        let other = env(BusAddr::Avr, type_id, vec![0x02; len]);
        assert!(
            result_ack(DeviceModel::Xi, &other).is_none(),
            "0x{type_id:02X}"
        );
    }
    let text = env(BusAddr::Avr, 0xE5, b"PROCESSED\0".to_vec());
    assert!(result_ack(DeviceModel::Xi, &text).is_none());
}

#[test]
fn keepalive_polls_pi_only_with_camera() {
    assert_eq!(
        sent(&keepalive_polls(DeviceModel::Mevo)),
        [
            (BusAddr::Dsp, 0xAA),
            (BusAddr::Avr, 0xAA),
            (BusAddr::Pi, 0xAA)
        ]
    );
    assert_eq!(
        sent(&keepalive_polls(DeviceModel::Xi)),
        [(BusAddr::Dsp, 0xAA), (BusAddr::Avr, 0xAA)]
    );
}
