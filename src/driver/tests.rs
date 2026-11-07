//! Whole driver sessions on [`FakeDevice`], through
//! [`Mevo::connect_with`].

use std::cell::Cell;
use std::io;
use std::rc::Rc;
use std::time::{Duration, Instant};

use flightrelay::config::DriverConfig;
use flightrelay::driver::{
    ConnectCtx, ConnectHost, DeviceCommand, DeviceDriver, DeviceEvent, Events, HostFlags, Level,
    Stop,
};
use flightrelay::types::{Basis, ClubData, FlightResult as Flight, Launch};
use flightrelay::{DetectionMode, FrpEvent, RawDir, Severity, Source};

use super::*;
use crate::addr::BusAddr;
use crate::config::{Config, SpinSource};
use crate::protocol::{TYPE_CLUB_RESULT, TYPE_FLIGHT_RESULT, TYPE_FLIGHT_RESULT_V1};
use crate::testing::{FakeDevice, Shot as FakeShot, fixtures};

struct Host(DeviceCommand);

impl ConnectHost for Host {
    fn tick(&mut self) {}
    fn latched(&self) -> &DeviceCommand {
        &self.0
    }
}

/// A host for one test: its flags and every event the driver emitted.
struct Run {
    flags: HostFlags,
    events: Vec<DeviceEvent>,
}

impl Run {
    fn new() -> Self {
        let flags = HostFlags::new();
        flags.set_wants_raw(true);
        Self {
            flags,
            events: Vec::new(),
        }
    }

    fn connect<S: Read + Write>(
        &mut self,
        driver: &mut Mevo,
        stream: S,
        latched: DeviceCommand,
    ) -> Result<MevoSession<S>, Error> {
        let stop = Stop::new();
        self.connect_until(driver, stream, latched, &stop)
    }

    fn connect_until<S: Read + Write>(
        &mut self,
        driver: &mut Mevo,
        stream: S,
        latched: DeviceCommand,
        stop: &Stop,
    ) -> Result<MevoSession<S>, Error> {
        let mut host = Host(latched);
        let events = &mut self.events;
        let mut sink = |e| events.push(e);
        let mut ctx = ConnectCtx::new(stop, Events::new(&mut sink, &self.flags), &mut host);
        driver.connect_with(&mut ctx, stream)
    }

    fn poll<S: Read + Write>(&mut self, session: &mut MevoSession<S>) -> Result<(), Error> {
        let events = &mut self.events;
        let mut sink = |e| events.push(e);
        session.poll(
            Duration::from_millis(10),
            &mut Events::new(&mut sink, &self.flags),
        )
    }

    /// Poll until an event matching `pred` arrives (10 s at most).
    fn poll_until<S: Read + Write>(
        &mut self,
        session: &mut MevoSession<S>,
        pred: impl Fn(&DeviceEvent) -> bool,
    ) {
        let start = self.events.len();
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            self.poll(session).expect("poll");
            if self.events[start..].iter().any(&pred) {
                return;
            }
        }
        panic!("event not seen; got {:#?}", &self.events[start..]);
    }

    /// Poll until the device is armed and ready.
    fn armed<S: Read + Write>(&mut self, session: &mut MevoSession<S>) {
        self.poll_until(session, |e| telemetry(e, "ready") == Some("true"));
    }

    /// Push `shot` and poll until it has finished; return the shot's
    /// events.
    fn shot(
        &mut self,
        device: &FakeDevice,
        session: &mut MevoSession<impl Read + Write>,
        shot: &FakeShot,
    ) -> Vec<DeviceEvent> {
        let start = self.events.len();
        device.push_shot(shot);
        self.poll_until(session, |e| {
            matches!(e, DeviceEvent::Frp(FrpEvent::ShotFinished { .. }))
        });
        // The device is ready again once the shot is done.
        let finished = start
            + self.events[start..]
                .iter()
                .position(|e| matches!(e, DeviceEvent::Frp(FrpEvent::ShotFinished { .. })))
                .expect("finished");
        if !self.events[finished..]
            .iter()
            .any(|e| telemetry(e, "ready") == Some("true"))
        {
            self.poll_until(session, |e| telemetry(e, "ready") == Some("true"));
        }
        self.events[start..].to_vec()
    }

    fn close<S: Read + Write>(&mut self, session: MevoSession<S>) {
        let events = &mut self.events;
        let mut sink = |e| events.push(e);
        session
            .close(&mut Events::new(&mut sink, &self.flags))
            .expect("close");
    }
}

fn telemetry<'a>(e: &'a DeviceEvent, key: &str) -> Option<&'a str> {
    match e {
        DeviceEvent::Frp(FrpEvent::DeviceTelemetry {
            telemetry: Some(t), ..
        }) => t.get(key).map(String::as_str),
        _ => None,
    }
}

/// The shot lifecycle events and alerts, by kind, in order.
fn kinds(events: &[DeviceEvent]) -> Vec<&'static str> {
    events
        .iter()
        .filter_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::ShotTrigger { .. }) => Some("trigger"),
            DeviceEvent::Frp(FrpEvent::Launch { .. }) => Some("launch"),
            DeviceEvent::Frp(FrpEvent::FlightResult { .. }) => Some("flight"),
            DeviceEvent::Frp(FrpEvent::ClubPath { .. }) => Some("club"),
            DeviceEvent::Frp(FrpEvent::ShotFinished { .. }) => Some("finished"),
            DeviceEvent::Frp(FrpEvent::Alert { .. }) => Some("alert"),
            _ => None,
        })
        .collect()
}

fn launch_of(events: &[DeviceEvent]) -> Launch {
    events
        .iter()
        .find_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::Launch { launch, .. }) => Some(launch.clone()),
            _ => None,
        })
        .expect("a launch")
}

fn flight_of(events: &[DeviceEvent]) -> Option<Flight> {
    events.iter().find_map(|e| match e {
        DeviceEvent::Frp(FrpEvent::FlightResult { flight, .. }) => Some(flight.clone()),
        _ => None,
    })
}

fn club_of(events: &[DeviceEvent]) -> Option<ClubData> {
    events.iter().find_map(|e| match e {
        DeviceEvent::Frp(FrpEvent::ClubPath { club, .. }) => Some(club.clone()),
        _ => None,
    })
}

fn alerts(events: &[DeviceEvent]) -> Vec<String> {
    events
        .iter()
        .filter_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::Alert { severity, message }) => {
                assert_eq!(*severity, Severity::Warn);
                Some(message.clone())
            }
            _ => None,
        })
        .collect()
}

fn raw_frames(events: &[DeviceEvent]) -> Vec<&flightrelay::RawFrame> {
    events
        .iter()
        .filter_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::RawFrame(f)) => Some(f),
            _ => None,
        })
        .collect()
}

fn driver(config: Config) -> Mevo {
    Mevo::new(config).expect("valid config")
}

fn full() -> DeviceCommand {
    DeviceCommand::default().with_mode(DetectionMode::Full)
}

/// `payload` with the big-endian INT24 at `offset` set to `value`.
fn set_i24(payload: &mut [u8], offset: usize, value: i32) {
    let b = value.to_be_bytes();
    payload[offset..offset + 3].copy_from_slice(&b[1..]);
}

/// The Xi full swing's full result with no spin and no flight (carry,
/// flight time, apex, landing and spin all 0), launched at `elevation`
/// degrees.
fn d4_without_flight(elevation: f64) -> Vec<u8> {
    let mut d4 = fixtures::XI_SHOT_D4.to_vec();
    for offset in (25..70).step_by(3) {
        set_i24(&mut d4, offset, 0);
    }
    #[allow(clippy::cast_possible_truncation)]
    set_i24(&mut d4, 22, (elevation * 1000.0).round() as i32);
    d4
}

/// A shot whose only results are `d4` and the club result.
fn shot_with_d4(d4: Vec<u8>) -> FakeShot {
    FakeShot::new(
        40,
        BusAddr::Avr,
        [
            (TYPE_FLIGHT_RESULT, d4),
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
        ],
        fixtures::XI_SHOT_ED.to_vec(),
    )
}

/// The drawn Xi shot without club data: early and full result.
fn xi_draw() -> FakeShot {
    FakeShot::new(
        50,
        BusAddr::Avr,
        [
            (TYPE_FLIGHT_RESULT_V1, fixtures::XI_DRAW_E8.to_vec()),
            (TYPE_FLIGHT_RESULT, fixtures::XI_DRAW_D4.to_vec()),
        ],
        fixtures::XI_NO_READ_ED.to_vec(),
    )
}

#[test]
fn a_whole_xi_session_identifies_arms_and_reports_a_shot_once() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");

    let id = session.identity().clone();
    assert_eq!(id.device, "FlightScope Xi (192.168.2.1:5100)");
    assert_eq!(id.manufacturer.as_deref(), Some("FlightScope"));
    assert_eq!(id.model.as_deref(), Some("Xi"));
    assert_eq!(id.label.as_deref(), Some("Xi"));
    // The AVR's firmware, never the DSP device info.
    assert_eq!(id.firmware.as_deref(), Some("3.18 (Jan 1 2020)"));
    assert_eq!(session.model(), DeviceModel::Xi);
    assert_eq!(session.mode(), DetectionMode::Full);

    run.armed(&mut session);
    let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
    let key = match &shot
        .iter()
        .find(|e| matches!(e, DeviceEvent::Frp(FrpEvent::ShotTrigger { .. })))
    {
        Some(DeviceEvent::Frp(FrpEvent::ShotTrigger { key, timestamp })) => {
            assert!(timestamp.is_some_and(|t| t > 1_600_000_000_000));
            key.clone()
        }
        _ => unreachable!(),
    };
    assert_eq!(key.shot_number, 1);

    let launch = launch_of(&shot);
    assert_eq!(launch.backspin_rpm, Some(7127));
    assert_eq!(launch.sidespin_rpm, Some(652)); // curving right: FRP positive
    assert!(
        launch
            .launch_azimuth
            .is_some_and(|a| (a + 8.141).abs() < 1e-9)
    );
    assert!(launch.total_spin_rpm.is_some() && launch.spin_axis.is_some());
    let flight = flight_of(&shot).expect("a flight result");
    assert_eq!(flight.basis, Basis::Modeled);
    // Landing 10.756 m left of the origin: FRP negative.
    assert!(
        flight
            .carry_offline
            .is_some_and(|d| (d.as_meters() + 10.756).abs() < 1e-9)
    );
    assert!(flight.descent_angle.is_some());
    let club = club_of(&shot).expect("club data");
    assert!(club.path.is_some_and(|p| (p + 11.40).abs() < 0.01));

    // Ready drops at the trigger and comes back after the shot.
    let ready: Vec<&str> = shot.iter().filter_map(|e| telemetry(e, "ready")).collect();
    assert_eq!(ready, ["false", "true"]);

    run.close(session);
    assert_eq!(
        run.events.last().and_then(|e| telemetry(e, "ready")),
        Some("false")
    );
}

#[test]
fn the_identity_telemetry_comes_before_the_first_shot_and_has_no_placeholders() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let first = run
        .events
        .iter()
        .find_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::DeviceTelemetry {
                telemetry: Some(t), ..
            }) => Some(t.clone()),
            _ => None,
        })
        .expect("telemetry at connect");
    assert_eq!(first.get("ready").map(String::as_str), Some("false"));
    assert_eq!(first.get("tilt").map(String::as_str), Some("0.0"));
    assert!(first.contains_key("roll"));
    // The Xi's status layout has no decoded battery, temperature or power.
    for key in ["battery_pct", "temp_c", "external_power"] {
        assert!(
            !run.events.iter().any(|e| telemetry(e, key).is_some()),
            "{key}"
        );
    }
}

#[test]
fn club_data_waits_for_the_ball_and_the_launch_goes_out_at_the_full_result() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let club_first = FakeShot::new(
        60,
        BusAddr::Avr,
        [
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
            (TYPE_FLIGHT_RESULT_V1, fixtures::XI_SHOT_E8.to_vec()),
            (TYPE_FLIGHT_RESULT, fixtures::XI_SHOT_D4.to_vec()),
        ],
        fixtures::XI_SHOT_ED.to_vec(),
    );
    let shot = run.shot(&device, &mut session, &club_first);
    // The club result came first; it goes out after the ball.
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
}

#[test]
fn xi_acks_and_results_are_raw_frames_and_identity_frames_only_audit_lines() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    run.shot(&device, &mut session, &FakeShot::xi_full_swing());

    let frames = raw_frames(&run.events);
    for f in &frames {
        let label = f.label.as_deref().expect("label");
        let t = u8::from_str_radix(&label[..2], 16).expect("type");
        assert!(!carries_identity(t), "{label}");
        assert_eq!(f.seq, 0, "the host numbers frames");
        assert!(f.timestamp.is_some());
        assert_eq!(f.key, None);
    }
    let find = |label: &str, data: &[u8]| {
        frames
            .iter()
            .any(|f| f.label.as_deref() == Some(label) && f.data == data)
    };
    assert!(find("D4 AVR>APP", fixtures::XI_SHOT_D4));
    assert!(find("E8 AVR>APP", fixtures::XI_SHOT_E8));
    assert!(find("95 APP>AVR", &[0x02, 0x10, 0x54]), "D4 ack");
    assert!(find("95 APP>AVR", &[0x02, 0x10, 0x68]), "E8 ack");
    let ack = frames
        .iter()
        .find(|f| f.label.as_deref() == Some("95 APP>AVR"))
        .expect("an ack");
    assert_eq!(ack.dir, RawDir::Tx);
    // Shot texts go out; the device's free text and debug output, and every
    // frame of the handshake (its config queries and the DSP power-mode
    // write here), only to the audit log.
    assert!(
        frames
            .iter()
            .any(|f| f.label.as_deref() == Some("E5 AVR>APP"))
    );
    for f in &frames {
        let label = f.label.as_deref().expect("label");
        assert!(
            !label.starts_with("E3 ") && !label.starts_with("F0 "),
            "{label}"
        );
        assert!(
            !label.starts_with("21 ") && !label.starts_with("A0 "),
            "{label}"
        );
        assert!(label != "BF APP>DSP", "{label}");
    }

    // Status polls and device info carry the serial: audit lines with hex.
    let audit: Vec<&str> = run
        .events
        .iter()
        .filter_map(|e| match e {
            DeviceEvent::Log(Level::Trace, line) => Some(line.as_str()),
            _ => None,
        })
        .collect();
    assert!(audit.iter().any(|l| l.starts_with("rx E7 AVR>APP ")));
    assert!(audit.iter().any(|l| l.starts_with("tx AA APP>DSP ")));
    assert!(audit.iter().any(|l| l.starts_with("rx AA DSP>APP 1c00")));
    assert!(audit.iter().any(|l| l.starts_with("tx 21 APP>AVR ")));
    assert!(audit.iter().any(|l| l.starts_with("rx E3 AVR>APP ")));

    // Raw frames are built only while someone wants them.
    let mut quiet = Run::new();
    quiet.flags.set_wants_raw(false);
    let device = FakeDevice::xi();
    let mut session = quiet
        .connect(&mut driver(Config::default()), device.clone(), full())
        .expect("connect");
    quiet.armed(&mut session);
    quiet.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert!(raw_frames(&quiet.events).is_empty());
}

#[test]
fn raw_frames_of_bulk_shot_data_are_bulk() {
    let info = Mevo::info(&Config::default());
    assert_eq!(info.source, Source::Measured);
    assert_eq!(info.kind, "ironsight");
    assert!(!info.exclusive);
    let raw = info.raw.expect("a raw profile");
    assert_eq!(raw.transport, "flightscope-tcp");
    assert!(!raw.text);
    for t in 0..=255u8 {
        let frame = flightrelay::RawFrame {
            label: Some(format!("{t:02X} AVR>APP")),
            ..flightrelay::RawFrame::new(RawDir::Rx, 0, vec![])
        };
        assert_eq!((raw.is_bulk)(&frame), is_bulk_shot_data(t), "{t:02X}");
    }
    assert!(!(raw.is_bulk)(&flightrelay::RawFrame::new(
        RawDir::Rx,
        0,
        vec![]
    )));
}

/// The launch-only full result here is synthesized: the recorded Xi full
/// swing with every flight and spin value zeroed and the launch raised to
/// 30°.
#[test]
fn a_launch_only_chip_sends_no_spin_and_no_flight_result() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let shot = run.shot(
        &device,
        &mut session,
        &shot_with_d4(d4_without_flight(30.0)),
    );
    assert_eq!(kinds(&shot), ["trigger", "launch", "club", "finished"]);
    let launch = launch_of(&shot);
    assert!(launch.launch_speed.is_some());
    assert_eq!(launch.launch_elevation, Some(30.0));
    assert!(launch.launch_azimuth.is_some());
    assert_eq!(
        (
            launch.backspin_rpm,
            launch.sidespin_rpm,
            launch.total_spin_rpm,
            launch.spin_axis
        ),
        (None, None, None, None)
    );
    assert_eq!(flight_of(&shot), None);
}

#[test]
fn a_low_launch_without_flight_is_a_measured_ground_ball() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let shot = run.shot(&device, &mut session, &shot_with_d4(d4_without_flight(2.0)));
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
    let launch = launch_of(&shot);
    assert_eq!(
        (launch.backspin_rpm, launch.sidespin_rpm),
        (Some(0), Some(0))
    );
    assert_eq!(launch.spin_axis, None);
    let flight = flight_of(&shot).expect("a flight result");
    assert_eq!(
        flight.max_height.map(flightrelay::Distance::as_meters),
        Some(0.0)
    );
    assert_eq!(
        flight.carry_distance.map(flightrelay::Distance::as_meters),
        Some(0.0)
    );
    assert_eq!(flight.carry_offline, None);
}

#[test]
fn the_xi_spin_source_applies_live_and_an_e8_total_without_an_axis_goes_out_alone() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);

    let d4 = launch_of(&run.shot(&device, &mut session, &xi_draw()));
    assert_eq!(
        (d4.backspin_rpm, d4.sidespin_rpm),
        (Some(2756), Some(-1464))
    );

    let e8 = Config {
        spin_source: SpinSource::E8,
        ..Config::default()
    };
    assert!(!Config::restart_needed(mevo.config(), &e8));
    mevo.apply_config(&e8);
    assert_eq!(mevo.config().spin_source, SpinSource::E8);

    // No club data: the early-result total alone, with no axis.
    let total = launch_of(&run.shot(&device, &mut session, &xi_draw()));
    assert_eq!(
        (
            total.backspin_rpm,
            total.sidespin_rpm,
            total.total_spin_rpm,
            total.spin_axis
        ),
        (None, None, Some(6897), None)
    );
    // Club data: the total split by the full result's axis.
    let split = launch_of(&run.shot(&device, &mut session, &FakeShot::xi_full_swing()));
    assert_eq!(
        (split.backspin_rpm, split.sidespin_rpm),
        (Some(7127), Some(652))
    );
    // The measured early-result total and the full result's axis, as
    // measured, not recomputed from the rounded parts.
    assert_eq!(split.total_spin_rpm, Some(7157));
    let d4 = FlightResult::decode(fixtures::XI_SHOT_D4).expect("D4");
    let axis = -f64::from(d4.sidespin_rpm)
        .atan2(f64::from(d4.backspin_rpm))
        .to_degrees();
    assert_eq!(split.spin_axis, Some(axis));

    // Auto: the E8 rule in full swing only.
    mevo.apply_config(&Config {
        spin_source: SpinSource::Auto,
        ..Config::default()
    });
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Chipping))
        .expect("command");
    run.armed(&mut session);
    let chip = launch_of(&run.shot(&device, &mut session, &xi_draw()));
    assert_eq!(chip.backspin_rpm, Some(2756));
}

#[test]
fn the_session_starts_in_the_latched_mode_and_reconfigures_only_when_the_wire_changes() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let chipping = DeviceCommand::default().with_mode(DetectionMode::Chipping);
    let mut session = run
        .connect(&mut mevo, device.clone(), chipping)
        .expect("connect");
    assert_eq!(session.mode(), DetectionMode::Chipping);
    run.armed(&mut session);

    // The Xi's club type parameter (0x16): 2 = pitching, 1 = full swing.
    let club_type = |device: &FakeDevice| {
        device
            .sent()
            .iter()
            .rev()
            .find(|f| {
                f.dest == BusAddr::Avr && f.type_id == 0xBF && f.payload.get(3) == Some(&0x16)
            })
            .map(|f| f.payload[6])
    };
    let arms = |device: &FakeDevice| {
        device
            .sent()
            .iter()
            .filter(|f| f.type_id == 0xB0 && f.payload == [0x01, 0x01])
            .count()
    };
    assert_eq!(club_type(&device), Some(2));
    let armed_once = arms(&device);

    // Chipping to putting: the same session on the Xi, nothing is sent.
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Putting))
        .expect("command");
    for _ in 0..20 {
        run.poll(&mut session).expect("poll");
    }
    assert_eq!(session.mode(), DetectionMode::Putting);
    assert_eq!(arms(&device), armed_once);

    // Putting to full swing: reconfigured and re-armed.
    let before = run.events.len();
    session.command(&full()).expect("command");
    run.poll_until(&mut session, |e| telemetry(e, "ready") == Some("true"));
    assert_eq!(session.mode(), DetectionMode::Full);
    assert_eq!(club_type(&device), Some(1));
    assert_eq!(arms(&device), armed_once + 1);
    let ready: Vec<&str> = run.events[before..]
        .iter()
        .filter_map(|e| telemetry(e, "ready"))
        .collect();
    assert_eq!(ready, ["false", "true"]);
    assert!(run.events[before..].iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Info, l) if l.starts_with("mode change: putting -> full")
    )));

    // A hand or club is not a device setting: nothing is configured or
    // armed after it.
    let sent = device.sent().len();
    session
        .command(&DeviceCommand::default().with_club(flightrelay::Club::Driver))
        .expect("command");
    for _ in 0..5 {
        run.poll(&mut session).expect("poll");
    }
    let after = device.sent();
    assert!(
        !after[sent..]
            .iter()
            .any(|f| matches!(f.type_id, 0xBF | 0xA5 | 0xB0)),
        "{:?}",
        &after[sent..]
    );
}

#[test]
fn a_wire_error_raises_an_alert_and_the_session_continues() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let before = run.events.len();
    device.push(BusAddr::Avr, TYPE_FLIGHT_RESULT, vec![0x9E, 0x00]);
    run.poll_until(&mut session, |e| {
        matches!(e, DeviceEvent::Frp(FrpEvent::Alert { .. }))
    });
    let alert = alerts(&run.events[before..]);
    assert_eq!(alert.len(), 1);
    assert!(
        alert[0].starts_with("Could not process message"),
        "{alert:?}"
    );
    let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
}

#[test]
fn a_shot_the_device_did_not_read_alerts_and_sends_no_ball() {
    let mut run = Run::new();
    let mut mevo = driver(Config {
        use_estimated: Some(true),
        ..Config::default()
    });
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let shot = run.shot(&device, &mut session, &FakeShot::xi_no_read());
    assert_eq!(kinds(&shot), ["trigger", "alert", "finished"]);
    assert_eq!(
        alerts(&shot),
        ["Shot #1: Xi didn't track the ball (no read)"]
    );
}

#[test]
fn an_early_result_makes_the_shot_only_with_use_estimated() {
    for (use_estimated, want) in [
        (None, &["trigger", "club", "finished"][..]),
        (
            Some(true),
            &["trigger", "launch", "flight", "club", "finished"][..],
        ),
    ] {
        let mut run = Run::new();
        let mut mevo = driver(Config {
            use_estimated,
            ..Config::default()
        });
        let device = FakeDevice::xi();
        let mut session = run
            .connect(&mut mevo, device.clone(), full())
            .expect("connect");
        run.armed(&mut session);
        let shot = run.shot(&device, &mut session, &FakeShot::xi_chip());
        assert_eq!(kinds(&shot), want, "{use_estimated:?}");
        if use_estimated.is_some() {
            let launch = launch_of(&shot);
            assert_eq!(launch.backspin_rpm, Some(4037));
            assert_eq!(launch.sidespin_rpm, None);
            assert_eq!(flight_of(&shot).map(|f| f.basis), Some(Basis::Modeled));
        }
    }
}

#[test]
fn shot_numbers_keep_counting_across_sessions() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    for n in 1..=2 {
        let device = FakeDevice::xi();
        let mut session = run
            .connect(&mut mevo, device.clone(), full())
            .expect("connect");
        run.armed(&mut session);
        let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
        let number = shot.iter().find_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::ShotTrigger { key, .. }) => Some(key.shot_number),
            _ => None,
        });
        assert_eq!(number, Some(n));
        run.close(session);
    }
}

/// A stream over a [`FakeDevice`] that can go silent (reads find nothing,
/// writes vanish) or closed (reads return 0 bytes).
struct Line {
    device: FakeDevice,
    silent: Rc<Cell<bool>>,
    closed: Rc<Cell<bool>>,
}

impl Line {
    fn new(device: &FakeDevice) -> (Self, Rc<Cell<bool>>, Rc<Cell<bool>>) {
        let silent = Rc::new(Cell::new(false));
        let closed = Rc::new(Cell::new(false));
        let line = Self {
            device: device.clone(),
            silent: Rc::clone(&silent),
            closed: Rc::clone(&closed),
        };
        (line, silent, closed)
    }
}

impl Read for Line {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.closed.get() {
            return Ok(0);
        }
        if self.silent.get() {
            return Err(io::ErrorKind::WouldBlock.into());
        }
        self.device.read(buf)
    }
}

impl Write for Line {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.silent.get() {
            return Ok(buf.len());
        }
        self.device.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn a_closed_connection_ends_the_session() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let (line, _, closed) = Line::new(&device);
    let mut session = run.connect(&mut mevo, line, full()).expect("connect");
    run.armed(&mut session);
    closed.set(true);
    let err = run.poll(&mut session).expect_err("closed");
    assert!(
        matches!(err, Error::Connection(ConnError::Disconnected)),
        "{err:?}"
    );
}

#[test]
fn a_silent_device_ends_the_session_after_ten_seconds() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let (line, silent, _) = Line::new(&device);
    let mut session = run.connect(&mut mevo, line, full()).expect("connect");
    run.armed(&mut session);
    silent.set(true);
    let start = Instant::now();
    let err = loop {
        if let Err(e) = run.poll(&mut session) {
            break e;
        }
        assert!(start.elapsed() < Duration::from_secs(15), "never ended");
    };
    assert!(matches!(err, Error::Stale), "{err:?}");
    let took = start.elapsed();
    assert!(took >= Duration::from_secs(9), "{took:?}");
}

#[test]
fn a_stop_ends_the_handshake() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let (line, silent, _) = Line::new(&device);
    silent.set(true);
    let stop = Stop::new();
    let stopper = stop.clone();
    let thread = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(100));
        stopper.stop();
    });
    let start = Instant::now();
    let err = run
        .connect_until(&mut mevo, line, full(), &stop)
        .expect_err("stopped");
    assert!(matches!(err, Error::Stopped), "{err:?}");
    assert!(start.elapsed() < Mevo::STOP_WAIT);
    thread.join().expect("thread");
}

#[test]
fn a_stop_before_the_tcp_connect_returns_at_once() {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr");
    let mut mevo = driver(Config {
        address: Some(addr.to_string()),
        ..Config::default()
    });
    let stop = Stop::new();
    stop.stop();
    let flags = HostFlags::new();
    let mut sink = |_| {};
    let mut host = Host(full());
    let mut ctx = ConnectCtx::new(&stop, Events::new(&mut sink, &flags), &mut host);
    let err = mevo.connect(&mut ctx).expect_err("stopped");
    assert!(matches!(err, Error::Stopped), "{err:?}");
}

#[test]
fn a_stop_during_the_tcp_connect_returns_at_once() {
    // 192.0.2.1 (TEST-NET-1) is never routed to a host: the connect waits
    // for its 5 s timeout unless the stop ends it.
    let mut mevo = driver(Config {
        address: Some("192.0.2.1:5100".into()),
        ..Config::default()
    });
    let stop = Stop::new();
    let stopper = stop.clone();
    let thread = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(300));
        stopper.stop();
    });
    let flags = HostFlags::new();
    let mut sink = |_| {};
    let mut host = Host(full());
    let mut ctx = ConnectCtx::new(&stop, Events::new(&mut sink, &flags), &mut host);
    let start = Instant::now();
    let result = mevo.connect(&mut ctx);
    let took = start.elapsed();
    thread.join().expect("thread");
    match result {
        Err(Error::Stopped) => {
            assert!(took >= Duration::from_millis(250), "{took:?}");
            assert!(took < Duration::from_secs(1), "{took:?}");
        }
        // A machine without a route refuses the connect at once; there is
        // nothing to stop then.
        Err(Error::Connect { .. }) if took < Duration::from_millis(250) => {}
        other => panic!("{other:?} after {took:?}"),
    }
}

#[test]
fn a_refused_connect_is_an_error_naming_the_address() {
    let addr = {
        let l = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        l.local_addr().expect("addr")
    };
    let mut mevo = driver(Config {
        address: Some(addr.to_string()),
        ..Config::default()
    });
    let stop = Stop::new();
    let flags = HostFlags::new();
    let mut sink = |_| {};
    let mut host = Host(full());
    let mut ctx = ConnectCtx::new(&stop, Events::new(&mut sink, &flags), &mut host);
    let err = mevo.connect(&mut ctx).expect_err("refused");
    assert!(matches!(err, Error::Connect { .. }), "{err:?}");
    assert!(err.to_string().contains(&addr.to_string()));
}

#[test]
fn new_refuses_an_invalid_config() {
    let err = Mevo::new(Config {
        address: Some("192.168.2.1".into()),
        ..Config::default()
    })
    .expect_err("no port");
    assert_eq!(err.field.as_deref(), Some("address"));
    let err = Mevo::new(Config {
        track_pct: Some(f64::NAN),
        ..Config::default()
    })
    .expect_err("not a number");
    assert_eq!(err.field.as_deref(), Some("track_pct"));
}

/// What `--set key=value` does to the config: the value read as TOML (or
/// taken as text) over the defaults, then read and validated.
#[test]
fn optional_keys_are_settable_like_the_command_line_sets_them() {
    let mut map = Config::default().to_map();
    map.insert("address".into(), "10.0.0.9:5100".into());
    map.insert("range".into(), "10ft".into());
    map.insert("use_estimated".into(), true.into());
    let config = Config::from_map(map).expect("valid");
    assert_eq!(config.address.as_deref(), Some("10.0.0.9:5100"));
    assert_eq!(config.range, Some(flightrelay::Distance::Feet(10.0)));
    assert!(config.use_estimated());
    // Every key, the optional ones included, is in the listed config.
    let listed = serde_json::to_value(Config::example()).expect("json");
    for key in ["address", "range", "use_estimated", "environment"] {
        assert!(listed.get(key).is_some(), "{key}");
    }
}

#[cfg(feature = "frp")]
#[test]
fn the_command_line_refuses_bad_settings_before_connecting() {
    use std::process::ExitCode;
    let run = |args: &[&str]| {
        flightrelay::run_args::<Mevo>(
            args.iter().map(|a| (*a).to_owned()),
            flightrelay::RunOptions::default(),
        )
    };
    assert_eq!(run(&["--set", "track_pct=150"]), ExitCode::from(2));
    assert_eq!(run(&["--set", "address=mevo"]), ExitCode::from(2));
    assert_eq!(run(&["--set", "no_such_key=1"]), ExitCode::from(2));
    assert_eq!(run(&["mevo-addr"]), ExitCode::from(2));
    assert_eq!(run(&["--help"]), ExitCode::SUCCESS);
}

#[test]
fn mevo_plus_status_gives_battery_temperature_and_power_and_others_give_none() {
    let mut p = vec![0u8; 129];
    p[0] = 0x80;
    p[40..42].copy_from_slice(&2345i16.to_be_bytes()); // 23.45 °C
    p[61] = 87; // power level high byte
    p[63] = 1; // external power
    let v80 = DspStatus::decode(&p).expect("Mevo+ status");
    let got: HashMap<&str, String> = dsp_telemetry(&v80).into_iter().collect();
    assert_eq!(got.get("temp_c").map(String::as_str), Some("23.4"));
    assert_eq!(got.get("external_power").map(String::as_str), Some("true"));
    assert_eq!(
        got.get("battery_pct"),
        Some(&v80.battery_percent().to_string())
    );
    let mut p = vec![0u8; 71];
    p[0] = 0x46;
    let v46 = DspStatus::decode(&p).expect("Gen2 status");
    assert!(dsp_telemetry(&v46).is_empty());

    let mut avr = crate::testing::avr_status([0, 0]);
    let mut angles = Vec::new();
    crate::codec::write_float40(&mut angles, 1.5); // tilt
    crate::codec::write_float40(&mut angles, 2.25); // roll
    avr[15..25].copy_from_slice(&angles);
    let avr = AvrStatus::decode(&avr).expect("AVR status");
    let got: HashMap<&str, String> = avr_telemetry(&avr).into_iter().collect();
    assert_eq!(got["tilt"], format!("{:.1}", avr.tilt));
    assert_eq!(got["roll"], format!("{:.1}", -avr.roll));
    assert_eq!(got["roll"], "-2.2");
}

// -- Decode errors carry no payload bytes out ---------------------------------

/// Every way a test checks for `payload` in a line: hex in either case,
/// `Debug`'s decimal list, and the ASCII it spells.
fn mentions(line: &str, payload: &[u8], ascii: &str) -> bool {
    let upper = hex(payload);
    let decimal = payload
        .iter()
        .take(6)
        .map(u8::to_string)
        .collect::<Vec<_>>()
        .join(", ");
    line.contains(&upper)
        || line.contains(&upper.to_lowercase())
        || line.contains(&decimal)
        || line.contains(ascii)
}

#[test]
fn a_short_identity_frame_raises_an_alert_without_its_bytes() {
    let mut run = Run::new();
    run.flags.set_min_level(Level::Trace);
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let before = run.events.len();
    // A 20-byte AVR status (it needs 25) with a serial number in it.
    let mut status = vec![0x01];
    status.extend_from_slice(b"SERIAL-1234");
    status.resize(20, 0x55);
    device.push(BusAddr::Avr, 0xAA, status.clone());
    run.poll_until(&mut session, |e| {
        matches!(e, DeviceEvent::Frp(FrpEvent::Alert { .. }))
    });
    for _ in 0..3 {
        run.poll(&mut session).expect("poll");
    }
    let events = &run.events[before..];
    let alert = alerts(events);
    assert_eq!(
        alert,
        [
            "Could not process message: wire error: payload too short for AvrStatus: need 25 bytes, got 20"
        ]
    );
    for e in events {
        match e {
            DeviceEvent::Log(level, line) if *level >= Level::Debug => {
                assert!(!mentions(line, &status, "SERIAL"), "{level:?} {line}");
            }
            DeviceEvent::Frp(FrpEvent::RawFrame(f)) => assert_ne!(f.data, status),
            _ => {}
        }
    }
    // The bytes are in the audit log, as hex.
    assert!(events.iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Trace, l) if l.starts_with("rx undecoded ") && mentions(l, &status, "-")
    )));

    // An error that ends a session names no bytes either.
    let err = Error::Connection(ConnError::Wire(
        crate::error::WireError::payload_too_short("AvrStatus", 25, 20).with_raw(&status),
    ));
    assert!(!mentions(&err.to_string(), &status, "SERIAL"), "{err}");
}

// -- Staleness ---------------------------------------------------------------

#[test]
fn a_zero_timeout_poll_on_a_silent_device_ends_the_session_after_ten_seconds() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let (line, silent, _) = Line::new(&device);
    let mut session = run.connect(&mut mevo, line, full()).expect("connect");
    run.armed(&mut session);
    silent.set(true);
    let start = Instant::now();
    let err = loop {
        let events = &mut run.events;
        let mut sink = |e| events.push(e);
        if let Err(e) = session.poll(Duration::ZERO, &mut Events::new(&mut sink, &run.flags)) {
            break e;
        }
        assert!(start.elapsed() < Duration::from_secs(15), "never ended");
        std::thread::sleep(Duration::from_millis(5));
    };
    assert!(matches!(err, Error::Stale), "{err:?}");
    let took = start.elapsed();
    assert!(took >= Duration::from_secs(9), "{took:?}");
}

/// A stream that answers every read with one frame that does not decode (a
/// 3-byte AVR status) and swallows writes; counts the reads.
struct Garbage {
    frame: Vec<u8>,
    reads: Rc<Cell<usize>>,
}

impl Read for Garbage {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.reads.set(self.reads.get() + 1);
        let n = self.frame.len().min(buf.len());
        buf[..n].copy_from_slice(&self.frame[..n]);
        Ok(n)
    }
}

impl Write for Garbage {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn a_handshake_that_gets_only_undecodable_frames_goes_stale_without_spinning() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let reads = Rc::new(Cell::new(0));
    let frame = crate::frame::RawFrame {
        dest: BusAddr::App,
        src: BusAddr::Avr,
        type_id: 0xAA,
        payload: vec![0x18, 0x00, 0x00],
    }
    .encode();
    let garbage = Garbage {
        frame,
        reads: Rc::clone(&reads),
    };
    let start = Instant::now();
    let err = run.connect(&mut mevo, garbage, full()).expect_err("stale");
    let took = start.elapsed();
    assert!(matches!(err, Error::Stale), "{err:?}");
    assert!(took >= Duration::from_secs(9), "{took:?}");
    assert!(took < Duration::from_secs(15), "{took:?}");
    // One bad frame per idle sleep: about 500 in 10 s, not a busy loop.
    assert!(reads.get() <= 700, "{} reads", reads.get());
    assert!(alerts(&run.events).len() <= 700);
}

// -- A trigger while a shot is open ------------------------------------------

#[test]
fn a_second_trigger_finishes_the_open_shot_before_the_next_one_starts() {
    let mut run = Run::new();
    run.flags.set_min_level(Level::Debug);
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    device.push(
        BusAddr::Avr,
        crate::protocol::TYPE_SHOT_TEXT,
        crate::testing::shot_text("BALL TRIGGER: 20 ms"),
    );
    let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert_eq!(
        kinds(&shot),
        [
            "trigger", "finished", "trigger", "launch", "flight", "club", "finished"
        ]
    );
    let keys: Vec<(&str, u32)> = shot
        .iter()
        .filter_map(|e| match e {
            DeviceEvent::Frp(FrpEvent::ShotTrigger { key, .. }) => {
                Some(("trigger", key.shot_number))
            }
            DeviceEvent::Frp(FrpEvent::ShotFinished { key }) => Some(("finished", key.shot_number)),
            DeviceEvent::Frp(FrpEvent::Launch { key, .. }) => Some(("launch", key.shot_number)),
            _ => None,
        })
        .collect();
    assert_eq!(
        keys,
        [
            ("trigger", 1),
            ("finished", 1),
            ("trigger", 2),
            ("launch", 2),
            ("finished", 2)
        ]
    );
    assert!(shot.iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Debug, l) if l.starts_with("shot #1 ended by a new trigger")
    )));
}

// -- Xi: deferred mode changes, one D4 per shot, launch-only with E8 spin ----

/// The Xi's latest club type parameter (0x16): 2 = pitching, 1 = full swing.
fn xi_club_type(device: &FakeDevice) -> Option<u8> {
    device
        .sent()
        .iter()
        .rev()
        .find(|f| f.dest == BusAddr::Avr && f.type_id == 0xBF && f.payload.get(3) == Some(&0x16))
        .map(|f| f.payload[6])
}

/// How many arms (B0 `[01 01]`) the client sent.
fn arms(device: &FakeDevice) -> usize {
    device
        .sent()
        .iter()
        .filter(|f| f.type_id == 0xB0 && f.payload == [0x01, 0x01])
        .count()
}

/// `shot` split into its trigger text and the rest.
fn split_trigger(shot: &FakeShot) -> (crate::frame::RawFrame, FakeShot) {
    let mut rest = shot.clone();
    let trigger = rest.frames.remove(0);
    (trigger, rest)
}

#[test]
fn a_mode_change_during_a_shot_is_applied_when_the_shot_completes() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let armed = arms(&device);

    let (trigger, rest) = split_trigger(&FakeShot::xi_full_swing());
    device.push(trigger.src, trigger.type_id, trigger.payload);
    run.poll_until(&mut session, |e| {
        matches!(e, DeviceEvent::Frp(FrpEvent::ShotTrigger { .. }))
    });
    let before = run.events.len();
    let sent = device.sent().len();
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Chipping))
        .expect("command");
    for _ in 0..5 {
        run.poll(&mut session).expect("poll");
    }
    // Deferred: nothing is sent while the shot is open.
    assert_eq!(device.sent().len(), sent);
    assert_eq!(session.mode(), DetectionMode::Full);
    assert!(run.events[before..].iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Info, l) if l.starts_with("deferred mode change to chipping")
    )));

    let shot = run.shot(&device, &mut session, &rest);
    assert_eq!(kinds(&shot), ["launch", "flight", "club", "finished"]);
    run.poll_until(&mut session, |_| arms(&device) == armed + 2);
    run.armed(&mut session);
    assert_eq!(session.mode(), DetectionMode::Chipping);
    assert_eq!(xi_club_type(&device), Some(2));
    // Configured after the shot's own re-arm.
    let sent = device.sent();
    let result_request = sent.iter().position(|f| f.type_id == 0x6D).expect("0x6D");
    let reconfigure = sent
        .iter()
        .rposition(|f| f.type_id == 0xBF && f.payload.get(3) == Some(&0x16))
        .expect("club type");
    assert!(reconfigure > result_request);
}

#[test]
fn a_mode_change_before_the_first_arm_is_applied_once_armed() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    // The connect queued the configuration and the arm; nothing is armed yet.
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Chipping))
        .expect("command");
    run.poll_until(&mut session, |_| arms(&device) == 2);
    run.armed(&mut session);
    assert_eq!(session.mode(), DetectionMode::Chipping);
    assert_eq!(xi_club_type(&device), Some(2));
    assert!(run.events.iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Info, l) if l.starts_with("applying deferred mode change to chipping")
    )));
}

#[test]
fn a_command_back_to_the_current_mode_drops_the_deferred_change() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let (trigger, rest) = split_trigger(&FakeShot::xi_full_swing());
    device.push(trigger.src, trigger.type_id, trigger.payload);
    run.poll_until(&mut session, |e| {
        matches!(e, DeviceEvent::Frp(FrpEvent::ShotTrigger { .. }))
    });
    let sent = device.sent().len();
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Chipping))
        .expect("command");
    session.command(&full()).expect("command");
    run.shot(&device, &mut session, &rest);
    for _ in 0..20 {
        run.poll(&mut session).expect("poll");
    }
    assert_eq!(session.mode(), DetectionMode::Full);
    // No configuration after the commands: only the shot's own traffic.
    assert!(
        !device.sent()[sent..]
            .iter()
            .any(|f| matches!(f.type_id, 0xBF | 0xA5)),
        "{:?}",
        &device.sent()[sent..]
    );
    assert_eq!(xi_club_type(&device), Some(1));
}

#[test]
fn a_second_full_result_in_the_same_shot_is_ignored() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let second = d4_without_flight(12.0);
    let shot = FakeShot::new(
        70,
        BusAddr::Avr,
        [
            (TYPE_FLIGHT_RESULT, fixtures::XI_SHOT_D4.to_vec()),
            (TYPE_FLIGHT_RESULT, second),
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
        ],
        fixtures::XI_SHOT_ED.to_vec(),
    );
    let events = run.shot(&device, &mut session, &shot);
    assert_eq!(
        kinds(&events),
        ["trigger", "launch", "flight", "club", "finished"]
    );
    assert_eq!(launch_of(&events).backspin_rpm, Some(7127));
}

/// The launch-only full result is synthesized, as in
/// [`a_launch_only_chip_sends_no_spin_and_no_flight_result`].
#[test]
fn a_launch_only_chip_with_the_e8_spin_source_sends_the_e8_total_and_no_flight() {
    let mut run = Run::new();
    let mut mevo = driver(Config {
        spin_source: SpinSource::E8,
        ..Config::default()
    });
    let device = FakeDevice::xi();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let shot = FakeShot::new(
        80,
        BusAddr::Avr,
        [
            (TYPE_FLIGHT_RESULT_V1, fixtures::XI_SHOT_E8.to_vec()),
            (TYPE_FLIGHT_RESULT, d4_without_flight(30.0)),
            (TYPE_CLUB_RESULT, fixtures::XI_SHOT_ED.to_vec()),
        ],
        fixtures::XI_SHOT_ED.to_vec(),
    );
    let events = run.shot(&device, &mut session, &shot);
    assert_eq!(kinds(&events), ["trigger", "launch", "club", "finished"]);
    let launch = launch_of(&events);
    assert_eq!(launch.launch_elevation, Some(30.0));
    assert_eq!(
        (
            launch.backspin_rpm,
            launch.sidespin_rpm,
            launch.total_spin_rpm,
            launch.spin_axis
        ),
        (None, None, Some(7157), None)
    );
}

// -- Mevo+ and Mevo Gen2 sessions ----------------------------------------------

/// Index of the first frame the client sent matching `pred`.
fn sent_index(
    device: &FakeDevice,
    pred: impl Fn(&crate::frame::RawFrame) -> bool,
) -> Option<usize> {
    device.sent().iter().position(pred)
}

fn ever(events: &[DeviceEvent], key: &str) -> bool {
    events.iter().any(|e| telemetry(e, key).is_some())
}

#[test]
fn a_whole_mevo_plus_session_identifies_configures_arms_and_reports_a_shot() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::mevo_plus();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");

    let id = session.identity().clone();
    assert_eq!(id.device, "FS M2-TEST01");
    assert_eq!(id.manufacturer.as_deref(), Some("FlightScope"));
    assert_eq!(id.model.as_deref(), Some("Mevo+"));
    assert_eq!(id.firmware.as_deref(), Some("1.23 (Feb 3 2024)"));
    assert_eq!(id.label, None);
    assert_eq!(session.model(), DeviceModel::Mevo);

    run.armed(&mut session);
    // AVR configuration (full settings, ending with the radar calibration),
    // then the camera's warmup config, then the arm.
    let handshake_end = sent_index(&device, |f| f.dest == BusAddr::Pi && f.type_id == 0xDE)
        .expect("Pi network query");
    let params =
        sent_index(&device, |f| f.dest == BusAddr::Avr && f.type_id == 0xBF).expect("AVR params");
    let radar_cal =
        sent_index(&device, |f| f.dest == BusAddr::Avr && f.type_id == 0xA4).expect("radar cal");
    let warmup = CamConfig::warmup_preset().encode();
    let cam = sent_index(&device, |f| {
        f.dest == BusAddr::Pi && f.type_id == 0x82 && f.payload == warmup
    })
    .expect("warmup cam config");
    let arm = sent_index(&device, |f| f.type_id == 0xB0 && f.payload == [0x01, 0x01]).expect("arm");
    assert!(
        handshake_end < params && params < radar_cal,
        "{params} {radar_cal}"
    );
    assert!(radar_cal < cam && cam < arm, "{radar_cal} {cam} {arm}");

    // The Mevo+ DSP status gives battery, temperature and power at once.
    let first = run
        .events
        .iter()
        .find(|e| telemetry(e, "ready").is_some())
        .expect("telemetry");
    assert_eq!(telemetry(first, "battery_pct"), Some("87"));
    assert_eq!(telemetry(first, "temp_c"), Some("31.0"));
    assert_eq!(telemetry(first, "external_power"), Some("false"));

    // The Xi's result frames: the Mevo sends the same layout from the AVR.
    let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
    assert_eq!(launch_of(&shot).backspin_rpm, Some(7127));
    // A Mevo's results are not acked.
    assert!(!device.sent().iter().any(|f| f.type_id == 0x95));
    // The handshake (Pi network config included) left only as audit lines.
    assert!(raw_frames(&run.events).iter().all(|f| {
        let label = f.label.as_deref().unwrap_or_default();
        !label.starts_with("DE ") && !label.starts_with("E7 ") && !label.starts_with("E3 ")
    }));
    run.close(session);
}

#[test]
fn a_mevo_without_a_wifi_name_is_named_by_model_and_address() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::mevo_plus().with_ssid("");
    let session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    assert_eq!(
        session.identity().device,
        "FlightScope Mevo (192.168.2.1:5100)"
    );
}

#[test]
fn mevo_plus_keepalives_send_battery_temperature_and_power_when_they_change() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::mevo_plus();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    session
        .client
        .set_keepalive_interval(Duration::from_millis(50));
    device.set_dsp_status(crate::testing::mevo_plus_dsp_status(55, 35.5, true));
    let before = run.events.len();
    run.poll_until(&mut session, |e| telemetry(e, "battery_pct") == Some("55"));
    let changed = run.events[before..]
        .iter()
        .find(|e| telemetry(e, "battery_pct").is_some())
        .expect("telemetry");
    assert_eq!(telemetry(changed, "temp_c"), Some("35.5"));
    assert_eq!(telemetry(changed, "external_power"), Some("true"));
    // Unchanged values are not sent again.
    assert_eq!(telemetry(changed, "tilt"), None);
}

#[test]
fn a_mevo_gen2_session_sends_no_battery_temperature_or_power() {
    let mut run = Run::new();
    let mut mevo = driver(Config::default());
    let device = FakeDevice::mevo_gen2();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    assert_eq!(session.identity().device, "FS MG2-TEST01");
    assert_eq!(session.identity().model.as_deref(), Some("Mevo Gen2"));
    assert_eq!(
        session.identity().firmware.as_deref(),
        Some("4.56 (Feb 3 2024)")
    );
    run.armed(&mut session);
    session
        .client
        .set_keepalive_interval(Duration::from_millis(50));
    let polls = |device: &FakeDevice| {
        device
            .sent()
            .iter()
            .filter(|f| f.dest == BusAddr::Dsp && f.type_id == 0xAA)
            .count()
    };
    let before = polls(&device);
    run.poll_until(&mut session, |_| polls(&device) >= before + 2);
    let shot = run.shot(&device, &mut session, &FakeShot::xi_full_swing());
    assert_eq!(
        kinds(&shot),
        ["trigger", "launch", "flight", "club", "finished"]
    );
    for key in ["battery_pct", "temp_c", "external_power"] {
        assert!(!ever(&run.events, key), "{key}");
    }
    assert!(!device.sent().iter().any(|f| f.type_id == 0x95));
}

#[test]
fn the_fusion_camera_config_follows_the_warmup_and_waits_for_an_open_shot() {
    let mut run = Run::new();
    let mut mevo = driver(Config {
        camera_mode: Some(CameraMode::Fusion),
        ..Config::default()
    });
    let device = FakeDevice::mevo_plus();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let fusion = CamConfig::fusion_preset().encode();
    let fusion_sent = |device: &FakeDevice| {
        sent_index(device, |f| {
            f.dest == BusAddr::Pi && f.type_id == 0x82 && f.payload == fusion
        })
    };
    // Still warming up.
    for _ in 0..5 {
        run.poll(&mut session).expect("poll");
    }
    assert_eq!(fusion_sent(&device), None);

    // Warmed up, but a shot is open: held.
    let (trigger, rest) = split_trigger(&FakeShot::xi_full_swing());
    device.push(trigger.src, trigger.type_id, trigger.payload);
    run.poll_until(&mut session, |e| {
        matches!(e, DeviceEvent::Frp(FrpEvent::ShotTrigger { .. }))
    });
    session.warmup = Duration::ZERO;
    for _ in 0..5 {
        run.poll(&mut session).expect("poll");
    }
    assert_eq!(fusion_sent(&device), None);

    // Applied once the shot is done, then re-armed.
    let armed = arms(&device);
    run.shot(&device, &mut session, &rest);
    run.poll_until(&mut session, |_| arms(&device) == armed + 2);
    run.armed(&mut session);
    let cam = fusion_sent(&device).expect("fusion config");
    let result_request = sent_index(&device, |f| f.type_id == 0x6D).expect("0x6D");
    let rearm = device
        .sent()
        .iter()
        .rposition(|f| f.type_id == 0xB0 && f.payload == [0x01, 0x01])
        .expect("arm");
    assert!(
        result_request < cam && cam < rearm,
        "{result_request} {cam} {rearm}"
    );
    assert!(run.events.iter().any(|e| matches!(
        e,
        DeviceEvent::Log(Level::Info, l) if l.starts_with("camera warmup complete")
    )));
}

#[test]
fn a_mevo_mode_change_sends_only_the_mode_and_re_arms() {
    let mut run = Run::new();
    let config = Config::default();
    let mut mevo = driver(config.clone());
    let device = FakeDevice::mevo_plus();
    let mut session = run
        .connect(&mut mevo, device.clone(), full())
        .expect("connect");
    run.armed(&mut session);
    let armed = arms(&device);
    let sent = device.sent().len();
    session
        .command(&DeviceCommand::default().with_mode(DetectionMode::Chipping))
        .expect("command");
    run.poll_until(&mut session, |_| arms(&device) == armed + 1);
    run.armed(&mut session);
    assert_eq!(session.mode(), DetectionMode::Chipping);
    let after: Vec<(u8, Vec<u8>)> = device.sent()[sent..]
        .iter()
        .filter(|f| f.dest == BusAddr::Avr && f.type_id != 0xAA)
        .map(|f| (f.type_id, f.payload.clone()))
        .collect();
    let chipping = config.mevo_mode(DetectionMode::Chipping);
    // Disarm, config gate, the mode and its commit, then the arm: no
    // parameters and no radar calibration.
    assert_eq!(
        after.iter().map(|(t, _)| *t).collect::<Vec<_>>(),
        [0xB0, 0xB0, 0xA5, 0xB0, 0xB0]
    );
    assert_eq!(after[2].1.last(), Some(&chipping));
    assert_eq!(after[4].1, [0x01, 0x01]);
}
