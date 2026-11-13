//! `ironsight-frp` end to end: the driver connects over TCP to a scripted
//! Xi, flightrelay's `FrpServer` runs it as a launch monitor actor, and a
//! peer on the bus reads the shot.

#![cfg(all(feature = "frp", feature = "testing"))]

use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use flightrelay::driver::{DeviceDriver, Stop};
use flightrelay::{
    BusActor, EndpointConfig, FrpBody, FrpEndpoint, FrpServer, ServerConfig, ServerEvent, Source,
    Transport,
};
use ironsight::config::Config;
use ironsight::driver::Mevo;
use ironsight::testing::{FakeDevice, Shot};

/// The launch monitor actor's id.
const ACTOR: &str = "3c9e1f27b04d68a5";

/// The actor's blockers as `"true"` (none: armed) or `"false"`
/// (`not_armed`).
fn ready(actor: &BusActor) -> String {
    match actor.blockers.as_slice() {
        [] => "true".to_owned(),
        [b] if b == "not_armed" => "false".to_owned(),
        other => panic!("unexpected blockers {other:?}"),
    }
}

/// What the scripted device does next.
enum Do {
    /// Send this shot.
    Shot(Shot),
    /// Drop the connection; the next one gets a fresh device.
    Hang,
}

/// Serve a `FakeDevice::xi()` on a local TCP port, one connection at a time,
/// doing what arrives on the returned channel. Each device lives on this
/// thread.
fn fake_xi_over_tcp(stop: Stop) -> (SocketAddr, mpsc::Sender<Do>, std::thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr");
    let (todo, todo_rx) = mpsc::channel::<Do>();
    let thread = std::thread::spawn(move || {
        listener.set_nonblocking(true).expect("nonblocking");
        while !stop.is_stopped() {
            let mut socket = match listener.accept() {
                Ok((s, _)) => s,
                Err(e) if e.kind() == ErrorKind::WouldBlock => {
                    std::thread::sleep(Duration::from_millis(5));
                    continue;
                }
                Err(e) => panic!("{e}"),
            };
            socket.set_nonblocking(true).expect("nonblocking");
            let mut device = FakeDevice::xi();
            let mut buf = [0u8; 4096];
            'link: while !stop.is_stopped() {
                while let Ok(next) = todo_rx.try_recv() {
                    match next {
                        Do::Shot(shot) => device.push_shot(&shot),
                        Do::Hang => break 'link,
                    }
                }
                match socket.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => device.write_all(&buf[..n]).expect("device write"),
                    Err(e) if e.kind() == ErrorKind::WouldBlock => {}
                    Err(_) => break,
                }
                match device.read(&mut buf) {
                    Ok(n) => {
                        if socket.write_all(&buf[..n]).is_err() {
                            break;
                        }
                    }
                    Err(_) => std::thread::sleep(Duration::from_millis(1)),
                }
            }
        }
    });
    (addr, todo, thread)
}

#[test]
#[allow(clippy::too_many_lines)] // one end-to-end story, read top to bottom
fn a_controller_reads_an_xi_shot_and_the_driver_reconnects() {
    let stop = Stop::new();
    let (device_addr, todo, device) = fake_xi_over_tcp(stop.clone());
    let driver = Mevo::new(Config {
        address: Some(device_addr.to_string()),
        ..Config::default()
    })
    .expect("config");
    assert_eq!(Mevo::info(driver.config()).source, Source::Measured);

    let (addr_tx, addr_rx) = mpsc::channel();
    let server_stop = stop.clone();
    let server = std::thread::spawn(move || {
        FrpServer::new(
            driver,
            ServerConfig::new(Transport::listen("127.0.0.1:0"), ACTOR),
        )
        .on_event(move |event| {
            if let ServerEvent::Listening(addr) = event {
                let _ = addr_tx.send(*addr);
            }
        })
        .run(&server_stop)
    });
    let addr: SocketAddr = addr_rx
        .recv_timeout(Duration::from_secs(5))
        .expect("listening");

    let mut peer = FrpEndpoint::open(EndpointConfig::new(Transport::connect(format!(
        "ws://{addr}/frp"
    ))))
    .expect("open");
    let mut conn = peer.establish().expect("connect");

    let give_up = Instant::now() + Duration::from_secs(30);
    let mut seen = Vec::new();
    let mut identity = None;
    let mut ready_seen = false;
    let mut pushed = false;
    loop {
        assert!(Instant::now() < give_up, "saw {seen:?}");
        let msg = conn.recv().expect("recv");
        assert_eq!(msg.from, ACTOR);
        // The device's own id (its address here) never goes on the bus.
        let json = msg.to_json().expect("json");
        assert!(!json.contains(&device_addr.to_string()), "{json}");
        match msg.body {
            FrpBody::BusActor(actor) => {
                assert!(actor.roles.iter().any(|r| r == "launch_monitor"));
                let device = actor.device.as_ref().expect("device block");
                assert_eq!(device.source, Some(Source::Measured));
                if device.model.is_some() {
                    identity = Some((device.model.clone(), device.firmware.clone()));
                }
                if ready(&actor) == "true" {
                    ready_seen = true;
                }
            }
            FrpBody::ShotTrigger(_) => seen.push("trigger"),
            FrpBody::ShotLaunch(_) => seen.push("launch"),
            FrpBody::ShotFlight(_) => seen.push("flight"),
            FrpBody::ShotClub(_) => seen.push("club"),
            FrpBody::ShotFinished(_) => {
                seen.push("finished");
                break;
            }
            _ => {}
        }
        if ready_seen && !pushed {
            assert!(identity.is_some(), "identity before the first shot");
            todo.send(Do::Shot(Shot::xi_full_swing())).expect("push");
            pushed = true;
        }
    }
    assert_eq!(seen, ["trigger", "launch", "flight", "club", "finished"]);
    let (model, firmware) = identity.expect("identity in the device block");
    assert_eq!(model.as_deref(), Some("Xi"));
    assert_eq!(firmware.as_deref(), Some("3.18 (Jan 1 2020)"));

    // The device drops the connection: `not_armed`, a lost-connection alert,
    // then the driver reconnects and arms again.
    todo.send(Do::Hang).expect("hang");
    let mut states = Vec::new();
    let mut lost = false;
    // (The shot's own re-arm may still be on its way.)
    let reconnected = |states: &[String]| {
        states
            .iter()
            .position(|r| r == "false")
            .is_some_and(|i| states[i..].iter().any(|r| r == "true"))
    };
    while !(lost && reconnected(&states)) {
        assert!(Instant::now() < give_up, "saw {states:?}");
        let msg = conn.recv().expect("recv");
        assert_eq!(msg.from, ACTOR);
        match msg.body {
            FrpBody::BusActor(actor) => states.push(ready(&actor)),
            // The driver's alert about the loss, under the actor's id.
            FrpBody::BusAlert(alert) => {
                assert!(alert.message.contains("closed"), "{}", alert.message);
                lost = true;
            }
            _ => {}
        }
    }

    stop.stop();
    server
        .join()
        .expect("server thread")
        .expect("server result");
    device.join().expect("device thread");
}
