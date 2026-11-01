//! Non-blocking client example: connect, handshake, configure, arm, shot loop.
//!
//! Usage: cargo run --example client [device-addr]
//!
//! Requires: connected to the device's WiFi (Mevo+: SSID = device serial).
//! `device-addr` defaults to 192.168.2.1:5100.

use std::fmt::Write as _;
use std::net::SocketAddr;
use std::process;
use std::time::Duration;

use ironsight::BinaryConnection;
use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::DEFAULT_ADDR;
use ironsight::protocol::camera::CamConfig;
use ironsight::protocol::config::{MODE_CHIPPING, RadarCal};
use ironsight::protocol::handshake::DeviceModel;
use ironsight::seq::{AvrSettings, MevoSettings, XiSettings};

fn ms_to_mph(ms: f64) -> f64 {
    ms * 2.23694
}

fn m_to_yd(m: f64) -> f64 {
    m * 1.09361
}

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), ironsight::ConnError> {
    let addr: SocketAddr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_owned())
        .parse()
        .expect("device address must be ip:port");
    println!("Connecting to {addr}...");
    let mut conn = BinaryConnection::connect_timeout(&addr, Duration::from_secs(5))?;
    println!("Connected.");

    conn.set_on_send(|cmd, dest| println!(">> {}", cmd.debug_hex(dest)));
    conn.set_on_recv(|env| println!("<< {env:?}"));

    let mut client = BinaryClient::from_tcp(conn)?;

    // Configuration depends on the model, so it is enqueued once the
    // handshake has identified the device.
    client.handshake();

    println!("Starting poll loop...");
    loop {
        if let Some(event) = client.poll()? {
            match event {
                BinaryEvent::Handshake(h) => {
                    println!("\n=== Handshake complete: {} ===", h.model);
                    if let Some(ref pi) = h.pi {
                        println!("  SSID: {}  password: {}", pi.ssid, pi.password);
                    }
                    println!("  DSP: {}", h.dsp.dev_info.text);
                    println!("  AVR: {}", h.avr.dev_info.text);
                    if let Some(ref pi) = h.pi {
                        println!("  PI:  {}", pi.dev_info.text);
                    }
                    println!(
                        "  Battery: {}%{}",
                        h.dsp.status.battery_percent(),
                        if h.dsp.status.external_power() {
                            " (plugged in)"
                        } else {
                            ""
                        },
                    );

                    if h.model == DeviceModel::Xi {
                        // No camera; full swing in limited-flight mode.
                        client.configure_xi(XiSettings::default());
                    } else {
                        client.configure_avr(default_avr_settings());
                        client.configure_cam(default_cam_config());
                    }
                    client.arm();
                }
                BinaryEvent::Configured => {
                    println!("\n=== Configured ===");
                }
                BinaryEvent::Armed => {
                    println!("\n=== ARMED — waiting for shots ===");
                }
                BinaryEvent::Trigger => {
                    println!("\n  ** BALL TRIGGER **");
                }
                BinaryEvent::ShotComplete(data) => {
                    println!("\n  === Shot complete ===");
                    if let Some(ref f) = data.flight {
                        println!(
                            "    Ball: {:.1} mph  Carry: {:.0} yd  VLA: {:.1}°  HLA: {:.1}°",
                            ms_to_mph(f.launch_speed),
                            m_to_yd(f.carry_distance),
                            f.launch_elevation,
                            f.launch_azimuth,
                        );
                    }
                    if let Some(ref c) = data.club {
                        println!(
                            "    Club: {:.1} mph  Smash: {:.2}  Path: {:.1}°  Face: {:.1}°",
                            ms_to_mph(c.pre_club_speed),
                            c.smash_factor,
                            c.strike_direction,
                            c.face_angle,
                        );
                    }
                    if let Some(ref f) = data.flight {
                        println!(
                            "    Spin (D4): back {} rpm  side {} rpm",
                            f.backspin_rpm, f.sidespin_rpm,
                        );
                    }
                    if let Some(ref s) = data.spin {
                        println!(
                            "    Spin (EF): {} rpm  Axis: {:.1}°",
                            s.pm_spin_final, s.spin_axis,
                        );
                    }
                    println!("  === RE-ARMED ===");
                }
                BinaryEvent::Disarmed => {
                    println!("\n=== Disarmed ===");
                }
                BinaryEvent::StepRefused(env) => {
                    let hex = env.raw.iter().fold(String::new(), |mut s, b| {
                        let _ = write!(s, "{b:02x}");
                        s
                    });
                    println!(
                        "  WARNING: step refused, continuing: 0x{:02X} {hex}",
                        env.type_id
                    );
                }
                BinaryEvent::ShotDatum(_) | BinaryEvent::Keepalive(_) | BinaryEvent::Message(_) => {
                    // on_recv callback already printed it
                }
            }
        }
    }
}

fn default_avr_settings() -> AvrSettings {
    MevoSettings::new(
        MODE_CHIPPING,
        0,      // standard ball
        1.0,    // minimum outdoor track 100 %
        0.0381, // tee height 1.5 inches
        RadarCal {
            range_mm: 2743,
            height_mm: 25,
        },
    )
    .into()
}

fn default_cam_config() -> CamConfig {
    CamConfig::warmup_preset()
}
