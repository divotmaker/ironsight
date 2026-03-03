//! Non-blocking client example: connect, handshake, configure, arm, shot loop.
//!
//! Usage: cargo run --example client
//!
//! Requires: connected to Mevo+ WiFi (SSID = device serial).

use std::net::SocketAddr;
use std::process;
use std::time::Duration;

use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::DEFAULT_ADDR;
use ironsight::protocol::camera::CamConfig;
use ironsight::protocol::config::{ParamData, ParamValue, RadarCal, MODE_CHIPPING};
use ironsight::seq::AvrSettings;
use ironsight::BinaryConnection;

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
    let addr: SocketAddr = DEFAULT_ADDR.parse().unwrap();
    println!("Connecting to {addr}...");
    let mut conn = BinaryConnection::connect_timeout(&addr, Duration::from_secs(5))?;
    println!("Connected.");

    conn.set_on_send(|cmd, dest| println!(">> {}", cmd.debug_hex(dest)));
    conn.set_on_recv(|env| println!("<< {env:?}"));

    let mut client = BinaryClient::from_tcp(conn)?;

    // Enqueue the full startup sequence.
    client.handshake();
    client.configure_avr(default_avr_settings());
    client.configure_cam(default_cam_config());
    client.arm();

    println!("Starting poll loop...");
    loop {
        if let Some(event) = client.poll()? {
            match event {
                BinaryEvent::Handshake(h) => {
                    println!("\n=== Handshake complete ===");
                    println!("  SSID: {}  password: {}", h.pi.ssid, h.pi.password);
                    println!("  DSP: {}", h.dsp.dev_info.text);
                    println!("  AVR: {}", h.avr.dev_info.text);
                    println!("  PI:  {}", h.pi.dev_info.text);
                    println!(
                        "  Battery: {}%{}",
                        h.dsp.status.battery_percent(),
                        if h.dsp.status.external_power() {
                            " (plugged in)"
                        } else {
                            ""
                        },
                    );
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
                BinaryEvent::Shot(data) => {
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
                    if let Some(ref s) = data.spin {
                        println!(
                            "    Spin: {} rpm  Axis: {:.1}°",
                            s.pm_spin_final, s.spin_axis,
                        );
                    }
                    println!("  === RE-ARMED ===");
                }
                BinaryEvent::Disarmed => {
                    println!("\n=== Disarmed ===");
                }
                BinaryEvent::Keepalive(_) | BinaryEvent::Message(_) => {
                    // on_recv callback already printed it
                }
            }
        }
    }
}

fn default_avr_settings() -> AvrSettings {
    AvrSettings {
        mode: MODE_CHIPPING,
        params: vec![
            ParamValue {
                param_id: 0x06,
                value: ParamData::Int24(0),
            },
            ParamValue {
                param_id: 0x0F,
                value: ParamData::Float40(1.0),
            },
            ParamValue {
                param_id: 0x26,
                value: ParamData::Float40(0.0381),
            },
        ],
        radar_cal: Some(RadarCal {
            range_mm: 2743,
            height_mm: 25,
        }),
    }
}

fn default_cam_config() -> CamConfig {
    CamConfig {
        dynamic_config: true,
        resolution_width: 1024,
        resolution_height: 768,
        rotation: 0,
        ev: 0,
        quality: 80,
        framerate: 20,
        streaming_framerate: 1,
        ringbuffer_pretime_ms: 1000,
        ringbuffer_posttime_ms: 4000,
        raw_camera_mode: 0,
        fusion_camera_mode: false,
        raw_shutter_speed_max: 0.0,
        raw_ev_roi_x: -1,
        raw_ev_roi_y: -1,
        raw_ev_roi_width: -1,
        raw_ev_roi_height: -1,
        raw_x_offset: -1,
        raw_bin44: false,
        raw_live_preview_write_interval_ms: -1,
        raw_y_offset: -1,
        buffer_sub_sampling_pre_trigger_div: -1,
        buffer_sub_sampling_post_trigger_div: -1,
        buffer_sub_sampling_switch_time_offset: -1.0,
        buffer_sub_sampling_total_buffer_size: -1,
        buffer_sub_sampling_pre_trigger_buffer_size: -1,
    }
}
