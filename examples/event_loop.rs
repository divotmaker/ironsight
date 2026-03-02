//! Main processing loop: connect to a live Mevo+ and print all messages.
//!
//! Usage: cargo run --example event_loop
//!
//! Requires: connected to Mevo+ WiFi (SSID = device serial).

use std::net::SocketAddr;
use std::process;
use std::time::{Duration, Instant};

use ironsight::conn::DEFAULT_ADDR;
use ironsight::protocol::camera::CamConfig;
use ironsight::protocol::config::{ParamData, ParamValue, RadarCal, MODE_CHIPPING};
use ironsight::seq::{self, Action, AvrSettings, AvrSync, DspSync, PiSync, ShotSequencer};
use ironsight::{BinaryConnection, ConnError, Message, Sequence};

// ---------------------------------------------------------------------------
// Unit conversion helpers
// ---------------------------------------------------------------------------

/// m/s → mph
fn ms_to_mph(ms: f64) -> f64 {
    ms * 2.23694
}

/// meters → yards
fn m_to_yd(m: f64) -> f64 {
    m * 1.09361
}

/// meters → feet
fn m_to_ft(m: f64) -> f64 {
    m * 3.28084
}

/// meters → inches
fn m_to_in(m: f64) -> f64 {
    m * 39.3701
}

// ---------------------------------------------------------------------------
// Default settings
// ---------------------------------------------------------------------------

fn default_avr_settings() -> AvrSettings {
    AvrSettings {
        mode: MODE_CHIPPING,
        params: vec![
            // Ball type: 0 = RCT
            ParamValue {
                param_id: 0x06,
                value: ParamData::Int24(0),
            },
            // Outdoor minimum track %: 1.0
            ParamValue {
                param_id: 0x0F,
                value: ParamData::Float40(1.0),
            },
            // Driver tee height: 1.5 inches = 0.0381m
            ParamValue {
                param_id: 0x26,
                value: ParamData::Float40(0.0381),
            },
        ],
        radar_cal: RadarCal {
            range_mm: 2743, // 9 feet
            height_mm: 25,  // floor(1.0 inches * 25.4) = 25
        },
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

// ---------------------------------------------------------------------------
// Pretty-printers
// ---------------------------------------------------------------------------

fn print_handshake(dsp: &DspSync, avr: &AvrSync, pi: &PiSync) {
    println!("--- Handshake complete ---");
    println!(
        "  DSP: type=0x{:02X} pcb={}",
        dsp.hw_info.dsp_type, dsp.hw_info.pcb
    );
    println!("  DSP info: {}", dsp.dev_info.text);
    println!("  AVR info: {}", avr.dev_info.text);
    println!(
        "  AVR tilt={:.1}° roll={:.1}°",
        avr.status.tilt, -avr.status.roll
    );
    println!("  PI info: {}", pi.dev_info.text);
    println!("  SSID: {}  password: {}", pi.ssid, pi.password);
    println!(
        "  Battery: {}%  {}",
        dsp.status.battery_percent(),
        if dsp.status.external_power() {
            "(plugged in)"
        } else {
            ""
        },
    );
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        process::exit(1);
    }
}

fn send_action(
    conn: &mut BinaryConnection<std::net::TcpStream>,
    action: Action,
) -> Result<(), ConnError> {
    seq::send_action(conn, action)
}

fn run() -> Result<(), ConnError> {
    // 1. Connect
    let addr: SocketAddr = DEFAULT_ADDR.parse().unwrap();
    println!("Connecting to {addr}...");
    let mut conn = BinaryConnection::connect_timeout(&addr, Duration::from_secs(5))?;
    println!("Connected.");

    // Wire-level trace callbacks.
    conn.set_on_send(|cmd, dest| println!("\n>> {:?} [{}]", cmd, cmd.debug_hex(dest)));
    conn.set_on_recv(|env| println!("\n<< {env:?}"));

    // 2. Handshake (blocking via drive())
    println!("DSP sync...");
    let dsp = seq::sync_dsp(&mut conn)?;
    println!("AVR sync...");
    let avr = seq::sync_avr(&mut conn)?;
    println!("PI sync...");
    let pi = seq::sync_pi(&mut conn)?;
    print_handshake(&dsp, &avr, &pi);

    // 3. Configure
    let settings = default_avr_settings();
    println!(
        "Configuring... mode=Outdoor ball=RCT range={}mm height={}mm",
        settings.radar_cal.range_mm, settings.radar_cal.height_mm,
    );
    seq::configure_avr(&mut conn, &settings)?;
    seq::configure_camera(&mut conn, &default_cam_config())?;

    // 4. Arm
    seq::arm(&mut conn)?;
    println!("=== ARMED — waiting for shots ===");

    // 5. Non-blocking event loop
    conn.stream_mut()
        .set_read_timeout(Some(Duration::from_millis(1)))?;
    let mut last_keepalive = Instant::now();
    let mut shot: Option<ShotSequencer> = None;

    loop {
        if let Some(env) = conn.recv()? {
            // If a shot sequence is active, feed it first.
            if let Some(ref mut s) = shot {
                let actions = s.feed(&env);
                for a in actions {
                    send_action(&mut conn, a)?;
                }
                if s.is_complete() {
                    let data = shot.take().unwrap().into_result();
                    print_shot(&data);
                    continue;
                }
            }

            // Handle messages not consumed by the shot sequencer.
            match &env.message {
                Message::ShotText(st) if st.is_processed() && shot.is_none() => {
                    println!("  Shot processed — starting shot sequence...");
                    let (s, actions) = ShotSequencer::new();
                    for a in actions {
                        send_action(&mut conn, a)?;
                    }
                    shot = Some(s);
                }

                // --- Flight result (primary) ---
                Message::FlightResult(r) => {
                    println!("  Flight result (shot #{}):", r.total);
                    println!(
                        "    Ball speed: {:.1} mph  VLA: {:.1}°  HLA: {:.1}°",
                        ms_to_mph(r.launch_speed),
                        r.launch_elevation,
                        r.launch_azimuth,
                    );
                    println!(
                        "    Carry: {:.1} yd  Total: {:.1} yd  Height: {:.1} ft",
                        m_to_yd(r.carry_distance),
                        m_to_yd(r.total_distance),
                        m_to_ft(r.max_height),
                    );
                    println!(
                        "    Backspin: {} rpm  Sidespin: {} rpm",
                        r.backspin_rpm, r.sidespin_rpm,
                    );
                    println!(
                        "    Club speed: {:.1} mph  Path: {:.1}°  AoA: {:.1}°",
                        ms_to_mph(r.clubhead_speed),
                        r.club_strike_direction,
                        r.club_attack_angle,
                    );
                    println!(
                        "    Face: {:.1}°  Loft: {:.1}°",
                        r.club_face_angle, r.club_effective_loft,
                    );
                }

                // --- Flight result v1 (early) ---
                Message::FlightResultV1(r) => {
                    println!("  Flight result v1 (shot #{}):", r.total);
                    println!(
                        "    Ball: {:.1} mph  Club: {:.1} mph  VLA: {:.1}°  HLA: {:.1}°",
                        ms_to_mph(r.ball_velocity),
                        ms_to_mph(r.club_velocity),
                        r.elevation,
                        r.azimuth,
                    );
                    println!(
                        "    Dist: {:.1} yd  Height: {:.1} ft  Backspin: {} rpm",
                        m_to_yd(r.distance),
                        m_to_ft(r.height),
                        r.backspin_rpm,
                    );
                }

                // --- Club result ---
                Message::ClubResult(r) => {
                    println!("  Club result:");
                    println!(
                        "    Club speed: {:.1} mph (pre) / {:.1} mph (post)  Smash: {:.2}",
                        ms_to_mph(r.pre_club_speed),
                        ms_to_mph(r.post_club_speed),
                        r.smash_factor,
                    );
                    println!(
                        "    Path: {:.1}°  Face: {:.1}°  AoA: {:.1}°  Loft: {:.1}°",
                        r.strike_direction, r.face_angle, r.attack_angle, r.dynamic_loft,
                    );
                    println!(
                        "    Low point: {:.1} in  Club height: {:.1} in",
                        m_to_in(r.club_offset),
                        m_to_in(r.club_height),
                    );
                }

                // --- Spin result ---
                Message::SpinResult(r) => {
                    println!("  Spin result:");
                    println!(
                        "    Total spin: {} rpm (PM final)  Axis: {:.1}°  Method: {}",
                        r.pm_spin_final, r.spin_axis, r.spin_method,
                    );
                    println!(
                        "    AM: {} rpm  PM: {} rpm  Launch: {} rpm  AOD: {} rpm  PLL: {} rpm",
                        r.am_spin, r.pm_spin, r.launch_spin, r.aod_spin, r.pll_spin,
                    );
                }

                // --- Speed profile ---
                Message::SpeedProfile(r) => {
                    let peak = r.speeds.iter().copied().fold(0.0_f64, f64::max);
                    println!(
                        "  Speed profile: {} samples ({}+{})  peak: {:.1} mph",
                        r.speeds.len(),
                        r.num_pre,
                        r.num_post,
                        ms_to_mph(peak),
                    );
                }

                // --- Tracking status ---
                Message::TrackingStatus(r) => {
                    println!(
                        "  Tracking: iter={} quality={} prc_count={} trigger_idx={}",
                        r.processing_iteration,
                        r.result_quality,
                        r.prc_tracking_count,
                        r.trigger_idx,
                    );
                }

                // --- PRC data ---
                Message::PrcData(r) => {
                    println!("  PRC: seq={} points={}", r.sequence, r.points.len());
                }

                // --- Club PRC data ---
                Message::ClubPrc(r) => {
                    println!("  Club PRC: points={}", r.points.len());
                }

                // --- Camera image available ---
                Message::CamImageAvail(r) => {
                    println!(
                        "  Camera: streaming={} fusion={} video={}",
                        r.streaming_available, r.fusion_available, r.video_available,
                    );
                }

                // --- DspStatus from keepalive ---
                Message::DspStatus(s) => {
                    println!(
                        "[keepalive] battery={}%{}  temp={:.1}°C",
                        s.battery_percent(),
                        if s.external_power() {
                            " (plugged in)"
                        } else {
                            ""
                        },
                        s.temperature_c(),
                    );
                }
                Message::AvrStatus(s) => {
                    println!("[keepalive] tilt={:.1}° roll={:.1}°", s.tilt, -s.roll);
                }

                // --- Config ack / noise ---
                Message::ConfigAck(_) | Message::PiStatus(_) => {}

                // --- Everything else: debug line is enough ---
                _ => {}
            }
        }

        // Heartbeats always fire — even during shot sequencing.
        if last_keepalive.elapsed() >= Duration::from_secs(1) {
            for a in seq::keepalive_actions() {
                send_action(&mut conn, a)?;
            }
            last_keepalive = Instant::now();
        }
    }
}

fn print_shot(data: &seq::ShotData) {
    println!("\n  === Shot complete ===");
    if let Some(ref f) = data.flight {
        println!(
            "    Ball: {:.1} mph  Carry: {:.0} yd",
            ms_to_mph(f.launch_speed),
            m_to_yd(f.carry_distance),
        );
    }
    if let Some(ref c) = data.club {
        println!(
            "    Club: {:.1} mph  Smash: {:.2}",
            ms_to_mph(c.pre_club_speed),
            c.smash_factor,
        );
    }
    println!("  === RE-ARMED ===\n");
}
