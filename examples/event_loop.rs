//! Main processing loop: connect to a live Mevo+ and print all messages.
//!
//! Usage: cargo run --example event_loop
//!
//! Requires: connected to Mevo+ WiFi (SSID = device serial).

use std::net::SocketAddr;
use std::process;
use std::time::Duration;

use ironsight::conn::DEFAULT_ADDR;
use ironsight::protocol::camera::CamConfig;
use ironsight::protocol::config::{MODE_OUTDOOR, ParamData, ParamValue, RadarCal};
use ironsight::seq::{self, AvrSettings, AvrSync, DspSync, KeepaliveStatus, PiSync};
use ironsight::{ConnError, Connection, Message};

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
        mode: MODE_OUTDOOR,
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
        raw_ev_roi_x: 0,
        raw_ev_roi_y: 0,
        raw_ev_roi_width: 0,
        raw_ev_roi_height: 0,
        raw_x_offset: 0,
        raw_bin44: false,
        raw_live_preview_write_interval_ms: 0,
        raw_y_offset: 0,
        buffer_sub_sampling_pre_trigger_div: 1,
        buffer_sub_sampling_post_trigger_div: 1,
        buffer_sub_sampling_switch_time_offset: 0.0,
        buffer_sub_sampling_total_buffer_size: 0,
        buffer_sub_sampling_pre_trigger_buffer_size: 0,
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

fn print_keepalive(status: &KeepaliveStatus) {
    println!(
        "[keepalive] tilt={:.1}° roll={:.1}° temp={:.1}°C battery={}%{}",
        status.avr.tilt,
        -status.avr.roll,
        status.dsp.temperature_c(),
        status.dsp.battery_percent(),
        if status.dsp.external_power() {
            " (plugged in)"
        } else {
            ""
        },
    );
}

// ---------------------------------------------------------------------------
// Retry helper
// ---------------------------------------------------------------------------

/// Returns true for transient errors that should be retried with "...".
fn is_transient(e: &ConnError) -> bool {
    matches!(e, ConnError::Timeout { .. } | ConnError::Protocol(_))
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

fn run() -> Result<(), ConnError> {
    // 1. Connect
    let addr: SocketAddr = DEFAULT_ADDR.parse().unwrap();
    println!("Connecting to {addr}...");
    let mut conn = Connection::connect_timeout(&addr, Duration::from_secs(5))?;
    println!("Connected.");

    // Wire-level trace callbacks.
    conn.set_on_send(|cmd, dest| println!("\n>> {:?} [{}]", cmd, cmd.debug_hex(dest)));
    conn.set_on_recv(|env| println!("\n<< {env:?}"));

    // 2. Handshake
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

    // 5. Main event loop
    let recv_timeout = Duration::from_millis(900);
    loop {
        match conn.recv_timeout(recv_timeout) {
            Ok(env) => {
                match env.message {
                    // --- Shot state transitions ---
                    Message::ShotText(ref st) => {
                        if st.is_processed() {
                            // Drive the entire post-shot cycle:
                            // ack → drain to IDLE → rearm → ARMED.
                            loop {
                                match seq::complete_shot(&mut conn, |s| println!("  {s}")) {
                                    Ok(()) => break,
                                    Err(ref e) if is_transient(e) => {
                                        println!("  ... {e}");
                                        continue;
                                    }
                                    Err(e) => return Err(e),
                                }
                            }
                        }
                    }

                    // --- Flight result (primary) ---
                    Message::FlightResult(ref r) => {
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
                    Message::FlightResultV1(ref r) => {
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
                    Message::ClubResult(ref r) => {
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
                    Message::SpinResult(ref r) => {
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
                    Message::SpeedProfile(ref r) => {
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
                    Message::TrackingStatus(ref r) => {
                        println!(
                            "  Tracking: iter={} quality={} prc_count={} trigger_idx={}",
                            r.processing_iteration,
                            r.result_quality,
                            r.prc_tracking_count,
                            r.trigger_idx,
                        );
                    }

                    // --- PRC data ---
                    Message::PrcData(ref r) => {
                        println!("  PRC: seq={} points={}", r.sequence, r.points.len());
                    }

                    // --- Club PRC data ---
                    Message::ClubPrc(ref r) => {
                        println!("  Club PRC: points={}", r.points.len());
                    }

                    // --- Camera image available ---
                    Message::CamImageAvail(ref r) => {
                        println!(
                            "  Camera: streaming={} fusion={} video={}",
                            r.streaming_available, r.fusion_available, r.video_available,
                        );
                    }

                    // --- Config ack (suppressed above) ---
                    Message::ConfigAck(_) => {}

                    // --- Everything else: debug line is enough ---
                    _ => {}
                }
            }

            Err(ConnError::Timeout { .. }) => match seq::keepalive(&mut conn) {
                Ok(status) => print_keepalive(&status),
                Err(ref e) if is_transient(e) => println!("  ... {e}"),
                Err(e) => return Err(e),
            },

            Err(ConnError::Disconnected) => {
                println!("Device disconnected.");
                return Ok(());
            }

            Err(e) => return Err(e),
        }
    }
}
