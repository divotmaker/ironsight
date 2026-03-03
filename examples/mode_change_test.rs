//! Live device integration test: connect, arm, mode change, keepalive, mode change.
//!
//! Usage: cargo run --example mode_change_test
//!
//! Requires: connected to Mevo+ WiFi (192.168.2.1:5100).

use std::net::SocketAddr;
use std::process;
use std::time::{Duration, Instant};

use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::DEFAULT_ADDR;
use ironsight::protocol::camera::CamConfig;
use ironsight::protocol::config::{
    MODE_CHIPPING, MODE_OUTDOOR, MODE_PUTTING, ParamData, ParamValue, RadarCal,
};
use ironsight::seq::AvrSettings;
use ironsight::BinaryConnection;

// ── Test steps ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
enum Step {
    /// Handshake + initial configure + cam configure + arm
    InitialArm,
    /// Wait for at least one keepalive while armed
    WaitKeepalive,
    /// First mode change: Outdoor → Chipping (disarm → configure → arm)
    ModeChange1,
    /// Wait for keepalive after first mode change
    WaitKeepalive2,
    /// Second mode change: Chipping → Putting (disarm → configure → arm)
    ModeChange2,
    /// Wait for keepalive after second mode change
    WaitKeepalive3,
    /// All done
    Done,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("\n✗ FAILED: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), ironsight::ConnError> {
    let addr: SocketAddr = DEFAULT_ADDR.parse().unwrap();
    println!("=== Mode Change Integration Test ===\n");

    // ── Step 1: Connect ──────────────────────────────────────────────────
    println!("[connect] connecting to {addr}...");
    let mut conn = BinaryConnection::connect_timeout(&addr, Duration::from_secs(5))?;
    println!("[connect] OK\n");

    conn.set_on_send(|cmd, dest| {
        eprintln!("  >> {dest:?} {}", cmd.debug_hex(dest));
    });
    conn.set_on_recv(|env| {
        let hex: String = env.raw.iter().map(|b| format!("{b:02X}")).collect();
        eprintln!("  << 0x{:02X} {hex} | {:?}", env.type_id, env.message);
    });

    let mut client = BinaryClient::from_tcp(conn)?;
    client.set_keepalive_interval(Duration::from_secs(3));

    // Enqueue: handshake → configure(outdoor) → cam → arm
    client.handshake();
    client.configure_avr(avr_settings_full(MODE_OUTDOOR));
    client.configure_cam(cam_config());
    client.arm();

    let mut step = Step::InitialArm;
    let mut phase_start = Instant::now();
    let timeout = Duration::from_secs(60);

    // Track sub-events within InitialArm
    let mut handshake_done = false;
    let mut configure_count = 0u8; // need 2: AVR + cam

    println!("[step] InitialArm: handshake → configure(outdoor/9) → cam → arm");

    loop {
        if phase_start.elapsed() > timeout {
            return Err(ironsight::ConnError::Protocol(format!(
                "timeout in step {step:?} after {timeout:?}"
            )));
        }

        let event = client.poll()?;
        let Some(event) = event else {
            continue;
        };

        match (&step, &event) {
            // ── InitialArm phase ──────────────────────────────────────
            (Step::InitialArm, BinaryEvent::Handshake(h)) => {
                println!(
                    "  [handshake] OK — {} | battery {}%",
                    h.avr.dev_info.text.trim(),
                    h.dsp.status.battery_percent()
                );
                handshake_done = true;
            }
            (Step::InitialArm, BinaryEvent::Configured) => {
                configure_count += 1;
                let label = if configure_count == 1 { "AVR" } else { "cam" };
                println!("  [configured] {label} OK");
            }
            (Step::InitialArm, BinaryEvent::Armed) => {
                println!("  [armed] OK — device armed in outdoor mode");
                assert!(handshake_done, "armed before handshake");
                assert!(configure_count >= 2, "armed before both configures");

                // Transition: wait for keepalive
                step = Step::WaitKeepalive;
                phase_start = Instant::now();
                println!("\n[step] WaitKeepalive: waiting for keepalive...");
            }

            // ── WaitKeepalive ─────────────────────────────────────────
            (Step::WaitKeepalive, BinaryEvent::Keepalive(snap)) => {
                let avr = snap.avr.as_ref().map(|a| format!("tilt={:.1}", a.tilt));
                let dsp = snap
                    .dsp
                    .as_ref()
                    .map(|d| format!("bat={}%", d.battery_percent()));
                println!("  [keepalive] OK — {} {}", dsp.unwrap_or_default(), avr.unwrap_or_default());

                // Transition: first mode change outdoor → chipping
                step = Step::ModeChange1;
                phase_start = Instant::now();
                println!("\n[step] ModeChange1: outdoor(9) → chipping(5)");
                client.configure_avr(avr_settings_mode_only(MODE_CHIPPING));
                client.arm();
            }

            // ── ModeChange1 (outdoor → chipping) ─────────────────────
            (Step::ModeChange1, BinaryEvent::Disarmed) => {
                println!("  [disarmed] OK");
            }
            (Step::ModeChange1, BinaryEvent::Configured) => {
                println!("  [configured] chipping OK");
            }
            (Step::ModeChange1, BinaryEvent::Armed) => {
                println!("  [armed] OK — chipping mode");

                step = Step::WaitKeepalive2;
                phase_start = Instant::now();
                println!("\n[step] WaitKeepalive2: waiting for keepalive...");
            }

            // ── WaitKeepalive2 ────────────────────────────────────────
            (Step::WaitKeepalive2, BinaryEvent::Keepalive(snap)) => {
                let dsp = snap
                    .dsp
                    .as_ref()
                    .map(|d| format!("bat={}%", d.battery_percent()));
                println!("  [keepalive] OK — {}", dsp.unwrap_or_default());

                // Transition: second mode change chipping → putting
                step = Step::ModeChange2;
                phase_start = Instant::now();
                println!("\n[step] ModeChange2: chipping(5) → putting(3)");
                client.configure_avr(avr_settings_mode_only(MODE_PUTTING));
                client.arm();
            }

            // ── ModeChange2 (chipping → putting) ─────────────────────
            (Step::ModeChange2, BinaryEvent::Disarmed) => {
                println!("  [disarmed] OK");
            }
            (Step::ModeChange2, BinaryEvent::Configured) => {
                println!("  [configured] putting OK");
            }
            (Step::ModeChange2, BinaryEvent::Armed) => {
                println!("  [armed] OK — putting mode");

                step = Step::WaitKeepalive3;
                phase_start = Instant::now();
                println!("\n[step] WaitKeepalive3: waiting for keepalive...");
            }

            // ── WaitKeepalive3 ────────────────────────────────────────
            (Step::WaitKeepalive3, BinaryEvent::Keepalive(snap)) => {
                let dsp = snap
                    .dsp
                    .as_ref()
                    .map(|d| format!("bat={}%", d.battery_percent()));
                println!("  [keepalive] OK — {}", dsp.unwrap_or_default());

                step = Step::Done;
            }

            // ── Done ──────────────────────────────────────────────────
            (Step::Done, _) => unreachable!(),

            // ── Passthrough (text, messages, etc.) ────────────────────
            (_, BinaryEvent::Message(_)) => {}
            (_, BinaryEvent::Keepalive(_)) => {} // extra keepalives
            (_, other) => {
                println!("  [unexpected] {other:?} in step {step:?}");
            }
        }

        if step == Step::Done {
            break;
        }
    }

    println!("\n=== ALL STEPS PASSED ===");
    println!("  1. Connect + handshake + configure(outdoor) + arm  ✓");
    println!("  2. Keepalive received while armed                  ✓");
    println!("  3. Mode change outdoor → chipping                  ✓");
    println!("  4. Keepalive after mode change                     ✓");
    println!("  5. Mode change chipping → putting                  ✓");
    println!("  6. Keepalive after second mode change              ✓");

    Ok(())
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn avr_settings_full(mode: u8) -> AvrSettings {
    AvrSettings {
        mode,
        params: vec![
            ParamValue {
                param_id: 0x06,
                value: ParamData::Int24(0), // standard ball
            },
            ParamValue {
                param_id: 0x0F,
                value: ParamData::Float40(0.8), // 80% track
            },
            ParamValue {
                param_id: 0x26,
                value: ParamData::Float40(0.0381), // 1.5" tee height
            },
        ],
        radar_cal: Some(RadarCal {
            range_mm: 2133, // ~7 feet
            height_mm: 0,
        }),
    }
}

/// Minimal mode change settings: ModeSet only, no params, no RadarCal.
/// This is the fastest mode change — tested and working in run 7.
fn avr_settings_mode_only(mode: u8) -> AvrSettings {
    AvrSettings {
        mode,
        params: vec![],
        radar_cal: None,
    }
}

fn cam_config() -> CamConfig {
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
