//! ironsight-frp — FlightRelay Protocol device server for Mevo+/Gen2.
//!
//! Connects to a FlightScope Mevo+ or Gen2 on TCP 5100, arms it, and serves
//! shot data over FRP (WebSocket on port 5880) to any connected controller.

use std::io::Write;
use std::process::ExitCode;
use std::thread;
use std::time::Duration;

use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::{BinaryConnection, DEFAULT_ADDR};
use ironsight::frp::FrpServer;
use ironsight::protocol::config;
use ironsight::seq::AvrSettings;

fn main() -> ExitCode {
    let mevo_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_owned());
    let frp_addr = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "0.0.0.0:5880".to_owned());

    eprintln!("ironsight-frp: connecting to Mevo at {mevo_addr}");
    eprintln!("ironsight-frp: FRP server on {frp_addr}");

    // Bind FRP server first so controllers can connect while we handshake
    let mut frp = match FrpServer::bind(&frp_addr) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ironsight-frp: failed to bind FRP server: {e}");
            return ExitCode::FAILURE;
        }
    };
    eprintln!("ironsight-frp: FRP server listening");

    // Accept controller connection (blocking)
    eprintln!("ironsight-frp: waiting for FRP controller...");
    if let Err(e) = frp.accept() {
        eprintln!("ironsight-frp: controller accept failed: {e}");
        return ExitCode::FAILURE;
    }
    eprintln!("ironsight-frp: controller connected");

    // Connect to Mevo
    let conn = match BinaryConnection::connect(&mevo_addr) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("ironsight-frp: failed to connect to Mevo: {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut client = match BinaryClient::from_tcp(conn) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("ironsight-frp: failed to create client: {e}");
            return ExitCode::FAILURE;
        }
    };

    client.handshake();

    let mut armed = false;

    loop {
        match client.poll() {
            Ok(Some(event)) => {
                match &event {
                    BinaryEvent::Handshake(h) => {
                        eprintln!("ironsight-frp: handshake complete — {}", h.pi.ssid);
                        frp.set_device_name(h);
                        if let Err(e) = frp.send_device_info(h) {
                            eprintln!("ironsight-frp: send device_info failed: {e}");
                        }

                        // Configure for indoor full-swing and arm
                        client.configure_avr(AvrSettings {
                            mode: config::MODE_INDOOR,
                            params: vec![],
                            radar_cal: None,
                        });
                        client.arm();
                    }
                    BinaryEvent::Armed => {
                        armed = true;
                        eprintln!("ironsight-frp: armed");
                    }
                    BinaryEvent::Trigger => {
                        eprintln!("ironsight-frp: shot triggered");
                    }
                    BinaryEvent::ShotComplete(data) => {
                        if let Some(ref f) = data.flight {
                            eprintln!(
                                "ironsight-frp: shot #{} — carry {:.1}m, speed {:.1}m/s",
                                f.total, f.carry_distance, f.launch_speed
                            );
                        }
                    }
                    BinaryEvent::Keepalive(_) => {}
                    _ => {}
                }

                if let Err(e) = frp.handle_event(&event) {
                    eprintln!("ironsight-frp: FRP send error: {e}");
                }
            }
            Ok(None) => {
                // Check for controller commands
                if let Some(mode) = frp.check_controller() {
                    eprintln!("ironsight-frp: detection mode → {mode}");
                    let avr_mode = ironsight::frp::detection_mode_to_avr(mode);
                    client.configure_avr(AvrSettings {
                        mode: avr_mode,
                        params: vec![],
                        radar_cal: None,
                    });
                    client.arm();
                    armed = false;
                }

                thread::sleep(Duration::from_millis(1));
            }
            Err(e) => {
                eprintln!("ironsight-frp: poll error: {e}");
                if armed {
                    // Try to recover — re-arm
                    eprintln!("ironsight-frp: attempting re-arm...");
                    client.arm();
                    armed = false;
                } else {
                    return ExitCode::FAILURE;
                }
            }
        }

        // Flush stderr
        let _ = std::io::stderr().flush();
    }
}
