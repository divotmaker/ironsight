//! ironsight-frp — Flight Relay Protocol device for Mevo+/Gen2/Xi.
//!
//! Connects to a FlightScope Mevo+, Gen2 or Xi on TCP 5100, arms it, and
//! streams shot data over FRP to a controller.
//!
//! ```text
//! ironsight-frp [mevo-addr] [frp-target]
//! ```
//!
//! `frp-target` selects the transport direction. A `ws://` or `wss://` URL
//! bridges this device to a central controller such as flighthook; anything
//! else is a bind address that controllers connect to. Defaults to
//! `0.0.0.0:5880`.

use std::fmt::Write as _;
use std::io::Write;
use std::process::ExitCode;
use std::thread;
use std::time::Duration;

use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::{BinaryConnection, DEFAULT_ADDR};
use ironsight::frp::FrpDevice;
use ironsight::protocol::config;
use ironsight::protocol::handshake::DeviceModel;
use ironsight::seq::{AvrSettings, XiSettings};

fn main() -> ExitCode {
    let mevo_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_owned());
    let frp_target = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "0.0.0.0:5880".to_owned());
    let bridging = frp_target.starts_with("ws://") || frp_target.starts_with("wss://");

    eprintln!("ironsight-frp: connecting to Mevo at {mevo_addr}");

    // Open the FRP endpoint first so it connects while we handshake the Mevo
    let frp = if bridging {
        eprintln!("ironsight-frp: bridging to controller at {frp_target}");
        FrpDevice::bridge(&frp_target, "ironsight")
    } else {
        eprintln!("ironsight-frp: serving controllers on {frp_target}");
        FrpDevice::serve(&frp_target)
    };
    let mut frp = match frp {
        Ok(s) => s,
        Err(e) => {
            eprintln!("ironsight-frp: failed to open FRP endpoint: {e}");
            return ExitCode::FAILURE;
        }
    };

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
                        frp.set_device_name(h);
                        eprintln!(
                            "ironsight-frp: handshake complete — {} ({})",
                            frp.device_name(),
                            h.model,
                        );
                        if let Err(e) = frp.send_device_info(h) {
                            eprintln!("ironsight-frp: send device_info failed: {e}");
                        }

                        if h.model == DeviceModel::Xi {
                            // FS Golf's full-swing configuration, limited-flight mode (9)
                            client.configure_xi(XiSettings::default());
                        } else {
                            // Configure for indoor full-swing
                            client.configure_avr(AvrSettings {
                                mode: config::MODE_INDOOR,
                                params: vec![],
                                radar_cal: None,
                            });
                        }
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
                    BinaryEvent::StepRefused(env) => {
                        let hex: String = env.raw.iter().fold(String::new(), |mut s, b| {
                            let _ = write!(s, "{b:02x}");
                            s
                        });
                        eprintln!(
                            "ironsight-frp: warning: step refused, continuing: 0x{:02X} {hex}",
                            env.type_id
                        );
                    }
                    BinaryEvent::Keepalive(_) => {}
                    _ => {}
                }

                if let Err(e) = frp.handle_event(&event) {
                    eprintln!("ironsight-frp: FRP send error: {e}");
                }
            }
            Ok(None) => {
                // Adopt a newly established controller connection
                match frp.poll_connection() {
                    Ok(true) => eprintln!("ironsight-frp: controller connected"),
                    Ok(false) => {}
                    Err(e) => eprintln!("ironsight-frp: telemetry resend failed: {e}"),
                }

                // Check for controller commands
                if let Some(mode) = frp.check_controller() {
                    if client.model() == DeviceModel::Xi {
                        eprintln!(
                            "ironsight-frp: detection mode → {mode} ignored (Xi stays in limited-flight mode)"
                        );
                    } else {
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
