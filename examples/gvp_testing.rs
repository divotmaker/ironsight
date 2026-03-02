//! Face impact smoke test: connect to Mevo+, enable Fusion mode, and print
//! camera tracking results alongside radar data.
//!
//! Usage: cargo run --features gvp --example face_impact
//!
//! Requires: connected to Mevo+ WiFi (SSID = device serial).
//!
//! Architecture: **single-thread**, two non-blocking TCP connections.
//!   - **Binary connection** (port 5100): handshake, configure, arm, event loop.
//!   - **GVP connection** (port 1258): camera config, trigger, tracking results.
//!
//! Both connections use non-blocking I/O. The event loop polls both clients
//! in a tight loop, eliminating the need for threads and cross-thread signals.

use std::net::SocketAddr;
use std::process;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::conn::DEFAULT_ADDR;
use ironsight::gvp::client::GvpClient;
use ironsight::gvp::config::GvpConfig;
use ironsight::gvp::conn::DEFAULT_PORT;
use ironsight::gvp::track::ExpectedTrack;
use ironsight::gvp::trigger::Trigger;
use ironsight::gvp::{GvpConnection, GvpError, GvpEvent};
use ironsight::protocol::camera::{CamConfig, CamConfigReq, CamState};
use ironsight::protocol::config::{MODE_CHIPPING, ParamData, ParamValue, RadarCal};
use ironsight::protocol::shot::{ClubResult, FlightResult};
use ironsight::seq::{self, AvrSettings, ShotData};
use ironsight::{BinaryConnection, BusAddr, Command, ConnError, Message};

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

fn ms_to_mph(ms: f64) -> f64 {
    ms * 2.23694
}

fn m_to_in(m: f64) -> f64 {
    m * 39.3701
}

fn new_guid() -> String {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    format!(
        "{{{:08x}-{:04x}-{:04x}-{:04x}-{:012x}}}",
        (nanos >> 96) as u32,
        (nanos >> 80) as u16,
        (nanos >> 64) as u16,
        (nanos >> 48) as u16,
        nanos as u64 & 0xFFFF_FFFF_FFFF,
    )
}

fn epoch_now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}

// -------------------------------------------------------------------------
// Polynomial math (pure Rust, no dependencies)
// -------------------------------------------------------------------------

/// Evaluate polynomial `c[0] + c[1]*t + c[2]*t^2 + ...` using Horner's method.
fn eval_poly(coeffs: &[f64], t: f64) -> f64 {
    let mut result = 0.0;
    for &c in coeffs.iter().rev() {
        result = result * t + c;
    }
    result
}

/// Solve 5×5 linear system Ax=b via Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)] // index-based mutation is clearest for elimination
fn solve_5x5(a: &mut [[f64; 5]; 5], b: &mut [f64; 5]) -> [f64; 5] {
    // Forward elimination
    for col in 0..5 {
        // Partial pivoting: find row with largest absolute value in this column
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..5 {
            let val = a[row][col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        for row in (col + 1)..5 {
            let factor = a[row][col] / pivot;
            for j in col..5 {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = [0.0; 5];
    for i in (0..5).rev() {
        let mut sum = b[i];
        for j in (i + 1)..5 {
            sum -= a[i][j] * x[j];
        }
        x[i] = sum / a[i][i];
    }
    x
}

/// Fit 4th-order polynomial (5 coefficients) to (ts, ys) via least-squares.
fn poly_fit_4(ts: &[f64], ys: &[f64]) -> [f64; 5] {
    let mut vtv = [[0.0; 5]; 5];
    let mut vty = [0.0; 5];

    for k in 0..ts.len() {
        let t = ts[k];
        let y = ys[k];
        let mut ti = 1.0;
        for i in 0..5 {
            let mut tj = 1.0;
            for j in 0..5 {
                vtv[i][j] += ti * tj;
                tj *= t;
            }
            vty[i] += ti * y;
            ti *= t;
        }
    }

    solve_5x5(&mut vtv, &mut vty)
}

// -------------------------------------------------------------------------
// Trajectory hint computation (3D radar → 2D camera pixel space)
// -------------------------------------------------------------------------

const CAM_FX: f64 = 500.0;
const CAM_FY: f64 = 500.0;
const CAM_CX: f64 = 320.0;
const CAM_CY: f64 = 240.0;
const CAM_HEIGHT: f64 = 0.10;
const CAM_TILT_DEG: f64 = 12.5;
const BALL_RADIUS_M: f64 = 0.021;
const CLUB_RADIUS_M: f64 = 0.050;
const N_SAMPLES: usize = 21;
const BALL_DURATION: f64 = 0.1;
const DEFAULT_START_TIME: f64 = 0.014;

fn project_to_pixel(x: f64, y: f64, z: f64, object_radius: f64) -> (f64, f64, f64) {
    let theta = CAM_TILT_DEG.to_radians();
    let (sin_t, cos_t) = theta.sin_cos();
    let dx = x;
    let dy = y - CAM_HEIGHT;
    let x_cam = (dx * cos_t + dy * sin_t).max(0.1);
    let y_cam = -dx * sin_t + dy * cos_t;
    let u = CAM_FX * z / x_cam + CAM_CX;
    let v = CAM_CY - CAM_FY * y_cam / x_cam;
    let r = CAM_FX * object_radius / x_cam;
    (u, v, r)
}

fn fit_track_polynomials(
    poly_x: &[f64],
    poly_y: &[f64],
    poly_z: &[f64],
    range_m: f64,
    start_time: f64,
    duration: f64,
) -> ([f64; 5], [f64; 5]) {
    let mut ts = Vec::with_capacity(N_SAMPLES);
    let mut us = Vec::with_capacity(N_SAMPLES);
    let mut vs = Vec::with_capacity(N_SAMPLES);

    for i in 0..N_SAMPLES {
        let t_local = i as f64 * duration / (N_SAMPLES - 1) as f64;
        let t_real = start_time + t_local;
        let x = range_m + eval_poly(poly_x, t_real);
        let y = eval_poly(poly_y, t_real);
        let z = eval_poly(poly_z, t_real);
        let (u, v, _) = project_to_pixel(x, y, z, 0.0);

        ts.push(t_local);
        us.push(u);
        vs.push(v);
    }

    (poly_fit_4(&ts, &us), poly_fit_4(&ts, &vs))
}

fn make_poly_radius(
    poly_x: &[f64],
    object_radius: f64,
    range_m: f64,
    start_time: f64,
) -> [f64; 5] {
    let x0 = (range_m + eval_poly(poly_x, start_time)).max(0.1);
    let r0 = CAM_FX * object_radius / x0;
    [1.0, r0, r0 * 0.15, 0.0, 0.0]
}

struct TrajectoryHints {
    club_track: Option<ExpectedTrack>,
    ball_track: ExpectedTrack,
}

fn compute_trajectory_hints(
    flight: &FlightResult,
    club: Option<&ClubResult>,
    guid: &str,
    range_m: f64,
    prc_start_time: Option<f64>,
) -> TrajectoryHints {
    let start_time = prc_start_time.unwrap_or(DEFAULT_START_TIME);

    let x0 = range_m + eval_poly(&flight.poly_x, start_time);
    let y0 = eval_poly(&flight.poly_y, start_time);
    let z0 = eval_poly(&flight.poly_z, start_time);
    println!(
        "  [traj] poly_x={:?}\n  [traj] poly_y={:?}\n  [traj] poly_z={:?}",
        flight.poly_x, flight.poly_y, flight.poly_z,
    );
    println!(
        "  [traj] poly_scale={} range_m={:.3} startTime={:.6}s{} h={:.3}m tilt={:.1}°",
        flight.poly_scale,
        range_m,
        start_time,
        if prc_start_time.is_some() {
            " (prc)"
        } else {
            " (default)"
        },
        CAM_HEIGHT,
        CAM_TILT_DEG,
    );
    println!(
        "  [traj] abs@t0=({:.4}, {:.4}, {:.4}) → projected=({:.1}, {:.1})",
        x0,
        y0,
        z0,
        CAM_FX * z0 / x0.max(0.1) + CAM_CX,
        CAM_CY + CAM_FY * (CAM_HEIGHT - y0) / x0.max(0.1),
    );

    let (ball_poly_u, ball_poly_v) = fit_track_polynomials(
        &flight.poly_x,
        &flight.poly_y,
        &flight.poly_z,
        range_m,
        start_time,
        BALL_DURATION,
    );
    let ball_poly_r = make_poly_radius(&flight.poly_x, BALL_RADIUS_M, range_m, start_time);

    let ball_track = ExpectedTrack {
        guid: guid.to_string(),
        duration: BALL_DURATION,
        start_time,
        poly_u: ball_poly_u,
        poly_v: ball_poly_v,
        poly_radius: ball_poly_r,
    };

    let club_track = if let Some(club) = club {
        if club.dynamic_loft == 0.0 {
            None
        } else {
            let cx = &club.poly_coeffs[2];
            let cy = &club.poly_coeffs[4];
            let cz = &club.poly_coeffs[6];
            let club_px = [cx[0], cx[1], cx[2], 0.0, 0.0];
            let club_py = [cy[0], cy[1], cy[2], 0.0, 0.0];
            let club_pz = [cz[0], cz[1], cz[2], 0.0, 0.0];

            let (poly_u, poly_v) = fit_track_polynomials(
                &club_px, &club_py, &club_pz, range_m, start_time, BALL_DURATION,
            );
            let poly_r = make_poly_radius(&club_px, CLUB_RADIUS_M, range_m, start_time);

            Some(ExpectedTrack {
                guid: guid.to_string(),
                duration: BALL_DURATION,
                start_time,
                poly_u,
                poly_v,
                poly_radius: poly_r,
            })
        }
    } else {
        Some(ExpectedTrack {
            guid: guid.to_string(),
            duration: BALL_DURATION,
            start_time,
            poly_u: ball_poly_u,
            poly_v: ball_poly_v,
            poly_radius: make_poly_radius(
                &flight.poly_x,
                CLUB_RADIUS_M,
                range_m,
                start_time,
            ),
        })
    };

    println!(
        "  [traj] Ball: polyU[0]={:.1} polyV[0]={:.1} polyR=[1, {:.1}, {:.2}, 0, 0]",
        ball_poly_u[0], ball_poly_v[0], ball_poly_r[1], ball_poly_r[2],
    );
    if let Some(ref ct) = club_track {
        println!(
            "  [traj] Club: polyU[0]={:.1} polyV[0]={:.1} startTime={:.6}s{}",
            ct.poly_u[0],
            ct.poly_v[0],
            ct.start_time,
            if club.is_none() { " (dummy)" } else { "" },
        );
    }

    TrajectoryHints {
        club_track,
        ball_track,
    }
}

fn print_shot_summary(shot_num: u32, data: &ShotData) {
    let has_club_data = data
        .club
        .as_ref()
        .is_some_and(|c| c.dynamic_loft != 0.0);
    let is_full = data.flight.is_some() && data.spin.is_some() && has_club_data;

    let title = if is_full {
        format!("FULL SHOT RECORDED  (#{shot_num})")
    } else {
        let mut missing = Vec::new();
        if data.flight.is_none() {
            missing.push("flight");
        }
        if data.spin.is_none() {
            missing.push("spin");
        }
        if !has_club_data {
            missing.push("club");
        }
        format!("PARTIAL SHOT  (#{shot_num})  no {}", missing.join(", "))
    };

    println!();
    println!("  ╔══════════════════════════════════════════════╗");
    println!("  ║{title:^46}║");
    println!("  ╚══════════════════════════════════════════════╝");

    if let Some(ref f) = data.flight {
        println!();
        println!(
            "  Ball Speed   {:>7.1} mph",
            ms_to_mph(f.launch_speed),
        );
        if has_club_data {
            let c = data.club.as_ref().unwrap();
            println!(
                "  Club Speed   {:>7.1} mph     Smash  {:.2}",
                ms_to_mph(c.pre_club_speed),
                c.smash_factor,
            );
        }
        println!(
            "  Launch Angle {:>7.1}\u{00b0}       HLA    {:+.1}\u{00b0}",
            f.launch_elevation, f.launch_azimuth,
        );
        println!(
            "  Carry        {:>7.0} yd      Height {:.0} ft",
            f.carry_distance * 1.09361,
            f.max_height * 3.28084,
        );
    }

    println!("  ──────────────────────────────────────────────");

    if let Some(ref f) = data.flight {
        println!("  Backspin     {:>7} rpm", f.backspin_rpm);
        println!("  Sidespin     {:>7} rpm", f.sidespin_rpm);
    }
    if let Some(ref s) = data.spin {
        println!(
            "  Total Spin   {:>7} rpm     Axis   {:+.1}\u{00b0}",
            s.pm_spin_final, s.spin_axis,
        );
    }

    if has_club_data {
        let c = data.club.as_ref().unwrap();
        println!("  ──────────────────────────────────────────────");
        println!(
            "  Face Angle   {:>+7.1}\u{00b0}       Loft   {:.1}\u{00b0}",
            c.face_angle, c.dynamic_loft,
        );
        println!(
            "  Club Path    {:>+7.1}\u{00b0}       AoA    {:+.1}\u{00b0}",
            c.strike_direction, c.attack_angle,
        );
        println!("  Low Point    {:>+7.1} in", m_to_in(c.club_offset));
    }

    if !data.prc.is_empty() || !data.club_prc.is_empty() {
        let ball_pts: usize = data.prc.iter().map(|p| p.points.len()).sum();
        let club_pts: usize = data.club_prc.iter().map(|p| p.points.len()).sum();
        println!("  ──────────────────────────────────────────────");
        println!("  Radar PRC      ball={ball_pts} pts  club={club_pts} pts");
    }

    println!();
}

fn print_gvp_event(event: &GvpEvent) {
    match event {
        GvpEvent::Config(cfg) => {
            println!(
                "  [gvp] CONFIG: {}x{}",
                cfg.camera_configuration.roi_width,
                cfg.camera_configuration.roi_height,
            );
        }
        GvpEvent::Status(s) => {
            println!("  [gvp] STATUS: {}", s.status());
        }
        GvpEvent::Log(l) => {
            println!("  [gvp] LOG({}): {}", l.level, l.message);
        }
        GvpEvent::Result(r) => {
            println!("  [gvp] *** RESULT guid={} ***", r.guid);
            println!("  [gvp]   {} tracks:", r.tracks.len());
            for track in &r.tracks {
                let label = match track.track_id {
                    0 => "ball",
                    1 => "club",
                    _ => "ref",
                };
                println!(
                    "  [gvp]   trackId={} ({}) points={}",
                    track.track_id, label, track.len(),
                );
                if !track.is_empty() {
                    if track.track_id <= 1 {
                        for i in 0..track.len() {
                            println!(
                                "  [gvp]     [{:>2}] f={:<3} u={:>7.2} v={:>7.2} r={:>5.1} shutter={:.3}ms",
                                i, track.frame_number[i],
                                track.u[i], track.v[i], track.radius[i],
                                track.shutter_time_ms[i],
                            );
                        }
                    } else {
                        println!(
                            "  [gvp]     u={:.1} v={:.1} r={:.1}",
                            track.u[0], track.v[0], track.radius[0],
                        );
                    }
                }
            }
        }
        GvpEvent::VideoAvailable(v) => {
            println!("  [gvp] VIDEO: {}", v.absolute_path);
        }
        GvpEvent::Unknown { msg_type, .. } => {
            println!("  [gvp] {msg_type} (unknown)");
        }
    }
}

// -------------------------------------------------------------------------
// Camera startup helpers
// -------------------------------------------------------------------------

fn cam_state_poll(
    conn: &mut BinaryConnection<std::net::TcpStream>,
    state: u8,
) -> Result<u8, ConnError> {
    conn.send(&Command::CamState(CamState { state }), BusAddr::Pi)?;
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Err(ConnError::Timeout);
        }
        let env = conn.recv_timeout(remaining)?;
        if matches!(env.src, BusAddr::Pi) {
            if let Message::CamState(cs) = env.message {
                return Ok(cs.state);
            }
        }
    }
}

fn start_camera(
    conn: &mut BinaryConnection<std::net::TcpStream>,
    config: &CamConfig,
    label: &str,
) -> Result<(), ConnError> {
    let t = Duration::from_secs(2);

    conn.send(&Command::CamConfig(config.clone()), BusAddr::Pi)?;
    let _ = seq::recv_msg(conn, t)?;

    conn.send(&Command::CamConfigReq(CamConfigReq), BusAddr::Pi)?;
    let _ = seq::recv_msg(conn, t)?;

    let state = cam_state_poll(conn, 0x00)?;
    println!("  [{label}] stop → state=0x{state:02X}");

    for attempt in 1..=8 {
        thread::sleep(Duration::from_secs(5));
        let state = cam_state_poll(conn, 0x03)?;
        println!("  [{label}] poll {attempt}: state=0x{state:02X}");
        if state == 0x01 {
            println!("  [{label}] Camera ready.");
            return Ok(());
        }
    }
    Err(ConnError::Timeout)
}

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), ConnError> {
    let binary_addr: SocketAddr = DEFAULT_ADDR.parse().unwrap();
    let gvp_addr: SocketAddr = format!("{}:{DEFAULT_PORT}", binary_addr.ip()).parse().unwrap();

    // --- Blocking setup: handshake + configure + camera startup -----------
    // These require blocking send/recv and cannot go through BinaryClient.
    println!("Connecting to binary protocol at {binary_addr}...");
    let mut conn = BinaryConnection::connect_timeout(&binary_addr, Duration::from_secs(5))?;
    println!("Connected to binary protocol.");

    conn.set_on_send(|cmd, dest| println!("  >> {}", cmd.debug_hex(dest)));
    conn.set_on_recv(|env| {
        println!("  << {} 0x{:02X} {}B", env.src, env.type_id, env.raw.len());
    });

    println!("\nDSP sync...");
    let dsp = seq::sync_dsp(&mut conn)?;
    println!("AVR sync...");
    let avr = seq::sync_avr(&mut conn)?;
    println!("PI sync...");
    let pi = seq::sync_pi(&mut conn)?;

    println!("\n--- Handshake complete ---");
    println!("  DSP: {}", dsp.dev_info.text);
    println!("  AVR: {}", avr.dev_info.text);
    println!("  PI:  {}", pi.dev_info.text);
    println!("  Battery: {}%", dsp.status.battery_percent());

    let settings = AvrSettings {
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
        radar_cal: RadarCal {
            range_mm: 2134,
            height_mm: 25,
        },
    };

    println!("\nConfiguring AVR...");
    seq::configure_avr(&mut conn, &settings)?;

    println!("\nPhase A: Standard camera warmup...");
    start_camera(&mut conn, &CamConfig::standard_preset(), "std")?;

    let fusion_config = CamConfig::raw_fusion_preset();
    println!(
        "Phase B: Raw Fusion ({}x{}, raw_mode={}, fusion_camera_mode={})...",
        fusion_config.resolution_width,
        fusion_config.resolution_height,
        fusion_config.raw_camera_mode,
        fusion_config.fusion_camera_mode,
    );
    start_camera(&mut conn, &fusion_config, "fusion")?;
    println!("Camera ready.\n");

    // --- Switch to non-blocking clients for the event loop ---------------
    let mut binary = BinaryClient::from_tcp(conn)?;
    println!("Arming...");
    binary.arm();

    println!("[gvp] Connecting to {gvp_addr}...");
    let gvp_conn = GvpConnection::connect_timeout(&gvp_addr, Duration::from_secs(5))
        .map_err(|e| ConnError::Io(std::io::Error::other(format!("GVP: {e}"))))?;
    let mut gvp = GvpClient::from_tcp(gvp_conn)
        .map_err(|e| ConnError::Io(std::io::Error::other(format!("GVP: {e}"))))?;
    println!("[gvp] Connected.");

    let range_m = f64::from(settings.radar_cal.range_mm) / 1000.0;
    let mut shot_count = 0u32;
    let mut shot_guid = String::new();
    let mut pending_club: Option<ClubResult> = None;
    let mut hints_sent = false;
    let mut prc_start_time: Option<f64> = None;

    loop {
        // --- Poll binary client ---
        if let Some(event) = binary.poll()? {
            match event {
                BinaryEvent::Armed => {
                    println!("=== ARMED — hit a ball! ===\n");
                }

                BinaryEvent::Trigger => {
                    shot_count += 1;
                    shot_guid = new_guid();
                    pending_club = None;
                    hints_sent = false;
                    prc_start_time = None;
                    let epoch = epoch_now();
                    println!(
                        "\n=== BALL TRIGGER (shot #{shot_count}) guid={shot_guid} epoch={epoch:.6} ==="
                    );

                    // Fire trigger to GVP inline — no cross-thread signal needed.
                    println!("[gvp] Sending TRIGGER guid={shot_guid}");
                    let _ = gvp.send_trigger(&Trigger::new(shot_guid.clone(), epoch));
                    let _ = gvp.send_config(&GvpConfig::fusion());
                }

                BinaryEvent::Shot(data) => {
                    // Send trajectory hints if not already sent from pre-PROCESSED data.
                    if !hints_sent {
                        if let Some(ref flight) = data.flight {
                            // Extract prc_start_time from drain data if not captured earlier.
                            if prc_start_time.is_none() {
                                for prc in &data.prc {
                                    if !prc.points.is_empty() {
                                        prc_start_time =
                                            Some(f64::from(prc.points[0].time) * 26.7e-6);
                                        break;
                                    }
                                }
                            }
                            let hints = compute_trajectory_hints(
                                flight,
                                data.club.as_ref().or(pending_club.as_ref()),
                                &shot_guid,
                                range_m,
                                prc_start_time,
                            );
                            send_gvp_hints(&mut gvp, &hints);
                        }
                    }

                    print_shot_summary(shot_count, &data);
                    println!("=== Re-armed ===\n");
                }

                BinaryEvent::Message(env) => {
                    match env.message {
                        Message::FlightResult(ref r) => {
                            println!(
                                "  Flight (shot #{}): ball={:.1}mph club={:.1}mph VLA={:.1}° carry={:.0}yd",
                                r.total,
                                ms_to_mph(r.launch_speed),
                                ms_to_mph(r.clubhead_speed),
                                r.launch_elevation,
                                r.carry_distance * 1.09361,
                            );
                            println!(
                                "    face={:.1}° loft={:.1}° path={:.1}° AoA={:.1}°",
                                r.club_face_angle,
                                r.club_effective_loft,
                                r.club_strike_direction,
                                r.club_attack_angle,
                            );

                            let hints = compute_trajectory_hints(
                                r,
                                pending_club.as_ref(),
                                &shot_guid,
                                range_m,
                                prc_start_time,
                            );
                            send_gvp_hints(&mut gvp, &hints);
                            hints_sent = true;
                        }

                        Message::ClubResult(ref r) => {
                            println!(
                                "  Club: speed={:.1}mph smash={:.2} low_point={:.1}in pre_impact={:.3}ms",
                                ms_to_mph(r.pre_club_speed),
                                r.smash_factor,
                                m_to_in(r.club_offset),
                                r.pre_impact_time,
                            );
                            pending_club = Some(r.clone());
                        }

                        Message::SpinResult(ref r) => {
                            println!(
                                "  Spin: total={}rpm axis={:.1}°",
                                r.pm_spin_final, r.spin_axis
                            );
                        }

                        Message::PrcData(ref r) => {
                            if prc_start_time.is_none() && !r.points.is_empty() {
                                let t = f64::from(r.points[0].time) * 26.7e-6;
                                prc_start_time = Some(t);
                                println!(
                                    "  PRC: seq={} points={} startTime={:.6}s (tick={})",
                                    r.sequence,
                                    r.points.len(),
                                    t,
                                    r.points[0].time
                                );
                            } else {
                                println!("  PRC: seq={} points={}", r.sequence, r.points.len());
                            }
                        }

                        Message::ClubPrc(ref r) => {
                            println!("  Club PRC: points={}", r.points.len());
                        }

                        Message::CamImageAvail(ref r) => {
                            println!(
                                "  Camera: streaming={} fusion={} video={}",
                                r.streaming_available, r.fusion_available, r.video_available,
                            );
                        }

                        Message::FlightResultV1(ref r) => {
                            println!(
                                "  FlightV1: ball={:.1}mph VLA={:.1}° dist={:.0}yd",
                                ms_to_mph(r.ball_velocity),
                                r.elevation,
                                r.distance * 1.09361,
                            );
                        }

                        Message::Unknown { type_id, .. } => {
                            println!("  [unknown msg 0x{type_id:02X}]");
                        }
                        _ => {}
                    }
                }

                BinaryEvent::Configured | BinaryEvent::Handshake(_) => {}
            }
        }

        // --- Poll GVP client ---
        match gvp.poll() {
            Ok(Some(event)) => print_gvp_event(&event),
            Ok(None) => {}
            Err(GvpError::Disconnected) => {
                println!("[gvp] Disconnected.");
            }
            Err(e) => {
                eprintln!("[gvp] recv error: {e}");
            }
        }
    }
}

/// Send trajectory hints to GVP.
fn send_gvp_hints(
    gvp: &mut GvpClient<std::net::TcpStream>,
    hints: &TrajectoryHints,
) {
    if let Some(ref club_track) = hints.club_track {
        println!("[gvp] Sending EXPECTED_CLUB_TRACK");
        let _ = gvp.send_club_track(club_track);
    }
    println!("[gvp] Sending EXPECTED_TRACK");
    let _ = gvp.send_ball_track(&hints.ball_track);
}
