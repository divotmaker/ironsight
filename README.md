# ironsight

[![CI](https://github.com/divotmaker/ironsight/actions/workflows/ci.yml/badge.svg)](https://github.com/divotmaker/ironsight/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/ironsight.svg)](https://crates.io/crates/ironsight)
[![docs.rs](https://docs.rs/ironsight/badge.svg)](https://docs.rs/ironsight)

## Disclaimer

This project is not affiliated with or endorsed by FlightScope (Pty) Ltd.
FlightScope and Mevo are trademarks of their respective owner.

## Description

Client library for the FlightScope Mevo+ / Mevo Gen2 / Xi protocol. Two
non-blocking clients share the same `poll()` pattern:

- **`BinaryClient`** — binary radar protocol (TCP port 5100). Handles
  handshake, configuration, arming, keepalives, and shot detection.
- **`GvpClient`** — GVP camera protocol (TCP port 1258, `gvp` feature). Handles
  trigger, trajectory hints, and ball tracker results.
- **`driver::Mevo`** — a device driver for the
  [Flight Relay Protocol](https://github.com/flightrelay/spec) SDK's driver
  contract (`driver` feature): connect, configure, arm, reconnect and report
  shots as FRP events. The `ironsight-frp` binary (`frp` feature) serves it to
  FRP controllers over WebSocket.

Both clients are generic over any `Read + Write` stream. Blocking convenience
wrappers and low-level sequencer state machines are also available. No async
runtime or heavyweight dependencies.

## Legal Basis — DMCA Section 1201(f)

This project is an exercise of the interoperability exception under
[17 U.S.C. § 1201(f)](https://www.law.cornell.edu/uscode/text/17/1201):

> (f) Reverse Engineering.—
>
> (1) Notwithstanding the provisions of subsection (a)(1)(A), a person who has
> lawfully obtained the right to use a copy of a computer program may
> circumvent a technological measure that effectively controls access to a
> particular portion of that program for the sole purpose of identifying and
> analyzing those elements of the program that are necessary to achieve
> interoperability of an independently created computer program with other
> programs, and that have not previously been readily made available to the
> person engaging in the circumvention, to the extent any such acts of
> identification and analysis do not constitute infringement under this title.
>
> (2) Notwithstanding the provisions of subsections (a)(2) and (b), a person
> may develop and employ technological means to circumvent a technological
> measure, or to circumvent protection afforded by a technological measure, in
> order to enable the identification and analysis described in paragraph (1),
> or for the purpose of enabling interoperability of an independently created
> computer program with other programs, if such means are necessary to achieve
> such interoperability, to the extent that doing so does not constitute
> infringement under this title.

The FlightScope Mevo+ uses a proprietary binary protocol over TCP to
communicate shot data (ball speed, launch angle, spin, club data, etc.) to
companion software. FlightScope does not publish this protocol or provide an
SDK for third-party integration. The protocol was worked out from WiFi traffic
between the researcher's own lawfully purchased device and the official FS Golf
app, and from analysis of that app, solely to enable interoperability with
third-party golf simulation software.

No FlightScope code is reproduced here; the protocol is described and
implemented in our own words and names. No access controls were circumvented —
the device operates as an open WiFi access point, and all traffic was captured
from the researcher's own hardware.

## Acceptable Use

This project exists solely to enable interoperability between the FlightScope
Mevo+ / Mevo Gen2 and third-party golf simulation software.

**It must not be used to:**

- Circumvent licensing or subscription requirements on FlightScope products
- Unlock paid features (ProPackage, Fusion Tracking, etc.) without purchase
- Bypass any access controls on FlightScope software or services

The sensor activation exchange (`0x90`/`0x89`) is documented only to the extent
needed for device communication. The encrypted licensing certificate is opaque
to this library and is not decoded, forged, or tampered with.

Issues or discussions proposing circumvention of licensing will be closed and
the user blocked.

## License

Licensed under either of:

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)

at your option.

## Status

Pre-1.0 (`0.x`). The protocol is reverse-engineered and still being mapped —
some message types and modes aren't decoded yet, and the API may change as
that work continues. This crate follows [SemVer](https://semver.org/): while
the version is `0.x`, any minor release may contain breaking changes. Once the
API stabilizes it will move to `1.0`.

## Supported Devices

| Device       | SSID            | Status          |
| ------------ | --------------- | --------------- |
| Mevo+ (Gen1) | `FS M2-XXXXXX`  | Fully supported |
| Mevo Gen2    | `FS MG2-XXXXXX` | Fully supported |
| Xi           | &mdash;         | **Supported**   |

The model family (`DeviceModel`) is detected during the handshake from the DSP
device info (`0xE7`): the Xi names itself `FS Xi` there. Mevo generation is
detected from the `0xC8` DSP query response (`dspType`: `0x80` = Mevo+,
`0xC0` = Gen2).

### FlightScope Xi

The Xi speaks the same protocol on the same address and port
(`192.168.2.1:5100`), with the same framing and bus addresses. It has no camera
Pi, so `BinaryClient` adapts once the handshake has identified it:

- The PI handshake phase is skipped; `HandshakeOutcome::pi` is `None`.
- Nothing is ever sent to the PI bus address: no PI status polls in arming or
  keepalives, and `configure_cam()` sends nothing (it reports `Configured`
  straight away).
- ironsight does not rely on the DSP query (`0x48`): the Xi's DSP may not
  answer it, and without an answer the handshake moves on after
  `DSP_QUERY_TIMEOUT` (2 s), leaving `DspSync::hw_info` as `None`.
- The DSP sync ends with a power-mode write to the DSP,
  `BF [06 00 00 01 00 00 01]` (DSP param `0x01` = 1, Normal; 4 is Standby),
  answered by `0x95 [02 40 3F]`. It wakes a DSP that was left in Standby,
  and changes nothing on one already in Normal. With no answer within
  `DSP_QUERY_TIMEOUT` (2 s) the handshake carries on and sets
  `DspSync::power_mode_unanswered`.
- The AVR sync stops after the IF calibration exchange (no factory
  calibration, hardware config query, param `0x64` or time sync). For a few
  seconds after power-on the Xi's AVR refuses status polls (`0x94
  [02 30 2A]`); the handshake re-sends the poll every
  `STATUS_REPOLL_INTERVAL` (1 s) until the AVR answers. This applies to every
  model, and changes nothing when the AVR answers the first poll.
- `BinaryClient::configure_xi(XiSettings)` sends the Xi its full parameter
  set, radar calibration and mode on the connection's first configuration
  (the connect-time block, with the radar calibration from
  `XiSettings::radar_cal`, followed by the mode block), then the mode block
  alone on a later call, disarming first if the Xi is armed. The mode block
  carries the detection mode and the shot type. Build the settings with
  `XiSettings::full_swing(mode, radar_cal)`, in `MODE_LIMITED_FLIGHT` (9) for
  nets and simulator screens or `MODE_FULL_OUTDOOR` (8), or
  `XiSettings::pitching(radar_cal)` for chips and pitches, which always runs
  in mode 9. `XiSettings::default()` is full swing in mode 9 with 2438 mm to
  the ball and a 25 mm surface height. Each frame waits for its reply. A `0x94` refusing a frame (or the DSP
  power-mode write) ends that step too: `BinaryClient` reports it as
  `BinaryEvent::StepRefused`, carrying the `0x94` frame, and carries on.
- `AvrSettings::xi()` and `AvrSettings::xi_with_mode(mode)` are deprecated:
  they send the detection mode only, with no parameter writes or radar
  calibration.
- Shot results (`E8`, `D4`, `ED`, `EF`) decode with the Mevo+ layouts.
  `BinaryClient` acks every `E8` and `D4` with `0x95` the moment it arrives
  (`[02 10 68]` / `[02 10 54]`); `seq::result_ack` builds the ack for callers
  that drive the sequencers themselves.

It connects, arms and reports shots on a real Xi. Full swing and pitching in
mode 9 are tested; mode 8 sends the same frames with only the mode changed,
and is untested. ironsight has no putting configuration for the Xi. See
[SEQUENCE.md](docs/SEQUENCE.md#8-flightscope-xi).

### Mevo+ and Mevo Gen2 configuration

`seq::MevoSettings` holds a full configuration: detection mode, ball type
(param `0x06`), minimum track percentage as a fraction (written to param
`0x0F`, whose exact use by the device is not yet confirmed), tee height in
meters (param `0x26`) and the radar calibration. Build it with
`MevoSettings::new`. It converts into
the `AvrSettings` that `BinaryClient::configure_avr` takes. For a mode change,
send `AvrSettings::mode_only(mode)`: the device re-arms sooner after the mode
alone than after a full parameter set.

Send `CamConfig::warmup_preset()` when the session starts. The camera accepts
a Fusion configuration (`CamConfig::fusion_preset()`,
`CamConfig::raw_fusion_preset()`) only after it has run in standard mode for
`camera::WARMUP` (15 s).

### Sign convention

Shot results report directions as the device measures them: azimuth, lateral
distances, sidespin, club path, face angle and swing plane direction are
positive to the **left** of the target line and negative to the right. Positive
sidespin curves the ball left. The usual convention in golf software (and in
FRP) is positive to the right, so negate these when converting; the driver's
conversions (`driver::convert`) do. See the `protocol::shot` docs.

## Protocol Documentation

Detailed specs live in [`docs/`](docs/):

- **[WIRE.md](docs/WIRE.md)** — Frame format, byte stuffing, checksum, field
  encodings, and the complete message catalog (43 decoded message types).
- **[SEQUENCE.md](docs/SEQUENCE.md)** — Connection lifecycle, six handshake
  phases with timing, shot-to-shot message flow, re-arm logic.
- **[CAMERA.md](docs/CAMERA.md)** — Port 1258 JSON protocol (GVP camera API),
  port 8080 MJPEG stream, per-shot video pipeline.

## Quick Start

See [`examples/client.rs`](examples/client.rs) for a complete standalone
example: connect, handshake, configure, arm, and poll for shots.

`BinaryClient` wraps the connection and all protocol state machines behind a
single `poll()` call. Keepalives, operation sequencing, and shot detection are
handled automatically:

```rust
use ironsight::client::{BinaryClient, BinaryEvent};
use ironsight::protocol::handshake::DeviceModel;
use ironsight::seq::XiSettings;
use ironsight::BinaryConnection;

let conn = BinaryConnection::connect("192.168.2.1:5100")?;
let mut client = BinaryClient::from_tcp(conn)?;

client.handshake();

loop {
    match client.poll()? {
        Some(BinaryEvent::Handshake(h)) => {
            println!("Connected to {}", h.model);
            if h.model == DeviceModel::Xi {
                client.configure_xi(XiSettings::default());
            } else {
                client.configure_avr(avr_settings.clone());
                client.configure_cam(cam_config.clone());
            }
            client.arm();
        }
        Some(BinaryEvent::Armed) => println!("Ready for shots"),
        Some(BinaryEvent::ShotComplete(data)) => println!("Shot! {data:?}"),
        _ => {}
    }
}
```

### Low-level sequencer API

For callers that need full control over the event loop, the `Sequence` trait
exposes the underlying pollable state machines. Each sequencer has `feed()`
(accept a message, return commands to send) and `is_complete()`. The `drive()`
function runs any sequencer to completion on a blocking stream. See
[`examples/event_loop.rs`](examples/event_loop.rs).

### Device driver and FRP binary

Three features build on the [flightrelay](https://crates.io/crates/flightrelay)
SDK (0.5, FRP 0.2.0):

- **`config`**: `ironsight::config::Config`, the driver's settings as a serde
  struct (flightrelay's `DriverConfig`), with `validate()` and the mapping to
  `MevoSettings` / `XiSettings`. It needs no device stack and builds for
  `wasm32-unknown-unknown`, so a settings form can use it.
- **`driver`**: `ironsight::driver::Mevo`, flightrelay's `DeviceDriver` for the
  Mevo+, Gen2 and Xi, and `driver::convert` (shot results to FRP types).
- **`frp`**: the `ironsight-frp` binary.

The driver configures the device for the detected model in the detection mode
the host has latched, and re-arms it when a controller's
`set_detection_mode` changes what is sent to the device (on the Xi, chipping
and putting are one session). Full swing runs in mode 9 (limited flight) or 8
(`environment = "outdoor"`); on the Xi, chipping and putting use the pitching
shot type. The radar has no club or handedness setting.

What it sends:

- **Identity:** manufacturer `FlightScope`, model `Mevo+`, `Mevo Gen2` or
  `Xi`, firmware from the AVR device info (version and build date). The FRP
  `device` id is the Mevo's WiFi SSID, or `FlightScope Xi (<address>)`
  (`FlightScope Mevo (<address>)` for a Mevo that reports no SSID). No
  `label` is set, so the card text is manufacturer and model
  (`FlightScope Xi`).
- **Telemetry:** `ready`, `tilt`, `roll`, and on the Mevo+ `battery_pct`,
  `temp_c` and `external_power` (the Gen2's and Xi's status layout has no
  decoded values for them, so they are left out).
- **Shots:** `shot_trigger` with a timestamp, then `launch` and
  `flight_result` (`basis: modeled`: carry, offline, apex, flight time,
  descent, and total and roll when the device reports them) as soon as the
  full result (`D4`) arrives, `club_path` once the shot has a ball, and
  `shot_finished` when the device is done. Each goes out once.
  - A launch-only full result (no spin and no flight, launched at 5° or more,
    as the Xi sends for very short chips) sends a `launch` without spin and no
    `flight_result`. Below 5° the same result is a ball that stayed on the
    ground, and goes out as measured (apex 0).
  - Without a full result, the early result (`E8`) makes the shot when
    `use_estimated = true`; a shot the device did not read raises an alert
    and sends no ball.
  - Xi only, `spin_source`: `d4` (the default, the spin FS Golf displays),
    `e8` (the early result's total spin, split by the full result's axis when
    the shot has club data, otherwise sent alone with no axis) or `auto`
    (`e8` in full swing). It applies without a new session.
- **Raw frames** when a controller asks for them, once the handshake is
  done: one message each, the payload as `data` and `"<type> <src>><dst>"`
  as `label` (e.g. `"D4 AVR>APP"`). The handshake's messages, those that can
  identify the unit, its network or the host (`protocol::carries_identity`),
  and the device's free text (`0xE3`) and DSP debug output (`0xF0`) are never
  sent; they go to the host's log as hex trace lines.

A message the driver cannot decode raises an alert naming the error (never
the message's bytes, which go to the trace log) and the session goes on.
A closed connection, an operation timeout or 10 s without a message ends the
session, and the host reconnects (1 s, growing by 1 s per failed attempt to
15 s, back to 1 s after a session that lasted 10 s).

#### `ironsight-frp`

```text
ironsight-frp [--config <path>] [--target <listen-addr | ws://host:port/frp>]
              [--set <key>=<value>]... [--log <info|debug|trace>] [--help]
```

- `--config`: a TOML file with the settings below at the top level.
- `--set key=value`: one setting, over the file; repeatable.
- `--target`: an address to listen on for controllers (default
  `0.0.0.0:5880`; several controllers at once), or a `ws://` URL of a
  controller to dial, such as flighthook.
- `--log`: the lowest level of the driver's log lines on stderr (default
  `info`).
- `--help`: usage and every setting with its default.

| Key | Default | |
|---|---|---|
| `address` | `192.168.2.1:5100` | the device's IP address and port |
| `environment` | `limited_flight` | or `outdoor` (full flight, about 40 yd needed) |
| `range` | `8ft` | distance from the device to the ball |
| `surface_height` | `0in` | height of the hitting surface |
| `tee_height` | `1.5in` | Mevo+ and Gen2 |
| `ball_type` | `1` | Mevo+ and Gen2: 0 = standard, 1 = RCT |
| `track_pct` | `80` | Mevo+ and Gen2, 0 to 100 |
| `camera_mode` | `standard` | Mevo+ and Gen2: `fusion` or `raw_fusion` (Pro Package) |
| `use_estimated` | `false` | send a shot from the early result when the full one is missing |
| `spin_source` | `d4` | Xi: `e8` or `auto` |

```sh
# Mevo+ at the default address, serve controllers on port 5880
cargo run --features frp --bin ironsight-frp

# Another address, bridged to flighthook
cargo run --features frp --bin ironsight-frp -- \
    --set address=10.0.0.20:5100 --target ws://flighthook:5880/frp
```

Pre-built binaries for Linux and Windows are attached to each
[GitHub release](https://github.com/divotmaker/ironsight/releases).

### Testing support

The `testing` feature (off by default) adds `ironsight::testing` for the
tests of applications built on ironsight:

- `FakeDevice`: a scripted in-memory device (`Read + Write`) to hand to
  `BinaryConnection::new`. It answers status polls, the arm and the
  shot-completion requests itself; `FakeDevice::xi()` also answers the Xi's
  handshake and configuration, and `FakeDevice::mevo_plus()` /
  `FakeDevice::mevo_gen2()` the Mevo handshake, AVR and camera
  configuration (`with_ssid` sets the camera Pi's WiFi name,
  `set_dsp_status` the status it reports).
- `Shot`: one shot's frames (`Shot::xi_full_swing()`, `Shot::xi_chip()`,
  `Shot::xi_no_read()`, or your own results with
  `Shot::new`), queued with `FakeDevice::push_shot`.
- `fixtures`: result frames as an Xi sent them, none of which identifies a
  unit.
- Payload encoders for test frames: `avr_status`, `dev_info`, `shot_text`,
  `mevo_plus_dsp_status`.
- With the `driver` feature, `driver::Mevo::connect_with` runs a whole
  driver session (handshake, configuration, shots) on any of these fakes.

```toml
[dev-dependencies]
ironsight = { version = "0.5.1", features = ["testing"] }
```

## Dependencies

Minimal by design:

- **`thiserror`** — error enum derives
- **`serde`** (optional, `serde` feature) — serialization support
- **`serde_json`** (optional, `gvp` feature) — camera protocol support
- **`flightrelay`** (optional, `config`, `driver` and `frp` features) — FRP
  driver contract, settings and server

No async runtime. No logging framework. Standard library TCP only.
