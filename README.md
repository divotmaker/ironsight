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
- **`FrpDevice`** — [Flight Relay Protocol](https://github.com/flightrelay/spec)
  device (`frp` feature). Bridges `BinaryClient` shot data to an FRP controller
  over WebSocket, either serving controllers on a local port (default 5880) or
  dialing a central controller such as flighthook.

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
SDK for third-party integration. The protocol was reverse-engineered from
publicly broadcast WiFi traffic between the device and the official FS Golf
app, solely to enable interoperability with third-party golf simulation
software.

No FlightScope code is reproduced here. No access controls were circumvented —
the device operates as an open WiFi access point, and all traffic was captured
from the researcher's own lawfully purchased hardware.

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
- ironsight does not rely on the DSP query (`0x48`): a response from the Xi
  has not been observed, and without one the handshake moves on after
  `DSP_QUERY_TIMEOUT` (2 s), leaving `DspSync::hw_info` as `None`.
- The AVR sync stops after the IF calibration exchange (no factory
  calibration, hardware config query, param `0x64` or time sync).
- `AvrSettings::xi()` configures detection mode 9 (`MODE_LIMITED_FLIGHT`, for
  nets and simulator screens) with no parameter writes or radar calibration,
  so the Xi keeps the values stored on it. `AvrSettings::xi_with_mode(mode)`
  selects another mode, e.g. `MODE_FULL_OUTDOOR` (8) for full-flight outdoor
  use; only the mode is written, not the extra params FS Golf sends with it.
- Shot results (`E8`, `D4`, `ED`, `EF`) decode with the Mevo+ layouts.

It connects, arms and reports shots, and has been verified against
FlightScope's own FS Golf app on a real Xi. Only limited flight (9) has been
verified this way; full-flight outdoor (8) is untested, and putting/chipping
modes have not been mapped for the Xi yet. See [SEQUENCE.md](docs/SEQUENCE.md#8-flightscope-xi).

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
use ironsight::seq::AvrSettings;
use ironsight::BinaryConnection;

let conn = BinaryConnection::connect("192.168.2.1:5100")?;
let mut client = BinaryClient::from_tcp(conn)?;

client.handshake();

loop {
    match client.poll()? {
        Some(BinaryEvent::Handshake(h)) => {
            println!("Connected to {}", h.model);
            if h.model == DeviceModel::Xi {
                client.configure_avr(AvrSettings::xi());
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

### FRP device server

The `frp` feature adds an FRP device server that makes the Mevo+ appear as a
standard [FlightRelay](https://github.com/flightrelay/spec) device. Any FRP
controller (flighthook, dashboards, etc.) can connect over WebSocket and receive
shot data without knowing anything about the Mevo+ binary protocol.

A standalone binary is included:

```sh
# Connect to Mevo+ at default address, serve FRP on port 5880
cargo run --features frp --bin ironsight-frp

# Custom addresses
cargo run --features frp --bin ironsight-frp -- 192.168.2.1:5100 0.0.0.0:5880
```

Pre-built binaries for Linux and Windows are attached to each
[GitHub release](https://github.com/divotmaker/ironsight/releases).

## Dependencies

Minimal by design:

- **`thiserror`** — error enum derives
- **`serde`** (optional, `serde` feature) — serialization support
- **`serde_json`** (optional, `gvp` feature) — camera protocol support
- **`flightrelay`** (optional, `frp` feature) — FRP WebSocket server

No async runtime. No logging framework. Standard library TCP only.
