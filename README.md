# ironsight

## Disclaimer

This project is not affiliated with or endorsed by FlightScope (Pty) Ltd.
FlightScope and Mevo are trademarks of their respective owner.

## Description

Client library for the FlightScope Mevo+ / Mevo Gen2 protocol. Two non-blocking
clients share the same `poll()` pattern:

- **`BinaryClient`** — binary radar protocol (TCP port 5100). Handles
  handshake, configuration, arming, keepalives, and shot detection.
- **`GvpClient`** — GVP camera protocol (TCP port 1258, `gvp` feature). Handles
  trigger, trajectory hints, and ball tracker results.
- **`FrpServer`** — [Flight Relay Protocol](https://github.com/flightrelay/spec)
  device server (`frp` feature). Bridges `BinaryClient` shot data to any FRP
  controller over WebSocket (port 5880).

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

**Alpha** (`0.0.x`). The protocol is reverse-engineered and still being mapped —
the API will change as new message types are decoded and existing ones are
refined. This crate follows [SemVer](https://semver.org/): while the version is
`0.0.x`, any release may contain breaking changes. Once the API stabilizes it
will move to `0.1.0`, after which breaking changes will bump the minor version
until `1.0`.

## Supported Devices

| Device       | SSID            | Status          |
| ------------ | --------------- | --------------- |
| Mevo+ (Gen1) | `FS M2-XXXXXX`  | Fully supported |
| Mevo Gen2    | `FS MG2-XXXXXX` | Fully supported |

Device generation is detected at connection time via the `0xC8` DSP query
response (`dspType`: `0x80` = Mevo+, `0xC0` = Gen2).

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
use ironsight::BinaryConnection;

let conn = BinaryConnection::connect("192.168.2.1:5100")?;
let mut client = BinaryClient::from_tcp(conn)?;

client.handshake();
client.configure_avr(avr_settings);
client.configure_cam(cam_config);
client.arm();

loop {
    match client.poll()? {
        Some(BinaryEvent::Handshake(h)) => println!("SSID: {}", h.pi.ssid),
        Some(BinaryEvent::Armed) => println!("Ready for shots"),
        Some(BinaryEvent::Shot(data)) => println!("Shot! {data:?}"),
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
