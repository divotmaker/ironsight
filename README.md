# ironsight

## Disclaimer

This project is not affiliated with or endorsed by FlightScope (Pty) Ltd.
FlightScope and Mevo are trademarks of their respective owner.

## Description

Client library for the FlightScope Mevo+ / Mevo Gen2 binary protocol (TCP port
5100). Handles wire framing, device handshake, keepalive, arming, and shot result
parsing. The connection type is generic over any `Read + Write` stream, and
`recv()` is non-blocking — callers own the event loop and drive all I/O. Blocking
convenience wrappers are provided for simple use cases. No async runtime or
heavyweight dependencies.

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

See [`examples/event_loop.rs`](examples/event_loop.rs) for a complete
standalone example: connect, handshake, arm, poll for shots, and print results.

The core pattern is a non-blocking event loop. `BinaryConnection<S>` is generic
over any `Read + Write` stream, and `recv()` returns `Ok(None)` when no data is
available:

```rust
use ironsight::{BinaryConnection, Sequence};
use ironsight::seq::{self, ShotSequencer};

// Handshake (blocking via drive())
let dsp = seq::sync_dsp(&mut conn)?;
let avr = seq::sync_avr(&mut conn)?;
let pi = seq::sync_pi(&mut conn)?;

// Non-blocking event loop
conn.stream_mut().set_read_timeout(Some(Duration::from_millis(1)))?;
let mut shot: Option<ShotSequencer> = None;

loop {
    if let Some(env) = conn.recv()? {
        // Feed active shot sequencer
        if let Some(ref mut s) = shot {
            for a in s.feed(&env) { seq::send_action(&mut conn, a)?; }
            if s.is_complete() { /* extract result */ }
        }
        // Start new shot on "PROCESSED"
        if let Message::ShotText(st) = &env.message {
            if st.is_processed() {
                let (s, actions) = ShotSequencer::new();
                for a in actions { seq::send_action(&mut conn, a)?; }
                shot = Some(s);
            }
        }
    }
    // Keepalive on timer
    for a in seq::keepalive_actions() { seq::send_action(&mut conn, a)?; }
}
```

Protocol sequences are pollable state machines implementing the `Sequence` trait.
Each has `feed()` (accept a message, return commands to send) and `is_complete()`.
The `drive()` function runs any sequencer to completion on a blocking stream.

## Dependencies

Minimal by design:

- **`thiserror`** — error enum derives
- **`serde`** (optional, behind `serde` feature) — serialization support
- **`serde_json`** (optional, behind `gvp` feature) — camera protocol support

No async runtime. No logging framework. Standard library TCP only.
