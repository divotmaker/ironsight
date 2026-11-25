//! Client library for the FlightScope Mevo+, Mevo Gen2, Xi and X2 binary
//! protocol (TCP port 5100).
//!
//! [`BinaryClient`] runs a device over a [`BinaryConnection`]: the
//! handshake, configuration, arming, keepalives and shot detection, through
//! one non-blocking [`poll()`](BinaryClient::poll) that returns
//! [`BinaryEvent`]s. It is generic over any `Read + Write` stream, needs no
//! async runtime, and its timers run on the time it is given
//! ([`poll_at`](BinaryClient::poll_at)). [`protocol`] decodes and encodes
//! the messages.
//!
//! ```no_run
//! use ironsight::protocol::handshake::DeviceModel;
//! use ironsight::{BinaryClient, BinaryConnection, BinaryEvent, DEFAULT_ADDR, XiSettings};
//!
//! let conn = BinaryConnection::connect(DEFAULT_ADDR)?;
//! let mut client = BinaryClient::from_tcp(conn)?;
//! client.handshake();
//! loop {
//!     match client.poll()? {
//!         Some(BinaryEvent::Handshake(h)) if h.model == DeviceModel::Xi => {
//!             client.configure_xi(XiSettings::default());
//!             client.arm();
//!         }
//!         Some(BinaryEvent::ShotComplete(shot)) => println!("{:?}", shot.flight),
//!         Some(_) | None => {}
//!     }
//! }
//! # Ok::<(), ironsight::ConnError>(())
//! ```
//!
//! # Features
//!
//! None is on by default.
//!
//! - `serde`: `Serialize`/`Deserialize` on the decoded shot results (flight,
//!   early flight, club and spin results).
//! - `gvp`: `ironsight::gvp`, a client for the camera processor's JSON protocol (TCP
//!   port 1258). Work in progress: `gvp` is unstable and outside the semver promise; any
//!   release may change it.
//! - `config`: `ironsight::config`, the FRP driver's settings (serde types only, no
//!   device stack; builds for WebAssembly).
//! - `driver`: `ironsight::driver`, the [flightrelay](https://docs.rs/flightrelay)
//!   driver for FlightScope devices (implies `config`).
//! - `frp`: the `ironsight-frp` binary, the driver served over FRP (implies
//!   `driver`).
//! - `testing`: `ironsight::testing`, a scripted in-memory device, recorded result
//!   frames and payload encoders, for tests.

mod addr;
mod client;
mod codec;
#[cfg(feature = "config")]
pub mod config;
mod conn;
#[cfg(feature = "driver")]
pub mod driver;
mod error;
mod frame;
#[cfg(feature = "gvp")]
pub mod gvp;
pub mod protocol;
mod seq;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

pub use addr::BusAddr;
pub use client::{
    BinaryClient, BinaryEvent, CAMERA_SETUP_TIMEOUT, HandshakeOutcome, StatusSnapshot,
};
#[cfg(feature = "config")]
pub use config::Config;
pub use conn::{BinaryConnection, ConnError, DEFAULT_ADDR, Envelope};
pub use error::WireError;
/// The flightrelay SDK this crate was built with, so callers name its types at the same version.
#[cfg(feature = "config")]
pub use flightrelay;
pub use frame::{FrameSplitter, RawFrame};
pub use protocol::{Command, Message};
pub use seq::{
    AvrSettings, AvrSync, DspSync, MevoSettings, PiSync, ShotData, ShotDatum, XiSettings,
    XiShotType,
};
