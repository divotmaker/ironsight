//! ironsight-frp — Flight Relay Protocol device for the FlightScope Mevo+,
//! Mevo Gen2 and Xi.
//!
//! ```text
//! ironsight-frp [--config <path>] [--target <listen-addr | ws://host:port/frp>]
//!               [--set <key>=<value>]... [--log <info|debug|trace>] [--help]
//! ```
//!
//! Connects to the device (`--set address=<ip:port>`, default
//! `192.168.2.1:5100`), configures and arms it, and serves its shots to FRP
//! controllers on `0.0.0.0:5880`, or bridges to the controller at a `ws://`
//! target. `--help` lists every setting with its default.

use std::process::ExitCode;

fn main() -> ExitCode {
    flightrelay::run::<ironsight::driver::Mevo>()
}
