pub mod addr;
pub mod client;
pub mod codec;
#[cfg(feature = "config")]
pub mod config;
pub mod conn;
#[cfg(feature = "driver")]
pub mod driver;
pub mod error;
pub mod frame;
#[cfg(feature = "gvp")]
pub mod gvp;
pub mod protocol;
pub mod seq;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

pub use addr::BusAddr;
pub use client::{BinaryClient, BinaryEvent, HandshakeOutcome, StatusSnapshot};
pub use conn::{BinaryConnection, ConnError, Envelope};
pub use error::WireError;
pub use frame::{FrameSplitter, RawFrame};
pub use protocol::{Command, Message};
pub use seq::{MevoSettings, ShotDatum, XiSettings, XiShotType};
