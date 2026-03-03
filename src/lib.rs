pub mod addr;
pub mod client;
pub mod codec;
pub mod conn;
pub mod error;
pub mod frame;
#[cfg(feature = "gvp")]
pub mod gvp;
pub mod protocol;
pub mod seq;

pub use addr::BusAddr;
pub use client::{BinaryClient, BinaryEvent, HandshakeOutcome, StatusSnapshot};
pub use conn::{BinaryConnection, ConnError, Connection, Envelope};
pub use error::WireError;
pub use frame::{FrameSplitter, RawFrame};
pub use protocol::{Command, Message};
pub use seq::{
    Action, ArmSequencer, AvrConfigSequencer, AvrSequencer, CameraConfigSequencer,
    DisarmSequencer, DspSequencer, PiSequencer, Sequence, ShotSequencer,
};
