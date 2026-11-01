pub mod addr;
pub mod client;
pub mod codec;
pub mod conn;
pub mod error;
pub mod frame;
#[cfg(feature = "frp")]
pub mod frp;
#[cfg(feature = "gvp")]
pub mod gvp;
pub mod protocol;
pub mod seq;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

pub use addr::BusAddr;
pub use client::{BinaryClient, BinaryEvent, HandshakeOutcome, StatusSnapshot};
pub use conn::{BinaryConnection, ConnError, Connection, Envelope};
pub use error::WireError;
pub use frame::{FrameSplitter, RawFrame};
pub use protocol::{Command, Message};
pub use seq::{
    Action, ArmSequencer, AvrConfigSequencer, AvrSequencer, CameraConfigSequencer, DisarmSequencer,
    DspSequencer, MevoSettings, PiSequencer, Sequence, ShotDatum, ShotSequencer, XiConfigSequencer,
    XiSettings, XiShotType,
};
