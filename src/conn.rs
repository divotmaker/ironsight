//! TCP connection to a Mevo+ device on port 5100.
//!
//! Handles TCP I/O, frame splitting, and message encode/decode.
//! No application logic — callers drive timing and sequencing.

use std::fmt;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::addr::BusAddr;
use crate::error::WireError;
use crate::frame::{FrameSplitter, RawFrame};
use crate::protocol::{Command, Message};

/// Default device address and port.
pub const DEFAULT_ADDR: &str = "192.168.2.1:5100";

/// A decoded device message with its source bus address and raw payload.
#[derive(Clone)]
pub struct Envelope {
    pub src: BusAddr,
    /// Wire message type ID (e.g. 0xD4).
    pub type_id: u8,
    /// Unstuffed payload bytes (before decode).
    pub raw: Vec<u8>,
    pub message: Message,
}

impl fmt::Debug for Envelope {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // "FlightResult { total: 3, ... } [AVR 0xD4 158B | 9E 00 03 ...]"
        write!(f, "{:?}", self.message)?;
        write!(f, " [{} 0x{:02X} {}B", self.src, self.type_id, self.raw.len())?;
        if !self.raw.is_empty() {
            write!(f, " | ")?;
            for b in self.raw.iter() {
                write!(f, "{b:02X}")?;
            }
        }
        write!(f, "]")
    }
}

/// Errors from connection operations.
#[derive(Debug)]
pub enum ConnError {
    /// TCP I/O error.
    Io(io::Error),
    /// Wire protocol decode error.
    Wire(WireError),
    /// `recv_timeout` exceeded without a complete frame.
    Timeout { timeout: Duration },
    /// TCP stream closed by peer.
    Disconnected,
    /// Protocol violation (unexpected message type during a sequence).
    Protocol(String),
}

impl std::fmt::Display for ConnError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnError::Io(e) => write!(f, "I/O error: {e}"),
            ConnError::Wire(e) => write!(f, "wire error: {e}"),
            ConnError::Timeout { timeout } => {
                write!(f, "recv timed out after {timeout:?}")
            }
            ConnError::Disconnected => write!(f, "connection closed by device"),
            ConnError::Protocol(msg) => write!(f, "protocol error: {msg}"),
        }
    }
}

impl std::error::Error for ConnError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConnError::Io(e) => Some(e),
            ConnError::Wire(e) => Some(e),
            ConnError::Timeout { .. } | ConnError::Disconnected | ConnError::Protocol(_) => None,
        }
    }
}

impl From<io::Error> for ConnError {
    fn from(e: io::Error) -> Self {
        ConnError::Io(e)
    }
}

impl From<WireError> for ConnError {
    fn from(e: WireError) -> Self {
        ConnError::Wire(e)
    }
}

/// TCP connection to a Mevo+ device on port 5100.
///
/// Synchronous, single-threaded. Callers drive timing via `recv_timeout()`.
///
/// # Example
///
/// ```no_run
/// use std::time::Duration;
/// use ironsight::{Connection, ConnError};
///
/// let mut conn = Connection::connect(ironsight::conn::DEFAULT_ADDR)?;
/// loop {
///     match conn.recv_timeout(Duration::from_secs(1)) {
///         Ok(env) => println!("{:?}", env.message),
///         Err(ConnError::Timeout { .. }) => { /* send keepalive */ }
///         Err(e) => return Err(e),
///     }
/// }
/// # Ok::<(), ConnError>(())
/// ```
pub struct Connection {
    stream: TcpStream,
    splitter: FrameSplitter,
    read_buf: [u8; 4096],
    /// Frames split from TCP stream but not yet consumed by `recv()`.
    pending: Vec<Vec<u8>>,
    /// Called at the top of `send()` with the command and destination.
    on_send: Option<Box<dyn FnMut(&Command, BusAddr)>>,
    /// Called after each successful frame decode in `recv_inner()`.
    on_recv: Option<Box<dyn FnMut(&Envelope)>>,
}

impl Connection {
    /// Connect to a Mevo+ device with the system default timeout.
    pub fn connect(addr: impl ToSocketAddrs) -> Result<Self, ConnError> {
        let stream = TcpStream::connect(addr)?;
        Ok(Self::from_stream(stream))
    }

    /// Connect with an explicit timeout.
    pub fn connect_timeout(addr: &SocketAddr, timeout: Duration) -> Result<Self, ConnError> {
        let stream = TcpStream::connect_timeout(addr, timeout)?;
        Ok(Self::from_stream(stream))
    }

    fn from_stream(stream: TcpStream) -> Self {
        // Small frames (5-250B) — disable Nagle to avoid latency.
        let _ = stream.set_nodelay(true);
        Self {
            stream,
            splitter: FrameSplitter::new(),
            read_buf: [0u8; 4096],
            pending: Vec::new(),
            on_send: None,
            on_recv: None,
        }
    }

    /// Register a callback invoked at the top of every [`send()`](Self::send) call.
    pub fn set_on_send(&mut self, f: impl FnMut(&Command, BusAddr) + 'static) {
        self.on_send = Some(Box::new(f));
    }

    /// Register a callback invoked after every successful frame decode.
    pub fn set_on_recv(&mut self, f: impl FnMut(&Envelope) + 'static) {
        self.on_recv = Some(Box::new(f));
    }

    /// Send a command to the given bus address.
    pub fn send(&mut self, cmd: &Command, dest: BusAddr) -> Result<(), ConnError> {
        if let Some(cb) = self.on_send.as_mut() {
            cb(cmd, dest);
        }
        let frame = cmd.encode(dest);
        self.send_raw(&frame)
    }

    /// Send a pre-built raw frame (for messages not in [`Command`],
    /// e.g. `ClubPrc::encode_request()`).
    pub fn send_raw(&mut self, frame: &RawFrame) -> Result<(), ConnError> {
        let wire = frame.encode();
        self.stream.write_all(&wire)?;
        Ok(())
    }

    /// Block until a complete frame arrives and decode it.
    pub fn recv(&mut self) -> Result<Envelope, ConnError> {
        // Clear any read timeout so we block indefinitely.
        self.stream.set_read_timeout(None)?;
        self.recv_inner()
    }

    /// Block up to `timeout` for a complete frame.
    ///
    /// Returns `ConnError::Timeout` if no complete frame arrives in time.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Envelope, ConnError> {
        self.stream.set_read_timeout(Some(timeout))?;
        match self.recv_inner() {
            Err(ConnError::Io(ref e))
                if e.kind() == io::ErrorKind::WouldBlock
                    || e.kind() == io::ErrorKind::TimedOut =>
            {
                Err(ConnError::Timeout { timeout })
            }
            other => other,
        }
    }

    /// The peer address of the underlying TCP connection.
    pub fn peer_addr(&self) -> Result<SocketAddr, ConnError> {
        Ok(self.stream.peer_addr()?)
    }

    /// Shut down the TCP connection.
    pub fn shutdown(&self) -> Result<(), ConnError> {
        self.stream.shutdown(std::net::Shutdown::Both)?;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Internal
    // -----------------------------------------------------------------------

    fn recv_inner(&mut self) -> Result<Envelope, ConnError> {
        loop {
            // Drain pending frames first.
            if let Some(wire) = self.pending.pop() {
                let env = self.decode_wire(&wire)?;
                if let Some(cb) = self.on_recv.as_mut() {
                    cb(&env);
                }
                return Ok(env);
            }

            // Read from TCP.
            let n = self.stream.read(&mut self.read_buf)?;
            if n == 0 {
                return Err(ConnError::Disconnected);
            }

            let mut frames = self.splitter.feed(&self.read_buf[..n]);
            if let Some(first) = frames.pop() {
                // Stash extras (in arrival order — frames was built left-to-right,
                // pop takes from the end, so reverse before extending pending).
                frames.reverse();
                self.pending.extend(frames);
                let env = self.decode_wire(&first)?;
                if let Some(cb) = self.on_recv.as_mut() {
                    cb(&env);
                }
                return Ok(env);
            }
            // No complete frame yet — loop for more TCP data.
        }
    }

    fn decode_wire(&self, wire: &[u8]) -> Result<Envelope, ConnError> {
        let frame = RawFrame::parse(wire)?;
        let src = frame.src;
        let type_id = frame.type_id;
        let raw = frame.payload.clone();
        let message = Message::decode(&frame)
            .map_err(|e| ConnError::Wire(e.with_raw(&raw)))?;
        Ok(Envelope { src, type_id, raw, message })
    }
}
