//! Binary protocol connection to a Mevo+ device on port 5100.
//!
//! Handles frame splitting, message encode/decode, and I/O over any
//! `Read + Write` stream. No application logic — callers drive timing
//! and sequencing.

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
    /// I/O error from the underlying stream.
    Io(io::Error),
    /// Wire protocol decode error.
    Wire(WireError),
    /// No complete frame available (stream returned `WouldBlock`/`TimedOut`).
    Timeout,
    /// Stream closed by peer (read returned 0 bytes).
    Disconnected,
    /// Protocol violation (unexpected message type during a sequence).
    Protocol(String),
}

impl std::fmt::Display for ConnError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnError::Io(e) => write!(f, "I/O error: {e}"),
            ConnError::Wire(e) => write!(f, "wire error: {e}"),
            ConnError::Timeout => write!(f, "recv timed out"),
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
            ConnError::Timeout | ConnError::Disconnected | ConnError::Protocol(_) => None,
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

/// Binary protocol connection to a Mevo+ device.
///
/// Generic over `S: Read + Write` so callers can use any stream type.
/// For non-blocking usage, configure the stream's read timeout externally
/// and call [`recv()`](Self::recv) — it returns `Ok(None)` on
/// `WouldBlock`/`TimedOut` instead of blocking.
///
/// # Example (TcpStream)
///
/// ```no_run
/// use std::time::Duration;
/// use ironsight::{BinaryConnection, ConnError};
///
/// let mut conn = BinaryConnection::connect(ironsight::conn::DEFAULT_ADDR)?;
/// conn.stream_mut().set_read_timeout(Some(Duration::from_millis(100)))?;
/// loop {
///     match conn.recv()? {
///         Some(env) => println!("{:?}", env.message),
///         None => { /* send keepalive */ }
///     }
/// }
/// # Ok::<(), ConnError>(())
/// ```
pub struct BinaryConnection<S: Read + Write> {
    stream: S,
    splitter: FrameSplitter,
    read_buf: [u8; 4096],
    /// Frames split from the stream but not yet consumed by `recv()`.
    pending: Vec<Vec<u8>>,
    /// Called at the top of `send()` with the command and destination.
    #[allow(clippy::type_complexity)]
    on_send: Option<Box<dyn FnMut(&Command, BusAddr)>>,
    /// Called after each successful frame decode in `recv()`.
    #[allow(clippy::type_complexity)]
    on_recv: Option<Box<dyn FnMut(&Envelope)>>,
}

// -- Generic methods (any Read + Write stream) --------------------------------

impl<S: Read + Write> BinaryConnection<S> {
    /// Wrap any `Read + Write` stream as a binary protocol connection.
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            splitter: FrameSplitter::new(),
            read_buf: [0u8; 4096],
            pending: Vec::new(),
            on_send: None,
            on_recv: None,
        }
    }

    /// Borrow the underlying stream (e.g. to query peer address).
    pub fn stream(&self) -> &S {
        &self.stream
    }

    /// Mutably borrow the underlying stream (e.g. to set read timeout).
    pub fn stream_mut(&mut self) -> &mut S {
        &mut self.stream
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

    /// Try to receive the next complete frame.
    ///
    /// - `Ok(Some(env))` — decoded message.
    /// - `Ok(None)` — no data available (`WouldBlock`/`TimedOut` from stream).
    /// - `Err(Disconnected)` — stream closed by peer.
    /// - `Err(Wire|Io)` — decode or I/O error.
    ///
    /// The stream's read timeout controls whether this blocks or returns
    /// `Ok(None)` immediately. Set the timeout on the stream before calling.
    pub fn recv(&mut self) -> Result<Option<Envelope>, ConnError> {
        loop {
            // Drain pending frames first.
            if let Some(wire) = self.pending.pop() {
                let env = Self::decode_wire(&wire)?;
                if let Some(cb) = self.on_recv.as_mut() {
                    cb(&env);
                }
                return Ok(Some(env));
            }

            // Read from stream.
            match self.stream.read(&mut self.read_buf) {
                Ok(0) => return Err(ConnError::Disconnected),
                Ok(n) => {
                    let mut frames = self.splitter.feed(&self.read_buf[..n]);
                    if let Some(first) = frames.pop() {
                        // Stash extras (in arrival order — frames was built
                        // left-to-right, pop takes from the end, so reverse).
                        frames.reverse();
                        self.pending.extend(frames);
                        let env = Self::decode_wire(&first)?;
                        if let Some(cb) = self.on_recv.as_mut() {
                            cb(&env);
                        }
                        return Ok(Some(env));
                    }
                    // No complete frame yet — loop for more data.
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    return Ok(None);
                }
                Err(e) => return Err(ConnError::Io(e)),
            }
        }
    }

    // -- Internal -------------------------------------------------------------

    fn decode_wire(wire: &[u8]) -> Result<Envelope, ConnError> {
        let frame = RawFrame::parse(wire)?;
        let src = frame.src;
        let type_id = frame.type_id;
        let raw = frame.payload.clone();
        let message =
            Message::decode(&frame).map_err(|e| ConnError::Wire(e.with_raw(&raw)))?;
        Ok(Envelope {
            src,
            type_id,
            raw,
            message,
        })
    }
}

// -- TcpStream convenience methods --------------------------------------------

impl BinaryConnection<TcpStream> {
    /// Connect to a Mevo+ device with the system default timeout.
    pub fn connect(addr: impl ToSocketAddrs) -> Result<Self, ConnError> {
        let stream = TcpStream::connect(addr)?;
        let _ = stream.set_nodelay(true);
        Ok(Self::new(stream))
    }

    /// Connect with an explicit timeout.
    pub fn connect_timeout(addr: &SocketAddr, timeout: Duration) -> Result<Self, ConnError> {
        let stream = TcpStream::connect_timeout(addr, timeout)?;
        let _ = stream.set_nodelay(true);
        Ok(Self::new(stream))
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

    /// Block up to `timeout` for a complete frame.
    ///
    /// Returns `ConnError::Timeout` if no complete frame arrives in time.
    ///
    /// This is a convenience wrapper for callers that prefer the old
    /// timeout-per-call style. Prefer setting the stream read timeout
    /// once and using [`recv()`](Self::recv) directly.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<Envelope, ConnError> {
        self.stream.set_read_timeout(Some(timeout))?;
        match self.recv()? {
            Some(env) => Ok(env),
            None => Err(ConnError::Timeout),
        }
    }
}

/// Type alias for backwards compatibility.
pub type Connection = BinaryConnection<TcpStream>;
