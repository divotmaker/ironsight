//! GVP camera processor connection on port 1258.
//!
//! Handles null-byte frame splitting, JSON decode/encode, and I/O over any
//! `Read + Write` stream. No application logic — callers drive timing
//! and sequencing.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use super::splitter::NullSplitter;
use super::{GvpCommand, GvpError, GvpMessage};

/// Default GVP camera port.
pub const DEFAULT_PORT: u16 = 1258;

/// GVP camera processor connection.
///
/// Generic over `S: Read + Write` so callers can use any stream type.
/// For non-blocking usage, configure the stream's read timeout externally
/// and call [`recv()`](Self::recv) — it returns `Ok(None)` on
/// `WouldBlock`/`TimedOut`.
///
/// Mirrors [`crate::conn::BinaryConnection`] for the binary protocol.
pub struct GvpConnection<S: Read + Write> {
    stream: S,
    splitter: NullSplitter,
    read_buf: [u8; 8192],
    /// JSON strings split from the stream but not yet consumed by `recv()`.
    pending: Vec<String>,
    /// Called at the top of `send()` with the command.
    #[allow(clippy::type_complexity)]
    on_send: Option<Box<dyn FnMut(&GvpCommand)>>,
    /// Called after each successful message decode in `recv()`.
    /// Receives the raw JSON string (hex-first: for JSON protocol, raw JSON
    /// IS the canonical form).
    #[allow(clippy::type_complexity)]
    on_recv: Option<Box<dyn FnMut(&str, &GvpMessage)>>,
}

// -- Generic methods (any Read + Write stream) --------------------------------

impl<S: Read + Write> GvpConnection<S> {
    /// Wrap any `Read + Write` stream as a GVP connection.
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            splitter: NullSplitter::new(),
            read_buf: [0u8; 8192],
            pending: Vec::new(),
            on_send: None,
            on_recv: None,
        }
    }

    /// Borrow the underlying stream.
    pub fn stream(&self) -> &S {
        &self.stream
    }

    /// Mutably borrow the underlying stream (e.g. to set read timeout).
    pub fn stream_mut(&mut self) -> &mut S {
        &mut self.stream
    }

    /// Register a callback invoked at the top of every [`send()`](Self::send) call.
    pub fn set_on_send(&mut self, f: impl FnMut(&GvpCommand) + 'static) {
        self.on_send = Some(Box::new(f));
    }

    /// Register a callback invoked after every successful message decode.
    ///
    /// The callback receives the raw JSON string and the decoded message.
    /// Per the hex-first logging policy: for JSON protocol, the raw JSON
    /// string IS the canonical representation.
    pub fn set_on_recv(&mut self, f: impl FnMut(&str, &GvpMessage) + 'static) {
        self.on_recv = Some(Box::new(f));
    }

    /// Send a command to the GVP.
    pub fn send(&mut self, cmd: &GvpCommand) -> Result<(), GvpError> {
        if let Some(cb) = self.on_send.as_mut() {
            cb(cmd);
        }
        let bytes = cmd.encode();
        self.stream.write_all(&bytes)?;
        Ok(())
    }

    /// Try to receive the next complete message.
    ///
    /// - `Ok(Some(msg))` — decoded message.
    /// - `Ok(None)` — no data available (`WouldBlock`/`TimedOut` from stream).
    /// - `Err(Disconnected)` — stream closed by peer.
    /// - `Err(Json|Io)` — decode or I/O error.
    pub fn recv(&mut self) -> Result<Option<GvpMessage>, GvpError> {
        loop {
            // Drain pending messages first.
            if let Some(json) = self.pending.pop() {
                let msg = GvpMessage::decode(&json)?;
                if let Some(cb) = self.on_recv.as_mut() {
                    cb(&json, &msg);
                }
                return Ok(Some(msg));
            }

            // Read from stream.
            match self.stream.read(&mut self.read_buf) {
                Ok(0) => return Err(GvpError::Disconnected),
                Ok(n) => {
                    let mut messages = self.splitter.feed(&self.read_buf[..n]);
                    if let Some(first) = messages.pop() {
                        // Stash extras (in arrival order — messages was built
                        // left-to-right, pop takes from the end, so reverse).
                        messages.reverse();
                        self.pending.extend(messages);
                        let msg = GvpMessage::decode(&first)?;
                        if let Some(cb) = self.on_recv.as_mut() {
                            cb(&first, &msg);
                        }
                        return Ok(Some(msg));
                    }
                    // No complete message yet — loop for more data.
                }
                Err(ref e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.kind() == io::ErrorKind::TimedOut =>
                {
                    return Ok(None);
                }
                Err(e) => return Err(GvpError::Io(e)),
            }
        }
    }
}

// -- TcpStream convenience methods --------------------------------------------

impl GvpConnection<TcpStream> {
    /// Connect to the GVP camera processor with the system default timeout.
    pub fn connect(addr: impl ToSocketAddrs) -> Result<Self, GvpError> {
        let stream = TcpStream::connect(addr)?;
        let _ = stream.set_nodelay(true);
        Ok(Self::new(stream))
    }

    /// Connect with an explicit timeout.
    pub fn connect_timeout(addr: &SocketAddr, timeout: Duration) -> Result<Self, GvpError> {
        let stream = TcpStream::connect_timeout(addr, timeout)?;
        let _ = stream.set_nodelay(true);
        Ok(Self::new(stream))
    }

    /// The peer address of the underlying TCP connection.
    pub fn peer_addr(&self) -> Result<SocketAddr, GvpError> {
        Ok(self.stream.peer_addr()?)
    }

    /// Shut down the TCP connection.
    pub fn shutdown(&self) -> Result<(), GvpError> {
        self.stream.shutdown(std::net::Shutdown::Both)?;
        Ok(())
    }

    /// Block up to `timeout` for a complete message.
    ///
    /// Returns `GvpError::Timeout` if no complete message arrives in time.
    ///
    /// Convenience wrapper. Prefer setting the stream read timeout once
    /// and using [`recv()`](Self::recv) directly.
    pub fn recv_timeout(&mut self, timeout: Duration) -> Result<GvpMessage, GvpError> {
        self.stream.set_read_timeout(Some(timeout))?;
        match self.recv()? {
            Some(msg) => Ok(msg),
            None => Err(GvpError::Timeout),
        }
    }
}
