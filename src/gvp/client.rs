//! Non-blocking client for the GVP camera protocol (port 1258).
//!
//! Wraps [`GvpConnection`] and provides a single `poll() -> Option<GvpEvent>`
//! entry point matching the [`BinaryClient`](crate::client::BinaryClient)
//! pattern.
//!
//! Unlike the binary protocol, GVP has no multi-step handshake or keepalive.
//! All sends are immediate (no operation queue). The client caches the latest
//! camera config and provides typed send methods for trigger, hints, and
//! config.

use std::io::{Read, Write};
use std::net::TcpStream;

use super::config::GvpConfig;
use super::conn::GvpConnection;
use super::log::GvpLog;
use super::result::BallTrackerResult;
use super::status::GvpStatus;
use super::track::ExpectedTrack;
use super::trigger::Trigger;
use super::video::VideoAvailable;
use super::{GvpCommand, GvpError, GvpMessage};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Events emitted by [`GvpClient::poll()`].
#[derive(Debug)]
pub enum GvpEvent {
    /// Camera configuration (response to config query or push).
    Config(GvpConfig),
    /// Buffer status update (IDLE → TRIGGERED → PROCESSING → ...).
    Status(GvpStatus),
    /// Ball tracker result with raw pixel tracking data.
    Result(Box<BallTrackerResult>),
    /// Video file saved notification.
    VideoAvailable(VideoAvailable),
    /// Debug/info log message from the GVP processor.
    Log(GvpLog),
    /// Unknown or unrecognized message (forward compatibility).
    Unknown { msg_type: String, raw: String },
}

// ---------------------------------------------------------------------------
// GvpClient
// ---------------------------------------------------------------------------

/// Non-blocking client for the GVP camera protocol (port 1258).
///
/// Wraps a [`GvpConnection`] and provides a single [`poll()`](Self::poll)
/// entry point that receives messages and emits [`GvpEvent`]s.
///
/// # Usage
///
/// ```no_run
/// use ironsight::gvp::client::{GvpClient, GvpEvent};
/// use ironsight::gvp::GvpConnection;
///
/// let conn = GvpConnection::connect("192.168.2.1:1258")?;
/// let mut client = GvpClient::from_tcp(conn)?;
/// client.query_config()?;
///
/// loop {
///     match client.poll()? {
///         Some(GvpEvent::Config(cfg)) => {
///             println!("Camera: {}x{}", cfg.camera_configuration.roi_width,
///                      cfg.camera_configuration.roi_height);
///         }
///         Some(GvpEvent::Result(r)) => {
///             println!("Tracks: {}", r.tracks.len());
///         }
///         Some(_) => {}
///         None => {}
///     }
/// }
/// # Ok::<(), ironsight::gvp::GvpError>(())
/// ```
pub struct GvpClient<S: Read + Write> {
    conn: GvpConnection<S>,
    config: Option<GvpConfig>,
}

impl<S: Read + Write> GvpClient<S> {
    /// Create a new GVP client wrapping the given connection.
    ///
    /// The underlying stream **must** be non-blocking (or have a short read
    /// timeout). [`poll()`](Self::poll) relies on `recv()` returning
    /// `Ok(None)` promptly to yield control back to the caller. A blocking
    /// stream will stall the poll loop until a message arrives.
    ///
    /// For `TcpStream`, use [`from_tcp()`](Self::from_tcp) which sets
    /// non-blocking mode automatically.
    pub fn new(conn: GvpConnection<S>) -> Self {
        Self { conn, config: None }
    }

    /// Poll for the next GVP event.
    ///
    /// Returns `Ok(None)` when no data is available (non-blocking).
    pub fn poll(&mut self) -> Result<Option<GvpEvent>, GvpError> {
        let msg = match self.conn.recv()? {
            Some(msg) => msg,
            None => return Ok(None),
        };

        let event = match msg {
            GvpMessage::Config(cfg) => {
                self.config = Some(cfg.clone());
                GvpEvent::Config(cfg)
            }
            GvpMessage::Status(s) => GvpEvent::Status(s),
            GvpMessage::Result(r) => GvpEvent::Result(Box::new(r)),
            GvpMessage::VideoAvailable(v) => GvpEvent::VideoAvailable(v),
            GvpMessage::Log(l) => GvpEvent::Log(l),
            GvpMessage::Unknown { msg_type, raw } => GvpEvent::Unknown { msg_type, raw },
        };

        Ok(Some(event))
    }

    // -- Send methods (immediate, no queuing) -------------------------------

    /// Request the current camera configuration from the GVP.
    pub fn query_config(&mut self) -> Result<(), GvpError> {
        self.conn.send(&GvpCommand::ConfigRequest)
    }

    /// Push a camera configuration to the GVP.
    pub fn send_config(&mut self, config: &GvpConfig) -> Result<(), GvpError> {
        self.conn.send(&GvpCommand::Config(config.clone()))
    }

    /// Send a shot trigger to the GVP.
    pub fn send_trigger(&mut self, trigger: &Trigger) -> Result<(), GvpError> {
        self.conn.send(&GvpCommand::Trigger(trigger.clone()))
    }

    /// Send a club head trajectory hint to the GVP.
    pub fn send_club_track(&mut self, track: &ExpectedTrack) -> Result<(), GvpError> {
        self.conn
            .send(&GvpCommand::ExpectedClubTrack(track.clone()))
    }

    /// Send a ball trajectory hint to the GVP.
    pub fn send_ball_track(&mut self, track: &ExpectedTrack) -> Result<(), GvpError> {
        self.conn.send(&GvpCommand::ExpectedTrack(track.clone()))
    }

    // -- Read-only accessors ------------------------------------------------

    /// Latest camera configuration, cached from the most recent CONFIG message.
    #[must_use]
    pub fn config(&self) -> Option<&GvpConfig> {
        self.config.as_ref()
    }
}

// -- TcpStream convenience --------------------------------------------------

impl GvpClient<TcpStream> {
    /// Create a GVP client from a TCP connection, setting the stream to
    /// non-blocking mode for use with [`poll()`](Self::poll).
    pub fn from_tcp(conn: GvpConnection<TcpStream>) -> Result<Self, GvpError> {
        // Best-effort: already set by connect()/connect_timeout(), but the
        // caller may have constructed the connection manually. Failure is
        // harmless (just higher latency on small writes).
        let _ = conn.stream().set_nodelay(true);
        conn.stream().set_nonblocking(true)?;
        Ok(Self::new(conn))
    }
}
