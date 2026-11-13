//! The driver's settings: [`Config`] and the enums it uses.
//!
//! Requires the `config` feature, which adds only serde and flightrelay's
//! dependency-light `config` feature, so a settings form or a WebAssembly UI
//! can use it without the device stack. The driver (`ironsight::driver`,
//! feature `driver`) runs on it.
//!
//! Every field has a default and is left out of the serialized form while it
//! holds it, so a default config writes nothing. Most fields are `Option`s:
//! unset means the default given in the field's docs.
//!
//! ```
//! use flightrelay::config::DriverConfig;
//! use ironsight::config::{Config, FlightScopeEnvironment};
//!
//! let config = Config::from_map(
//!     serde_json::json!({ "address": "192.168.2.1:5100", "environment": "outdoor" })
//!         .as_object()
//!         .cloned()
//!         .unwrap_or_default(),
//! )?;
//! assert_eq!(config.environment, Some(FlightScopeEnvironment::Outdoor));
//! assert!(Config::default().to_map().is_empty());
//! # Ok::<(), flightrelay::config::ConfigError>(())
//! ```
// "FlightScope" and "WiFi" in the docs are names, not code.
#![allow(clippy::doc_markdown)]

use std::net::SocketAddr;

use flightrelay::config::{ConfigError, DriverConfig, is_default};
use flightrelay::{Distance, Mode};
use serde::{Deserialize, Serialize};

use crate::protocol::config::{
    MODE_CHIPPING, MODE_FULL_OUTDOOR, MODE_LIMITED_FLIGHT, MODE_PUTTING, RadarCal,
};
use crate::protocol::handshake::DeviceModel;
use crate::seq::{AvrSettings, MevoSettings, XiSettings};

/// The device address used while [`Config::address`] is unset.
pub const DEFAULT_ADDRESS: &str = crate::conn::DEFAULT_ADDR;
/// Ball type used while [`Config::ball_type`] is unset: RCT.
pub const DEFAULT_BALL_TYPE: u8 = 1;
/// Tee height used while [`Config::tee_height`] is unset.
pub const DEFAULT_TEE_HEIGHT: Distance = Distance::Inches(1.5);
/// Range used while [`Config::range`] is unset.
pub const DEFAULT_RANGE: Distance = Distance::Feet(8.0);
/// Surface height used while [`Config::surface_height`] is unset.
pub const DEFAULT_SURFACE_HEIGHT: Distance = Distance::Inches(0.0);
/// Track percentage used while [`Config::track_pct`] is unset.
pub const DEFAULT_TRACK_PCT: f64 = 80.0;

/// Settings of the FlightScope driver (Mevo, Mevo+, Mevo Gen2, Xi).
///
/// The keys are the ones flighthook's FlightScope card writes, so the same
/// table works in both.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Config {
    /// The device's address, `host:port`. Unset: [`DEFAULT_ADDRESS`]
    /// (`192.168.2.1:5100`, the device's own WiFi network).
    #[serde(skip_serializing_if = "is_default")]
    pub address: Option<String>,
    /// Ball type: 0 = standard, 1 = RCT. Unset: 1. Mevo+ and Gen2 only.
    #[serde(skip_serializing_if = "is_default")]
    pub ball_type: Option<u8>,
    /// Height of the tee. Unset: 1.5 in. Mevo+ and Gen2 only.
    #[serde(skip_serializing_if = "is_default")]
    pub tee_height: Option<Distance>,
    /// Distance from the device to the ball. Unset: 8 ft.
    #[serde(skip_serializing_if = "is_default")]
    pub range: Option<Distance>,
    /// Height of the hitting surface above the device's base. Unset: 0 in.
    #[serde(skip_serializing_if = "is_default")]
    pub surface_height: Option<Distance>,
    /// Minimum track percentage, 0 to 100. Unset: 80. Mevo+ and Gen2 only.
    #[serde(skip_serializing_if = "is_default")]
    pub track_pct: Option<f64>,
    /// Send a shot from the device's early result when its full result does
    /// not arrive (e.g. a flight too short to read in outdoor mode). Such a
    /// shot may lack sidespin and club data. Unset: off.
    #[serde(skip_serializing_if = "is_default")]
    pub use_estimated: Option<bool>,
    /// Camera mode, applied once per session. Unset: [`CameraMode::Standard`].
    /// Mevo+ and Gen2 only.
    #[serde(skip_serializing_if = "is_default")]
    pub camera_mode: Option<CameraMode>,
    /// Where the device is used, which picks the full-swing detection mode.
    /// Unset: [`FlightScopeEnvironment::LimitedFlight`].
    #[serde(skip_serializing_if = "is_default")]
    pub environment: Option<FlightScopeEnvironment>,
    /// Xi only: which of the device's two spin readings a shot reports.
    /// Applied live, without a new session.
    #[serde(skip_serializing_if = "is_default")]
    pub spin_source: SpinSource,
}

/// Camera mode requested from the device at session start (Mevo+ and Gen2).
///
/// Every mode reports ball flight and the radar's club data. Fusion
/// processing adds the full club data set; it needs the Pro Package on the
/// device, and the two Fusion variants depend on the firmware: the wrong one
/// yields no Fusion club data.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraMode {
    /// Ball flight and radar club data, no Fusion processing. The default.
    #[default]
    Standard,
    /// High-resolution JPEG Fusion (1640x1232), older firmware.
    Fusion,
    /// Raw Fusion (640x480 at 180 fps), firmware BM17.04 and newer.
    RawFusion,
}

/// Where a FlightScope device is used, which picks its full-swing detection
/// mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlightScopeEnvironment {
    /// Nets and simulator screens, where the ball's flight is cut short. The
    /// default.
    #[default]
    LimitedFlight,
    /// Full ball flight outdoors. Needs about 40 yd of unobstructed flight;
    /// shots into a net or screen do not read.
    Outdoor,
}

/// Which backspin and sidespin a FlightScope Xi shot reports.
///
/// The Xi sends two readings per shot: its full result, the spin FS Golf
/// displays, and its early result. A Mevo+ or Gen2 has one, so every source
/// gives the same shot there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SpinSource {
    /// Backspin and sidespin from the full result, as reported. The default.
    #[default]
    D4,
    /// Total spin from the early result. When the shot has club data and a
    /// full-result backspin, the total is split by the full result's spin
    /// axis; otherwise it goes out alone, with no axis. A shot without an
    /// early-result spin keeps the full result's.
    E8,
    /// The `E8` rule when the mode at the trigger is full swing,
    /// otherwise `D4`.
    Auto,
}

impl DriverConfig for Config {
    fn validate(&self) -> Result<(), ConfigError> {
        self.socket_addr()?;
        if self.ball_type.is_some_and(|t| t > 1) {
            return Err(ConfigError::field("ball_type", "must be 0 or 1"));
        }
        if self
            .track_pct
            .is_some_and(|p| !(p.is_finite() && (0.0..=100.0).contains(&p)))
        {
            return Err(ConfigError::field("track_pct", "must be 0 to 100"));
        }
        for (field, value) in [
            ("tee_height", self.tee_height),
            ("range", self.range),
            ("surface_height", self.surface_height),
        ] {
            if value.is_some_and(|d| !(d.value().is_finite() && d.value() >= 0.0)) {
                return Err(ConfigError::field(field, "must be a distance of 0 or more"));
            }
        }
        Ok(())
    }

    /// `spin_source` applies live; every other change needs a new session.
    fn restart_needed(old: &Self, new: &Self) -> bool {
        let new = Self {
            spin_source: old.spin_source,
            ..new.clone()
        };
        *old != new
    }

    fn example() -> Self {
        Self {
            address: Some(String::new()),
            ball_type: Some(DEFAULT_BALL_TYPE),
            tee_height: Some(DEFAULT_TEE_HEIGHT),
            range: Some(DEFAULT_RANGE),
            surface_height: Some(DEFAULT_SURFACE_HEIGHT),
            track_pct: Some(DEFAULT_TRACK_PCT),
            use_estimated: Some(false),
            camera_mode: Some(CameraMode::Standard),
            environment: Some(FlightScopeEnvironment::LimitedFlight),
            spin_source: SpinSource::D4,
        }
    }
}

impl Config {
    /// The device's address: [`address`](Self::address), or
    /// [`DEFAULT_ADDRESS`] while it is unset.
    ///
    /// # Errors
    ///
    /// Returns a [`ConfigError`] naming `address` when it is not an IP
    /// address and port.
    pub fn socket_addr(&self) -> Result<SocketAddr, ConfigError> {
        let text = self.address.as_deref().unwrap_or(DEFAULT_ADDRESS);
        text.trim().parse().map_err(|_| {
            ConfigError::field(
                "address",
                format!("must be an IP address and port, e.g. {DEFAULT_ADDRESS} (got {text:?})"),
            )
        })
    }

    /// Whether to send estimated shots ([`use_estimated`](Self::use_estimated)).
    #[must_use]
    pub fn use_estimated(&self) -> bool {
        self.use_estimated.unwrap_or(false)
    }

    /// The camera mode ([`camera_mode`](Self::camera_mode)).
    #[must_use]
    pub fn camera_mode(&self) -> CameraMode {
        self.camera_mode.unwrap_or_default()
    }

    /// The environment ([`environment`](Self::environment)).
    #[must_use]
    pub fn environment(&self) -> FlightScopeEnvironment {
        self.environment.unwrap_or_default()
    }

    /// The radar calibration from [`range`](Self::range) and
    /// [`surface_height`](Self::surface_height): the range in millimeters
    /// and the surface height in whole millimeters, rounded down.
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // saturating, as the wire fields are
    pub fn radar_cal(&self) -> RadarCal {
        let range = self.range.unwrap_or(DEFAULT_RANGE);
        let surface = self.surface_height.unwrap_or(DEFAULT_SURFACE_HEIGHT);
        RadarCal {
            range_mm: range.as_millimeters() as u16,
            height_mm: (surface.as_inches() * 25.4).floor() as u8,
        }
    }

    /// The full-swing detection mode for the environment, on the Mevo+ and
    /// the Xi.
    #[must_use]
    pub fn full_swing_mode(&self) -> u8 {
        match self.environment() {
            FlightScopeEnvironment::LimitedFlight => MODE_LIMITED_FLIGHT,
            FlightScopeEnvironment::Outdoor => MODE_FULL_OUTDOOR,
        }
    }

    /// The Mevo+ or Gen2 detection mode for `mode`.
    #[must_use]
    pub fn mevo_mode(&self, mode: Mode) -> u8 {
        match mode {
            Mode::Full => self.full_swing_mode(),
            Mode::Putting => MODE_PUTTING,
            Mode::Chipping => MODE_CHIPPING,
        }
    }

    /// The full Mevo+ or Gen2 settings for `mode`, for a session's first
    /// configuration: the detection mode, ball type, track percentage, tee
    /// height and radar calibration.
    #[must_use]
    pub fn mevo_settings(&self, mode: Mode) -> AvrSettings {
        let track = self.track_pct.unwrap_or(DEFAULT_TRACK_PCT);
        MevoSettings::new(
            self.mevo_mode(mode),
            self.ball_type.unwrap_or(DEFAULT_BALL_TYPE),
            track / 100.0,
            self.tee_height.unwrap_or(DEFAULT_TEE_HEIGHT).as_meters(),
            self.radar_cal(),
        )
        .into()
    }

    /// The Mevo+ or Gen2 settings for a mode change: the detection mode
    /// alone.
    #[must_use]
    pub fn mevo_mode_settings(&self, mode: Mode) -> AvrSettings {
        AvrSettings::mode_only(self.mevo_mode(mode))
    }

    /// The Xi configuration for `mode`: full swing in the environment's mode,
    /// or the pitching shot type for chipping and putting, with the radar
    /// calibration.
    #[must_use]
    pub fn xi_settings(&self, mode: Mode) -> XiSettings {
        match mode {
            Mode::Full => XiSettings::full_swing(self.full_swing_mode(), self.radar_cal()),
            Mode::Chipping | Mode::Putting => XiSettings::pitching(self.radar_cal()),
        }
    }

    /// What `mode` selects on a `model`, with the detection mode number sent
    /// to the device, for log lines: e.g. `"limited-flight(9)"` or
    /// `"pitching limited-flight(9)"`.
    #[must_use]
    pub fn wire_label(&self, model: DeviceModel, mode: Mode) -> &'static str {
        let outdoor = self.environment() == FlightScopeEnvironment::Outdoor;
        match (model, mode, outdoor) {
            (DeviceModel::Xi, Mode::Full, false) => "full-swing limited-flight(9)",
            (DeviceModel::Xi, Mode::Full, true) => "full-swing outdoor(8)",
            (DeviceModel::Xi, _, _) => "pitching limited-flight(9)",
            (DeviceModel::Mevo, Mode::Full, false) => "limited-flight(9)",
            (DeviceModel::Mevo, Mode::Full, true) => "outdoor(8)",
            (DeviceModel::Mevo, Mode::Putting, _) => "putting(3)",
            (DeviceModel::Mevo, Mode::Chipping, _) => "chipping(5)",
        }
    }

    /// Whether moving from `from` to `to` changes what is sent to a `model`.
    /// On the Xi, chipping and putting select the same session.
    #[must_use]
    pub fn changes_wire(&self, model: DeviceModel, from: Mode, to: Mode) -> bool {
        match model {
            DeviceModel::Xi => self.xi_settings(from) != self.xi_settings(to),
            DeviceModel::Mevo => from != to,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::XiShotType;

    fn map(json: serde_json::Value) -> serde_json::Map<String, serde_json::Value> {
        match json {
            serde_json::Value::Object(map) => map,
            _ => serde_json::Map::new(),
        }
    }

    #[test]
    fn a_default_config_writes_nothing_and_reads_back() {
        let config = Config::default();
        assert!(config.to_map().is_empty());
        assert_eq!(Config::from_map(config.to_map()).unwrap(), config);
        assert_eq!(
            Config::from_map(map(serde_json::json!({}))).unwrap(),
            config
        );
    }

    #[test]
    fn every_field_round_trips_with_the_keys_flighthook_writes() {
        let config = Config::example();
        let config = Config {
            address: Some("10.0.0.5:5100".into()),
            spin_source: SpinSource::Auto,
            camera_mode: Some(CameraMode::RawFusion),
            environment: Some(FlightScopeEnvironment::Outdoor),
            ..config
        };
        let written = config.to_map();
        let keys: Vec<&str> = written.keys().map(String::as_str).collect();
        let mut sorted = keys.clone();
        sorted.sort_unstable();
        assert_eq!(
            sorted,
            [
                "address",
                "ball_type",
                "camera_mode",
                "environment",
                "range",
                "spin_source",
                "surface_height",
                "tee_height",
                "track_pct",
                "use_estimated",
            ]
        );
        assert_eq!(written["spin_source"], "auto");
        assert_eq!(written["camera_mode"], "raw_fusion");
        assert_eq!(written["environment"], "outdoor");
        assert_eq!(Config::from_map(written).unwrap(), config);
    }

    #[test]
    fn a_file_section_reads_as_flighthook_wrote_it() {
        let config: Config = serde_json::from_value(serde_json::json!({
            "name": "Mevo WiFi",
            "address": "192.168.2.1:5100",
            "ball_type": 1,
            "tee_height": "1.5in",
            "range": "8ft",
            "track_pct": 80,
            "spin_source": "e8"
        }))
        .unwrap();
        assert_eq!(config.range, Some(Distance::Feet(8.0)));
        assert_eq!(config.tee_height, Some(Distance::Inches(1.5)));
        assert_eq!(config.track_pct, Some(80.0));
        assert_eq!(config.spin_source, SpinSource::E8);
        assert_eq!(config.environment, None);
    }

    #[test]
    fn invalid_values_name_their_field() {
        for (json, field) in [
            (serde_json::json!({ "address": "mevo" }), "address"),
            (serde_json::json!({ "address": "" }), "address"),
            (serde_json::json!({ "ball_type": 2 }), "ball_type"),
            (serde_json::json!({ "track_pct": 101 }), "track_pct"),
            (serde_json::json!({ "track_pct": -1 }), "track_pct"),
            (serde_json::json!({ "range": "-2ft" }), "range"),
            (serde_json::json!({ "spin_source": "e9" }), "spin_source"),
            (serde_json::json!({ "track_pct": "lots" }), "track_pct"),
        ] {
            let err = Config::from_map(map(json.clone())).unwrap_err();
            assert_eq!(err.field.as_deref(), Some(field), "{json}: {err}");
        }
    }

    #[test]
    fn only_a_spin_source_change_applies_live() {
        let old = Config::default();
        let spin = Config {
            spin_source: SpinSource::E8,
            ..Config::default()
        };
        assert!(!Config::restart_needed(&old, &spin));
        assert!(!Config::restart_needed(&old, &old));
        for new in [
            Config {
                range: Some(Distance::Feet(9.0)),
                ..spin.clone()
            },
            Config {
                use_estimated: Some(true),
                ..Config::default()
            },
            Config {
                camera_mode: Some(CameraMode::Fusion),
                ..Config::default()
            },
            Config {
                address: Some("10.0.0.5:5100".into()),
                ..Config::default()
            },
        ] {
            assert!(Config::restart_needed(&old, &new), "{new:?}");
        }
    }

    #[test]
    fn mevo_modes_follow_the_environment() {
        let limited = Config::default();
        let outdoor = Config {
            environment: Some(FlightScopeEnvironment::Outdoor),
            ..Config::default()
        };
        assert_eq!(limited.mevo_settings(Mode::Full).mode, 9);
        assert_eq!(outdoor.mevo_settings(Mode::Full).mode, 8);
        for c in [&limited, &outdoor] {
            assert_eq!(c.mevo_mode_settings(Mode::Chipping).mode, 5);
            assert_eq!(c.mevo_mode_settings(Mode::Putting).mode, 3);
            assert_eq!(c.mevo_settings(Mode::Putting).mode, 3);
        }
        assert_eq!(
            limited.wire_label(DeviceModel::Mevo, Mode::Full),
            "limited-flight(9)"
        );
        assert_eq!(
            outdoor.wire_label(DeviceModel::Mevo, Mode::Chipping),
            "chipping(5)"
        );
    }

    #[test]
    fn xi_chipping_and_putting_are_one_pitching_session() {
        for env in [
            FlightScopeEnvironment::LimitedFlight,
            FlightScopeEnvironment::Outdoor,
        ] {
            let c = Config {
                environment: Some(env),
                ..Config::default()
            };
            for mode in [Mode::Chipping, Mode::Putting] {
                let xi = c.xi_settings(mode);
                assert_eq!(xi.mode, MODE_LIMITED_FLIGHT);
                assert_eq!(xi.shot_type, XiShotType::Pitching);
            }
            assert!(!c.changes_wire(DeviceModel::Xi, Mode::Chipping, Mode::Putting));
            assert!(c.changes_wire(DeviceModel::Xi, Mode::Full, Mode::Putting));
            assert!(c.changes_wire(DeviceModel::Mevo, Mode::Chipping, Mode::Putting));
            assert_eq!(c.xi_settings(Mode::Full).shot_type, XiShotType::FullSwing);
        }
    }

    #[test]
    fn radar_calibration_from_range_and_surface_height() {
        let c = Config {
            range: Some(Distance::Feet(10.0)),
            surface_height: Some(Distance::Inches(1.0)),
            ..Config::default()
        };
        let cal = c.radar_cal();
        assert_eq!((cal.range_mm, cal.height_mm), (3048, 25));
        assert_eq!(Some(cal.clone()), c.mevo_settings(Mode::Full).radar_cal);
        assert_eq!(cal, c.xi_settings(Mode::Chipping).radar_cal);
        let cal = Config::default().radar_cal();
        assert_eq!((cal.range_mm, cal.height_mm), (2438, 0));
    }
}
