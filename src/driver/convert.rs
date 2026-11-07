//! Convert ironsight's shot results to FRP types.
//!
//! The device reports directions positive = left (see
//! [`protocol::shot`](crate::protocol::shot#sign-convention)); FRP's are
//! positive = right, so azimuth, sidespin, lateral offsets, club path, face
//! angle and swing plane direction are negated.
//!
//! A full result (0xD4) becomes a [`Launch`] (the measured launch) and a
//! [`FlightResult`] with [`Basis::Modeled`]: the device's flight values come
//! from its own flight model. An early result (0xE8) is converted the same
//! way, for a shot whose full result never came.

use flightrelay::types::{Basis, ClubData, FlightResult, Launch};
use flightrelay::units::{Distance, Velocity};

use crate::protocol::shot::{ClubResult, FlightResult as FullResult, FlightResultV1};

/// Lowest launch angle (degrees) of a launch-only full result. Below it a
/// result without spin or flight is a ball that stayed on the ground (a putt,
/// a bladed shot), and its apex of 0 is a measurement.
pub const LAUNCH_ONLY_MIN_LAUNCH_DEG: f64 = 5.0;

/// The measured launch of a full result: speed, angles, backspin and
/// sidespin, with total spin and spin axis worked out from them.
#[must_use]
pub fn launch(f: &FullResult) -> Launch {
    let mut launch = Launch {
        launch_speed: Some(Velocity::MetersPerSecond(f.launch_speed)),
        launch_azimuth: Some(-f.launch_azimuth),
        launch_elevation: Some(f.launch_elevation),
        backspin_rpm: Some(f.backspin_rpm),
        sidespin_rpm: Some(-f.sidespin_rpm),
        ..Launch::default()
    };
    launch.complete_spin();
    launch
}

/// The flight of a full result, [`Basis::Modeled`]: carry, apex and flight
/// time, total and roll when they are not 0, and the landing's offline and
/// descent angle when the ball flew (carry, flight time and apex all above
/// 0).
#[must_use]
pub fn flight_result(f: &FullResult) -> FlightResult {
    let flew = flew(f.carry_distance, f.flight_time, f.max_height);
    FlightResult {
        carry_distance: Some(Distance::Meters(f.carry_distance)),
        carry_offline: flew
            .then(|| finite(-f.landing_position[2]))
            .flatten()
            .map(Distance::Meters),
        total_distance: nonzero(f.total_distance).map(Distance::Meters),
        roll_distance: nonzero(f.roll_distance).map(Distance::Meters),
        max_height: Some(Distance::Meters(f.max_height)),
        flight_time: Some(f.flight_time),
        descent_angle: flew.then(|| descent_angle(f.landing_velocity)).flatten(),
        ..FlightResult::new(Basis::Modeled)
    }
}

/// Whether a full result is launch-only: launch data, but no spin (backspin
/// and sidespin 0) and no flight at all (carry, flight time and apex each 0,
/// negative or not a number), launched at [`LAUNCH_ONLY_MIN_LAUNCH_DEG`] or
/// more. The Xi sends these on very short chips; their zeros are not
/// measurements, so a shot from one carries no spin and no flight result.
#[must_use]
pub fn launch_only(f: &FullResult) -> bool {
    let absent = |v: f64| !(v.is_finite() && v > 0.0);
    absent(f.carry_distance)
        && absent(f.flight_time)
        && absent(f.max_height)
        && f.backspin_rpm == 0
        && f.sidespin_rpm == 0
        && f.launch_elevation >= LAUNCH_ONLY_MIN_LAUNCH_DEG
}

/// The launch of an early result: speed, angles and backspin. The early
/// result's sidespin is always 0, so it is left out.
#[must_use]
pub fn early_launch(e: &FlightResultV1) -> Launch {
    Launch {
        launch_speed: Some(Velocity::MetersPerSecond(e.ball_velocity)),
        launch_azimuth: Some(-e.azimuth),
        launch_elevation: Some(e.elevation),
        backspin_rpm: Some(e.backspin_rpm),
        ..Launch::default()
    }
}

/// The flight of an early result, [`Basis::Modeled`]: carry, apex and flight
/// time, and the landing's offline when the ball flew.
#[must_use]
pub fn early_flight_result(e: &FlightResultV1) -> FlightResult {
    let flew = flew(e.distance, e.flight_time, e.height);
    FlightResult {
        carry_distance: Some(Distance::Meters(e.distance)),
        carry_offline: flew
            .then(|| finite(-e.lateral))
            .flatten()
            .map(Distance::Meters),
        max_height: Some(Distance::Meters(e.height)),
        flight_time: Some(e.flight_time),
        ..FlightResult::new(Basis::Modeled)
    }
}

/// Convert a club result (0xED) to FRP [`ClubData`].
#[must_use]
pub fn club_data(c: &ClubResult) -> ClubData {
    ClubData {
        club_speed: Some(Velocity::MetersPerSecond(c.pre_club_speed)),
        club_speed_post: Some(Velocity::MetersPerSecond(c.post_club_speed)),
        path: Some(-c.strike_direction),
        attack_angle: Some(c.attack_angle),
        face_angle: Some(-c.face_angle),
        dynamic_loft: Some(c.dynamic_loft),
        smash_factor: Some(c.smash_factor),
        swing_plane_horizontal: Some(-c.swing_plane_horizontal),
        swing_plane_vertical: Some(c.swing_plane_vertical),
        club_offset: Some(Distance::Meters(c.club_offset)),
        club_height: Some(Distance::Meters(c.club_height)),
        ..ClubData::default()
    }
}

/// Whether a result describes a ball in flight: positive, finite carry,
/// flight time and apex.
fn flew(carry: f64, flight_time: f64, apex: f64) -> bool {
    [carry, flight_time, apex]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
}

fn finite(v: f64) -> Option<f64> {
    v.is_finite().then_some(v)
}

#[allow(clippy::float_cmp)] // the device sends an exact 0 for "none"
fn nonzero(v: f64) -> Option<f64> {
    (v != 0.0 && v.is_finite()).then_some(v)
}

/// Angle below horizontal (degrees) of the landing velocity
/// `[forward, vertical, lateral]`, or `None` when the ball is not descending.
fn descent_angle(v: [f64; 3]) -> Option<f64> {
    let [forward, vertical, lateral] = v;
    let angle = (-vertical).atan2(forward.hypot(lateral)).to_degrees();
    (vertical < 0.0 && angle.is_finite()).then_some(angle)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A slice that starts and lands right of target (wire negative = right).
    fn flight() -> FullResult {
        FullResult {
            total: 2,
            track_time: 0.5,
            start_position: [0.0; 3],
            launch_speed: 60.0,
            launch_azimuth: -2.5,
            launch_elevation: 13.0,
            carry_distance: 152.0,
            flight_time: 6.0,
            max_height: 25.0,
            landing_position: [151.7, 0.0, -9.5],
            backspin_rpm: 4200,
            sidespin_rpm: -900,
            riflespin_rpm: 0,
            landing_spin_rpm: [0; 3],
            landing_velocity: [24.0, -18.0, -1.5],
            total_distance: 0.0,
            roll_distance: 0.0,
            final_position: [0.0; 3],
            clubhead_speed: 40.0,
            club_strike_direction: -1.0,
            club_attack_angle: -4.0,
            clubhead_speed_post: 30.0,
            club_swing_plane_tilt: 60.0,
            club_swing_plane_rotation: 3.0,
            club_effective_loft: 20.0,
            club_face_angle: -3.0,
            poly_scale: 0,
            poly_x: [0.0; 5],
            poly_y: [0.0; 5],
            poly_z: [0.0; 5],
        }
    }

    #[test]
    fn launch_conversion() {
        let launch = launch(&flight());
        assert_eq!(launch.launch_speed, Some(Velocity::MetersPerSecond(60.0)));
        assert_eq!(launch.backspin_rpm, Some(4200));
        assert_eq!(launch.launch_azimuth, Some(2.5)); // wire -2.5 (neg=right) → FRP +2.5
        assert_eq!(launch.sidespin_rpm, Some(900)); // wire -900 (neg=slice) → FRP +900
    }

    #[test]
    fn flight_result_conversion() {
        let flight = flight_result(&flight());
        assert_eq!(flight.basis, Basis::Modeled);
        assert_eq!(flight.carry_distance, Some(Distance::Meters(152.0)));
        assert_eq!(flight.max_height, Some(Distance::Meters(25.0)));
        assert_eq!(flight.flight_time, Some(6.0));
        // A total of 0 is "none".
        assert_eq!(flight.total_distance, None);
        assert_eq!(flight.roll_distance, None);
        assert_eq!(flight.total_offline, None);
        let mut f = super::tests::flight();
        f.total_distance = 160.0;
        f.roll_distance = 8.0;
        let flight = flight_result(&f);
        assert_eq!(flight.total_distance, Some(Distance::Meters(160.0)));
        assert_eq!(flight.roll_distance, Some(Distance::Meters(8.0)));
    }

    #[test]
    fn carry_offline_right_is_positive() {
        // wire lateral -9.5 (neg=right) → FRP +9.5, same side as launch_azimuth
        assert_eq!(
            flight_result(&flight()).carry_offline,
            Some(Distance::Meters(9.5))
        );
        let mut f = flight();
        f.landing_position = [80.0, 0.0, 3.5];
        assert_eq!(
            flight_result(&f).carry_offline,
            Some(Distance::Meters(-3.5))
        );
    }

    #[test]
    fn descent_angle_from_landing_velocity() {
        let descent = flight_result(&flight()).descent_angle.unwrap();
        let expected = 18.0_f64.atan2(24.0_f64.hypot(1.5)).to_degrees();
        assert!((descent - expected).abs() < 1e-9);
        assert!((descent - 36.82).abs() < 0.01);
    }

    #[test]
    fn landing_fields_need_a_ball_that_flew() {
        for change in [
            |f: &mut FullResult| f.carry_distance = 0.0,
            |f: &mut FullResult| f.flight_time = 0.0,
            |f: &mut FullResult| f.max_height = 0.0,
        ] {
            let mut f = flight();
            change(&mut f);
            let flight = flight_result(&f);
            assert_eq!(flight.carry_offline, None);
            assert_eq!(flight.descent_angle, None);
        }
    }

    #[test]
    fn descent_angle_omitted_when_not_descending() {
        let mut f = flight();
        f.landing_velocity = [19.0, 0.0, 0.0];
        assert_eq!(flight_result(&f).descent_angle, None);
        f.landing_velocity = [0.0; 3];
        assert_eq!(flight_result(&f).descent_angle, None);
    }

    #[test]
    fn total_spin_and_axis_from_components() {
        let launch = launch(&flight());
        // √(4200² + 900²) = 4295.3
        assert_eq!(launch.total_spin_rpm, Some(4295));
        let axis = launch.spin_axis.unwrap();
        assert!((axis - 900_f64.atan2(4200.0).to_degrees()).abs() < 1e-9);
        assert!(axis > 0.0);
        let mut f = flight();
        f.sidespin_rpm = 600; // wire + = curves left
        let launch = super::launch(&f);
        assert_eq!(launch.sidespin_rpm, Some(-600));
        assert!(launch.spin_axis.unwrap() < 0.0);
    }

    #[test]
    fn zero_spin_has_no_axis() {
        let mut f = flight();
        f.backspin_rpm = 0;
        f.sidespin_rpm = 0;
        let launch = launch(&f);
        assert_eq!(launch.total_spin_rpm, Some(0));
        assert_eq!(launch.spin_axis, None);
    }

    /// A chip the device returned with launch data only.
    fn launch_only_result() -> FullResult {
        FullResult {
            launch_speed: 9.0,
            launch_elevation: 28.0,
            carry_distance: 0.0,
            flight_time: 0.0,
            max_height: 0.0,
            landing_position: [0.0; 3],
            landing_velocity: [0.0; 3],
            backspin_rpm: 0,
            sidespin_rpm: 0,
            ..flight()
        }
    }

    #[test]
    fn launch_only_needs_no_spin_no_flight_and_five_degrees() {
        assert!(launch_only(&launch_only_result()));
        let mut f = launch_only_result();
        f.launch_elevation = LAUNCH_ONLY_MIN_LAUNCH_DEG;
        assert!(launch_only(&f));
        f.launch_elevation = 4.9;
        assert!(!launch_only(&f), "a low launch is a ground ball");
        for change in [
            |f: &mut FullResult| f.carry_distance = 1.0,
            |f: &mut FullResult| f.flight_time = 0.2,
            |f: &mut FullResult| f.max_height = 0.1,
            |f: &mut FullResult| f.backspin_rpm = 900,
            |f: &mut FullResult| f.sidespin_rpm = -50,
        ] {
            let mut f = launch_only_result();
            change(&mut f);
            assert!(!launch_only(&f), "{f:?}");
        }
        let mut f = launch_only_result();
        f.carry_distance = f64::NAN;
        f.max_height = -0.5;
        assert!(launch_only(&f), "not a number and negative are absent");
        assert!(!launch_only(&flight()));
    }

    #[test]
    fn early_result_conversion() {
        let e = FlightResultV1 {
            total: 1,
            club_velocity: 20.0,
            ball_velocity: 30.0,
            flight_time: 3.0,
            distance: 60.0,
            height: 12.0,
            lateral: 2.0,
            elevation: 25.0,
            azimuth: 1.5,
            tracked_time: 1.0,
            drag: 0.0,
            backspin_rpm: 5000,
            sidespin_rpm: 0,
            acceleration: 0.0,
            club_strike_direction: 0.0,
            poly_scale: 0,
            poly_x: [0.0; 5],
            poly_y: [0.0; 5],
            poly_z: [0.0; 5],
        };
        let launch = early_launch(&e);
        assert_eq!(launch.launch_azimuth, Some(-1.5));
        assert_eq!(launch.backspin_rpm, Some(5000));
        assert_eq!(launch.sidespin_rpm, None);
        assert_eq!(launch.total_spin_rpm, None);
        let flight = early_flight_result(&e);
        assert_eq!(flight.basis, Basis::Modeled);
        assert_eq!(flight.carry_distance, Some(Distance::Meters(60.0)));
        assert_eq!(flight.carry_offline, Some(Distance::Meters(-2.0)));
        let ground = FlightResultV1 { height: 0.0, ..e };
        assert_eq!(early_flight_result(&ground).carry_offline, None);
    }

    #[test]
    fn club_data_conversion() {
        let club = ClubResult {
            num_club_prc_points: 12,
            flags: 0,
            pre_club_speed: 42.1,
            post_club_speed: 38.6,
            strike_direction: -2.1,
            attack_angle: -3.5,
            face_angle: 1.2,
            dynamic_loft: 18.4,
            smash_factor: 1.50,
            dispersion_correction: 0.0,
            swing_plane_horizontal: 5.3,
            swing_plane_vertical: 58.1,
            club_azimuth: 0.0,
            club_elevation: 0.0,
            club_offset: 0.012,
            club_height: 0.003,
            poly_scale: 0,
            poly_coeffs: [[0.0; 3]; 12],
            pre_impact_time: 0.0,
            post_impact_time: 0.0,
            club_to_ball_time: 0.0,
        };
        let data = club_data(&club);
        assert_eq!(data.club_speed, Some(Velocity::MetersPerSecond(42.1)));
        assert_eq!(data.path, Some(2.1));
        assert_eq!(data.face_angle, Some(-1.2));
        assert_eq!(data.swing_plane_horizontal, Some(-5.3));
        assert_eq!(data.smash_factor, Some(1.50));
        assert_eq!(data.spin_loft, None);
        assert_eq!(data.low_point_distance, None);
    }
}
