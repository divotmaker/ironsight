//! Convert ironsight protocol types to FRP domain types.

use flightrelay::types::{BallFlight, ClubData};
use flightrelay::units::{Distance, Velocity};

use crate::protocol::shot::{ClubResult, FlightResult};

/// Convert a [`FlightResult`] (0xD4) to an FRP [`BallFlight`].
///
/// `carry_offline` is the lateral component of the landing position and
/// `descent_angle` the angle of the landing velocity below horizontal; both are
/// omitted when the result has no landing (zero forward landing distance, or a
/// landing velocity that is not descending). `total_spin_rpm` and `spin_axis`
/// are computed from backspin and sidespin.
#[must_use]
pub fn ball_flight(f: &FlightResult) -> BallFlight {
    // Wire sign convention (DSP) is opposite FRP for azimuth, sidespin and the
    // lateral axis of position vectors.
    // Wire: neg=right, neg=slice(RH). FRP: pos=right, pos=slice(RH).
    let mut ball = BallFlight {
        launch_speed: Some(Velocity::MetersPerSecond(f.launch_speed)),
        launch_azimuth: Some(-f.launch_azimuth),
        launch_elevation: Some(f.launch_elevation),
        carry_distance: Some(Distance::Meters(f.carry_distance)),
        carry_offline: carry_offline(f),
        total_distance: if f.total_distance != 0.0 {
            Some(Distance::Meters(f.total_distance))
        } else {
            None
        },
        roll_distance: if f.roll_distance != 0.0 {
            Some(Distance::Meters(f.roll_distance))
        } else {
            None
        },
        max_height: Some(Distance::Meters(f.max_height)),
        flight_time: Some(f.flight_time),
        descent_angle: descent_angle(f),
        backspin_rpm: Some(f.backspin_rpm),
        sidespin_rpm: Some(-f.sidespin_rpm),
        ..BallFlight::default()
    };
    ball.complete_spin();
    ball
}

/// Lateral landing offset, + = right of the target line.
///
/// `landing_position` is `[forward, vertical, lateral]` in DSP coordinates,
/// where lateral is negative to the right.
fn carry_offline(f: &FlightResult) -> Option<Distance> {
    let [forward, _, lateral] = f.landing_position;
    (forward > 0.0).then_some(Distance::Meters(-lateral))
}

/// Angle of the landing velocity below horizontal (degrees, + = descending).
fn descent_angle(f: &FlightResult) -> Option<f64> {
    if f.landing_position[0] <= 0.0 {
        return None;
    }
    let [forward, vertical, lateral] = f.landing_velocity;
    (vertical < 0.0).then(|| (-vertical).atan2(forward.hypot(lateral)).to_degrees())
}

/// Convert a [`ClubResult`] (0xED) to FRP [`ClubData`].
#[must_use]
pub fn club_data(c: &ClubResult) -> ClubData {
    // Wire sign convention (DSP) is opposite FRP for path, face angle,
    // and swing plane horizontal. DSP→PC negation per PROTOCOL.md §4.3.3.
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A slice that starts and lands right of target (wire neg = right).
    fn flight() -> FlightResult {
        FlightResult {
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
    fn ball_flight_conversion() {
        let ball = ball_flight(&flight());
        assert_eq!(ball.launch_speed, Some(Velocity::MetersPerSecond(60.0)));
        assert_eq!(ball.carry_distance, Some(Distance::Meters(152.0)));
        assert_eq!(ball.backspin_rpm, Some(4200));
        assert_eq!(ball.launch_azimuth, Some(2.5)); // wire -2.5 (neg=right) → FRP +2.5
        assert_eq!(ball.sidespin_rpm, Some(900)); // wire -900 (neg=slice) → FRP +900
        // total_distance is 0.0 → None
        assert_eq!(ball.total_distance, None);
        assert_eq!(ball.roll_distance, None);
        assert_eq!(ball.total_offline, None);
    }

    #[test]
    fn carry_offline_right_is_positive() {
        let ball = ball_flight(&flight());
        // wire lateral -9.5 (neg=right) → FRP +9.5, same side as launch_azimuth
        assert_eq!(ball.carry_offline, Some(Distance::Meters(9.5)));
    }

    #[test]
    fn carry_offline_left_is_negative() {
        let mut f = flight();
        f.landing_position = [80.0, 0.0, 3.5];
        assert_eq!(ball_flight(&f).carry_offline, Some(Distance::Meters(-3.5)));
    }

    #[test]
    fn descent_angle_from_landing_velocity() {
        let ball = ball_flight(&flight());
        let expected = 18.0_f64.atan2(24.0_f64.hypot(1.5)).to_degrees();
        let descent = ball.descent_angle.unwrap();
        assert!((descent - expected).abs() < 1e-9);
        assert!((descent - 36.82).abs() < 0.01);
    }

    #[test]
    fn landing_fields_omitted_without_landing() {
        let mut f = flight();
        f.landing_position = [0.0; 3];
        f.landing_velocity = [0.0; 3];
        let ball = ball_flight(&f);
        assert_eq!(ball.carry_offline, None);
        assert_eq!(ball.descent_angle, None);
    }

    #[test]
    fn descent_angle_omitted_when_not_descending() {
        let mut f = flight();
        f.landing_velocity = [19.0, 0.0, 0.0];
        assert_eq!(ball_flight(&f).descent_angle, None);
        f.landing_velocity = [0.0; 3];
        assert_eq!(ball_flight(&f).descent_angle, None);
    }

    #[test]
    fn total_spin_and_axis_from_components() {
        let ball = ball_flight(&flight());
        // √(4200² + 900²) = 4295.3
        assert_eq!(ball.total_spin_rpm, Some(4295));
        // sidespin +900 (curves right) → axis tilted right
        let axis = ball.spin_axis.unwrap();
        assert!((axis - 900_f64.atan2(4200.0).to_degrees()).abs() < 1e-9);
        assert!(axis > 0.0);
    }

    #[test]
    fn spin_axis_left_is_negative() {
        let mut f = flight();
        f.sidespin_rpm = 600; // wire + = draw (RH), curves left
        let ball = ball_flight(&f);
        assert_eq!(ball.sidespin_rpm, Some(-600));
        assert!(ball.spin_axis.unwrap() < 0.0);
    }

    #[test]
    fn zero_spin_has_no_axis() {
        let mut f = flight();
        f.backspin_rpm = 0;
        f.sidespin_rpm = 0;
        let ball = ball_flight(&f);
        assert_eq!(ball.total_spin_rpm, Some(0));
        assert_eq!(ball.spin_axis, None);
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
        assert_eq!(data.path, Some(2.1)); // wire -2.1 negated → FRP +2.1
        assert_eq!(data.face_angle, Some(-1.2)); // wire +1.2 negated → FRP -1.2
        assert_eq!(data.smash_factor, Some(1.50));
        // Spin loft and low point are not reported by the device.
        assert_eq!(data.spin_loft, None);
        assert_eq!(data.low_point_distance, None);
    }
}
