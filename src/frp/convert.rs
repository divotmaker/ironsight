//! Convert ironsight protocol types to FRP domain types.

use flightrelay::units::{Distance, Velocity};
use flightrelay::types::{BallFlight, ClubData};

use crate::protocol::shot::{ClubResult, FlightResult};

/// Convert a [`FlightResult`] (0xD4) to an FRP [`BallFlight`].
#[must_use]
pub fn ball_flight(f: &FlightResult) -> BallFlight {
    BallFlight {
        launch_speed: Some(Velocity::MetersPerSecond(f.launch_speed)),
        launch_azimuth: Some(f.launch_azimuth),
        launch_elevation: Some(f.launch_elevation),
        carry_distance: Some(Distance::Meters(f.carry_distance)),
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
        backspin_rpm: Some(f.backspin_rpm),
        sidespin_rpm: Some(f.sidespin_rpm),
    }
}

/// Convert a [`ClubResult`] (0xED) to FRP [`ClubData`].
#[must_use]
pub fn club_data(c: &ClubResult) -> ClubData {
    ClubData {
        club_speed: Some(Velocity::MetersPerSecond(c.pre_club_speed)),
        club_speed_post: Some(Velocity::MetersPerSecond(c.post_club_speed)),
        path: Some(c.strike_direction),
        attack_angle: Some(c.attack_angle),
        face_angle: Some(c.face_angle),
        dynamic_loft: Some(c.dynamic_loft),
        smash_factor: Some(c.smash_factor),
        swing_plane_horizontal: Some(c.swing_plane_horizontal),
        swing_plane_vertical: Some(c.swing_plane_vertical),
        club_offset: Some(Distance::Meters(c.club_offset)),
        club_height: Some(Distance::Meters(c.club_height)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ball_flight_conversion() {
        let flight = FlightResult {
            total: 1,
            track_time: 0.5,
            start_position: [0.0; 3],
            launch_speed: 67.2,
            launch_azimuth: -1.3,
            launch_elevation: 14.2,
            carry_distance: 180.5,
            flight_time: 6.2,
            max_height: 28.3,
            landing_position: [0.0; 3],
            backspin_rpm: 3200,
            sidespin_rpm: -450,
            riflespin_rpm: 0,
            landing_spin_rpm: [0; 3],
            landing_velocity: [0.0; 3],
            total_distance: 0.0,
            roll_distance: 0.0,
            final_position: [0.0; 3],
            clubhead_speed: 42.0,
            club_strike_direction: -2.1,
            club_attack_angle: -3.5,
            clubhead_speed_post: 38.0,
            club_swing_plane_tilt: 58.0,
            club_swing_plane_rotation: 5.3,
            club_effective_loft: 18.4,
            club_face_angle: 1.2,
            poly_scale: 0,
            poly_x: [0.0; 5],
            poly_y: [0.0; 5],
            poly_z: [0.0; 5],
        };

        let ball = ball_flight(&flight);
        assert_eq!(ball.launch_speed, Some(Velocity::MetersPerSecond(67.2)));
        assert_eq!(ball.carry_distance, Some(Distance::Meters(180.5)));
        assert_eq!(ball.backspin_rpm, Some(3200));
        assert_eq!(ball.sidespin_rpm, Some(-450));
        // total_distance is 0.0 → None
        assert_eq!(ball.total_distance, None);
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
        assert_eq!(data.path, Some(-2.1));
        assert_eq!(data.smash_factor, Some(1.50));
    }
}
