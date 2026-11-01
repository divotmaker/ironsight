//! Result frames as a FlightScope Xi sent them, for tests.
//!
//! Each constant is a frame's payload, without the frame header. None of
//! them identifies a unit: shot results carry no serial number, network or
//! certificate. Decode them with the matching type in
//! [`protocol::shot`](crate::protocol::shot).
//!
//! Directions follow the device's sign convention, positive = left (see
//! [`protocol::shot`](crate::protocol::shot#sign-convention)).

/// Decode `N` bytes from a hex string, skipping whitespace (a byte's two
/// digits may be split across lines).
const fn hex<const N: usize>(s: &str) -> [u8; N] {
    const fn nibble(c: u8) -> u8 {
        match c {
            b'0'..=b'9' => c - b'0',
            b'a'..=b'f' => c - b'a' + 10,
            b'A'..=b'F' => c - b'A' + 10,
            _ => panic!("not a hex digit"),
        }
    }
    let s = s.as_bytes();
    let mut out = [0u8; N];
    let (mut i, mut digits) = (0, 0);
    while i < s.len() {
        if !s[i].is_ascii_whitespace() {
            assert!(digits < 2 * N, "hex string longer than N bytes");
            let shift = if digits % 2 == 0 { 4 } else { 0 };
            out[digits / 2] |= nibble(s[i]) << shift;
            digits += 1;
        }
        i += 1;
    }
    assert!(digits == 2 * N, "hex string shorter than N bytes");
    out
}

/// Early result (0xE8) of an Xi full swing with club data. Launched 8.1°
/// left and faded back: 85.2 m carry, landing 12.1 m left. Backspin 7157
/// rpm.
pub const XI_SHOT_E8: &[u8] = &hex::<94>(
    "5d000004000000008a28000ff5014c9f0034ef002f1b005209001fcd000000000000001bf500000000005600
     000001e00f0000003d08e0f5d3a3019570ffe4ea00000017ccdefb5b2eff69290012cb00000008bc60fe8a
     0e003a8afffc10",
);

/// Full result (0xD4) of the same shot as [`XI_SHOT_E8`]: azimuth 8.141°
/// left, landing 10.756 m left of the origin at 84.472 m, backspin 7127
/// rpm, sidespin −652 rpm (curving right), club path 11.407° and face
/// 6.519° left.
pub const XI_SHOT_D4: &[u8] = &hex::<157>(
    "9c00000400005600000000002700020a008a28001fcd005209014ca2000ff30034e30149f8000000002a04
     001bd7fffd740000000003e9fffefbffe76d003c1affce900006f70000000000000000000000000000000
     06c96002c8fffc66e004e9500d43600571500975a00197701df710000003d08ebf5cc2b019bbcffe3fd000
     00017c8affb5496ff6c7f00126500000008b809fe35a1004bd6fffab0",
);

/// Club result (0xED) of the same shot as [`XI_SHOT_E8`]: 27.79 m/s, path
/// 11.40° left, attack angle −14.73°, face 6.51° left, swing plane
/// horizontal 22.29° left, vertical 54.32°.
pub const XI_SHOT_ED: &[u8] = &hex::<167>(
    "a62f002040000adb0007db000474fffa3f00028b000f220004f80002240008b50015380003f2fffed00002
     1f0000040028460000000000030000020009ea000000045a3bfffcf203ec210e3ea8fffcf203ebe9e12e7a
     000030ff23313d086c000030ff233a2a9ca70015de00f05d3531680015df00f041146dc2ffff02fffa6f00
     1f47ffff02fffa6f001f470016bb000dd80019820016bb000dd8001982fff3a0001080000102",
);

/// Spin result (0xEF) of the same shot as [`XI_SHOT_E8`].
pub const XI_SHOT_EF: &[u8] = &hex::<138>(
    "8930040003520008266a000251000643e00001a500052eb000016c0005216300016600054b0800
     0164000521e900028800072c0f000264000732e00001b600062ed500027900093329000223000
     821cb000220000800000000000000000000000000000000000000000011111bf500741bf50000
     000000000000030000051bf503691111000000000000",
);

/// Early result (0xE8) of an Xi full swing without club data that drew:
/// launched 3.1° left, landing 5.3 m left. Backspin 6897 rpm.
pub const XI_DRAW_E8: &[u8] = &hex::<94>(
    "5d00000600000000942400128d0178410049d60014a4005d7e000c48000000000000001af100000000005800
     000001c6220000003d08f2f57fd4018863ffe88b0000001b5f02fb475bff7f2a000f4f000000035a4eff6c
     7b00157efffeb8",
);

/// Full result (0xD4) of the same shot as [`XI_DRAW_E8`]: azimuth 3.144°
/// left, landing 11.548 m left of the origin at 100.222 m, backspin 2756
/// rpm, sidespin 1464 rpm (curving left), no club data.
pub const XI_DRAW_D4: &[u8] = &hex::<157>(
    "9c00000600005800000000000d00021a009424000c48005d7e018a1600113700434201877e000000002d1c
     000ac40005b8000000ffffa6000262fff32a00433bffc671000a9000000000000000000000000000000000
     79fc000000000000005c5000000000000000000000000001c55f0000003d08edf75422014102ffeb5b0000
     001b82defa4676ffadd3000bd50000000373af00b838ffd83c000317",
);

/// Early result (0xE8) of an Xi chip in limited-flight mode that has no
/// full result: 17.94 m/s at 30.948°, azimuth 3.958° left, 28.164 m carry,
/// 2.145 m left, backspin 4037 rpm.
pub const XI_CHIP_E8: &[u8] = &hex::<94>(
    "5d00000c003652004619000879006e040012f20008610078e4000f76000000019640000fc50002040000df
     00000003fac40000003d08faf913c6017639ffdd7300000024a1b0eec81bffddb40021d9000000043815ff
     d4c300034200009b",
);

/// Club result (0xED) of the same chip as [`XI_CHIP_E8`]: 13.9 m/s, face
/// 5.92° left, loft 46.31°.
pub const XI_CHIP_ED: &[u8] = &hex::<167>(
    "a60700000000056e00000000000000000000025000121700050a0000000000000000000000000000000000
     000000000463970000000000ce0005ab0000000000003d08f4000000000000000000000000000000000000
     00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
     00000000000000000000000000000000000000000000000000000000000000000000000003f1",
);

/// Early result (0xE8) of a shot the Xi triggered on but did not read:
/// every measurement is 0 ([`FlightResultV1::is_no_read`]).
///
/// [`FlightResultV1::is_no_read`]: crate::protocol::shot::FlightResultV1::is_no_read
pub const XI_NO_READ_E8: &[u8] = &hex::<94>(
    "5d000009000000000000000000000000000000000000000000000000000000000000000000000000000000
     0000003d090000000000000000000000000000000000000000000000000000000000000000000000000000
     0000000000000000",
);

/// Club result (0xED) of the same shot as [`XI_NO_READ_E8`]: all zeros.
pub const XI_NO_READ_ED: &[u8] = &hex::<167>(
    "a6000000000000000000000000000000000000000000000000000000000000000000000000000000000000
     000000003d0900000000000000000000000000000000000000000000000000000000000000000000000000
     00000000000000000000000000000000000000000000000000000000000000000000000000000000000000
     0000000000000000000000000000000000000000000000000000000000000000000000000000",
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::shot::{ClubResult, FlightResult, FlightResultV1, SpinResult};

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn fixtures_decode() {
        let d4 = FlightResult::decode(XI_SHOT_D4).unwrap();
        assert_eq!((d4.backspin_rpm, d4.sidespin_rpm), (7127, -652));
        assert!(close(d4.launch_azimuth, 8.141));
        assert!(close(d4.landing_position[0], 84.472));
        assert!(close(d4.landing_position[2], 10.756));
        let e8 = FlightResultV1::decode(XI_SHOT_E8).unwrap();
        assert_eq!(e8.backspin_rpm, 7157);
        assert!(close(e8.azimuth, 8.141));
        assert!(close(e8.lateral, 12.059));
        let ed = ClubResult::decode(XI_SHOT_ED).unwrap();
        assert!(close(ed.strike_direction, 11.4));
        assert!(close(ed.face_angle, 6.51));
        assert!(close(ed.swing_plane_horizontal, 22.29));
        assert!(SpinResult::decode(XI_SHOT_EF).is_ok());

        let d4 = FlightResult::decode(XI_DRAW_D4).unwrap();
        assert_eq!((d4.backspin_rpm, d4.sidespin_rpm), (2756, 1464));
        assert_eq!(
            FlightResultV1::decode(XI_DRAW_E8).unwrap().backspin_rpm,
            6897
        );

        let chip = FlightResultV1::decode(XI_CHIP_E8).unwrap();
        assert!(!chip.is_no_read());
        assert_eq!(chip.backspin_rpm, 4037);
        assert!(close(
            ClubResult::decode(XI_CHIP_ED).unwrap().face_angle,
            5.92
        ));

        let no_read = FlightResultV1::decode(XI_NO_READ_E8).unwrap();
        assert!(no_read.is_no_read());
        assert!(ClubResult::decode(XI_NO_READ_ED).is_ok());
    }

    #[test]
    fn result_predicates() {
        let mut d4 = FlightResult::decode(XI_SHOT_D4).unwrap();
        assert!(!d4.is_model_failure());
        for speed in [0.0, -0.1, f64::NAN] {
            d4.launch_speed = speed;
            assert!(d4.is_model_failure(), "{speed}");
        }
        // As the device sends it: launch speed and elevation zeroed.
        let mut raw = XI_SHOT_D4.to_vec();
        raw[16..19].fill(0);
        raw[22..25].fill(0);
        assert!(FlightResult::decode(&raw).unwrap().is_model_failure());

        let mut e8 = FlightResultV1::decode(XI_CHIP_E8).unwrap();
        e8.ball_velocity = f64::NAN;
        assert!(e8.is_no_read());
    }

    /// Lateral distances share the launch azimuth's sign: these shots
    /// launched left (positive) and landed left (positive).
    #[test]
    fn lateral_has_the_azimuth_sign() {
        for (d4, e8) in [(XI_SHOT_D4, XI_SHOT_E8), (XI_DRAW_D4, XI_DRAW_E8)] {
            let d4 = FlightResult::decode(d4).unwrap();
            let e8 = FlightResultV1::decode(e8).unwrap();
            assert!(d4.launch_azimuth > 0.0 && e8.azimuth > 0.0);
            assert!(close(d4.launch_azimuth, e8.azimuth));
            assert!(d4.landing_position[2] > 0.0 && e8.lateral > 0.0);
        }
    }

    /// Positive sidespin curves the ball left. Relative to its start, the
    /// fade (sidespin −652) lands right of its launch line and the draw
    /// (sidespin +1464) lands left of it.
    #[test]
    fn sidespin_sign_follows_the_curve() {
        for (raw, curves_left) in [(XI_SHOT_D4, false), (XI_DRAW_D4, true)] {
            let d4 = FlightResult::decode(raw).unwrap();
            let [x, _, z] = d4.landing_position;
            let landing_deg = (z - d4.start_position[2]).atan2(x).to_degrees();
            assert_eq!(
                landing_deg > d4.launch_azimuth,
                curves_left,
                "{landing_deg}"
            );
            assert_eq!(d4.sidespin_rpm > 0, curves_left);
        }
    }

    /// The club fields share the sign: the fade's face (6.51° left) is
    /// right of its path (11.40° left), open to the path, which fades the
    /// ball. With a downward attack the swing direction lies left of the
    /// path by |attack| / tan(plane), the swing plane horizontal reported.
    #[test]
    fn club_signs_are_consistent() {
        let ed = ClubResult::decode(XI_SHOT_ED).unwrap();
        let d4 = FlightResult::decode(XI_SHOT_D4).unwrap();
        assert!(d4.sidespin_rpm < 0);
        assert!(ed.face_angle < ed.strike_direction);
        assert!(close(d4.club_face_angle, 6.519) && close(d4.club_strike_direction, 11.407));
        assert!(ed.attack_angle < 0.0);
        let swing_dir =
            ed.strike_direction - ed.attack_angle / ed.swing_plane_vertical.to_radians().tan();
        assert!(
            (swing_dir - ed.swing_plane_horizontal).abs() < 0.5,
            "{swing_dir}"
        );
    }
}
