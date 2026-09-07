//! Earth orientation and solar geometry.
//!
//! `gmst_rad` gives the Greenwich Mean Sidereal Time angle (IAU 1982 model),
//! which rotates the TEME/inertial frame onto the Earth-fixed frame about the
//! pole — the standard recipe for SGP4-class work. `sun_direction` is
//! Vallado's low-precision solar ephemeris (~0.01° in ecliptic longitude),
//! plenty for lighting, terminator rendering, and umbra/penumbra eclipse
//! tests. `eci_to_ecef` applies that same GMST rotation to a state vector,
//! which is what the Earth-fixed view of a trajectory is made of.
//! UT1 ≈ UTC (< 0.9 s) is accepted throughout — ~0.004° of rotation.

use glam::DVec3;
use hifitime::Epoch;
use std::f64::consts::TAU;

/// Julian date (UT1 ≈ UTC) of an epoch.
fn jd_utc(epoch: Epoch) -> f64 {
    2_440_587.5 + epoch.to_unix_seconds() / 86_400.0
}

/// Greenwich Mean Sidereal Time, radians in [0, 2π). IAU 1982 model.
pub fn gmst_rad(epoch: Epoch) -> f64 {
    let t = (jd_utc(epoch) - 2_451_545.0) / 36_525.0;
    // Seconds of sidereal time (Vallado eq. 3-47).
    let gmst_s = 67_310.548_41
        + (876_600.0 * 3600.0 + 8_640_184.812_866) * t
        + 0.093_104 * t * t
        - 6.2e-6 * t * t * t;
    (gmst_s.rem_euclid(86_400.0) / 86_400.0 * TAU).rem_euclid(TAU)
}

/// Earth's rotation rate, rad/s (IAU 1976 mean sidereal rate).
pub const OMEGA_EARTH: f64 = 7.292_115e-5;

/// Rotate an inertial (TEME) vector onto the Earth-fixed frame: R_z(-GMST).
///
/// Strictly this lands in PEF, not ITRF — the polar-motion step is omitted, as
/// it is a few tens of metres at the pole and invisible here. Longitude 0
/// (Greenwich) sits on +X of the result by construction.
pub fn eci_to_ecef(v: DVec3, gmst: f64) -> DVec3 {
    let (s, c) = gmst.sin_cos();
    DVec3::new(c * v.x + s * v.y, c * v.y - s * v.x, v.z)
}

/// Inverse of [`eci_to_ecef`]: R_z(+GMST).
pub fn ecef_to_eci(v: DVec3, gmst: f64) -> DVec3 {
    let (s, c) = gmst.sin_cos();
    DVec3::new(c * v.x - s * v.y, c * v.y + s * v.x, v.z)
}

/// Earth-fixed velocity of an inertial state: the frame rotation plus the
/// transport term `-omega x r`, which is what makes a geostationary satellite
/// read as motionless rather than 3.07 km/s.
pub fn eci_to_ecef_vel(r: DVec3, v: DVec3, gmst: f64) -> DVec3 {
    let omega = DVec3::new(0.0, 0.0, OMEGA_EARTH);
    eci_to_ecef(v - omega.cross(r), gmst)
}

/// Unit vector from Earth's center to the Sun in the TEME/mean-equator frame.
/// Vallado's low-precision algorithm (Alg. 29).
pub fn sun_direction(epoch: Epoch) -> DVec3 {
    let t = (jd_utc(epoch) - 2_451_545.0) / 36_525.0;
    // Mean longitude and mean anomaly of the Sun, degrees.
    let lambda_mean = 280.460 + 36_000.771 * t;
    let m = (357.529_109_2 + 35_999.050_34 * t).to_radians();
    // Ecliptic longitude with equation-of-center correction.
    let lambda_ecl =
        (lambda_mean + 1.914_666_471 * m.sin() + 0.019_994_643 * (2.0 * m).sin()).to_radians();
    // Mean obliquity of the ecliptic.
    let eps = (23.439_291 - 0.013_004_2 * t).to_radians();
    DVec3::new(
        lambda_ecl.cos(),
        eps.cos() * lambda_ecl.sin(),
        eps.sin() * lambda_ecl.sin(),
    )
    .normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// GMST at the J2000.0 epoch is the textbook 280.4606°.
    #[test]
    fn gmst_at_j2000() {
        let epoch = Epoch::from_gregorian_utc_hms(2000, 1, 1, 12, 0, 0);
        let got = gmst_rad(epoch).to_degrees();
        assert!((got - 280.4606).abs() < 0.01, "GMST {got} deg");
    }

    /// At the March equinox the Sun sits at the vernal equinox direction
    /// (+X, RA ≈ 0, dec ≈ 0); 2026 equinox: Mar 20 ~14:46 UTC.
    #[test]
    fn sun_at_march_equinox() {
        let epoch = Epoch::from_gregorian_utc_hms(2026, 3, 20, 14, 46, 0);
        let s = sun_direction(epoch);
        assert!(s.x > 0.999, "sun {s:?} should point near +X");
        let dec_deg = s.z.asin().to_degrees();
        assert!(dec_deg.abs() < 0.3, "declination {dec_deg} deg");
    }

    /// At the June solstice the Sun's declination is +23.44°.
    #[test]
    fn sun_at_june_solstice() {
        let epoch = Epoch::from_gregorian_utc_hms(2026, 6, 21, 2, 25, 0);
        let s = sun_direction(epoch);
        let dec_deg = s.z.asin().to_degrees();
        assert!((dec_deg - 23.44).abs() < 0.1, "declination {dec_deg} deg");
    }

    /// The GMST rotation puts the Greenwich meridian on +X: an inertial
    /// vector at right ascension = GMST is Earth-fixed longitude 0.
    #[test]
    fn greenwich_lands_on_plus_x() {
        let gmst = 1.234_f64;
        let r_eci = DVec3::new(gmst.cos(), gmst.sin(), 0.0) * 6_378.0;
        let r_ecef = eci_to_ecef(r_eci, gmst);
        assert!(r_ecef.y.abs() < 1e-9, "{r_ecef:?} should be on +X");
        assert!((r_ecef.x - 6_378.0).abs() < 1e-9, "{r_ecef:?}");
    }

    /// Rotating about the pole preserves radius, and the inverse round-trips.
    #[test]
    fn rotation_preserves_radius_and_inverts() {
        let r = DVec3::new(1_234.0, -5_678.0, 3_456.0);
        let gmst = 2.5_f64;
        let back = ecef_to_eci(eci_to_ecef(r, gmst), gmst);
        assert!((eci_to_ecef(r, gmst).length() - r.length()).abs() < 1e-9);
        assert!((back - r).length() < 1e-9, "{back:?} vs {r:?}");
    }

    /// A geostationary satellite is motionless in the Earth-fixed frame:
    /// the transport term cancels its 3.07 km/s inertial speed, and its
    /// position does not move as the frame rotates with it.
    #[test]
    fn geostationary_is_stationary_in_ecef() {
        let a = 42_164.0;
        let n = (crate::MU_EARTH / (a * a * a)).sqrt();
        let r0 = DVec3::new(a, 0.0, 0.0);
        let v0 = DVec3::new(0.0, n * a, 0.0);
        let speed = eci_to_ecef_vel(r0, v0, 0.0).length();
        assert!(speed < 2.0e-3, "earth-fixed speed {speed} km/s");

        // A quarter day later the satellite and the frame have both turned.
        let dt = 21_600.0;
        let theta = n * dt;
        let r1 = DVec3::new(a * theta.cos(), a * theta.sin(), 0.0);
        let moved = (eci_to_ecef(r1, OMEGA_EARTH * dt) - r0).length();
        assert!(moved < 2.0, "drifted {moved} km in 6 h");
    }

    /// Self-consistency: at 12:00 UTC the subsolar longitude (Sun RA − GMST)
    /// is near Greenwich (within the equation of time, ±4°).
    #[test]
    fn subsolar_longitude_near_greenwich_at_noon_utc() {
        let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 12, 0, 0);
        let s = sun_direction(epoch);
        let ra = s.y.atan2(s.x);
        let lon = (ra - gmst_rad(epoch)).rem_euclid(TAU).to_degrees();
        let lon_signed = if lon > 180.0 { lon - 360.0 } else { lon };
        assert!(
            lon_signed.abs() < 4.5,
            "subsolar longitude {lon_signed} deg at noon UTC"
        );
    }
}
