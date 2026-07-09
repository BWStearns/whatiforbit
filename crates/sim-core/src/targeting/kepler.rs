//! Keplerian timing helpers: converting between true anomaly and time along
//! an unperturbed ellipse. Used by the impulsive templates to place burns.

use crate::MU_EARTH;
use std::f64::consts::TAU;

/// Orbital period, seconds.
pub fn period_s(a_km: f64) -> f64 {
    TAU * (a_km.powi(3) / MU_EARTH).sqrt()
}

/// Speed on an orbit with semi-major axis `a` at radius `r` (vis-viva), km/s.
pub fn visviva_speed(a_km: f64, r_km: f64) -> f64 {
    (MU_EARTH * (2.0 / r_km - 1.0 / a_km)).max(0.0).sqrt()
}

/// Mean anomaly from true anomaly, radians (elliptical only).
pub fn mean_from_true(ecc: f64, nu_rad: f64) -> f64 {
    let e_anom = 2.0 * ((1.0 - ecc).sqrt() * (nu_rad / 2.0).sin())
        .atan2((1.0 + ecc).sqrt() * (nu_rad / 2.0).cos());
    let m = e_anom - ecc * e_anom.sin();
    m.rem_euclid(TAU)
}

/// Coast time from true anomaly `nu_from` forward to `nu_to`, seconds.
/// Always non-negative (goes the long way around if needed).
pub fn time_between_anomalies_s(a_km: f64, ecc: f64, nu_from_rad: f64, nu_to_rad: f64) -> f64 {
    let n = (MU_EARTH / a_km.powi(3)).sqrt();
    let dm = (mean_from_true(ecc, nu_to_rad) - mean_from_true(ecc, nu_from_rad)).rem_euclid(TAU);
    dm / n
}
