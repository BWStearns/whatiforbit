use crate::{MU_EARTH, R_EARTH};
use glam::DVec3;
use std::f64::consts::{PI, TAU};

/// Classical (Keplerian) orbital elements derived from an inertial state.
/// Angles in degrees for display convenience.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitalElements {
    pub sma_km: f64,
    pub ecc: f64,
    pub inc_deg: f64,
    pub raan_deg: f64,
    pub argp_deg: f64,
    pub true_anomaly_deg: f64,
    /// Orbital period, seconds. None for parabolic/hyperbolic.
    pub period_s: Option<f64>,
    /// Apoapsis altitude above the equatorial radius, km. None if escaping.
    pub apoapsis_alt_km: Option<f64>,
    pub periapsis_alt_km: f64,
}

/// Vallado's RV -> COE algorithm with the usual degenerate-case fallbacks
/// (circular and/or equatorial orbits).
pub fn elements_from_rv(r: DVec3, v: DVec3) -> OrbitalElements {
    let rn = r.length();
    let vn = v.length();
    let h = r.cross(v);
    let hn = h.length();
    let n = DVec3::Z.cross(h); // node vector
    let nn = n.length();

    let e_vec = ((vn * vn - MU_EARTH / rn) * r - r.dot(v) * v) / MU_EARTH;
    let ecc = e_vec.length();

    let energy = vn * vn / 2.0 - MU_EARTH / rn;
    let sma = -MU_EARTH / (2.0 * energy);

    let inc = (h.z / hn).clamp(-1.0, 1.0).acos();

    let circular = ecc < 1e-8;
    let equatorial = inc < 1e-8 || (PI - inc) < 1e-8;

    let raan = if equatorial {
        0.0
    } else {
        let mut o = (n.x / nn).clamp(-1.0, 1.0).acos();
        if n.y < 0.0 {
            o = TAU - o;
        }
        o
    };

    let argp = if circular {
        0.0
    } else if equatorial {
        // Longitude of periapsis measured from x-axis.
        let mut w = (e_vec.x / ecc).clamp(-1.0, 1.0).acos();
        if e_vec.y < 0.0 {
            w = TAU - w;
        }
        w
    } else {
        let mut w = (n.dot(e_vec) / (nn * ecc)).clamp(-1.0, 1.0).acos();
        if e_vec.z < 0.0 {
            w = TAU - w;
        }
        w
    };

    let nu = if circular {
        // Argument of latitude (from node) or true longitude if equatorial.
        let ref_vec = if equatorial { DVec3::X } else { n / nn };
        let mut u = (ref_vec.dot(r) / rn).clamp(-1.0, 1.0).acos();
        let sign = if equatorial { r.y } else { r.z };
        if sign < 0.0 {
            u = TAU - u;
        }
        u
    } else {
        let mut nu = (e_vec.dot(r) / (ecc * rn)).clamp(-1.0, 1.0).acos();
        if r.dot(v) < 0.0 {
            nu = TAU - nu;
        }
        nu
    };

    let elliptical = ecc < 1.0 && sma > 0.0;
    let period = elliptical.then(|| TAU * (sma.powi(3) / MU_EARTH).sqrt());
    let apo = elliptical.then_some(sma * (1.0 + ecc) - R_EARTH);
    let peri = if elliptical {
        sma * (1.0 - ecc) - R_EARTH
    } else {
        // For hyperbolic orbits periapsis radius is still a*(1-e) with a<0.
        sma * (1.0 - ecc) - R_EARTH
    };

    OrbitalElements {
        sma_km: sma,
        ecc,
        inc_deg: inc.to_degrees(),
        raan_deg: raan.to_degrees(),
        argp_deg: argp.to_degrees(),
        true_anomaly_deg: nu.to_degrees(),
        period_s: period,
        apoapsis_alt_km: apo,
        periapsis_alt_km: peri,
    }
}
