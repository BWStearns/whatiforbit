use crate::{state::vnc_to_inertial, J2, MU_EARTH, R_EARTH};
use glam::DVec3;

/// Integration state vector layout: [rx, ry, rz, vx, vy, vz, mass].
pub type Y = [f64; 7];

pub fn pack(r: DVec3, v: DVec3, mass: f64) -> Y {
    [r.x, r.y, r.z, v.x, v.y, v.z, mass]
}

pub fn unpack(y: &Y) -> (DVec3, DVec3, f64) {
    (
        DVec3::new(y[0], y[1], y[2]),
        DVec3::new(y[3], y[4], y[5]),
        y[6],
    )
}

/// Point-mass gravity, optionally plus the J2 zonal harmonic (Vallado,
/// eq. 8-30). Input km, output km/s^2. The J2 z-axis is the inertial z axis.
pub fn gravity_accel(r: DVec3, with_j2: bool) -> DVec3 {
    let rn = r.length();
    let two_body = -MU_EARTH / (rn * rn * rn) * r;
    if !with_j2 {
        return two_body;
    }

    let k = -1.5 * J2 * MU_EARTH * R_EARTH * R_EARTH / rn.powi(5);
    let z2_r2 = (r.z * r.z) / (rn * rn);
    let j2 = DVec3::new(
        k * r.x * (1.0 - 5.0 * z2_r2),
        k * r.y * (1.0 - 5.0 * z2_r2),
        k * r.z * (3.0 - 5.0 * z2_r2),
    );

    two_body + j2
}

/// Continuous thrust description for a burn arc. Direction is fixed in the
/// VNC frame and re-evaluated against the instantaneous state, i.e. a
/// velocity-tracking (not inertially-fixed) burn attitude.
#[derive(Debug, Clone, Copy)]
pub struct ThrustArc {
    /// Unit direction in VNC components.
    pub direction_vnc: DVec3,
    /// Engine thrust in newtons, already scaled by throttle.
    pub thrust_n: f64,
    /// Propellant mass flow, kg/s (0 for the infinite-fuel cheat).
    pub mass_flow_kg_s: f64,
}

/// Equations of motion: coast or powered flight.
pub fn eom(y: &Y, thrust: Option<&ThrustArc>, with_j2: bool) -> Y {
    let (r, v, mass) = unpack(y);
    let mut a = gravity_accel(r, with_j2);
    let mut mdot = 0.0;

    if let Some(t) = thrust {
        let dir = vnc_to_inertial(r, v, t.direction_vnc).normalize();
        // N / kg = m/s^2 -> km/s^2
        a += dir * (t.thrust_n / mass / 1000.0);
        mdot = -t.mass_flow_kg_s;
    }

    [v.x, v.y, v.z, a.x, a.y, a.z, mdot]
}
