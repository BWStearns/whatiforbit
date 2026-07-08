use glam::DVec3;
use hifitime::Epoch;

/// Spacecraft translational state plus total mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub epoch: Epoch,
    /// Position, km, inertial (TEME-at-epoch).
    pub r: DVec3,
    /// Velocity, km/s, inertial.
    pub v: DVec3,
    /// Total vehicle mass, kg.
    pub mass: f64,
}

impl State {
    pub fn new(epoch: Epoch, r: DVec3, v: DVec3, mass: f64) -> Self {
        Self { epoch, r, v, mass }
    }
}

/// Orthonormal VNC (velocity / normal / co-normal) basis at a given state.
///
/// V = velocity direction, N = orbit normal (r x v), C = V x N which points
/// roughly along the outward radial for near-circular orbits. Maneuver
/// components are expressed in this frame and rotated to inertial here.
pub fn vnc_basis(r: DVec3, v: DVec3) -> (DVec3, DVec3, DVec3) {
    let v_hat = v.normalize();
    let n_hat = r.cross(v).normalize();
    let c_hat = v_hat.cross(n_hat);
    (v_hat, n_hat, c_hat)
}

/// Rotate a vector expressed in VNC components into the inertial frame.
pub fn vnc_to_inertial(r: DVec3, v: DVec3, vnc: DVec3) -> DVec3 {
    let (v_hat, n_hat, c_hat) = vnc_basis(r, v);
    v_hat * vnc.x + n_hat * vnc.y + c_hat * vnc.z
}
