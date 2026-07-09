//! WhatIfOrbit simulation core.
//!
//! Pure physics library — no rendering dependencies — so it can be unit-tested
//! natively and reused from the Bevy/WASM front end.
//!
//! Units throughout: kilometers, km/s, kilograms, seconds.
//! Frame: the inertial frame is TEME-at-epoch as produced by SGP4. For the
//! what-if fidelity of this MVP (two-body + J2) the ~0.3° offset from GCRF is
//! accepted and documented rather than corrected.

pub mod dynamics;
pub mod earth;
pub mod elements;
pub mod integrator;
pub mod opm;
pub mod scenario;
pub mod state;
pub mod targeting;
pub mod tle;

pub use elements::{elements_from_rv, OrbitalElements};
pub use opm::opm_kvn;
pub use scenario::{Cheats, Maneuver, ManeuverReport, Scenario, SimResult, Trajectory, Vehicle};
pub use state::State;
pub use tle::{parse_tle, TleState};

/// Earth gravitational parameter, km^3/s^2 (JGM-3).
pub const MU_EARTH: f64 = 398_600.441_8;
/// Earth equatorial radius, km.
pub const R_EARTH: f64 = 6_378.136_3;
/// Earth J2 zonal harmonic (dimensionless).
pub const J2: f64 = 1.082_626_68e-3;
/// Standard gravity, m/s^2 (used for Isp -> exhaust velocity).
pub const G0: f64 = 9.806_65;
