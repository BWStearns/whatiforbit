//! Target-orbit specification and validation.
//!
//! Per-element-optional by design (see PLAN-target-orbit.md forward
//! compatibility): elements left `None` are unconstrained, and future targets
//! (RAAN, argp, phase) become new optional fields + residuals, not an API
//! break. All element constraints are interpreted as *mean elements at
//! arrival epoch*.

use crate::{OrbitalElements, R_EARTH};

/// One targeted element: desired value and acceptance tolerance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElementTarget {
    pub value: f64,
    pub tol: f64,
}

/// The target orbit. Units match `OrbitalElements` (km, dimensionless, deg).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TargetOrbit {
    pub sma_km: Option<ElementTarget>,
    pub ecc: Option<ElementTarget>,
    pub inc_deg: Option<ElementTarget>,
}

/// Default acceptance tolerances. Sized for finite-burn reality: with
/// minutes-long burn arcs (low-thrust chemical vehicles), constant-direction
/// arcs can't null residuals to arbitrary precision, and ±10 km / ±2e-3 is
/// well within what a what-if tool needs.
pub const DEFAULT_SMA_TOL_KM: f64 = 10.0;
pub const DEFAULT_ECC_TOL: f64 = 2e-3;
pub const DEFAULT_INC_TOL_DEG: f64 = 0.05;

/// Altitude floor below which a target (or transfer) periapsis is rejected.
pub const DEFAULT_PERIGEE_FLOOR_ALT_KM: f64 = 150.0;

#[derive(Debug, Clone, PartialEq)]
pub enum TargetInvalid {
    /// Apoapsis altitude below periapsis altitude.
    ApoBelowPeri { apo_alt_km: f64, peri_alt_km: f64 },
    /// Target periapsis dips below the atmosphere floor.
    PeriapsisBelowFloor { peri_alt_km: f64, floor_alt_km: f64 },
    /// Eccentricity outside [0, 1).
    NotElliptical { ecc: f64 },
    /// No elements targeted at all.
    NothingTargeted,
}

impl std::fmt::Display for TargetInvalid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TargetInvalid::ApoBelowPeri {
                apo_alt_km,
                peri_alt_km,
            } => write!(
                f,
                "apoapsis altitude ({apo_alt_km:.1} km) is below periapsis ({peri_alt_km:.1} km)"
            ),
            TargetInvalid::PeriapsisBelowFloor {
                peri_alt_km,
                floor_alt_km,
            } => write!(
                f,
                "periapsis altitude {peri_alt_km:.1} km is below the {floor_alt_km:.0} km floor"
            ),
            TargetInvalid::NotElliptical { ecc } => {
                write!(f, "eccentricity {ecc:.4} is not a closed orbit in [0, 1)")
            }
            TargetInvalid::NothingTargeted => write!(f, "no target elements specified"),
        }
    }
}

impl TargetOrbit {
    /// Build from user-facing apoapsis/periapsis altitudes + inclination.
    pub fn from_apo_peri_inc(apo_alt_km: f64, peri_alt_km: f64, inc_deg: f64) -> Self {
        let ra = apo_alt_km + R_EARTH;
        let rp = peri_alt_km + R_EARTH;
        let a = (ra + rp) / 2.0;
        // Deliberately unclamped: an inverted apo/peri pair yields e < 0,
        // which validation reports as ApoBelowPeri instead of silently
        // "fixing" the user's input.
        let e = (ra - rp) / (ra + rp);
        Self {
            sma_km: Some(ElementTarget {
                value: a,
                tol: DEFAULT_SMA_TOL_KM,
            }),
            ecc: Some(ElementTarget {
                value: e,
                tol: DEFAULT_ECC_TOL,
            }),
            inc_deg: Some(ElementTarget {
                value: inc_deg,
                tol: DEFAULT_INC_TOL_DEG,
            }),
        }
    }

    /// Resolve unset elements against the current orbit ("keep as is").
    pub fn resolved(&self, current: &OrbitalElements) -> (f64, f64, f64) {
        (
            self.sma_km.map_or(current.sma_km, |t| t.value),
            self.ecc.map_or(current.ecc, |t| t.value),
            self.inc_deg.map_or(current.inc_deg, |t| t.value),
        )
    }

    pub fn validate(
        &self,
        current: &OrbitalElements,
        floor_alt_km: f64,
    ) -> Result<(), TargetInvalid> {
        if self.sma_km.is_none() && self.ecc.is_none() && self.inc_deg.is_none() {
            return Err(TargetInvalid::NothingTargeted);
        }
        let (a, e, _) = self.resolved(current);
        let peri_alt = a * (1.0 - e) - R_EARTH;
        let apo_alt = a * (1.0 + e) - R_EARTH;
        if apo_alt < peri_alt - 1e-9 {
            return Err(TargetInvalid::ApoBelowPeri {
                apo_alt_km: apo_alt,
                peri_alt_km: peri_alt,
            });
        }
        if !(0.0..1.0).contains(&e) {
            return Err(TargetInvalid::NotElliptical { ecc: e });
        }
        if peri_alt < floor_alt_km {
            return Err(TargetInvalid::PeriapsisBelowFloor {
                peri_alt_km: peri_alt,
                floor_alt_km,
            });
        }
        Ok(())
    }
}

/// Solver-wide constraints and knobs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolveConstraints {
    /// Transfer arcs must keep periapsis above this altitude.
    pub min_transfer_perigee_alt_km: f64,
    /// Cap on intermediate apoapsis altitude (bounds bi-elliptic sweeps).
    pub max_intermediate_apo_alt_km: f64,
    /// Optional arrival deadline, seconds from scenario epoch.
    pub deadline_s: Option<f64>,
    /// A single burn arc longer than this fraction of the local orbital
    /// period is considered outside the discrete-burn envelope.
    pub max_burn_period_fraction: f64,
}

impl Default for SolveConstraints {
    fn default() -> Self {
        Self {
            min_transfer_perigee_alt_km: DEFAULT_PERIGEE_FLOOR_ALT_KM,
            max_intermediate_apo_alt_km: 200_000.0,
            deadline_s: None,
            max_burn_period_fraction: 0.25,
        }
    }
}
