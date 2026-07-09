//! Delta-v budget checks, shortfall quantification, and closest-achievable
//! orbit computation.

use super::target::{SolveConstraints, TargetOrbit};
use super::templates::enumerate_templates;
use crate::{OrbitalElements, Vehicle, G0};

/// The vehicle's total delta-v capability, km/s (infinite-fuel returns +inf).
pub fn dv_budget_km_s(vehicle: &Vehicle, infinite_fuel: bool) -> f64 {
    if infinite_fuel {
        return f64::INFINITY;
    }
    vehicle.ve_km_s() * (vehicle.wet_mass_kg / vehicle.dry_mass_kg).ln()
}

/// Propellant consumed by a total delta-v, kg (rocket equation).
pub fn prop_for_dv_kg(vehicle: &Vehicle, dv_km_s: f64) -> f64 {
    vehicle.wet_mass_kg * (1.0 - (-dv_km_s / vehicle.ve_km_s()).exp())
}

/// Quantified vehicle-limitation report.
#[derive(Debug, Clone, PartialEq)]
pub struct Shortfall {
    pub required_dv_km_s: f64,
    pub available_dv_km_s: f64,
    pub shortfall_dv_km_s: f64,
    /// Additional propellant (kg, added to the wet mass with the same tank
    /// dry mass) that would close the gap.
    pub extra_prop_kg: f64,
    /// Isp (s) that would close the gap with the current masses.
    pub isp_required_s: f64,
    /// Closest orbit along (current -> target) reachable with the budget,
    /// as resolved (a, e, i) values.
    pub closest_achievable: (f64, f64, f64),
}

/// Cheapest impulsive cost to reach (a2, e2, i2), or +inf if no template
/// survives constraints.
pub fn best_template_dv(
    current: &OrbitalElements,
    a2: f64,
    e2: f64,
    i2: f64,
    constraints: &SolveConstraints,
    accel_m_s2: f64,
) -> f64 {
    enumerate_templates(current, a2, e2, i2, constraints, accel_m_s2)
        .iter()
        .map(|p| p.total_dv_km_s)
        .fold(f64::INFINITY, f64::min)
}

/// Build the shortfall report for a target whose best plan exceeds budget.
pub fn shortfall(
    current: &OrbitalElements,
    target: &TargetOrbit,
    vehicle: &Vehicle,
    constraints: &SolveConstraints,
    required_dv_km_s: f64,
    available_dv_km_s: f64,
) -> Shortfall {
    let ve = vehicle.ve_km_s();
    // Extra prop x such that ve*ln((m0+x)/m_dry) = required.
    let extra_prop_kg =
        (vehicle.dry_mass_kg * (required_dv_km_s / ve).exp() - vehicle.wet_mass_kg).max(0.0);
    // Isp such that the current mass ratio suffices.
    let mass_ratio_ln = (vehicle.wet_mass_kg / vehicle.dry_mass_kg).ln();
    let isp_required_s = if mass_ratio_ln > 0.0 {
        required_dv_km_s * 1000.0 / (G0 * mass_ratio_ln)
    } else {
        f64::INFINITY
    };

    // Bisect the largest fraction s of the element delta that fits budget.
    let (a2, e2, i2) = target.resolved(current);
    let accel = vehicle.thrust_n / vehicle.wet_mass_kg;
    let cost_at = |s: f64| {
        let a = current.sma_km + s * (a2 - current.sma_km);
        let e = current.ecc + s * (e2 - current.ecc);
        let i = current.inc_deg + s * (i2 - current.inc_deg);
        best_template_dv(current, a, e, i, constraints, accel)
    };
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    for _ in 0..32 {
        let mid = (lo + hi) / 2.0;
        if cost_at(mid) <= available_dv_km_s {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let closest_achievable = (
        current.sma_km + lo * (a2 - current.sma_km),
        current.ecc + lo * (e2 - current.ecc),
        current.inc_deg + lo * (i2 - current.inc_deg),
    );

    Shortfall {
        required_dv_km_s,
        available_dv_km_s,
        shortfall_dv_km_s: required_dv_km_s - available_dv_km_s,
        extra_prop_kg,
        isp_required_s,
        closest_achievable,
    }
}
