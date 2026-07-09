//! Target-orbit solver: given the current state, vehicle, and a target
//! (a, e, i), enumerate transfer candidates, check feasibility, and (M2)
//! refine them into finite-burn plans.
//!
//! See PLAN-target-orbit.md for the architecture and scope decisions.

pub mod feasibility;
pub mod kepler;
pub mod optimize;
pub mod refine;
pub mod target;
pub mod templates;

use crate::{elements_from_rv, Cheats, State, Vehicle};
pub use feasibility::{dv_budget_km_s, prop_for_dv_kg, Shortfall};
pub use refine::{RefineContext, RefineStatus, RefinedPlan, Refiner};
pub use target::{ElementTarget, SolveConstraints, TargetInvalid, TargetOrbit};
pub use templates::{ImpulsiveBurn, ImpulsivePlan, PlaneChangeSite, StrategyKind};

/// Which constrained objective ranks the candidates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveMode {
    /// Minimize propellant (delta-v), optionally under a deadline.
    Cheapest,
    /// Minimize arrival time, within available propellant.
    Fastest,
}

/// One transfer candidate with its user-facing costs.
#[derive(Debug, Clone, PartialEq)]
pub struct Candidate {
    pub plan: ImpulsivePlan,
    pub prop_kg: f64,
    /// Fits the vehicle's delta-v budget.
    pub within_budget: bool,
    /// Meets the deadline, when one is set.
    pub within_deadline: bool,
    /// Every burn arc fits the discrete-burn envelope for this vehicle
    /// (impulse realizable as a finite burn shorter than the cap).
    pub within_burn_envelope: bool,
}

impl Candidate {
    pub fn feasible(&self) -> bool {
        self.within_budget && self.within_deadline && self.within_burn_envelope
    }
}

/// Solver outcome: candidates (feasible and not, for display) + verdict.
#[derive(Debug, Clone, PartialEq)]
pub struct SolveResult {
    pub candidates: Vec<Candidate>,
    pub verdict: Verdict,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    /// At least one candidate is fully feasible.
    Feasible,
    /// The target itself is malformed.
    InvalidTarget(TargetInvalid),
    /// No strategy fits the vehicle's delta-v budget.
    VehicleLimited(Shortfall),
    /// Only the low-thrust spiral regime reaches it (burn arcs too long for
    /// the discrete-burn model). `estimate` is Edelbaum (dv km/s, duration s)
    /// when the geometry is near-circular enough to quote one.
    LowThrustOnly { estimate: Option<(f64, f64)> },
    /// Reachable, but not within the requested deadline.
    DeadlineMiss { fastest_s: f64 },
}

/// Build the refinement context for a candidate of this solve. Unconstrained
/// elements get infinite tolerance (their residual vanishes).
pub fn refine_context(
    state: &State,
    vehicle: &Vehicle,
    cheats: &Cheats,
    enable_j2: bool,
    target: &TargetOrbit,
    mode: SolveMode,
    constraints: &SolveConstraints,
) -> RefineContext {
    let current = elements_from_rv(state.r, state.v);
    let pick = |t: Option<ElementTarget>, current_v: f64| -> (f64, f64) {
        t.map_or((current_v, f64::INFINITY), |t| (t.value, t.tol))
    };
    RefineContext {
        init: *state,
        vehicle: *vehicle,
        cheats: *cheats,
        enable_j2,
        target_a: pick(target.sma_km, current.sma_km),
        target_e: pick(target.ecc, current.ecc),
        target_i: pick(target.inc_deg, current.inc_deg),
        constraints: *constraints,
        minimize_time: mode == SolveMode::Fastest,
    }
}

/// Enumerate, cost, and rank impulsive candidates (M1 layer). The finite-burn
/// refinement (M2) consumes these as seeds.
pub fn solve_impulsive(
    state: &State,
    vehicle: &Vehicle,
    cheats: &Cheats,
    target: &TargetOrbit,
    mode: SolveMode,
    constraints: &SolveConstraints,
) -> SolveResult {
    let current = elements_from_rv(state.r, state.v);

    if let Err(invalid) = target.validate(&current, constraints.min_transfer_perigee_alt_km) {
        return SolveResult {
            candidates: vec![],
            verdict: Verdict::InvalidTarget(invalid),
        };
    }

    let (a2, e2, i2) = target.resolved(&current);
    let accel_m_s2 = vehicle.thrust_n / vehicle.wet_mass_kg;
    let budget = dv_budget_km_s(vehicle, cheats.infinite_fuel);

    let plans = templates::enumerate_templates(&current, a2, e2, i2, constraints, accel_m_s2);

    let mut candidates: Vec<Candidate> = plans
        .into_iter()
        .map(|plan| {
            // The infinite-fuel cheat consumes nothing; show that honestly.
            let prop_kg = if cheats.infinite_fuel {
                0.0
            } else {
                prop_for_dv_kg(vehicle, plan.total_dv_km_s)
            };
            let within_budget = plan.total_dv_km_s <= budget + 1e-12;
            let within_deadline = constraints
                .deadline_s
                .is_none_or(|d| plan.duration_s <= d);
            // Burn-envelope: realize each impulse as a full-throttle finite
            // burn and compare to the cap (fraction of the local period).
            // The infinite-fuel cheat suspends this vehicle-reality gate too —
            // it exists to flag where the impulsive model degrades, and cheat
            // mode is explicitly "let me try it anyway".
            let within_burn_envelope = match plan.kind {
                StrategyKind::EdelbaumSpiral => false, // estimate, not a plan
                _ if cheats.infinite_fuel => true,
                _ => plan.burns.iter().all(|b| {
                    let dv = b.dv_mag();
                    if dv < 1e-9 {
                        return true;
                    }
                    let prop = prop_for_dv_kg(vehicle, dv);
                    let mdot = vehicle.thrust_n / (vehicle.isp_s * crate::G0);
                    let burn_s = if cheats.infinite_fuel {
                        // constant-mass burn time: dv = F*t/m
                        dv * 1000.0 * vehicle.wet_mass_kg / vehicle.thrust_n.max(1e-9)
                    } else if mdot > 0.0 {
                        prop / mdot
                    } else {
                        f64::INFINITY
                    };
                    let local_period = kepler::period_s(b.radius_km);
                    burn_s <= constraints.max_burn_period_fraction * local_period
                }),
            };
            Candidate {
                plan,
                prop_kg,
                within_budget,
                within_deadline,
                within_burn_envelope,
            }
        })
        .collect();

    // Rank by mode: feasible first, then the mode's cost, then fewer burns.
    let sort_key = |c: &Candidate| -> (u8, f64, usize) {
        let cost = match mode {
            SolveMode::Cheapest => c.plan.total_dv_km_s,
            SolveMode::Fastest => c.plan.duration_s,
        };
        (u8::from(!c.feasible()), cost, c.plan.burns.len())
    };
    candidates.sort_by(|x, y| {
        let (fx, cx, bx) = sort_key(x);
        let (fy, cy, by) = sort_key(y);
        fx.cmp(&fy).then(cx.total_cmp(&cy)).then(bx.cmp(&by))
    });

    // Verdict, in order of severity.
    let verdict = if candidates.iter().any(Candidate::feasible) {
        Verdict::Feasible
    } else if candidates.iter().all(|c| !c.within_budget) {
        let required = candidates
            .iter()
            .map(|c| c.plan.total_dv_km_s)
            .fold(f64::INFINITY, f64::min);
        Verdict::VehicleLimited(feasibility::shortfall(
            &current,
            target,
            vehicle,
            constraints,
            required,
            budget,
        ))
    } else if candidates
        .iter()
        .any(|c| c.within_budget && c.within_burn_envelope && !c.within_deadline)
    {
        let fastest = candidates
            .iter()
            .filter(|c| c.within_budget && c.within_burn_envelope)
            .map(|c| c.plan.duration_s)
            .fold(f64::INFINITY, f64::min);
        Verdict::DeadlineMiss { fastest_s: fastest }
    } else {
        // Budget fits somewhere and no deadline blocks, but every discrete
        // plan needs burn arcs longer than the envelope: low-thrust regime.
        let estimate = candidates
            .iter()
            .find(|c| matches!(c.plan.kind, StrategyKind::EdelbaumSpiral))
            .map(|c| (c.plan.total_dv_km_s, c.plan.duration_s));
        Verdict::LowThrustOnly { estimate }
    };

    SolveResult {
        candidates,
        verdict,
    }
}
