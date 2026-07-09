//! M2 gate tests: finite-burn refinement converges from impulsive seeds and
//! lands inside target tolerances against the real propagator.

use glam::DVec3;
use hifitime::Epoch;
use whatiforbit_sim::targeting::{
    refine_context, solve_impulsive, RefineStatus, Refiner, SolveConstraints, SolveMode,
    StrategyKind, TargetOrbit, Verdict,
};
use whatiforbit_sim::{Cheats, State, Vehicle, MU_EARTH, R_EARTH};

fn circular_state(radius_km: f64) -> State {
    let v = (MU_EARTH / radius_km).sqrt();
    State::new(
        Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0),
        DVec3::new(radius_km, 0.0, 0.0),
        DVec3::new(0.0, v, 0.0),
        1000.0,
    )
}

fn strong_vehicle() -> Vehicle {
    Vehicle {
        wet_mass_kg: 5_000.0,
        dry_mass_kg: 1_000.0,
        thrust_n: 50_000.0,
        isp_s: 320.0,
    }
}

/// Refine the best discrete candidate to convergence; panics on failure.
fn refine_best(
    state: &State,
    vehicle: &Vehicle,
    target: &TargetOrbit,
    mode: SolveMode,
) -> whatiforbit_sim::targeting::RefinedPlan {
    let constraints = SolveConstraints::default();
    let cheats = Cheats::default();
    let solve = solve_impulsive(state, vehicle, &cheats, target, mode, &constraints);
    assert_eq!(solve.verdict, Verdict::Feasible, "impulsive layer infeasible");
    let seed = solve
        .candidates
        .iter()
        .find(|c| c.feasible() && !matches!(c.plan.kind, StrategyKind::EdelbaumSpiral))
        .expect("a feasible discrete candidate");

    let ctx = refine_context(state, vehicle, &cheats, true, target, mode, &constraints);
    let mut refiner = Refiner::new(ctx, &seed.plan).expect("refiner seeds");
    for _ in 0..200 {
        match refiner.step(2_000) {
            RefineStatus::Converged(plan) => return plan,
            RefineStatus::Failed(e) => panic!("refinement failed: {e}"),
            RefineStatus::Running { .. } => {}
        }
    }
    panic!("refinement did not finish within evaluation budget");
}

/// Coplanar raise 7000 -> 9000 km circular: the finite-burn solution must hit
/// the target elements within tolerance and cost close to the impulsive
/// prediction (small gravity losses at this thrust).
#[test]
fn finite_burn_hohmann_converges_near_impulsive() {
    let r1 = 7000.0;
    let r2 = 9000.0;
    let state = circular_state(r1);
    let vehicle = strong_vehicle();
    let target = TargetOrbit::from_apo_peri_inc(r2 - R_EARTH, r2 - R_EARTH, 0.0);

    // Impulsive reference.
    let a_t = (r1 + r2) / 2.0;
    let dv_imp = ((MU_EARTH * (2.0 / r1 - 1.0 / a_t)).sqrt() - (MU_EARTH / r1).sqrt())
        + ((MU_EARTH / r2).sqrt() - (MU_EARTH * (2.0 / r2 - 1.0 / a_t)).sqrt());

    let plan = refine_best(&state, &vehicle, &target, SolveMode::Cheapest);

    let (a, e, _i) = plan.achieved;
    assert!((a - r2).abs() < 5.0, "achieved SMA {a} vs {r2}");
    assert!(e < 1e-3, "achieved ecc {e}");
    assert!(
        plan.dv_km_s > 0.97 * dv_imp && plan.dv_km_s < 1.06 * dv_imp,
        "finite-burn dv {} vs impulsive {dv_imp}",
        plan.dv_km_s
    );
    assert_eq!(plan.burns.len(), 2);
}

/// Adding a 2-degree plane change: converged inclination must match.
#[test]
fn finite_burn_with_plane_change() {
    let r1 = 7000.0;
    let r2 = 9000.0;
    let state = circular_state(r1);
    let vehicle = strong_vehicle();
    let target = TargetOrbit::from_apo_peri_inc(r2 - R_EARTH, r2 - R_EARTH, 2.0);

    let plan = refine_best(&state, &vehicle, &target, SolveMode::Cheapest);
    let (a, e, i) = plan.achieved;
    assert!((a - r2).abs() < 5.0, "achieved SMA {a}");
    assert!(e < 1e-3, "achieved ecc {e}");
    assert!((i - 2.0).abs() < 0.05, "achieved inc {i}");
}

/// Low-thrust-chemical regression (the case a user hit): 400 N on 1000 kg
/// from ISS altitude to 800 km — burns run ~250 s each, and earlier optimizer
/// tuning (unconditional mu escalation) stalled one residual-width from
/// feasible. Must converge near the impulsive cost.
#[test]
fn finite_burn_weak_vehicle_converges() {
    let r1 = 6798.0;
    let r2 = 7178.0;
    let state = circular_state(r1);
    let vehicle = Vehicle {
        wet_mass_kg: 1000.0,
        dry_mass_kg: 600.0,
        thrust_n: 400.0,
        isp_s: 300.0,
    };
    let target = TargetOrbit::from_apo_peri_inc(r2 - R_EARTH, r2 - R_EARTH, 0.0);

    let a_t = (r1 + r2) / 2.0;
    let dv_imp = ((MU_EARTH * (2.0 / r1 - 1.0 / a_t)).sqrt() - (MU_EARTH / r1).sqrt())
        + ((MU_EARTH / r2).sqrt() - (MU_EARTH * (2.0 / r2 - 1.0 / a_t)).sqrt());

    let plan = refine_best(&state, &vehicle, &target, SolveMode::Cheapest);
    let (a, e, _) = plan.achieved;
    assert!((a - r2).abs() < 10.0, "achieved SMA {a} vs {r2}");
    assert!(e < 2e-3, "achieved ecc {e}");
    assert!(
        plan.dv_km_s < 1.05 * dv_imp,
        "weak-vehicle dv {} vs impulsive {dv_imp}",
        plan.dv_km_s
    );
}

/// Same seed, same answer: the pipeline has no hidden nondeterminism.
#[test]
fn refinement_is_deterministic() {
    let state = circular_state(7000.0);
    let vehicle = strong_vehicle();
    let target = TargetOrbit::from_apo_peri_inc(9000.0 - R_EARTH, 9000.0 - R_EARTH, 0.0);
    let p1 = refine_best(&state, &vehicle, &target, SolveMode::Cheapest);
    let p2 = refine_best(&state, &vehicle, &target, SolveMode::Cheapest);
    assert_eq!(p1.burns, p2.burns);
    assert_eq!(p1.prop_kg, p2.prop_kg);
}
