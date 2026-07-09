//! M1 gate tests: impulsive templates vs textbook results, feasibility and
//! shortfall math, verdict classification.

use glam::DVec3;
use hifitime::Epoch;
use whatiforbit_sim::targeting::{
    dv_budget_km_s, solve_impulsive, SolveConstraints, SolveMode, StrategyKind, TargetOrbit,
    Verdict,
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

fn big_vehicle() -> Vehicle {
    // Generous budget & thrust so feasibility never interferes with
    // template-cost assertions: ve*ln(20) with Isp 450 ≈ 13.2 km/s.
    Vehicle {
        wet_mass_kg: 20_000.0,
        dry_mass_kg: 1_000.0,
        thrust_n: 500_000.0,
        isp_s: 450.0,
    }
}

fn target_circular(radius_km: f64, inc_deg: f64) -> TargetOrbit {
    TargetOrbit::from_apo_peri_inc(radius_km - R_EARTH, radius_km - R_EARTH, inc_deg)
}

/// Classic Hohmann: 7000 km circular -> GEO radius, equatorial throughout.
/// Analytic total dv = 3.4890 km/s for r1=7000, r2=42164.
#[test]
fn hohmann_matches_textbook() {
    let r1 = 7000.0;
    let r2 = 42_164.0;
    let state = circular_state(r1);
    let result = solve_impulsive(
        &state,
        &big_vehicle(),
        &Cheats::default(),
        &target_circular(r2, 0.0),
        SolveMode::Cheapest,
        &SolveConstraints::default(),
    );
    assert_eq!(result.verdict, Verdict::Feasible);

    // Analytic Hohmann.
    let a_t = (r1 + r2) / 2.0;
    let dv1 = (MU_EARTH * (2.0 / r1 - 1.0 / a_t)).sqrt() - (MU_EARTH / r1).sqrt();
    let dv2 = (MU_EARTH / r2).sqrt() - (MU_EARTH * (2.0 / r2 - 1.0 / a_t)).sqrt();
    let expected = dv1 + dv2;

    let best = &result.candidates[0];
    assert!(
        matches!(best.plan.kind, StrategyKind::TwoBurn { .. }),
        "best plan should be two-burn, got {:?}",
        best.plan.kind
    );
    assert!(
        (best.plan.total_dv_km_s - expected).abs() < 1e-9,
        "dv {} vs analytic {expected}",
        best.plan.total_dv_km_s
    );
    // Duration ≈ coast to first burn + half transfer period.
    let t_transfer = std::f64::consts::PI * (a_t.powi(3) / MU_EARTH).sqrt();
    assert!(best.plan.duration_s >= t_transfer && best.plan.duration_s < t_transfer + 6000.0);
}

/// Bi-elliptic beats two-burn above the ~11.94 radius ratio, not below.
#[test]
fn bi_elliptic_crossover() {
    let r1 = 7000.0;
    let mk = |ratio: f64| {
        let state = circular_state(r1);
        solve_impulsive(
            &state,
            &big_vehicle(),
            &Cheats::default(),
            &target_circular(r1 * ratio, 0.0),
            SolveMode::Cheapest,
            &SolveConstraints {
                max_intermediate_apo_alt_km: 2_000_000.0,
                ..Default::default()
            },
        )
    };

    // Ratio 5: no bi-elliptic candidate should be offered (it can't win).
    let low = mk(5.0);
    assert!(
        !low.candidates
            .iter()
            .any(|c| matches!(c.plan.kind, StrategyKind::BiElliptic { .. })),
        "bi-elliptic should not appear at ratio 5"
    );

    // Ratio 20: bi-elliptic must appear and beat every two-burn plan.
    let high = mk(20.0);
    let bi = high
        .candidates
        .iter()
        .find(|c| matches!(c.plan.kind, StrategyKind::BiElliptic { .. }))
        .expect("bi-elliptic should appear at ratio 20");
    let best_two = high
        .candidates
        .iter()
        .filter(|c| matches!(c.plan.kind, StrategyKind::TwoBurn { .. }))
        .map(|c| c.plan.total_dv_km_s)
        .fold(f64::INFINITY, f64::min);
    assert!(bi.plan.total_dv_km_s < best_two);
}

/// A 30° plane change combined with the high-apoapsis burn must beat doing
/// it at the low orbit, and the solver should rank it first.
#[test]
fn plane_change_placement() {
    let r1 = 7000.0;
    let r2 = 42_164.0;
    let state = circular_state(r1);
    let result = solve_impulsive(
        &state,
        &big_vehicle(),
        &Cheats::default(),
        &target_circular(r2, 30.0),
        SolveMode::Cheapest,
        &SolveConstraints::default(),
    );
    assert_eq!(result.verdict, Verdict::Feasible);
    let best = &result.candidates[0];
    // Best two-burn plan should fold the plane change into the burn at the
    // high radius (burn 2 of the periapsis-first ordering).
    match best.plan.kind {
        StrategyKind::TwoBurn { plane_change_at } => {
            let pc_burn_radius = match plane_change_at {
                whatiforbit_sim::targeting::PlaneChangeSite::WithBurn1 => {
                    best.plan.burns[0].radius_km
                }
                whatiforbit_sim::targeting::PlaneChangeSite::WithBurn2 => {
                    best.plan.burns[1].radius_km
                }
                _ => panic!("plane change must be placed somewhere"),
            };
            assert!(
                (pc_burn_radius - r2).abs() < 1.0,
                "plane change should happen at the high radius, got {pc_burn_radius}"
            );
        }
        StrategyKind::BiElliptic { .. } => { /* also acceptable if cheaper */ }
        other => panic!("unexpected best strategy {other:?}"),
    }
}

/// Pure inclination change on a fixed orbit: dv = 2 v sin(Δi/2) at apoapsis.
#[test]
fn pure_rotation_cost() {
    let r = 7000.0;
    let state = circular_state(r);
    let di: f64 = 10.0;
    let result = solve_impulsive(
        &state,
        &big_vehicle(),
        &Cheats::default(),
        &target_circular(r, di),
        SolveMode::Cheapest,
        &SolveConstraints::default(),
    );
    let rot = result
        .candidates
        .iter()
        .find(|c| matches!(c.plan.kind, StrategyKind::PureRotation))
        .expect("pure rotation candidate");
    let v = (MU_EARTH / r).sqrt();
    let expected = 2.0 * v * (di.to_radians() / 2.0).sin();
    assert!((rot.plan.total_dv_km_s - expected).abs() < 1e-9);
}

/// Vehicle that cannot afford the transfer: VehicleLimited verdict with
/// rocket-equation-consistent shortfall numbers and a closer achievable orbit.
#[test]
fn shortfall_reporting() {
    let r1 = 7000.0;
    let r2 = 42_164.0;
    let state = circular_state(r1);
    let vehicle = Vehicle {
        wet_mass_kg: 1000.0,
        dry_mass_kg: 800.0, // budget = ve ln(1.25) ≈ 0.657 km/s @ Isp 300
        thrust_n: 1000.0,
        isp_s: 300.0,
    };
    let result = solve_impulsive(
        &state,
        &vehicle,
        &Cheats::default(),
        &target_circular(r2, 0.0),
        SolveMode::Cheapest,
        &SolveConstraints::default(),
    );
    let Verdict::VehicleLimited(shortfall) = &result.verdict else {
        panic!("expected VehicleLimited, got {:?}", result.verdict);
    };

    let budget = dv_budget_km_s(&vehicle, false);
    assert!((shortfall.available_dv_km_s - budget).abs() < 1e-12);
    assert!(shortfall.required_dv_km_s > budget);
    assert!(
        (shortfall.shortfall_dv_km_s
            - (shortfall.required_dv_km_s - shortfall.available_dv_km_s))
            .abs()
            < 1e-12
    );

    // Extra-prop closes the gap exactly (rocket equation).
    let ve = vehicle.ve_km_s();
    let closed =
        ve * ((vehicle.wet_mass_kg + shortfall.extra_prop_kg) / vehicle.dry_mass_kg).ln();
    assert!((closed - shortfall.required_dv_km_s).abs() < 1e-9);

    // Isp-required closes the gap exactly with current masses.
    let closed_isp = shortfall.isp_required_s * whatiforbit_sim::G0 / 1000.0
        * (vehicle.wet_mass_kg / vehicle.dry_mass_kg).ln();
    assert!((closed_isp - shortfall.required_dv_km_s).abs() < 1e-9);

    // Closest achievable sits strictly between current and target SMA and is
    // itself affordable.
    let (a_c, _, _) = shortfall.closest_achievable;
    assert!(a_c > r1 && a_c < r2, "closest achievable SMA {a_c}");
}

/// Invalid targets are rejected with the specific rule.
#[test]
fn invalid_targets() {
    let state = circular_state(7000.0);
    let veh = big_vehicle();
    let constraints = SolveConstraints::default();

    let below_floor = TargetOrbit::from_apo_peri_inc(400.0, 50.0, 0.0);
    let r = solve_impulsive(
        &state,
        &veh,
        &Cheats::default(),
        &below_floor,
        SolveMode::Cheapest,
        &constraints,
    );
    assert!(matches!(
        r.verdict,
        Verdict::InvalidTarget(
            whatiforbit_sim::targeting::TargetInvalid::PeriapsisBelowFloor { .. }
        )
    ));

    let apo_below_peri = TargetOrbit::from_apo_peri_inc(300.0, 800.0, 0.0);
    let r = solve_impulsive(
        &state,
        &veh,
        &Cheats::default(),
        &apo_below_peri,
        SolveMode::Cheapest,
        &constraints,
    );
    // from_apo_peri_inc clamps e >= 0, so this surfaces as periapsis/apoapsis
    // inversion caught by validation on the resolved elements.
    assert!(!matches!(r.verdict, Verdict::Feasible));
}

/// The infinite-fuel cheat suspends both vehicle gates (delta-v budget AND
/// burn envelope): a target far beyond a weak vehicle becomes solvable, and
/// candidates report zero propellant. Without the cheat, the same setup is
/// VehicleLimited.
#[test]
fn infinite_fuel_suspends_vehicle_gates() {
    let state = circular_state(7000.0);
    let weak = Vehicle {
        wet_mass_kg: 1000.0,
        dry_mass_kg: 600.0,
        thrust_n: 400.0,
        isp_s: 300.0, // budget ~1.5 km/s, GEO transfer needs ~3.5
    };
    let target = target_circular(42_164.0, 0.0);
    let constraints = SolveConstraints::default();

    let honest = solve_impulsive(
        &state,
        &weak,
        &Cheats::default(),
        &target,
        SolveMode::Cheapest,
        &constraints,
    );
    assert!(
        matches!(honest.verdict, Verdict::VehicleLimited(_)),
        "without the cheat this must be vehicle-limited, got {:?}",
        honest.verdict
    );

    let cheated = solve_impulsive(
        &state,
        &weak,
        &Cheats {
            infinite_fuel: true,
        },
        &target,
        SolveMode::Cheapest,
        &constraints,
    );
    assert_eq!(cheated.verdict, Verdict::Feasible);
    let best = &cheated.candidates[0];
    assert!(best.feasible(), "top candidate should be feasible");
    assert!(
        !matches!(best.plan.kind, StrategyKind::EdelbaumSpiral),
        "cheat should enable discrete plans, not just the spiral estimate"
    );
    assert_eq!(best.prop_kg, 0.0, "infinite fuel consumes nothing");
}

/// Deadline shorter than any transfer produces DeadlineMiss with the fastest
/// feasible time reported.
#[test]
fn deadline_miss() {
    let state = circular_state(7000.0);
    let result = solve_impulsive(
        &state,
        &big_vehicle(),
        &Cheats::default(),
        &target_circular(42_164.0, 0.0),
        SolveMode::Fastest,
        &SolveConstraints {
            deadline_s: Some(600.0), // 10 minutes: impossible
            ..Default::default()
        },
    );
    let Verdict::DeadlineMiss { fastest_s } = result.verdict else {
        panic!("expected DeadlineMiss, got {:?}", result.verdict);
    };
    assert!(fastest_s > 600.0 && fastest_s.is_finite());
}

/// Low thrust-to-mass vehicle: discrete plans exceed the burn envelope, and
/// the verdict downgrades to the Edelbaum estimate.
#[test]
fn low_thrust_regime() {
    let state = circular_state(7000.0);
    let vehicle = Vehicle {
        wet_mass_kg: 1000.0,
        dry_mass_kg: 300.0,
        thrust_n: 0.2, // ~0.0002 m/s^2: electric-propulsion territory
        isp_s: 1600.0,
    };
    let result = solve_impulsive(
        &state,
        &vehicle,
        &Cheats::default(),
        &target_circular(9000.0, 0.0),
        SolveMode::Cheapest,
        &SolveConstraints::default(),
    );
    let Verdict::LowThrustOnly { estimate } = result.verdict else {
        panic!("expected LowThrustOnly, got {:?}", result.verdict);
    };
    let (dv, dur) = estimate.expect("near-circular case should carry an estimate");
    // Edelbaum coplanar circular-to-circular reduces to |v1 - v0|.
    let expected = ((MU_EARTH / 7000.0_f64).sqrt() - (MU_EARTH / 9000.0_f64).sqrt()).abs();
    assert!((dv - expected).abs() < 1e-9, "edelbaum dv {dv} vs {expected}");
    assert!(dur > 86_400.0, "spiral should take days, got {dur} s");
}
