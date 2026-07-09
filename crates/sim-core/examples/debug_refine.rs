//! Scratch diagnostics for the finite-burn refiner. Run with:
//! cargo run -p whatiforbit-sim --example debug_refine

use glam::DVec3;
use hifitime::Epoch;
use whatiforbit_sim::targeting::{
    refine_context, solve_impulsive, RefineStatus, Refiner, SolveConstraints, SolveMode,
    StrategyKind, TargetOrbit,
};
use whatiforbit_sim::{elements_from_rv, Cheats, Maneuver, Scenario, State, Vehicle, MU_EARTH, R_EARTH};

fn main() {
    let r1: f64 = std::env::var("R1").ok().and_then(|s| s.parse().ok()).unwrap_or(7000.0);
    let r2: f64 = std::env::var("R2").ok().and_then(|s| s.parse().ok()).unwrap_or(9000.0);
    let v = (MU_EARTH / r1).sqrt();
    let state = State::new(
        Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0),
        DVec3::new(r1, 0.0, 0.0),
        DVec3::new(0.0, v, 0.0),
        1000.0,
    );
    let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|s| s.parse().ok()).unwrap_or(d);
    let vehicle = Vehicle {
        wet_mass_kg: env("WET", 5_000.0),
        dry_mass_kg: env("DRY", 1_000.0),
        thrust_n: env("THRUST", 50_000.0),
        isp_s: env("ISP", 320.0),
    };
    let target = TargetOrbit::from_apo_peri_inc(r2 - R_EARTH, r2 - R_EARTH, 0.0);
    let constraints = SolveConstraints::default();
    let cheats = Cheats::default();

    let solve = solve_impulsive(&state, &vehicle, &cheats, &target, SolveMode::Cheapest, &constraints);
    for c in &solve.candidates {
        println!(
            "candidate {:?}: dv={:.4} dur={:.0}s feasible={}",
            c.plan.kind,
            c.plan.total_dv_km_s,
            c.plan.duration_s,
            c.feasible()
        );
        for b in &c.plan.burns {
            println!(
                "   burn @t={:.0}s r={:.0}km dv_vnc=({:.4},{:.4},{:.4})",
                b.t_offset_s, b.radius_km, b.dv_vnc_km_s.x, b.dv_vnc_km_s.y, b.dv_vnc_km_s.z
            );
        }
    }
    let seed = solve
        .candidates
        .iter()
        .find(|c| c.feasible() && !matches!(c.plan.kind, StrategyKind::EdelbaumSpiral))
        .unwrap();

    // Evaluate the *raw seed realization* (impulses as centered finite burns).
    let mdot = vehicle.thrust_n / (vehicle.isp_s * whatiforbit_sim::G0);
    let mut mass = vehicle.wet_mass_kg;
    let mut seed_burns = Vec::new();
    for b in &seed.plan.burns {
        let dv = b.dv_vnc_km_s.length();
        let prop = mass * (1.0 - (-dv / vehicle.ve_km_s()).exp());
        let dur = prop / mdot;
        mass -= prop;
        seed_burns.push(Maneuver::FiniteBurn {
            t_offset_s: (b.t_offset_s - dur / 2.0).max(0.0),
            duration_s: dur,
            direction_vnc: b.dv_vnc_km_s.normalize(),
            throttle: 1.0,
        });
    }
    let sc = Scenario {
        init: state,
        vehicle,
        maneuvers: seed_burns.clone(),
        cheats,
        t_start_s: 0.0,
        t_end_s: seed.plan.duration_s + 8000.0,
        output_dt_s: 10.0,
        enable_j2: true,
    };
    let res = sc.propagate();
    let last = res.trajectory.samples.last().unwrap();
    let el = elements_from_rv(last.r, last.v);
    println!(
        "\nraw seed realization: dv={:.4} prop={:.1}kg  final osc a={:.1} e={:.5} i={:.3}",
        res.total_dv_km_s,
        vehicle.wet_mass_kg - res.final_mass_kg,
        el.sma_km,
        el.ecc,
        el.inc_deg
    );

    // Now refine and report.
    let ctx = refine_context(&state, &vehicle, &cheats, true, &target, SolveMode::Cheapest, &constraints);
    let mut refiner = Refiner::new(ctx, &seed.plan).unwrap();
    let mut steps = 0;
    loop {
        steps += 1;
        match refiner.step(2_000) {
            RefineStatus::Converged(plan) => {
                println!("\nconverged after {steps} step calls:");
                println!(
                    "  dv={:.4} prop={:.1}kg dur={:.0}s achieved a={:.1} e={:.5} i={:.3}",
                    plan.dv_km_s, plan.prop_kg, plan.duration_s, plan.achieved.0, plan.achieved.1, plan.achieved.2
                );
                for b in &plan.burns {
                    if let Maneuver::FiniteBurn { t_offset_s, duration_s, direction_vnc, .. } = b {
                        println!(
                            "  burn t={t_offset_s:.0}s dur={duration_s:.0}s dir=({:.3},{:.3},{:.3})",
                            direction_vnc.x, direction_vnc.y, direction_vnc.z
                        );
                    }
                }
                break;
            }
            RefineStatus::Failed(e) => {
                println!("FAILED: {e}");
                break;
            }
            RefineStatus::Running { evals_so_far, best_cost, .. } => {
                println!("running: evals={evals_so_far} best_merit={best_cost:.4}");
            }
        }
        if steps > 100 {
            println!("gave up");
            break;
        }
    }
}
