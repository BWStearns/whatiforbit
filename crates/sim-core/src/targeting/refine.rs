//! Finite-burn refinement: takes an impulsive template plan as a seed,
//! transcribes it into a finite-burn decision vector, and converges it
//! against the real propagator (DP5(4), two-body + J2 — the same code path
//! that renders) under an augmented-Lagrangian formulation.
//!
//! Decision vector layout, per burn i (4 params each):
//!   [start_i, duration_i, alpha_i, beta_i]
//! where start_0 is absolute (s from epoch), start_{i>0} is the *gap* after
//! the previous burn's end (enforces ordering by construction), and
//! (alpha, beta) define the constant VNC thrust direction:
//!   dir = (cos beta * cos alpha, sin beta, cos beta * sin alpha).

use super::kepler::period_s;
use super::optimize::{AugLag, Eval, NelderMead};
use super::target::SolveConstraints;
use super::templates::ImpulsivePlan;
use crate::{
    elements_from_rv, Cheats, Maneuver, Scenario, State, Vehicle, G0, R_EARTH,
};
use glam::DVec3;

/// Everything the refiner needs to evaluate a candidate decision vector.
#[derive(Clone)]
pub struct RefineContext {
    pub init: State,
    pub vehicle: Vehicle,
    pub cheats: Cheats,
    pub enable_j2: bool,
    /// Resolved targets and tolerances: (value, tol) for a, e, i(deg).
    pub target_a: (f64, f64),
    pub target_e: (f64, f64),
    pub target_i: (f64, f64),
    pub constraints: SolveConstraints,
    /// Objective: false = minimize prop, true = minimize duration.
    pub minimize_time: bool,
}

/// A converged finite-burn plan ready to materialize as scenario maneuvers.
#[derive(Debug, Clone, PartialEq)]
pub struct RefinedPlan {
    pub burns: Vec<Maneuver>,
    pub prop_kg: f64,
    pub dv_km_s: f64,
    /// Time from epoch to the end of the last burn, seconds.
    pub duration_s: f64,
    /// Mean elements achieved over the post-burn revolution: (a, e, i deg).
    pub achieved: (f64, f64, f64),
}

pub enum RefineStatus {
    Running {
        evals_so_far: usize,
        best_cost: f64,
        worst_violation: f64,
    },
    Converged(RefinedPlan),
    Failed(String),
}

struct Bounds {
    lo: Vec<f64>,
    hi: Vec<f64>,
}

/// Resumable refiner: call `step()` with an evaluation budget per frame.
pub struct Refiner {
    ctx: RefineContext,
    bounds: Bounds,
    auglag: AugLag,
    nm: NelderMead,
    outer: usize,
    evals: usize,
    max_outer: usize,
    n_burns: usize,
    /// NM iterations accumulated toward the next outer update.
    inner_done: usize,
    /// Worst scaled violation at the last outer update (progress signal).
    pub last_violation: f64,
    /// Best constraint-satisfying point seen so far and its cost.
    best_feasible: Option<(Vec<f64>, f64)>,
}

fn direction_angles(dv: DVec3) -> (f64, f64) {
    let d = dv.normalize_or_zero();
    let alpha = d.z.atan2(d.x);
    let beta = d.y.asin();
    (alpha, beta)
}

fn angles_direction(alpha: f64, beta: f64) -> DVec3 {
    DVec3::new(
        beta.cos() * alpha.cos(),
        beta.sin(),
        beta.cos() * alpha.sin(),
    )
}

impl Refiner {
    /// Seed a refiner from an impulsive template plan.
    pub fn new(ctx: RefineContext, seed: &ImpulsivePlan) -> Result<Self, String> {
        let veh = ctx.vehicle;
        if seed.burns.is_empty() {
            return Err("template has no discrete burns to refine".into());
        }
        let mdot = veh.thrust_n / (veh.isp_s * G0);

        // Build seed vector + bounds burn by burn.
        let mut x0 = Vec::new();
        let mut lo = Vec::new();
        let mut hi = Vec::new();
        let mut steps = Vec::new();
        let mut mass = veh.wet_mass_kg;
        let mut prev_end = 0.0;
        let current_period = period_s(elements_from_rv(ctx.init.r, ctx.init.v).sma_km);

        for (i, b) in seed.burns.iter().enumerate() {
            let dv = b.dv_mag();
            // Burn time realizing this impulse at full throttle.
            let dur = if ctx.cheats.infinite_fuel || mdot <= 0.0 {
                (dv * 1000.0 * mass / veh.thrust_n.max(1e-9)).max(1.0)
            } else {
                let prop = mass * (1.0 - (-dv / veh.ve_km_s()).exp());
                (prop / mdot).max(1.0)
            };
            mass -= if ctx.cheats.infinite_fuel {
                0.0
            } else {
                mass * (1.0 - (-dv / veh.ve_km_s()).exp())
            };

            let local_period = period_s(b.radius_km);
            let mut centered_start = b.t_offset_s - dur / 2.0;
            // Thrust can't start before the scenario epoch. A first burn
            // centered on t <= 0 would get clamp-biased (half the arc
            // missing); seed it one revolution later instead, where the same
            // orbital geometry recurs.
            if i == 0 && centered_start < 0.0 {
                centered_start += current_period;
            }
            let (alpha, beta) = direction_angles(b.dv_vnc_km_s);

            if i == 0 {
                let start = centered_start;
                x0.push(start);
                lo.push((start - 0.45 * local_period).max(0.0));
                hi.push(start + 0.45 * local_period);
                steps.push(0.02 * local_period);
            } else {
                let gap = (centered_start - prev_end).max(0.0);
                x0.push(gap);
                lo.push((gap - 0.45 * local_period).max(0.0));
                hi.push(gap + 0.45 * local_period);
                steps.push(0.02 * local_period);
            }
            x0.push(dur);
            lo.push(0.25 * dur);
            hi.push(4.0 * dur);
            steps.push(0.1 * dur);
            x0.push(alpha);
            lo.push(alpha - 0.6);
            hi.push(alpha + 0.6);
            steps.push(0.05);
            x0.push(beta);
            lo.push((beta - 0.6).max(-1.5));
            hi.push((beta + 0.6).min(1.5));
            steps.push(0.05);

            prev_end = centered_start + dur;
        }

        let bounds = Bounds { lo, hi };
        let n_burns = seed.burns.len();
        let auglag = AugLag::new(3);
        let mut merit_evals = 0usize;
        let mut merit = |x: &[f64]| {
            merit_evals += 1;
            let e = evaluate(&ctx, &bounds, x, n_burns);
            auglag.merit(&e)
        };
        let nm = NelderMead::new(&x0, &steps, &mut merit);
        let evals = merit_evals;

        Ok(Self {
            ctx,
            bounds,
            auglag,
            nm,
            outer: 0,
            evals,
            max_outer: 10,
            n_burns,
            inner_done: 0,
            last_violation: f64::INFINITY,
            best_feasible: None,
        })
    }

    /// Advance by up to `eval_budget` objective evaluations. Small budgets
    /// are honored: NM runs in short chunks so a frame can spend e.g. 40
    /// evaluations and return.
    pub fn step(&mut self, eval_budget: usize) -> RefineStatus {
        let inner_per_outer = 60 * self.n_burns; // NM iterations per outer loop
        const CHUNK_ITERS: usize = 6;
        let mut spent = 0usize;

        while spent < eval_budget {
            // Run a short NM chunk under the current multipliers.
            {
                let ctx = self.ctx.clone();
                let bounds = Bounds {
                    lo: self.bounds.lo.clone(),
                    hi: self.bounds.hi.clone(),
                };
                let n_burns = self.n_burns;
                let auglag_snapshot = AugLag {
                    lambda_eq: self.auglag.lambda_eq.clone(),
                    mu: self.auglag.mu,
                    last_violation: f64::INFINITY,
                };
                let mut merit = |x: &[f64]| {
                    let e = evaluate(&ctx, &bounds, x, n_burns);
                    auglag_snapshot.merit(&e)
                };
                let used = self.nm.run(CHUNK_ITERS, &mut merit);
                spent += used;
                self.evals += used;
                self.inner_done += CHUNK_ITERS;
            }
            if self.inner_done < inner_per_outer {
                continue;
            }
            self.inner_done = 0;

            // Outer update from the current best.
            let (best_x, _) = self.nm.best();
            let best_x = best_x.to_vec();
            let e = evaluate(&self.ctx, &self.bounds, &best_x, self.n_burns);
            let violation = AugLag::violation(&e);
            self.last_violation = violation;
            self.outer += 1;

            // Feasibility alone isn't convergence: keep optimizing until the
            // feasible cost plateaus (<0.5% improvement between outers).
            if violation <= 1.0 {
                let plateaued = self
                    .best_feasible
                    .as_ref()
                    .is_some_and(|(_, c)| e.cost >= c * 0.995);
                let improved = self
                    .best_feasible
                    .as_ref()
                    .is_none_or(|(_, c)| e.cost < *c);
                if improved {
                    self.best_feasible = Some((best_x.clone(), e.cost));
                }
                if plateaued && self.outer >= 3 {
                    let (x, _) = self.best_feasible.as_ref().unwrap();
                    return RefineStatus::Converged(self.materialize(&x.clone()));
                }
            }
            if self.outer >= self.max_outer {
                return match &self.best_feasible {
                    Some((x, _)) => RefineStatus::Converged(self.materialize(&x.clone())),
                    None => RefineStatus::Failed(format!(
                        "did not converge: worst scaled violation {violation:.2} after {} outer loops",
                        self.outer
                    )),
                };
            }
            self.auglag.update(&e);
            // Restart the simplex around the incumbent; step size scales with
            // how far from feasible we still are (wide when lost, tight when
            // polishing).
            let frac = 0.02 + 0.08 * (violation.min(3.0) / 3.0);
            let steps: Vec<f64> = self
                .bounds
                .lo
                .iter()
                .zip(&self.bounds.hi)
                .map(|(l, h)| frac * (h - l))
                .collect();
            let ctx2 = self.ctx.clone();
            let bounds2 = Bounds {
                lo: self.bounds.lo.clone(),
                hi: self.bounds.hi.clone(),
            };
            let auglag2 = AugLag {
                lambda_eq: self.auglag.lambda_eq.clone(),
                mu: self.auglag.mu,
                last_violation: f64::INFINITY,
            };
            let mut merit2 = |x: &[f64]| {
                let e = evaluate(&ctx2, &bounds2, x, self.n_burns);
                auglag2.merit(&e)
            };
            self.nm = NelderMead::new(&best_x, &steps, &mut merit2);
            let used = best_x.len() + 1; // simplex re-seed evaluations
            spent += used;
            self.evals += used;
        }

        let (_, best_m) = self.nm.best();
        RefineStatus::Running {
            evals_so_far: self.evals,
            best_cost: best_m,
            worst_violation: self.last_violation,
        }
    }

    fn materialize(&self, x: &[f64]) -> RefinedPlan {
        let burns = decode(&self.bounds, x, self.n_burns);
        let (scenario, result) = propagate_decision(&self.ctx, &burns);
        let last_end = burns
            .iter()
            .map(|m| match m {
                Maneuver::FiniteBurn {
                    t_offset_s,
                    duration_s,
                    ..
                } => t_offset_s + duration_s,
                _ => 0.0,
            })
            .fold(0.0, f64::max);
        let achieved = mean_elements_after(&result, last_end);
        let prop_kg = self.ctx.vehicle.wet_mass_kg - result.final_mass_kg;
        RefinedPlan {
            burns: scenario.maneuvers,
            prop_kg,
            dv_km_s: result.total_dv_km_s,
            duration_s: last_end,
            achieved,
        }
    }
}

/// Clamp-decode a decision vector into finite-burn maneuvers.
fn decode(bounds: &Bounds, x: &[f64], n_burns: usize) -> Vec<Maneuver> {
    let mut burns = Vec::with_capacity(n_burns);
    let mut prev_end = 0.0;
    for i in 0..n_burns {
        let base = i * 4;
        let clamped: Vec<f64> = (0..4)
            .map(|k| x[base + k].clamp(bounds.lo[base + k], bounds.hi[base + k]))
            .collect();
        let start = if i == 0 {
            clamped[0]
        } else {
            prev_end + clamped[0]
        };
        let duration = clamped[1];
        let dir = angles_direction(clamped[2], clamped[3]);
        burns.push(Maneuver::FiniteBurn {
            t_offset_s: start,
            duration_s: duration,
            direction_vnc: dir,
            throttle: 1.0,
        });
        prev_end = start + duration;
    }
    burns
}

fn propagate_decision(
    ctx: &RefineContext,
    burns: &[Maneuver],
) -> (Scenario, crate::SimResult) {
    let last_end = burns
        .iter()
        .map(|m| match m {
            Maneuver::FiniteBurn {
                t_offset_s,
                duration_s,
                ..
            } => t_offset_s + duration_s,
            _ => 0.0,
        })
        .fold(0.0, f64::max);
    // Post-burn observation window: one revolution of the target-sized orbit.
    let obs_period = period_s(ctx.target_a.0.max(R_EARTH + 200.0));
    let t_end = last_end + obs_period;
    let scenario = Scenario {
        init: ctx.init,
        vehicle: ctx.vehicle,
        maneuvers: burns.to_vec(),
        cheats: ctx.cheats,
        t_start_s: 0.0,
        t_end_s: t_end,
        // Coarser sampling than display quality: every output point forces an
        // integrator step, and the optimizer runs thousands of these.
        output_dt_s: (t_end / 400.0).clamp(1.0, 300.0),
        enable_j2: ctx.enable_j2,
    };
    let result = scenario.propagate();
    (scenario, result)
}

/// Mean (a, e, i) averaged over samples after `t_after`.
fn mean_elements_after(result: &crate::SimResult, t_after: f64) -> (f64, f64, f64) {
    let samples: Vec<_> = result
        .trajectory
        .samples
        .iter()
        .filter(|s| s.t_s >= t_after)
        .collect();
    if samples.is_empty() {
        return (f64::NAN, f64::NAN, f64::NAN);
    }
    let mut acc = (0.0, 0.0, 0.0);
    for s in &samples {
        let el = elements_from_rv(s.r, s.v);
        acc.0 += el.sma_km;
        acc.1 += el.ecc;
        acc.2 += el.inc_deg;
    }
    let n = samples.len() as f64;
    (acc.0 / n, acc.1 / n, acc.2 / n)
}

/// Full evaluation of a decision vector: objective + scaled constraints.
fn evaluate(ctx: &RefineContext, bounds: &Bounds, x: &[f64], n_burns: usize) -> Eval {
    let burns = decode(bounds, x, n_burns);
    let (_, result) = propagate_decision(ctx, &burns);
    let last_end = burns
        .iter()
        .map(|m| match m {
            Maneuver::FiniteBurn {
                t_offset_s,
                duration_s,
                ..
            } => t_offset_s + duration_s,
            _ => 0.0,
        })
        .fold(0.0, f64::max);

    let (a, e, i) = mean_elements_after(&result, last_end);
    // Unconstrained elements carry tol = +inf, making their residual ~0.
    // A degenerate propagation (NaN elements) is pushed away hard.
    let eq = if a.is_nan() {
        vec![1e6, 1e6, 1e6]
    } else {
        vec![
            (a - ctx.target_a.0) / ctx.target_a.1,
            (e - ctx.target_e.0) / ctx.target_e.1,
            (i - ctx.target_i.0) / ctx.target_i.1,
        ]
    };

    // Transfer perigee floor over the burn+transfer arc (not the parking
    // observation rev, which is the target itself).
    let min_alt = result
        .trajectory
        .samples
        .iter()
        .filter(|s| s.t_s <= last_end)
        .map(|s| s.r.length() - R_EARTH)
        .fold(f64::INFINITY, f64::min);
    let mut ineq = vec![(ctx.constraints.min_transfer_perigee_alt_km - min_alt) / 100.0];

    let prop_kg = ctx.vehicle.wet_mass_kg - result.final_mass_kg;
    if ctx.minimize_time {
        // Prop becomes a constraint in fastest mode.
        let prop_avail = ctx.vehicle.wet_mass_kg - ctx.vehicle.dry_mass_kg;
        if !ctx.cheats.infinite_fuel {
            ineq.push((prop_kg - prop_avail) / prop_avail.max(1e-9));
        }
    }

    let cost = if ctx.minimize_time {
        last_end / period_s(ctx.target_a.0)
    } else if ctx.cheats.infinite_fuel {
        // Constant mass: prop is zero; use accumulated dv as the objective.
        result.total_dv_km_s
    } else {
        prop_kg / ctx.vehicle.wet_mass_kg
    };

    Eval { cost, eq, ineq }
}
