//! Bridges UI inputs to the sim core: TLE loading, CelesTrak fetches,
//! scenario recompute, and playback time advance.

use crate::types::*;
use bevy::prelude::*;
use glam::DVec3;
use whatiforbit_sim::targeting::{
    self, RefineStatus, Refiner, SolveConstraints, SolveMode, StrategyKind, TargetOrbit, Verdict,
};
use whatiforbit_sim::{parse_tle, Cheats, Maneuver, Scenario, Vehicle, R_EARTH};

/// The query string this run was launched with: `location.search` on the web,
/// and the first CLI argument natively, so a share link can be opened in the
/// dev build without a browser.
fn launch_query() -> Option<String> {
    #[cfg(target_arch = "wasm32")]
    {
        web_sys::window().and_then(|w| w.location().search().ok())
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::env::args().nth(1)
    }
}

/// Restore app state from the launch query at startup.
///
/// Two shapes are accepted. A share link (`?v=1&...`, written by
/// `share::sync_share_link`) carries the whole scenario and wins outright. The
/// older `?demo` flags stay for browser verification and tooling: they load the
/// ISS sample, optionally add a burn, and optionally run a canned solve.
pub fn autoload_from_url(
    mut input: ResMut<ScenarioInput>,
    mut solve: ResMut<TargetSolve>,
    mut playback: ResMut<Playback>,
    mut frame: ResMut<ViewFrame>,
    mut camera: Query<&mut crate::scene::OrbitCamera>,
) {
    let Some(search) = launch_query() else {
        return;
    };

    if let Ok(mut cam) = camera.single_mut() {
        if crate::share::apply_query(
            &search,
            &mut input,
            &mut playback,
            &mut frame,
            &mut cam,
            &mut solve,
        ) {
            if !input.tle_text.trim().is_empty() {
                // Overwrites `dirty`/`loaded` on its own; the scrub time set
                // from the link survives, since `recompute` only clamps it.
                load_tle_into(&mut input);
            }
            // A link that silently fails to apply is near-impossible to
            // diagnose from the outside; say what came back.
            info!(
                "restored share link: {} maneuver(s), {}",
                input.maneuvers.len(),
                match &input.loaded {
                    Some(l) => l.name.as_str(),
                    None => "no spacecraft",
                }
            );
            return;
        }
    }

    if search.contains("demo") {
        input.tle_text = SAMPLE_TLE.to_string();
        load_tle_into(&mut input);
    }
    // `?demo&burn`: also add a visible prograde impulse so the what-if
    // trajectory diverges from the baseline immediately.
    if search.contains("burn") {
        input.maneuvers.push(ManeuverInput {
            kind: ManeuverKind::Impulsive,
            t_offset_min: 20.0,
            dv_vnc_m_s: [60.0, 0.0, 0.0],
            ..Default::default()
        });
        input.dirty = true;
    }
    // `?demo&target`: run a canned target-orbit solve (ISS -> 800 km circular)
    // so the solver path is exercisable from tooling without clicks.
    if search.contains("target") {
        solve.apo_alt_km = 800.0;
        solve.peri_alt_km = 800.0;
        solve.inc_deg = 51.64;
        run_target_solve(&input, &mut solve);
    }
}

pub fn load_tle_into(input: &mut ScenarioInput) {
    match parse_tle(&input.tle_text) {
        Ok(parsed) => {
            input.loaded = Some(LoadedOrbit {
                name: parsed
                    .name
                    .unwrap_or_else(|| format!("NORAD {}", parsed.norad_id)),
                state: parsed.state,
                object_id: parsed.international_designator,
            });
            input.error = None;
            input.dirty = true;
        }
        Err(e) => {
            input.error = Some(e.0);
        }
    }
}

/// Kick off an async GET of the given NORAD catalog number from CelesTrak.
pub fn start_fetch(input: &mut ScenarioInput, channel: &FetchChannel) {
    let id = input.norad_query.trim().to_string();
    if id.is_empty() || !id.chars().all(|c| c.is_ascii_digit()) {
        input.error = Some("NORAD ID must be a number".into());
        return;
    }
    input.fetch_in_flight = true;
    input.error = None;
    let url = format!("https://celestrak.org/NORAD/elements/gp.php?CATNR={id}&FORMAT=TLE");
    let tx = channel.tx.clone();
    ehttp::fetch(ehttp::Request::get(url), move |result| {
        let msg = match result {
            Ok(resp) if resp.ok => {
                let body = String::from_utf8_lossy(&resp.bytes).to_string();
                if body.contains("No GP data found") || body.trim().is_empty() {
                    Err(format!("CelesTrak has no data for NORAD {id}"))
                } else {
                    Ok(body)
                }
            }
            Ok(resp) => Err(format!("CelesTrak returned HTTP {}", resp.status)),
            Err(e) => Err(format!("fetch failed: {e}")),
        };
        let _ = tx.send(msg);
    });
}

pub fn apply_fetch_results(channel: Res<FetchChannel>, mut input: ResMut<ScenarioInput>) {
    let Ok(rx) = channel.rx.lock() else { return };
    while let Ok(msg) = rx.try_recv() {
        input.fetch_in_flight = false;
        match msg {
            Ok(tle) => {
                input.tle_text = tle.trim().to_string();
                load_tle_into(&mut input);
            }
            Err(e) => input.error = Some(e),
        }
    }
}

/// Absolute start times (seconds from epoch) for stacked maneuver rows:
/// each row's offset is measured from the end of the previous maneuver
/// (impulses end instantly; burns end after their duration).
pub fn stacked_start_times_s(rows: &[ManeuverInput]) -> Vec<f64> {
    let mut prev_end = 0.0;
    rows.iter()
        .map(|m| {
            let start = prev_end + m.t_offset_min.max(0.0) * 60.0;
            prev_end = start
                + match m.kind {
                    ManeuverKind::Impulsive => 0.0,
                    ManeuverKind::FiniteBurn => m.duration_s.max(0.0),
                };
            start
        })
        .collect()
}

fn maneuvers_from_input(rows: &[ManeuverInput]) -> Vec<Maneuver> {
    let starts = stacked_start_times_s(rows);
    rows.iter()
        .zip(starts)
        .map(|(m, start_s)| match m.kind {
            ManeuverKind::Impulsive => Maneuver::Impulsive {
                t_offset_s: start_s,
                dv_vnc_km_s: DVec3::new(
                    m.dv_vnc_m_s[0] / 1000.0,
                    m.dv_vnc_m_s[1] / 1000.0,
                    m.dv_vnc_m_s[2] / 1000.0,
                ),
            },
            ManeuverKind::FiniteBurn => Maneuver::FiniteBurn {
                t_offset_s: start_s,
                duration_s: m.duration_s,
                direction_vnc: DVec3::new(
                    m.direction_vnc[0],
                    m.direction_vnc[1],
                    m.direction_vnc[2],
                ),
                throttle: m.throttle,
            },
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stacking_orders_maneuvers() {
        let rows = vec![
            ManeuverInput {
                kind: ManeuverKind::FiniteBurn,
                t_offset_min: 10.0,
                duration_s: 120.0,
                ..Default::default()
            },
            ManeuverInput {
                kind: ManeuverKind::Impulsive,
                t_offset_min: 5.0,
                ..Default::default()
            },
            ManeuverInput {
                kind: ManeuverKind::FiniteBurn,
                t_offset_min: 0.0, // back-to-back with the impulse
                duration_s: 60.0,
                ..Default::default()
            },
        ];
        let starts = stacked_start_times_s(&rows);
        // Burn 1: 600s..720s. Impulse: 720 + 300 = 1020s. Burn 3: 1020s.
        assert_eq!(starts, vec![600.0, 1020.0, 1020.0]);
        // Negative gaps clamp to zero: order is preserved by construction.
        let rows2 = vec![
            rows[0].clone(),
            ManeuverInput {
                t_offset_min: -30.0,
                ..rows[1].clone()
            },
        ];
        let starts2 = stacked_start_times_s(&rows2);
        assert_eq!(starts2[1], 720.0);
    }
}

pub fn recompute(
    mut input: ResMut<ScenarioInput>,
    mut output: ResMut<SimOutput>,
    mut playback: ResMut<Playback>,
) {
    if !input.dirty {
        return;
    }
    input.dirty = false;

    let Some(loaded) = input.loaded.clone() else {
        output.baseline = None;
        output.whatif = None;
        return;
    };

    // Sanitize vehicle numbers so the sim never sees nonsense.
    let wet = input.wet_mass_kg.max(1.0);
    let dry = input.dry_mass_kg.clamp(0.1, wet);
    input.wet_mass_kg = wet;
    input.dry_mass_kg = dry;
    let vehicle = Vehicle {
        wet_mass_kg: wet,
        dry_mass_kg: dry,
        thrust_n: input.thrust_n.max(0.0),
        isp_s: input.isp_s.max(1.0),
    };

    let t_start = -(input.backward_days.max(0.0)) * 86_400.0;
    let t_end = input.forward_days.clamp(0.001, 60.0) * 86_400.0;
    let output_dt = ((t_end - t_start) / 2500.0).clamp(1.0, 120.0);

    let scenario = Scenario {
        init: loaded.state,
        vehicle,
        maneuvers: maneuvers_from_input(&input.maneuvers),
        cheats: Cheats {
            infinite_fuel: input.infinite_fuel,
        },
        t_start_s: t_start,
        t_end_s: t_end,
        output_dt_s: output_dt,
        enable_j2: input.enable_j2,
    };

    let whatif = scenario.propagate();
    let baseline = {
        let mut b = scenario.clone();
        b.maneuvers.clear();
        b.propagate()
    };

    output.baseline = Some(baseline);
    output.whatif = Some(whatif);
    output.scenario = Some(scenario);
    output.t_start_s = t_start;
    output.t_end_s = t_end;
    playback.t_s = playback.t_s.clamp(t_start, t_end);
}

/// Assemble solver inputs from the current app state. None until a
/// spacecraft is loaded.
pub fn targeting_inputs(
    input: &ScenarioInput,
    solve: &TargetSolve,
) -> Option<(
    whatiforbit_sim::State,
    Vehicle,
    Cheats,
    TargetOrbit,
    SolveMode,
    SolveConstraints,
)> {
    let loaded = input.loaded.as_ref()?;
    let vehicle = Vehicle {
        wet_mass_kg: input.wet_mass_kg.max(1.0),
        dry_mass_kg: input.dry_mass_kg.clamp(0.1, input.wet_mass_kg.max(1.0)),
        thrust_n: input.thrust_n.max(0.0),
        isp_s: input.isp_s.max(1.0),
    };
    let cheats = Cheats {
        infinite_fuel: input.infinite_fuel,
    };
    let target = TargetOrbit::from_apo_peri_inc(solve.apo_alt_km, solve.peri_alt_km, solve.inc_deg);
    let mode = match solve.mode {
        TargetModeUi::Cheapest => SolveMode::Cheapest,
        TargetModeUi::Fastest => SolveMode::Fastest,
    };
    let constraints = SolveConstraints {
        deadline_s: solve
            .deadline_enabled
            .then_some(solve.deadline_hours * 3600.0),
        ..Default::default()
    };
    Some((loaded.state, vehicle, cheats, target, mode, constraints))
}

/// Run the (fast) impulsive solve and populate the candidate table.
pub fn run_target_solve(input: &ScenarioInput, solve: &mut TargetSolve) {
    solve.rows.clear();
    solve.verdict_text = None;
    solve.closest_offer = None;
    solve.active_refine = None;
    solve.apply_when_done = None;
    solve.preview = None;
    solve.preview_row = None;

    let Some((state, vehicle, cheats, target, mode, constraints)) =
        targeting_inputs(input, solve)
    else {
        solve.verdict_text = Some("Load a spacecraft first.".into());
        return;
    };

    let result = targeting::solve_impulsive(&state, &vehicle, &cheats, &target, mode, &constraints);

    solve.rows = result
        .candidates
        .iter()
        .map(|c| {
            let mut blocking = Vec::new();
            if !c.within_budget {
                blocking.push("over Δv budget");
            }
            if !c.within_deadline {
                blocking.push("misses deadline");
            }
            if !c.within_burn_envelope {
                blocking.push(if matches!(c.plan.kind, StrategyKind::EdelbaumSpiral) {
                    "estimate only"
                } else {
                    "burns too long for thrust"
                });
            }
            CandidateRow {
                label: c.plan.kind.label(),
                dv_km_s: c.plan.total_dv_km_s,
                prop_kg: c.prop_kg,
                duration_s: c.plan.duration_s,
                n_burns: c.plan.burns.len(),
                feasible: c.feasible(),
                blocking: blocking.join(", "),
                plan: c.plan.clone(),
                state: RowPlanState::Impulsive,
            }
        })
        .collect();

    solve.verdict_text = Some(match &result.verdict {
        Verdict::Feasible => "Reachable — refining best plan…".into(),
        Verdict::InvalidTarget(t) => format!("Invalid target: {t}"),
        Verdict::VehicleLimited(s) => {
            let (a, e, i) = s.closest_achievable;
            solve.closest_offer =
                Some((a * (1.0 + e) - R_EARTH, a * (1.0 - e) - R_EARTH, i));
            format!(
                "Out of reach: needs {:.0} m/s but the vehicle has {:.0} m/s ({:.0} m/s short).\n\
                 Would need ~{:.1} kg more propellant, or Isp ≥ {:.0} s with the current tank.",
                s.required_dv_km_s * 1000.0,
                s.available_dv_km_s * 1000.0,
                s.shortfall_dv_km_s * 1000.0,
                s.extra_prop_kg,
                s.isp_required_s
            )
        }
        Verdict::LowThrustOnly { estimate } => match estimate {
            Some((dv, dur)) => format!(
                "Discrete burns are too long for this thrust-to-mass.\n\
                 Reachable as a low-thrust spiral: ~{:.0} m/s over ~{:.1} days (estimate; v1 does not plan spirals).",
                dv * 1000.0,
                dur / 86_400.0
            ),
            None => "Burn arcs exceed the discrete-burn envelope for this vehicle, and the \
                     geometry is too eccentric for a spiral estimate."
                .into(),
        },
        Verdict::DeadlineMiss { fastest_s } => format!(
            "Reachable, but not within the deadline — fastest feasible arrival is {:.1} h.",
            fastest_s / 3600.0
        ),
    });

    // Auto-refine the top feasible discrete candidate.
    if matches!(result.verdict, Verdict::Feasible) {
        start_refine_for_best(input, solve);
    }
}

fn start_refine_row(input: &ScenarioInput, solve: &mut TargetSolve, idx: usize) -> bool {
    let Some((state, vehicle, cheats, target, mode, constraints)) =
        targeting_inputs(input, solve)
    else {
        return false;
    };
    let ctx = targeting::refine_context(
        &state,
        &vehicle,
        &cheats,
        input.enable_j2,
        &target,
        mode,
        &constraints,
    );
    match Refiner::new(ctx, &solve.rows[idx].plan) {
        Ok(refiner) => {
            solve.rows[idx].state = RowPlanState::Refining;
            solve.active_refine = Some((idx, refiner));
            true
        }
        Err(e) => {
            solve.rows[idx].state = RowPlanState::Failed(e);
            false
        }
    }
}

fn start_refine_for_best(input: &ScenarioInput, solve: &mut TargetSolve) {
    let best = solve.rows.iter().position(|r| {
        r.feasible && !matches!(r.plan.kind, StrategyKind::EdelbaumSpiral)
    });
    if let Some(idx) = best {
        start_refine_row(input, solve, idx);
    }
}

/// Request that a row be applied to the maneuver list: refined plans apply
/// immediately; unrefined feasible rows refine first, then apply.
pub fn request_apply(input: &mut ScenarioInput, solve: &mut TargetSolve, idx: usize) {
    match &solve.rows[idx].state {
        RowPlanState::Refined(plan) => {
            let plan = plan.clone();
            apply_refined(input, &plan);
            solve.verdict_text = Some(format!("Applied: {}.", solve.rows[idx].label));
        }
        RowPlanState::Refining => solve.apply_when_done = Some(idx),
        RowPlanState::Failed(_) => {
            // Refinement already failed once; apply the impulsive
            // approximation rather than dead-ending.
            let plan = solve.rows[idx].plan.clone();
            apply_impulsive_approx(input, &plan);
            solve.verdict_text = Some(format!(
                "Applied impulsive approximation of: {} (refinement did not converge).",
                solve.rows[idx].label
            ));
        }
        RowPlanState::Impulsive => {
            if start_refine_row(input, solve, idx) {
                solve.apply_when_done = Some(idx);
            }
        }
    }
}

/// Realize an impulsive plan as centered full-throttle finite burns (the
/// same seeding the refiner uses) and materialize it. Fallback path when
/// refinement can't converge — approximate but honest.
fn apply_impulsive_approx(input: &mut ScenarioInput, plan: &targeting::ImpulsivePlan) {
    let Some(loaded) = input.loaded.as_ref() else {
        return;
    };
    let vehicle = Vehicle {
        wet_mass_kg: input.wet_mass_kg.max(1.0),
        dry_mass_kg: input.dry_mass_kg.clamp(0.1, input.wet_mass_kg.max(1.0)),
        thrust_n: input.thrust_n.max(1e-9),
        isp_s: input.isp_s.max(1.0),
    };
    let mdot = vehicle.thrust_n / (vehicle.isp_s * whatiforbit_sim::G0);
    let current_period = targeting::kepler::period_s(
        whatiforbit_sim::elements_from_rv(loaded.state.r, loaded.state.v).sma_km,
    );
    let mut mass = vehicle.wet_mass_kg;
    let mut rows = Vec::new();
    let mut prev_end = 0.0;
    for (i, b) in plan.burns.iter().enumerate() {
        let dv = b.dv_mag();
        if dv < 1e-9 {
            continue;
        }
        let prop = if input.infinite_fuel {
            0.0
        } else {
            mass * (1.0 - (-dv / vehicle.ve_km_s()).exp())
        };
        let dur = if input.infinite_fuel || mdot <= 0.0 {
            (dv * 1000.0 * mass / vehicle.thrust_n).max(1.0)
        } else {
            (prop / mdot).max(1.0)
        };
        mass -= prop;
        let mut start = b.t_offset_s - dur / 2.0;
        if i == 0 && start < 0.0 {
            start += current_period;
        }
        let start = start.max(prev_end); // stacked: never before the previous end
        let dir = b.dv_vnc_km_s.normalize_or_zero();
        rows.push(ManeuverInput {
            kind: ManeuverKind::FiniteBurn,
            t_offset_min: (start - prev_end) / 60.0,
            duration_s: dur,
            throttle: 1.0,
            direction_vnc: [dir.x, dir.y, dir.z],
            ..Default::default()
        });
        prev_end = start + dur;
    }
    input.maneuvers = rows;
    input.dirty = true;
}

/// Materialize a refined plan into the (replaced) maneuver list, converting
/// the plan's absolute burn times into stacked gaps.
fn apply_refined(input: &mut ScenarioInput, plan: &targeting::RefinedPlan) {
    let mut prev_end = 0.0;
    input.maneuvers = plan
        .burns
        .iter()
        .filter_map(|m| match m {
            Maneuver::FiniteBurn {
                t_offset_s,
                duration_s,
                direction_vnc,
                throttle,
            } => {
                let gap_min = ((t_offset_s - prev_end).max(0.0)) / 60.0;
                prev_end = t_offset_s + duration_s;
                Some(ManeuverInput {
                    kind: ManeuverKind::FiniteBurn,
                    t_offset_min: gap_min,
                    duration_s: *duration_s,
                    throttle: *throttle,
                    direction_vnc: [direction_vnc.x, direction_vnc.y, direction_vnc.z],
                    ..Default::default()
                })
            }
            _ => None,
        })
        .collect();
    input.dirty = true;
}

/// Per-frame driver: advance the active refinement within a bounded budget so
/// the UI stays responsive (wasm runs this on the main thread).
pub fn drive_refinement(mut input: ResMut<ScenarioInput>, mut solve: ResMut<TargetSolve>) {
    let Some((idx, refiner)) = solve.active_refine.as_mut() else {
        return;
    };
    let idx = *idx;
    // ~40-80 propagations per frame keeps wasm frames responsive while a
    // typical refinement (a few thousand evals) completes within seconds.
    match refiner.step(60) {
        RefineStatus::Running { evals_so_far, .. } => {
            if let Some(v) = &mut solve.verdict_text {
                if v.starts_with("Reachable") {
                    *v = format!("Reachable — refining best plan… ({evals_so_far} evaluations)");
                }
            }
        }
        RefineStatus::Converged(plan) => {
            if solve.apply_when_done.take() == Some(idx) {
                apply_refined(&mut input, &plan);
            }
            solve.rows[idx].dv_km_s = plan.dv_km_s;
            solve.rows[idx].prop_kg = plan.prop_kg;
            solve.rows[idx].duration_s = plan.duration_s;
            solve.rows[idx].state = RowPlanState::Refined(plan);
            solve.active_refine = None;
            if let Some(v) = &mut solve.verdict_text {
                if v.starts_with("Reachable") {
                    *v = "Reachable — best plan refined against full dynamics.".into();
                }
            }
        }
        RefineStatus::Failed(e) => {
            // Never silent: surface it, and honor a pending Apply with the
            // impulsive approximation.
            solve.rows[idx].state = RowPlanState::Failed(e.clone());
            solve.active_refine = None;
            let pending_apply = solve.apply_when_done.take() == Some(idx);
            if pending_apply {
                let plan = solve.rows[idx].plan.clone();
                apply_impulsive_approx(&mut input, &plan);
                solve.verdict_text = Some(format!(
                    "Applied impulsive approximation of: {} (refinement did not converge: {e}).",
                    solve.rows[idx].label
                ));
            } else {
                solve.verdict_text = Some(format!(
                    "Reachable, but refinement did not converge ({e}). \
                     Apply will use the impulsive approximation."
                ));
            }
        }
    }
}

/// Compute (cheap, impulsive) trajectory preview for a hovered row.
pub fn compute_preview(input: &ScenarioInput, solve: &mut TargetSolve, idx: usize) {
    if solve.preview_row == Some(idx) {
        return;
    }
    solve.preview_row = Some(idx);
    solve.preview = None;
    let Some((state, vehicle, cheats, _, _, _)) = targeting_inputs(input, solve) else {
        return;
    };
    let plan = &solve.rows[idx].plan;
    if plan.burns.is_empty() {
        return;
    }
    let maneuvers: Vec<Maneuver> = plan
        .burns
        .iter()
        .map(|b| Maneuver::Impulsive {
            t_offset_s: b.t_offset_s,
            dv_vnc_km_s: b.dv_vnc_km_s,
        })
        .collect();
    let t_end = plan.duration_s + targeting::kepler::period_s(
        plan.burns.last().map(|b| b.radius_km).unwrap_or(R_EARTH + 400.0),
    );
    let scenario = Scenario {
        init: state,
        vehicle,
        maneuvers,
        cheats,
        t_start_s: 0.0,
        t_end_s: t_end,
        output_dt_s: (t_end / 1500.0).clamp(1.0, 120.0),
        enable_j2: input.enable_j2,
    };
    solve.preview = Some(scenario.propagate().trajectory);
}

pub fn advance_playback(
    time: Res<Time>,
    output: Res<SimOutput>,
    mut playback: ResMut<Playback>,
) {
    if !playback.playing || output.whatif.is_none() {
        return;
    }
    playback.t_s += time.delta_secs() as f64 * playback.rate;
    if playback.t_s >= output.t_end_s {
        playback.t_s = output.t_end_s;
        playback.playing = false;
    }
}
