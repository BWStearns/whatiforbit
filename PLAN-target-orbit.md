# Target Orbit — Feature Plan

Branch: `target-orbit`. Builds on the existing sim-core (DP5(4) propagator, two-body + J2,
impulsive + finite-burn maneuvers with mass tracking) and the Bevy/egui app.

## Feature summary

The user adjusts target orbital elements — **apoapsis altitude, periapsis altitude,
inclination** (equivalently a, e, i) — and the system finds maneuver plans that take the
current vehicle from its present orbit to the target. Plans are **finite-burn optimized**
(per scope decision) for the actual vehicle thrust/Isp/mass, not idealized impulses.

If the target cannot be reached, the system says so and quantifies the shortfall.

## Optimality (decided)

- **Internal currency: ΔV**; propellant is its monotone image via the rocket equation and
  is what the user sees ("uses 82 of your 400 kg"). Finite-burn optimization naturally
  captures gravity/steering losses that make real prop worse than impulsive ΔV.
- **Time is the orthogonal axis** — mostly coast/wait time, not burn time.
- **Two selection modes** (constrained single objectives, no weighted blends):
  1. **Cheapest** — minimize prop, optional deadline `T ≤ T_max`.
  2. **Fastest** — minimize arrival time, subject to prop ≤ available.
- **Pareto presentation**: the solver always returns every feasible candidate with
  (ΔV, prop kg, duration, #burns); the UI lists them sorted by the active mode with the
  frontier visible. Equal-cost ties break toward fewer burns.
- Side constraints: minimum transfer perigee altitude (default 150 km — no accidental
  reentry mid-transfer), maximum intermediate apoapsis (caps bi-elliptic excursions),
  target-element tolerances (defaults: ±5 km on apo/peri, ±0.05° on i).

## Architecture

New sim-core module `targeting/`, pure and natively testable like the rest of the crate:

```
targeting/
├── target.rs      TargetOrbit spec + validation (peri > surface, e < 1, apo ≥ peri…)
├── templates.rs   Impulsive strategy templates → structure + seeds + cost bounds
├── transcribe.rs  Template plan → finite-burn decision vector + bounds
├── optimize.rs    Augmented-Lagrangian + Nelder–Mead solver core (resumable)
├── feasibility.rs ΔV budget checks, shortfall computation, closest-achievable orbit
└── mod.rs         solve(current_state, vehicle, target, mode, constraints) -> Candidates
```

### 1. Strategy templates (impulsive analytics — seeds, not answers)

Direct finite-burn optimal control fails without good initial guesses, so each candidate
starts from a closed-form impulsive skeleton:

- **Two-burn coplanar** (generalized Hohmann between arbitrary coaxial-ish ellipses):
  burn at periapsis/apoapsis to match the target's r_p and r_a.
- **Bi-elliptic** (three burns via intermediate apoapsis r_b; r_b swept within the
  max-apoapsis constraint; only offered when it beats two-burn).
- **Plane change placement**: pure-rotation burn at the slowest point vs **combined**
  with an apoapsis burn (vector-summed, usually much cheaper than sequential).
- **Edelbaum low-thrust estimate** (circular-to-circular + Δi): analytic ΔV and duration
  for the many-rev spiral regime. Not converted to a burn plan in v1 — used to (a) cost
  the low-T/m regime honestly and (b) power shortfall reporting when discrete burn arcs
  can't close the case.

Each template yields: burn count/placement, impulsive ΔV vector seeds, and an impulsive
cost that lower-bounds the finite-burn result (sanity check on convergence).

### 2. Finite-burn transcription

Decision vector per candidate, directly in the existing `Maneuver::FiniteBurn` terms:
for each burn arc — start time (relative), duration, constant VNC direction (2 angles),
throttle fixed at 1.0. Two-burn plan ⇒ 8 parameters; bi-elliptic ⇒ 12. Bounds from the
template (e.g. burn window centered on the impulsive point, duration bounded by
propellant and by a fraction of orbital period).

Objective/constraint evaluation = **propagate the candidate scenario with the existing
DP5(4) + J2 pipeline** (the same code path the user's trajectory uses, so what the
optimizer converges on is exactly what renders). Terminal elements are evaluated
**averaged over one final revolution** to suppress J2 osculation wobble.

### 3. Optimizer

Hand-rolled, WASM-friendly, no heavy NLP dependency:

- **Augmented Lagrangian** outer loop on the element-matching equality constraints
  (and the perigee-floor path constraint, checked on the sampled trajectory).
- **Adaptive Nelder–Mead** inner minimizer (≤ 12 dims, derivative-free — robust to the
  noisy-ish objective from adaptive-step propagation).
- **Resumable**: `Optimizer::step(n_iters)` API so the app runs N iterations per frame
  with a progress indicator (wasm has no threads in our build; native can just loop).
  Budget ~2–5 s wall clock; convergence typically well under that for seeded starts.
- Convergence acceptance: constraints within tolerance AND cost within a few percent of
  stable across the last outer iterations. Divergent candidates are dropped with a note
  (the impulsive lower bound catches silently-wrong "solutions").

### 4. Feasibility and shortfall

Checked in order, each producing a structured, user-readable result:

1. **Invalid target** — periapsis below the atmosphere floor, apo < peri, e ≥ 1:
   rejected with the specific rule violated.
2. **Vehicle ΔV budget** — best template ΔV (min over strategies, incl. Edelbaum) vs
   budget `vₑ·ln(m₀/m_dry)`. If short: report required vs available ΔV, shortfall in
   m/s **and** kg ("needs ~37 kg more prop at Isp 300 s, or Isp ≥ 512 s with the current
   tank"), and the **closest achievable orbit**: bisect the requested element delta along
   (current → target) until the best-template cost fits the budget; report that orbit.
3. **Thrust/time envelope** — if required burn arcs exceed the per-arc duration cap the
   discrete-burn model allows, fall back to the Edelbaum estimate: report its ΔV and
   duration ("reachable as a ~34-day low-thrust spiral; discrete-burn planning not
   supported for this thrust-to-mass in v1").
4. **Deadline** — cheapest feasible plan misses `T_max`: report the fastest feasible
   time so the user can decide whether to relax the deadline or the target.

With the infinite-fuel cheat on, checks 2–3 are skipped (budget = ∞), and the solver
optimizes ΔV anyway so numbers stay meaningful.

### 5. UI (app crate)

New **Target orbit** panel section:

- Inputs: apoapsis alt, periapsis alt, inclination (+ "copy from current" button);
  mode selector (Cheapest / Fastest); optional deadline; tolerance disclosure.
- **Solve** button → incremental solve with progress; results as a compact candidate
  table: strategy name, ΔV, prop, duration, #burns — sorted by mode, Pareto-dominant
  rows highlighted, infeasible candidates greyed with their blocking reason.
- Selecting a candidate **materializes its burns into the existing maneuver list** as
  ordinary `FiniteBurn` rows — editable, rendered, OPM-exportable like hand-made ones.
  A ghost preview renders on hover before committing.
- Shortfall states render in the panel (shortfall numbers + closest-achievable orbit,
  with a "target that instead" button).

## Milestones

- **M1 — Targeting core, impulsive layer**: `TargetOrbit`, validation, templates with
  costs/seeds, feasibility + shortfall math. Tests: template costs vs textbook cases
  (Hohmann, bi-elliptic crossover ratio ≈ 11.94, combined-vs-sequential plane change),
  shortfall numbers vs rocket-equation hand calcs, closest-achievable monotonicity.
- **M2 — Finite-burn optimizer**: transcription, AL+NM solver, resumable stepping.
  Tests: high-T/m finite-burn solutions converge to within gravity-loss margin (~1–3%)
  of impulsive seeds; low-T/m correctly defers to Edelbaum; perigee-floor constraint
  actively binds on a crafted case; deterministic given a seed.
- **M3 — UI integration**: panel, incremental solve UX, candidate table, materialize +
  ghost preview, shortfall presentation. Browser-verified end to end (`?demo`-style
  URL hook for a canned target to keep it testable from tooling).
- **M4 — Polish**: deadline mode, tolerances UI, CHOICES/README updates, candidate
  export notes in OPM COMMENTs.

## Risks

| Risk | Mitigation |
|---|---|
| Optimizer non-convergence / local minima | Always seeded from analytic templates; impulsive lower bound as sanity check; drop-with-reason rather than return garbage |
| WASM main-thread stalls during solve | Resumable step API, iterations amortized across frames, hard wall-clock budget |
| J2 osculating-element wobble vs targets | Terminal elements averaged over a final rev; tolerances default loose enough to be honest |
| Low-thrust vehicles break discrete-burn assumptions | Explicit T/m envelope check; Edelbaum path for costing + shortfall instead of a fake plan |
| Elliptical + inclined initial orbits (templates assume near-coaxial geometry) | Templates handle apsis-aligned cases exactly; general argp misalignment costed conservatively in v1 and refined by the optimizer; documented limitation |

## Forward compatibility: RAAN, argp, phasing (deferred, designed-for)

Two v1 design rules keep the deferred elements additive rather than a rework:

1. **Per-element-optional targets.** `TargetOrbit` stores an `Option` per element with a
   generic residual/tolerance vector; the solver and feasibility code iterate over the
   targeted set rather than assuming (a, e, i). RAAN/argp/phase later become new
   residuals, not an API break.
2. **Constraints in mean elements at arrival epoch.** Under J2, RAAN/argp keep drifting
   after arrival, so "match element X" is only well-posed with an epoch attached; define
   it that way for all elements now (free for a/e/i, drop-in for RAAN later). The
   transcription likewise admits coast-duration decision variables (not just burn
   parameters) so wait-based strategies fit the existing pipeline.

Expected later effort: direct RAAN via combined plane rotation at the mutual node —
small (one template + one residual). J2 differential-drift RAAN strategy — the real
chunk (mean-element planning over weeks-to-months horizons, analytic drift rates for
search + numerical verification of the winner); roughly one milestone. Phasing — small
template + periodic residual with rev-count enumerated per candidate; primarily valuable
once rendezvous/multi-spacecraft exists. Inner-minimizer note: Nelder–Mead is fine at
v1's ~8–12 dims; RAAN + coast variables push toward ~16 where it strains — plan is to
swap in a gradient-aware minimizer behind the same resumable interface, not to
restructure.

## Out of scope (v1)

RAAN / argp / phase targeting (and J2-drift RAAN strategies), rendezvous/intercept,
multi-rev discrete-burn optimal control beyond the template structures, low-thrust
spiral *planning* (estimate only), TARGET propagation with drag.
