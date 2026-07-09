# CHOICES

Judgment calls made while building toward MVP, with the question each one answered.
Newest at the bottom.

## 1. ANISE in the browser vs. hardcoded Earth constants

**Question:** The README calls for ANISE. Do we ship ANISE (SPICE kernels, byte-loading
in WASM unproven) for the MVP, or hardcode Earth constants?

**Decision:** Hardcode Earth constants (GM, radius, J2) and use `hifitime` for time.
Two-body + J2 dynamics don't need SPICE kernels, ANISE's WASM/byte-loading story was the
plan's #1 risk, and skipping it removes a multi-MB kernel download from first page load.
ANISE slots back in when we add frame conversions or third-body ephemerides (the sim-core
API already isolates frames/time so this is contained).

## 2. Inertial frame: TEME treated as inertial

**Question:** SGP4 outputs states in the TEME frame. Convert properly to GCRF/J2000
(needs precession/nutation models or ANISE), or treat TEME-at-epoch as the app's
inertial frame?

**Decision:** Treat TEME as inertial. The error is ~0.3° of frame orientation at current
epochs — irrelevant for "what happens if I burn prograde here" exploration, and invisible
at MVP rendering fidelity. Documented in code and README as a known simplification.

## 3. Propagator: hand-rolled DP5(4) vs. nyx-space

**Question:** Adopt nyx-space (validated, heavy, WASM-proven) or write a small
Dormand–Prince 5(4) integrator with two-body + J2?

**Decision:** Hand-rolled. ~150 lines of integrator plus ~40 of dynamics, validated by a
test suite (energy conservation < 1e-9 over 10 orbits, J2 RAAN drift matches the analytic
rate to 0.05° over 2 days, Hohmann transfer reproduces the rocket equation to 0.5 kg).
Nyx would have added minutes of compile time and megabytes of WASM for capability the MVP
doesn't use. Revisit when we want drag/SRP/third-body or trajectory optimization.

## 4. Bevy 0.16 + bevy_egui 0.34, not latest (0.19)

**Question:** Latest Bevy is 0.19; do we use it?

**Decision:** Pin Bevy 0.16 + bevy_egui 0.34 — a version pairing whose APIs I could write
confidently and correctly in one pass (it compiled first try). Bevy upgrades are routine,
mechanical, and well-documented per release; getting the app working mattered more than
being current. Upgrade as a standalone task later if desired.

## 5. UI entirely in egui, units chosen for humans

**Question:** HTML/JS panels talking to the WASM sim, or egui inside the Bevy canvas?

**Decision:** All egui — one Rust binary, no JS interop layer. Maneuver inputs use
minutes-from-epoch and m/s (not seconds and km/s) because that's how people type these
numbers; conversion happens at the sim boundary.

## 6. Maneuver frame: VNC only

**Question:** Offer maneuver components in multiple frames (VNC, RIC/LVLH, inertial)?

**Decision:** VNC only for MVP (V = along velocity, N = orbit normal, C = V×N ≈ outward
radial). Prograde/retrograde and plane-change intuition maps directly onto it, and one
frame keeps the UI and tests simple. RIC is a small additive change later.

## 7. Infeasible maneuvers: clamp and flag, don't refuse

**Question:** When a burn needs more propellant than the tank holds, error out or do
what's physically possible?

**Decision:** Clamp to what the propellant allows (impulses scale down via the rocket
equation, finite burns cut off at dry mass), execute that, and flag the maneuver as
propellant-limited in the UI. Refusing would make the comparison view useless right when
it's most interesting; silent clamping would be a lie. This also gives the infinite-fuel
cheat a crisp meaning: it just disables the clamp and mass depletion.

## 8. Backward propagation is coast-only

**Question:** Should maneuvers apply when propagating backward from the epoch?

**Decision:** No — back-propagation answers "where was this spacecraft before the TLE
epoch", which is a coast. Maneuvers in negative time create mass-bookkeeping paradoxes
(un-burning propellant) for no user value. Maneuver offsets are validated to be >= 0.

## 9. Sample ISS TLE is fabricated

**Question:** The "ISS sample" button needs a TLE that works offline. Ship a real
historical TLE or a fabricated one?

**Decision:** Fabricated-but-plausible ISS-like TLE (correct inclination/altitude/format,
valid checksums, epoch 2026-01-01) labeled as a sample. A real TLE is stale the moment
it ships anyway; the CelesTrak fetch path is the way to get real current data.

## 10. Browser verification via a `?demo` URL parameter

**Question:** egui renders inside the canvas, so there are no DOM elements to drive from
test tooling, and synthetic PointerEvents don't make it through winit's input pipeline
(they aren't trusted pointers). How do we verify the app end-to-end in a real browser?

**Decision:** Added a `?demo` query parameter that auto-loads the ISS sample on startup —
the exact same code path as the "ISS sample" button. This gives an automatable smoke test
of the full pipeline (TLE parse → SGP4 → propagation → 3D render) and doubles as a
shareable demo link. Real user clicks use winit's trusted-pointer path and are unaffected
by the test-harness limitation.

## 11. Keyboard input: force canvas focus in HTML, not in Rust

**Question:** Typing into egui fields (e.g. maneuver DragValues) showed a caret but no
characters. Browser keyboard events only reach the app when the winit canvas has DOM
focus; winit is supposed to focus it on click but that path proved unreliable (the caret
is egui-rendered, so it appears even when the canvas has no focus — which is exactly why
the symptom is confusing). Fix in winit/Bevy config or in the page?

**Decision:** A three-line shim in index.html: give the canvas `tabindex="0"` and
re-focus it on every `pointerdown` if not already focused. This is independent of
winit/bevy_egui version behavior, costs nothing, and can't break native builds. Kept
Bevy's `prevent_default_event_handling` at its default since bevy_egui's mobile virtual
keyboard support expects it. Verified in the preview: blur → click → focus restored.

## 12. bevy_egui upgraded 0.34 → 0.36, UI moved to EguiPrimaryContextPass

**Question:** Typing produced no text despite the canvas having focus; diagnostics showed
keyboard events being swallowed before reaching Bevy. Keep debugging 0.34's legacy
single-pass input path, or move to the current bevy_egui architecture?

**Decision:** Upgrade to bevy_egui 0.36 (latest supporting Bevy 0.16), enable multipass,
and move the UI system into the `EguiPrimaryContextPass` schedule as 0.35+ requires
(running it in `Update` panics with "No fonts available until first call to
Context::run()"). This puts us on the actively-maintained input path instead of the
legacy one where the typing bug lives. Temporary on-screen input diagnostics (bevy event
count / egui text count / modifier state) stay in the panel until typing is confirmed
fixed end-to-end.

## 13. Typing bug root cause: stuck egui modifier state — fixed by ground-truth sync

**Question:** With bevy_egui 0.36, keystrokes reached Bevy but produced no text. The
on-screen diagnostics showed egui believing Ctrl+Shift+Cmd were all held (bevy_egui
suppresses ALL text input while Ctrl/Cmd appear pressed) — modifier releases had been
missed (e.g. keys released mid page-reload before listeners attach). Patch bevy_egui, or
correct the state from outside?

**Decision:** Added an `unstick_egui_modifiers` system between bevy_egui's modifier
tracking and its text-event conversion: any modifier flagged held whose key isn't pressed
in `ButtonInput<KeyCode>` (the engine's ground truth, cleared on focus loss) gets cleared.
Clear-only — it never sets flags, so it can't fight legitimate key-down states.

**Postscript:** after this fix, typing worked everywhere except the user's main Chrome
profile, where digits 1–9 (but not letters or 0) were still swallowed — an installed
extension binding digit shortcuts, confirmed by it working fine in incognito. Not an app
issue. The temporary on-screen input diagnostics were removed once typing and hover were
confirmed working.

## 14. Bevy asset `.meta` probing disabled for web

**Question:** The Earth texture never rendered in the browser: Bevy probes for
`earth.jpg.meta`, and `trunk serve`'s SPA fallback answers missing files with HTTP 200 +
index.html, which Bevy tries to parse as asset metadata — failing the whole texture load
(and with it the Earth mesh, which waits on its material).

**Decision:** `AssetPlugin { meta_check: AssetMetaCheck::Never }` — we ship no .meta
files, so the probe is pure downside on any web server with an SPA fallback. This is the
standard Bevy-on-web configuration.

## 15. Clipboard combos hidden from winit so browser paste events fire

**Question:** Paste (Cmd+V) did nothing in browser egui fields. winit `preventDefault()`s
every keydown on the canvas, and preventing the keydown's default suppresses the native
`paste`/`copy`/`cut` events — the only channel bevy_egui's web clipboard has. Disable
`prevent_default_event_handling` globally, or carve out the clipboard combos?

**Decision:** A capture-phase keydown listener in index.html that
`stopImmediatePropagation()`s Cmd/Ctrl+V/C/X before winit's handler runs (page scripts
register before the wasm boots, so ours wins the ordering). The browser then performs its
default action and fires the clipboard event bevy_egui consumes. Disabling
prevent-default globally was rejected: it would re-enable browser handling for *all* keys
(Tab focus loss, space scrolling) and bevy_egui's mobile virtual-keyboard support
expects it on. Verified: Cmd+V keydown reaches the page un-prevented while plain keys
remain consumed by winit.

## 16. Target-orbit optimality: ΔV as currency, time as constraint, Pareto presentation

**Question:** For the target-orbit solver, what does "optimal" mean — time, propellant,
ΔV, or some combination?

**Decision (with user):** ΔV is the internal optimization currency; propellant is its
per-vehicle monotone image and the user-facing number/feasibility gate — they are one
axis, not two. Time is the genuinely orthogonal axis (mostly coast/wait, not burn time).
No weighted blends (incommensurate units, unexplainable answers). Two constrained modes —
cheapest-with-optional-deadline and fastest-within-prop — over an always-visible Pareto
candidate list; ties break toward fewer burns. User additionally chose **finite-burn
optimization** over impulsive-plus-check: impulsive templates remain as seeds and lower
bounds, but candidates are converged as finite burn arcs against the real propagator.
Scope: target a/e/i (as apo/peri/inc); RAAN & phase deferred. See PLAN-target-orbit.md.

## 17. ΔV accounting: expended, not net velocity change

**Question:** During M2 testing, finite-burn realizations of a 0.888 km/s Hohmann
reported 1.037 km/s of "ΔV" while consuming exactly the rocket-equation propellant for
0.888. Which number is right?

**Decision:** The reported metric was wrong: `SimResult::total_dv` summed `|v_after −
v_before|` per burn, which folds in gravity's contribution to the velocity change over
the arc (~350 m/s over a 43 s LEO burn — gravity mostly *turns* the vector). Changed to
expended ΔV: `vₑ·ln(m_before/m_after)` for mass-depleting burns, `F·t/m` under the
infinite-fuel cheat. Gravity *losses* still show up honestly — as more expended ΔV needed
to hit the target — which is exactly what the optimizer should be charged for.

## 18. Finite-burn optimizer: feasibility is not convergence

**Question:** The first optimizer version returned the first constraint-satisfying point,
which can be far from cheapest. When is a refinement "done"?

**Decision:** Track the best *feasible* point across augmented-Lagrangian outer loops and
stop only when the feasible cost plateaus (<0.5% improvement between outers, minimum 3
outers), returning the best-seen rather than the last point. Also: the optimizer runs
against the same DP5(4)+J2 propagation the display uses, targets are evaluated as mean
elements over a post-burn revolution, and the whole thing is resumable
(`Refiner::step(eval_budget)`) so the app amortizes it across frames on wasm.

## 19. Optimizer robustness for low-thrust-chemical vehicles

**Question:** A user's solve stalled at "refining best plan…" forever, and Apply did
nothing. Reproduced natively: the default vehicle (400 N / 1000 kg) from ISS altitude to
800 km fails refinement one residual-width from feasible (violation 1.29 after 10 outer
loops), and the failure was silent in the UI.

**Decision:** Four fixes. (1) The augmented-Lagrangian penalty weight now escalates only
when the violation stops improving, capped at 1e5 — the previous unconditional ×8 ramp to
1e7 turned the merit surface into a cliff Nelder–Mead couldn't walk. (2) Default element
tolerances widened (±10 km SMA, ±2e-3 ecc) to match what constant-direction multi-minute
burn arcs can physically null. (3) A first burn whose centered arc would start before
t=0 is seeded one revolution later instead of being clamp-biased. (4) UX: refinement
progress is shown (evaluation count), failure is surfaced in the verdict line, and Apply
falls back to an honestly-labeled impulsive approximation instead of dead-ending.
Regression test pinned at the user's exact scenario. Also cut per-evaluation cost ~3× by
sampling optimizer propagations coarsely (every output point forces an integrator step)
and made `Refiner::step` honor small budgets (it previously ran a full outer loop per
call, freezing wasm frames for seconds on large orbits).

## 20. Earth rotation via GMST + real solar lighting (eclipse groundwork)

**Question:** The user wants Earth rotation, motivated by future sunlit/eclipse
modeling. Decorative spin, or physically meaningful rotation?

**Decision:** Meaningful: the globe's yaw is GMST (IAU 1982) at the scrubbed epoch —
the standard TEME→Earth-fixed rotation for SGP4-class work — so longitude under the
spacecraft is now approximately correct (UT1≈UTC accepted, ~0.004°). Alongside it,
`sim_core::earth` gained Vallado's low-precision solar ephemeris (~0.01°), and the
scene's directional light is aimed along the real sun vector per epoch: the day/night
terminator on the globe is now physically correct and animates with playback. This is
exactly the groundwork spacecraft eclipse tests need — sunlit/umbra determination is a
line-sphere test between a trajectory sample, the sun vector, and Earth's radius.
Validated: GMST matches the J2000 textbook value; sun passes equinox/solstice checks;
and subsolar longitude lands on Greenwich (±equation of time) at 12:00 UTC, which ties
the two together. The texture-longitude offset (Greenwich on mesh −X, hence yaw =
GMST − π) is derived in a code comment and visually verified against the terminator.

## 21. Infinite fuel suspends *both* vehicle gates in the solver

**Question:** With infinite fuel on, the solver still blocked plans behind the
burn-envelope gate ("burns too long for this thrust"), returning estimate-only verdicts
for ambitious targets. Should the cheat suspend that too?

**Decision:** Yes — as the plan originally specified. The envelope gate exists to flag
where the impulsive model degrades (long burn arcs), which is a realism warning, not a
conservation law; cheat mode is explicitly "let me try orbital ideas without real
constraints". With infinite fuel: ΔV budget and burn-envelope checks are both skipped,
candidates report zero propellant ("—" in the table), and refinement still runs against
full dynamics (long arcs may converge roughly or fall back to the labeled impulsive
approximation). The deadline gate is NOT suspended — it's a user-set constraint, not a
vehicle limit. Pinned by test: a weak vehicle targeting GEO is VehicleLimited honestly
and Feasible with the cheat.

## 22. Maneuvers stack chronologically (relative timing, not clamps)

**Question (user):** A later maneuver shouldn't be able to commence before an earlier one.
Enforce by clamping absolute times, or restructure the timing model?

**Decision:** Relative/stacked timing: each row's offset is "minutes after the previous
maneuver ends" (first row: after epoch), clamped ≥ 0, with the computed absolute time
shown in each row's header. Ordering is then correct *by construction* — no clamp
fighting the user mid-edit (the earlier typing saga taught that lesson), and editing an
early maneuver's time naturally shifts everything downstream, which matches how burn
sequences are actually planned. This is also exactly how the finite-burn optimizer
already parameterizes its decision vector (start + gaps), so solver plans materialize
losslessly. sim-core's `Maneuver` keeps absolute offsets — stacking is a UI-layer
conversion (`stacked_start_times_s`, unit-tested).

## 23. Trajectories rendered as gizmo polylines

**Question:** Build mesh geometry for orbit paths or use Bevy's immediate-mode gizmos?

**Decision:** Gizmos. ~2500 points per trajectory re-submitted per frame is well within
budget, needs zero mesh management when the scenario changes, and WebGL2 handles it fine.
Revisit only if we render many spacecraft at once.
