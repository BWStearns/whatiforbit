# Future work

## RAAN, argp, and phasing targets for the target-orbit solver

Deferred from target-orbit v1 (see [PLAN-target-orbit.md](PLAN-target-orbit.md)); the v1
design keeps them additive via per-element-optional targets and mean-elements-at-arrival
constraint semantics. Expected effort when picked up:

- **Direct RAAN change — small.** A combined inclination+RAAN change is a single plane
  rotation: rotation angle from spherical trig
  (cos θ = cos i₁ cos i₂ + sin i₁ sin i₂ cos ΔΩ), burn at the mutual node. One new
  template plus one residual; expensive in ΔV (~2·v·sin(θ/2)) but trivial to plan.
- **J2 differential-drift RAAN — about a milestone.** Park in an offset drift orbit,
  accumulate nodal-rate difference for weeks, transfer back. Needs: coast-duration
  decision variables (transcription already admits them), mean-element planning with
  analytic J2 secular rates over week-to-month horizons (numerically verify only the
  winning candidate), and timeline/sampling UX stretched to long spans. The Pareto
  presentation already accommodates "nearly free but takes 23 days" candidates.
- **argp rotation — small.** In-plane apsidal rotation templates (two-impulse, or free
  via J2 apsidal drift for the patient); same shape as RAAN work, less of it.
- **Phasing — small, pairs with rendezvous.** Phasing-orbit template is closed-form;
  phase is a periodic residual with the rev-count k enumerated per candidate (the
  candidate-enumeration architecture already handles discrete structure). Only
  meaningful relative to a reference (slot or second spacecraft), so schedule it with a
  rendezvous/multi-spacecraft feature.
- **Inner minimizer upgrade.** Nelder–Mead is comfortable at v1's ~8–12 decision
  dimensions; RAAN + coast variables push toward ~16 where it strains. Swap in a
  gradient-aware minimizer behind the same resumable `step()` interface.

## Sunlit / eclipse modeling

Groundwork landed: `sim_core::earth` has GMST (IAU 1982) and a low-precision solar
ephemeris, the globe rotates to GMST, and scene lighting follows the real sun vector.
Remaining for spacecraft eclipse states: per-sample umbra/penumbra test (cylindrical
shadow is a one-liner against `sun_direction`; conical adds sun/Earth angular radii),
sunlit-fraction readout per orbit, eclipse spans drawn on the trajectory/timeline, and
optionally battery/power what-ifs on top.

## Other parked items

See the parking lot in [PLAN.md](PLAN.md): Space-Track proxy, drag/SRP/third-body,
ground tracks (Earth rotation is now modeled via GMST, so ground tracks are mostly a
lat/lon projection + polyline away), conjunctions, scenario save/share, OEM export,
WASM bundle size pass.
