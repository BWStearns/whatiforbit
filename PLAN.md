# WhatIfOrbit — Build Plan

An in-browser orbital "what-if" sandbox: load a real spacecraft's orbit, fly maneuvers
against it (with real or cheated propellant constraints), and see the resulting trajectory
in 3D. Simulation and rendering are Rust (Bevy + ANISE) compiled to WASM.

## MVP definition

A user can, entirely in the browser:

1. **Get a spacecraft state** — paste a TLE/OMM, fetch one by NORAD ID from CelesTrak,
   or enter Keplerian/Cartesian elements by hand.
2. **Define the vehicle** — wet mass, dry mass, thrust, Isp; or skip it and fly
   impulsive-only.
3. **Propagate** forward and backward in time and see the trajectory rendered around a
   3D Earth, with a time scrubber.
4. **Add maneuvers** on a timeline — impulsive Δv (in VNC/RIC frame components) and
   finite burns (thrust/Isp/duration, mass depletion) — and see the "what-if" trajectory
   vs. the original, side by side.
5. **Toggle cheats** — infinite fuel (no mass depletion / no Δv budget), and see the
   propellant budget when cheats are off.
6. **Read out state** — live orbital elements (a, e, i, RAAN, AoP, ν), apogee/perigee,
   period, remaining propellant, at the scrubbed time.

Explicitly **out of scope for MVP** (candidates for later): Space-Track login,
atmospheric drag / SRP / third-body perturbations, multi-spacecraft / conjunctions,
rendezvous targeting, ground tracks, saving/sharing scenarios, mobile layout.

## Architecture

```
whatiforbit/            (cargo workspace)
├── crates/
│   ├── sim-core/       # Pure simulation library. No Bevy. Testable natively.
│   │   ├── TLE/OMM parsing + SGP4 init state (sgp4 crate)
│   │   ├── numerical propagator (RK: Dormand–Prince 5(4), fixed-step fallback)
│   │   ├── dynamics: two-body + J2 (more perturbations later)
│   │   ├── maneuvers: impulsive Δv, finite burn w/ mass flow (thrust, Isp)
│   │   ├── frames/time via ANISE + hifitime
│   │   └── scenario model (vehicle, epoch span, maneuver list, cheat flags)
│   └── app/            # Bevy app: rendering + UI, targets wasm32 + native
│       ├── 3D scene: Earth, orbit polylines, spacecraft marker, maneuver nodes
│       ├── camera (orbit/zoom controls)
│       ├── UI: bevy_egui panels (vehicle, maneuvers, elements readout, time scrub)
│       └── data fetch: CelesTrak GP query, ephemeris kernel download
├── web/                # index.html + Trunk config, static assets
└── assets/             # Earth texture, packaged ANISE kernels (de440s.bsp, pck)
```

Key choices and why:

- **`sim-core` has no Bevy dependency.** The physics is the risky, correctness-critical
  part; keeping it a plain library means fast native unit tests against known results
  (e.g., Vallado test cases, GMAT/Nyx cross-checks) without any WASM/graphics in the loop.
- **UI in `bevy_egui`, not HTML/JS.** Keeps the whole app one Rust binary and avoids a
  JS↔WASM state-sync layer for MVP. If the UI outgrows egui we can move panels to HTML later.
- **Propagator: hand-rolled RK + two-body + J2 for MVP, not nyx-space.** Nyx would give us
  validated finite-burn dynamics for free and is proven in WASM, but it's a heavy
  dependency with its own ecosystem (ODE solvers, estimation, MD types) we mostly don't
  need. Decision point in M0: if the spike shows nyx compiles cleanly to wasm32 at
  acceptable binary size, adopting it instead of hand-rolling is on the table.
- **CelesTrak, not Space-Track, for MVP.** Space-Track requires authenticated POST login
  and doesn't serve CORS headers, so browser access needs a proxy service — that's
  deferred. CelesTrak's GP API is unauthenticated (`.../NORAD_CAT_ID=xxxxx&FORMAT=json`)
  and works from a browser. Paste-a-TLE always works offline.
- **Ephemeris data:** ship `de440s.bsp` (~30 MB) is too big; MVP likely only needs Earth
  GM + J2 constants and UTC↔TDB time handling, so start with the small `pck` /
  planetary-constants ANISE files and fetch-on-demand into `Almanac` from bytes.
  If ANISE byte-loading fights us in WASM (see Risks), fall back to hardcoded Earth
  constants for MVP — two-body+J2 doesn't strictly need SPICE kernels.

### Simulation model (MVP fidelity)

- Initial state: SGP4 from TLE at epoch → TEME → convert to an inertial frame (ANISE) →
  hand off to the numerical propagator. (SGP4 is only used to get the starting state;
  what-if propagation is numerical so maneuvers compose correctly.)
- Dynamics: two-body + J2. Good enough for "orbital ideas" over days-to-weeks in LEO/GEO;
  drag and third-body come later behind the same `Dynamics` trait.
- Finite burns: constant thrust along a commanded direction (VNC frame for MVP),
  ṁ = T/(Isp·g₀), integrated with the state. Infinite-fuel cheat: skip mass depletion
  and budget checks.
- Backward propagation: same integrator, negative step. (Burns in reverse time are
  disallowed in MVP — back-prop only from the scenario start, pre-maneuver.)

## Milestones

### M0 — De-risking spike (goal: walking skeleton in the browser)
- Cargo workspace scaffold; CI (fmt, clippy, test, wasm build).
- Bevy "spinning cube + egui panel" building via Trunk to wasm32 and running in-browser
  (WebGL2 target for compatibility).
- ANISE in WASM: load planetary constants from bytes, do one frame transform. **Go/no-go
  on ANISE-in-browser**; fallback is hardcoded constants.
- `sgp4` crate: parse a pasted ISS TLE, produce a state vector, print elements.
- Exit criteria: one page that takes a TLE and prints orbital elements, next to a Bevy canvas.

### M1 — Sim core (native-first)
- State/frame/time types; scenario model (vehicle, span, maneuvers, cheats).
- DP5(4) integrator + two-body + J2; unit tests vs. published test cases (energy/angular-
  momentum conservation, GMAT or Nyx reference trajectories).
- Impulsive Δv and finite-burn maneuver execution with mass tracking; propellant budget.
- Forward/backward propagation over a scenario; trajectory sampled for rendering.
- Exit criteria: CLI example reproduces a Hohmann transfer to expected tolerance, with
  correct propellant usage.

### M2 — Visualization
- Earth (textured sphere, correct radius/orientation), inertial-frame orbit polylines,
  spacecraft marker, maneuver-node markers.
- Orbit camera (rotate/zoom/pan), sensible scale handling (LEO through GEO+).
- Time scrubber driving marker position along the sampled trajectory; play/pause/rate.
- Original vs. what-if trajectory rendered in distinct colors.
- Exit criteria: paste ISS TLE → see ISS orbit animating around Earth.

### M3 — Scenario UI
- egui panels: vehicle setup (mass/Isp/thrust), maneuver list editor (add/edit/delete,
  frame + components + epoch/duration), cheats toggle, elements readout at scrub time.
- CelesTrak fetch by NORAD ID / name search.
- Recompute-on-edit pipeline (async task so the UI doesn't hitch on propagation).
- Propellant budget display; "burn infeasible" feedback when cheats are off.
- Exit criteria: the full MVP definition above, end to end.

### M4 — Polish & ship
- Error handling (bad TLE, fetch failures), loading states, empty states.
- Binary-size pass (`wasm-opt`, feature-trim Bevy) and load-time budget (< ~15 MB ideally).
- Deploy as a static site (GitHub Pages or Cloudflare Pages) with CI deploy on main.
- README rewrite: what it is, how to run, how to develop.

## Risks & open questions

| Risk | Mitigation |
|---|---|
| ANISE almanac loading in WASM (no filesystem; docs don't advertise wasm) | M0 spike; fallback to hardcoded Earth constants + hifitime for MVP |
| Bevy WASM binary size / load time | WebGL2 backend, trim default features, `wasm-opt -Oz`; accept ~10–20 MB for MVP |
| CelesTrak CORS or rate limits | Paste-TLE path always works; cache responses in localStorage |
| Numerical propagation correctness | Native test suite vs. published cases before any UI work (M1 gate) |
| Scale/precision in rendering (f32 GPU vs. km-scale orbits) | Render in scaled scene units (e.g., 1 unit = 1000 km), keep physics in f64 in sim-core |
| Space-Track integration (auth + CORS) | Post-MVP; needs a small proxy (e.g., Cloudflare Worker holding credentials) |

## Post-MVP ideas (parking lot)

Space-Track login via proxy · drag/SRP/third-body dynamics · ground tracks · multiple
spacecraft & conjunction screening · maneuver targeting (change apogee to X) · scenario
save/share via URL · Lambert solver for rendezvous · ephemeris export (CCSDS OEM).
