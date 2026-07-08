# WhatIfOrbit

WhatIfOrbit is a browser-based orbital "what-if" sandbox: load a real spacecraft's orbit
(paste a TLE or fetch one from CelesTrak by NORAD ID), define the vehicle (mass, thrust,
Isp), then fly maneuver ideas against it — impulsive Δv or finite burns — and compare the
what-if trajectory against the original around a 3D Earth. Cheats like infinite fuel let
you explore orbital ideas without real propellant constraints.

The whole app is Rust: a pure-physics simulation core plus a [Bevy](https://bevyengine.org)
front end with egui panels, compiled to WASM and served as a static site. See
[PLAN.md](PLAN.md) for the roadmap.

## Running

Native (development):

```sh
cargo run -p whatiforbit-app
```

Browser (WASM) — requires [Trunk](https://trunkrs.dev) (`brew install trunk` or
`cargo install trunk`) and the wasm target (`rustup target add wasm32-unknown-unknown`):

```sh
trunk serve        # dev server at http://127.0.0.1:8080
trunk build        # static bundle in dist/
```

Handy demo links once the server is up: [`/?demo`](http://127.0.0.1:8080/?demo) auto-loads
the ISS sample; [`/?demo&burn`](http://127.0.0.1:8080/?demo&burn) also adds a 60 m/s
prograde impulse so the what-if trajectory visibly diverges from the baseline.

Tests (the physics validation suite — conservation laws, analytic J2 rates, Hohmann
transfer against the rocket equation):

```sh
cargo test -p whatiforbit-sim
```

## Using it

1. **Load a spacecraft** — paste a TLE, click "ISS sample", or enter a NORAD ID and fetch
   from CelesTrak (needs network).
2. **Set up the vehicle** — wet/dry mass, thrust, Isp. Toggle the infinite-fuel cheat or
   turn off J2 for pure Keplerian motion.
3. **Add maneuvers** — impulses (Δv in velocity/normal/co-normal components) or finite
   burns (duration, throttle, direction). Each maneuver reports what it achieved and how
   much propellant it used; propellant-limited burns are flagged.
4. **Scrub time** — play/pause and drag the timeline; the readout panel shows live orbital
   elements, altitude, mass, and remaining propellant. Gray = original orbit,
   orange = what-if.

## Architecture

```
crates/sim-core   physics: SGP4 init state, DP5(4) integrator, two-body + J2,
                  impulsive & finite-burn maneuvers with mass tracking
crates/app        Bevy 3D scene + egui UI, native and wasm32 targets
```

Known MVP simplifications (see PLAN.md for the roadmap):

- The SGP4 TEME frame is treated as inertial (~0.3° from GCRF at current epochs).
- Dynamics are two-body + J2 only — no drag, SRP, or third bodies yet.
- Space-Track integration needs an auth proxy and is deferred; CelesTrak covers fetch-by-ID.
