//! Validation tests for the sim core: conservation laws, analytic J2 rates,
//! Hohmann transfer geometry and propellant, TLE ingestion.

use glam::DVec3;
use hifitime::Epoch;
use std::f64::consts::TAU;
use whatiforbit_sim::{
    elements_from_rv, parse_tle, Cheats, Maneuver, Scenario, State, Vehicle, G0, J2, MU_EARTH,
    R_EARTH,
};

const ISS_TLE: &str = "ISS (ZARYA)
1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9001
2 25544  51.6400 208.9163 0006317  69.9862 290.2000 15.49560000123452";

fn circular_state(radius_km: f64, epoch: Epoch) -> State {
    let v = (MU_EARTH / radius_km).sqrt();
    State::new(
        epoch,
        DVec3::new(radius_km, 0.0, 0.0),
        DVec3::new(0.0, v, 0.0),
        1000.0,
    )
}

fn scenario(init: State, vehicle: Vehicle, maneuvers: Vec<Maneuver>, t_end_s: f64) -> Scenario {
    Scenario {
        init,
        vehicle,
        maneuvers,
        cheats: Cheats::default(),
        t_start_s: 0.0,
        t_end_s,
        output_dt_s: 30.0,
        enable_j2: true,
    }
}

/// Two-body energy must be conserved to high precision over many orbits.
/// (Equatorial orbit, so J2 only rescales the effective potential radially
/// and energy including J2 is still conserved; we check specific orbital
/// energy drift stays tiny.)
#[test]
fn energy_conservation_over_ten_orbits() {
    let r0 = 7000.0;
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0);
    let init = circular_state(r0, epoch);
    let period = TAU * (r0.powi(3) / MU_EARTH).sqrt();

    let sc = scenario(init, Vehicle::default(), vec![], 10.0 * period);
    let result = sc.propagate();

    let energy = |r: DVec3, v: DVec3| {
        let rn = r.length();
        // Include the J2 potential so the invariant is exact for our dynamics.
        let u_j2 = -MU_EARTH / rn
            * (1.0 - J2 * (R_EARTH / rn).powi(2) * (1.5 * (r.z / rn).powi(2) - 0.5));
        v.length_squared() / 2.0 + u_j2
    };

    let e0 = energy(init.r, init.v);
    let max_drift = result
        .trajectory
        .samples
        .iter()
        .map(|s| ((energy(s.r, s.v) - e0) / e0).abs())
        .fold(0.0, f64::max);
    assert!(max_drift < 1e-9, "relative energy drift {max_drift}");
}

/// J2 secular RAAN drift for an inclined LEO must match the analytic rate.
#[test]
fn j2_raan_regression_matches_analytic_rate() {
    let a = 7000.0;
    let inc: f64 = 51.6_f64.to_radians();
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0);
    let v = (MU_EARTH / a).sqrt();
    // Circular orbit inclined about the x-axis, starting at the ascending node.
    let init = State::new(
        epoch,
        DVec3::new(a, 0.0, 0.0),
        DVec3::new(0.0, v * inc.cos(), v * inc.sin()),
        1000.0,
    );

    let n = (MU_EARTH / a.powi(3)).sqrt();
    let raan_dot = -1.5 * n * J2 * (R_EARTH / a).powi(2) * inc.cos(); // rad/s, e=0
    let span = 2.0 * 86_400.0;

    let sc = scenario(init, Vehicle::default(), vec![], span);
    let result = sc.propagate();
    let last = result.trajectory.samples.last().unwrap();
    let elems = elements_from_rv(last.r, last.v);

    let expected_deg = (raan_dot * span).to_degrees().rem_euclid(360.0);
    let got_deg = elems.raan_deg.rem_euclid(360.0);
    let diff = (got_deg - expected_deg + 180.0).rem_euclid(360.0) - 180.0;
    assert!(
        diff.abs() < 0.05,
        "RAAN after 2 days: got {got_deg:.4} deg, expected {expected_deg:.4} deg"
    );
}

/// Impulsive Hohmann transfer from 7000 km to 42164 km: check apoapsis after
/// the first burn, near-circularity after the second, and propellant use
/// against the rocket equation.
#[test]
fn hohmann_transfer_geometry_and_propellant() {
    let r1 = 7000.0;
    let r2 = 42_164.0;
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0);
    let init = circular_state(r1, epoch);

    let a_t = (r1 + r2) / 2.0;
    let v1 = (MU_EARTH / r1).sqrt();
    let v_p = (MU_EARTH * (2.0 / r1 - 1.0 / a_t)).sqrt();
    let dv1 = v_p - v1;
    let v_a = (MU_EARTH * (2.0 / r2 - 1.0 / a_t)).sqrt();
    let v2 = (MU_EARTH / r2).sqrt();
    let dv2 = v2 - v_a;
    let t_transfer = TAU / 2.0 * (a_t.powi(3) / MU_EARTH).sqrt();

    let vehicle = Vehicle {
        wet_mass_kg: 2000.0,
        dry_mass_kg: 500.0,
        thrust_n: 400.0,
        isp_s: 320.0,
    };
    let maneuvers = vec![
        Maneuver::Impulsive {
            t_offset_s: 60.0,
            dv_vnc_km_s: DVec3::new(dv1, 0.0, 0.0),
        },
        Maneuver::Impulsive {
            t_offset_s: 60.0 + t_transfer,
            dv_vnc_km_s: DVec3::new(dv2, 0.0, 0.0),
        },
    ];

    // Run with J2 disabled so the analytic two-body Hohmann expectations are
    // exact; the J2 case is covered by the RAAN regression test.
    let mut sc = scenario(init, vehicle, maneuvers, 60.0 + t_transfer + 3600.0);
    sc.enable_j2 = false;
    let result = sc.propagate();

    assert_eq!(result.reports.len(), 2);
    assert!(result.reports.iter().all(|r| r.feasible), "burns infeasible");

    let last = result.trajectory.samples.last().unwrap();
    let elems = elements_from_rv(last.r, last.v);
    assert!(
        (elems.sma_km - r2).abs() < 1.0,
        "final SMA {} km vs target {r2}",
        elems.sma_km
    );
    assert!(elems.ecc < 1e-4, "final ecc {} not near-circular", elems.ecc);

    // Rocket equation cross-check on total propellant.
    let ve = vehicle.isp_s * G0 / 1000.0;
    let expected_final_mass = vehicle.wet_mass_kg * (-(dv1 + dv2) / ve).exp();
    assert!(
        (result.final_mass_kg - expected_final_mass).abs() < 0.5,
        "final mass {} vs rocket equation {expected_final_mass}",
        result.final_mass_kg
    );
}

/// A finite burn must deplete propellant at thrust/(Isp*g0) and cut off at
/// the dry mass when the tank runs dry — unless the infinite-fuel cheat is on.
#[test]
fn finite_burn_mass_flow_and_infinite_fuel_cheat() {
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0);
    let init = circular_state(7000.0, epoch);
    let vehicle = Vehicle {
        wet_mass_kg: 1000.0,
        dry_mass_kg: 900.0,
        thrust_n: 1000.0,
        isp_s: 300.0,
    };
    let mdot = vehicle.thrust_n / (vehicle.isp_s * G0);
    let full_tank_s = (vehicle.wet_mass_kg - vehicle.dry_mass_kg) / mdot;

    // Request twice the achievable burn time.
    let burn = Maneuver::FiniteBurn {
        t_offset_s: 0.0,
        duration_s: 2.0 * full_tank_s,
        direction_vnc: DVec3::X,
        throttle: 1.0,
    };

    let sc = scenario(init, vehicle, vec![burn], 4.0 * full_tank_s);
    let result = sc.propagate();
    let report = &result.reports[0];
    assert!(!report.feasible);
    assert!((report.achieved - full_tank_s).abs() < 1e-6);
    assert!((result.final_mass_kg - vehicle.dry_mass_kg).abs() < 1e-6);

    // Same burn with infinite fuel: full duration, no mass change.
    let mut sc2 = scenario(init, vehicle, vec![burn], 4.0 * full_tank_s);
    sc2.cheats.infinite_fuel = true;
    let r2 = sc2.propagate();
    assert!(r2.reports[0].feasible);
    assert!((r2.final_mass_kg - vehicle.wet_mass_kg).abs() < 1e-9);
    assert!(r2.total_dv_km_s > result.total_dv_km_s);
}

/// Back-propagation then forward propagation must return to the start.
#[test]
fn backward_propagation_is_reversible() {
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0);
    let init = circular_state(7000.0, epoch);
    let sc = Scenario {
        init,
        vehicle: Vehicle::default(),
        maneuvers: vec![],
        cheats: Cheats::default(),
        t_start_s: -3600.0,
        t_end_s: 3600.0,
        output_dt_s: 60.0,
        enable_j2: true,
    };
    let result = sc.propagate();
    let first = result.trajectory.samples.first().unwrap();
    assert!((first.t_s - (-3600.0)).abs() < 1e-6);

    // Propagate forward from the earliest sample; we must pass through init.
    let sc_fwd = Scenario {
        init: State::new(
            epoch + hifitime::Duration::from_seconds(first.t_s),
            first.r,
            first.v,
            1000.0,
        ),
        vehicle: Vehicle::default(),
        maneuvers: vec![],
        cheats: Cheats::default(),
        t_start_s: 0.0,
        t_end_s: 3600.0,
        output_dt_s: 60.0,
        enable_j2: true,
    };
    let fwd = sc_fwd.propagate();
    let at_init = fwd.trajectory.sample_at(3600.0).unwrap();
    let pos_err = (at_init.r - init.r).length();
    assert!(pos_err < 1e-3, "round-trip position error {pos_err} km");
}

/// Orbital element extraction round-trips a known state.
#[test]
fn elements_from_rv_known_case() {
    // Vallado example 2-5 style check: use a state built from elements.
    let a = 8000.0;
    let e = 0.1;
    let rp = a * (1.0 - e);
    let vp = (MU_EARTH * (2.0 / rp - 1.0 / a)).sqrt();
    // Periapsis on +x, velocity along +y, inclined 30 deg about x-axis.
    let inc: f64 = 30.0_f64.to_radians();
    let r = DVec3::new(rp, 0.0, 0.0);
    let v = DVec3::new(0.0, vp * inc.cos(), vp * inc.sin());
    let el = elements_from_rv(r, v);
    assert!((el.sma_km - a).abs() < 1e-6);
    assert!((el.ecc - e).abs() < 1e-9);
    assert!((el.inc_deg - 30.0).abs() < 1e-9);
    assert!(el.true_anomaly_deg.abs() < 1e-6 || (el.true_anomaly_deg - 360.0).abs() < 1e-6);
}

/// TLE ingestion produces a plausible ISS state at epoch.
#[test]
fn tle_parse_iss() {
    let parsed = parse_tle(ISS_TLE).expect("TLE should parse");
    assert_eq!(parsed.norad_id, 25544);
    let el = elements_from_rv(parsed.state.r, parsed.state.v);
    assert!((el.inc_deg - 51.64).abs() < 0.1, "inc {}", el.inc_deg);
    let alt = parsed.state.r.length() - R_EARTH;
    assert!(
        (300.0..500.0).contains(&alt),
        "ISS altitude {alt} km out of range"
    );
}
