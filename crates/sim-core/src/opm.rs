//! CCSDS Orbit Parameter Message (OPM) export in KVN format.
//!
//! Produces an OPM v3 message describing the scenario's initial state vector,
//! osculating elements, spacecraft mass, and one MANEUVER block per executed
//! maneuver. Maneuver delta-v components are expressed in the CCSDS TNW
//! rotating frame; this app's native VNC frame maps onto it as
//! T = V (along-track), W = N (orbit normal), N_tnw = W x T = -C.

use crate::{
    elements_from_rv,
    scenario::{Maneuver, Scenario, SimResult},
    MU_EARTH,
};
use hifitime::{Duration, Epoch};
use std::fmt::Write;

/// Format an epoch as CCSDS-style ISO-8601 UTC without the trailing " UTC".
fn iso(epoch: Epoch) -> String {
    epoch.to_string().trim_end_matches(" UTC").to_string()
}

/// Generate an OPM (KVN) for the scenario and its propagation result.
///
/// `object_name`/`object_id` identify the spacecraft (OBJECT_ID falls back to
/// "UNKNOWN" when no international designator is available from a TLE).
pub fn opm_kvn(
    scenario: &Scenario,
    result: &SimResult,
    object_name: &str,
    object_id: &str,
    creation_date: Epoch,
) -> String {
    let mut s = String::new();
    let veh = scenario.vehicle;

    let _ = writeln!(s, "CCSDS_OPM_VERS = 3.0");
    let _ = writeln!(s, "CREATION_DATE = {}", iso(creation_date));
    let _ = writeln!(s, "ORIGINATOR = WHATIFORBIT");
    let _ = writeln!(s);
    let _ = writeln!(s, "OBJECT_NAME = {object_name}");
    let _ = writeln!(s, "OBJECT_ID = {object_id}");
    let _ = writeln!(s, "CENTER_NAME = EARTH");
    // TEME-at-epoch is this app's working inertial frame (see lib.rs docs).
    let _ = writeln!(s, "REF_FRAME = TEME");
    let _ = writeln!(s, "TIME_SYSTEM = UTC");
    let _ = writeln!(s);
    let _ = writeln!(s, "COMMENT State vector at scenario epoch");
    let _ = writeln!(s, "EPOCH = {}", iso(scenario.init.epoch));
    let _ = writeln!(s, "X = {:.6} [km]", scenario.init.r.x);
    let _ = writeln!(s, "Y = {:.6} [km]", scenario.init.r.y);
    let _ = writeln!(s, "Z = {:.6} [km]", scenario.init.r.z);
    let _ = writeln!(s, "X_DOT = {:.9} [km/s]", scenario.init.v.x);
    let _ = writeln!(s, "Y_DOT = {:.9} [km/s]", scenario.init.v.y);
    let _ = writeln!(s, "Z_DOT = {:.9} [km/s]", scenario.init.v.z);

    let el = elements_from_rv(scenario.init.r, scenario.init.v);
    let _ = writeln!(s);
    let _ = writeln!(s, "COMMENT Osculating Keplerian elements");
    let _ = writeln!(s, "SEMI_MAJOR_AXIS = {:.6} [km]", el.sma_km);
    let _ = writeln!(s, "ECCENTRICITY = {:.9}", el.ecc);
    let _ = writeln!(s, "INCLINATION = {:.6} [deg]", el.inc_deg);
    let _ = writeln!(s, "RA_OF_ASC_NODE = {:.6} [deg]", el.raan_deg);
    let _ = writeln!(s, "ARG_OF_PERICENTER = {:.6} [deg]", el.argp_deg);
    let _ = writeln!(s, "TRUE_ANOMALY = {:.6} [deg]", el.true_anomaly_deg);
    let _ = writeln!(s, "GM = {MU_EARTH:.4} [km**3/s**2]");
    let _ = writeln!(s);
    let _ = writeln!(s, "COMMENT Spacecraft parameters");
    let _ = writeln!(s, "MASS = {:.3} [kg]", veh.wet_mass_kg);

    // One MANEUVER block per executed maneuver, in execution order. Delta-v
    // is the *achieved* delta-v (propellant-limited maneuvers export what
    // actually happened, matching the propagated trajectory).
    let mut mass = veh.wet_mass_kg;
    for report in &result.reports {
        let Some(m) = scenario.maneuvers.get(report.index) else {
            continue;
        };
        let epoch = scenario.init.epoch + Duration::from_seconds(report.t_offset_s);
        let mass_after = mass - report.prop_used_kg;

        // Achieved delta-v vector in VNC.
        let (dv_vnc, duration_s) = match *m {
            Maneuver::Impulsive { dv_vnc_km_s, .. } => {
                let scale = if report.requested > 0.0 {
                    report.achieved / report.requested
                } else {
                    0.0
                };
                (dv_vnc_km_s * scale, 0.0)
            }
            Maneuver::FiniteBurn {
                direction_vnc,
                throttle,
                ..
            } => {
                let dir = direction_vnc.normalize_or_zero();
                let dv_mag = if report.prop_used_kg > 0.0 {
                    // Rocket equation over the burn arc.
                    veh.ve_km_s() * (mass / mass_after).ln()
                } else {
                    // Infinite-fuel cheat: constant mass, dv = a * t.
                    veh.thrust_n * throttle.clamp(0.0, 1.0) * report.achieved
                        / mass
                        / 1000.0
                };
                (dir * dv_mag, report.achieved)
            }
        };

        // VNC (V, N, C) -> CCSDS TNW (T, N, W): T = V, W = N, N_tnw = -C.
        let _ = writeln!(s);
        let _ = writeln!(
            s,
            "COMMENT Maneuver {} ({}){}",
            report.index + 1,
            match m {
                Maneuver::Impulsive { .. } => "impulsive",
                Maneuver::FiniteBurn { .. } => "finite burn",
            },
            if report.feasible {
                ""
            } else {
                " - propellant-limited, achieved values exported"
            }
        );
        let _ = writeln!(s, "MAN_EPOCH_IGNITION = {}", iso(epoch));
        let _ = writeln!(s, "MAN_DURATION = {duration_s:.3} [s]");
        let _ = writeln!(s, "MAN_DELTA_MASS = {:.6} [kg]", -report.prop_used_kg);
        let _ = writeln!(s, "MAN_REF_FRAME = TNW");
        let _ = writeln!(s, "MAN_DV_1 = {:.9} [km/s]", dv_vnc.x);
        let _ = writeln!(s, "MAN_DV_2 = {:.9} [km/s]", -dv_vnc.z);
        let _ = writeln!(s, "MAN_DV_3 = {:.9} [km/s]", dv_vnc.y);

        mass = mass_after;
    }

    s
}
