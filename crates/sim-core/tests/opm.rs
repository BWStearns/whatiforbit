//! OPM export validation: structure, maneuver blocks, achieved-dv semantics.

use glam::DVec3;
use hifitime::Epoch;
use whatiforbit_sim::{opm_kvn, Cheats, Maneuver, Scenario, State, Vehicle, G0, MU_EARTH};

fn test_scenario() -> Scenario {
    let r = 7000.0;
    let v = (MU_EARTH / r).sqrt();
    let epoch = Epoch::from_gregorian_utc_hms(2026, 1, 1, 12, 0, 0);
    Scenario {
        init: State::new(
            epoch,
            DVec3::new(r, 0.0, 0.0),
            DVec3::new(0.0, v, 0.0),
            0.0,
        ),
        vehicle: Vehicle {
            wet_mass_kg: 1000.0,
            dry_mass_kg: 500.0,
            thrust_n: 400.0,
            isp_s: 300.0,
        },
        maneuvers: vec![
            Maneuver::Impulsive {
                t_offset_s: 600.0,
                // 100 m/s prograde + 20 m/s "outward" (C axis).
                dv_vnc_km_s: DVec3::new(0.1, 0.0, 0.02),
            },
            Maneuver::FiniteBurn {
                t_offset_s: 3600.0,
                duration_s: 120.0,
                direction_vnc: DVec3::X,
                throttle: 1.0,
            },
        ],
        cheats: Cheats::default(),
        t_start_s: 0.0,
        t_end_s: 7200.0,
        output_dt_s: 60.0,
        enable_j2: true,
    }
}

#[test]
fn opm_structure_and_maneuvers() {
    let sc = test_scenario();
    let result = sc.propagate();
    let creation = Epoch::from_gregorian_utc_hms(2026, 7, 7, 0, 0, 0);
    let opm = opm_kvn(&sc, &result, "TESTSAT", "2026-001A", creation);

    // Header and metadata.
    assert!(opm.contains("CCSDS_OPM_VERS = 3.0"));
    assert!(opm.contains("OBJECT_NAME = TESTSAT"));
    assert!(opm.contains("OBJECT_ID = 2026-001A"));
    assert!(opm.contains("REF_FRAME = TEME"));
    assert!(opm.contains("EPOCH = 2026-01-01T12:00:00"));

    // State vector: X should equal 7000 km.
    assert!(opm.contains("X = 7000.000000 [km]"));
    assert!(opm.contains("MASS = 1000.000 [kg]"));

    // Two maneuver blocks.
    assert_eq!(opm.matches("MAN_EPOCH_IGNITION").count(), 2);
    assert!(opm.contains("MAN_EPOCH_IGNITION = 2026-01-01T12:10:00"));
    assert!(opm.contains("MAN_EPOCH_IGNITION = 2026-01-01T13:00:00"));

    // Impulse: TNW mapping T=V, N=-C, W=N.
    assert!(opm.contains("MAN_DV_1 = 0.100000000 [km/s]"));
    assert!(opm.contains("MAN_DV_2 = -0.020000000 [km/s]"));

    // Finite burn delta-mass must match mdot * duration.
    let mdot = 400.0 / (300.0 * G0);
    let expected_dm = -(mdot * 120.0);
    let dm_line = opm
        .lines()
        .filter(|l| l.starts_with("MAN_DELTA_MASS"))
        .nth(1)
        .expect("second MAN_DELTA_MASS line");
    let dm: f64 = dm_line
        .split('=')
        .nth(1)
        .unwrap()
        .replace("[kg]", "")
        .trim()
        .parse()
        .unwrap();
    assert!(
        (dm - expected_dm).abs() < 1e-3,
        "delta mass {dm} vs expected {expected_dm}"
    );

    // Burn dv magnitude consistent with the rocket equation.
    let dv1_burn: f64 = opm
        .lines()
        .filter(|l| l.starts_with("MAN_DV_1"))
        .nth(1)
        .unwrap()
        .split('=')
        .nth(1)
        .unwrap()
        .replace("[km/s]", "")
        .trim()
        .parse()
        .unwrap();
    // Mass before the burn = wet - impulse propellant (first report).
    let m_before = 1000.0 - result.reports[0].prop_used_kg;
    let m_after = m_before + expected_dm;
    let ve = 300.0 * G0 / 1000.0;
    let expected_dv = ve * (m_before / m_after).ln();
    assert!(
        (dv1_burn - expected_dv).abs() < 1e-6,
        "burn dv {dv1_burn} vs expected {expected_dv}"
    );
}

#[test]
fn opm_infinite_fuel_burn_dv() {
    let mut sc = test_scenario();
    sc.cheats.infinite_fuel = true;
    let result = sc.propagate();
    let creation = Epoch::from_gregorian_utc_hms(2026, 7, 7, 0, 0, 0);
    let opm = opm_kvn(&sc, &result, "TESTSAT", "UNKNOWN", creation);

    // No propellant used anywhere.
    for line in opm.lines().filter(|l| l.starts_with("MAN_DELTA_MASS")) {
        assert!(line.contains("-0.000000") || line.contains("= 0.000000"));
    }
    // Burn dv = F*t/m at constant mass.
    let expected = 400.0 * 120.0 / 1000.0 / 1000.0;
    let dv1_burn: f64 = opm
        .lines()
        .filter(|l| l.starts_with("MAN_DV_1"))
        .nth(1)
        .unwrap()
        .split('=')
        .nth(1)
        .unwrap()
        .replace("[km/s]", "")
        .trim()
        .parse()
        .unwrap();
    assert!(
        (dv1_burn - expected).abs() < 1e-9,
        "infinite-fuel burn dv {dv1_burn} vs {expected}"
    );
}
