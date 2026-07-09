use crate::{
    dynamics::{eom, pack, unpack, ThrustArc},
    integrator::Dp54,
    state::vnc_to_inertial,
    State, G0,
};
use glam::DVec3;
use hifitime::{Duration, Epoch};

/// Vehicle propulsion/mass properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vehicle {
    pub wet_mass_kg: f64,
    pub dry_mass_kg: f64,
    pub thrust_n: f64,
    pub isp_s: f64,
}

impl Vehicle {
    /// Exhaust velocity in km/s.
    pub fn ve_km_s(&self) -> f64 {
        self.isp_s * G0 / 1000.0
    }
}

impl Default for Vehicle {
    fn default() -> Self {
        Self {
            wet_mass_kg: 1000.0,
            dry_mass_kg: 600.0,
            thrust_n: 400.0,
            isp_s: 300.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Cheats {
    /// Ignore propellant limits and mass depletion entirely.
    pub infinite_fuel: bool,
}

/// A maneuver on the scenario timeline. Times are seconds relative to the
/// scenario's initial epoch and must be >= 0 (no burns in back-propagation).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Maneuver {
    Impulsive {
        t_offset_s: f64,
        /// Delta-v in km/s, VNC components.
        dv_vnc_km_s: DVec3,
    },
    FiniteBurn {
        t_offset_s: f64,
        duration_s: f64,
        /// Thrust direction in VNC components (normalized internally).
        direction_vnc: DVec3,
        /// 0..=1 throttle on the vehicle's rated thrust.
        throttle: f64,
    },
}

impl Maneuver {
    pub fn t_offset_s(&self) -> f64 {
        match self {
            Maneuver::Impulsive { t_offset_s, .. } => *t_offset_s,
            Maneuver::FiniteBurn { t_offset_s, .. } => *t_offset_s,
        }
    }
}

/// What actually happened when a maneuver executed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManeuverReport {
    pub index: usize,
    pub t_offset_s: f64,
    /// Magnitude requested vs achieved (km/s for impulses; seconds of burn
    /// for finite burns).
    pub requested: f64,
    pub achieved: f64,
    pub prop_used_kg: f64,
    pub feasible: bool,
}

/// One propagated point. `t_s` is seconds relative to the scenario epoch.
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub t_s: f64,
    pub r: DVec3,
    pub v: DVec3,
    pub mass: f64,
}

#[derive(Debug, Clone, Default)]
pub struct Trajectory {
    pub epoch: Option<Epoch>,
    /// Samples in ascending time order, spanning [t_start, t_end].
    pub samples: Vec<Sample>,
}

impl Trajectory {
    /// Linear interpolation between samples; clamps outside the span.
    pub fn sample_at(&self, t_s: f64) -> Option<Sample> {
        let s = &self.samples;
        if s.is_empty() {
            return None;
        }
        if t_s <= s[0].t_s {
            return Some(s[0]);
        }
        if t_s >= s[s.len() - 1].t_s {
            return Some(s[s.len() - 1]);
        }
        let idx = s.partition_point(|p| p.t_s < t_s);
        let (a, b) = (&s[idx - 1], &s[idx]);
        let f = ((t_s - a.t_s) / (b.t_s - a.t_s)).clamp(0.0, 1.0);
        Some(Sample {
            t_s,
            r: a.r.lerp(b.r, f),
            v: a.v.lerp(b.v, f),
            mass: a.mass + (b.mass - a.mass) * f,
        })
    }

    pub fn epoch_at(&self, t_s: f64) -> Option<Epoch> {
        self.epoch.map(|e| e + Duration::from_seconds(t_s))
    }
}

#[derive(Debug, Clone)]
pub struct Scenario {
    /// Initial orbital state; `init.mass` is overwritten by the vehicle wet mass.
    pub init: State,
    pub vehicle: Vehicle,
    pub maneuvers: Vec<Maneuver>,
    pub cheats: Cheats,
    /// Propagation span relative to `init.epoch`, seconds. start <= 0 <= end.
    pub t_start_s: f64,
    pub t_end_s: f64,
    pub output_dt_s: f64,
    /// Include the J2 zonal harmonic (off = pure two-body Keplerian motion).
    pub enable_j2: bool,
}

#[derive(Debug, Clone, Default)]
pub struct SimResult {
    pub trajectory: Trajectory,
    pub reports: Vec<ManeuverReport>,
    pub final_mass_kg: f64,
    pub total_dv_km_s: f64,
}

impl Scenario {
    /// Propagate the scenario and return the sampled trajectory plus a
    /// per-maneuver execution report.
    pub fn propagate(&self) -> SimResult {
        let integ = Dp54::default();
        let veh = self.vehicle;
        let j2 = self.enable_j2;
        let infinite = self.cheats.infinite_fuel;
        let dt_out = self.output_dt_s.max(0.1);

        let mut samples: Vec<Sample> = Vec::new();
        let mut reports = Vec::new();
        let mut total_dv = 0.0;

        // --- Backward arc (coast only), from t=0 down to t_start. ---
        if self.t_start_s < -1e-9 {
            let y0 = pack(self.init.r, self.init.v, veh.wet_mass_kg);
            let mut back: Vec<Sample> = Vec::new();
            integ.integrate(
                |y| eom(y, None, j2),
                0.0,
                y0,
                self.t_start_s,
                dt_out,
                |t, y| {
                    let (r, v, mass) = unpack(y);
                    back.push(Sample { t_s: t, r, v, mass });
                },
            );
            back.reverse();
            samples.extend(back);
        }

        // t = 0 sample.
        samples.push(Sample {
            t_s: 0.0,
            r: self.init.r,
            v: self.init.v,
            mass: veh.wet_mass_kg,
        });

        // --- Forward arc with maneuvers. ---
        let mut events: Vec<(usize, Maneuver)> = self
            .maneuvers
            .iter()
            .copied()
            .enumerate()
            .filter(|(_, m)| m.t_offset_s() >= 0.0 && m.t_offset_s() <= self.t_end_s)
            .collect();
        events.sort_by(|a, b| a.1.t_offset_s().total_cmp(&b.1.t_offset_s()));

        let mut t = 0.0;
        let mut y = pack(self.init.r, self.init.v, veh.wet_mass_kg);

        let record = |samples: &mut Vec<Sample>, t: f64, y: &[f64; 7]| {
            let (r, v, mass) = unpack(y);
            samples.push(Sample { t_s: t, r, v, mass });
        };

        for (index, m) in events {
            let t_event = m.t_offset_s().max(t);
            // Coast up to the event.
            if t_event > t + 1e-9 {
                y = integ.integrate(|yy| eom(yy, None, j2), t, y, t_event, dt_out, |tt, yy| {
                    record(&mut samples, tt, yy)
                });
                t = t_event;
            }

            let (r, v, mass) = unpack(&y);
            match m {
                Maneuver::Impulsive { dv_vnc_km_s, .. } => {
                    let requested = dv_vnc_km_s.length();
                    let ve = veh.ve_km_s();
                    let (achieved, mass_after) = if infinite {
                        (requested, mass)
                    } else {
                        let max_dv = ve * (mass / veh.dry_mass_kg).ln();
                        let a = requested.min(max_dv.max(0.0));
                        (a, mass * (-a / ve).exp())
                    };
                    let dv_inertial = if requested > 0.0 {
                        vnc_to_inertial(r, v, dv_vnc_km_s) * (achieved / requested)
                    } else {
                        DVec3::ZERO
                    };
                    y = pack(r, v + dv_inertial, mass_after);
                    record(&mut samples, t, &y);
                    total_dv += achieved;
                    reports.push(ManeuverReport {
                        index,
                        t_offset_s: m.t_offset_s(),
                        requested,
                        achieved,
                        prop_used_kg: mass - mass_after,
                        feasible: achieved >= requested - 1e-12,
                    });
                }
                Maneuver::FiniteBurn {
                    duration_s,
                    direction_vnc,
                    throttle,
                    ..
                } => {
                    let throttle = throttle.clamp(0.0, 1.0);
                    let thrust_n = veh.thrust_n * throttle;
                    let mdot = if infinite || veh.isp_s <= 0.0 {
                        0.0
                    } else {
                        thrust_n / (veh.isp_s * G0)
                    };
                    let requested = duration_s.max(0.0);
                    // Clamp burn time to available propellant so the EOM never
                    // has to handle in-flight cutoff.
                    let achievable = if infinite || mdot <= 0.0 {
                        requested
                    } else {
                        requested.min(((mass - veh.dry_mass_kg) / mdot).max(0.0))
                    };
                    let burn_end = (t + achievable).min(self.t_end_s);
                    let burn_s = (burn_end - t).max(0.0);
                    let mass_before = mass;
                    if burn_end > t + 1e-9 && thrust_n > 0.0 && direction_vnc.length() > 0.0 {
                        let arc = ThrustArc {
                            direction_vnc: direction_vnc.normalize(),
                            thrust_n,
                            mass_flow_kg_s: mdot,
                        };
                        y = integ.integrate(
                            |yy| eom(yy, Some(&arc), j2),
                            t,
                            y,
                            burn_end,
                            dt_out.min(10.0), // sample burns densely
                            |tt, yy| record(&mut samples, tt, yy),
                        );
                        t = burn_end;
                    }
                    let (_, _, mass_after) = unpack(&y);
                    // Expended delta-v, not net velocity change: |v_after -
                    // v_before| would fold in gravity's contribution over the
                    // arc. Constant thrust/mdot gives the exact expressions.
                    total_dv += if infinite || mdot <= 0.0 {
                        thrust_n * burn_s / mass_before / 1000.0
                    } else {
                        veh.ve_km_s() * (mass_before / mass_after).ln()
                    };
                    reports.push(ManeuverReport {
                        index,
                        t_offset_s: m.t_offset_s(),
                        requested,
                        achieved: achievable,
                        prop_used_kg: mass_before - mass_after,
                        feasible: achievable >= requested - 1e-9,
                    });
                }
            }
        }

        // Final coast to the end of the span.
        if self.t_end_s > t + 1e-9 {
            y = integ.integrate(|yy| eom(yy, None, j2), t, y, self.t_end_s, dt_out, |tt, yy| {
                record(&mut samples, tt, yy)
            });
        }

        samples.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
        samples.dedup_by(|a, b| (a.t_s - b.t_s).abs() < 1e-6);

        let final_mass = unpack(&y).2;
        SimResult {
            trajectory: Trajectory {
                epoch: Some(self.init.epoch),
                samples,
            },
            reports,
            final_mass_kg: final_mass,
            total_dv_km_s: total_dv,
        }
    }
}
