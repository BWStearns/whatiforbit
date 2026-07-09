//! Impulsive strategy templates.
//!
//! These produce the *structure* of a transfer (how many burns, where, with
//! what impulsive delta-v) in a coaxial/apsis-aligned approximation. They are
//! seeds and cost bounds for the finite-burn optimizer, and the basis for
//! feasibility math — not the final answer.
//!
//! Approximations, documented deliberately:
//! - Orbits are treated as coaxial (argp alignment ignored); costs are exact
//!   for apsis-aligned geometry and conservative-ish otherwise.
//! - Plane changes are priced at the radius of the burn they're combined
//!   with; the sign of the normal component is nominal (the optimizer refines
//!   node placement in M2).

use super::kepler::{period_s, time_between_anomalies_s, visviva_speed};
use super::target::SolveConstraints;
use crate::{OrbitalElements, R_EARTH};
use glam::DVec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneChangeSite {
    None,
    WithBurn1,
    WithBurn2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StrategyKind {
    /// Up-to-two tangential burns at apsides (generalized Hohmann), with the
    /// plane change optionally folded into one of them.
    TwoBurn { plane_change_at: PlaneChangeSite },
    /// Three burns via a high intermediate apoapsis; plane change combined
    /// with the slow middle burn.
    BiElliptic { rb_km: f64 },
    /// Pure plane rotation, no size/shape change.
    PureRotation,
    /// Low-thrust many-rev spiral estimate (Edelbaum). No discrete burns —
    /// cost/duration estimate only.
    EdelbaumSpiral,
}

impl StrategyKind {
    pub fn label(&self) -> String {
        match self {
            StrategyKind::TwoBurn { plane_change_at } => match plane_change_at {
                PlaneChangeSite::None => "Two-burn transfer".into(),
                PlaneChangeSite::WithBurn1 => "Two-burn, plane change at burn 1".into(),
                PlaneChangeSite::WithBurn2 => "Two-burn, plane change at burn 2".into(),
            },
            StrategyKind::BiElliptic { rb_km } => {
                format!("Bi-elliptic via {:.0} km", rb_km - R_EARTH)
            }
            StrategyKind::PureRotation => "Plane change only".into(),
            StrategyKind::EdelbaumSpiral => "Low-thrust spiral (estimate)".into(),
        }
    }
}

/// One impulsive burn of a template plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpulsiveBurn {
    /// Seconds after scenario epoch (coast timing via Kepler).
    pub t_offset_s: f64,
    /// Delta-v in VNC components, km/s.
    pub dv_vnc_km_s: DVec3,
    /// Radius at which the burn occurs, km.
    pub radius_km: f64,
}

impl ImpulsiveBurn {
    pub fn dv_mag(&self) -> f64 {
        self.dv_vnc_km_s.length()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ImpulsivePlan {
    pub kind: StrategyKind,
    /// Burns in time order. Empty for `EdelbaumSpiral`.
    pub burns: Vec<ImpulsiveBurn>,
    pub total_dv_km_s: f64,
    /// Coast + transfer duration from scenario epoch to arrival, seconds.
    pub duration_s: f64,
    /// Lowest transfer periapsis altitude encountered, km (for the floor
    /// constraint check).
    pub min_transfer_peri_alt_km: f64,
}

/// A burn at radius `r` on the orbit `(r, other_from)` that moves the other
/// apsis to `other_to`, optionally rotating the plane by `di_rad`.
/// Returns (dv_vnc, dv_mag).
fn apsis_burn(r: f64, other_from: f64, other_to: f64, di_rad: f64) -> (DVec3, f64) {
    let a_from = (r + other_from) / 2.0;
    let a_to = (r + other_to) / 2.0;
    let v_from = visviva_speed(a_from, r);
    let v_to = visviva_speed(a_to, r);
    if di_rad.abs() < 1e-12 {
        let dv = v_to - v_from;
        (DVec3::new(dv, 0.0, 0.0), dv.abs())
    } else {
        // Combined magnitude change + rotation: law of cosines.
        let dv_along = v_to * di_rad.cos() - v_from;
        let dv_normal = v_to * di_rad.sin();
        let mag = (v_from * v_from + v_to * v_to - 2.0 * v_from * v_to * di_rad.cos()).sqrt();
        (DVec3::new(dv_along, dv_normal, 0.0), mag)
    }
}

/// Shared context for template generation.
struct Ctx {
    rp1: f64,
    ra1: f64,
    rp2: f64,
    ra2: f64,
    di_rad: f64,
    a1: f64,
    e1: f64,
    nu1_rad: f64,
}

impl Ctx {
    /// Coast time from the current position to first periapsis / apoapsis.
    fn time_to_apsis(&self, apsis_r: f64) -> f64 {
        let nu_target = if (apsis_r - self.rp1).abs() < (apsis_r - self.ra1).abs() {
            0.0
        } else {
            std::f64::consts::PI
        };
        time_between_anomalies_s(self.a1, self.e1, self.nu1_rad, nu_target)
    }
}

/// Build a two-burn (generalized Hohmann) plan for one apsis ordering.
/// `first_at_apo`: whether burn 1 happens at the current orbit's apoapsis
/// (changing periapsis first) or at its periapsis (changing apoapsis first).
fn two_burn(ctx: &Ctx, first_at_apo: bool, plane_at: PlaneChangeSite) -> ImpulsivePlan {
    // Burn 1 at radius r1 changes the opposite apsis from o_from to o_to;
    // burn 2 then happens at radius o_to and fixes the final other apsis.
    let (r1, o_from, o_to, final_other) = if first_at_apo {
        (ctx.ra1, ctx.rp1, ctx.rp2, ctx.ra2)
    } else {
        (ctx.rp1, ctx.ra1, ctx.ra2, ctx.rp2)
    };

    let (di1, di2) = match plane_at {
        PlaneChangeSite::None => (0.0, 0.0),
        PlaneChangeSite::WithBurn1 => (ctx.di_rad, 0.0),
        PlaneChangeSite::WithBurn2 => (0.0, ctx.di_rad),
    };

    let (dv1_vnc, dv1) = apsis_burn(r1, o_from, o_to, di1);
    let (dv2_vnc, dv2) = apsis_burn(o_to, r1, final_other, di2);

    let t1 = ctx.time_to_apsis(r1);
    let transfer_a = (r1 + o_to) / 2.0;
    let t2 = t1 + period_s(transfer_a) / 2.0;

    // Floor check applies to the transfer arc; the final orbit is validated
    // separately as the target.
    let min_peri = r1.min(o_to) - R_EARTH;

    ImpulsivePlan {
        kind: StrategyKind::TwoBurn {
            plane_change_at: plane_at,
        },
        burns: vec![
            ImpulsiveBurn {
                t_offset_s: t1,
                dv_vnc_km_s: dv1_vnc,
                radius_km: r1,
            },
            ImpulsiveBurn {
                t_offset_s: t2,
                dv_vnc_km_s: dv2_vnc,
                radius_km: o_to,
            },
        ],
        total_dv_km_s: dv1 + dv2,
        duration_s: t2,
        min_transfer_peri_alt_km: min_peri,
    }
}

/// Bi-elliptic via intermediate apoapsis rb: raise apo to rb at current
/// periapsis, do the periapsis-retarget + full plane change at rb (slowest
/// point), trim apoapsis at the final periapsis.
fn bi_elliptic(ctx: &Ctx, rb: f64) -> ImpulsivePlan {
    let (dv1_vnc, dv1) = apsis_burn(ctx.rp1, ctx.ra1, rb, 0.0);
    let (dv2_vnc, dv2) = apsis_burn(rb, ctx.rp1, ctx.rp2, ctx.di_rad);
    let (dv3_vnc, dv3) = apsis_burn(ctx.rp2, rb, ctx.ra2, 0.0);

    let t1 = ctx.time_to_apsis(ctx.rp1);
    let t2 = t1 + period_s((ctx.rp1 + rb) / 2.0) / 2.0;
    let t3 = t2 + period_s((ctx.rp2 + rb) / 2.0) / 2.0;

    ImpulsivePlan {
        kind: StrategyKind::BiElliptic { rb_km: rb },
        burns: vec![
            ImpulsiveBurn {
                t_offset_s: t1,
                dv_vnc_km_s: dv1_vnc,
                radius_km: ctx.rp1,
            },
            ImpulsiveBurn {
                t_offset_s: t2,
                dv_vnc_km_s: dv2_vnc,
                radius_km: rb,
            },
            ImpulsiveBurn {
                t_offset_s: t3,
                dv_vnc_km_s: dv3_vnc,
                radius_km: ctx.rp2,
            },
        ],
        total_dv_km_s: dv1 + dv2 + dv3,
        duration_s: t3,
        min_transfer_peri_alt_km: ctx.rp1.min(ctx.rp2) - R_EARTH,
    }
}

/// Edelbaum circular-to-circular + inclination low-thrust estimate.
/// Returns None when either endpoint is meaningfully eccentric.
pub fn edelbaum_estimate(
    current: &OrbitalElements,
    a2: f64,
    e2: f64,
    di_rad: f64,
    accel_m_s2: f64,
) -> Option<ImpulsivePlan> {
    if current.ecc > 0.15 || e2 > 0.15 {
        return None;
    }
    let v0 = visviva_speed(current.sma_km, current.sma_km);
    let v1 = visviva_speed(a2, a2);
    let dv = (v0 * v0 + v1 * v1
        - 2.0 * v0 * v1 * (std::f64::consts::FRAC_PI_2 * di_rad.abs()).cos())
    .sqrt();
    let duration = if accel_m_s2 > 0.0 {
        dv * 1000.0 / accel_m_s2
    } else {
        f64::INFINITY
    };
    Some(ImpulsivePlan {
        kind: StrategyKind::EdelbaumSpiral,
        burns: vec![],
        total_dv_km_s: dv,
        duration_s: duration,
        min_transfer_peri_alt_km: current.sma_km.min(a2) - R_EARTH,
    })
}

/// Enumerate impulsive candidate plans for reaching (a2, e2, i2).
///
/// `accel_m_s2` (vehicle thrust / wet mass) powers the Edelbaum estimate.
pub fn enumerate_templates(
    current: &OrbitalElements,
    a2: f64,
    e2: f64,
    i2_deg: f64,
    constraints: &SolveConstraints,
    accel_m_s2: f64,
) -> Vec<ImpulsivePlan> {
    let ctx = Ctx {
        rp1: current.sma_km * (1.0 - current.ecc),
        ra1: current.sma_km * (1.0 + current.ecc),
        rp2: a2 * (1.0 - e2),
        ra2: a2 * (1.0 + e2),
        di_rad: (i2_deg - current.inc_deg).to_radians(),
        a1: current.sma_km,
        e1: current.ecc,
        nu1_rad: current.true_anomaly_deg.to_radians(),
    };

    let mut plans: Vec<ImpulsivePlan> = Vec::new();
    let shape_changes =
        (ctx.rp2 - ctx.rp1).abs() > 1e-6 || (ctx.ra2 - ctx.ra1).abs() > 1e-6;
    let plane_changes = ctx.di_rad.abs() > 1e-9;

    if shape_changes {
        let sites: &[PlaneChangeSite] = if plane_changes {
            &[PlaneChangeSite::WithBurn1, PlaneChangeSite::WithBurn2]
        } else {
            &[PlaneChangeSite::None]
        };
        for &site in sites {
            for first_at_apo in [true, false] {
                plans.push(two_burn(&ctx, first_at_apo, site));
            }
        }

        // Bi-elliptic sweep, only kept if it beats the best two-burn.
        let best_two = plans
            .iter()
            .map(|p| p.total_dv_km_s)
            .fold(f64::INFINITY, f64::min);
        let rb_min = ctx.ra1.max(ctx.ra2) * 1.05;
        let rb_max = constraints.max_intermediate_apo_alt_km + R_EARTH;
        if rb_max > rb_min {
            let best_bi = (0..16)
                .map(|k| {
                    let f = k as f64 / 15.0;
                    // Geometric spacing explores the long tail sensibly.
                    let rb = rb_min * (rb_max / rb_min).powf(f);
                    bi_elliptic(&ctx, rb)
                })
                .min_by(|x, y| x.total_dv_km_s.total_cmp(&y.total_dv_km_s));
            if let Some(bi) = best_bi {
                if bi.total_dv_km_s < best_two {
                    plans.push(bi);
                }
            }
        }
    } else if plane_changes {
        // Pure rotation at the slowest point (apoapsis).
        let (dv_vnc, dv) = apsis_burn(ctx.ra1, ctx.rp1, ctx.rp1, ctx.di_rad);
        let t1 = ctx.time_to_apsis(ctx.ra1);
        plans.push(ImpulsivePlan {
            kind: StrategyKind::PureRotation,
            burns: vec![ImpulsiveBurn {
                t_offset_s: t1,
                dv_vnc_km_s: dv_vnc,
                radius_km: ctx.ra1,
            }],
            total_dv_km_s: dv,
            duration_s: t1,
            min_transfer_peri_alt_km: ctx.rp1 - R_EARTH,
        });
    }

    if let Some(ed) = edelbaum_estimate(current, a2, e2, ctx.di_rad, accel_m_s2) {
        plans.push(ed);
    }

    // Drop plans violating the transfer perigee floor.
    plans.retain(|p| p.min_transfer_peri_alt_km >= constraints.min_transfer_perigee_alt_km - 1e-9);
    plans
}
