//! 3D scene: Earth, trajectory rendering, spacecraft markers, orbit camera.

use crate::types::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy_egui::EguiContexts;
use whatiforbit_sim::R_EARTH;

/// Render scale: 1 scene unit = 1000 km. Physics stays f64 km; only the
/// final positions are cast to f32.
pub const KM_PER_UNIT: f64 = 1000.0;

/// Map a physics vector (km, z = north) to render space (units, Y-up).
/// (x, y, z)_phys -> (x, z, -y)_render is a proper rotation.
pub fn to_render(v: glam::DVec3) -> Vec3 {
    Vec3::new(
        (v.x / KM_PER_UNIT) as f32,
        (v.z / KM_PER_UNIT) as f32,
        (-v.y / KM_PER_UNIT) as f32,
    )
}

/// The view's frame transform, resolved from the scenario epoch.
///
/// Inertial mode is the identity. Earth-fixed rotates every point by the GMST
/// at *its own* time — a whole-track rotation would just spin the picture; it
/// is the per-sample angle that draws the westward corkscrew. Interpolation
/// therefore has to happen in the inertial frame and be transformed here, at
/// draw time, or the curve cuts corners across the rotation.
#[derive(Clone, Copy)]
pub struct FrameXform {
    frame: ViewFrame,
    /// Epoch of t_s = 0, shared by every trajectory in the scenario.
    epoch0: Option<hifitime::Epoch>,
}

impl FrameXform {
    pub fn new(frame: ViewFrame, output: &SimOutput, input: &ScenarioInput) -> Self {
        let epoch0 = output
            .whatif
            .as_ref()
            .and_then(|w| w.trajectory.epoch)
            .or_else(|| output.baseline.as_ref().and_then(|b| b.trajectory.epoch))
            .or_else(|| input.loaded.as_ref().map(|l| l.state.epoch));
        Self { frame, epoch0 }
    }

    /// Rotation angle at `t_s`; zero in the inertial view, and zero with no
    /// epoch to hang it on (nothing is drawn in that case anyway).
    pub fn gmst(&self, t_s: f64) -> f64 {
        match (self.frame, self.epoch0) {
            (ViewFrame::EarthFixed, Some(e)) => {
                whatiforbit_sim::earth::gmst_rad(e + hifitime::Duration::from_seconds(t_s))
            }
            _ => 0.0,
        }
    }

    /// A physics vector at `t_s`, rotated into the view frame (still km).
    pub fn apply(&self, t_s: f64, v: glam::DVec3) -> glam::DVec3 {
        match self.frame {
            ViewFrame::Inertial => v,
            ViewFrame::EarthFixed => whatiforbit_sim::earth::eci_to_ecef(v, self.gmst(t_s)),
        }
    }

    /// A physics position at `t_s`, in render space. The one call every
    /// drawing and picking site must go through.
    pub fn r(&self, t_s: f64, r: glam::DVec3) -> Vec3 {
        to_render(self.apply(t_s, r))
    }

    /// Velocity in the view frame, including the transport term when
    /// Earth-fixed. Speed is frame-dependent; altitude is not.
    pub fn vel(&self, t_s: f64, r: glam::DVec3, v: glam::DVec3) -> glam::DVec3 {
        match self.frame {
            ViewFrame::Inertial => v,
            ViewFrame::EarthFixed => {
                whatiforbit_sim::earth::eci_to_ecef_vel(r, v, self.gmst(t_s))
            }
        }
    }
}

#[derive(Component)]
pub struct EarthGlobe;

#[derive(Component)]
pub struct SunLight;

#[derive(Component)]
pub struct WhatIfMarker;

#[derive(Component)]
pub struct BaselineMarker;

#[derive(Component)]
pub struct OrbitCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    commands.spawn((
        Camera3d::default(),
        Camera {
            clear_color: ClearColorConfig::Custom(Color::srgb(0.008, 0.01, 0.02)),
            ..default()
        },
        Transform::from_xyz(0.0, 15.0, 35.0).looking_at(Vec3::ZERO, Vec3::Y),
        OrbitCamera {
            yaw: 0.6,
            pitch: 0.35,
            dist: 38.0,
        },
    ));

    // Sunlight: direction is driven per-frame from the solar ephemeris at the
    // scrubbed epoch (see `update_earth_and_sun`).
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            ..default()
        },
        Transform::from_xyz(50.0, 20.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
        SunLight,
    ));
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        ..default()
    });

    // Earth. NASA Blue Marble (land_shallow_topo_2048, public domain),
    // equirectangular on a UV sphere. The globe's rotation is driven from
    // GMST at the scrubbed epoch (see `update_earth_and_sun`), so longitude
    // alignment against the TEME frame is meaningful at SGP4 fidelity.
    let earth_radius = (R_EARTH / KM_PER_UNIT) as f32;
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(earth_radius).mesh().uv(64, 32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(asset_server.load("earth.jpg")),
            perceptual_roughness: 0.9,
            ..default()
        })),
        Transform::from_rotation(earth_rotation(0.0)),
        EarthGlobe,
    ));

    // Spacecraft markers: what-if (orange) and baseline ghost (gray).
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.12).mesh().uv(16, 12))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.6, 0.1),
            emissive: LinearRgba::rgb(2.0, 1.0, 0.1),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Visibility::Hidden,
        WhatIfMarker,
    ));
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.09).mesh().uv(16, 12))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.7, 0.7, 0.8, 0.6),
            emissive: LinearRgba::rgb(0.4, 0.4, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        Visibility::Hidden,
        BaselineMarker,
    ));
}

/// Globe orientation for a given GMST angle.
///
/// Composition: first tip the mesh poles from +Z onto +Y (render north), then
/// yaw about the pole. The texture's Greenwich meridian sits on mesh -X
/// (equirectangular u=0 is longitude -180 on mesh +X), and GMST measures
/// Greenwich eastward from the vernal equinox (+X inertial), so the yaw is
/// GMST - pi.
pub fn earth_rotation(gmst_rad: f64) -> Quat {
    Quat::from_rotation_y((gmst_rad - std::f64::consts::PI) as f32)
        * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
}

/// The epoch currently shown on screen: the scrubbed time when a trajectory
/// exists, else the loaded TLE epoch.
fn display_epoch(
    output: &SimOutput,
    input: &ScenarioInput,
    playback: &Playback,
) -> Option<hifitime::Epoch> {
    output
        .whatif
        .as_ref()
        .and_then(|w| w.trajectory.epoch_at(playback.t_s))
        .or_else(|| input.loaded.as_ref().map(|l| l.state.epoch))
}

/// Rotate the globe to GMST and aim the sunlight along the real solar
/// direction for the epoch being displayed. This is what makes the day/night
/// terminator and (future) eclipse geometry meaningful.
///
/// In the Earth-fixed view the globe instead holds still at yaw 0, which puts
/// Greenwich on physics +X — exactly where `eci_to_ecef` sends the GMST
/// meridian. The two views therefore agree on where the spacecraft is over the
/// ground at the scrubbed instant; they differ only in what the track does
/// away from it. The sun has to take the same rotation, or the terminator
/// lands on the wrong meridian.
#[allow(clippy::type_complexity)]
pub fn update_earth_and_sun(
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    playback: Res<Playback>,
    frame: Res<ViewFrame>,
    mut globe: Query<&mut Transform, (With<EarthGlobe>, Without<SunLight>)>,
    mut sun: Query<&mut Transform, (With<SunLight>, Without<EarthGlobe>)>,
) {
    let Some(epoch) = display_epoch(&output, &input, &playback) else {
        return;
    };
    let gmst = whatiforbit_sim::earth::gmst_rad(epoch);
    if let Ok(mut tf) = globe.single_mut() {
        tf.rotation = earth_rotation(if frame.is_earth_fixed() { 0.0 } else { gmst });
    }
    if let Ok(mut tf) = sun.single_mut() {
        let s = whatiforbit_sim::earth::sun_direction(epoch);
        let s = if frame.is_earth_fixed() {
            whatiforbit_sim::earth::eci_to_ecef(s, gmst)
        } else {
            s
        };
        // Light travels from the sun toward Earth: along -s.
        let dir = -Vec3::new(s.x as f32, s.z as f32, -s.y as f32);
        *tf = Transform::default().looking_to(dir, Vec3::Y);
    }
}

/// Immediate-mode rendering of both trajectories, pole axis, maneuver node
/// markers, and the hovered-point highlight.
pub fn draw_trajectories(
    mut gizmos: Gizmos,
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    hover: Res<TrackHover>,
    solve: Res<TargetSolve>,
    frame: Res<ViewFrame>,
) {
    let xf = FrameXform::new(*frame, &output, &input);

    // Candidate preview (hovered row in the target-orbit table): dashed cyan.
    if let Some(preview) = &solve.preview {
        for pair in preview.samples.chunks(2) {
            if let [a, b] = pair {
                gizmos.line(
                    xf.r(a.t_s, a.r),
                    xf.r(b.t_s, b.r),
                    Color::srgba(0.3, 0.9, 1.0, 0.8),
                );
            }
        }
    }
    if let Some(h) = &hover.0 {
        gizmos.sphere(
            Isometry3d::from_translation(xf.r(h.t_s, h.r_km)),
            0.14,
            Color::WHITE,
        );
    }
    // Polar axis for orientation. Invariant under the frame rotation, which
    // is about this very axis, so it needs no transform.
    let pole = to_render(glam::DVec3::new(0.0, 0.0, R_EARTH + 1500.0));
    gizmos.line(-pole, pole, Color::srgba(0.6, 0.6, 0.9, 0.4));

    if let Some(baseline) = &output.baseline {
        let pts: Vec<Vec3> = baseline
            .trajectory
            .samples
            .iter()
            .map(|s| xf.r(s.t_s, s.r))
            .collect();
        gizmos.linestrip(pts, Color::srgba(0.55, 0.6, 0.7, 0.7));
    }
    if let Some(whatif) = &output.whatif {
        let pts: Vec<Vec3> = whatif
            .trajectory
            .samples
            .iter()
            .map(|s| xf.r(s.t_s, s.r))
            .collect();
        gizmos.linestrip(pts, Color::srgb(1.0, 0.6, 0.1));

        // Maneuver nodes (stacked timing: absolute starts derived from gaps).
        let starts = crate::sim::stacked_start_times_s(&input.maneuvers);
        for (m, start_s) in input.maneuvers.iter().zip(starts) {
            if let Some(s) = whatif.trajectory.sample_at(start_s) {
                let color = match m.kind {
                    ManeuverKind::Impulsive => Color::srgb(0.3, 1.0, 0.4),
                    ManeuverKind::FiniteBurn => Color::srgb(1.0, 0.3, 0.4),
                };
                gizmos.sphere(Isometry3d::from_translation(xf.r(s.t_s, s.r)), 0.18, color);
            }
        }
    }
}

#[allow(clippy::type_complexity)]
pub fn update_markers(
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    playback: Res<Playback>,
    frame: Res<ViewFrame>,
    mut whatif_q: Query<(&mut Transform, &mut Visibility), (With<WhatIfMarker>, Without<BaselineMarker>)>,
    mut baseline_q: Query<(&mut Transform, &mut Visibility), (With<BaselineMarker>, Without<WhatIfMarker>)>,
) {
    let xf = FrameXform::new(*frame, &output, &input);
    if let Ok((mut tf, mut vis)) = whatif_q.single_mut() {
        match output
            .whatif
            .as_ref()
            .and_then(|w| w.trajectory.sample_at(playback.t_s))
        {
            Some(s) => {
                tf.translation = xf.r(s.t_s, s.r);
                *vis = Visibility::Visible;
            }
            None => *vis = Visibility::Hidden,
        }
    }
    if let Ok((mut tf, mut vis)) = baseline_q.single_mut() {
        match output
            .baseline
            .as_ref()
            .and_then(|w| w.trajectory.sample_at(playback.t_s))
        {
            Some(s) => {
                tf.translation = xf.r(s.t_s, s.r);
                *vis = Visibility::Visible;
            }
            None => *vis = Visibility::Hidden,
        }
    }
}

pub fn orbit_camera(
    mut contexts: EguiContexts,
    buttons: Res<ButtonInput<MouseButton>>,
    mut motion: EventReader<MouseMotion>,
    mut wheel: EventReader<MouseWheel>,
    mut q: Query<(&mut OrbitCamera, &mut Transform)>,
) {
    let egui_wants_input = contexts
        .ctx_mut()
        .map(|ctx| ctx.wants_pointer_input() || ctx.is_pointer_over_area())
        .unwrap_or(false);

    let Ok((mut cam, mut tf)) = q.single_mut() else {
        return;
    };

    if !egui_wants_input {
        if buttons.pressed(MouseButton::Left) || buttons.pressed(MouseButton::Right) {
            for ev in motion.read() {
                cam.yaw -= ev.delta.x * 0.005;
                cam.pitch = (cam.pitch + ev.delta.y * 0.005).clamp(-1.54, 1.54);
            }
        } else {
            motion.clear();
        }
        for ev in wheel.read() {
            let amount = match ev.unit {
                bevy::input::mouse::MouseScrollUnit::Line => ev.y,
                bevy::input::mouse::MouseScrollUnit::Pixel => ev.y / 50.0,
            };
            cam.dist = (cam.dist * (1.0 - amount * 0.1)).clamp(7.5, 400.0);
        }
    } else {
        motion.clear();
        wheel.clear();
    }

    let pos = Vec3::new(
        cam.dist * cam.pitch.cos() * cam.yaw.sin(),
        cam.dist * cam.pitch.sin(),
        cam.dist * cam.pitch.cos() * cam.yaw.cos(),
    );
    *tf = Transform::from_translation(pos).looking_at(Vec3::ZERO, Vec3::Y);
}

#[cfg(test)]
mod tests {
    use super::*;
    use whatiforbit_sim::earth::eci_to_ecef;

    /// The two views must agree on where the spacecraft is *over the ground*
    /// at a given instant: rotating the globe by GMST and drawing the
    /// inertial position has to place the point on the same patch of Earth as
    /// holding the globe still and drawing the Earth-fixed position. This
    /// pins down the sign conventions shared by `to_render`, `earth_rotation`
    /// and `eci_to_ecef` — get any one of them backwards and the marker jumps
    /// when the view frame is toggled.
    #[test]
    fn both_frames_agree_on_the_ground_point() {
        let r = glam::DVec3::new(4_100.0, -3_900.0, 4_500.0);
        for gmst in [0.0, 0.7, 2.9, 4.5, 6.0] {
            let inertial = earth_rotation(gmst).inverse() * to_render(r);
            let earth_fixed = earth_rotation(0.0).inverse() * to_render(eci_to_ecef(r, gmst));
            assert!(
                inertial.distance(earth_fixed) < 1e-4,
                "GMST {gmst}: {inertial:?} vs {earth_fixed:?}",
            );
        }
    }

    /// The Earth-fixed view is a rotation about the pole, so it moves neither
    /// altitude nor the polar axis the orientation gizmo is drawn along.
    #[test]
    fn earth_fixed_view_preserves_altitude_and_pole() {
        let xf = FrameXform {
            frame: ViewFrame::EarthFixed,
            epoch0: Some(hifitime::Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0)),
        };
        let r = glam::DVec3::new(4_100.0, -3_900.0, 4_500.0);
        for t_s in [0.0, 600.0, 43_200.0] {
            assert!((xf.apply(t_s, r).length() - r.length()).abs() < 1e-9);
            let pole = glam::DVec3::new(0.0, 0.0, 7_000.0);
            assert!((xf.apply(t_s, pole) - pole).length() < 1e-9);
        }
    }

    /// The inertial view is the identity, and stays so as time advances.
    #[test]
    fn inertial_view_is_the_identity() {
        let xf = FrameXform {
            frame: ViewFrame::Inertial,
            epoch0: Some(hifitime::Epoch::from_gregorian_utc_hms(2026, 1, 1, 0, 0, 0)),
        };
        let (r, v) = (
            glam::DVec3::new(4_100.0, -3_900.0, 4_500.0),
            glam::DVec3::new(-5.0, 4.0, 2.0),
        );
        assert_eq!(xf.apply(9_000.0, r), r);
        assert_eq!(xf.vel(9_000.0, r, v), v);
        assert_eq!(xf.gmst(9_000.0), 0.0);
    }
}
