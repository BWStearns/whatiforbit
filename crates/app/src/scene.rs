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
#[allow(clippy::type_complexity)]
pub fn update_earth_and_sun(
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    playback: Res<Playback>,
    mut globe: Query<&mut Transform, (With<EarthGlobe>, Without<SunLight>)>,
    mut sun: Query<&mut Transform, (With<SunLight>, Without<EarthGlobe>)>,
) {
    let Some(epoch) = display_epoch(&output, &input, &playback) else {
        return;
    };
    if let Ok(mut tf) = globe.single_mut() {
        tf.rotation = earth_rotation(whatiforbit_sim::earth::gmst_rad(epoch));
    }
    if let Ok(mut tf) = sun.single_mut() {
        let s = whatiforbit_sim::earth::sun_direction(epoch);
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
) {
    // Candidate preview (hovered row in the target-orbit table): dashed cyan.
    if let Some(preview) = &solve.preview {
        for pair in preview.samples.chunks(2) {
            if let [a, b] = pair {
                gizmos.line(
                    to_render(a.r),
                    to_render(b.r),
                    Color::srgba(0.3, 0.9, 1.0, 0.8),
                );
            }
        }
    }
    if let Some(h) = &hover.0 {
        gizmos.sphere(
            Isometry3d::from_translation(to_render(h.r_km)),
            0.14,
            Color::WHITE,
        );
    }
    // Polar axis for orientation.
    let pole = to_render(glam::DVec3::new(0.0, 0.0, R_EARTH + 1500.0));
    gizmos.line(-pole, pole, Color::srgba(0.6, 0.6, 0.9, 0.4));

    if let Some(baseline) = &output.baseline {
        let pts: Vec<Vec3> = baseline
            .trajectory
            .samples
            .iter()
            .map(|s| to_render(s.r))
            .collect();
        gizmos.linestrip(pts, Color::srgba(0.55, 0.6, 0.7, 0.7));
    }
    if let Some(whatif) = &output.whatif {
        let pts: Vec<Vec3> = whatif
            .trajectory
            .samples
            .iter()
            .map(|s| to_render(s.r))
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
                gizmos.sphere(Isometry3d::from_translation(to_render(s.r)), 0.18, color);
            }
        }
    }
}

#[allow(clippy::type_complexity)]
pub fn update_markers(
    output: Res<SimOutput>,
    playback: Res<Playback>,
    mut whatif_q: Query<(&mut Transform, &mut Visibility), (With<WhatIfMarker>, Without<BaselineMarker>)>,
    mut baseline_q: Query<(&mut Transform, &mut Visibility), (With<BaselineMarker>, Without<WhatIfMarker>)>,
) {
    if let Ok((mut tf, mut vis)) = whatif_q.single_mut() {
        match output
            .whatif
            .as_ref()
            .and_then(|w| w.trajectory.sample_at(playback.t_s))
        {
            Some(s) => {
                tf.translation = to_render(s.r);
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
                tf.translation = to_render(s.r);
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
