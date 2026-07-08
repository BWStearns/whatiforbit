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

    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            ..default()
        },
        Transform::from_xyz(50.0, 20.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        ..default()
    });

    // Earth. NASA Blue Marble (land_shallow_topo_2048, public domain),
    // equirectangular on a UV sphere. The globe does not model Earth's
    // rotation — the texture provides visual context only; longitude
    // alignment to the inertial frame is not meaningful here.
    let earth_radius = (R_EARTH / KM_PER_UNIT) as f32;
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(earth_radius).mesh().uv(64, 32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(asset_server.load("earth.jpg")),
            perceptual_roughness: 0.9,
            ..default()
        })),
        // Bevy's UV sphere has its texture poles on +-Z; rotate so the
        // texture's north pole sits on +Y, the scene's north axis.
        Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
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

/// Immediate-mode rendering of both trajectories, pole axis, maneuver node
/// markers, and the hovered-point highlight.
pub fn draw_trajectories(
    mut gizmos: Gizmos,
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    hover: Res<TrackHover>,
) {
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

        // Maneuver nodes.
        for m in &input.maneuvers {
            if let Some(s) = whatif.trajectory.sample_at(m.t_offset_min * 60.0) {
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
