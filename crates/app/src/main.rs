mod scene;
mod sim;
mod types;
mod ui;

use bevy::prelude::*;
use bevy_egui::{
    input::ModifierKeysState, EguiInputSet, EguiPlugin, EguiPreUpdateSet, EguiPrimaryContextPass,
};
use types::{FetchChannel, Playback, ScenarioInput, SimOutput};

/// bevy_egui tracks modifier keys from its own view of the event stream; a
/// missed key-release (e.g. keys released mid page-reload, before listeners
/// attach) leaves a modifier flagged "held" forever, and text_input_is_allowed
/// then silently discards every typed character. ButtonInput<KeyCode> is the
/// engine's ground truth and is cleared on focus loss, so trust it for
/// releases: clear any egui modifier whose key isn't actually down.
/// Runs between bevy_egui's modifier tracking and its text-event conversion.
fn unstick_egui_modifiers(
    keys: Res<ButtonInput<KeyCode>>,
    mut mods: ResMut<ModifierKeysState>,
) {
    let down = |a, b| keys.pressed(a) || keys.pressed(b);
    if mods.shift && !down(KeyCode::ShiftLeft, KeyCode::ShiftRight) {
        mods.shift = false;
    }
    if mods.ctrl && !down(KeyCode::ControlLeft, KeyCode::ControlRight) {
        mods.ctrl = false;
    }
    if mods.alt && !down(KeyCode::AltLeft, KeyCode::AltRight) {
        mods.alt = false;
    }
    if mods.win && !down(KeyCode::SuperLeft, KeyCode::SuperRight) {
        mods.win = false;
    }
}

fn main() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "WhatIfOrbit".into(),
                        canvas: Some("#whatiforbit-canvas".into()),
                        fit_canvas_to_parent: true,
                        ..default()
                    }),
                    ..default()
                })
                // Web servers (incl. `trunk serve`) answer missing `.meta`
                // probes with a 200 HTML fallback, which poisons the asset
                // load; we ship no .meta files, so skip the probe entirely.
                .set(AssetPlugin {
                    meta_check: bevy::asset::AssetMetaCheck::Never,
                    ..default()
                }),
        )
        .add_plugins(EguiPlugin::default())
        .init_resource::<ScenarioInput>()
        .init_resource::<SimOutput>()
        .init_resource::<Playback>()
        .init_resource::<FetchChannel>()
        .init_resource::<types::TrackHover>()
        .init_resource::<types::OpmExport>()
        .init_resource::<types::TargetSolve>()
        .add_systems(Startup, scene::setup_scene)
        .add_systems(Startup, {
            #[cfg(target_arch = "wasm32")]
            {
                sim::autoload_from_url
            }
            #[cfg(not(target_arch = "wasm32"))]
            {
                || {}
            }
        })
        .add_systems(
            PreUpdate,
            unstick_egui_modifiers
                .in_set(EguiPreUpdateSet::ProcessInput)
                .after(EguiInputSet::InitReading)
                .before(EguiInputSet::ReadBevyEvents),
        )
        // egui UI must run in bevy_egui's dedicated pass schedule (0.35+).
        .add_systems(
            EguiPrimaryContextPass,
            (ui::ui_system, ui::track_hover_system).chain(),
        )
        .add_systems(
            Update,
            (
                sim::apply_fetch_results,
                sim::drive_refinement,
                sim::recompute,
                sim::advance_playback,
                scene::update_earth_and_sun,
                scene::update_markers,
                scene::draw_trajectories,
                scene::orbit_camera,
            )
                .chain(),
        )
        .run();
}
