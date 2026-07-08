//! egui control panels: spacecraft source, vehicle, maneuvers, cheats,
//! readout, and the timeline scrubber.

use crate::sim::{load_tle_into, start_fetch};
use crate::types::*;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use whatiforbit_sim::{elements_from_rv, R_EARTH};

pub fn fmt_hms(t_s: f64) -> String {
    let sign = if t_s < 0.0 { "-" } else { "+" };
    let t = t_s.abs();
    let h = (t / 3600.0).floor();
    let m = ((t - h * 3600.0) / 60.0).floor();
    let s = t - h * 3600.0 - m * 60.0;
    format!("{sign}{h:02.0}:{m:02.0}:{s:02.0}")
}

pub fn ui_system(
    mut contexts: EguiContexts,
    mut input: ResMut<ScenarioInput>,
    mut playback: ResMut<Playback>,
    output: Res<SimOutput>,
    channel: Res<FetchChannel>,
    mut opm: ResMut<OpmExport>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    let mut changed = false;

    egui::SidePanel::left("controls")
        .default_width(330.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("WhatIfOrbit");
                ui.label(
                    egui::RichText::new("Fly what-if maneuvers on a real orbit")
                        .small()
                        .weak(),
                );
                ui.separator();

                spacecraft_section(ui, &mut input, &channel);
                ui.separator();
                changed |= vehicle_section(ui, &mut input);
                ui.separator();
                changed |= propagation_section(ui, &mut input);
                ui.separator();
                changed |= maneuvers_section(ui, &mut input, &output);
                ui.separator();
                export_section(ui, &input, &output, &mut opm);
                ui.separator();
                readout_section(ui, &input, &output, &playback);
            });
        });

    egui::TopBottomPanel::bottom("timeline").show(ctx, |ui| {
        timeline_bar(ui, &mut playback, &output);
    });

    // OPM viewer window (open while there is generated text).
    let mut open = opm.text.is_some();
    if let Some(text) = opm.text.clone() {
        egui::Window::new("OPM export")
            .open(&mut open)
            .default_width(560.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if ui.button("Copy to clipboard").clicked() {
                        ctx.copy_text(text.clone());
                        opm.status = Some("Copied.".into());
                    }
                    if ui.button("Download .opm").clicked() {
                        opm.status = Some(download_opm(&text));
                    }
                    if let Some(status) = &opm.status {
                        ui.label(egui::RichText::new(status).small().weak());
                    }
                });
                egui::ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(&mut text.as_str())
                            .font(egui::TextStyle::Monospace)
                            .desired_width(f32::INFINITY),
                    );
                });
            });
    }
    if !open {
        opm.text = None;
        opm.status = None;
    }

    if changed {
        input.dirty = true;
    }
}

fn export_section(
    ui: &mut egui::Ui,
    input: &ScenarioInput,
    output: &SimOutput,
    opm: &mut OpmExport,
) {
    ui.strong("Export");
    let ready = output.scenario.is_some() && output.whatif.is_some() && input.loaded.is_some();
    let btn = ui.add_enabled(ready, egui::Button::new("Generate OPM (CCSDS)"));
    if btn.clicked() {
        let (Some(scenario), Some(whatif), Some(loaded)) =
            (&output.scenario, &output.whatif, &input.loaded)
        else {
            return;
        };
        let object_id = loaded.object_id.as_deref().unwrap_or("UNKNOWN");
        opm.text = Some(whatiforbit_sim::opm_kvn(
            scenario,
            whatif,
            &loaded.name,
            object_id,
            now_utc(),
        ));
        opm.status = None;
    }
    if !ready {
        ui.label(egui::RichText::new("Load a spacecraft first.").small().weak());
    }
}

/// Picks the trajectory sample nearest the mouse (within a screen-space
/// threshold) and shows a tooltip with relative/absolute time and state.
/// Runs in the egui pass, after the panels, so it can defer to them.
pub fn track_hover_system(
    mut contexts: EguiContexts,
    windows: Query<&Window, With<bevy::window::PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    output: Res<SimOutput>,
    input: Res<ScenarioInput>,
    mut hover: ResMut<TrackHover>,
) {
    const PICK_RADIUS_PX: f32 = 14.0;
    hover.0 = None;

    let Ok(ctx) = contexts.ctx_mut() else { return };
    if ctx.is_pointer_over_area() {
        return;
    }
    let Ok(window) = windows.single() else { return };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    let Ok((camera, cam_tf)) = cameras.single() else {
        return;
    };

    let mut best: Option<(f32, HoverInfo)> = None;
    let tracks = [
        ("What-if", output.whatif.as_ref()),
        ("Original", output.baseline.as_ref()),
    ];
    for (label, result) in tracks {
        let Some(result) = result else { continue };
        for s in &result.trajectory.samples {
            let Ok(pos) = camera.world_to_viewport(cam_tf, crate::scene::to_render(s.r)) else {
                continue;
            };
            let d = pos.distance(cursor);
            if d < PICK_RADIUS_PX && best.as_ref().is_none_or(|(bd, _)| d < *bd) {
                best = Some((
                    d,
                    HoverInfo {
                        track: label,
                        t_s: s.t_s,
                        r_km: s.r,
                        speed_km_s: s.v.length(),
                        mass_kg: s.mass,
                    },
                ));
            }
        }
    }

    let Some((_, info)) = best else { return };
    hover.0 = Some(info);

    egui::show_tooltip_at_pointer(
        ctx,
        egui::LayerId::background(),
        egui::Id::new("track_hover"),
        |ui| {
            ui.strong(info.track);
            ui.label(format!("T{}", fmt_hms(info.t_s)));
            if let Some(epoch) = output
                .whatif
                .as_ref()
                .and_then(|w| w.trajectory.epoch_at(info.t_s))
            {
                ui.label(format!("{epoch}"));
            }
            ui.label(format!(
                "Alt {:.1} km   |v| {:.4} km/s",
                info.r_km.length() - whatiforbit_sim::R_EARTH,
                info.speed_km_s
            ));
            if !input.infinite_fuel && info.track == "What-if" {
                ui.label(format!("Mass {:.1} kg", info.mass_kg));
            }
        },
    );
}

/// Current UTC time as a hifitime Epoch, on both native and wasm.
fn now_utc() -> hifitime::Epoch {
    #[cfg(target_arch = "wasm32")]
    {
        hifitime::Epoch::from_unix_seconds(js_sys::Date::now() / 1000.0)
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        hifitime::Epoch::from_unix_seconds(secs)
    }
}

/// Save the OPM: native writes a file next to the executable's cwd; wasm
/// triggers a browser download via a Blob object URL.
fn download_opm(text: &str) -> String {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let path = "whatiforbit.opm";
        match std::fs::write(path, text) {
            Ok(()) => format!("Saved to ./{path}"),
            Err(e) => format!("Save failed: {e}"),
        }
    }
    #[cfg(target_arch = "wasm32")]
    {
        use wasm_bindgen::JsCast;
        let result = (|| -> Result<(), wasm_bindgen::JsValue> {
            let array = js_sys::Array::of1(&wasm_bindgen::JsValue::from_str(text));
            let blob = web_sys::Blob::new_with_str_sequence(&array)?;
            let url = web_sys::Url::create_object_url_with_blob(&blob)?;
            let document = web_sys::window()
                .ok_or("no window")?
                .document()
                .ok_or("no document")?;
            let a: web_sys::HtmlAnchorElement =
                document.create_element("a")?.dyn_into()?;
            a.set_href(&url);
            a.set_download("whatiforbit.opm");
            a.click();
            web_sys::Url::revoke_object_url(&url)?;
            Ok(())
        })();
        match result {
            Ok(()) => "Download started.".into(),
            Err(e) => format!("Download failed: {e:?}"),
        }
    }
}

fn spacecraft_section(ui: &mut egui::Ui, input: &mut ScenarioInput, channel: &FetchChannel) {
    ui.strong("Spacecraft");
    ui.add(
        egui::TextEdit::multiline(&mut input.tle_text)
            .hint_text("Paste a TLE here (2 or 3 lines)")
            .font(egui::TextStyle::Monospace)
            .desired_rows(3)
            .desired_width(f32::INFINITY),
    );
    ui.horizontal(|ui| {
        if ui.button("Load TLE").clicked() {
            load_tle_into(input);
        }
        if ui.button("ISS sample").clicked() {
            input.tle_text = SAMPLE_TLE.to_string();
            load_tle_into(input);
        }
    });
    ui.horizontal(|ui| {
        ui.label("NORAD ID:");
        ui.add(egui::TextEdit::singleline(&mut input.norad_query).desired_width(70.0));
        let btn = ui.add_enabled(
            !input.fetch_in_flight,
            egui::Button::new(if input.fetch_in_flight {
                "Fetching…"
            } else {
                "Fetch from CelesTrak"
            }),
        );
        if btn.clicked() {
            start_fetch(input, channel);
        }
    });
    if let Some(err) = &input.error {
        ui.colored_label(egui::Color32::from_rgb(255, 100, 90), err);
    }
    if let Some(loaded) = &input.loaded {
        ui.colored_label(
            egui::Color32::from_rgb(120, 220, 130),
            format!("Loaded: {}", loaded.name),
        );
        ui.label(
            egui::RichText::new(format!("Epoch: {}", loaded.state.epoch))
                .small()
                .weak(),
        );
    } else {
        ui.label(egui::RichText::new("No spacecraft loaded yet.").weak());
    }
}

fn drag(ui: &mut egui::Ui, label: &str, v: &mut f64, speed: f64, suffix: &str) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        changed = ui
            .add(egui::DragValue::new(v).speed(speed).suffix(suffix))
            .changed();
    });
    changed
}

fn vehicle_section(ui: &mut egui::Ui, input: &mut ScenarioInput) -> bool {
    ui.strong("Vehicle");
    let mut c = false;
    c |= drag(ui, "Wet mass", &mut input.wet_mass_kg, 10.0, " kg");
    c |= drag(ui, "Dry mass", &mut input.dry_mass_kg, 10.0, " kg");
    c |= drag(ui, "Thrust", &mut input.thrust_n, 5.0, " N");
    c |= drag(ui, "Isp", &mut input.isp_s, 1.0, " s");
    ui.add_space(4.0);
    ui.strong("Cheats & physics");
    c |= ui
        .checkbox(&mut input.infinite_fuel, "Infinite fuel (ignore propellant)")
        .changed();
    c |= ui
        .checkbox(&mut input.enable_j2, "J2 perturbation (off = pure Kepler)")
        .changed();
    c
}

fn propagation_section(ui: &mut egui::Ui, input: &mut ScenarioInput) -> bool {
    ui.strong("Propagation span");
    let mut c = false;
    c |= drag(ui, "Forward", &mut input.forward_days, 0.1, " days");
    c |= drag(ui, "Backward", &mut input.backward_days, 0.1, " days");
    input.forward_days = input.forward_days.clamp(0.01, 60.0);
    input.backward_days = input.backward_days.clamp(0.0, 60.0);
    c
}

fn maneuvers_section(
    ui: &mut egui::Ui,
    input: &mut ScenarioInput,
    output: &SimOutput,
) -> bool {
    ui.strong("Maneuvers");
    let mut c = false;
    let mut remove: Option<usize> = None;

    for (i, m) in input.maneuvers.iter_mut().enumerate() {
        let title = match m.kind {
            ManeuverKind::Impulsive => format!("#{} impulse @ {:+.1} min", i + 1, m.t_offset_min),
            ManeuverKind::FiniteBurn => format!("#{} burn @ {:+.1} min", i + 1, m.t_offset_min),
        };
        egui::CollapsingHeader::new(title)
            .id_salt(("maneuver", i))
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Type:");
                    c |= ui
                        .selectable_value(&mut m.kind, ManeuverKind::Impulsive, "Impulse")
                        .changed();
                    c |= ui
                        .selectable_value(&mut m.kind, ManeuverKind::FiniteBurn, "Finite burn")
                        .changed();
                });
                c |= drag(ui, "Time from epoch", &mut m.t_offset_min, 1.0, " min");
                match m.kind {
                    ManeuverKind::Impulsive => {
                        ui.label("Δv (VNC), m/s:");
                        c |= drag(ui, "  along-track (V)", &mut m.dv_vnc_m_s[0], 1.0, " m/s");
                        c |= drag(ui, "  normal (N)", &mut m.dv_vnc_m_s[1], 1.0, " m/s");
                        c |= drag(ui, "  radial-ish (C)", &mut m.dv_vnc_m_s[2], 1.0, " m/s");
                    }
                    ManeuverKind::FiniteBurn => {
                        c |= drag(ui, "Duration", &mut m.duration_s, 5.0, " s");
                        c |= drag(ui, "Throttle (0–1)", &mut m.throttle, 0.01, "");
                        m.throttle = m.throttle.clamp(0.0, 1.0);
                        ui.label("Direction (VNC components):");
                        c |= drag(ui, "  V", &mut m.direction_vnc[0], 0.05, "");
                        c |= drag(ui, "  N", &mut m.direction_vnc[1], 0.05, "");
                        c |= drag(ui, "  C", &mut m.direction_vnc[2], 0.05, "");
                    }
                }
                // Execution report from the last propagation.
                if let Some(whatif) = &output.whatif {
                    if let Some(rep) = whatif.reports.iter().find(|r| r.index == i) {
                        let unit = match m.kind {
                            ManeuverKind::Impulsive => "km/s",
                            ManeuverKind::FiniteBurn => "s",
                        };
                        let text = format!(
                            "achieved {:.4}/{:.4} {unit}, {:.1} kg propellant",
                            rep.achieved, rep.requested, rep.prop_used_kg
                        );
                        if rep.feasible {
                            ui.label(egui::RichText::new(text).small().weak());
                        } else {
                            ui.colored_label(
                                egui::Color32::from_rgb(255, 160, 60),
                                format!("⚠ propellant-limited: {text}"),
                            );
                        }
                    }
                }
                if ui.button("Remove").clicked() {
                    remove = Some(i);
                }
            });
    }

    if let Some(i) = remove {
        input.maneuvers.remove(i);
        c = true;
    }
    ui.horizontal(|ui| {
        if ui.button("+ Impulse").clicked() {
            input.maneuvers.push(ManeuverInput::default());
            c = true;
        }
        if ui.button("+ Finite burn").clicked() {
            input.maneuvers.push(ManeuverInput {
                kind: ManeuverKind::FiniteBurn,
                ..Default::default()
            });
            c = true;
        }
    });
    c
}

fn readout_section(
    ui: &mut egui::Ui,
    input: &ScenarioInput,
    output: &SimOutput,
    playback: &Playback,
) {
    ui.strong("State readout (what-if)");
    let Some(whatif) = &output.whatif else {
        ui.label(egui::RichText::new("Load a spacecraft to see state.").weak());
        return;
    };
    let Some(s) = whatif.trajectory.sample_at(playback.t_s) else {
        return;
    };
    if let Some(epoch) = whatif.trajectory.epoch_at(s.t_s) {
        ui.label(format!("UTC: {epoch}"));
    }
    let el = elements_from_rv(s.r, s.v);
    let alt = s.r.length() - R_EARTH;
    egui::Grid::new("elements").num_columns(2).show(ui, |ui| {
        ui.label("Altitude");
        ui.label(format!("{alt:.1} km"));
        ui.end_row();
        ui.label("Speed");
        ui.label(format!("{:.4} km/s", s.v.length()));
        ui.end_row();
        ui.label("SMA");
        ui.label(format!("{:.1} km", el.sma_km));
        ui.end_row();
        ui.label("Ecc");
        ui.label(format!("{:.5}", el.ecc));
        ui.end_row();
        ui.label("Inc");
        ui.label(format!("{:.3}°", el.inc_deg));
        ui.end_row();
        ui.label("RAAN");
        ui.label(format!("{:.3}°", el.raan_deg));
        ui.end_row();
        ui.label("Arg periapsis");
        ui.label(format!("{:.3}°", el.argp_deg));
        ui.end_row();
        ui.label("True anomaly");
        ui.label(format!("{:.3}°", el.true_anomaly_deg));
        ui.end_row();
        if let Some(p) = el.period_s {
            ui.label("Period");
            ui.label(format!("{:.1} min", p / 60.0));
            ui.end_row();
        }
        if let Some(apo) = el.apoapsis_alt_km {
            ui.label("Apoapsis alt");
            ui.label(format!("{apo:.1} km"));
            ui.end_row();
        }
        ui.label("Periapsis alt");
        ui.label(format!("{:.1} km", el.periapsis_alt_km));
        ui.end_row();
        ui.label("Mass");
        ui.label(format!("{:.1} kg", s.mass));
        ui.end_row();
        if !input.infinite_fuel {
            ui.label("Propellant left");
            ui.label(format!(
                "{:.1} kg",
                (s.mass - input.dry_mass_kg).max(0.0)
            ));
            ui.end_row();
        }
        ui.label("Total Δv used");
        ui.label(format!("{:.1} m/s", whatif.total_dv_km_s * 1000.0));
        ui.end_row();
    });
    if el.periapsis_alt_km < 100.0 {
        ui.colored_label(
            egui::Color32::from_rgb(255, 100, 90),
            "⚠ Periapsis below ~100 km: reentry!",
        );
    }
}

fn timeline_bar(ui: &mut egui::Ui, playback: &mut Playback, output: &SimOutput) {
    ui.horizontal(|ui| {
        let enabled = output.whatif.is_some();
        ui.add_enabled_ui(enabled, |ui| {
            let label = if playback.playing { "⏸" } else { "▶" };
            if ui.button(label).clicked() {
                playback.playing = !playback.playing;
                if playback.playing && playback.t_s >= output.t_end_s {
                    playback.t_s = output.t_start_s;
                }
            }
            egui::ComboBox::from_id_salt("rate")
                .selected_text(format!("{}×", playback.rate))
                .width(70.0)
                .show_ui(ui, |ui| {
                    for r in [1.0, 10.0, 60.0, 600.0, 3600.0] {
                        ui.selectable_value(&mut playback.rate, r, format!("{r}×"));
                    }
                });
            let (lo, hi) = (output.t_start_s, output.t_end_s.max(output.t_start_s + 1.0));
            ui.spacing_mut().slider_width = ui.available_width() - 110.0;
            ui.add(
                egui::Slider::new(&mut playback.t_s, lo..=hi)
                    .show_value(false)
                    .custom_formatter(|v, _| fmt_hms(v)),
            );
            ui.monospace(fmt_hms(playback.t_s));
        });
    });
}
