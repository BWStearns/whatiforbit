//! Bridges UI inputs to the sim core: TLE loading, CelesTrak fetches,
//! scenario recompute, and playback time advance.

use crate::types::*;
use bevy::prelude::*;
use glam::DVec3;
use whatiforbit_sim::{parse_tle, Cheats, Maneuver, Scenario, Vehicle};

/// On the web, `?demo` in the URL auto-loads the ISS sample so the app can be
/// demoed (and smoke-tested) without any clicking.
#[cfg(target_arch = "wasm32")]
pub fn autoload_from_url(mut input: ResMut<ScenarioInput>) {
    let Some(search) = web_sys::window().and_then(|w| w.location().search().ok()) else {
        return;
    };
    if search.contains("demo") {
        input.tle_text = SAMPLE_TLE.to_string();
        load_tle_into(&mut input);
    }
    // `?demo&burn`: also add a visible prograde impulse so the what-if
    // trajectory diverges from the baseline immediately.
    if search.contains("burn") {
        input.maneuvers.push(ManeuverInput {
            kind: ManeuverKind::Impulsive,
            t_offset_min: 20.0,
            dv_vnc_m_s: [60.0, 0.0, 0.0],
            ..Default::default()
        });
        input.dirty = true;
    }
}

pub fn load_tle_into(input: &mut ScenarioInput) {
    match parse_tle(&input.tle_text) {
        Ok(parsed) => {
            input.loaded = Some(LoadedOrbit {
                name: parsed
                    .name
                    .unwrap_or_else(|| format!("NORAD {}", parsed.norad_id)),
                state: parsed.state,
                object_id: parsed.international_designator,
            });
            input.error = None;
            input.dirty = true;
        }
        Err(e) => {
            input.error = Some(e.0);
        }
    }
}

/// Kick off an async GET of the given NORAD catalog number from CelesTrak.
pub fn start_fetch(input: &mut ScenarioInput, channel: &FetchChannel) {
    let id = input.norad_query.trim().to_string();
    if id.is_empty() || !id.chars().all(|c| c.is_ascii_digit()) {
        input.error = Some("NORAD ID must be a number".into());
        return;
    }
    input.fetch_in_flight = true;
    input.error = None;
    let url = format!("https://celestrak.org/NORAD/elements/gp.php?CATNR={id}&FORMAT=TLE");
    let tx = channel.tx.clone();
    ehttp::fetch(ehttp::Request::get(url), move |result| {
        let msg = match result {
            Ok(resp) if resp.ok => {
                let body = String::from_utf8_lossy(&resp.bytes).to_string();
                if body.contains("No GP data found") || body.trim().is_empty() {
                    Err(format!("CelesTrak has no data for NORAD {id}"))
                } else {
                    Ok(body)
                }
            }
            Ok(resp) => Err(format!("CelesTrak returned HTTP {}", resp.status)),
            Err(e) => Err(format!("fetch failed: {e}")),
        };
        let _ = tx.send(msg);
    });
}

pub fn apply_fetch_results(channel: Res<FetchChannel>, mut input: ResMut<ScenarioInput>) {
    let Ok(rx) = channel.rx.lock() else { return };
    while let Ok(msg) = rx.try_recv() {
        input.fetch_in_flight = false;
        match msg {
            Ok(tle) => {
                input.tle_text = tle.trim().to_string();
                load_tle_into(&mut input);
            }
            Err(e) => input.error = Some(e),
        }
    }
}

fn maneuvers_from_input(rows: &[ManeuverInput]) -> Vec<Maneuver> {
    rows.iter()
        .map(|m| match m.kind {
            ManeuverKind::Impulsive => Maneuver::Impulsive {
                t_offset_s: m.t_offset_min * 60.0,
                dv_vnc_km_s: DVec3::new(
                    m.dv_vnc_m_s[0] / 1000.0,
                    m.dv_vnc_m_s[1] / 1000.0,
                    m.dv_vnc_m_s[2] / 1000.0,
                ),
            },
            ManeuverKind::FiniteBurn => Maneuver::FiniteBurn {
                t_offset_s: m.t_offset_min * 60.0,
                duration_s: m.duration_s,
                direction_vnc: DVec3::new(
                    m.direction_vnc[0],
                    m.direction_vnc[1],
                    m.direction_vnc[2],
                ),
                throttle: m.throttle,
            },
        })
        .collect()
}

pub fn recompute(
    mut input: ResMut<ScenarioInput>,
    mut output: ResMut<SimOutput>,
    mut playback: ResMut<Playback>,
) {
    if !input.dirty {
        return;
    }
    input.dirty = false;

    let Some(loaded) = input.loaded.clone() else {
        output.baseline = None;
        output.whatif = None;
        return;
    };

    // Sanitize vehicle numbers so the sim never sees nonsense.
    let wet = input.wet_mass_kg.max(1.0);
    let dry = input.dry_mass_kg.clamp(0.1, wet);
    input.wet_mass_kg = wet;
    input.dry_mass_kg = dry;
    let vehicle = Vehicle {
        wet_mass_kg: wet,
        dry_mass_kg: dry,
        thrust_n: input.thrust_n.max(0.0),
        isp_s: input.isp_s.max(1.0),
    };

    let t_start = -(input.backward_days.max(0.0)) * 86_400.0;
    let t_end = input.forward_days.clamp(0.001, 60.0) * 86_400.0;
    let output_dt = ((t_end - t_start) / 2500.0).clamp(1.0, 120.0);

    let scenario = Scenario {
        init: loaded.state,
        vehicle,
        maneuvers: maneuvers_from_input(&input.maneuvers),
        cheats: Cheats {
            infinite_fuel: input.infinite_fuel,
        },
        t_start_s: t_start,
        t_end_s: t_end,
        output_dt_s: output_dt,
        enable_j2: input.enable_j2,
    };

    let whatif = scenario.propagate();
    let baseline = {
        let mut b = scenario.clone();
        b.maneuvers.clear();
        b.propagate()
    };

    output.baseline = Some(baseline);
    output.whatif = Some(whatif);
    output.scenario = Some(scenario);
    output.t_start_s = t_start;
    output.t_end_s = t_end;
    playback.t_s = playback.t_s.clamp(t_start, t_end);
}

pub fn advance_playback(
    time: Res<Time>,
    output: Res<SimOutput>,
    mut playback: ResMut<Playback>,
) {
    if !playback.playing || output.whatif.is_none() {
        return;
    }
    playback.t_s += time.delta_secs() as f64 * playback.rate;
    if playback.t_s >= output.t_end_s {
        playback.t_s = output.t_end_s;
        playback.playing = false;
    }
}
