//! Shareable links: the whole user-visible scenario encoded as URL query
//! parameters, so pasting a link reproduces what the sender was looking at.
//!
//! Named `key=value` pairs rather than one opaque blob: a link stays readable
//! and hand-editable, unknown keys from a newer build are ignored instead of
//! poisoning the parse, and missing keys fall back to the app's own defaults,
//! so old links keep working as fields are added.
//!
//! The spacecraft travels as its TLE text, not as a NORAD id. Re-fetching by
//! id would hand the recipient whatever CelesTrak serves *today*, which is a
//! different orbit from the one the sender was looking at; the TLE pins it.
//!
//! Encoding is pure and lives here so it can be round-trip tested natively;
//! only the address-bar plumbing in `sync_share_link` is web-specific.

use crate::scene::OrbitCamera;
use crate::types::*;
use bevy::prelude::*;

/// Payload version. Written as `v`, and required on read — its absence is how
/// we tell a share link from the legacy `?demo` flags.
pub const VERSION: u32 = 1;

/// The link for the current app state, rebuilt a few times a second.
#[derive(Resource, Default)]
pub struct ShareLink {
    /// Query string without the leading `?`.
    pub query: String,
    /// Full URL to hand to someone (just the query on native, which has no
    /// address bar to read an origin from).
    pub url: String,
    /// Feedback for the copy button; set by the UI, never by the sync system.
    pub status: Option<String>,
    seconds_until_rebuild: f32,
}

// ---------------------------------------------------------------- encoding

/// Percent-encode for a query value, form-style: space becomes `+`. TLEs are
/// nearly a quarter spaces, so `+` over `%20` is worth the small asymmetry
/// with `decode`.
fn encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + s.len() / 4);
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => {
                out.push(b as char)
            }
            b' ' => out.push('+'),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Inverse of [`encode`]. Operates on bytes throughout: a malformed escape in
/// a hand-edited link must not slice a `str` off a char boundary.
fn decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b'%' if i + 2 < b.len() => match (hex_digit(b[i + 1]), hex_digit(b[i + 2])) {
                (Some(hi), Some(lo)) => {
                    out.push(hi * 16 + lo);
                    i += 3;
                }
                _ => {
                    out.push(b'%');
                    i += 1;
                }
            },
            c => {
                out.push(c);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// Parsed query string. Values are decoded once, up front.
struct Params(Vec<(String, String)>);

impl Params {
    fn parse(query: &str) -> Self {
        let query = query.trim_start_matches('?');
        Params(
            query
                .split('&')
                .filter(|kv| !kv.is_empty())
                .map(|kv| match kv.split_once('=') {
                    Some((k, v)) => (k.to_string(), decode(v)),
                    None => (kv.to_string(), String::new()),
                })
                .collect(),
        )
    }

    fn has(&self, key: &str) -> bool {
        self.0.iter().any(|(k, _)| k == key)
    }

    fn get(&self, key: &str) -> Option<&str> {
        self.0
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    fn all<'a>(&'a self, key: &'a str) -> impl Iterator<Item = &'a str> {
        self.0
            .iter()
            .filter(move |(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }

    /// Finite floats only — a link carrying `nan` or `inf` must not reach the
    /// integrator, where it would silently poison every downstream sample.
    fn num(&self, key: &str, fallback: f64) -> f64 {
        self.get(key)
            .and_then(|v| v.parse::<f64>().ok())
            .filter(|f| f.is_finite())
            .unwrap_or(fallback)
    }

    fn flag(&self, key: &str, fallback: bool) -> bool {
        match self.get(key) {
            Some("1") => true,
            Some("0") => false,
            _ => fallback,
        }
    }
}

/// Comma-separated finite floats, or None if the field count is wrong or any
/// value fails to parse — partial application of a malformed row is worse
/// than ignoring it.
fn nums<const N: usize>(s: &str) -> Option<[f64; N]> {
    let mut out = [0.0; N];
    let mut parts = s.split(',');
    for slot in out.iter_mut() {
        let v = parts.next()?.parse::<f64>().ok()?;
        if !v.is_finite() {
            return None;
        }
        *slot = v;
    }
    parts.next().is_none().then_some(out)
}

/// Rust's float `Display` is shortest-round-trip, so this is exact and still
/// prints `1000` rather than `1000.0000000000`.
fn num(v: f64) -> String {
    format!("{v}")
}

// ------------------------------------------------------------------ write

/// Encode the full app state as a query string (no leading `?`).
pub fn to_query(
    input: &ScenarioInput,
    playback: &Playback,
    frame: ViewFrame,
    cam: &OrbitCamera,
    solve: &TargetSolve,
) -> String {
    let mut p: Vec<String> = vec![format!("v={VERSION}")];

    if !input.tle_text.trim().is_empty() {
        p.push(format!("tle={}", encode(input.tle_text.trim())));
    }

    p.push(format!("wet={}", num(input.wet_mass_kg)));
    p.push(format!("dry={}", num(input.dry_mass_kg)));
    p.push(format!("thr={}", num(input.thrust_n)));
    p.push(format!("isp={}", num(input.isp_s)));
    p.push(format!("fwd={}", num(input.forward_days)));
    p.push(format!("bwd={}", num(input.backward_days)));
    p.push(format!("j2={}", u8::from(input.enable_j2)));
    p.push(format!("fuel={}", u8::from(input.infinite_fuel)));

    // Every maneuver field goes out, not just the ones its kind uses: the UI
    // keeps both sets alive so switching kind back and forth is lossless, and
    // a shared link should behave the same way.
    for m in &input.maneuvers {
        let kind = match m.kind {
            ManeuverKind::Impulsive => "i",
            ManeuverKind::FiniteBurn => "b",
        };
        p.push(format!(
            "mv={kind},{},{},{},{},{},{},{},{},{}",
            num(m.t_offset_min),
            num(m.dv_vnc_m_s[0]),
            num(m.dv_vnc_m_s[1]),
            num(m.dv_vnc_m_s[2]),
            num(m.duration_s),
            num(m.throttle),
            num(m.direction_vnc[0]),
            num(m.direction_vnc[1]),
            num(m.direction_vnc[2]),
        ));
    }

    p.push(format!(
        "frame={}",
        match frame {
            ViewFrame::Inertial => "eci",
            ViewFrame::EarthFixed => "ecef",
        }
    ));

    p.push(format!("t={}", num(playback.t_s)));
    p.push(format!("rate={}", num(playback.rate)));
    p.push(format!("play={}", u8::from(playback.playing)));
    p.push(format!(
        "cam={},{},{}",
        num(cam.yaw as f64),
        num(cam.pitch as f64),
        num(cam.dist as f64),
    ));

    // Solver *inputs* only. The candidate table is derived, costs real time to
    // produce, and would balloon the link; the recipient presses Solve.
    p.push(format!(
        "tgt={},{},{},{},{},{}",
        num(solve.apo_alt_km),
        num(solve.peri_alt_km),
        num(solve.inc_deg),
        match solve.mode {
            TargetModeUi::Cheapest => "c",
            TargetModeUi::Fastest => "f",
        },
        u8::from(solve.deadline_enabled),
        num(solve.deadline_hours),
    ));

    p.join("&")
}

// ------------------------------------------------------------------- read

/// Apply a share link's settings. Returns false (touching nothing) when the
/// query carries no share payload, which is how `?demo` still reaches its own
/// handler.
///
/// The TLE lands in `tle_text` but is not parsed here; the caller decides how
/// to load it. Every field is best-effort: anything missing or unparseable
/// keeps the value it already had.
pub fn apply_query(
    query: &str,
    input: &mut ScenarioInput,
    playback: &mut Playback,
    frame: &mut ViewFrame,
    cam: &mut OrbitCamera,
    solve: &mut TargetSolve,
) -> bool {
    let p = Params::parse(query);
    if !p.has("v") {
        return false;
    }

    if let Some(tle) = p.get("tle") {
        input.tle_text = tle.to_string();
    }

    input.wet_mass_kg = p.num("wet", input.wet_mass_kg);
    input.dry_mass_kg = p.num("dry", input.dry_mass_kg);
    input.thrust_n = p.num("thr", input.thrust_n);
    input.isp_s = p.num("isp", input.isp_s);
    input.forward_days = p.num("fwd", input.forward_days);
    input.backward_days = p.num("bwd", input.backward_days);
    input.enable_j2 = p.flag("j2", input.enable_j2);
    input.infinite_fuel = p.flag("fuel", input.infinite_fuel);

    // Only replace the maneuver list if the link actually carries one, so a
    // hand-trimmed link doesn't silently mean "no maneuvers".
    if p.has("mv") {
        input.maneuvers = p
            .all("mv")
            .filter_map(|row| {
                let (kind, rest) = row.split_once(',')?;
                let kind = match kind {
                    "i" => ManeuverKind::Impulsive,
                    "b" => ManeuverKind::FiniteBurn,
                    _ => return None,
                };
                let f = nums::<9>(rest)?;
                Some(ManeuverInput {
                    kind,
                    t_offset_min: f[0],
                    dv_vnc_m_s: [f[1], f[2], f[3]],
                    duration_s: f[4],
                    throttle: f[5],
                    direction_vnc: [f[6], f[7], f[8]],
                })
            })
            .collect();
    }

    match p.get("frame") {
        Some("ecef") => *frame = ViewFrame::EarthFixed,
        Some("eci") => *frame = ViewFrame::Inertial,
        _ => {}
    }

    playback.t_s = p.num("t", playback.t_s);
    playback.rate = p.num("rate", playback.rate);
    playback.playing = p.flag("play", playback.playing);

    if let Some(c) = p.get("cam").and_then(nums::<3>) {
        cam.yaw = c[0] as f32;
        // Mirror the live camera's own limits: a link must not be able to put
        // the camera somewhere the mouse cannot.
        cam.pitch = (c[1] as f32).clamp(-1.54, 1.54);
        cam.dist = (c[2] as f32).clamp(7.5, 400.0);
    }

    let tgt = p.get("tgt").map(|v| v.split(',').collect::<Vec<_>>());
    if let Some([apo, peri, inc, mode, deadline, hours]) = tgt.as_deref() {
        if let Some(o) = nums::<3>(&format!("{apo},{peri},{inc}")) {
            solve.apo_alt_km = o[0];
            solve.peri_alt_km = o[1];
            solve.inc_deg = o[2];
        }
        solve.mode = match *mode {
            "f" => TargetModeUi::Fastest,
            _ => TargetModeUi::Cheapest,
        };
        solve.deadline_enabled = *deadline == "1";
        if let Some(h) = hours.parse::<f64>().ok().filter(|h| h.is_finite()) {
            solve.deadline_hours = h;
        }
    }

    input.dirty = true;
    true
}

// --------------------------------------------------------------- plumbing

/// Rebuild [`ShareLink`] a few times a second, and on the web keep the address
/// bar in step so the link is always there to copy.
///
/// Two deliberate limits. The rebuild is throttled rather than driven by
/// change detection because the camera is a component the orbit controller
/// rewrites every frame. And the address bar is left alone while playback is
/// running: `t` would change continuously, and browsers rate-limit
/// `replaceState` (Safari at roughly 100 calls per 30 s). The link itself
/// stays live throughout, so the Copy button always yields the current view,
/// and the address bar catches up the moment playback stops.
pub fn sync_share_link(
    time: Res<Time>,
    input: Res<ScenarioInput>,
    playback: Res<Playback>,
    frame: Res<ViewFrame>,
    solve: Res<TargetSolve>,
    cam: Query<&OrbitCamera>,
    mut link: ResMut<ShareLink>,
) {
    link.seconds_until_rebuild -= time.delta_secs();
    if link.seconds_until_rebuild > 0.0 {
        return;
    }
    link.seconds_until_rebuild = 0.25;

    let Ok(cam) = cam.single() else { return };
    let query = to_query(&input, &playback, *frame, cam, &solve);
    if query == link.query {
        return;
    }
    link.query = query;
    link.url = share_url(&link.query);

    if !playback.playing {
        write_address_bar(&link.url);
    }
}

/// The full link, given a query string.
fn share_url(query: &str) -> String {
    #[cfg(target_arch = "wasm32")]
    {
        if let Some(loc) = web_sys::window().map(|w| w.location()) {
            if let (Ok(origin), Ok(path)) = (loc.origin(), loc.pathname()) {
                return format!("{origin}{path}?{query}");
            }
        }
    }
    format!("?{query}")
}

#[cfg(target_arch = "wasm32")]
fn write_address_bar(url: &str) {
    // replaceState, not pushState: a shareable address bar should not turn
    // every nudge of the camera into a Back-button step.
    if let Some(history) = web_sys::window().and_then(|w| w.history().ok()) {
        let _ = history.replace_state_with_url(&wasm_bindgen::JsValue::NULL, "", Some(url));
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn write_address_bar(_url: &str) {}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_state() -> (ScenarioInput, Playback, ViewFrame, OrbitCamera, TargetSolve) {
        let input = ScenarioInput {
            tle_text: SAMPLE_TLE.to_string(),
            wet_mass_kg: 1234.5,
            dry_mass_kg: 610.25,
            thrust_n: 425.0,
            isp_s: 311.0,
            forward_days: 2.5,
            backward_days: 0.75,
            enable_j2: false,
            infinite_fuel: true,
            maneuvers: vec![
                ManeuverInput {
                    kind: ManeuverKind::Impulsive,
                    t_offset_min: 12.5,
                    dv_vnc_m_s: [60.0, -3.25, 0.5],
                    duration_s: 90.0,
                    throttle: 0.5,
                    direction_vnc: [0.0, 1.0, 0.0],
                },
                ManeuverInput {
                    kind: ManeuverKind::FiniteBurn,
                    t_offset_min: 44.0,
                    dv_vnc_m_s: [1.0, 2.0, 3.0],
                    duration_s: 180.5,
                    throttle: 0.8,
                    direction_vnc: [-1.0, 0.0, 0.25],
                },
            ],
            ..Default::default()
        };
        let playback = Playback {
            t_s: 3_600.5,
            playing: true,
            rate: 120.0,
        };
        let cam = OrbitCamera {
            yaw: 1.25,
            pitch: -0.5,
            dist: 42.5,
        };
        let solve = TargetSolve {
            apo_alt_km: 1200.0,
            peri_alt_km: 780.5,
            inc_deg: 28.5,
            mode: TargetModeUi::Fastest,
            deadline_enabled: true,
            deadline_hours: 6.25,
            ..Default::default()
        };
        (input, playback, ViewFrame::EarthFixed, cam, solve)
    }

    /// Every user-visible setting survives the trip through a URL exactly —
    /// the whole point of the feature, and the test that catches a field
    /// added to the app but not to the encoder.
    #[test]
    fn round_trips_every_setting() {
        let (input, playback, frame, cam, solve) = sample_state();
        let query = to_query(&input, &playback, frame, &cam, &solve);

        let (mut got_input, mut got_playback, mut got_frame, mut got_cam, mut got_solve) = (
            ScenarioInput::default(),
            Playback::default(),
            ViewFrame::default(),
            OrbitCamera {
                yaw: 0.0,
                pitch: 0.0,
                dist: 10.0,
            },
            TargetSolve::default(),
        );
        assert!(apply_query(
            &query,
            &mut got_input,
            &mut got_playback,
            &mut got_frame,
            &mut got_cam,
            &mut got_solve,
        ));

        assert_eq!(got_input.tle_text, input.tle_text);
        assert_eq!(got_input.wet_mass_kg, input.wet_mass_kg);
        assert_eq!(got_input.dry_mass_kg, input.dry_mass_kg);
        assert_eq!(got_input.thrust_n, input.thrust_n);
        assert_eq!(got_input.isp_s, input.isp_s);
        assert_eq!(got_input.forward_days, input.forward_days);
        assert_eq!(got_input.backward_days, input.backward_days);
        assert_eq!(got_input.enable_j2, input.enable_j2);
        assert_eq!(got_input.infinite_fuel, input.infinite_fuel);
        assert_eq!(got_input.maneuvers, input.maneuvers);

        assert_eq!(got_playback.t_s, playback.t_s);
        assert_eq!(got_playback.rate, playback.rate);
        assert_eq!(got_playback.playing, playback.playing);
        assert_eq!(got_frame, frame);
        assert_eq!((got_cam.yaw, got_cam.pitch, got_cam.dist), (cam.yaw, cam.pitch, cam.dist));

        assert_eq!(got_solve.apo_alt_km, solve.apo_alt_km);
        assert_eq!(got_solve.peri_alt_km, solve.peri_alt_km);
        assert_eq!(got_solve.inc_deg, solve.inc_deg);
        assert!(got_solve.mode == TargetModeUi::Fastest);
        assert_eq!(got_solve.deadline_enabled, solve.deadline_enabled);
        assert_eq!(got_solve.deadline_hours, solve.deadline_hours);
    }

    /// Re-encoding what we decoded gives byte-identical output, so the address
    /// bar settles instead of flickering between two spellings of one state.
    #[test]
    fn encoding_is_stable_across_a_round_trip() {
        let (input, playback, frame, cam, solve) = sample_state();
        let first = to_query(&input, &playback, frame, &cam, &solve);
        let (mut i2, mut p2, mut f2, mut c2, mut s2) = (
            ScenarioInput::default(),
            Playback::default(),
            ViewFrame::default(),
            OrbitCamera { yaw: 0.0, pitch: 0.0, dist: 10.0 },
            TargetSolve::default(),
        );
        apply_query(&first, &mut i2, &mut p2, &mut f2, &mut c2, &mut s2);
        assert_eq!(first, to_query(&i2, &p2, f2, &c2, &s2));
    }

    /// A multi-line TLE full of spaces has to survive intact, and must not
    /// break the `&`/`=` framing of the query it sits in.
    #[test]
    fn tle_survives_percent_encoding() {
        let encoded = encode(SAMPLE_TLE);
        assert!(!encoded.contains(' '));
        assert!(!encoded.contains('&'));
        assert!(!encoded.contains('='));
        assert_eq!(decode(&encoded), SAMPLE_TLE);
        assert_eq!(Params::parse(&format!("tle={encoded}&v=1")).get("tle").unwrap(), SAMPLE_TLE);
    }

    /// A link with no `v` is not ours — `?demo` must fall through untouched.
    #[test]
    fn ignores_a_query_without_a_version() {
        let mut input = ScenarioInput::default();
        let before = input.wet_mass_kg;
        assert!(!apply_query(
            "?demo&burn&wet=99",
            &mut input,
            &mut Playback::default(),
            &mut ViewFrame::default(),
            &mut OrbitCamera { yaw: 0.0, pitch: 0.0, dist: 10.0 },
            &mut TargetSolve::default(),
        ));
        assert_eq!(input.wet_mass_kg, before);
    }

    /// Junk in a hand-edited link falls back to defaults rather than reaching
    /// the integrator: no panic, no NaN, no half-applied maneuver row.
    #[test]
    fn survives_a_mangled_link() {
        let mut input = ScenarioInput::default();
        let (wet, count) = (input.wet_mass_kg, input.maneuvers.len());
        assert!(apply_query(
            "v=1&wet=nan&dry=inf&fwd=&isp=abc&t=NaN&cam=1,2&mv=i,1,2&mv=x,0,0,0,0,0,0,0,0,0\
             &tle=%ZZ%&tgt=&frame=galactic&j2=maybe",
            &mut input,
            &mut Playback::default(),
            &mut ViewFrame::default(),
            &mut OrbitCamera { yaw: 0.0, pitch: 0.0, dist: 10.0 },
            &mut TargetSolve::default(),
        ));
        assert_eq!(input.wet_mass_kg, wet);
        assert!(input.isp_s.is_finite() && input.forward_days.is_finite());
        // Both rows were malformed, so the list is emptied, not corrupted.
        assert!(input.maneuvers.is_empty() || input.maneuvers.len() == count);
    }

    /// Links from a build that predates a field keep working: absent keys
    /// leave the app's own defaults in place.
    #[test]
    fn a_minimal_link_leaves_defaults_alone() {
        let mut input = ScenarioInput::default();
        let mut solve = TargetSolve::default();
        let (isp, inc) = (input.isp_s, solve.inc_deg);
        assert!(apply_query(
            "v=1&fwd=3",
            &mut input,
            &mut Playback::default(),
            &mut ViewFrame::default(),
            &mut OrbitCamera { yaw: 0.0, pitch: 0.0, dist: 10.0 },
            &mut solve,
        ));
        assert_eq!(input.forward_days, 3.0);
        assert_eq!(input.isp_s, isp);
        assert_eq!(solve.inc_deg, inc);
        assert!(input.maneuvers.is_empty());
    }
}
