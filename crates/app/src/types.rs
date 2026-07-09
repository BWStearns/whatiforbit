//! App-level resources: user inputs, sim outputs, playback state.

use bevy::prelude::*;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Mutex;
use whatiforbit_sim::{SimResult, State};

/// Fabricated-but-plausible ISS TLE (epoch 2026-01-01) used by the
/// "load sample" button so the app works offline.
pub const SAMPLE_TLE: &str = "ISS (ZARYA)
1 25544U 98067A   26001.50000000  .00016717  00000-0  10270-3 0  9001
2 25544  51.6400 208.9163 0006317  69.9862 290.2000 15.49560000123452";

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ManeuverKind {
    Impulsive,
    FiniteBurn,
}

/// UI-friendly maneuver row; converted to `whatiforbit_sim::Maneuver` on
/// recompute. Offsets in minutes and delta-v in m/s to match how people
/// actually type these numbers.
///
/// Maneuvers stack chronologically: `t_offset_min` is minutes after the
/// *previous maneuver ends* (for the first row: after the scenario epoch),
/// so a later row can never execute before an earlier one. Absolute times
/// come from `sim::stacked_start_times_s`.
#[derive(Clone, PartialEq, Debug)]
pub struct ManeuverInput {
    pub kind: ManeuverKind,
    pub t_offset_min: f64,
    /// Impulsive delta-v components, m/s, VNC.
    pub dv_vnc_m_s: [f64; 3],
    /// Finite burn fields.
    pub duration_s: f64,
    pub throttle: f64,
    pub direction_vnc: [f64; 3],
}

impl Default for ManeuverInput {
    fn default() -> Self {
        Self {
            kind: ManeuverKind::Impulsive,
            t_offset_min: 10.0,
            dv_vnc_m_s: [10.0, 0.0, 0.0],
            duration_s: 120.0,
            throttle: 1.0,
            direction_vnc: [1.0, 0.0, 0.0],
        }
    }
}

#[derive(Clone, Debug)]
pub struct LoadedOrbit {
    pub name: String,
    pub state: State,
    /// International designator for OPM OBJECT_ID, when known.
    pub object_id: Option<String>,
}

#[derive(Resource)]
pub struct ScenarioInput {
    pub tle_text: String,
    pub norad_query: String,
    pub fetch_in_flight: bool,
    pub loaded: Option<LoadedOrbit>,
    pub error: Option<String>,

    pub wet_mass_kg: f64,
    pub dry_mass_kg: f64,
    pub thrust_n: f64,
    pub isp_s: f64,

    pub infinite_fuel: bool,
    pub enable_j2: bool,

    pub forward_days: f64,
    pub backward_days: f64,

    pub maneuvers: Vec<ManeuverInput>,
    pub dirty: bool,
}

impl Default for ScenarioInput {
    fn default() -> Self {
        Self {
            tle_text: String::new(),
            norad_query: "25544".into(),
            fetch_in_flight: false,
            loaded: None,
            error: None,
            wet_mass_kg: 1000.0,
            dry_mass_kg: 600.0,
            thrust_n: 400.0,
            isp_s: 300.0,
            infinite_fuel: false,
            enable_j2: true,
            forward_days: 1.0,
            backward_days: 0.0,
            maneuvers: Vec::new(),
            dirty: false,
        }
    }
}

#[derive(Resource, Default)]
pub struct SimOutput {
    /// Trajectory without any maneuvers.
    pub baseline: Option<SimResult>,
    /// Trajectory with maneuvers applied.
    pub whatif: Option<SimResult>,
    /// The scenario that produced `whatif` (kept for OPM export).
    pub scenario: Option<whatiforbit_sim::Scenario>,
    pub t_start_s: f64,
    pub t_end_s: f64,
}

/// A point on a rendered trajectory currently under the mouse cursor.
#[derive(Resource, Default)]
pub struct TrackHover(pub Option<HoverInfo>);

#[derive(Clone, Copy, Debug)]
pub struct HoverInfo {
    pub track: &'static str,
    pub t_s: f64,
    pub r_km: glam::DVec3,
    pub speed_km_s: f64,
    pub mass_kg: f64,
}

/// Generated OPM text awaiting display/copy/download.
#[derive(Resource, Default)]
pub struct OpmExport {
    pub text: Option<String>,
    pub status: Option<String>,
}

/// Refinement lifecycle of one target-orbit candidate row.
pub enum RowPlanState {
    /// Impulsive template numbers only (not yet refined).
    Impulsive,
    Refining,
    Refined(whatiforbit_sim::targeting::RefinedPlan),
    Failed(String),
}

pub struct CandidateRow {
    pub label: String,
    pub dv_km_s: f64,
    pub prop_kg: f64,
    pub duration_s: f64,
    pub n_burns: usize,
    pub feasible: bool,
    /// Why this row is infeasible (empty when feasible).
    pub blocking: String,
    pub plan: whatiforbit_sim::targeting::ImpulsivePlan,
    pub state: RowPlanState,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TargetModeUi {
    Cheapest,
    Fastest,
}

/// Target-orbit solver panel state.
#[derive(Resource)]
pub struct TargetSolve {
    pub apo_alt_km: f64,
    pub peri_alt_km: f64,
    pub inc_deg: f64,
    pub mode: TargetModeUi,
    pub deadline_enabled: bool,
    pub deadline_hours: f64,

    pub rows: Vec<CandidateRow>,
    /// Human-readable verdict/shortfall block (None until first solve).
    pub verdict_text: Option<String>,
    /// Closest-achievable orbit offered on shortfall: (apo alt, peri alt, inc).
    pub closest_offer: Option<(f64, f64, f64)>,

    /// In-flight refinement: (row index, refiner).
    pub active_refine: Option<(usize, whatiforbit_sim::targeting::Refiner)>,
    /// Apply this row to the maneuver list as soon as its refinement lands.
    pub apply_when_done: Option<usize>,
    /// Trajectory preview for the hovered row.
    pub preview: Option<whatiforbit_sim::Trajectory>,
    pub preview_row: Option<usize>,
}

impl Default for TargetSolve {
    fn default() -> Self {
        Self {
            apo_alt_km: 800.0,
            peri_alt_km: 800.0,
            inc_deg: 51.6,
            mode: TargetModeUi::Cheapest,
            deadline_enabled: false,
            deadline_hours: 24.0,
            rows: Vec::new(),
            verdict_text: None,
            closest_offer: None,
            active_refine: None,
            apply_when_done: None,
            preview: None,
            preview_row: None,
        }
    }
}

#[derive(Resource)]
pub struct Playback {
    /// Current scrub time, seconds relative to scenario epoch.
    pub t_s: f64,
    pub playing: bool,
    /// Sim seconds per wall-clock second.
    pub rate: f64,
}

impl Default for Playback {
    fn default() -> Self {
        Self {
            t_s: 0.0,
            playing: false,
            rate: 60.0,
        }
    }
}

pub type FetchResult = Result<String, String>;

/// Channel for async CelesTrak fetches (ehttp callback -> Bevy system).
#[derive(Resource)]
pub struct FetchChannel {
    pub tx: Sender<FetchResult>,
    pub rx: Mutex<Receiver<FetchResult>>,
}

impl Default for FetchChannel {
    fn default() -> Self {
        let (tx, rx) = std::sync::mpsc::channel();
        Self {
            tx,
            rx: Mutex::new(rx),
        }
    }
}
