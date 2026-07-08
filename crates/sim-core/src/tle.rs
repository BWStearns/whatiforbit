use crate::State;
use glam::DVec3;
use hifitime::Epoch;

#[derive(Debug)]
pub struct TleError(pub String);

impl std::fmt::Display for TleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for TleError {}

/// Result of parsing a TLE: the state at TLE epoch and the object name if
/// a title line was present.
pub struct TleState {
    pub state: State,
    pub name: Option<String>,
    pub norad_id: u64,
    /// International designator (e.g. "1998-067A") when present in the TLE.
    pub international_designator: Option<String>,
}

/// Parse a TLE from free-form text: accepts 2-line and 3-line (name first)
/// sets, ignoring surrounding blank lines. The state is the SGP4 solution at
/// the TLE's own epoch, in TEME (treated as this app's inertial frame).
///
/// Mass is set to 0 here — the caller overwrites it from the vehicle config.
pub fn parse_tle(text: &str) -> Result<TleState, TleError> {
    let lines: Vec<&str> = text
        .lines()
        .map(str::trim_end)
        .filter(|l| !l.trim().is_empty())
        .collect();

    let l1_idx = lines
        .iter()
        .position(|l| l.starts_with("1 "))
        .ok_or_else(|| TleError("no line starting with '1 ' found".into()))?;
    let l2 = lines
        .get(l1_idx + 1)
        .filter(|l| l.starts_with("2 "))
        .ok_or_else(|| TleError("line 1 not followed by a line starting with '2 '".into()))?;
    let name = (l1_idx > 0).then(|| lines[l1_idx - 1].trim().to_string());

    let elements = sgp4::Elements::from_tle(
        name.clone(),
        lines[l1_idx].as_bytes(),
        l2.as_bytes(),
    )
    .map_err(|e| TleError(format!("TLE parse error: {e}")))?;

    let constants = sgp4::Constants::from_elements(&elements)
        .map_err(|e| TleError(format!("SGP4 init error: {e}")))?;

    let prediction = constants
        .propagate(sgp4::MinutesSinceEpoch(0.0))
        .map_err(|e| TleError(format!("SGP4 propagation error: {e}")))?;

    let dt = elements.datetime;
    let epoch = Epoch::from_unix_seconds(
        dt.and_utc().timestamp() as f64 + f64::from(dt.and_utc().timestamp_subsec_nanos()) * 1e-9,
    );

    let r = DVec3::from_array(prediction.position);
    let v = DVec3::from_array(prediction.velocity);

    Ok(TleState {
        state: State::new(epoch, r, v, 0.0),
        name: elements.object_name.clone().or(name),
        norad_id: elements.norad_id,
        international_designator: elements.international_designator.clone(),
    })
}
