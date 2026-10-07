//! road_match265: map matching of noisy GPS fixes to a directed metric
//! road network using a Viterbi hidden Markov model.
//!
//! All coordinates and distances are meters, timestamps are seconds.

pub mod candidate;
pub mod graph;
pub mod matcher;
pub mod model;

pub use matcher::match_input;
pub use model::*;

/// Parse a JSON request, run matching, and serialize the result as JSON.
/// Validation and parse failures are returned as MatchError.
pub fn match_json(input: &str) -> Result<String, MatchError> {
    let parsed: Input = serde_json::from_str(input)
        .map_err(|e| MatchError(format!("invalid JSON input: {e}")))?;
    let output = match_input(&parsed)?;
    serde_json::to_string_pretty(&output)
        .map_err(|e| MatchError(format!("failed to serialize output: {e}")))
}
