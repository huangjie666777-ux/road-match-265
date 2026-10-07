//! road_match265: map matching of noisy GPS traces onto a directed road network.
//!
//! The network uses metric node coordinates and unique edge ids. Edge length is
//! the endpoint distance; two-way roads need two edges; geometric crossings do
//! not connect edges. Matching uses a Viterbi search with geometric emission
//! costs and routing-based transition costs.

mod candidates;
mod matcher;
mod network;
mod shortest;
mod viterbi;

pub use candidates::{candidates_for, project_point, Candidate};
pub use matcher::{MatchOutput, MatchedPoint, Matcher, MatcherConfig, Observation, SegmentResult, TransitionResult};
pub use network::{Edge, MatchError, Node, RoadNetwork};
pub use shortest::shortest_path;
