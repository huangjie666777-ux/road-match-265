use serde::{Deserialize, Serialize};

pub const MAX_EDGES: usize = 200;
pub const MAX_OBSERVATIONS: usize = 100;
pub const MIN_K: usize = 1;
pub const MAX_K: usize = 8;

#[derive(Debug, Clone, Deserialize)]
pub struct Input {
    pub road_network: RoadNetwork,
    pub parameters: Parameters,
    pub observations: Vec<Observation>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct RoadNetwork {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Node {
    pub id: u64,
    /// meters
    pub x: f64,
    /// meters
    pub y: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Edge {
    pub id: u64,
    pub from: u64,
    pub to: u64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Parameters {
    /// candidate search radius, meters, > 0
    pub radius: f64,
    /// number of nearest candidates kept per observation, 1..=8
    pub k: usize,
    /// GPS noise stddev, meters, > 0
    pub sigma: f64,
    /// transition scale, meters, > 0
    pub beta: f64,
    /// speed limit, meters per second, > 0
    pub max_speed: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Observation {
    /// meters
    pub x: f64,
    /// meters
    pub y: f64,
    /// seconds, strictly increasing
    pub t: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MatchPoint {
    pub index: usize,
    pub segment: usize,
    pub edge_id: u64,
    /// projected point on the edge, meters
    pub x: f64,
    pub y: f64,
    /// distance along the edge from its start node, meters
    pub along: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct UnmatchedPoint {
    pub index: usize,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Transition {
    pub from_index: usize,
    pub to_index: usize,
    /// full directed path as edge id sequence
    pub path: Vec<u64>,
    /// total route distance along the network, meters
    pub distance: f64,
    pub cost: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Segment {
    pub id: usize,
    /// sum of emission and transition costs inside the segment
    pub cost: f64,
    pub observations: Vec<usize>,
    pub transitions: Vec<Transition>,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct Output {
    pub matches: Vec<MatchPoint>,
    pub unmatched: Vec<UnmatchedPoint>,
    pub segments: Vec<Segment>,
    /// global minimal total cost of the chosen solution
    pub total_cost: f64,
}

#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct MatchError(pub String);

impl std::fmt::Display for MatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for MatchError {}

pub fn err<T>(msg: impl Into<String>) -> Result<T, MatchError> {
    Err(MatchError(msg.into()))
}
