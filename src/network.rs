use std::collections::HashMap;
use std::fmt;

/// Maximum number of edges accepted in a network.
pub const MAX_EDGES: usize = 200;
/// Maximum number of observations accepted in a trace.
pub const MAX_OBSERVATIONS: usize = 100;

/// Errors rejected by the library.
#[derive(Debug, Clone, PartialEq)]
pub enum MatchError {
    DuplicateNodeId(u64),
    DuplicateEdgeId(u64),
    UnknownNode { edge: u64, node: u64 },
    ZeroLengthEdge(u64),
    NonFiniteValue(&'static str),
    TooManyEdges,
    TooManyObservations,
    EmptyTrace,
    NonIncreasingTimestamps { index: usize },
    InvalidParameter(&'static str),
}

impl fmt::Display for MatchError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MatchError::DuplicateNodeId(id) => write!(f, "duplicate node id {id}"),
            MatchError::DuplicateEdgeId(id) => write!(f, "duplicate edge id {id}"),
            MatchError::UnknownNode { edge, node } => {
                write!(f, "edge {edge} references unknown node {node}")
            }
            MatchError::ZeroLengthEdge(id) => write!(f, "edge {id} has zero length"),
            MatchError::NonFiniteValue(what) => write!(f, "non-finite value for {what}"),
            MatchError::TooManyEdges => write!(f, "network exceeds {MAX_EDGES} edges"),
            MatchError::TooManyObservations => {
                write!(f, "trace exceeds {MAX_OBSERVATIONS} observations")
            }
            MatchError::EmptyTrace => write!(f, "trace is empty"),
            MatchError::NonIncreasingTimestamps { index } => {
                write!(f, "timestamps not strictly increasing at index {index}")
            }
            MatchError::InvalidParameter(what) => write!(f, "invalid parameter: {what}"),
        }
    }
}

impl std::error::Error for MatchError {}

/// A network node with metric coordinates (metres).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Node {
    pub id: u64,
    pub x: f64,
    pub y: f64,
}

/// A directed straight edge between two nodes.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Edge {
    pub id: u64,
    pub from: u64,
    pub to: u64,
    /// Endpoint distance in metres.
    pub length: f64,
}

/// Directed road network.
#[derive(Debug, Clone, Default)]
pub struct RoadNetwork {
    nodes: HashMap<u64, Node>,
    edges: Vec<Edge>,
    edge_index: HashMap<u64, usize>,
    adjacency: HashMap<u64, Vec<usize>>,
}

impl RoadNetwork {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_node(&mut self, id: u64, x: f64, y: f64) -> Result<(), MatchError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(MatchError::NonFiniteValue("node coordinate"));
        }
        if self.nodes.contains_key(&id) {
            return Err(MatchError::DuplicateNodeId(id));
        }
        self.nodes.insert(id, Node { id, x, y });
        Ok(())
    }

    pub fn add_edge(&mut self, id: u64, from: u64, to: u64) -> Result<(), MatchError> {
        if self.edges.len() >= MAX_EDGES {
            return Err(MatchError::TooManyEdges);
        }
        if self.edge_index.contains_key(&id) {
            return Err(MatchError::DuplicateEdgeId(id));
        }
        let a = *self
            .nodes
            .get(&from)
            .ok_or(MatchError::UnknownNode { edge: id, node: from })?;
        let b = *self
            .nodes
            .get(&to)
            .ok_or(MatchError::UnknownNode { edge: id, node: to })?;
        let length = (b.x - a.x).hypot(b.y - a.y);
        if length <= 0.0 {
            return Err(MatchError::ZeroLengthEdge(id));
        }
        self.edge_index.insert(id, self.edges.len());
        self.adjacency.entry(from).or_default().push(self.edges.len());
        self.edges.push(Edge { id, from, to, length });
        Ok(())
    }

    pub fn node(&self, id: u64) -> Option<&Node> {
        self.nodes.get(&id)
    }

    pub fn edge(&self, id: u64) -> Option<&Edge> {
        self.edge_index.get(&id).map(|&i| &self.edges[i])
    }

    pub fn edges(&self) -> &[Edge] {
        &self.edges
    }

    pub(crate) fn outgoing(&self, node: u64) -> &[usize] {
        self.adjacency.get(&node).map(Vec::as_slice).unwrap_or(&[])
    }

    pub(crate) fn edge_at(&self, index: usize) -> &Edge {
        &self.edges[index]
    }
}
