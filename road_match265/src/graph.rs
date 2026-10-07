use crate::model::*;
use std::collections::BinaryHeap;
use std::collections::HashMap;

/// A directed edge with precomputed metric geometry.
#[derive(Debug, Clone)]
pub struct EdgeGeom {
    pub id: u64,
    pub from: usize,
    pub to: usize,
    pub ax: f64,
    pub ay: f64,
    pub bx: f64,
    pub by: f64,
    pub length: f64,
}

/// Metric directed graph built from the validated road network.
#[derive(Debug, Clone)]
pub struct Graph {
    pub node_xy: Vec<(f64, f64)>,
    pub edges: Vec<EdgeGeom>,
    /// adjacency: node index -> edge indices, sorted by edge id
    pub adj: Vec<Vec<usize>>,
}

pub fn validate_and_build(input: &Input) -> Result<Graph, MatchError> {
    let net = &input.road_network;
    let p = &input.parameters;

    if net.edges.len() > MAX_EDGES {
        return err(format!("too many edges: {} > {}", net.edges.len(), MAX_EDGES));
    }
    if input.observations.len() > MAX_OBSERVATIONS {
        return err(format!(
            "too many observations: {} > {}",
            input.observations.len(),
            MAX_OBSERVATIONS
        ));
    }
    for (name, v) in [
        ("radius", p.radius),
        ("sigma", p.sigma),
        ("beta", p.beta),
        ("max_speed", p.max_speed),
    ] {
        if !v.is_finite() || v <= 0.0 {
            return err(format!("parameter {name} must be finite and > 0, got {v}"));
        }
    }
    if !(MIN_K..=MAX_K).contains(&p.k) {
        return err(format!("parameter k must be in {MIN_K}..={MAX_K}, got {}", p.k));
    }

    let mut node_index: HashMap<u64, usize> = HashMap::new();
    let mut node_xy = Vec::with_capacity(net.nodes.len());
    for n in &net.nodes {
        if !n.x.is_finite() || !n.y.is_finite() {
            return err(format!("node {} has non-finite coordinates", n.id));
        }
        if node_index.insert(n.id, node_xy.len()).is_some() {
            return err(format!("duplicate node id {}", n.id));
        }
        node_xy.push((n.x, n.y));
    }

    let mut edge_ids = std::collections::HashSet::new();
    let mut edges = Vec::with_capacity(net.edges.len());
    let mut adj = vec![Vec::new(); node_xy.len()];
    for e in &net.edges {
        if !edge_ids.insert(e.id) {
            return err(format!("duplicate edge id {}", e.id));
        }
        let from = *node_index
            .get(&e.from)
            .ok_or_else(|| MatchError(format!("edge {} references unknown node {}", e.id, e.from)))?;
        let to = *node_index
            .get(&e.to)
            .ok_or_else(|| MatchError(format!("edge {} references unknown node {}", e.id, e.to)))?;
        let (ax, ay) = node_xy[from];
        let (bx, by) = node_xy[to];
        let length = (bx - ax).hypot(by - ay);
        if length == 0.0 {
            return err(format!("edge {} has zero length", e.id));
        }
        adj[from].push(edges.len());
        edges.push(EdgeGeom { id: e.id, from, to, ax, ay, bx, by, length });
    }
    for list in &mut adj {
        list.sort_by_key(|&i| edges[i].id);
    }

    let mut prev_t: Option<f64> = None;
    for (i, o) in input.observations.iter().enumerate() {
        if !o.x.is_finite() || !o.y.is_finite() || !o.t.is_finite() {
            return err(format!("observation {i} has non-finite values"));
        }
        if let Some(pt) = prev_t {
            if o.t <= pt {
                return err(format!(
                    "observation timestamps must be strictly increasing (index {i})"
                ));
            }
        }
        prev_t = Some(o.t);
    }

    Ok(Graph { node_xy, edges, adj })
}

/// Dijkstra shortest directed path between two node indices.
/// Ties in distance are broken by the lexicographically smallest
/// sequence of edge ids. Returns (distance, edge id sequence).
pub fn shortest_path(graph: &Graph, from: usize, to: usize) -> Option<(f64, Vec<u64>)> {
    if from == to {
        return Some((0.0, Vec::new()));
    }
    #[derive(Debug, PartialEq)]
    struct State {
        dist: f64,
        path: Vec<u64>,
        node: usize,
    }
    impl Eq for State {}
    impl Ord for State {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            // reversed: smallest (dist, path) pops first
            other
                .dist
                .total_cmp(&self.dist)
                .then_with(|| other.path.cmp(&self.path))
        }
    }
    impl PartialOrd for State {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut best: HashMap<usize, (f64, Vec<u64>)> = HashMap::new();
    let mut heap = BinaryHeap::new();
    heap.push(State { dist: 0.0, path: Vec::new(), node: from });
    while let Some(State { dist, path, node }) = heap.pop() {
        if let Some((d, p)) = best.get(&node) {
            if (*d, p.clone()) <= (dist, path.clone()) {
                continue;
            }
        }
        if node == to {
            return Some((dist, path));
        }
        best.insert(node, (dist, path.clone()));
        for &ei in &graph.adj[node] {
            let e = &graph.edges[ei];
            let mut npath = path.clone();
            npath.push(e.id);
            heap.push(State { dist: dist + e.length, path: npath, node: e.to });
        }
    }
    None
}
