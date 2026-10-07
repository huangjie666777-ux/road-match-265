use crate::graph::Graph;

/// A candidate match of one observation on one edge.
#[derive(Debug, Clone)]
pub struct Candidate {
    pub edge: usize,
    /// projected point, meters
    pub x: f64,
    pub y: f64,
    /// distance along the edge from its start node, meters
    pub along: f64,
    /// perpendicular distance from observation to the edge, meters
    pub perp: f64,
}

/// Project the point onto every edge and keep the nearest k edges
/// within radius. Ties in distance are broken by ascending edge id.
pub fn candidates_for(graph: &Graph, x: f64, y: f64, radius: f64, k: usize) -> Vec<Candidate> {
    let mut cands: Vec<Candidate> = Vec::new();
    for (ei, e) in graph.edges.iter().enumerate() {
        let dx = e.bx - e.ax;
        let dy = e.by - e.ay;
        let len2 = e.length * e.length;
        let t = (((x - e.ax) * dx + (y - e.ay) * dy) / len2).clamp(0.0, 1.0);
        let px = e.ax + t * dx;
        let py = e.ay + t * dy;
        let perp = (x - px).hypot(y - py);
        if perp <= radius {
            cands.push(Candidate { edge: ei, x: px, y: py, along: t * e.length, perp });
        }
    }
    cands.sort_by(|a, b| {
        a.perp
            .total_cmp(&b.perp)
            .then_with(|| graph.edges[a.edge].id.cmp(&graph.edges[b.edge].id))
    });
    cands.truncate(k);
    cands
}
