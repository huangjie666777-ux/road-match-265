use crate::network::RoadNetwork;
use serde::Serialize;

/// A projection of one observation onto one edge.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Candidate {
    pub edge_id: u64,
    /// Distance along the edge from its start node, in metres.
    pub along: f64,
    /// Projected point on the edge, in metres.
    pub x: f64,
    pub y: f64,
    /// Perpendicular distance from the observation to the edge, in metres.
    pub distance: f64,
}

/// Projects (px, py) onto the segment (ax, ay)-(bx, by).
/// Returns (projected x, projected y, distance along the segment, perpendicular distance).
pub fn project_point(
    px: f64,
    py: f64,
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
) -> (f64, f64, f64, f64) {
    let dx = bx - ax;
    let dy = by - ay;
    let len_sq = dx * dx + dy * dy;
    let t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let x = ax + t * dx;
    let y = ay + t * dy;
    let along = t * len_sq.sqrt();
    let distance = (px - x).hypot(py - y);
    (x, y, along, distance)
}

/// Collects the nearest `k` candidates within `radius` of the observation.
/// Ties in distance are broken by ascending edge id.
pub fn candidates_for(net: &RoadNetwork, px: f64, py: f64, radius: f64, k: usize) -> Vec<Candidate> {
    let mut all: Vec<Candidate> = Vec::new();
    for edge in net.edges() {
        let a = net.node(edge.from).expect("edge endpoints are validated");
        let b = net.node(edge.to).expect("edge endpoints are validated");
        let (x, y, along, distance) = project_point(px, py, a.x, a.y, b.x, b.y);
        if distance <= radius {
            all.push(Candidate { edge_id: edge.id, along, x, y, distance });
        }
    }
    all.sort_by(|c1, c2| {
        c1.distance
            .partial_cmp(&c2.distance)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(c1.edge_id.cmp(&c2.edge_id))
    });
    all.truncate(k);
    all
}
