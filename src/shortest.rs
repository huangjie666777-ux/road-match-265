use crate::network::RoadNetwork;
use std::collections::HashMap;

/// Directed shortest path between two nodes.
///
/// Returns (total metres, edge id path). A start == end query yields zero
/// metres and an empty path. Among equal-length paths the lexicographically
/// smallest edge id sequence wins.
pub fn shortest_path(net: &RoadNetwork, start: u64, goal: u64) -> Option<(f64, Vec<u64>)> {
    // best[node] = (distance, edge id path from start)
    let mut best: HashMap<u64, (f64, Vec<u64>)> = HashMap::new();
    best.insert(start, (0.0, Vec::new()));
    // Small networks (<= 200 edges): a simple O(V*E) relaxation loop is fine
    // and keeps the lexicographic tie-break exact.
    loop {
        let mut improved = false;
        let snapshot: Vec<(u64, f64, Vec<u64>)> = best
            .iter()
            .map(|(&n, (d, p))| (n, *d, p.clone()))
            .collect();
        for (node, dist, path) in snapshot {
            for &ei in net.outgoing(node) {
                let edge = net.edge_at(ei);
                let nd = dist + edge.length;
                let mut np = path.clone();
                np.push(edge.id);
                let replace = match best.get(&edge.to) {
                    None => true,
                    Some((d, p)) => nd < *d || (nd == *d && np < *p),
                };
                if replace {
                    best.insert(edge.to, (nd, np));
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
    best.get(&goal).map(|(d, p)| (*d, p.clone()))
}
