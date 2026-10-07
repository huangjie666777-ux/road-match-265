//! Demonstrates parallel-road disambiguation and a broken (unreachable) road.
//! Prints the match output as JSON. Run with: cargo run --example demo
use road_match265::{Matcher, MatcherConfig, Observation, RoadNetwork};

fn main() {
    let config = MatcherConfig {
        sigma: 5.0,
        beta: 10.0,
        max_speed: 30.0,
        radius: 20.0,
        k: 4,
    };

    // Scenario 1: two parallel one-way roads 8m apart. The trace follows the
    // northern road (edge 2), but one noisy fix sits closer to edge 1.
    // Per-point nearest-edge matching would jump; Viterbi stays on edge 2.
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_node(3, 0.0, 8.0).unwrap();
    net.add_node(4, 100.0, 8.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 3, 4).unwrap();
    let trace = vec![
        Observation { x: 10.0, y: 8.5, t: 0.0 },
        Observation { x: 30.0, y: 8.2, t: 1.0 },
        Observation { x: 50.0, y: 1.0, t: 2.0 }, // noisy fix near edge 1
        Observation { x: 70.0, y: 8.3, t: 3.0 },
        Observation { x: 90.0, y: 8.4, t: 4.0 },
    ];
    let matcher = Matcher::new(&net, config).unwrap();
    let out = matcher.match_trace(&trace).unwrap();
    println!("=== parallel roads (global Viterbi stays on edge 2) ===");
    println!("{}", serde_json::to_string_pretty(&out).unwrap());

    // Scenario 2: broken road. Edge 2 starts at a node that cannot be reached
    // from edge 1's end (one-way gap), so the trace splits into two segments.
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 50.0, 0.0).unwrap();
    net.add_node(3, 60.0, 0.0).unwrap();
    net.add_node(4, 110.0, 0.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 3, 4).unwrap();
    let trace = vec![
        Observation { x: 40.0, y: 1.0, t: 0.0 },
        Observation { x: 70.0, y: 1.0, t: 1.0 },
        Observation { x: 90.0, y: 1.0, t: 2.0 },
    ];
    let matcher = Matcher::new(&net, config).unwrap();
    let out = matcher.match_trace(&trace).unwrap();
    println!("=== broken road (segment restarts on edge 2) ===");
    println!("{}", serde_json::to_string_pretty(&out).unwrap());
}
