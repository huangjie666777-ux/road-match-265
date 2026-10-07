use road_match265::graph::validate_and_build;
use road_match265::*;

fn params() -> Parameters {
    Parameters { radius: 8.0, k: 2, sigma: 3.0, beta: 30.0, max_speed: 50.0 }
}

fn node(id: u64, x: f64, y: f64) -> Node {
    Node { id, x, y }
}

fn edge(id: u64, from: u64, to: u64) -> Edge {
    Edge { id, from, to }
}

fn obs(x: f64, y: f64, t: f64) -> Observation {
    Observation { x, y, t }
}

fn input(net: RoadNetwork, observations: Vec<Observation>) -> Input {
    Input { road_network: net, parameters: params(), observations }
}

fn two_parallel_roads() -> RoadNetwork {
    // road A: edge 10 along y=0, road B: edge 20 along y=10 (disconnected)
    RoadNetwork {
        nodes: vec![node(1, 0.0, 0.0), node(2, 100.0, 0.0), node(3, 0.0, 10.0), node(4, 100.0, 10.0)],
        edges: vec![edge(10, 1, 2), edge(20, 3, 4)],
    }
}

#[test]
fn rejects_duplicate_node_id() {
    let mut net = two_parallel_roads();
    net.nodes.push(node(1, 5.0, 5.0));
    let e = match_input(&input(net, vec![])).unwrap_err();
    assert!(e.0.contains("duplicate node id 1"), "{e}");
}

#[test]
fn rejects_duplicate_edge_id() {
    let mut net = two_parallel_roads();
    net.edges.push(edge(10, 3, 4));
    let e = match_input(&input(net, vec![])).unwrap_err();
    assert!(e.0.contains("duplicate edge id 10"), "{e}");
}

#[test]
fn rejects_unknown_node() {
    let mut net = two_parallel_roads();
    net.edges.push(edge(30, 1, 99));
    let e = match_input(&input(net, vec![])).unwrap_err();
    assert!(e.0.contains("unknown node 99"), "{e}");
}

#[test]
fn rejects_zero_length_edge() {
    let net = RoadNetwork {
        nodes: vec![node(1, 0.0, 0.0), node(2, 0.0, 0.0)],
        edges: vec![edge(10, 1, 2)],
    };
    let e = match_input(&input(net, vec![])).unwrap_err();
    assert!(e.0.contains("zero length"), "{e}");
}

#[test]
fn rejects_non_finite_and_bad_params() {
    let mut i = input(two_parallel_roads(), vec![obs(0.0, 0.0, 0.0)]);
    i.parameters.sigma = 0.0;
    assert!(match_input(&i).unwrap_err().0.contains("sigma"));
    i.parameters.sigma = f64::NAN;
    assert!(match_input(&i).unwrap_err().0.contains("sigma"));
    i.parameters.sigma = 3.0;
    i.parameters.k = 0;
    assert!(match_input(&i).unwrap_err().0.contains("k must be"));
    i.parameters.k = 9;
    assert!(match_input(&i).unwrap_err().0.contains("k must be"));
    i.parameters.k = 2;
    i.parameters.radius = -1.0;
    assert!(match_input(&i).unwrap_err().0.contains("radius"));
    i.parameters.radius = 8.0;
    i.observations = vec![obs(f64::INFINITY, 0.0, 0.0)];
    assert!(match_input(&i).unwrap_err().0.contains("non-finite"));
}

#[test]
fn rejects_non_increasing_timestamps() {
    let i = input(two_parallel_roads(), vec![obs(0.0, 0.0, 5.0), obs(1.0, 0.0, 5.0)]);
    assert!(match_input(&i).unwrap_err().0.contains("strictly increasing"));
}

#[test]
fn rejects_too_many_edges_and_observations() {
    let mut net = two_parallel_roads();
    for j in 0..199u64 {
        net.nodes.push(node(1000 + 2 * j, 0.0, 100.0 + j as f64));
        net.nodes.push(node(1001 + 2 * j, 1.0, 100.0 + j as f64));
        net.edges.push(edge(1000 + j, 1000 + 2 * j, 1001 + 2 * j));
    }
    let e = match_input(&input(net.clone(), vec![])).unwrap_err();
    assert!(e.0.contains("too many edges"), "{e}");
    let many: Vec<Observation> = (0..101).map(|j| obs(0.0, 0.0, j as f64)).collect();
    let e = match_input(&input(net, many)).unwrap_err();
    assert!(e.0.contains("too many"), "{e}");
}

#[test]
fn viterbi_beats_nearest_on_parallel_roads() {
    // one noisy fix at (50, 6) is closer to road B (perp 4) than to
    // road A (perp 6); a greedy matcher would snap to B, but B is
    // unreachable from A, so the global optimum stays on edge 10.
    let observations = vec![
        obs(10.0, 1.0, 0.0),
        obs(30.0, -2.0, 1.0),
        obs(50.0, 6.0, 2.0),
        obs(70.0, 1.0, 3.0),
        obs(90.0, -1.0, 4.0),
    ];
    let out = match_input(&input(two_parallel_roads(), observations)).unwrap();
    assert_eq!(out.unmatched.len(), 0);
    assert_eq!(out.segments.len(), 1);
    assert!(out.matches.iter().all(|m| m.edge_id == 10));
    // along-edge distances increase monotonically
    let alongs: Vec<f64> = out.matches.iter().map(|m| m.along).collect();
    assert!(alongs.windows(2).all(|w| w[0] < w[1]));
    // transitions stay on edge 10 with direct forward distance
    assert!((out.segments[0].transitions[0].distance - 20.0).abs() < 1e-9);
    assert_eq!(out.segments[0].transitions[0].path, vec![10]);
}

#[test]
fn unmatched_point_breaks_segment() {
    let observations = vec![
        obs(10.0, 0.0, 0.0),
        obs(500.0, 500.0, 1.0), // far away from every edge
        obs(30.0, 0.0, 2.0),
    ];
    let out = match_input(&input(two_parallel_roads(), observations)).unwrap();
    assert_eq!(out.unmatched.len(), 1);
    assert_eq!(out.unmatched[0].index, 1);
    assert!(out.unmatched[0].reason.contains("no candidate"));
    assert_eq!(out.segments.len(), 2);
    assert_eq!(out.segments[0].observations, vec![0]);
    assert_eq!(out.segments[1].observations, vec![2]);
}

#[test]
fn disconnected_road_restarts_segment() {
    // jump from road A to road B: no directed route, so a new segment
    // must start even though both points have candidates.
    let observations = vec![obs(10.0, 0.0, 0.0), obs(90.0, 10.0, 1.0)];
    let out = match_input(&input(two_parallel_roads(), observations)).unwrap();
    assert_eq!(out.matches.len(), 2);
    assert_eq!(out.segments.len(), 2);
    assert_eq!(out.matches[0].edge_id, 10);
    assert_eq!(out.matches[1].edge_id, 20);
    assert_eq!(out.matches[1].segment, 1);
}

#[test]
fn speed_limit_blocks_transition() {
    // 90 m in 1 s with max_speed 50 m/s: unreachable even directly.
    let mut i = input(two_parallel_roads(), vec![obs(0.0, 0.0, 0.0), obs(90.0, 0.0, 1.0)]);
    i.parameters.max_speed = 50.0;
    let out = match_input(&i).unwrap();
    assert_eq!(out.segments.len(), 2);
}

#[test]
fn same_edge_backward_uses_legal_loop() {
    // two-way street: edges 10 (1->2) and 11 (2->1). A fix at x=80
    // followed by x=20 on edge 10 must loop: 20 remaining + 100 back
    // on edge 11 + 20 into edge 10 = 140 m.
    let net = RoadNetwork {
        nodes: vec![node(1, 0.0, 0.0), node(2, 100.0, 0.0)],
        edges: vec![edge(10, 1, 2), edge(11, 2, 1)],
    };
    let mut i = input(net, vec![obs(80.0, 0.0, 0.0), obs(20.0, 0.0, 10.0)]);
    i.parameters.k = 1;
    let out = match_input(&i).unwrap();
    assert_eq!(out.segments.len(), 1);
    assert!(out.matches.iter().all(|m| m.edge_id == 10));
    let tr = &out.segments[0].transitions[0];
    assert_eq!(tr.path, vec![10, 11, 10]);
    assert!((tr.distance - 140.0).abs() < 1e-9);
}

#[test]
fn shortest_path_tie_breaks_by_edge_id_sequence() {
    // diamond: 1->2 (edge 5), 1->3 (edge 7), 2->4 (edge 8), 3->4 (edge 9)
    // both routes 1->4 are 20 m; [5, 8] wins over [7, 9].
    let net = RoadNetwork {
        nodes: vec![
            node(1, 0.0, 0.0),
            node(2, 10.0, 5.0),
            node(3, 10.0, -5.0),
            node(4, 20.0, 0.0),
        ],
        edges: vec![edge(5, 1, 2), edge(7, 1, 3), edge(8, 2, 4), edge(9, 3, 4)],
    };
    let g = validate_and_build(&Input {
        road_network: net,
        parameters: params(),
        observations: vec![],
    })
    .unwrap();
    let (d, path) = graph::shortest_path(&g, 0, 3).unwrap();
    let expect = 2.0 * (10.0f64.hypot(5.0));
    assert!((d - expect).abs() < 1e-9);
    assert_eq!(path, vec![5, 8]);
}

#[test]
fn candidate_tie_breaks_by_edge_id() {
    // point exactly between the two parallel roads (y=5): both perp=5
    let net = two_parallel_roads();
    let g = validate_and_build(&Input {
        road_network: net,
        parameters: params(),
        observations: vec![],
    })
    .unwrap();
    let cands = candidate::candidates_for(&g, 50.0, 5.0, 8.0, 2);
    assert_eq!(cands.len(), 2);
    assert_eq!(g.edges[cands[0].edge].id, 10);
    assert_eq!(g.edges[cands[1].edge].id, 20);
}

#[test]
fn json_roundtrip() {
    let req = r#"{
        "road_network": {
            "nodes": [{"id":1,"x":0.0,"y":0.0},{"id":2,"x":100.0,"y":0.0}],
            "edges": [{"id":10,"from":1,"to":2}]
        },
        "parameters": {"radius":8.0,"k":1,"sigma":3.0,"beta":30.0,"max_speed":50.0},
        "observations": [{"x":10.0,"y":1.0,"t":0.0},{"x":30.0,"y":-1.0,"t":1.0}]
    }"#;
    let out = match_json(req).unwrap();
    let parsed: serde_json::Value = serde_json::from_str(&out).unwrap();
    assert_eq!(parsed["matches"][0]["edge_id"], 10);
    assert_eq!(parsed["segments"][0]["transitions"][0]["path"], serde_json::json!([10]));
    let bad = match_json("{ not json").unwrap_err();
    assert!(bad.0.contains("invalid JSON"));
}
