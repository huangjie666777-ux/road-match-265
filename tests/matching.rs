use road_match265::*;

fn cfg() -> MatcherConfig {
    MatcherConfig { sigma: 5.0, beta: 10.0, max_speed: 30.0, radius: 20.0, k: 4 }
}

fn obs(x: f64, y: f64, t: f64) -> Observation {
    Observation { x, y, t }
}

#[test]
fn rejects_duplicate_and_unknown_and_zero_length() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    assert_eq!(net.add_node(1, 1.0, 1.0), Err(MatchError::DuplicateNodeId(1)));
    assert_eq!(
        net.add_edge(1, 1, 99),
        Err(MatchError::UnknownNode { edge: 1, node: 99 })
    );
    net.add_node(2, 0.0, 0.0).unwrap();
    assert_eq!(net.add_edge(1, 1, 2), Err(MatchError::ZeroLengthEdge(1)));
    net.add_node(3, 10.0, 0.0).unwrap();
    net.add_edge(1, 1, 3).unwrap();
    assert_eq!(net.add_edge(1, 1, 3), Err(MatchError::DuplicateEdgeId(1)));
    assert_eq!(
        net.add_node(9, f64::NAN, 0.0),
        Err(MatchError::NonFiniteValue("node coordinate"))
    );
}

#[test]
fn rejects_invalid_params_and_trace() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 10.0, 0.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    let mut bad = cfg();
    bad.sigma = 0.0;
    assert!(Matcher::new(&net, bad).is_err());
    let mut bad = cfg();
    bad.k = 9;
    assert!(Matcher::new(&net, bad).is_err());
    let m = Matcher::new(&net, cfg()).unwrap();
    assert_eq!(m.match_trace(&[]).unwrap_err(), MatchError::EmptyTrace);
    assert_eq!(
        m.match_trace(&[obs(0.0, 0.0, 5.0), obs(1.0, 0.0, 5.0)]).unwrap_err(),
        MatchError::NonIncreasingTimestamps { index: 1 }
    );
    let many: Vec<Observation> = (0..101).map(|i| obs(0.0, 0.0, i as f64)).collect();
    assert_eq!(m.match_trace(&many).unwrap_err(), MatchError::TooManyObservations);
}

#[test]
fn projects_and_orders_candidates() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_node(3, 0.0, 10.0).unwrap();
    net.add_node(4, 100.0, 10.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 3, 4).unwrap();
    let cands = candidates_for(&net, 50.0, 4.0, 20.0, 8);
    assert_eq!(cands.len(), 2);
    assert_eq!(cands[0].edge_id, 1);
    assert!((cands[0].along - 50.0).abs() < 1e-9);
    assert!((cands[0].distance - 4.0).abs() < 1e-9);
}

#[test]
fn matches_straight_trace_with_viterbi() {
    // Two parallel one-way roads 8m apart; trace stays near road 2 except
    // one noisy point closer to road 1. Global Viterbi must keep road 2.
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_node(3, 0.0, 8.0).unwrap();
    net.add_node(4, 100.0, 8.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 3, 4).unwrap();
    let m = Matcher::new(&net, cfg()).unwrap();
    let trace = vec![
        obs(10.0, 8.5, 0.0),
        obs(30.0, 8.2, 1.0),
        obs(50.0, 1.0, 2.0), // noisy: nearer to road 1
        obs(70.0, 8.3, 3.0),
        obs(90.0, 8.4, 4.0),
    ];
    let out = m.match_trace(&trace).unwrap();
    assert_eq!(out.segments.len(), 1);
    for o in &out.observations {
        assert_eq!(o.matched.as_ref().unwrap().edge_id, 2);
    }
}

#[test]
fn unmatched_point_breaks_segment() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    let m = Matcher::new(&net, cfg()).unwrap();
    let trace = vec![
        obs(10.0, 1.0, 0.0),
        obs(30.0, 500.0, 1.0), // far away: no candidate
        obs(50.0, 1.0, 2.0),
    ];
    let out = m.match_trace(&trace).unwrap();
    assert_eq!(out.segments.len(), 2);
    assert!(out.observations[1].matched.is_none());
    assert_eq!(
        out.observations[1].unmatched_reason.as_deref(),
        Some("no candidate within radius")
    );
}

#[test]
fn broken_road_restarts_segment() {
    // Edge 2 starts at a node unreachable from edge 1's end (one-way gap).
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 50.0, 0.0).unwrap();
    net.add_node(3, 60.0, 0.0).unwrap();
    net.add_node(4, 110.0, 0.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 3, 4).unwrap();
    let m = Matcher::new(&net, cfg()).unwrap();
    let trace = vec![obs(40.0, 1.0, 0.0), obs(70.0, 1.0, 1.0)];
    let out = m.match_trace(&trace).unwrap();
    assert_eq!(out.segments.len(), 2);
    assert_eq!(out.observations[0].matched.as_ref().unwrap().edge_id, 1);
    assert_eq!(out.observations[1].matched.as_ref().unwrap().edge_id, 2);
}

#[test]
fn same_edge_backward_requires_loop() {
    // One-way edge 1->2 with a distant return loop 2->3->4->1.
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_node(3, 100.0, -100.0).unwrap();
    net.add_node(4, -10.0, -100.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    net.add_edge(2, 2, 3).unwrap();
    net.add_edge(3, 3, 4).unwrap();
    net.add_edge(4, 4, 1).unwrap();
    let mut c = cfg();
    c.max_speed = 1000.0; // allow the long loop
    let m = Matcher::new(&net, c).unwrap();
    let trace = vec![obs(80.0, 0.5, 0.0), obs(20.0, 0.5, 1.0)];
    let out = m.match_trace(&trace).unwrap();
    assert_eq!(out.segments.len(), 1);
    let tr = &out.segments[0].transitions[0];
    // remaining 20 + loop (100 + 110 + ~100.5) + 20 along
    assert_eq!(tr.path, vec![1, 2, 3, 4, 1]);
    let expected = 20.0 + 100.0 + 110.0 + (10.0f64.hypot(100.0)) + 20.0;
    assert!((tr.route_distance - expected).abs() < 1e-9, "got {} want {expected}", tr.route_distance);
}

#[test]
fn speed_cap_blocks_transition() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 100.0, 0.0).unwrap();
    net.add_edge(1, 1, 2).unwrap();
    let mut c = cfg();
    c.max_speed = 10.0;
    let m = Matcher::new(&net, c).unwrap();
    // 60m in 1s at 10 m/s cap: unreachable, segment restarts.
    let trace = vec![obs(10.0, 0.5, 0.0), obs(70.0, 0.5, 1.0)];
    let out = m.match_trace(&trace).unwrap();
    assert_eq!(out.segments.len(), 2);
}

#[test]
fn shortest_path_tie_breaks_by_edge_ids() {
    let mut net = RoadNetwork::new();
    net.add_node(1, 0.0, 0.0).unwrap();
    net.add_node(2, 10.0, 0.0).unwrap();
    net.add_node(3, 0.0, 10.0).unwrap();
    net.add_node(4, 10.0, 10.0).unwrap();
    net.add_edge(5, 1, 2).unwrap();
    net.add_edge(6, 2, 4).unwrap();
    net.add_edge(2, 1, 3).unwrap();
    net.add_edge(3, 3, 4).unwrap();
    let (d, path) = shortest_path(&net, 1, 4).unwrap();
    assert!((d - 20.0).abs() < 1e-9);
    assert_eq!(path, vec![2, 3]); // lexicographically smaller than [5, 6]
}
