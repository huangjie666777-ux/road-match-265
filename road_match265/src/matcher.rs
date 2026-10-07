use crate::candidate::{candidates_for, Candidate};
use crate::graph::{shortest_path, Graph};
use crate::model::*;

const NO_PREV: usize = usize::MAX;

#[derive(Debug, Clone)]
struct State {
    cost: f64,
    /// edge id sequence chosen so far (global tie-break key)
    seq: Vec<u64>,
    /// index of the previous candidate at the previous matched point,
    /// or NO_PREV when this state starts a new segment
    prev: usize,
}

/// Directed route between two candidates on the network.
/// Returns (path edge ids, route distance in meters) if a route exists.
fn route_between(graph: &Graph, a: &Candidate, b: &Candidate) -> Option<(Vec<u64>, f64)> {
    let e1 = &graph.edges[a.edge];
    let e2 = &graph.edges[b.edge];
    if a.edge == b.edge && b.along >= a.along {
        // forward along the same edge: direct
        return Some((vec![e1.id], b.along - a.along));
    }
    // remaining part of e1 + shortest path e1.to -> e2.from + initial part of e2
    let (mid_dist, mid_path) = shortest_path(graph, e1.to, e2.from)?;
    let mut path = Vec::with_capacity(mid_path.len() + 2);
    path.push(e1.id);
    path.extend(mid_path);
    path.push(e2.id);
    Some((path, (e1.length - a.along) + mid_dist + b.along))
}

fn emission_cost(perp: f64, sigma: f64) -> f64 {
    perp * perp / (2.0 * sigma * sigma)
}

fn better(cost_a: f64, seq_a: &[u64], cost_b: f64, seq_b: &[u64]) -> bool {
    cost_a.total_cmp(&cost_b).is_lt()
        || (cost_a == cost_b && seq_a < seq_b)
}

pub fn match_input(input: &Input) -> Result<Output, MatchError> {
    let graph = crate::graph::validate_and_build(input)?;
    let p = &input.parameters;
    let n = input.observations.len();

    // per-point candidates; empty means unmatched
    let cand_sets: Vec<Vec<Candidate>> = input
        .observations
        .iter()
        .map(|o| candidates_for(&graph, o.x, o.y, p.radius, p.k))
        .collect();

    // dp[i] : states for point i (empty if point i unmatched)
    let mut dp: Vec<Vec<State>> = Vec::with_capacity(n);
    for i in 0..n {
        let obs = &input.observations[i];
        let cands = &cand_sets[i];
        let mut states: Vec<State> = Vec::with_capacity(cands.len());
        for c in cands {
            let em = emission_cost(c.perp, p.sigma);
            let edge_id = graph.edges[c.edge].id;
            let mut best: Option<State> = None;
            if i > 0 && !cand_sets[i - 1].is_empty() {
                let prev_obs = &input.observations[i - 1];
                let dt = obs.t - prev_obs.t;
                let straight = (obs.x - prev_obs.x).hypot(obs.y - prev_obs.y);
                for (pi, ps) in dp[i - 1].iter().enumerate() {
                    let pc = &cand_sets[i - 1][pi];
                    let Some((_path, dist)) = route_between(&graph, pc, c) else {
                        continue;
                    };
                    if dist > p.max_speed * dt {
                        continue;
                    }
                    let tcost = (dist - straight).abs() / p.beta;
                    let mut seq = ps.seq.clone();
                    seq.push(edge_id);
                    let cand_state = State { cost: ps.cost + tcost + em, seq, prev: pi };
                    let replace = match &best {
                        None => true,
                        Some(b) => better(cand_state.cost, &cand_state.seq, b.cost, &b.seq),
                    };
                    if replace {
                        best = Some(cand_state);
                    }
                }
            }
            let state = best.unwrap_or(State {
                cost: em,
                seq: vec![edge_id],
                prev: NO_PREV,
            });
            states.push(state);
        }
        dp.push(states);
    }

    // best state per point (segments are independent, so the global
    // optimum is the concatenation of per-segment optima)
    let best_state: Vec<Option<usize>> = (0..n)
        .map(|i| {
            let mut best: Option<usize> = None;
            for (ci, s) in dp[i].iter().enumerate() {
                let replace = match best {
                    None => true,
                    Some(bi) => {
                        let b = &dp[i][bi];
                        better(s.cost, &s.seq, b.cost, &b.seq)
                    }
                };
                if replace {
                    best = Some(ci);
                }
            }
            best
        })
        .collect();
    let any_match = best_state.iter().any(|b| b.is_some());

    let mut matches: Vec<MatchPoint> = Vec::new();
    let mut unmatched: Vec<UnmatchedPoint> = Vec::new();
    let mut segments: Vec<Segment> = Vec::new();
    let mut total_cost = 0.0;

    if any_match {
        // backtrack chosen candidate per matched point, segment by
        // segment from the last matched point backwards
        let mut chosen: Vec<Option<usize>> = vec![None; n];
        let mut i = (0..n).rev().find(|&i| best_state[i].is_some()).unwrap();
        loop {
            let mut ci = best_state[i].unwrap();
            loop {
                chosen[i] = Some(ci);
                let prev = dp[i][ci].prev;
                if prev == NO_PREV {
                    break;
                }
                i -= 1;
                ci = prev;
            }
            // i is now the segment start; jump to the previous matched
            // point before it, if any
            match (0..i).rev().find(|&j| best_state[j].is_some()) {
                Some(j) => i = j,
                None => break,
            }
        }

        // assemble matches, segments, transitions
        let mut seg_id = 0usize;
        let mut cur: Option<Segment> = None;
        for i in 0..n {
            match chosen[i] {
                None => {
                    unmatched.push(UnmatchedPoint {
                        index: i,
                        reason: format!(
                            "no candidate edge within radius {} m",
                            p.radius
                        ),
                    });
                    if let Some(s) = cur.take() {
                        segments.push(s);
                    }
                }
                Some(ci) => {
                    let c = &cand_sets[i][ci];
                    let edge = &graph.edges[c.edge];
                    let new_segment = dp[i][ci].prev == NO_PREV;
                    if new_segment {
                        if let Some(s) = cur.take() {
                            segments.push(s);
                        }
                        cur = Some(Segment {
                            id: seg_id,
                            cost: 0.0,
                            observations: Vec::new(),
                            transitions: Vec::new(),
                        });
                        seg_id += 1;
                    }
                    let seg = cur.as_mut().expect("segment open");
                    seg.cost += emission_cost(c.perp, p.sigma);
                    if !new_segment {
                        let pi = i - 1;
                        let pci = chosen[pi].expect("previous point matched");
                        let pc = &cand_sets[pi][pci];
                        let prev_obs = &input.observations[pi];
                        let obs = &input.observations[i];
                        let (path, dist) =
                            route_between(&graph, pc, c).expect("chosen transition routed");
                        let straight =
                            (obs.x - prev_obs.x).hypot(obs.y - prev_obs.y);
                        let tcost = (dist - straight).abs() / p.beta;
                        seg.cost += tcost;
                        seg.transitions.push(Transition {
                            from_index: pi,
                            to_index: i,
                            path,
                            distance: dist,
                            cost: tcost,
                        });
                    }
                    seg.observations.push(i);
                    matches.push(MatchPoint {
                        index: i,
                        segment: seg.id,
                        edge_id: edge.id,
                        x: c.x,
                        y: c.y,
                        along: c.along,
                    });
                }
            }
        }
        if let Some(s) = cur.take() {
            segments.push(s);
        }
        total_cost = segments.iter().map(|s| s.cost).sum();
    } else {
        for i in 0..n {
            unmatched.push(UnmatchedPoint {
                index: i,
                reason: format!("no candidate edge within radius {} m", p.radius),
            });
        }
    }

    Ok(Output { matches, unmatched, segments, total_cost })
}
