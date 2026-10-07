use crate::candidates::{candidates_for, Candidate};
use crate::network::{MatchError, RoadNetwork, MAX_OBSERVATIONS};
use crate::shortest::shortest_path;
use crate::viterbi::{self, Transition};
use serde::Serialize;

/// One noisy localization record. `t` is seconds and must strictly increase.
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

/// Matching parameters. All distances are metres, times seconds.
#[derive(Debug, Clone, Copy)]
pub struct MatcherConfig {
    /// GPS noise standard deviation (m), must be > 0.
    pub sigma: f64,
    /// Transition scale (m), must be > 0.
    pub beta: f64,
    /// Speed limit (m/s), must be > 0.
    pub max_speed: f64,
    /// Candidate search radius (m), must be > 0.
    pub radius: f64,
    /// Number of candidates per observation, 1..=8.
    pub k: usize,
}

impl MatcherConfig {
    fn validate(&self) -> Result<(), MatchError> {
        for (name, v) in [
            ("sigma", self.sigma),
            ("beta", self.beta),
            ("max_speed", self.max_speed),
            ("radius", self.radius),
        ] {
            if !v.is_finite() {
                return Err(MatchError::NonFiniteValue(name));
            }
            if v <= 0.0 {
                return Err(MatchError::InvalidParameter(name));
            }
        }
        if !(1..=8).contains(&self.k) {
            return Err(MatchError::InvalidParameter("k must be in 1..=8"));
        }
        Ok(())
    }
}

/// A matched observation.
#[derive(Debug, Clone, Serialize)]
pub struct MatchedPoint {
    pub edge_id: u64,
    pub along: f64,
    pub x: f64,
    pub y: f64,
    pub emission_cost: f64,
}

/// A transition chosen between two consecutive matched observations.
#[derive(Debug, Clone, Serialize)]
pub struct TransitionResult {
    pub from_index: usize,
    pub to_index: usize,
    /// Full directed path as edge ids (includes both endpoint edges).
    pub path: Vec<u64>,
    pub route_distance: f64,
    pub transition_cost: f64,
}

/// One continuous matched segment.
#[derive(Debug, Clone, Serialize)]
pub struct SegmentResult {
    pub start_index: usize,
    pub end_index: usize,
    pub total_cost: f64,
    pub transitions: Vec<TransitionResult>,
}

/// Per-observation outcome.
#[derive(Debug, Clone, Serialize)]
pub struct ObservationResult {
    pub index: usize,
    pub matched: Option<MatchedPoint>,
    pub unmatched_reason: Option<String>,
}

/// Full matching output.
#[derive(Debug, Clone, Serialize)]
pub struct MatchOutput {
    pub observations: Vec<ObservationResult>,
    pub segments: Vec<SegmentResult>,
}

/// Map matcher over a road network.
pub struct Matcher<'a> {
    net: &'a RoadNetwork,
    config: MatcherConfig,
}

impl<'a> Matcher<'a> {
    pub fn new(net: &'a RoadNetwork, config: MatcherConfig) -> Result<Self, MatchError> {
        config.validate()?;
        Ok(Matcher { net, config })
    }

    fn emission_cost(&self, cand: &Candidate) -> f64 {
        let s = self.config.sigma;
        cand.distance * cand.distance / (2.0 * s * s)
    }

    /// Route distance and edge id path between two candidates, honouring
    /// one-way directions, loops for same-edge reversal, and the speed cap.
    fn transition(&self, from: &Candidate, to: &Candidate, dt: f64) -> Option<Transition> {
        let ea = self.net.edge(from.edge_id)?;
        let eb = self.net.edge(to.edge_id)?;
        let (route_distance, path) = if from.edge_id == to.edge_id && to.along >= from.along {
            (to.along - from.along, vec![from.edge_id])
        } else {
            // General case (including same-edge reversal, which must travel a
            // legal directed loop from the edge end back to its start).
            let (mid_d, mid_p) = shortest_path(self.net, ea.to, eb.from)?;
            let mut p = vec![from.edge_id];
            p.extend(mid_p);
            p.push(to.edge_id);
            (ea.length - from.along + mid_d + to.along, p)
        };
        if route_distance > self.config.max_speed * dt {
            return None;
        }
        Some(Transition {
            route_distance,
            cost: route_distance / self.config.beta,
            path,
        })
    }

    pub fn match_trace(&self, obs: &[Observation]) -> Result<MatchOutput, MatchError> {
        if obs.is_empty() {
            return Err(MatchError::EmptyTrace);
        }
        if obs.len() > MAX_OBSERVATIONS {
            return Err(MatchError::TooManyObservations);
        }
        for (i, o) in obs.iter().enumerate() {
            if !o.x.is_finite() || !o.y.is_finite() || !o.t.is_finite() {
                return Err(MatchError::NonFiniteValue("observation"));
            }
            if i > 0 && o.t <= obs[i - 1].t {
                return Err(MatchError::NonIncreasingTimestamps { index: i });
            }
        }

        let cfg = self.config;
        let cands: Vec<Vec<Candidate>> = obs
            .iter()
            .map(|o| candidates_for(self.net, o.x, o.y, cfg.radius, cfg.k))
            .collect();

        // Split into continuous segments: break at unmatched observations and
        // where every transition between consecutive points is unreachable.
        let mut segments: Vec<Vec<usize>> = Vec::new();
        let mut current: Vec<usize> = Vec::new();
        for i in 0..obs.len() {
            if cands[i].is_empty() {
                if !current.is_empty() {
                    segments.push(std::mem::take(&mut current));
                }
                continue;
            }
            if let Some(&prev) = current.last() {
                debug_assert_eq!(prev + 1, i);
                let dt = obs[i].t - obs[prev].t;
                let any = cands[prev].iter().any(|a| {
                    cands[i].iter().any(|b| self.transition(a, b, dt).is_some())
                });
                if !any {
                    segments.push(std::mem::take(&mut current));
                }
            }
            current.push(i);
        }
        if !current.is_empty() {
            segments.push(current);
        }

        let mut results: Vec<ObservationResult> = (0..obs.len())
            .map(|i| ObservationResult {
                index: i,
                matched: None,
                unmatched_reason: if cands[i].is_empty() {
                    Some("no candidate within radius".to_string())
                } else {
                    None
                },
            })
            .collect();
        let mut segment_results = Vec::new();

        for seg in segments {
            let seg_cands: Vec<Vec<Candidate>> = seg.iter().map(|&i| cands[i].clone()).collect();
            let emissions: Vec<Vec<f64>> = seg_cands
                .iter()
                .map(|cs| cs.iter().map(|c| self.emission_cost(c)).collect())
                .collect();
            let mut arcs: Vec<Vec<Vec<Option<Transition>>>> = Vec::new();
            for w in 0..seg.len() - 1 {
                let dt = obs[seg[w + 1]].t - obs[seg[w]].t;
                let row: Vec<Vec<Option<Transition>>> = seg_cands[w]
                    .iter()
                    .map(|a| seg_cands[w + 1].iter().map(|b| self.transition(a, b, dt)).collect())
                    .collect();
                arcs.push(row);
            }
            let Some((chosen, total)) = viterbi::viterbi(&seg_cands, &emissions, &arcs) else {
                continue;
            };
            let mut transitions = Vec::new();
            for w in 1..chosen.len() {
                let arc = arcs[w - 1][chosen[w - 1]][chosen[w]]
                    .as_ref()
                    .expect("chosen arc exists");
                transitions.push(TransitionResult {
                    from_index: seg[w - 1],
                    to_index: seg[w],
                    path: arc.path.clone(),
                    route_distance: arc.route_distance,
                    transition_cost: arc.cost,
                });
            }
            for (w, &ci) in chosen.iter().enumerate() {
                let cand = &seg_cands[w][ci];
                results[seg[w]].matched = Some(MatchedPoint {
                    edge_id: cand.edge_id,
                    along: cand.along,
                    x: cand.x,
                    y: cand.y,
                    emission_cost: emissions[w][ci],
                });
            }
            segment_results.push(SegmentResult {
                start_index: seg[0],
                end_index: seg[seg.len() - 1],
                total_cost: total,
                transitions,
            });
        }

        Ok(MatchOutput { observations: results, segments: segment_results })
    }
}
