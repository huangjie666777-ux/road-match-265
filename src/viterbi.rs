use crate::candidates::Candidate;

/// A transition arc between candidates of consecutive observations.
#[derive(Debug, Clone)]
pub struct Transition {
    pub route_distance: f64,
    pub cost: f64,
    pub path: Vec<u64>,
}

/// Runs the Viterbi dynamic program over one continuous segment.
///
/// `emissions[i][c]` is the emission cost of candidate c of observation i.
/// `transitions[i][a][b]` is the arc from candidate a of observation i to
/// candidate b of observation i+1 (`None` when unreachable).
///
/// Among equal-cost solutions the lexicographically smallest candidate edge
/// id sequence wins.
/// Returns the chosen candidate index per observation and the total cost.
pub fn viterbi(
    candidates: &[Vec<Candidate>],
    emissions: &[Vec<f64>],
    transitions: &[Vec<Vec<Option<Transition>>>],
) -> Option<(Vec<usize>, f64)> {
    if candidates.is_empty() {
        return None;
    }
    // dp[c] = (cost, edge id sequence, chosen candidate indices)
    let mut dp: Vec<Option<(f64, Vec<u64>, Vec<usize>)>> = candidates[0]
        .iter()
        .enumerate()
        .map(|(c, cand)| Some((emissions[0][c], vec![cand.edge_id], vec![c])))
        .collect();
    for i in 1..candidates.len() {
        let mut next: Vec<Option<(f64, Vec<u64>, Vec<usize>)>> = vec![None; candidates[i].len()];
        for (b, cand) in candidates[i].iter().enumerate() {
            let mut best: Option<(f64, Vec<u64>, Vec<usize>)> = None;
            for (a, prev) in dp.iter().enumerate() {
                let Some((pc, pseq, pidx)) = prev else { continue };
                let Some(arc) = &transitions[i - 1][a][b] else { continue };
                let cost = pc + arc.cost + emissions[i][b];
                let mut seq = pseq.clone();
                seq.push(cand.edge_id);
                let mut idx = pidx.clone();
                idx.push(b);
                let replace = match &best {
                    None => true,
                    Some((bc, bseq, _)) => cost < *bc || (cost == *bc && seq < *bseq),
                };
                if replace {
                    best = Some((cost, seq, idx));
                }
            }
            next[b] = best;
        }
        dp = next;
    }
    let (total, _, indices) = dp
        .into_iter()
        .flatten()
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.cmp(&b.1)))?;
    Some((indices, total))
}
