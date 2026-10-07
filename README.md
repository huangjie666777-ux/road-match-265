# road_match265

Map matching library: reconstructs the roads a vehicle travelled from noisy
localization records. Pure Rust library (Rust 1.85.1, serde 1.0.219,
serde_json 1.0.140) — no frontend, no HTTP, no training.

## Units and model

- Coordinates are metric (metres); timestamps are seconds, strictly increasing.
- The road network consists of nodes with unique ids and directed straight
  edges with unique ids. Edge length is the endpoint distance.
- Two-way roads need two opposite edges. Geometric crossings do **not**
  connect edges — connectivity comes only from shared node ids.
- Limits: at most 200 edges per network and 100 observations per trace.
- Rejected input: duplicate node/edge ids, edges referencing unknown nodes,
  zero-length edges, non-finite values, empty traces, non-increasing
  timestamps, and invalid parameters (`sigma`, `beta`, `max_speed`, `radius`
  must be positive; `k` must be in `1..=8`).

## Matching algorithm

1. **Candidates** (`src/candidates.rs`): each observation is projected onto
   every edge; the nearest `k` projections within `radius` are kept, ties
   broken by ascending edge id.
2. **Emission cost**: `d^2 / (2 * sigma^2)` where `d` is the perpendicular
   distance to the edge.
3. **Transition cost** (`src/shortest.rs`): the route distance between two
   candidates is the remaining part of the from-edge, plus the directed
   shortest path from the from-edge end node to the to-edge start node, plus
   the starting part of the to-edge. Forward movement on the same edge is
   direct; backward movement on the same edge must travel a legal directed
   loop. Equal-length shortest paths are tie-broken by the lexicographically
   smallest edge id sequence. The transition cost is
   `route_distance / beta`. A transition is impossible when no directed route
   exists or the route exceeds `max_speed * dt` — straight-line distance is
   never used as a substitute.
4. **Viterbi** (`src/viterbi.rs`): each continuous segment is solved globally
   for the minimum total cost (never per-point nearest edge). Equal-cost
   solutions are tie-broken by the lexicographically smallest candidate edge
   id sequence.
5. **Segmentation & assembly** (`src/matcher.rs`): observations without any
   candidate are marked unmatched (reason kept) and break the segment; when
   every transition between two consecutive points is unreachable, the old
   segment ends and a new one starts at the current point.

## Output

`MatchOutput` (serde-serializable) contains, per observation: original index,
matched edge id, projected coordinates, distance along the edge, emission
cost, or the unmatched reason; and per segment: index range, total cost, and
every transition with its full edge id path and route distance in metres.
See `examples/demo_output.json` for a real sample.

## Build, test, demo

```sh
cargo build          # builds the road_match265 library
cargo test           # runs the self-tests in tests/matching.rs
cargo run --example demo   # prints JSON: parallel-road ambiguity + broken road
```

## Usage sketch

```rust
use road_match265::{Matcher, MatcherConfig, Observation, RoadNetwork};

let mut net = RoadNetwork::new();
net.add_node(1, 0.0, 0.0)?;
net.add_node(2, 100.0, 0.0)?;
net.add_edge(1, 1, 2)?; // directed edge, length = 100 m

let matcher = Matcher::new(&net, MatcherConfig {
    sigma: 5.0, beta: 10.0, max_speed: 30.0, radius: 20.0, k: 4,
})?;
let trace = vec![
    Observation { x: 10.0, y: 1.0, t: 0.0 },
    Observation { x: 30.0, y: 1.2, t: 1.0 },
];
let output = matcher.match_trace(&trace)?;
let json = serde_json::to_string_pretty(&output)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```
