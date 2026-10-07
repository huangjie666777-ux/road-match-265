# road_match265

Map matching of noisy GPS fixes to a directed metric road network with a
Viterbi hidden Markov model. Pure Rust library (Rust 1.85.1, serde
1.0.219, serde_json 1.0.140); no frontend, no HTTP, no training.

## Units

- Coordinates, distances, radius, sigma, beta: **meters**
- Timestamps: **seconds**, strictly increasing
- max_speed: **meters per second**

## Input format (JSON)

See `examples/parallel.json` and `examples/gap.json`.

- `road_network.nodes`: `{id, x, y}` with unique ids, metric coordinates.
- `road_network.edges`: `{id, from, to}` directed straight edges; length
  is the endpoint distance. Two-way roads need two edges. Geometric
  crossings do not connect edges.
- `parameters`: `radius` (>0), `k` (1..=8), `sigma` (>0), `beta` (>0),
  `max_speed` (>0).
- `observations`: `{x, y, t}`, at most 100, strictly increasing `t`.

Limits: 200 edges, 100 observations. Duplicate ids, unknown nodes,
zero-length edges, non-finite values and illegal parameters are rejected.

## Model

- Each observation is projected onto every edge; the nearest `k` edges
  within `radius` are candidates (ties by ascending edge id).
- Emission cost: `perp^2 / (2 * sigma^2)`.
- Transition: forward on the same edge is direct; otherwise remaining
  part of the first edge + directed shortest path between the edges +
  initial part of the second edge. Moving backward on the same edge also
  requires a legal loop. A transition is infeasible when no directed
  route exists or the route exceeds `max_speed * dt`. Straight-line
  distance is never used as a substitute for route distance.
- Transition cost: `|route_distance - straight_distance| / beta`.
- Viterbi finds the global minimum-cost sequence per connected segment;
  equal-cost solutions are chosen by the lexicographically smallest
  candidate edge id sequence, and equal-length shortest paths by the
  lexicographically smallest path edge id sequence.
- Points without candidates are reported unmatched and break the
  segment; when every transition is infeasible the old segment ends and
  a new one starts at the current point.

## Output format (JSON)

- `matches`: `{index, segment, edge_id, x, y, along}` - original
  observation index, projected point, edge id, distance along the edge.
- `unmatched`: `{index, reason}`.
- `segments`: `{id, cost, observations, transitions}` where each
  transition carries the full path as an edge id sequence and its route
  distance in meters.
- `total_cost`: global minimal cost.

## Build, test, demo

```sh
cargo build
cargo test
cargo run --example demo   # parallel-road ambiguity + broken-road demo
```

## Library usage

```rust
let result_json = road_match265::match_json(&input_json)?;
// or work with typed values:
let input: road_match265::Input = serde_json::from_str(&input_json)?;
let output = road_match265::match_input(&input)?;
```
