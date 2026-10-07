//! Demonstrates map matching on two scenarios:
//! 1. parallel.json - parallel road ambiguity resolved globally
//! 2. gap.json      - broken network forcing a segment restart
//!
//! Run with: cargo run --example demo

fn main() {
    for name in ["parallel", "gap"] {
        let path = format!("{}/examples/{name}.json", env!("CARGO_MANIFEST_DIR"));
        let input = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
        println!("=== scenario: {name} ===");
        match road_match265::match_json(&input) {
            Ok(out) => println!("{out}"),
            Err(e) => println!("rejected: {e}"),
        }
        println!();
    }
}
