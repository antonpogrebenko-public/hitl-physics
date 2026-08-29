//! Regenerate `data/prop_coefficients.csv` from the model's own basis.
//!
//! The table is derived, not measured: every row is this crate's closed-form
//! coefficient model evaluated at that row's (diameter, pitch, blade count).
//! Keeping the generator in the tree makes that explicit and makes a
//! coefficient change reproducible instead of a hand-edit across 50 rows.
//!
//!   cargo run --example regenerate_prop_table > data/prop_coefficients.csv
use hitl_physics::build::PropellerSpec;
use hitl_physics::prop_coefficients;

fn main() {
    let existing = include_str!("../data/prop_coefficients.csv");
    for line in existing.lines() {
        if line.starts_with('#') || line.starts_with("diameter_in") {
            println!("{line}");
            continue;
        }
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 8 {
            continue;
        }
        let spec = PropellerSpec {
            diameter_in: f[0].parse().expect("diameter"),
            pitch_in: f[1].parse().expect("pitch"),
            blade_count: f[2].parse().expect("blades"),
            weight_g: None,
        };
        let c = prop_coefficients::derive_from_basis(&spec);
        println!(
            "{},{},{},{},{},{:.6e},{:.6e},{}",
            f[0], f[1], f[2], f[3], f[4], c.ct, c.cq, f[7]
        );
    }
}
