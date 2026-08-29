//! Regenerate `data/prop_coefficients.csv` from the model's own basis.
//!
//! The table is derived, not measured: every row is this crate's closed-form
//! coefficient model evaluated at that row's (diameter, pitch, blade count).
//! Keeping the generator in the tree makes that explicit and makes a
//! coefficient change reproducible instead of a hand-edit across 50 rows.
//!
//!   cargo run --example regenerate_prop_table > /tmp/prop.csv \
//!     && mv /tmp/prop.csv data/prop_coefficients.csv
//!
//! NOT `> data/prop_coefficients.csv` directly: the shell truncates the target
//! before this runs, and the row list comes from that same file.
use hitl_physics::build::PropellerSpec;
use hitl_physics::prop_coefficients;

fn main() {
    // NOTE: this reads the very file it is normally redirected into. A shell
    // `>` truncates the target BEFORE the program runs, so if the build is
    // stale or the run fails midway the table is destroyed and the next run
    // has no rows to emit. Generate to a temporary file and move it into
    // place, and treat an empty input as the error it is rather than
    // cheerfully emitting a header and nothing else.
    let existing = include_str!("../data/prop_coefficients.csv");
    let row_count = existing
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("diameter_in") && !l.trim().is_empty())
        .count();
    assert!(
        row_count > 0,
        "data/prop_coefficients.csv has no rows — it was probably truncated by redirecting \
         this generator's output straight into it. Restore it from git, then generate to a \
         temporary file and move it into place."
    );
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
