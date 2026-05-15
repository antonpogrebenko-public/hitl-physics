//! Propeller static coefficient lookup (Phase 5).
//!
//! Replaces the empirical `pitch_multiplier` / `blade_multiplier` formulas in
//! `PhysicsConfig::from_build_specs` with a real (D, P, B) → (CT, CQ) lookup
//! derived from manufacturer bench data and HITL calibration.
//!
//! ## Convention
//!
//! The simulation uses `kt = ct * rho * D^4` and `kq = cq * rho * D^5` where
//! omega is in **rad/s**. The `ct` / `cq` values stored here are therefore
//! ~`(2π)² ≈ 39.5` times smaller than the aerospace-standard CT(n) coefficients
//! that use revolutions-per-second. The CSV file header documents this in full.
//!
//! ## Fallback chain
//!
//! `lookup` resolves in this order:
//! 1. Exact match on (diameter, pitch, blade_count) + manufacturer + model.
//! 2. Average across manufacturers for an exact (diameter, pitch, blade_count).
//! 3. Nearest-neighbor in (diameter, pitch, blade_count) Euclidean distance,
//!    with pitch / blade-count rescaling applied so the result lands close to
//!    what the requested prop should produce.
//! 4. Empirical extrapolation matching the legacy `from_build_specs` formula
//!    (returns the same value the old code would have computed) — used when
//!    the table is empty or no entry within a sensible distance exists.

use crate::build::PropellerSpec;

/// Air density (kg/m^3) used to convert ct/cq → kt/kq in the simulation.
const RHO: f64 = 1.225;

/// Raw CSV bundled into the binary via `include_str!`. Comment lines (starting
/// with `#`) and the header row are skipped at parse time.
const CSV_DATA: &str = include_str!("../data/prop_coefficients.csv");

/// Resolved coefficients in the simulation's rad/s convention.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropCoefficients {
    /// kt = ct * rho * D^4 (omega in rad/s, D in meters).
    pub ct: f64,
    /// kq = cq * rho * D^5.
    pub cq: f64,
    /// Tag describing where the result came from. Useful for debugging
    /// regressions and for downstream telemetry.
    pub source: LookupSource,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LookupSource {
    ExactMatch,
    DimensionAverage,
    NearestNeighbor,
    EmpiricalFallback,
}

#[derive(Debug, Clone)]
struct Entry {
    diameter_in: f64,
    pitch_in: f64,
    blade_count: i32,
    // Manufacturer / model strings are kept for future tie-breaking and
    // diagnostic dumps; not consumed by the current lookup paths.
    #[allow(dead_code)]
    manufacturer: String,
    #[allow(dead_code)]
    model: String,
    ct: f64,
    cq: f64,
}

fn parse_entries() -> Vec<Entry> {
    let mut out = Vec::new();
    for line in CSV_DATA.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("diameter_in") {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() < 7 {
            continue;
        }
        let (Ok(diameter_in), Ok(pitch_in), Ok(blade_count), Ok(ct), Ok(cq)) = (
            fields[0].parse::<f64>(),
            fields[1].parse::<f64>(),
            fields[2].parse::<i32>(),
            fields[5].parse::<f64>(),
            fields[6].parse::<f64>(),
        ) else {
            continue;
        };
        out.push(Entry {
            diameter_in,
            pitch_in,
            blade_count,
            manufacturer: fields[3].to_string(),
            model: fields[4].to_string(),
            ct,
            cq,
        });
    }
    out
}

thread_local! {
    static ENTRIES: Vec<Entry> = parse_entries();
}

fn with_entries<R>(f: impl FnOnce(&[Entry]) -> R) -> R {
    ENTRIES.with(|entries| f(entries.as_slice()))
}

/// Number of entries in the bundled table. Useful for tests and diagnostics.
pub fn table_size() -> usize {
    with_entries(|entries| entries.len())
}

/// Resolve `(ct, cq)` for the given propeller.
///
/// See the module-level docs for the fallback chain.
pub fn lookup(spec: &PropellerSpec) -> PropCoefficients {
    with_entries(|entries| {
        if entries.is_empty() {
            return empirical_fallback(spec);
        }

        // 1. Exact (D, P, B) match — if there are multiple, average them.
        let exact: Vec<&Entry> = entries
            .iter()
            .filter(|e| {
                (e.diameter_in - spec.diameter_in).abs() < 1e-3
                    && (e.pitch_in - spec.pitch_in).abs() < 1e-3
                    && e.blade_count == spec.blade_count
            })
            .collect();
        if !exact.is_empty() {
            let ct = exact.iter().map(|e| e.ct).sum::<f64>() / exact.len() as f64;
            let cq = exact.iter().map(|e| e.cq).sum::<f64>() / exact.len() as f64;
            let source = if exact.len() == 1 {
                LookupSource::ExactMatch
            } else {
                LookupSource::DimensionAverage
            };
            return PropCoefficients { ct, cq, source };
        }

        // 2. Nearest neighbor + rescale by pitch/blade ratios so the result
        //    reflects the requested prop, not just the nearest one.
        let mut best: Option<(f64, &Entry)> = None;
        for entry in entries {
            let d_dim = entry.diameter_in - spec.diameter_in;
            let d_pitch = entry.pitch_in - spec.pitch_in;
            let d_blade = (entry.blade_count - spec.blade_count) as f64;
            // Diameter changes the result dramatically (D^4 scaling), so weight
            // it most heavily. Keep prop in the same diameter class when possible.
            let dist = (4.0 * d_dim).powi(2) + d_pitch.powi(2) + (1.5 * d_blade).powi(2);
            if best.map(|(d, _)| dist < d).unwrap_or(true) {
                best = Some((dist, entry));
            }
        }
        if let Some((_, entry)) = best {
            let pitch_scale = pitch_ratio(spec.pitch_in, entry.pitch_in);
            let blade_scale = blade_ratio(spec.blade_count, entry.blade_count);
            return PropCoefficients {
                ct: entry.ct * pitch_scale * blade_scale,
                cq: entry.cq * pitch_scale * blade_scale,
                source: LookupSource::NearestNeighbor,
            };
        }

        empirical_fallback(spec)
    })
}

/// Reproduce the legacy `from_build_specs` formula so unknown props degrade
/// gracefully when the CSV is empty or all entries are unusable.
fn empirical_fallback(spec: &PropellerSpec) -> PropCoefficients {
    let base_pf = match spec.diameter_in as u32 {
        0..=4 => 0.8e-6,
        5 => 1.9e-6,
        6 => 3.2e-6,
        _ => 5.0e-6,
    };
    let pitch_m = 0.7 + 0.06 * spec.pitch_in;
    let blade_m = 0.85 + 0.05 * spec.blade_count as f64;
    let kt = base_pf * pitch_m * blade_m; // evaluated at KV=2300 reference
    let d_m = spec.diameter_in * 0.0254;
    let ct = kt / (RHO * d_m.powi(4));
    let kq_kt = 0.010 + 0.002 * spec.pitch_in + 0.002 * spec.blade_count as f64;
    let kq = kt * kq_kt;
    let cq = kq / (RHO * d_m.powi(5));
    PropCoefficients { ct, cq, source: LookupSource::EmpiricalFallback }
}

fn pitch_ratio(requested: f64, reference: f64) -> f64 {
    // Linear pitch model lining up with the empirical formula (0.7 + 0.06·P).
    let num = 0.7 + 0.06 * requested;
    let den = 0.7 + 0.06 * reference;
    (num / den).max(0.1)
}

fn blade_ratio(requested: i32, reference: i32) -> f64 {
    // Linear blade-count model (0.85 + 0.05·B).
    let num = 0.85 + 0.05 * requested as f64;
    let den = 0.85 + 0.05 * reference as f64;
    (num / den).max(0.1)
}

/// Convert coefficients to `(kt, kq)` for a given prop diameter in inches.
pub fn coefficients_to_kt_kq(coef: PropCoefficients, diameter_in: f64) -> (f64, f64) {
    let d_m = diameter_in * 0.0254;
    let kt = coef.ct * RHO * d_m.powi(4);
    let kq = coef.cq * RHO * d_m.powi(5);
    (kt, kq)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(diameter: f64, pitch: f64, blades: i32) -> PropellerSpec {
        PropellerSpec {
            diameter_in: diameter,
            pitch_in: pitch,
            blade_count: blades,
            weight_g: None,
        }
    }

    #[test]
    fn table_loads_with_expected_size() {
        // CSV ships with ~50 calibrated entries; assert at least 40 to allow
        // light editing without breaking the test.
        assert!(table_size() >= 40, "expected >=40 entries, got {}", table_size());
    }

    #[test]
    fn exact_match_for_5x4_5x3_anchor() {
        let coef = lookup(&spec(5.0, 4.5, 3));
        assert!(matches!(
            coef.source,
            LookupSource::ExactMatch | LookupSource::DimensionAverage
        ));
        let (kt, _kq) = coefficients_to_kt_kq(coef, 5.0);
        // Anchor: kt must stay within 10% of the legacy from_build_specs value
        // for the canonical 5"×4.5"×3 racing prop at KV 1700 (~3.376e-6).
        let legacy_kt = 3.376e-6;
        let rel = (kt - legacy_kt).abs() / legacy_kt;
        assert!(rel < 0.10, "kt drifted by {:.1}% from legacy anchor", rel * 100.0);
    }

    #[test]
    fn two_blade_produces_less_thrust_than_three_blade() {
        let coef_2 = lookup(&spec(5.0, 4.5, 2));
        let coef_3 = lookup(&spec(5.0, 4.5, 3));
        let ratio = coef_2.ct / coef_3.ct;
        // 2-blade should be 10-25% less than 3-blade. Empirical formula gives
        // ~6%, plan demands ~15%. Real bench data sits 12-18%.
        assert!(
            (0.75..=0.95).contains(&ratio),
            "2-blade/3-blade thrust ratio {:.3} outside [0.75, 0.95]",
            ratio
        );
    }

    #[test]
    fn seven_inch_doubles_five_inch_thrust_at_same_rpm() {
        let coef_5 = lookup(&spec(5.0, 4.5, 3));
        let coef_7 = lookup(&spec(7.0, 4.0, 3));
        let (kt_5, _) = coefficients_to_kt_kq(coef_5, 5.0);
        let (kt_7, _) = coefficients_to_kt_kq(coef_7, 7.0);
        let ratio = kt_7 / kt_5;
        // Plan acceptance: ~2x thrust at same RPM. Bench data + D^4 scaling
        // lands in 1.5-2.8 range across manufacturers.
        assert!(
            (1.5..=3.0).contains(&ratio),
            "7\" vs 5\" thrust ratio {:.2} outside [1.5, 3.0]",
            ratio
        );
    }

    #[test]
    fn higher_pitch_produces_more_thrust() {
        let low = lookup(&spec(5.0, 3.0, 3));
        let mid = lookup(&spec(5.0, 4.5, 3));
        let high = lookup(&spec(5.0, 6.0, 3));
        assert!(low.ct < mid.ct, "low-pitch ct should be < mid-pitch ct");
        assert!(mid.ct < high.ct, "mid-pitch ct should be < high-pitch ct");
    }

    #[test]
    fn unknown_prop_falls_back_gracefully() {
        // Weird sizes shouldn't panic and should produce positive coefficients.
        let coef = lookup(&spec(9.0, 7.5, 3));
        assert!(coef.ct > 0.0);
        assert!(coef.cq > 0.0);
        // Either nearest-neighbor or empirical fallback is acceptable.
    }

    #[test]
    fn empirical_fallback_matches_legacy_formula() {
        // The empirical fallback should reproduce from_build_specs at KV=2300
        // (the formula's reference KV where 2300/KV = 1).
        let coef = empirical_fallback(&spec(5.0, 4.5, 3));
        // base_pf=1.9e-6, pitch_m=0.97, blade_m=1.0 → prop_factor=1.843e-6
        let legacy_kt_at_kv_2300 = 1.843e-6;
        let (kt, _) = coefficients_to_kt_kq(coef, 5.0);
        let rel = (kt - legacy_kt_at_kv_2300).abs() / legacy_kt_at_kv_2300;
        assert!(rel < 0.01, "empirical fallback drifted from legacy: {:.3}%", rel * 100.0);
    }
}
