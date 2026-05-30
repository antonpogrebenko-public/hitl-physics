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

/// Physical-CT calibration basis (2026-05-30), matching the recalibrated CSV.
/// `ct_static` is a real static thrust coefficient — roughly size-independent
/// (CT_aero ≈ 0.09–0.12 across FPV prop sizes); thrust scales with D⁴ via `kt`.
/// `K_CT` sets the absolute level, anchored to iFlight XING2 2207 bench data
/// (1576 g & 40.35 A at 16 V full throttle, 5" prop) together with
/// the torque-balance loaded-RPM model. The legacy table over-produced thrust
/// ~4× and had a non-physical diameter dependence; this replaces it.
const K_CT: f64 = 0.00283;
/// Torque/thrust de-inflation: bench kq/kt ≈ 0.0088 vs the legacy ~0.025 ratio.
const KQ_KT_SCALE: f64 = 0.348;

/// Physical-CT empirical fallback so unknown props degrade gracefully when the
/// CSV is empty or all entries are unusable. Same basis as the CSV.
fn empirical_fallback(spec: &PropellerSpec) -> PropCoefficients {
    let pitch_m = 0.7 + 0.06 * spec.pitch_in;
    let blade_m = 0.85 + 0.05 * spec.blade_count as f64;
    let ct = K_CT * pitch_m * blade_m;
    // kq = kt·kq_kt ⇒ cq = ct·kq_kt / D_meters.
    let kq_kt = KQ_KT_SCALE * (0.010 + 0.002 * spec.pitch_in + 0.002 * spec.blade_count as f64);
    let d_m = spec.diameter_in * 0.0254;
    let cq = ct * kq_kt / d_m;
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
        // Anchor: kt for the canonical 5"×4.5"×3 racing prop after the 2026-05-30
        // bench recalibration (CT_aero ≈ 0.109, a typical FPV 5" value). Was the
        // legacy ~3.376e-6 — over-produced thrust ~4× (see CSV header).
        let bench_kt = 8.77e-7;
        let rel = (kt - bench_kt).abs() / bench_kt;
        assert!(rel < 0.10, "kt drifted by {:.1}% from bench anchor", rel * 100.0);
    }

    #[test]
    fn two_blade_produces_less_thrust_than_three_blade() {
        let coef_2 = lookup(&spec(5.0, 4.5, 2));
        let coef_3 = lookup(&spec(5.0, 4.5, 3));
        let ratio = coef_2.ct / coef_3.ct;
        // 2-blade should be 5-25% less than 3-blade. Current linear model gives
        // (0.85+0.05*2)/(0.85+0.05*3) = 0.95. Physical range is 0.82-0.95.
        assert!(
            (0.75..=0.96).contains(&ratio),
            "2-blade/3-blade thrust ratio {:.3} outside [0.75, 0.96]",
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
        // Physical D^4 scaling: (7/5)^4 = 3.84. With lower pitch on 7" (4.0 vs
        // 4.5) the ct is slightly reduced, landing ~3.5-3.8× in practice.
        assert!(
            (2.0..=4.5).contains(&ratio),
            "7\" vs 5\" thrust ratio {:.2} outside [2.0, 4.5]",
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
    fn empirical_fallback_uses_physical_ct() {
        // The fallback uses the physical-CT basis: ct = K_CT·pitch_m·blade_m.
        let coef = empirical_fallback(&spec(5.0, 4.5, 3));
        let expected_ct = K_CT * (0.7 + 0.06 * 4.5) * (0.85 + 0.05 * 3.0);
        assert!((coef.ct - expected_ct).abs() / expected_ct < 1e-9);
        // Resulting CT_aero must sit in the physical FPV-prop range.
        let ct_aero = coef.ct * (2.0 * std::f64::consts::PI).powi(2);
        assert!((0.08..0.13).contains(&ct_aero), "CT_aero {ct_aero:.3} unphysical");
    }
}
