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

use std::sync::OnceLock;
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
    // Manufacturer / model are kept for future tie-breaking and diagnostic
    // dumps; not consumed by the current lookup paths.
    //
    // Borrowed from `CSV_DATA`, which is `include_str!`-ed and therefore
    // 'static, rather than allocated. Two `String`s per row over 52 rows was
    // 104 heap allocations for text nothing reads.
    #[allow(dead_code)]
    manufacturer: &'static str,
    #[allow(dead_code)]
    model: &'static str,
    ct: f64,
    cq: f64,
}

fn parse_entries() -> Vec<Entry> {
    // Sized up front: the table is 52 rows, and growing from zero reallocated
    // roughly seven times on the way there.
    let mut out = Vec::with_capacity(64);
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
            manufacturer: fields[3],
            model: fields[4],
            ct,
            cq,
        });
    }
    out
}

/// Parsed once per process, not once per thread.
///
/// `thread_local!` re-parsed the whole table on every thread that looked
/// anything up — the daemon has several — for a table that is `include_str!`-ed
/// and therefore identical in all of them.
static ENTRIES: OnceLock<Vec<Entry>> = OnceLock::new();

fn with_entries<R>(f: impl FnOnce(&[Entry]) -> R) -> R {
    f(ENTRIES.get_or_init(parse_entries).as_slice())
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
        //
        // Folded rather than collected: the previous form allocated a Vec of
        // references on every lookup before it knew whether anything matched.
        let mut sum_ct = 0.0;
        let mut sum_cq = 0.0;
        let mut matches = 0usize;
        for e in entries.iter().filter(|e| {
            (e.diameter_in - spec.diameter_in).abs() < 1e-3
                && (e.pitch_in - spec.pitch_in).abs() < 1e-3
                && e.blade_count == spec.blade_count
        }) {
            sum_ct += e.ct;
            sum_cq += e.cq;
            matches += 1;
        }
        if matches > 0 {
            let source = if matches == 1 {
                LookupSource::ExactMatch
            } else {
                LookupSource::DimensionAverage
            };
            return PropCoefficients {
                ct: sum_ct / matches as f64,
                cq: sum_cq / matches as f64,
                source,
            };
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
/// `K_CT` sets the absolute level. It is anchored to measured bench data for a
/// six-inch three-blade propeller, for which two unrelated sources agree:
///
/// - Cobra CM-2213/12 (22 mm 12N14P stator) on an HQ 6x4.5x3, 11.1 V, full
///   throttle: 1032.4 g at 17,253 rpm on 273.66 W → CT_aero 0.1853.
/// - APC's own performance file for the 6x4.2E-3 three-blade: static CT 0.1850.
///
/// Pitch-corrected to 6x4.0 that is CT_aero ≈ 0.180, which is what `K_CT`
/// reproduces. The previous 0.00283 gave 0.105 — 1.76x low — and the previous
/// `KQ_KT_SCALE` of 0.348 gave CQ_aero 0.0058 against a measured 0.0139, 2.4x
/// low. The two errors pointed opposite ways: too little drag torque let the
/// torque balance settle at 98% of no-load, and the excess omega then made up
/// most of the missing thrust. Thrust looked plausible while both halves were
/// wrong, and the giveaway was an implied figure of merit above 0.8 on builds
/// where no real propeller exceeds 0.68.
///
/// Because they compensate, these two constants must be changed together.
/// `PhysicsConfig::momentum_theory_thrust_ceiling_n` exists to catch the
/// combination that made this hard to see.
const K_CT: f64 = 0.003651555;
/// Torque/thrust de-inflation: bench kq/kt ≈ 0.0088 vs the legacy ~0.025 ratio.
const KQ_KT_SCALE: f64 = 0.47625;

/// Physical-CT empirical fallback so unknown props degrade gracefully when the
/// CSV is empty or all entries are unusable. Same basis as the CSV.
/// The closed-form coefficient basis, exposed so `examples/regenerate_prop_table`
/// can reproduce `data/prop_coefficients.csv` from it. Every row of that table is
/// this function's output — the table caches the basis, it does not supplement it.
pub fn derive_from_basis(spec: &PropellerSpec) -> PropCoefficients {
    empirical_fallback(spec)
}

fn empirical_fallback(spec: &PropellerSpec) -> PropCoefficients {
    let pitch_m = 0.7 + 0.06 * spec.pitch_in;
    let blade_m = blade_multiplier(spec.blade_count);
    let ct = K_CT * pitch_m * blade_m;
    // kq = kt·kq_kt ⇒ cq = ct·kq_kt / D_meters.
    let kq_kt = KQ_KT_SCALE * (0.010 + 0.002 * spec.pitch_in + 0.002 * spec.blade_count as f64);
    let d_m = spec.diameter_in * 0.0254;
    let cq = ct * kq_kt / d_m;
    PropCoefficients {
        ct,
        cq,
        source: LookupSource::EmpiricalFallback,
    }
}

fn pitch_ratio(requested: f64, reference: f64) -> f64 {
    // Linear pitch model lining up with the empirical formula (0.7 + 0.06·P).
    let num = 0.7 + 0.06 * requested;
    let den = 0.7 + 0.06 * reference;
    (num / den).max(0.1)
}

/// Thrust multiplier for blade count, normalised to a two-blade propeller.
///
/// `(B/2)^0.7`, fitted to the UIUC Propeller Data Site's blade-count control
/// pairs — the DA4022 and DA4052 in 5x3.75 and 9x6.75, each run as 2-, 3- and
/// 4-blade on the SAME physical blades with only the hub swapped, so the ratio
/// isolates blade count from every other variable. Their measured static
/// three-to-two ratios at 5000 rpm are 1.373, 1.350, 1.264 and 1.280.
///
/// Independently: APC's own files give 1.298 for 11x7E vs 11x7E-3 and 1.424
/// for 5.1x5.0E vs its three-blade, and CSIR-NAL measured 30% on a six-inch
/// pair of identical plan form. This model gives 1.328.
///
/// The previous `0.85 + 0.05*B` gave **1.053** — a three-blade barely ahead of
/// a two-blade, against measurement clustering near 1.3. A comment in the
/// coefficient table claimed 15%, which was also unsourced and also wrong.
/// Thrust does not scale with blade area (that would be 1.5); the exponent is
/// well below 1 because added blades work in each other's wake.
fn blade_multiplier(blade_count: i32) -> f64 {
    (blade_count as f64 / 2.0).powf(0.7)
}

fn blade_ratio(requested: i32, reference: i32) -> f64 {
    (blade_multiplier(requested) / blade_multiplier(reference)).max(0.1)
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
        assert!(
            table_size() >= 40,
            "expected >=40 entries, got {}",
            table_size()
        );
    }

    #[test]
    fn exact_match_for_5x4_5x3_anchor() {
        let coef = lookup(&spec(5.0, 4.5, 3));
        assert!(matches!(
            coef.source,
            LookupSource::ExactMatch | LookupSource::DimensionAverage
        ));
        let (kt, _kq) = coefficients_to_kt_kq(coef, 5.0);
        // Anchor: kt for the canonical 5"x4.5"x3 racing prop.
        //
        // 2026-08-29 re-anchor: 8.77e-7 -> 1.4993e-6 (CT_aero 0.109 -> 0.186).
        // The 2026-05-30 value was set from a single 5" operating point and was
        // 1.71x low against measured tri-blade data — Cobra CM-2213/12 on an
        // HQ 6x4.5x3 gives CT_aero 0.1853 and APC's own 6x4.2E-3 file gives
        // 0.1850. Changed deliberately; see the change
        // `fix-hitl-sim-flight-regression`, task 5.7.
        let bench_kt = 1.4993e-6;
        let rel = (kt - bench_kt).abs() / bench_kt;
        assert!(
            rel < 0.10,
            "kt drifted by {:.1}% from bench anchor",
            rel * 100.0
        );
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
        let expected_ct = K_CT * (0.7 + 0.06 * 4.5) * blade_multiplier(3);
        assert!((coef.ct - expected_ct).abs() / expected_ct < 1e-9);
        // Resulting CT_aero must sit in the physical FPV-prop range.
        let ct_aero = coef.ct * (2.0 * std::f64::consts::PI).powi(2);
        // Measured static CT for FPV tri-blades: APC 6x4.2E-3 0.185, Cobra
        // HQ 6x4.5x3 0.1853, UIUC three-blade props 0.149-0.212. The previous
        // 0.08-0.13 band was set from the model's own output rather than from
        // measurement, and every value in it is below what any of those report.
        assert!(
            (0.15..0.22).contains(&ct_aero),
            "CT_aero {ct_aero:.3} outside the measured range for a tri-blade"
        );
    }
}
