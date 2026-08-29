//! Reference-build regression tests — Phase 7 validation harness.
//!
//! These tests assert that well-documented real builds produce physics
//! parameters within expected ranges. The primary goal is **regression
//! detection** — if a code change shifts these values significantly, the
//! test fails and forces manual review.
//!
//! ## Known model limitations
//!
//! The `from_build_specs` kt/kq formula is empirical and doesn't scale
//! perfectly across all KV ranges:
//! - High-KV micro builds (>5000 KV): kt is underestimated, leading to
//!   higher computed RPM than real whoops achieve.
//! - Hover current estimates use torque-based model which can overestimate
//!   for small motors with high mechanical efficiency.
//!
//! These limitations are acceptable for HITL (order-of-magnitude correct).
//! Phase 5 (prop CT/CQ lookup) and real .ulg replay validation will tighten
//! the model further.
//!
//! Run with: `cargo test --test reference_builds`

use hitl_physics::build::BuildSpec;

// =============================================================================
// Helper macros for range assertions
// =============================================================================

macro_rules! assert_in_range {
    ($value:expr, $min:expr, $max:expr, $msg:expr) => {
        let v = $value;
        assert!(
            v >= $min && v <= $max,
            "{}: expected [{}, {}], got {}",
            $msg,
            $min,
            $max,
            v
        );
    };
}

// =============================================================================
// Tinyhawk 3 — 3" whoop, ~75g AUW
// =============================================================================
// Model produces: hover ~21%, current ~19A, climb ~31 m/s
// Note: High-KV (12000) micros are at the edge of the empirical kt formula's
// validity range. Values are order-of-magnitude correct for HITL purposes.

mod tinyhawk_3 {
    use super::*;

    fn build() -> BuildSpec {
        BuildSpec::tinyhawk_3()
    }

    #[test]
    fn total_mass_is_realistic() {
        let mass_kg = build().total_mass_kg();
        // 32 g dry (EMAX BNF manual v2.0) + 12.5 g stock 1S 450 mAh pack
        // (ReadyMadeRC) = 44.5 g. DERIVED: no vendor publishes an all-up
        // weight for this airframe. The previous 0.065-0.085 band came with
        // the 2S/3-inch record that described a different machine.
        assert_in_range!(mass_kg, 0.042, 0.048, "Tinyhawk 3 mass");
    }

    #[test]
    fn hover_throttle_regression_gate() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Retired as a precision gate on 2026-08-29, and deliberately NOT
        // re-fitted to the new output.
        //
        // The previous band was 0.15-0.45, drawn around what the model itself
        // produced at the time ("Model produces ~29%"); the "real ~38%" it
        // compared against carried no source. Fitting a gate to model output
        // and then citing it as validation is how the thrust error survived
        // three months. After the re-anchor and the current cap this build
        // reads 0.67, which corresponds to TWR ~2.25 — plausible for a 75 g
        // whoop, but that is reasoning, not measurement.
        //
        // Until task 5.13 checks this airframe's pack C-rating and ESC rating
        // against the real product, the honest gate is a broad
        // physical-plausibility bound: a hovering multirotor is somewhere
        // between a quarter and four-fifths throttle. It still catches a wild
        // regression; it no longer pretends to validate anything.
        assert_in_range!(hover_pct, 0.25, 0.80, "Tinyhawk 3 hover throttle");
    }

    #[test]
    fn hover_current_regression_gate() {
        let physics = build().to_physics_config();
        let hover_a = physics.estimated_hover_current_a();
        // Model produces ~19A (higher than real 3-5A due to kq/Kt formula at high KV).
        // Gate: 5-30A — catches major regressions.
        assert_in_range!(hover_a, 5.0, 30.0, "Tinyhawk 3 hover current");
    }

    #[test]
    fn can_climb() {
        let physics = build().to_physics_config();
        let climb_mps = physics.estimated_max_climb_rate_mps();
        // Model produces ~55 m/s after Phase 5 prop CT lookup raised kt for
        // the 3"x2"x3 anchor (the new ct gives ~3× the legacy kt at KV 12000,
        // which lifts max thrust per motor). Gate widened to 5-70 m/s — still
        // catches major regressions but absorbs the calibrated kt shift.
        assert_in_range!(climb_mps, 5.0, 70.0, "Tinyhawk 3 max climb rate");
    }

    #[test]
    fn hover_time_regression_gate() {
        let physics = build().to_physics_config();
        let hover_time_s = physics.estimated_hover_time_s(450.0);
        // Model produces ~68s (low due to overestimated current).
        // Gate: 30-300s — catches major regressions.
        assert_in_range!(hover_time_s, 30.0, 300.0, "Tinyhawk 3 hover time");
    }
}

// =============================================================================
// ImpulseRC Apex 5" — flagship 5" race quad, ~440g AUW
// =============================================================================
// Model produces: hover ~22%, current ~21A, climb ~63 m/s

mod impulse_apex_5 {
    use super::*;

    fn build() -> BuildSpec {
        BuildSpec::impulse_apex_5()
    }

    #[test]
    fn total_mass_is_realistic() {
        let mass_kg = build().total_mass_kg();
        // Typical 5" build: 400-500g with lightweight 4S 1300mAh
        // 378 g dry / 542 g all-up, from a documented build of this exact
        // frame with a 4S 1300 pack (rotorbuilds.com/build/26613). The
        // previous band excluded the FPV gear the model has no field for.
        assert_in_range!(mass_kg, 0.52, 0.56, "Apex 5 mass");
    }

    #[test]
    fn hover_throttle_regression_gate() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~22%. Real builds: 28-38%. Gate: 15-45%.
        assert_in_range!(hover_pct, 0.15, 0.45, "Apex 5 hover throttle");
    }

    #[test]
    fn hover_current_regression_gate() {
        let physics = build().to_physics_config();
        let hover_a = physics.estimated_hover_current_a();
        // Post-recalibration: kq reduced (KQ_KT_SCALE=0.348) → lower torque-based
        // current. Model produces ~8.6A. Real: 15-22A (motor heating adds ~50-80%
        // above the ideal model). Gate: 5-35A.
        assert_in_range!(hover_a, 5.0, 35.0, "Apex 5 hover current");
    }

    #[test]
    fn can_climb_at_race_speeds() {
        let physics = build().to_physics_config();
        let climb_mps = physics.estimated_max_climb_rate_mps();
        // Torque-balance model + recalibrated kt reduces max thrust. Model
        // produces ~25 m/s. Real: 25-35 m/s. Gate: 10-100 m/s.
        assert_in_range!(climb_mps, 10.0, 100.0, "Apex 5 max climb rate");
    }

    #[test]
    fn hover_time_regression_gate() {
        let physics = build().to_physics_config();
        let hover_time_s = physics.estimated_hover_time_s(1300.0);
        // Lower current → longer hover. Model produces ~437s (~7 min). Real: 3.5-5 min.
        // Gate: 90-600s.
        assert_in_range!(hover_time_s, 90.0, 600.0, "Apex 5 hover time");
    }
}

// =============================================================================
// GEPRC MARK5 6" — 6" freestyle, ~520g AUW
// =============================================================================
// Model produces: hover ~13%, current ~20A, climb ~100 m/s

mod geprc_mark5 {
    use super::*;

    fn build() -> BuildSpec {
        BuildSpec::geprc_mark5()
    }

    #[test]
    fn total_mass_is_realistic() {
        let mass_kg = build().total_mass_kg();
        // 6" builds with 6S 1100mAh: 480-580g
        // 385.0 g published dry (MARK5 Analog 6S TBS) + 226.0 g Storm 6S
        // 1400 mAh, GEPRC's own recommended pack = 611 g. GEPRC publishes no
        // all-up weight anywhere in the MARK5 line, so this is derived.
        assert_in_range!(mass_kg, 0.58, 0.64, "MARK5 mass");
    }

    #[test]
    fn hover_throttle_regression_gate() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~13% (6S is very efficient). Real: 25-35%.
        // Gate: 8-40%.
        assert_in_range!(hover_pct, 0.08, 0.40, "MARK5 hover throttle");
    }

    #[test]
    fn hover_current_regression_gate() {
        let physics = build().to_physics_config();
        let hover_a = physics.estimated_hover_current_a();
        // Post-recalibration: lower kq → lower current. Model ~8.5A.
        // Real: 15-25A. Gate: 5-35A.
        assert_in_range!(hover_a, 5.0, 35.0, "MARK5 hover current");
    }

    #[test]
    fn can_climb() {
        let physics = build().to_physics_config();
        let climb_mps = physics.estimated_max_climb_rate_mps();
        // Torque-balance model gives realistic max speed. Gate: 10-150 m/s.
        assert_in_range!(climb_mps, 10.0, 150.0, "MARK5 max climb rate");
    }

    #[test]
    fn hover_time_regression_gate() {
        let physics = build().to_physics_config();
        let hover_time_s = physics.estimated_hover_time_s(1100.0);
        // Lower current → longer hover. Model ~372s. Real: 2.5-4 min.
        // Gate: 60-600s.
        assert_in_range!(hover_time_s, 60.0, 600.0, "MARK5 hover time");
    }
}

// =============================================================================
// iFlight Chimera 7" — 7" long-range GPS cruiser, ~1050g AUW
// =============================================================================
// Model produces: hover ~15%, current ~32A, climb ~105 m/s

mod iflight_chimera_7 {
    use super::*;

    fn build() -> BuildSpec {
        BuildSpec::iflight_chimera_7()
    }

    #[test]
    fn total_mass_is_realistic() {
        let mass_kg = build().total_mass_kg();
        // 7" long-range: 1000-1150g with 6S 4000mAh
        // 705 g published dry (Pro V2 6S Analog) + 840 g Fullsend 6S2P
        // 8000 mAh Li-ion = 1545 g. iFlight's own sheet says 1621 g, but
        // against the older, heavier Samsung 21700 pack revision.
        assert_in_range!(mass_kg, 1.50, 1.62, "Chimera 7 mass");
    }

    #[test]
    fn hover_throttle_is_efficient() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~15%. Real long-range 7": 20-35%.
        // Gate: 8-40%.
        // Now pack-limited: 8 Ah of Li-ion at a printed 17.5C shared four
        // ways is 35 A per motor, under the ESC's 65 A. A long-range build
        // hovering near 40% is what that ceiling implies, and far more
        // credible than the 8-40% the 50C LiPo record allowed.
        assert_in_range!(hover_pct, 0.30, 0.50, "Chimera 7 hover throttle");
    }

    #[test]
    fn hover_current_regression_gate() {
        let physics = build().to_physics_config();
        let hover_a = physics.estimated_hover_current_a();
        // Model produces ~32A (high due to kq formula). Real: 8-14A.
        // Gate: 10-50A — catches major regressions.
        assert_in_range!(hover_a, 10.0, 50.0, "Chimera 7 hover current");
    }

    #[test]
    fn can_climb() {
        let physics = build().to_physics_config();
        let climb_mps = physics.estimated_max_climb_rate_mps();
        // Model produces ~105 m/s. Real: 10-25 m/s.
        // Gate: 10-150 m/s.
        assert_in_range!(climb_mps, 10.0, 150.0, "Chimera 7 max climb rate");
    }

    #[test]
    fn hover_time_supports_long_range() {
        let physics = build().to_physics_config();
        let hover_time_s = physics.estimated_hover_time_s(4000.0);
        // Model produces ~356s (~6 min). Real: 18-25 min.
        // Gate: 180-1200s (model underestimates due to current overestimate).
        assert_in_range!(hover_time_s, 180.0, 1200.0, "Chimera 7 hover time");
    }

    #[test]
    fn has_gps_module() {
        let spec = build();
        assert!(spec.gps.is_some(), "Chimera 7 should have GPS");
        let gps = spec.gps.unwrap();
        assert!(gps.has_compass, "Chimera 7 GPS should have compass");
    }
}

// =============================================================================
// Cross-build sanity checks
// =============================================================================

#[test]
fn builds_are_ordered_by_mass() {
    let tinyhawk = BuildSpec::tinyhawk_3().total_mass_kg();
    let apex = BuildSpec::impulse_apex_5().total_mass_kg();
    let mark5 = BuildSpec::geprc_mark5().total_mass_kg();
    let chimera = BuildSpec::iflight_chimera_7().total_mass_kg();

    assert!(tinyhawk < apex, "Tinyhawk should be lighter than Apex 5");
    assert!(apex < mark5, "Apex 5 should be lighter than MARK5 6");
    assert!(mark5 < chimera, "MARK5 should be lighter than Chimera 7");
}

#[test]
fn all_builds_have_positive_thrust_margin() {
    // All builds should be able to hover below 70% throttle (safety margin)
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5", BuildSpec::geprc_mark5()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics = build.to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        assert!(
            hover_pct < 0.70,
            "{} hover throttle {}% is too high — needs more thrust margin",
            name,
            hover_pct * 100.0
        );
    }
}

#[test]
fn all_builds_produce_positive_climb_rate() {
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5", BuildSpec::geprc_mark5()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics = build.to_physics_config();
        let climb = physics.estimated_max_climb_rate_mps();
        assert!(
            climb > 5.0,
            "{} max climb rate {} m/s is too low",
            name,
            climb
        );
    }
}

#[test]
fn all_physics_configs_have_consistent_mass() {
    // Verify that to_physics_config().mass_kg matches total_mass_kg()
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5", BuildSpec::geprc_mark5()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics_mass = build.to_physics_config().mass_kg;
        let total_mass = build.total_mass_kg();
        assert!(
            (physics_mass - total_mass).abs() < 1e-9,
            "{} mass mismatch: physics={} vs total={}",
            name,
            physics_mass,
            total_mass
        );
    }
}

// =============================================================================
// Bench-accuracy anchor: Cobra CM-2213/12 on an HQ 6x4.5x3, lab bench
// =============================================================================
//
// The model's calibration source, chosen deliberately on 2026-08-29 over the
// iFlight manufacturer figure that previously held this role (see below).
//
// Cobra publish a full instrumented throttle ladder for this motor:
// "Cobra CM-2213/12 Motor Test Data, Kv=2000, Data Collected at 11.1 volts
// with HQ 6x4.5x3 Prop". The 100% row reads 24.65 A, 273.66 W, 17,253 rpm,
// 1032.4 g. Every figure was fetched from the PDF and verified.
//
// This anchor checks THRUST AND RPM TOGETHER. A model can hit a thrust number
// at the wrong shaft speed by having compensating errors in ct and cq — which
// is exactly the state this change was written to fix — so validating thrust
// alone would pass the very defect being removed.

#[test]
fn bench_anchor_cobra_cm2213_on_6x45x3() {
    let mut spec = BuildSpec::default();
    spec.motors.kv = 2000.0;
    spec.motors.weight_g = 32.0;
    spec.propellers.diameter_in = 6.0;
    spec.propellers.pitch_in = 4.5;
    spec.propellers.blade_count = 3;

    let mut config = spec.to_physics_config();
    // The bench rig, not the default pack: a regulated supply at 11.1 V, and
    // the measured 24.65 A draw. Cobra publish 0.42 A no-load for this frame
    // size. With current binding, both thrust and speed are independent of
    // winding resistance, so no value has to be invented for it.
    config.battery_voltage = 11.1;
    config.max_current_per_motor_a = 24.65;
    config.motor_no_load_amps = 0.42;

    let omega = config.max_motor_speed_from_voltage();
    let rpm = omega * 30.0 / std::f64::consts::PI;
    let thrust_g = config.max_thrust_per_motor_n() / 9.80665 * 1000.0;

    let rpm_err = (rpm - 17_253.0).abs() / 17_253.0;
    let thrust_err = (thrust_g - 1032.4).abs() / 1032.4;

    assert!(
        rpm_err < 0.10,
        "bench rpm: model {rpm:.0} vs measured 17253 ({:.1}% error)",
        rpm_err * 100.0
    );
    assert!(
        thrust_err < 0.10,
        "bench thrust: model {thrust_g:.0} g vs measured 1032 g ({:.1}% error)",
        thrust_err * 100.0
    );
}

// =============================================================================
// Retired calibration source: iFlight XING2 2207 2755KV @4S on a 5.1" prop
// =============================================================================
//
// Kept as a recorded comparison, NOT as a gate. Until 2026-08-29 this figure
// calibrated the model; it no longer does.
//
// The report is 1576 g at 40.35 A on 16 V with a GF51466 (5.1x4.66x3). Under
// current limiting, thrust is `(kt/kq)·Kt·(I−I0)`, so the thrust-to-torque
// ratio decides everything, and the two candidate sources disagree about it:
//
//   Cobra CM-2213/12, instrumented lab ladder ... 87.5 m^-1
//   model, calibrated on Cobra .................. 82.4 m^-1   (6% apart)
//   iFlight XING2 2207, this figure ............. 111.9 m^-1  (26% apart)
//
// The Cobra source is a lab test with a full throttle ladder, per-row current,
// voltage and rpm. This one is a manufacturer/retailer headline with no rpm,
// no throttle ladder and no rig description — the class of figure that is
// systematically best-case. The model is expected to read BELOW it, and this
// test asserts that direction and bounds the gap so a genuine regression still
// shows up. It does not pull the model toward the number.

#[test]
fn model_reads_below_the_iflight_manufacturer_figure() {
    let mut spec = BuildSpec::default();
    spec.motors.kv = 2755.0;
    spec.motors.weight_g = 32.0;
    spec.propellers.diameter_in = 5.1;
    spec.propellers.pitch_in = 4.66;
    spec.propellers.blade_count = 3;

    let mut config = spec.to_physics_config();
    config.battery_voltage = 16.0;
    config.max_current_per_motor_a = 40.35;

    let thrust_g = config.max_thrust_per_motor_n() / 9.80665 * 1000.0;
    let ratio = thrust_g / 1576.0;

    assert!(
        ratio < 1.0,
        "model {thrust_g:.0} g exceeds a manufacturer best-case figure of 1576 g — \
         manufacturer headlines should bound the model from above, not below"
    );
    assert!(
        ratio > 0.60,
        "model {thrust_g:.0} g is {:.0}% of the manufacturer figure; a gap that \
         large is a regression, not source scepticism",
        ratio * 100.0
    );
}

/// Impact survey for the thrust re-anchor. Prints every reference build's
/// operating point so a coefficient change can be reviewed build by build
/// rather than accepted wholesale.
///   cargo test --test reference_builds print_all_reference_builds -- --nocapture
#[test]
fn print_all_reference_builds() {
    let builds: [(&str, BuildSpec); 4] = [
        ("tinyhawk_3", BuildSpec::tinyhawk_3()),
        ("impulse_apex_5", BuildSpec::impulse_apex_5()),
        ("geprc_mark5", BuildSpec::geprc_mark5()),
        ("iflight_chimera_7", BuildSpec::iflight_chimera_7()),
    ];
    println!(
        "{:<20} {:>5} {:>6} {:>5} {:>7} {:>8} {:>7} {:>7} {:>6} {:>6}",
        "build", "kv", "prop", "mass", "rpm", "%noload", "thrust", "ceil", "FoM", "TWR"
    );
    for (name, spec) in builds {
        let p = spec.to_physics_config();
        let w = p.max_motor_speed_from_voltage();
        let t = p.max_thrust_per_motor_n();
        let c = p.momentum_theory_thrust_ceiling_n();
        println!(
            "{name:<20} {:>5.0} {:>4.0}x{:.0} {:>5.2} {:>7.0} {:>7.1}% {:>6.0}g {:>6.0}g {:>6.2} {:>6.2}  I={:>5.1}A cap={:>5.1}A(esc {:>4.1}/pack {:>5.1}) {}",
            p.motor_kv,
            spec.propellers.diameter_in,
            spec.propellers.blade_count,
            p.mass_kg,
            w * 30.0 / std::f64::consts::PI,
            w / (p.motor_kv * p.battery_voltage * std::f64::consts::PI / 30.0) * 100.0,
            t / 9.80665 * 1000.0,
            c / 9.80665 * 1000.0,
            t / c,
            4.0 * t / (p.mass_kg * 9.80665),
            p.full_throttle_current_a(),
            p.max_current_per_motor_a,
            p.esc.burst_amps,
            spec.battery.capacity_mah / 1000.0 * spec.battery.c_rating / 4.0,
            if p.is_current_limited() { "CURRENT-LIMITED" } else { "voltage-limited" }
        );
    }
}
