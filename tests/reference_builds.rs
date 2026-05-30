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
            $msg, $min, $max, v
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
        // EMAX specs: 75g AUW with 2S 450mAh
        assert_in_range!(mass_kg, 0.065, 0.085, "Tinyhawk 3 mass");
    }

    #[test]
    fn hover_throttle_regression_gate() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~29% (higher than real ~38% because kt is underestimated).
        // Gate: 15-45% — catches major regressions without being too tight.
        assert_in_range!(hover_pct, 0.15, 0.45, "Tinyhawk 3 hover throttle");
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
        assert_in_range!(mass_kg, 0.38, 0.52, "Apex 5 mass");
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

mod geprc_mark5_6 {
    use super::*;

    fn build() -> BuildSpec {
        BuildSpec::geprc_mark5_6()
    }

    #[test]
    fn total_mass_is_realistic() {
        let mass_kg = build().total_mass_kg();
        // 6" builds with 6S 1100mAh: 480-580g
        assert_in_range!(mass_kg, 0.46, 0.60, "MARK5 6 mass");
    }

    #[test]
    fn hover_throttle_regression_gate() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~13% (6S is very efficient). Real: 25-35%.
        // Gate: 8-40%.
        assert_in_range!(hover_pct, 0.08, 0.40, "MARK5 6 hover throttle");
    }

    #[test]
    fn hover_current_regression_gate() {
        let physics = build().to_physics_config();
        let hover_a = physics.estimated_hover_current_a();
        // Post-recalibration: lower kq → lower current. Model ~8.5A.
        // Real: 15-25A. Gate: 5-35A.
        assert_in_range!(hover_a, 5.0, 35.0, "MARK5 6 hover current");
    }

    #[test]
    fn can_climb() {
        let physics = build().to_physics_config();
        let climb_mps = physics.estimated_max_climb_rate_mps();
        // Torque-balance model gives realistic max speed. Gate: 10-150 m/s.
        assert_in_range!(climb_mps, 10.0, 150.0, "MARK5 6 max climb rate");
    }

    #[test]
    fn hover_time_regression_gate() {
        let physics = build().to_physics_config();
        let hover_time_s = physics.estimated_hover_time_s(1100.0);
        // Lower current → longer hover. Model ~372s. Real: 2.5-4 min.
        // Gate: 60-600s.
        assert_in_range!(hover_time_s, 60.0, 600.0, "MARK5 6 hover time");
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
        assert_in_range!(mass_kg, 0.95, 1.20, "Chimera 7 mass");
    }

    #[test]
    fn hover_throttle_is_efficient() {
        let physics = build().to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        // Model produces ~15%. Real long-range 7": 20-35%.
        // Gate: 8-40%.
        assert_in_range!(hover_pct, 0.08, 0.40, "Chimera 7 hover throttle");
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
    let mark5 = BuildSpec::geprc_mark5_6().total_mass_kg();
    let chimera = BuildSpec::iflight_chimera_7().total_mass_kg();

    assert!(tinyhawk < apex, "Tinyhawk should be lighter than Apex 5");
    assert!(apex < mark5, "Apex 5 should be lighter than MARK5 6");
    assert!(mark5 < chimera, "MARK5 6 should be lighter than Chimera 7");
}

#[test]
fn all_builds_have_positive_thrust_margin() {
    // All builds should be able to hover below 70% throttle (safety margin)
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5 6", BuildSpec::geprc_mark5_6()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics = build.to_physics_config();
        let hover_pct = physics.hover_throttle_percent();
        assert!(
            hover_pct < 0.70,
            "{} hover throttle {}% is too high — needs more thrust margin",
            name, hover_pct * 100.0
        );
    }
}

#[test]
fn all_builds_produce_positive_climb_rate() {
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5 6", BuildSpec::geprc_mark5_6()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics = build.to_physics_config();
        let climb = physics.estimated_max_climb_rate_mps();
        assert!(
            climb > 5.0,
            "{} max climb rate {} m/s is too low",
            name, climb
        );
    }
}

#[test]
fn all_physics_configs_have_consistent_mass() {
    // Verify that to_physics_config().mass_kg matches total_mass_kg()
    for (name, build) in [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5 6", BuildSpec::geprc_mark5_6()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ] {
        let physics_mass = build.to_physics_config().mass_kg;
        let total_mass = build.total_mass_kg();
        assert!(
            (physics_mass - total_mass).abs() < 1e-9,
            "{} mass mismatch: physics={} vs total={}",
            name, physics_mass, total_mass
        );
    }
}

// =============================================================================
// Bench-accuracy anchor: iFlight XING2 2207 2755KV @4S on 5.1" prop
// =============================================================================

#[test]
fn bench_anchor_iflight_2207_2755kv_at_4s() {
    // iFlight XING2 2207 bench report (thingbits/iflight):
    // 2755KV @4S (16V), GF51466 (5.1"×4.66"×3-blade): 1576g thrust, 40.35A.
    // The model must reproduce max-thrust within ±15% to stay bench-validated.
    // Uses BuildSpec → to_physics_config() for the physical-CT lookup path.
    let mut spec = BuildSpec::default();
    spec.motors.kv = 2755.0;
    spec.motors.weight_g = 32.0;
    spec.propellers.diameter_in = 5.1;
    spec.propellers.pitch_in = 4.66;
    spec.propellers.blade_count = 3;
    spec.battery = hitl_physics::build::BatterySpec {
        cell_count: 4,
        capacity_mah: 1500.0,
        c_rating: 75.0,
        weight_g: 180.0,
        internal_resistance_mohm: None,
    };
    // Override battery_voltage to match bench power supply (16V, not nominal 14.8)
    let mut config = spec.to_physics_config();
    config.battery_voltage = 16.0;

    let max_omega = config.max_motor_speed_from_voltage();
    let max_thrust_per_motor_n = config.kt * max_omega * max_omega;
    let max_thrust_per_motor_g = max_thrust_per_motor_n / 9.80665 * 1000.0;
    let bench_g = 1576.0;

    let error_pct = ((max_thrust_per_motor_g - bench_g) / bench_g).abs() * 100.0;
    assert!(
        error_pct < 15.0,
        "bench anchor: model gives {:.0}g vs bench {:.0}g ({:.1}% error, limit 15%)",
        max_thrust_per_motor_g, bench_g, error_pct
    );
}

#[test]
fn user_build_hover_throttle_is_realistic() {
    // User's build: iFlight XING2 2207 1855KV, Gemfan 5030 (5×3), 4S (16.8V
    // charged), ~490g frame, 32g motor, battery Auline 21700 4S 4500mAh ~640g.
    // Total ~900g AUW. Expected hover: 55-80% (heavy build, moderate KV).
    let mut spec = BuildSpec::default();
    spec.motors.kv = 1855.0;
    spec.motors.weight_g = 32.0;
    spec.propellers.diameter_in = 5.0;
    spec.propellers.pitch_in = 3.0;
    spec.propellers.blade_count = 3;
    spec.frame.weight_g = 490.0;
    spec.battery = hitl_physics::build::BatterySpec {
        cell_count: 4,
        capacity_mah: 4500.0,
        c_rating: 30.0,
        weight_g: 640.0,
        internal_resistance_mohm: None,
    };

    let mut config = spec.to_physics_config();
    config.battery_voltage = 16.8; // fully charged 4S

    let hover_pct = config.hover_throttle_percent();
    assert!(
        hover_pct > 0.50 && hover_pct < 0.85,
        "user build hover {:.1}% outside expected [50%, 85%]",
        hover_pct * 100.0
    );
}
