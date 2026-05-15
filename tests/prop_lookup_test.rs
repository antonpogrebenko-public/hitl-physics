//! Phase 5 acceptance tests — propeller CT/CQ lookup integration.
//!
//! These cover the plan's acceptance criteria from
//! `docs/superpowers/plans/2026-05-15-phase5-prop-ct-cq-lookup.md`:
//!
//! - Default 5"x4.5"x3 build: `kt` within ±10% of legacy `from_build_specs`.
//! - 5" 2-blade produces ~15% less thrust than 5" 3-blade at same RPM.
//! - 7" prop produces ~2x thrust of 5" at same RPM (D^4 scaling combined
//!   with the new lookup-derived CT).
//! - Lookup table has at least 50 entries.

use hitl_physics::build::{BuildSpec, PropellerSpec};
use hitl_physics::config::PhysicsConfig;
use hitl_physics::prop_coefficients::{self, coefficients_to_kt_kq, LookupSource};

fn spec(diameter: f64, pitch: f64, blades: i32) -> PropellerSpec {
    PropellerSpec {
        diameter_in: diameter,
        pitch_in: pitch,
        blade_count: blades,
        weight_g: None,
    }
}

#[test]
fn lookup_table_has_at_least_50_entries() {
    assert!(
        prop_coefficients::table_size() >= 50,
        "expected >=50 entries, got {}",
        prop_coefficients::table_size()
    );
}

#[test]
fn default_5x4_5x3_kt_within_10_percent_of_legacy() {
    let via_build = BuildSpec::default().to_physics_config();
    let legacy = PhysicsConfig::from_build_specs(1700.0, 5.0, 4.5, 3, 350.0, 33.0, 14.8);
    let rel = (via_build.kt - legacy.kt).abs() / legacy.kt;
    assert!(
        rel < 0.10,
        "kt drifted by {:.2}% from legacy anchor (got {:.4e}, legacy {:.4e})",
        rel * 100.0, via_build.kt, legacy.kt
    );
}

#[test]
fn two_blade_5_inch_is_about_15_percent_less_than_three_blade() {
    let coef_2 = prop_coefficients::lookup(&spec(5.0, 4.5, 2));
    let coef_3 = prop_coefficients::lookup(&spec(5.0, 4.5, 3));
    let ratio = coef_2.ct / coef_3.ct;
    // Plan target: ~15% drop. Bench data: 10-20% drop. Accept 0.78..0.92.
    assert!(
        (0.78..=0.92).contains(&ratio),
        "5\" 2-blade / 5\" 3-blade ct ratio {:.3} outside [0.78, 0.92]",
        ratio
    );
}

#[test]
fn seven_inch_produces_about_double_five_inch_thrust_at_same_rpm() {
    let coef_5 = prop_coefficients::lookup(&spec(5.0, 4.5, 3));
    let coef_7 = prop_coefficients::lookup(&spec(7.0, 4.0, 3));
    let (kt_5, _) = coefficients_to_kt_kq(coef_5, 5.0);
    let (kt_7, _) = coefficients_to_kt_kq(coef_7, 7.0);
    let ratio = kt_7 / kt_5;
    // Plan acceptance: ~2x. D^4 = (7/5)^4 = 3.84 raw; calibrated CT pulls it
    // down to a realistic 1.5-2.8 range across published bench data.
    assert!(
        (1.5..=3.0).contains(&ratio),
        "7\" / 5\" kt ratio {:.2} outside [1.5, 3.0]",
        ratio
    );
}

#[test]
fn pitch_monotonicity_within_5_inch_class() {
    let low = prop_coefficients::lookup(&spec(5.0, 3.0, 3));
    let mid = prop_coefficients::lookup(&spec(5.0, 4.5, 3));
    let high = prop_coefficients::lookup(&spec(5.0, 6.0, 3));
    assert!(low.ct < mid.ct, "5x3 < 5x4.5 expected");
    assert!(mid.ct < high.ct, "5x4.5 < 5x6 expected");
}

#[test]
fn blade_monotonicity_at_fixed_diameter_pitch() {
    let two = prop_coefficients::lookup(&spec(5.0, 4.5, 2));
    let three = prop_coefficients::lookup(&spec(5.0, 4.5, 3));
    let four = prop_coefficients::lookup(&spec(5.0, 4.5, 4));
    assert!(two.ct < three.ct, "2-blade < 3-blade");
    assert!(three.ct < four.ct, "3-blade < 4-blade");
}

#[test]
fn lookup_is_kv_independent() {
    // Two motors with the same prop must produce the same kt regardless of KV.
    let mut low_kv = BuildSpec::default();
    let mut high_kv = BuildSpec::default();
    low_kv.motors.kv = 1500.0;
    high_kv.motors.kv = 2400.0;
    let kt_low = low_kv.to_physics_config().kt;
    let kt_high = high_kv.to_physics_config().kt;
    assert!(
        (kt_low - kt_high).abs() < 1e-12,
        "kt should be KV-independent after Phase 5 (got {} vs {})",
        kt_low, kt_high
    );
}

#[test]
fn reference_build_props_resolve_via_table() {
    // All four reference builds should hit ExactMatch (their props are in CSV).
    let builds: [(&str, BuildSpec); 4] = [
        ("Tinyhawk 3", BuildSpec::tinyhawk_3()),
        ("Apex 5", BuildSpec::impulse_apex_5()),
        ("MARK5 6", BuildSpec::geprc_mark5_6()),
        ("Chimera 7", BuildSpec::iflight_chimera_7()),
    ];
    for (name, build) in builds {
        let coef = prop_coefficients::lookup(&build.propellers);
        assert!(
            matches!(
                coef.source,
                LookupSource::ExactMatch | LookupSource::DimensionAverage
            ),
            "{}: prop {:?} did not match an entry (got {:?})",
            name, build.propellers, coef.source
        );
    }
}
