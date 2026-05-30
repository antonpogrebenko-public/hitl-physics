//! Verify the cmd → ω → thrust contract that the daemon advertises to PX4.
//!
//! The sim uses linear cmd → ω (matching real ESCs: BLHeli, Bluejay, etc.),
//! so thrust = kt·ω² is **quadratic** in cmd. The daemon pushes
//! `THR_MDL_FAC=1` to PX4, which makes PX4 output `cmd = sqrt(thr_desired)`
//! to the actuator. Round-trip: PX4 wants `thr_desired` → outputs
//! `sqrt(thr_desired)` → sim produces `thr_desired·max_thrust`. Effectively
//! linear from PX4's controller perspective.
//!
//! Regressions on either side of this contract — ω² interpolation in motor.rs
//! or `THR_MDL_FAC=0` in the daemon — break the contract and cause the
//! rate-loop trembling we hit in 2026-05-17.

use hitl_physics::{compute_thrust, throttle_to_omega_with_config, PhysicsConfig};

#[test]
fn omega_is_linear_in_actuator_command() {
    // ω = cmd · max_speed (pure linear, no idle offset after idle revert).
    let config = PhysicsConfig::default();
    let max_speed = config.max_motor_speed_from_voltage();

    for cmd in [0.0_f64, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let omega = throttle_to_omega_with_config(cmd, &config);
        let expected = cmd * max_speed;
        assert!(
            (omega - expected).abs() < 1e-6,
            "ω(cmd={cmd}) = {omega:.4}, expected {expected:.4} (linear)"
        );
    }
}

#[test]
fn px4_thr_mdl_fac_1_round_trip_is_exact() {
    // With no idle offset and THR_MDL_FAC=1: PX4 outputs cmd = sqrt(thr_desired),
    // sim produces ω = cmd·max → thrust = kt·(cmd·max)² = thr_desired·max_thrust.
    // The round-trip is exact (within floating-point epsilon).
    let config = PhysicsConfig::default();
    let max_omega = config.max_motor_speed_from_voltage();
    let max_thrust = compute_thrust(max_omega, &config);

    for thr_desired in [0.1_f64, 0.25, 0.5, 0.75, 1.0] {
        let cmd_from_px4 = thr_desired.sqrt();
        let omega = throttle_to_omega_with_config(cmd_from_px4, &config);
        let actual_fraction = compute_thrust(omega, &config) / max_thrust;
        assert!(
            (actual_fraction - thr_desired).abs() < 1e-9,
            "thr_desired={thr_desired:.2} → cmd={cmd_from_px4:.4} → \
             fraction={actual_fraction:.4}; round-trip should be exact"
        );
    }

    // Exact at full throttle.
    let full = compute_thrust(throttle_to_omega_with_config(1.0, &config), &config) / max_thrust;
    assert!((full - 1.0).abs() < 1e-9, "full-throttle thrust must equal max_thrust");
}

#[test]
fn cmd_to_thrust_is_quadratic_not_linear() {
    // Locks in the deliberate quadratic shape of cmd → thrust. If someone
    // re-introduces the ω² interpolation thinking "thrust should be linear in
    // cmd", this test fails and points them at the daemon's THR_MDL_FAC push.
    let config = PhysicsConfig::default();
    let max_omega = config.max_motor_speed_from_voltage();
    let max_thrust = compute_thrust(max_omega, &config);

    let omega_half = throttle_to_omega_with_config(0.5, &config);
    let thrust_half = compute_thrust(omega_half, &config);
    let ratio = thrust_half / max_thrust;

    // Linear-in-ω: ω(0.5) = MAX/2, so thrust(0.5)/thrust(1.0) = 0.25 exactly.
    assert!(
        ratio < 0.35,
        "thrust(cmd=0.5)/thrust(cmd=1.0) = {ratio:.4}, expected <0.35 (quadratic). \
         If this is ~0.5, motor.rs reverted to ω² interpolation — see the \
         note in throttle_to_omega_with_config."
    );
}
