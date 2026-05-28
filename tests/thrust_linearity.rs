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
    let config = PhysicsConfig::default();
    let max_speed = config.max_motor_speed_from_voltage();
    let min_speed = 100.0_f64; // MIN_MOTOR_SPEED

    for cmd in [0.0_f64, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let omega = throttle_to_omega_with_config(cmd, &config);
        let expected = min_speed + cmd * (max_speed - min_speed);
        assert!(
            (omega - expected).abs() < 1e-6,
            "ω(cmd={cmd}) = {omega:.4}, expected {expected:.4} (linear)"
        );
    }
}

#[test]
fn px4_thr_mdl_fac_1_round_trip_is_linear_in_desired_thrust() {
    // Simulates PX4 with `THR_MDL_FAC=1`: PX4's controller picks `thr_desired`
    // in [0, 1], outputs `cmd = sqrt(thr_desired)` to the actuator. We then
    // run that cmd through the sim and verify the resulting thrust fraction
    // matches `thr_desired` (i.e., the round-trip is linear).
    let config = PhysicsConfig::default();
    let max_omega = config.max_motor_speed_from_voltage();
    let max_thrust = compute_thrust(max_omega, &config);

    // Skip the very low end where the MIN_MOTOR_SPEED offset dominates and
    // the round-trip can't be perfectly linear (this is fine in practice —
    // PX4 never operates the motors near absolute zero anyway, since
    // MPC_THR_MIN ≥ 0.05 in any realistic config).
    for thr_desired in [0.05_f64, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let cmd_from_px4 = thr_desired.sqrt();
        let omega = throttle_to_omega_with_config(cmd_from_px4, &config);
        let actual_thrust = compute_thrust(omega, &config);
        let actual_fraction = actual_thrust / max_thrust;
        // 6% tolerance for the MIN_MOTOR_SPEED offset; tightens at higher thr.
        let tol = 0.06;
        assert!(
            (actual_fraction - thr_desired).abs() < tol,
            "thr_desired={thr_desired:.2} → cmd={cmd_from_px4:.4} → \
             actual_thrust_fraction={actual_fraction:.4}, expected {thr_desired:.2} \
             (within {tol})"
        );
    }
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

    // Linear-in-ω: ω(0.5) ≈ MAX/2, so thrust(0.5)/thrust(1.0) ≈ 0.25–0.28
    // depending on MIN_MOTOR_SPEED / max_speed ratio. The linear-thrust
    // alternative would give ~0.5, so we just need to rule that out.
    assert!(
        ratio < 0.35,
        "thrust(cmd=0.5)/thrust(cmd=1.0) = {ratio:.4}, expected <0.35 (quadratic). \
         If this is ~0.5, motor.rs reverted to ω² interpolation — see the \
         note in throttle_to_omega_with_config."
    );
}
