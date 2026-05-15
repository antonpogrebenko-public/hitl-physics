//! Conservation checks for debug builds — Phase 7 validation harness.
//!
//! These invariants run on every simulation step in debug builds to catch
//! numerical drift before it manifests as flight test failures.

use crate::config::PhysicsConfig;
use crate::quadrotor::QuadrotorState;

/// Check quaternion normalization invariant.
///
/// The quaternion must remain normalized (|q| = 1) throughout integration.
/// With `UnitQuaternion` this is guaranteed by construction, but we check
/// anyway in case the underlying storage has drifted due to numerical issues.
///
/// # Panics (debug builds only)
/// Panics if `|q| ∉ [1 - 1e-6, 1 + 1e-6]`.
#[inline]
pub fn check_quaternion_norm(state: &QuadrotorState) {
    #[cfg(debug_assertions)]
    {
        let q = state.quaternion.as_ref();
        let norm = q.norm();
        const TOLERANCE: f64 = 1e-6;
        assert!(
            (norm - 1.0).abs() < TOLERANCE,
            "Quaternion norm drift: |q| = {} (expected 1.0 ± {})",
            norm, TOLERANCE
        );
    }
}

/// Check that motor speeds are non-negative and bounded.
///
/// Negative motor speeds are physically impossible. Speeds exceeding the
/// voltage-limited maximum indicate a bug in the motor model or integrator.
///
/// # Panics (debug builds only)
/// Panics if any motor speed is negative or exceeds 1.5× the voltage-limited max.
#[inline]
pub fn check_motor_speeds(state: &QuadrotorState, config: &PhysicsConfig) {
    #[cfg(debug_assertions)]
    {
        let max_omega = config.max_motor_speed_from_voltage() * 1.5; // 50% margin for transients
        for (i, &omega) in state.motor_speeds.iter().enumerate() {
            assert!(
                omega >= 0.0,
                "Motor {} speed negative: {} rad/s",
                i + 1, omega
            );
            assert!(
                omega <= max_omega,
                "Motor {} speed {} rad/s exceeds max {} rad/s",
                i + 1, omega, max_omega
            );
        }
    }
}

/// Check that motor temperatures are physically reasonable.
///
/// Temperatures should stay between ambient and shutdown. Values outside this
/// range indicate thermal model drift or initialization bugs.
///
/// # Panics (debug builds only)
/// Panics if any motor temperature is below ambient - 10°C or above shutdown + 50°C.
#[inline]
pub fn check_motor_temperatures(state: &QuadrotorState, config: &PhysicsConfig) {
    #[cfg(debug_assertions)]
    {
        let min_temp = config.ambient_temp_c - 10.0;
        let max_temp = config.motor_shutdown_temp_c + 50.0;
        for (i, &temp) in state.motor_temps.iter().enumerate() {
            assert!(
                temp >= min_temp && temp <= max_temp,
                "Motor {} temperature {} °C out of bounds [{}, {}]",
                i + 1, temp, min_temp, max_temp
            );
        }
    }
}

/// Check that position components are finite and within simulation bounds.
///
/// Infinite or NaN positions indicate catastrophic numerical failure.
/// Positions beyond 10 km suggest the simulation has diverged.
///
/// # Panics (debug builds only)
/// Panics if any position component is non-finite or exceeds 10 km magnitude.
#[inline]
pub fn check_position_bounds(state: &QuadrotorState) {
    #[cfg(debug_assertions)]
    {
        const MAX_DISTANCE_M: f64 = 10_000.0;
        for (i, &p) in state.position.iter().enumerate() {
            let axis = ["X", "Y", "Z"][i];
            assert!(
                p.is_finite(),
                "Position {} is not finite: {}",
                axis, p
            );
            assert!(
                p.abs() <= MAX_DISTANCE_M,
                "Position {} = {} m exceeds {} m simulation bounds",
                axis, p, MAX_DISTANCE_M
            );
        }
    }
}

/// Check that velocity components are finite and within physical limits.
///
/// Terminal velocity for a quad is typically <100 m/s; exceeding 200 m/s
/// indicates either a bug or cosmic ray bit flip.
///
/// # Panics (debug builds only)
/// Panics if any velocity component is non-finite or exceeds 200 m/s magnitude.
#[inline]
pub fn check_velocity_bounds(state: &QuadrotorState) {
    #[cfg(debug_assertions)]
    {
        const MAX_VELOCITY_MPS: f64 = 200.0;
        for (i, &v) in state.velocity.iter().enumerate() {
            let axis = ["Vx", "Vy", "Vz"][i];
            assert!(
                v.is_finite(),
                "Velocity {} is not finite: {}",
                axis, v
            );
            assert!(
                v.abs() <= MAX_VELOCITY_MPS,
                "Velocity {} = {} m/s exceeds {} m/s physical limit",
                axis, v, MAX_VELOCITY_MPS
            );
        }
    }
}

/// Check that angular velocity is finite and within gyro saturation limits.
///
/// Most MEMS gyros saturate at ~2000°/s (35 rad/s). Exceeding this suggests
/// numerical instability or unrealistic physics.
///
/// # Panics (debug builds only)
/// Panics if any angular velocity component is non-finite or exceeds 50 rad/s.
#[inline]
pub fn check_angular_velocity_bounds(state: &QuadrotorState) {
    #[cfg(debug_assertions)]
    {
        const MAX_OMEGA_RADS: f64 = 50.0; // ~2865°/s, well beyond gyro saturation
        for (i, &w) in state.angular_velocity.iter().enumerate() {
            let axis = ["P", "Q", "R"][i];
            assert!(
                w.is_finite(),
                "Angular velocity {} is not finite: {}",
                axis, w
            );
            assert!(
                w.abs() <= MAX_OMEGA_RADS,
                "Angular velocity {} = {} rad/s exceeds {} rad/s limit",
                axis, w, MAX_OMEGA_RADS
            );
        }
    }
}

/// Run all conservation checks on the current state.
///
/// Call this after each RK4 step in debug builds. In release builds, all
/// checks are no-ops (zero runtime cost).
#[inline]
pub fn check_all(state: &QuadrotorState, config: &PhysicsConfig) {
    check_quaternion_norm(state);
    check_motor_speeds(state, config);
    check_motor_temperatures(state, config);
    check_position_bounds(state);
    check_velocity_bounds(state);
    check_angular_velocity_bounds(state);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_state() -> QuadrotorState {
        QuadrotorState::default()
    }

    fn default_config() -> PhysicsConfig {
        PhysicsConfig::default()
    }

    #[test]
    fn valid_state_passes_all_checks() {
        let state = default_state();
        let config = default_config();
        check_all(&state, &config);
    }

    #[test]
    fn hover_state_passes_all_checks() {
        let config = default_config();
        let state = QuadrotorState::at_hover(&config);
        check_all(&state, &config);
    }

    #[test]
    fn normalized_quaternion_passes() {
        // UnitQuaternion is always normalized by construction, so we just
        // verify that valid states pass the check.
        let state = default_state();
        check_quaternion_norm(&state);
    }

    #[test]
    #[should_panic(expected = "Motor 1 speed negative")]
    #[cfg(debug_assertions)]
    fn negative_motor_speed_fails() {
        let config = default_config();
        let mut state = default_state();
        state.motor_speeds[0] = -100.0;
        check_motor_speeds(&state, &config);
    }

    #[test]
    #[should_panic(expected = "exceeds max")]
    #[cfg(debug_assertions)]
    fn excessive_motor_speed_fails() {
        let config = default_config();
        let mut state = default_state();
        state.motor_speeds[0] = 100_000.0; // Way beyond voltage limit
        check_motor_speeds(&state, &config);
    }

    #[test]
    #[should_panic(expected = "Position X is not finite")]
    #[cfg(debug_assertions)]
    fn nan_position_fails() {
        let mut state = default_state();
        state.position[0] = f64::NAN;
        check_position_bounds(&state);
    }

    #[test]
    #[should_panic(expected = "exceeds")]
    #[cfg(debug_assertions)]
    fn extreme_position_fails() {
        let mut state = default_state();
        state.position[0] = 50_000.0; // 50 km
        check_position_bounds(&state);
    }

    #[test]
    #[should_panic(expected = "Velocity Vz")]
    #[cfg(debug_assertions)]
    fn extreme_velocity_fails() {
        let mut state = default_state();
        state.velocity[2] = 500.0; // 500 m/s
        check_velocity_bounds(&state);
    }

    #[test]
    #[should_panic(expected = "Angular velocity R")]
    #[cfg(debug_assertions)]
    fn extreme_angular_velocity_fails() {
        let mut state = default_state();
        state.angular_velocity[2] = 100.0; // 100 rad/s
        check_angular_velocity_bounds(&state);
    }
}
