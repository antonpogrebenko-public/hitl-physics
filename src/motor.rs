//! Motor dynamics model with first-order response.

use crate::config::PhysicsConfig;

/// Maximum motor speed in rad/s (typical for 2306 motor at full throttle)
pub const MAX_MOTOR_SPEED: f64 = 2500.0;

/// Minimum motor speed in rad/s (idle)
pub const MIN_MOTOR_SPEED: f64 = 100.0;

/// Convert throttle command (0.0 to 1.0) to commanded motor speed (rad/s).
///
/// # Arguments
/// * `throttle` - Normalized throttle command [0.0, 1.0]
///
/// # Returns
/// Commanded motor speed in rad/s
pub fn throttle_to_omega(throttle: f64) -> f64 {
    let clamped = throttle.clamp(0.0, 1.0);
    MIN_MOTOR_SPEED + clamped * (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED)
}

/// Compute motor speed derivative using first-order dynamics.
///
/// ω̇ = (ω_cmd - ω) / τ_motor
///
/// # Arguments
/// * `current_speed` - Current motor speed in rad/s
/// * `commanded_speed` - Commanded motor speed in rad/s
/// * `config` - Physics configuration containing tau_motor
///
/// # Returns
/// Rate of change of motor speed in rad/s²
pub fn motor_derivative(current_speed: f64, commanded_speed: f64, config: &PhysicsConfig) -> f64 {
    (commanded_speed - current_speed) / config.tau_motor
}

/// Compute thrust force from motor speed.
///
/// F = kt * ω²
///
/// # Arguments
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration containing kt
///
/// # Returns
/// Thrust force in Newtons (positive = upward in body frame)
pub fn compute_thrust(motor_speed: f64, config: &PhysicsConfig) -> f64 {
    config.kt * motor_speed * motor_speed
}

/// Compute reaction torque from motor speed.
///
/// τ = kq * ω²
///
/// # Arguments
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration containing kq
///
/// # Returns
/// Reaction torque magnitude in N·m
pub fn compute_reaction_torque(motor_speed: f64, config: &PhysicsConfig) -> f64 {
    config.kq * motor_speed * motor_speed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_throttle_to_omega_bounds() {
        assert!((throttle_to_omega(0.0) - MIN_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(1.0) - MAX_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(0.5) - (MIN_MOTOR_SPEED + MAX_MOTOR_SPEED) / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_throttle_clamping() {
        assert!((throttle_to_omega(-0.5) - MIN_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(1.5) - MAX_MOTOR_SPEED).abs() < 1e-10);
    }

    #[test]
    fn test_motor_derivative() {
        let config = PhysicsConfig::default();

        // At equilibrium, derivative should be zero
        let deriv = motor_derivative(1000.0, 1000.0, &config);
        assert!(deriv.abs() < 1e-10);

        // When commanded > current, derivative should be positive
        let deriv = motor_derivative(1000.0, 1500.0, &config);
        assert!(deriv > 0.0);

        // Verify magnitude
        let expected = (1500.0 - 1000.0) / config.tau_motor;
        assert!((deriv - expected).abs() < 1e-10);
    }

    #[test]
    fn test_compute_thrust() {
        let config = PhysicsConfig::default();
        let omega = 1000.0;
        let thrust = compute_thrust(omega, &config);
        let expected = config.kt * omega * omega;
        assert!((thrust - expected).abs() < 1e-10);
    }

    #[test]
    fn test_compute_reaction_torque() {
        let config = PhysicsConfig::default();
        let omega = 1000.0;
        let torque = compute_reaction_torque(omega, &config);
        let expected = config.kq * omega * omega;
        assert!((torque - expected).abs() < 1e-10);
    }
}
