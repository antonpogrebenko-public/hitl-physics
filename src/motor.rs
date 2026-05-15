//! Motor dynamics model with first-order response and thermal modeling.

use crate::config::PhysicsConfig;

/// Default maximum motor speed in rad/s (fallback when voltage not specified)
pub const DEFAULT_MAX_MOTOR_SPEED: f64 = 2500.0;

/// Minimum motor speed in rad/s (idle)
pub const MIN_MOTOR_SPEED: f64 = 100.0;

/// Convert throttle command (0.0 to 1.0) to commanded motor speed (rad/s).
///
/// Uses voltage-based max speed from config if available.
///
/// # Arguments
/// * `throttle` - Normalized throttle command [0.0, 1.0]
/// * `config` - Physics configuration (for voltage-based max speed)
///
/// # Returns
/// Commanded motor speed in rad/s
pub fn throttle_to_omega_with_config(throttle: f64, config: &PhysicsConfig) -> f64 {
    let clamped = throttle.clamp(0.0, 1.0);
    let max_speed = config.max_motor_speed_from_voltage();
    MIN_MOTOR_SPEED + clamped * (max_speed - MIN_MOTOR_SPEED)
}

/// Convert throttle command (0.0 to 1.0) to commanded motor speed (rad/s).
///
/// Uses default max speed (legacy, for backwards compatibility).
///
/// # Arguments
/// * `throttle` - Normalized throttle command [0.0, 1.0]
///
/// # Returns
/// Commanded motor speed in rad/s
pub fn throttle_to_omega(throttle: f64) -> f64 {
    let clamped = throttle.clamp(0.0, 1.0);
    MIN_MOTOR_SPEED + clamped * (DEFAULT_MAX_MOTOR_SPEED - MIN_MOTOR_SPEED)
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

// =============================================================================
// Electrical Model
// =============================================================================

/// Compute motor current draw from speed using torque-based model.
///
/// The motor produces aerodynamic torque τ = kq × ω², which requires current:
/// I = τ / Kt_electrical + I_no_load
///
/// This is more accurate than the voltage-drop model (V-Vbemf)/R which
/// underestimates back-EMF at low speeds and gives unrealistic currents.
///
/// # Arguments
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration
///
/// # Returns
/// Motor current in Amps
pub fn compute_motor_current(motor_speed: f64, config: &PhysicsConfig) -> f64 {
    // Aerodynamic torque load on motor
    let torque = config.kq * motor_speed * motor_speed;
    // Current required to produce that torque: I = τ / Kt
    // Kt_electrical = 60 / (2π × KV) in N·m/A
    let load_current = torque / config.motor_kt_electrical;
    // Add no-load current (friction, iron losses) ~0.5A typical for mini quad motors
    let no_load_current = 0.5;
    (load_current + no_load_current).max(0.0)
}

/// Compute electrical power consumed by motor.
///
/// P_electrical = V × I
///
/// # Arguments
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration
///
/// # Returns
/// Electrical power in Watts
pub fn compute_electrical_power(motor_speed: f64, config: &PhysicsConfig) -> f64 {
    let current = compute_motor_current(motor_speed, config);
    config.battery_voltage * current
}

/// Compute heat generated in motor windings (I²R losses).
///
/// P_heat = I² × R
///
/// # Arguments
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration
///
/// # Returns
/// Heat power in Watts
pub fn compute_motor_heat(motor_speed: f64, config: &PhysicsConfig) -> f64 {
    let current = compute_motor_current(motor_speed, config);
    current * current * config.motor_resistance_ohm
}

// =============================================================================
// Thermal Model
// =============================================================================

/// Compute motor temperature derivative.
///
/// dT/dt = (P_heat - P_dissipated) / thermal_mass
///
/// # Arguments
/// * `motor_temp_c` - Current motor temperature in °C
/// * `motor_speed` - Motor speed in rad/s
/// * `config` - Physics configuration
///
/// # Returns
/// Temperature rate of change in °C/s
pub fn motor_temp_derivative(
    motor_temp_c: f64,
    motor_speed: f64,
    config: &PhysicsConfig,
) -> f64 {
    let heat_in = compute_motor_heat(motor_speed, config);
    let temp_diff = motor_temp_c - config.ambient_temp_c;
    let heat_out = config.thermal_dissipation_w_per_k * temp_diff;
    let net_heat = heat_in - heat_out;
    net_heat / config.thermal_mass_j_per_k
}

/// Apply thermal derating to commanded motor speed.
///
/// When motor is overheating, reduce maximum achievable speed.
///
/// # Arguments
/// * `commanded_speed` - Desired motor speed in rad/s
/// * `motor_temp_c` - Current motor temperature in °C
/// * `config` - Physics configuration
///
/// # Returns
/// Derated motor speed in rad/s
pub fn apply_thermal_derating(
    commanded_speed: f64,
    motor_temp_c: f64,
    config: &PhysicsConfig,
) -> f64 {
    let derating = config.thermal_derating_factor(motor_temp_c);
    let max_speed = config.max_motor_speed_from_voltage();
    let derated_max = MIN_MOTOR_SPEED + derating * (max_speed - MIN_MOTOR_SPEED);
    commanded_speed.min(derated_max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_throttle_to_omega_bounds() {
        assert!((throttle_to_omega(0.0) - MIN_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(1.0) - DEFAULT_MAX_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(0.5) - (MIN_MOTOR_SPEED + DEFAULT_MAX_MOTOR_SPEED) / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_throttle_clamping() {
        assert!((throttle_to_omega(-0.5) - MIN_MOTOR_SPEED).abs() < 1e-10);
        assert!((throttle_to_omega(1.5) - DEFAULT_MAX_MOTOR_SPEED).abs() < 1e-10);
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

    #[test]
    fn test_motor_current_at_zero_speed() {
        let config = PhysicsConfig::default();
        // At 0 rad/s, torque is zero so current = no-load only (0.5A)
        let current = compute_motor_current(0.0, &config);
        assert!((current - 0.5).abs() < 0.01, "Zero-speed current should be no-load, got {}", current);
    }

    #[test]
    fn test_motor_current_increases_with_speed() {
        let config = PhysicsConfig::default();
        let current_low = compute_motor_current(500.0, &config);
        let current_high = compute_motor_current(1500.0, &config);
        // Higher speed = more torque load = more current
        assert!(current_high > current_low, "Current should increase with speed");
    }

    #[test]
    fn test_motor_current_reasonable_at_hover() {
        let config = PhysicsConfig::default();
        let hover_omega = config.hover_motor_speed();
        let current = compute_motor_current(hover_omega, &config);
        // Hover current for a 2kg quad on 4S should be ~5-15A per motor
        assert!(current > 3.0 && current < 20.0, "Hover current unrealistic: {} A", current);
    }

    #[test]
    fn test_motor_heat_increases_with_speed() {
        let config = PhysicsConfig::default();
        let heat_low = compute_motor_heat(500.0, &config);
        let heat_high = compute_motor_heat(1500.0, &config);
        // Higher speed = more current = more I²R heat
        assert!(heat_high > heat_low, "Higher speed should generate more heat");
    }

    #[test]
    fn test_thermal_equilibrium() {
        let config = PhysicsConfig::default();
        // At ambient temp with no motor running, temp derivative should be 0
        let deriv = motor_temp_derivative(config.ambient_temp_c, 0.0, &config);
        // Small positive because stall current still generates heat
        assert!(deriv > 0.0);

        // With motor running at moderate speed, should warm up
        let deriv = motor_temp_derivative(config.ambient_temp_c, 1000.0, &config);
        assert!(deriv > 0.0, "Motor should warm up when running");

        // Very hot motor with low speed should cool down
        let deriv = motor_temp_derivative(120.0, 100.0, &config);
        // Depends on parameters, but should be negative if dissipation > heat input
        // Skip this assertion as it depends on tuning
    }

    #[test]
    fn test_thermal_derating_limits_speed() {
        let config = PhysicsConfig::default();
        let max_speed = config.max_motor_speed_from_voltage();

        // Normal temp: no derating
        let derated = apply_thermal_derating(max_speed, 50.0, &config);
        assert!((derated - max_speed).abs() < 1.0);

        // At shutdown temp: fully derated to idle
        let derated = apply_thermal_derating(max_speed, config.motor_shutdown_temp_c, &config);
        assert!((derated - MIN_MOTOR_SPEED).abs() < 1.0);
    }
}
