//! RK4 integration for quadrotor dynamics.

use crate::config::PhysicsConfig;
use crate::quadrotor::QuadrotorState;
use nalgebra::{Quaternion, UnitQuaternion};

/// Default integration timestep (400 Hz)
pub const DEFAULT_DT: f64 = 0.0025;

/// Perform one RK4 integration step.
///
/// # Arguments
/// * `state` - Current quadrotor state
/// * `config` - Physics configuration
/// * `motor_commands` - Commanded motor speeds [rad/s]
/// * `dt` - Integration timestep in seconds
///
/// # Returns
/// New quadrotor state after integration
pub fn rk4_step(
    state: &QuadrotorState,
    config: &PhysicsConfig,
    motor_commands: [f64; 4],
    dt: f64,
) -> QuadrotorState {
    // k1 = f(t, y)
    let k1 = state.state_derivative(config, motor_commands);

    // k2 = f(t + dt/2, y + dt/2 * k1)
    let state2 = apply_derivative(state, &k1, dt / 2.0);
    let k2 = state2.state_derivative(config, motor_commands);

    // k3 = f(t + dt/2, y + dt/2 * k2)
    let state3 = apply_derivative(state, &k2, dt / 2.0);
    let k3 = state3.state_derivative(config, motor_commands);

    // k4 = f(t + dt, y + dt * k3)
    let state4 = apply_derivative(state, &k3, dt);
    let k4 = state4.state_derivative(config, motor_commands);

    // y_new = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    let mut new_state = QuadrotorState::default();

    // Position
    for i in 0..3 {
        new_state.position[i] = state.position[i]
            + (dt / 6.0)
                * (k1.velocity[i] + 2.0 * k2.velocity[i] + 2.0 * k3.velocity[i] + k4.velocity[i]);
    }

    // Velocity
    for i in 0..3 {
        new_state.velocity[i] = state.velocity[i]
            + (dt / 6.0)
                * (k1.acceleration[i]
                    + 2.0 * k2.acceleration[i]
                    + 2.0 * k3.acceleration[i]
                    + k4.acceleration[i]);
    }

    // Quaternion
    let quat = state.quaternion.as_ref();
    let dq = [
        (dt / 6.0)
            * (k1.quaternion_deriv[0]
                + 2.0 * k2.quaternion_deriv[0]
                + 2.0 * k3.quaternion_deriv[0]
                + k4.quaternion_deriv[0]),
        (dt / 6.0)
            * (k1.quaternion_deriv[1]
                + 2.0 * k2.quaternion_deriv[1]
                + 2.0 * k3.quaternion_deriv[1]
                + k4.quaternion_deriv[1]),
        (dt / 6.0)
            * (k1.quaternion_deriv[2]
                + 2.0 * k2.quaternion_deriv[2]
                + 2.0 * k3.quaternion_deriv[2]
                + k4.quaternion_deriv[2]),
        (dt / 6.0)
            * (k1.quaternion_deriv[3]
                + 2.0 * k2.quaternion_deriv[3]
                + 2.0 * k3.quaternion_deriv[3]
                + k4.quaternion_deriv[3]),
    ];

    let new_quat = Quaternion::new(
        quat.w + dq[0],
        quat.i + dq[1],
        quat.j + dq[2],
        quat.k + dq[3],
    );
    new_state.quaternion = UnitQuaternion::from_quaternion(new_quat);

    // Angular velocity
    for i in 0..3 {
        new_state.angular_velocity[i] = state.angular_velocity[i]
            + (dt / 6.0)
                * (k1.angular_accel[i]
                    + 2.0 * k2.angular_accel[i]
                    + 2.0 * k3.angular_accel[i]
                    + k4.angular_accel[i]);
    }

    // Motor speeds
    for i in 0..4 {
        new_state.motor_speeds[i] = state.motor_speeds[i]
            + (dt / 6.0)
                * (k1.motor_derivs[i]
                    + 2.0 * k2.motor_derivs[i]
                    + 2.0 * k3.motor_derivs[i]
                    + k4.motor_derivs[i]);
    }

    new_state
}

/// Apply a state derivative to a state with a given timestep.
fn apply_derivative(
    state: &QuadrotorState,
    deriv: &crate::quadrotor::StateDerivative,
    dt: f64,
) -> QuadrotorState {
    let mut new_state = state.clone();

    for i in 0..3 {
        new_state.position[i] += dt * deriv.velocity[i];
        new_state.velocity[i] += dt * deriv.acceleration[i];
        new_state.angular_velocity[i] += dt * deriv.angular_accel[i];
    }

    for i in 0..4 {
        new_state.motor_speeds[i] += dt * deriv.motor_derivs[i];
    }

    // Update quaternion
    let quat = state.quaternion.as_ref();
    let new_quat = Quaternion::new(
        quat.w + dt * deriv.quaternion_deriv[0],
        quat.i + dt * deriv.quaternion_deriv[1],
        quat.j + dt * deriv.quaternion_deriv[2],
        quat.k + dt * deriv.quaternion_deriv[3],
    );
    new_state.quaternion = UnitQuaternion::from_quaternion(new_quat);

    new_state
}

/// Run simulation for multiple steps.
///
/// # Arguments
/// * `state` - Initial state
/// * `config` - Physics configuration
/// * `motor_commands` - Commanded motor speeds [rad/s]
/// * `duration` - Total simulation duration in seconds
/// * `dt` - Integration timestep in seconds
///
/// # Returns
/// Final state after simulation
pub fn simulate(
    state: &QuadrotorState,
    config: &PhysicsConfig,
    motor_commands: [f64; 4],
    duration: f64,
    dt: f64,
) -> QuadrotorState {
    let steps = (duration / dt).round() as usize;
    let mut current = state.clone();

    for _ in 0..steps {
        current = rk4_step(&current, config, motor_commands, dt);
    }

    current
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_zero_input() {
        // State at rest on ground with no motor input should remain stationary
        // (In reality would fall due to gravity, but we test the integrator works)
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let hover_omega = config.hover_motor_speed();

        // Apply hover thrust
        let motor_commands = [hover_omega, hover_omega, hover_omega, hover_omega];
        let new_state = rk4_step(&state, &config, motor_commands, DEFAULT_DT);

        // Position should stay approximately the same
        assert_relative_eq!(new_state.position[0], 0.0, epsilon = 1e-6);
        assert_relative_eq!(new_state.position[1], 0.0, epsilon = 1e-6);
        assert_relative_eq!(new_state.position[2], 0.0, epsilon = 1e-3);
    }

    #[test]
    fn test_hover_equilibrium() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let hover_omega = config.hover_motor_speed();

        // Simulate for 1 second at hover
        let motor_commands = [hover_omega, hover_omega, hover_omega, hover_omega];
        let final_state = simulate(&state, &config, motor_commands, 1.0, DEFAULT_DT);

        // Altitude should remain approximately constant (Z position near 0)
        assert_relative_eq!(final_state.position[2], 0.0, epsilon = 0.05);

        // Horizontal position should stay at origin
        assert_relative_eq!(final_state.position[0], 0.0, epsilon = 1e-3);
        assert_relative_eq!(final_state.position[1], 0.0, epsilon = 1e-3);
    }

    #[test]
    fn test_motor_response() {
        let config = PhysicsConfig::default();
        let mut state = QuadrotorState::default();
        state.motor_speeds = [0.0, 0.0, 0.0, 0.0];

        let target_omega = 1000.0;
        let motor_commands = [target_omega, target_omega, target_omega, target_omega];

        // Simulate for one time constant (50ms)
        let final_state = simulate(
            &state,
            &config,
            motor_commands,
            config.tau_motor,
            DEFAULT_DT,
        );

        // First-order response should reach ~63.2% of target after one time constant
        let expected = target_omega * (1.0 - (-1.0_f64).exp());
        for &omega in &final_state.motor_speeds {
            assert_relative_eq!(omega, expected, epsilon = 5.0);
        }
    }

    #[test]
    fn test_free_fall() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::default();

        // No motor thrust - should fall
        let motor_commands = [0.0, 0.0, 0.0, 0.0];
        let final_state = simulate(&state, &config, motor_commands, 0.1, DEFAULT_DT);

        // After 0.1s of free fall: z = 0.5 * g * t^2 = 0.5 * 9.8 * 0.01 ≈ 0.049m
        // In NED, positive Z is down
        assert!(final_state.position[2] > 0.04);
        assert!(final_state.position[2] < 0.06);
    }

    #[test]
    fn test_quaternion_normalization() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let hover_omega = config.hover_motor_speed();

        // Run many steps
        let motor_commands = [hover_omega, hover_omega, hover_omega, hover_omega];
        let final_state = simulate(&state, &config, motor_commands, 5.0, DEFAULT_DT);

        // Quaternion should remain normalized
        let norm = final_state.quaternion.norm();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-6);
    }
}
