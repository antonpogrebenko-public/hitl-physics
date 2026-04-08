//! HITL Physics Engine for Quadrotor Simulation
//!
//! A physics simulation engine for quadrotor dynamics, designed for
//! Hardware-In-The-Loop (HITL) testing. Compiles to both native and WASM.
//!
//! # Features
//!
//! - Rigid body dynamics with 6-DOF motion
//! - First-order motor response model
//! - RK4 integration at 400 Hz
//! - NED (North-East-Down) coordinate frame
//!
//! # Example
//!
//! ```
//! use hitl_physics::{PhysicsConfig, QuadrotorState, rk4_step, DEFAULT_DT};
//!
//! let config = PhysicsConfig::default();
//! let state = QuadrotorState::at_hover(&config);
//! let hover_omega = config.hover_motor_speed();
//!
//! // Step simulation with hover thrust
//! let motor_commands = [hover_omega; 4];
//! let new_state = rk4_step(&state, &config, motor_commands, DEFAULT_DT);
//! ```

pub mod config;
pub mod integrator;
pub mod motor;
pub mod quadrotor;

// Re-export main types
pub use config::PhysicsConfig;
pub use integrator::{rk4_step, simulate, DEFAULT_DT};
pub use motor::{
    compute_reaction_torque, compute_thrust, motor_derivative, throttle_to_omega, MAX_MOTOR_SPEED,
    MIN_MOTOR_SPEED,
};
pub use quadrotor::{QuadrotorState, StateDerivative};

/// WASM bindings (enabled with "wasm" feature)
#[cfg(feature = "wasm")]
pub mod wasm_bindings;

#[cfg(feature = "wasm")]
pub use wasm_bindings::WasmSimulator;

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_integration_workflow() {
        // Test the typical usage workflow
        let config = PhysicsConfig::default();
        let mut state = QuadrotorState::at_hover(&config);
        let hover_omega = config.hover_motor_speed();

        // Simulate 100ms of hover
        for _ in 0..40 {
            state = rk4_step(&state, &config, [hover_omega; 4], DEFAULT_DT);
        }

        // Should remain stable
        assert_relative_eq!(state.position[2], 0.0, epsilon = 0.01);
    }
}
