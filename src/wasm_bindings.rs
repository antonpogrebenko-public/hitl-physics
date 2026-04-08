//! WebAssembly bindings for browser-based quadrotor simulation.
//!
//! This module provides WASM-exported types and functions that allow
//! the physics engine to run in web browsers via WebAssembly.

#![cfg(feature = "wasm")]

use wasm_bindgen::prelude::*;

use crate::motor::throttle_to_omega;
use crate::quadrotor::QuadrotorState;
use crate::{rk4_step, PhysicsConfig, DEFAULT_DT};

/// Maximum attitude angle (radians) before considering crashed (~60 degrees)
const MAX_ATTITUDE_ANGLE: f64 = 1.047;

/// WASM-exported simulator that runs in the browser.
///
/// Maintains simulation state and provides methods for stepping
/// the physics forward in time.
#[wasm_bindgen]
pub struct WasmSimulator {
    state: QuadrotorState,
    config: PhysicsConfig,
    time_us: u64,
}

#[wasm_bindgen]
impl WasmSimulator {
    /// Create a new simulator with default configuration.
    ///
    /// Initializes the quadrotor at hover with zero position and velocity.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        Self {
            state,
            config,
            time_us: 0,
        }
    }

    /// Advance simulation by dt_us microseconds.
    ///
    /// # Arguments
    /// * `motor_commands` - 4 floats [0-1] throttle for each motor
    /// * `dt_us` - Timestep in microseconds
    ///
    /// # Returns
    /// Packed state as Float32Array with 17 elements:
    /// - [0-2]: position NED (meters)
    /// - [3-5]: velocity NED (m/s)
    /// - [6-9]: quaternion [w, x, y, z]
    /// - [10-12]: angular velocity (rad/s)
    /// - [13-16]: motor speeds (rad/s)
    #[wasm_bindgen]
    pub fn step(&mut self, motor_commands: &[f32], dt_us: u32) -> Vec<f32> {
        // Convert throttle commands [0-1] to motor speeds [rad/s]
        let omega_commands: [f64; 4] = [
            throttle_to_omega(motor_commands.get(0).copied().unwrap_or(0.0) as f64),
            throttle_to_omega(motor_commands.get(1).copied().unwrap_or(0.0) as f64),
            throttle_to_omega(motor_commands.get(2).copied().unwrap_or(0.0) as f64),
            throttle_to_omega(motor_commands.get(3).copied().unwrap_or(0.0) as f64),
        ];

        // Convert microseconds to seconds
        let dt_s = (dt_us as f64) / 1_000_000.0;

        // Use multiple RK4 steps if dt is larger than default
        let num_steps = ((dt_s / DEFAULT_DT).ceil() as usize).max(1);
        let step_dt = dt_s / (num_steps as f64);

        for _ in 0..num_steps {
            self.state = rk4_step(&self.state, &self.config, omega_commands, step_dt);
        }

        self.time_us += dt_us as u64;

        self.pack_state()
    }

    /// Reset the simulation to initial hover state.
    #[wasm_bindgen]
    pub fn reset(&mut self) {
        self.state = QuadrotorState::at_hover(&self.config);
        self.time_us = 0;
    }

    /// Get current simulation time in microseconds.
    #[wasm_bindgen]
    pub fn get_time_us(&self) -> u64 {
        self.time_us
    }

    /// Check if the quadrotor has crashed.
    ///
    /// Returns true if:
    /// - Below ground (position down > 0 in NED, meaning altitude < 0)
    /// - Extreme attitude (roll or pitch > ~60 degrees)
    #[wasm_bindgen]
    pub fn is_crashed(&self) -> bool {
        // Check if below ground (NED: positive Z is down)
        // We consider crashed if altitude drops significantly below starting point
        if self.state.position[2] > 5.0 {
            return true;
        }

        // Check attitude - extract roll and pitch from quaternion
        let quat = self.state.quaternion.as_ref();
        let w = quat.w;
        let x = quat.i;
        let y = quat.j;
        let z = quat.k;

        // Compute roll and pitch angles
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            std::f64::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        roll.abs() > MAX_ATTITUDE_ANGLE || pitch.abs() > MAX_ATTITUDE_ANGLE
    }

    /// Get current position as [n, e, d] in meters.
    #[wasm_bindgen]
    pub fn get_position(&self) -> Vec<f32> {
        vec![
            self.state.position[0] as f32,
            self.state.position[1] as f32,
            self.state.position[2] as f32,
        ]
    }

    /// Get current quaternion as [w, x, y, z].
    #[wasm_bindgen]
    pub fn get_quaternion(&self) -> Vec<f32> {
        let quat = self.state.quaternion.as_ref();
        vec![
            quat.w as f32,
            quat.i as f32,
            quat.j as f32,
            quat.k as f32,
        ]
    }

    /// Get current velocity as [vn, ve, vd] in m/s.
    #[wasm_bindgen]
    pub fn get_velocity(&self) -> Vec<f32> {
        vec![
            self.state.velocity[0] as f32,
            self.state.velocity[1] as f32,
            self.state.velocity[2] as f32,
        ]
    }

    /// Get current angular velocity as [p, q, r] in rad/s.
    #[wasm_bindgen]
    pub fn get_angular_velocity(&self) -> Vec<f32> {
        vec![
            self.state.angular_velocity[0] as f32,
            self.state.angular_velocity[1] as f32,
            self.state.angular_velocity[2] as f32,
        ]
    }

    /// Get current motor speeds as [w1, w2, w3, w4] in rad/s.
    #[wasm_bindgen]
    pub fn get_motor_speeds(&self) -> Vec<f32> {
        vec![
            self.state.motor_speeds[0] as f32,
            self.state.motor_speeds[1] as f32,
            self.state.motor_speeds[2] as f32,
            self.state.motor_speeds[3] as f32,
        ]
    }

    /// Pack full state into a single Vec<f32> for efficient transfer.
    ///
    /// Layout (17 floats):
    /// - [0-2]: position NED
    /// - [3-5]: velocity NED
    /// - [6-9]: quaternion [w, x, y, z]
    /// - [10-12]: angular velocity
    /// - [13-16]: motor speeds
    fn pack_state(&self) -> Vec<f32> {
        let quat = self.state.quaternion.as_ref();
        vec![
            // Position [0-2]
            self.state.position[0] as f32,
            self.state.position[1] as f32,
            self.state.position[2] as f32,
            // Velocity [3-5]
            self.state.velocity[0] as f32,
            self.state.velocity[1] as f32,
            self.state.velocity[2] as f32,
            // Quaternion [6-9]
            quat.w as f32,
            quat.i as f32,
            quat.j as f32,
            quat.k as f32,
            // Angular velocity [10-12]
            self.state.angular_velocity[0] as f32,
            self.state.angular_velocity[1] as f32,
            self.state.angular_velocity[2] as f32,
            // Motor speeds [13-16]
            self.state.motor_speeds[0] as f32,
            self.state.motor_speeds[1] as f32,
            self.state.motor_speeds[2] as f32,
            self.state.motor_speeds[3] as f32,
        ]
    }
}

impl Default for WasmSimulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wasm_simulator_new() {
        let sim = WasmSimulator::new();
        assert_eq!(sim.time_us, 0);
        assert!(!sim.is_crashed());
    }

    #[test]
    fn test_wasm_simulator_step() {
        let mut sim = WasmSimulator::new();
        let motor_commands = [0.5_f32, 0.5, 0.5, 0.5];
        let state = sim.step(&motor_commands, 2500); // 2.5ms = one default step

        assert_eq!(state.len(), 17);
        assert_eq!(sim.time_us, 2500);
    }

    #[test]
    fn test_wasm_simulator_reset() {
        let mut sim = WasmSimulator::new();
        let motor_commands = [0.5_f32, 0.5, 0.5, 0.5];
        sim.step(&motor_commands, 10000);

        sim.reset();
        assert_eq!(sim.time_us, 0);
    }

    #[test]
    fn test_pack_state_layout() {
        let sim = WasmSimulator::new();
        let state = sim.pack_state();

        assert_eq!(state.len(), 17);
        // Quaternion w should be 1.0 for identity
        assert!((state[6] - 1.0).abs() < 1e-6);
    }
}
