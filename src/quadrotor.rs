//! Quadrotor state vector and rigid body dynamics.

use crate::config::PhysicsConfig;
use crate::motor::{compute_reaction_torque, compute_thrust};
use nalgebra::{UnitQuaternion, Vector3};

/// Complete state of a quadrotor in NED frame.
///
/// Motor layout (top-down view, X configuration, matches PX4 after remapping):
/// ```text
///     Front
///   1       2
///     \   /
///       X
///     /   \
///   4       3
///     Back
/// ```
/// After PX4→Sim remapping: Motors 1 and 3 spin CW, motors 2 and 4 spin CCW.
/// This matches PX4's Quad X: FL/BR are CW, FR/BL are CCW (diagonals same direction).
#[derive(Debug, Clone)]
pub struct QuadrotorState {
    /// Position in NED frame [North, East, Down] in meters
    pub position: [f64; 3],
    /// Velocity in NED frame [Vn, Ve, Vd] in m/s
    pub velocity: [f64; 3],
    /// Attitude quaternion (body to NED rotation)
    pub quaternion: UnitQuaternion<f64>,
    /// Angular velocity in body frame [p, q, r] in rad/s
    pub angular_velocity: [f64; 3],
    /// Motor speeds [ω1, ω2, ω3, ω4] in rad/s
    pub motor_speeds: [f64; 4],
}

impl Default for QuadrotorState {
    fn default() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            quaternion: UnitQuaternion::identity(),
            angular_velocity: [0.0, 0.0, 0.0],
            motor_speeds: [0.0, 0.0, 0.0, 0.0],
        }
    }
}

/// State derivative for integration.
#[derive(Debug, Clone)]
pub struct StateDerivative {
    /// Velocity (derivative of position)
    pub velocity: [f64; 3],
    /// Acceleration (derivative of velocity)
    pub acceleration: [f64; 3],
    /// Quaternion derivative
    pub quaternion_deriv: [f64; 4],
    /// Angular acceleration (derivative of angular velocity)
    pub angular_accel: [f64; 3],
    /// Motor speed derivatives
    pub motor_derivs: [f64; 4],
}

impl QuadrotorState {
    /// Create a new quadrotor state at hover.
    pub fn at_hover(config: &PhysicsConfig) -> Self {
        let hover_omega = config.hover_motor_speed();
        Self {
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            quaternion: UnitQuaternion::identity(),
            angular_velocity: [0.0, 0.0, 0.0],
            motor_speeds: [hover_omega, hover_omega, hover_omega, hover_omega],
        }
    }

    /// Compute total thrust and torques from motor speeds.
    ///
    /// Returns (thrust_body, torque_body) where:
    /// - thrust_body is [0, 0, -F] (negative Z in body frame = up)
    /// - torque_body is [τ_roll, τ_pitch, τ_yaw]
    pub fn compute_motor_forces(&self, config: &PhysicsConfig) -> (Vector3<f64>, Vector3<f64>) {
        let [w1, w2, w3, w4] = self.motor_speeds;
        let l = config.arm_length_m;

        // Compute individual thrusts
        let f1 = compute_thrust(w1, config);
        let f2 = compute_thrust(w2, config);
        let f3 = compute_thrust(w3, config);
        let f4 = compute_thrust(w4, config);

        // Total thrust (in body frame, -Z is up)
        let total_thrust = f1 + f2 + f3 + f4;
        let thrust_body = Vector3::new(0.0, 0.0, -total_thrust);

        // Roll torque (about body X axis, FRD convention: positive = right wing down)
        // Upward thrust on left (motors 1,4) lifts left side → right wing down → positive roll
        let cos45 = std::f64::consts::FRAC_1_SQRT_2;
        let tau_roll = l * cos45 * (f1 + f4 - f2 - f3);

        // Pitch torque (about body Y axis, FRD convention: positive = nose up)
        // Upward thrust on front (motors 1,2) lifts front → nose up → positive pitch
        let tau_pitch = l * cos45 * (f1 + f2 - f3 - f4);

        // Yaw torque from motor reaction torques
        // After PX4→Sim remapping: Motors 1,3 spin CW (negative torque), motors 2,4 spin CCW (positive torque)
        // CCW prop creates CW reaction on frame (positive yaw in NED/FRD)
        // CW prop creates CCW reaction on frame (negative yaw in NED/FRD)
        let q1 = compute_reaction_torque(w1, config);
        let q2 = compute_reaction_torque(w2, config);
        let q3 = compute_reaction_torque(w3, config);
        let q4 = compute_reaction_torque(w4, config);
        let tau_yaw = -q1 + q2 - q3 + q4;

        let torque_body = Vector3::new(tau_roll, tau_pitch, tau_yaw);

        (thrust_body, torque_body)
    }

    /// Compute drag force in body frame.
    pub fn compute_drag(&self, config: &PhysicsConfig) -> Vector3<f64> {
        // Transform velocity to body frame
        let vel_ned = Vector3::new(self.velocity[0], self.velocity[1], self.velocity[2]);
        let vel_body = self.quaternion.inverse() * vel_ned;

        // Quadratic drag: F = -c * v * |v|
        let drag_x = -config.drag_coeffs[0] * vel_body.x * vel_body.x.abs();
        let drag_y = -config.drag_coeffs[1] * vel_body.y * vel_body.y.abs();
        let drag_z = -config.drag_coeffs[2] * vel_body.z * vel_body.z.abs();

        Vector3::new(drag_x, drag_y, drag_z)
    }

    /// Compute total forces in NED frame.
    pub fn compute_forces(&self, config: &PhysicsConfig) -> Vector3<f64> {
        let (thrust_body, _) = self.compute_motor_forces(config);
        let drag_body = self.compute_drag(config);

        // Total force in body frame
        let force_body = thrust_body + drag_body;

        // Transform to NED frame
        let force_ned = self.quaternion * force_body;

        // Add gravity (positive Z in NED = down)
        let gravity = Vector3::new(0.0, 0.0, config.mass_kg * config.gravity);

        force_ned + gravity
    }

    /// Compute total torques in body frame.
    pub fn compute_torques(&self, config: &PhysicsConfig) -> Vector3<f64> {
        let (_, torque_body) = self.compute_motor_forces(config);
        torque_body
    }

    /// Compute the full state derivative.
    ///
    /// # Arguments
    /// * `config` - Physics configuration
    /// * `motor_commands` - Commanded motor speeds [rad/s]
    pub fn state_derivative(
        &self,
        config: &PhysicsConfig,
        motor_commands: [f64; 4],
    ) -> StateDerivative {
        // Position derivative = velocity
        let velocity = self.velocity;

        // Acceleration = F / m
        let force_ned = self.compute_forces(config);
        let acceleration = [
            force_ned.x / config.mass_kg,
            force_ned.y / config.mass_kg,
            force_ned.z / config.mass_kg,
        ];

        // Quaternion derivative
        // q̇ = 0.5 * q ⊗ ω_quat
        // where ω_quat = [0, p, q, r]
        let p = self.angular_velocity[0];
        let q = self.angular_velocity[1];
        let r = self.angular_velocity[2];
        let quat = self.quaternion.as_ref();
        let qw = quat.w;
        let qx = quat.i;
        let qy = quat.j;
        let qz = quat.k;

        let quaternion_deriv = [
            0.5 * (-qx * p - qy * q - qz * r), // dw
            0.5 * (qw * p + qy * r - qz * q),  // dx
            0.5 * (qw * q + qz * p - qx * r),  // dy
            0.5 * (qw * r + qx * q - qy * p),  // dz
        ];

        // Angular acceleration from Euler's equations
        // I * ω̇ = τ - ω × (I * ω)
        let torque = self.compute_torques(config);
        let [ixx, iyy, izz] = config.inertia;
        let omega = Vector3::new(p, q, r);
        let i_omega = Vector3::new(ixx * p, iyy * q, izz * r);
        let gyro_term = omega.cross(&i_omega);

        let angular_accel = [
            (torque.x - gyro_term.x) / ixx,
            (torque.y - gyro_term.y) / iyy,
            (torque.z - gyro_term.z) / izz,
        ];

        // Motor dynamics
        let motor_derivs = [
            (motor_commands[0] - self.motor_speeds[0]) / config.tau_motor,
            (motor_commands[1] - self.motor_speeds[1]) / config.tau_motor,
            (motor_commands[2] - self.motor_speeds[2]) / config.tau_motor,
            (motor_commands[3] - self.motor_speeds[3]) / config.tau_motor,
        ];

        StateDerivative {
            velocity,
            acceleration,
            quaternion_deriv,
            angular_accel,
            motor_derivs,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_default_state() {
        let state = QuadrotorState::default();
        assert_eq!(state.position, [0.0, 0.0, 0.0]);
        assert_eq!(state.velocity, [0.0, 0.0, 0.0]);
        assert_eq!(state.motor_speeds, [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_hover_state() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let hover_omega = config.hover_motor_speed();

        for &omega in &state.motor_speeds {
            assert_relative_eq!(omega, hover_omega, epsilon = 1e-6);
        }
    }

    #[test]
    fn test_hover_forces() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let forces = state.compute_forces(&config);

        // At hover, net force should be approximately zero
        assert_relative_eq!(forces.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(forces.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(forces.z, 0.0, epsilon = 1e-3); // Small tolerance for numerical precision
    }

    #[test]
    fn test_hover_torques() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::at_hover(&config);
        let torques = state.compute_torques(&config);

        // At hover with equal motor speeds, torques should be zero
        assert_relative_eq!(torques.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(torques.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(torques.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_state_derivative_at_rest() {
        let config = PhysicsConfig::default();
        let state = QuadrotorState::default();
        let deriv = state.state_derivative(&config, [0.0, 0.0, 0.0, 0.0]);

        // Velocity derivative should be zero (no velocity)
        assert_eq!(deriv.velocity, [0.0, 0.0, 0.0]);

        // Acceleration should be gravity only (falling)
        assert_relative_eq!(deriv.acceleration[2], config.gravity, epsilon = 1e-6);
    }
}
