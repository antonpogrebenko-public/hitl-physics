//! Physical parameters for quadrotor simulation.

/// Physical configuration for a quadrotor.
///
/// Default values are for a typical 5-inch racing quad.
#[derive(Debug, Clone)]
pub struct PhysicsConfig {
    /// Total mass in kg
    pub mass_kg: f64,
    /// Arm length from center to motor in meters
    pub arm_length_m: f64,
    /// Moments of inertia [Ixx, Iyy, Izz] in kg·m²
    pub inertia: [f64; 3],
    /// Thrust coefficient: thrust = kt * ω²
    pub kt: f64,
    /// Torque coefficient: torque = kq * ω²
    pub kq: f64,
    /// Motor time constant in seconds (first-order response)
    pub tau_motor: f64,
    /// Drag coefficients [lateral_x, lateral_y, vertical_z]
    pub drag_coeffs: [f64; 3],
    /// Gravitational acceleration in m/s²
    pub gravity: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        // Typical 5" FPV quad, ~2kg AUW
        // Motors: 2306 size, ~1200 KV, 5" props
        // Max thrust per motor: ~12N at 2500 rad/s
        Self {
            mass_kg: 2.0,
            arm_length_m: 0.11,              // 220mm diagonal frame
            inertia: [0.015, 0.015, 0.025],   // kg·m² (balanced for PX4 default PIDs - higher Izz reduces yaw oscillation)
            kt: 1.9e-6,                       // thrust = kt * ω², gives ~12N at 2500 rad/s
            kq: 5.0e-8,                       // torque = kq * ω², kq/kt ≈ 0.026 (aggressive 5" prop)
            tau_motor: 0.025,                 // 25ms motor time constant
            drag_coeffs: [0.25, 0.25, 0.15],
            gravity: 9.80665,
        }
    }
}

impl PhysicsConfig {
    /// Create a new physics config with custom parameters.
    pub fn new(
        mass_kg: f64,
        arm_length_m: f64,
        inertia: [f64; 3],
        kt: f64,
        kq: f64,
        tau_motor: f64,
        drag_coeffs: [f64; 3],
        gravity: f64,
    ) -> Self {
        Self {
            mass_kg,
            arm_length_m,
            inertia,
            kt,
            kq,
            tau_motor,
            drag_coeffs,
            gravity,
        }
    }

    /// Calculate hover throttle for this configuration.
    ///
    /// Returns the motor speed (rad/s) needed per motor to hover.
    pub fn hover_motor_speed(&self) -> f64 {
        // At hover: 4 * kt * ω² = m * g
        // ω = sqrt(m * g / (4 * kt))
        ((self.mass_kg * self.gravity) / (4.0 * self.kt)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = PhysicsConfig::default();
        assert!((config.mass_kg - 2.0).abs() < 1e-10);  // 2kg quad
        assert!((config.gravity - 9.80665).abs() < 1e-10);
    }

    #[test]
    fn test_hover_motor_speed() {
        let config = PhysicsConfig::default();
        let omega = config.hover_motor_speed();
        // Verify: 4 * kt * ω² ≈ m * g
        let thrust = 4.0 * config.kt * omega * omega;
        let weight = config.mass_kg * config.gravity;
        assert!((thrust - weight).abs() < 1e-6);
    }
}
