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

    pub fn from_motor_specs(
        kv: f64,
        prop_diameter_inches: f64,
        frame_weight_g: f64,
        motor_weight_g: f64,
    ) -> Self {
        Self::from_build_specs(kv, prop_diameter_inches, prop_diameter_inches * 0.9, 3, frame_weight_g, motor_weight_g)
    }

    pub fn from_build_specs(
        kv: f64,
        prop_diameter_inches: f64,
        prop_pitch_inches: f64,
        blade_count: i32,
        frame_weight_g: f64,
        motor_weight_g: f64,
    ) -> Self {
        // Base prop factor from diameter (empirical lookup)
        let base_prop_factor = match prop_diameter_inches as u32 {
            0..=4 => 0.8e-6,
            5 => 1.9e-6,
            6 => 3.2e-6,
            _ => 5.0e-6,
        };

        // Pitch multiplier: higher pitch = more thrust (linear approximation)
        // Reference: 5x4.5 prop as baseline (~1.0)
        let pitch_multiplier = 0.7 + 0.06 * prop_pitch_inches;

        // Blade count multiplier: more blades = more thrust but diminishing returns
        // 2-blade = 0.95, 3-blade = 1.0, 4-blade = 1.05, 5-blade = 1.10
        let blade_multiplier = 0.85 + 0.05 * (blade_count as f64);

        let prop_factor = base_prop_factor * pitch_multiplier * blade_multiplier;

        let kt = prop_factor * (2300.0 / kv).powi(2);
        let kq = kt * 0.026;
        let tau_motor = 0.02 + (1500.0 / kv) * 0.01;
        let mass_kg = (frame_weight_g + 4.0 * motor_weight_g) / 1000.0;
        let arm_length_m = prop_diameter_inches * 0.0254 * 1.1;
        let ixx = mass_kg * arm_length_m * arm_length_m * 0.5;
        let iyy = ixx;
        let izz = mass_kg * arm_length_m * arm_length_m * 0.8;

        Self {
            mass_kg,
            arm_length_m,
            inertia: [ixx, iyy, izz],
            kt,
            kq,
            tau_motor,
            drag_coeffs: [0.25, 0.25, 0.15],
            gravity: 9.80665,
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

    #[test]
    fn from_motor_specs_2306_1700kv_5inch() {
        let config = PhysicsConfig::from_motor_specs(1700.0, 5.0, 350.0, 33.0);

        assert!((config.mass_kg - 0.482).abs() < 1e-6);
        assert!((config.arm_length_m - 0.1397).abs() < 1e-4);
        assert!(config.kt > 3.0e-6 && config.kt < 4.0e-6);
        assert!((config.kq - config.kt * 0.026).abs() < 1e-12);
        assert!(config.tau_motor > 0.025 && config.tau_motor < 0.035);
        assert!(config.inertia[2] > config.inertia[0]);
        assert!(config.inertia[0] > 0.0);
    }

    #[test]
    fn from_motor_specs_high_kv_small_prop() {
        let config_small = PhysicsConfig::from_motor_specs(2500.0, 4.0, 200.0, 25.0);
        let config_5inch = PhysicsConfig::from_motor_specs(1700.0, 5.0, 350.0, 33.0);

        assert!((config_small.mass_kg - 0.3).abs() < 1e-6);
        assert!(config_small.kt < config_5inch.kt);
        assert!(config_small.tau_motor < config_5inch.tau_motor);
    }

    #[test]
    fn from_build_specs_pitch_and_blades_affect_kt() {
        // Same motor/frame, different props
        let low_pitch_2blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 3.0, 2, 350.0, 33.0);
        let high_pitch_3blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 3, 350.0, 33.0);
        let high_pitch_4blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 4, 350.0, 33.0);

        // Higher pitch = higher kt
        assert!(high_pitch_3blade.kt > low_pitch_2blade.kt);
        // More blades = higher kt
        assert!(high_pitch_4blade.kt > high_pitch_3blade.kt);
        // Mass should be the same for all
        assert!((low_pitch_2blade.mass_kg - high_pitch_4blade.mass_kg).abs() < 1e-10);
    }
}
