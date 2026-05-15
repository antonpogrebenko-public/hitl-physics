//! Physical parameters for quadrotor simulation.

/// Electronic Speed Controller configuration.
///
/// Currently used as a data carrier — Phase 2a wires `EscConfig` through the
/// BuildSpec → PhysicsConfig pipeline so the daemon has access to ESC
/// parameters per build. Loop integration (response cascade, current limit,
/// brake mode) is deferred until protocol/state changes can be coordinated
/// with the WebSocket binary format.
#[derive(Debug, Clone)]
pub struct EscConfig {
    /// Continuous current per ESC channel (Amps).
    pub continuous_amps: f64,
    /// Burst current per ESC channel (Amps).
    pub burst_amps: f64,
    /// How long burst can be sustained before falling back to continuous (s).
    pub burst_duration_s: f64,
    /// Command response time constant (first-order filter, s).
    pub response_tau_s: f64,
    /// Whether regenerative braking is enabled (BLHeli32 default: true).
    pub brake_enabled: bool,
}

impl Default for EscConfig {
    fn default() -> Self {
        // Typical 5" 4-in-1 race ESC with BLHeli32 + DShot600.
        Self {
            continuous_amps: 45.0,
            burst_amps: 55.0,
            burst_duration_s: 5.0,
            response_tau_s: 0.003, // 3 ms decode + smoothing
            brake_enabled: true,
        }
    }
}

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

    // Electrical parameters
    /// Motor KV rating (RPM per volt, unloaded)
    pub motor_kv: f64,
    /// Motor winding resistance in ohms
    pub motor_resistance_ohm: f64,
    /// Battery voltage (nominal)
    pub battery_voltage: f64,
    /// Motor torque constant Kt in N·m/A (derived from KV: Kt = 60 / (2π * KV))
    pub motor_kt_electrical: f64,
    /// No-load (idle) current draw per motor in Amps — covers bearing friction,
    /// iron losses, and ESC quiescent draw. Typical 0.3-1.5 A for mini-quad motors.
    pub motor_no_load_amps: f64,

    // ESC parameters (Phase 2a — data-carrier only, not yet used in loop)
    pub esc: EscConfig,

    // Thermal parameters
    /// Motor thermal mass in J/K (winding + stator heat capacity)
    pub thermal_mass_j_per_k: f64,
    /// Thermal dissipation coefficient in W/K (convective + radiative cooling)
    pub thermal_dissipation_w_per_k: f64,
    /// Ambient temperature in °C
    pub ambient_temp_c: f64,
    /// Maximum safe motor temperature in °C (before derating)
    pub max_motor_temp_c: f64,
    /// Temperature at which motor shuts down in °C
    pub motor_shutdown_temp_c: f64,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        // Typical 5" FPV quad, ~2kg AUW
        // Motors: 2306 size, ~1200 KV, 5" props
        // Max thrust per motor: ~12N at 2500 rad/s
        let motor_kv = 1200.0;
        Self {
            mass_kg: 2.0,
            arm_length_m: 0.11,              // 220mm diagonal frame
            inertia: [0.015, 0.015, 0.025],   // kg·m² (balanced for PX4 default PIDs - higher Izz reduces yaw oscillation)
            kt: 1.9e-6,                       // thrust = kt * ω², gives ~12N at 2500 rad/s
            kq: 5.0e-8,                       // torque = kq * ω², kq/kt ≈ 0.026 (aggressive 5" prop)
            tau_motor: 0.025,                 // 25ms motor time constant
            drag_coeffs: [0.016, 0.016, 0.022], // 5" quad: 0.5*ρ*Cd*A ≈ 0.016 lateral, 0.022 vertical
            gravity: 9.80665,

            // Electrical parameters for 2306 motor
            motor_kv,
            motor_resistance_ohm: 0.065,      // Typical 2306 motor winding resistance
            battery_voltage: 14.8,            // 4S LiPo nominal (4 × 3.7V)
            motor_kt_electrical: 60.0 / (2.0 * std::f64::consts::PI * motor_kv),
            motor_no_load_amps: 0.5,          // Quiescent draw — was hardcoded in motor.rs
            esc: EscConfig::default(),

            // Thermal parameters (estimated for 2306 motor ~33g)
            thermal_mass_j_per_k: 15.0,       // Copper windings + stator mass
            thermal_dissipation_w_per_k: 0.8, // Convective cooling with prop airflow
            ambient_temp_c: 25.0,
            max_motor_temp_c: 100.0,          // Start power derating
            motor_shutdown_temp_c: 150.0,     // Emergency shutdown
        }
    }
}

impl PhysicsConfig {
    /// Create a new physics config with custom parameters (legacy, uses defaults for electrical/thermal).
    #[allow(clippy::too_many_arguments)]
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
        let defaults = Self::default();
        Self {
            mass_kg,
            arm_length_m,
            inertia,
            kt,
            kq,
            tau_motor,
            drag_coeffs,
            gravity,
            // Use defaults for electrical/thermal
            motor_kv: defaults.motor_kv,
            motor_resistance_ohm: defaults.motor_resistance_ohm,
            battery_voltage: defaults.battery_voltage,
            motor_kt_electrical: defaults.motor_kt_electrical,
            motor_no_load_amps: defaults.motor_no_load_amps,
            esc: defaults.esc,
            thermal_mass_j_per_k: defaults.thermal_mass_j_per_k,
            thermal_dissipation_w_per_k: defaults.thermal_dissipation_w_per_k,
            ambient_temp_c: defaults.ambient_temp_c,
            max_motor_temp_c: defaults.max_motor_temp_c,
            motor_shutdown_temp_c: defaults.motor_shutdown_temp_c,
        }
    }

    pub fn from_motor_specs(
        kv: f64,
        prop_diameter_inches: f64,
        frame_weight_g: f64,
        motor_weight_g: f64,
    ) -> Self {
        Self::from_build_specs(kv, prop_diameter_inches, prop_diameter_inches * 0.9, 3, frame_weight_g, motor_weight_g, 14.8)
    }

    /// Create config from build specs with full electrical/thermal modeling.
    #[allow(clippy::too_many_arguments)]
    pub fn from_build_specs(
        kv: f64,
        prop_diameter_inches: f64,
        prop_pitch_inches: f64,
        blade_count: i32,
        frame_weight_g: f64,
        motor_weight_g: f64,
        battery_voltage: f64,
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
        // kq/kt ratio depends on prop efficiency:
        // Low-pitch 2-blade (~0.012) to high-pitch 5-blade (~0.035)
        // Base ratio 0.015 for a 2-blade 3-pitch, scales with pitch and blade count
        let kq_kt_ratio = 0.010 + 0.002 * prop_pitch_inches + 0.002 * (blade_count as f64);
        let kq = kt * kq_kt_ratio;
        let tau_motor = 0.02 + (1500.0 / kv) * 0.01;
        let mass_kg = (frame_weight_g + 4.0 * motor_weight_g) / 1000.0;
        let arm_length_m = prop_diameter_inches * 0.0254 * 1.1;
        // Inertia estimation: motors at arm tips dominate (point-mass model)
        // Ixx = Iyy ≈ 4 × m_motor × L² + frame_contribution
        // Izz ≈ 4 × m_motor × L² × 2 (all 4 arms contribute to yaw)
        //
        // Phase 1+6: legacy `0.012` / `0.020` floors removed. The floors existed
        // so PX4's default rate PIDs (tuned for ~5" inertia) didn't oscillate
        // on light builds. The daemon now pushes per-build PIDs computed by
        // `px4_pids::compute_pids` before flight, so real inertia can be used.
        let motor_mass_kg = motor_weight_g / 1000.0;
        let motor_inertia = 4.0 * motor_mass_kg * arm_length_m * arm_length_m;
        let frame_inertia = (frame_weight_g / 1000.0) * arm_length_m * arm_length_m * 0.17;
        let ixx = motor_inertia + frame_inertia;
        let iyy = ixx;
        let izz = motor_inertia * 2.0 + frame_inertia;

        // Estimate motor resistance from stator size (larger = lower resistance)
        // Typical: 2306 ~0.065Ω, 2207 ~0.080Ω, 1404 ~0.150Ω
        let stator_volume_proxy = kv / 1000.0; // Lower KV typically = larger stator
        let motor_resistance_ohm = 0.05 + 0.02 / stator_volume_proxy.max(0.5);

        // Thermal mass scales with motor weight (copper has ~385 J/kg·K)
        let thermal_mass_j_per_k = motor_weight_g * 0.4; // ~40% of motor is copper windings

        // Thermal dissipation improves with prop airflow (larger prop = better cooling)
        let thermal_dissipation_w_per_k = 0.5 + 0.1 * prop_diameter_inches;

        // Drag: F = c × v × |v|, where c ≈ 0.5 × ρ × Cd × A
        // Frontal area scales with prop diameter squared; Cd ≈ 1.0-1.3 for a quad frame
        let frontal_area_m2 = (prop_diameter_inches * 0.0254).powi(2) * 1.5; // ~1.5× prop disc overlap
        let drag_lateral = 0.5 * 1.225 * 1.1 * frontal_area_m2; // ρ=1.225, Cd≈1.1
        let drag_vertical = 0.5 * 1.225 * 1.5 * frontal_area_m2; // higher Cd for flat bottom

        Self {
            mass_kg,
            arm_length_m,
            inertia: [ixx, iyy, izz],
            kt,
            kq,
            tau_motor,
            drag_coeffs: [drag_lateral, drag_lateral, drag_vertical],
            gravity: 9.80665,

            // Electrical parameters
            motor_kv: kv,
            motor_resistance_ohm,
            battery_voltage,
            motor_kt_electrical: 60.0 / (2.0 * std::f64::consts::PI * kv),
            motor_no_load_amps: 0.5,
            esc: EscConfig::default(),

            // Thermal parameters
            thermal_mass_j_per_k,
            thermal_dissipation_w_per_k,
            ambient_temp_c: 25.0,
            max_motor_temp_c: 100.0,
            motor_shutdown_temp_c: 150.0,
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

    /// Calculate maximum motor speed based on battery voltage (no-load).
    ///
    /// max_rpm = KV × voltage, converted to rad/s
    pub fn max_motor_speed_from_voltage(&self) -> f64 {
        let max_rpm = self.motor_kv * self.battery_voltage;
        max_rpm * std::f64::consts::PI / 30.0 // RPM to rad/s
    }

    /// Hover throttle as a percentage of max motor speed (0.0–1.0).
    ///
    /// This is what a pilot sees on their OSD: e.g., 0.35 means 35% throttle to hover.
    pub fn hover_throttle_percent(&self) -> f64 {
        self.hover_motor_speed() / self.max_motor_speed_from_voltage()
    }

    /// Estimated hover current draw in Amps (all 4 motors combined).
    ///
    /// Uses the torque-based motor current model: I = kq·ω²/Kt + I_no_load
    pub fn estimated_hover_current_a(&self) -> f64 {
        let omega = self.hover_motor_speed();
        let torque_per_motor = self.kq * omega * omega;
        let current_per_motor = torque_per_motor / self.motor_kt_electrical + self.motor_no_load_amps;
        4.0 * current_per_motor
    }

    /// Estimated max climb rate in m/s at full throttle (simplified model).
    ///
    /// Assumes excess thrust accelerates the quad until drag balances it.
    /// F_excess = 4·kt·ω_max² − m·g; terminal velocity where F_drag = F_excess.
    pub fn estimated_max_climb_rate_mps(&self) -> f64 {
        let omega_max = self.max_motor_speed_from_voltage();
        let f_max = 4.0 * self.kt * omega_max * omega_max;
        let f_excess = f_max - self.mass_kg * self.gravity;
        if f_excess <= 0.0 {
            return 0.0; // Underpowered — can't climb
        }
        // F_drag = c_z · v², solve for v
        let c_z = self.drag_coeffs[2];
        (f_excess / c_z).sqrt()
    }

    /// Estimated flight time at hover in seconds.
    ///
    /// This is a rough estimate: `capacity_mah / 1000 × 3600 / hover_current_a × 0.8`
    /// (80% usable capacity before LVC).
    pub fn estimated_hover_time_s(&self, capacity_mah: f64) -> f64 {
        let hover_current = self.estimated_hover_current_a();
        if hover_current < 0.1 {
            return f64::INFINITY; // Unrealistic
        }
        let capacity_ah = capacity_mah / 1000.0;
        let usable_capacity_ah = capacity_ah * 0.8; // 80% before LVC
        usable_capacity_ah * 3600.0 / hover_current
    }

    /// Calculate power derating factor based on motor temperature.
    ///
    /// Returns 1.0 at or below max_motor_temp_c, linearly decreasing to 0.0 at shutdown temp.
    pub fn thermal_derating_factor(&self, motor_temp_c: f64) -> f64 {
        if motor_temp_c <= self.max_motor_temp_c {
            1.0
        } else if motor_temp_c >= self.motor_shutdown_temp_c {
            0.0
        } else {
            let range = self.motor_shutdown_temp_c - self.max_motor_temp_c;
            let excess = motor_temp_c - self.max_motor_temp_c;
            1.0 - (excess / range)
        }
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
        // kq/kt ratio for 3-blade 4.5-pitch (default from_motor_specs): 0.010 + 0.002*4.5 + 0.002*3 = 0.025
        let expected_ratio = 0.010 + 0.002 * (5.0 * 0.9) + 0.002 * 3.0;
        assert!((config.kq - config.kt * expected_ratio).abs() < 1e-12);
        assert!(config.tau_motor > 0.025 && config.tau_motor < 0.035);
        assert!(config.inertia[2] > config.inertia[0]);
        assert!(config.inertia[0] > 0.0);
        // Phase 1+6: floors removed. Real inertia for a 5" 1700KV 350g/33g
        // build sits around Ixx≈0.0037, Izz≈0.0063 — the daemon's Phase 6
        // PARAM_SET push scales PX4's rate PIDs to match. Assert physical
        // bounds only.
        assert!(config.inertia[0] < 0.012,
            "post-Phase-1 Ixx={} should be below the old floor", config.inertia[0]);
        assert!(config.inertia[2] > config.inertia[0],
            "Izz must exceed Ixx for a planar quadrotor");
    }

    #[test]
    fn from_motor_specs_lightweight_build_stable() {
        // 1800KV, 5" 2-blade, 350g frame, 33g motors. Pre-Phase-1 this hit
        // the inertia floor; post-Phase-1+6 the daemon's per-build PIDs
        // handle the real low-inertia airframe directly.
        let config = PhysicsConfig::from_motor_specs(1800.0, 5.0, 350.0, 33.0);
        assert!(config.inertia[0] > 0.0);
        assert!(config.inertia[0] < 0.012,
            "Ixx={} should be below the legacy floor without the override",
            config.inertia[0]);
        // Hover omega should be reasonable
        let hover = config.hover_motor_speed();
        let max = config.max_motor_speed_from_voltage();
        let hover_ratio = hover / max;
        assert!(hover_ratio > 0.15 && hover_ratio < 0.6,
            "Hover/max ratio {} out of range", hover_ratio);
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
        // Same motor/frame, different props (4S battery)
        let low_pitch_2blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 3.0, 2, 350.0, 33.0, 14.8);
        let high_pitch_3blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 3, 350.0, 33.0, 14.8);
        let high_pitch_4blade = PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 4, 350.0, 33.0, 14.8);

        // Higher pitch = higher kt
        assert!(high_pitch_3blade.kt > low_pitch_2blade.kt);
        // More blades = higher kt
        assert!(high_pitch_4blade.kt > high_pitch_3blade.kt);
        // Mass should be the same for all
        assert!((low_pitch_2blade.mass_kg - high_pitch_4blade.mass_kg).abs() < 1e-10);
    }

    #[test]
    fn test_voltage_limited_max_speed() {
        let config = PhysicsConfig::default(); // 1200KV, 14.8V
        let max_speed = config.max_motor_speed_from_voltage();
        // 1200 KV * 14.8V = 17760 RPM = 1860 rad/s
        assert!((max_speed - 1860.0).abs() < 10.0);
    }

    #[test]
    fn test_thermal_derating() {
        let config = PhysicsConfig::default();

        // Below max temp: no derating
        assert!((config.thermal_derating_factor(80.0) - 1.0).abs() < 1e-10);

        // At max temp: no derating yet
        assert!((config.thermal_derating_factor(100.0) - 1.0).abs() < 1e-10);

        // Midway between max and shutdown: 50% derating
        assert!((config.thermal_derating_factor(125.0) - 0.5).abs() < 1e-10);

        // At shutdown: full derating
        assert!((config.thermal_derating_factor(150.0) - 0.0).abs() < 1e-10);
    }
}
