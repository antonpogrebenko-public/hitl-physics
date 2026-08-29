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
    /// Propeller diameter in metres. Needed for the actuator-disc area, which
    /// sets the momentum-theory ceiling on how much thrust a given electrical
    /// power can possibly produce.
    pub prop_diameter_m: f64,
    /// Motor KV rating (RPM per volt, unloaded)
    pub motor_kv: f64,
    /// Motor winding resistance in ohms
    pub motor_resistance_ohm: f64,
    /// Battery voltage (nominal)
    pub battery_voltage: f64,
    /// Motor torque constant Kt in N·m/A (derived from KV: Kt = 60 / (2π * KV))
    pub motor_kt_electrical: f64,
    /// Hard ceiling on the current one motor may draw, in Amps.
    ///
    /// The torque balance alone has no upper bound: it lets a motor pull
    /// whatever current the winding resistance permits at the speed where prop
    /// drag matches motor torque. Real airframes hit a limit long before that —
    /// the ESC's rating, or the pack's C-rating shared across four motors.
    /// Without this a 2306 on a 6" tri-blade settled at 49.7 A each, 199 A for
    /// the airframe, and the model was content.
    pub max_current_per_motor_a: f64,
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

/// Sea-level air density in kg/m³, the basis for every aerodynamic coefficient
/// in this crate and in `prop_coefficients`.
pub const AIR_DENSITY: f64 = 1.225;

impl Default for PhysicsConfig {
    fn default() -> Self {
        // Typical 5" FPV quad, ~2kg AUW
        // Motors: 2306 size, ~1200 KV, 5" props
        // Max thrust per motor: ~12N at 2500 rad/s
        let motor_kv = 1200.0;
        Self {
            mass_kg: 2.0,
            arm_length_m: 0.11,                 // 220mm diagonal frame
            prop_diameter_m: 5.0 * 0.0254,      // 5" props
            inertia: [0.015, 0.015, 0.025], // kg·m² (balanced for PX4 default PIDs - higher Izz reduces yaw oscillation)
            kt: 1.9e-6,                     // thrust = kt * ω², gives ~12N at 2500 rad/s
            kq: 5.0e-8,                     // torque = kq * ω², kq/kt ≈ 0.026 (aggressive 5" prop)
            tau_motor: 0.025,               // 25ms motor time constant
            drag_coeffs: [0.016, 0.016, 0.022], // 5" quad: 0.5*ρ*Cd*A ≈ 0.016 lateral, 0.022 vertical
            gravity: 9.80665,

            // Electrical parameters for 2306 motor
            motor_kv,
            motor_resistance_ohm: 0.065, // Typical 2306 motor winding resistance
            battery_voltage: 14.8,       // 4S LiPo nominal (4 × 3.7V)
            motor_kt_electrical: 60.0 / (2.0 * std::f64::consts::PI * motor_kv),
            motor_no_load_amps: 0.5, // Quiescent draw — was hardcoded in motor.rs
            max_current_per_motor_a: EscConfig::default().burst_amps,
            esc: EscConfig::default(),

            // Thermal parameters (estimated for 2306 motor ~33g)
            thermal_mass_j_per_k: 15.0, // Copper windings + stator mass
            thermal_dissipation_w_per_k: 0.8, // Convective cooling with prop airflow
            ambient_temp_c: 25.0,
            max_motor_temp_c: 100.0,      // Start power derating
            motor_shutdown_temp_c: 150.0, // Emergency shutdown
        }
    }
}

impl PhysicsConfig {
    pub fn from_motor_specs(
        kv: f64,
        prop_diameter_inches: f64,
        frame_weight_g: f64,
        motor_weight_g: f64,
    ) -> Self {
        Self::from_build_specs(
            kv,
            prop_diameter_inches,
            prop_diameter_inches * 0.9,
            3,
            frame_weight_g,
            motor_weight_g,
            14.8,
        )
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

        // kt is a prop coefficient (thrust = kt·ω²); the (2300/kv)² term is an
        // empirical stand-in fit for ~1500-2600 KV race motors. Clamp KV to the
        // calibrated floor so large low-KV builds (8"+) don't inflate kt
        // quadratically. Mirrors KT_KV_FLOOR in the TS physics-model.
        let effective_kv = kv.max(1500.0);
        let kt = prop_factor * (2300.0 / effective_kv).powi(2);
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
            prop_diameter_m: prop_diameter_inches * 0.0254,
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
            max_current_per_motor_a: EscConfig::default().burst_amps,
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

    /// Calculate maximum (full-throttle, loaded) motor speed in rad/s.
    ///
    /// Solves the steady-state torque balance between the motor and prop:
    ///   motor torque:  Kt·(V − Kt·ω) / R
    ///   prop torque:   kq·ω²
    ///
    /// Setting equal gives quadratic `kq·ω² + (Kt²/R)·ω − Kt·V/R = 0`.
    /// The positive root is the loaded full-throttle RPM — no constant fraction
    /// needed. This correctly predicts ~0.88× no-load for mid-KV 5" builds AND
    /// the heavier ~0.5× bog on tiny high-KV motors (e.g. 0802 12000KV).
    pub fn max_motor_speed_from_voltage(&self) -> f64 {
        let kt_e = self.motor_kt_electrical;
        let r = self.motor_resistance_ohm;
        let v = self.battery_voltage;

        if self.kq < 1e-12 {
            return self.motor_kv * v * std::f64::consts::PI / 30.0;
        }

        // Voltage-limited: kq·ω² + (Kt²/R)·ω − Kt·V/R = 0. The motor accelerates
        // until back-EMF leaves only enough headroom to drive the current that
        // balances prop drag.
        let a = self.kq;
        let b = kt_e * kt_e / r;
        let c = kt_e * v / r;
        let disc = b * b + 4.0 * a * c;
        let omega_voltage = (-b + disc.sqrt()) / (2.0 * a);

        // Current-limited: once the ESC or the pack caps current, motor torque
        // is pinned at Kt·(I_max − I₀) and the propeller settles where its drag
        // matches that. Current RISES as speed falls, so the binding constraint
        // is whichever speed is lower — a motor cannot sit below the
        // current-limited speed without demanding more current than exists.
        let usable = (self.max_current_per_motor_a - self.motor_no_load_amps).max(0.0);
        if usable <= 0.0 {
            return 0.0;
        }
        let omega_current = (kt_e * usable / self.kq).sqrt();

        omega_voltage.min(omega_current)
    }

    /// Whether the full-throttle operating point is capped by available current
    /// rather than by battery voltage. True means the build is ESC- or
    /// pack-limited: more KV or more pitch would not buy more thrust.
    pub fn is_current_limited(&self) -> bool {
        let usable = (self.max_current_per_motor_a - self.motor_no_load_amps).max(0.0);
        if usable <= 0.0 || self.kq < 1e-12 {
            return true;
        }
        (self.motor_kt_electrical * usable / self.kq).sqrt() < self.voltage_limited_speed()
    }

    /// Full-throttle speed ignoring any current ceiling, in rad/s.
    fn voltage_limited_speed(&self) -> f64 {
        if self.kq < 1e-12 {
            return self.motor_kv * self.battery_voltage * std::f64::consts::PI / 30.0;
        }
        let b = self.motor_kt_electrical.powi(2) / self.motor_resistance_ohm;
        let c = self.motor_kt_electrical * self.battery_voltage / self.motor_resistance_ohm;
        (-b + (b * b + 4.0 * self.kq * c).sqrt()) / (2.0 * self.kq)
    }

    /// Actuator-disc area of one propeller in m².
    pub fn disc_area_m2(&self) -> f64 {
        std::f64::consts::PI * (self.prop_diameter_m / 2.0).powi(2)
    }

    /// Electrical current one motor draws at full throttle, in Amps.
    ///
    /// At the loaded speed the winding sees `V − Kt·ω`; that residual is what
    /// drives current through `R`. A motor spinning at nearly its no-load
    /// speed has almost no residual, and therefore almost no power — which is
    /// the cross-check `momentum_theory_thrust_ceiling_n` turns into a bound.
    pub fn full_throttle_current_a(&self) -> f64 {
        let omega = self.max_motor_speed_from_voltage();
        let back_emf = self.motor_kt_electrical * omega;
        let demanded = ((self.battery_voltage - back_emf) / self.motor_resistance_ohm).max(0.0);
        // When the ESC or the pack is the binding constraint the motor sits
        // below its voltage-limited speed, so the back-EMF term would imply a
        // current nobody is supplying. The cap is the real draw.
        demanded.min(self.max_current_per_motor_a)
    }

    /// Electrical power into one motor at full throttle, in Watts.
    pub fn full_throttle_power_w(&self) -> f64 {
        self.battery_voltage * self.full_throttle_current_a()
    }

    /// The most thrust one propeller could produce from its own full-throttle
    /// electrical power, in Newtons — a hard physical ceiling.
    ///
    /// Momentum theory gives the induced power to produce thrust `T` through a
    /// disc of area `A` as `T^(3/2) / sqrt(2·ρ·A)`. Inverting it bounds thrust
    /// by available power. This assumes a lossless motor, a lossless ESC and a
    /// figure of merit of 1.0, so a real propeller reaches perhaps 55–70% of
    /// it. Exceeding it is not "optimistic" — it is thrust from no energy, and
    /// it means the model's operating point is wrong however plausible the
    /// thrust figure looks on its own.
    pub fn momentum_theory_thrust_ceiling_n(&self) -> f64 {
        let p = self.full_throttle_power_w();
        if p <= 0.0 {
            return 0.0;
        }
        (p * (2.0 * AIR_DENSITY * self.disc_area_m2()).sqrt()).powf(2.0 / 3.0)
    }

    /// Static thrust one motor produces at full throttle, in Newtons.
    pub fn max_thrust_per_motor_n(&self) -> f64 {
        let omega = self.max_motor_speed_from_voltage();
        self.kt * omega * omega
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
        let current_per_motor =
            torque_per_motor / self.motor_kt_electrical + self.motor_no_load_amps;
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
        assert!((config.mass_kg - 2.0).abs() < 1e-10); // 2kg quad
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
        assert!(
            config.inertia[0] < 0.012,
            "post-Phase-1 Ixx={} should be below the old floor",
            config.inertia[0]
        );
        assert!(
            config.inertia[2] > config.inertia[0],
            "Izz must exceed Ixx for a planar quadrotor"
        );
    }

    #[test]
    fn from_motor_specs_lightweight_build_stable() {
        // 1800KV, 5" 2-blade, 350g frame, 33g motors. Pre-Phase-1 this hit
        // the inertia floor; post-Phase-1+6 the daemon's per-build PIDs
        // handle the real low-inertia airframe directly.
        let config = PhysicsConfig::from_motor_specs(1800.0, 5.0, 350.0, 33.0);
        assert!(config.inertia[0] > 0.0);
        assert!(
            config.inertia[0] < 0.012,
            "Ixx={} should be below the legacy floor without the override",
            config.inertia[0]
        );
        // Hover omega should be reasonable
        let hover = config.hover_motor_speed();
        let max = config.max_motor_speed_from_voltage();
        let hover_ratio = hover / max;
        assert!(
            hover_ratio > 0.15 && hover_ratio < 0.6,
            "Hover/max ratio {} out of range",
            hover_ratio
        );
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
        let low_pitch_2blade =
            PhysicsConfig::from_build_specs(1700.0, 5.0, 3.0, 2, 350.0, 33.0, 14.8);
        let high_pitch_3blade =
            PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 3, 350.0, 33.0, 14.8);
        let high_pitch_4blade =
            PhysicsConfig::from_build_specs(1700.0, 5.0, 5.0, 4, 350.0, 33.0, 14.8);

        // Higher pitch = higher kt
        assert!(high_pitch_3blade.kt > low_pitch_2blade.kt);
        // More blades = higher kt
        assert!(high_pitch_4blade.kt > high_pitch_3blade.kt);
        // Mass should be the same for all
        assert!((low_pitch_2blade.mass_kg - high_pitch_4blade.mass_kg).abs() < 1e-10);
    }

    #[test]
    fn low_kv_kt_clamped_to_floor() {
        // A 500 KV 8" build: without the clamp kt = prop_factor·(2300/500)² ≈ 21×.
        // Clamped to KV 1500, the big low-KV build's kt must match the floor build
        // and stay far below the unclamped value.
        let low_kv = PhysicsConfig::from_build_specs(500.0, 8.0, 4.5, 2, 600.0, 60.0, 22.2);
        let at_floor = PhysicsConfig::from_build_specs(1500.0, 8.0, 4.5, 2, 600.0, 60.0, 22.2);
        assert!(
            (low_kv.kt - at_floor.kt).abs() < 1e-12,
            "low-KV kt must clamp to the floor"
        );

        // Loaded full-throttle thrust should be physical (not the ~20:1 blowup):
        // total thrust < 10× weight.
        let omega = low_kv.max_motor_speed_from_voltage();
        let total_thrust_n = 4.0 * low_kv.kt * omega * omega;
        let weight_n = low_kv.mass_kg * low_kv.gravity;
        assert!(
            total_thrust_n / weight_n < 10.0,
            "TWR {} unphysical",
            total_thrust_n / weight_n
        );
    }

    #[test]
    fn high_kv_kt_unaffected_by_clamp() {
        // 1700 KV is above the floor — kt keeps the full (2300/1700)² term.
        let c = PhysicsConfig::from_build_specs(1700.0, 5.0, 4.5, 3, 350.0, 33.0, 14.8);
        let base = 1.9e-6 * (0.7 + 0.06 * 4.5) * (0.85 + 0.05 * 3.0);
        let expected = base * (2300.0_f64 / 1700.0).powi(2);
        assert!((c.kt - expected).abs() < 1e-12);
    }

    #[test]
    fn test_voltage_limited_max_speed() {
        let config = PhysicsConfig::default(); // 1200KV, 14.8V
        let max_speed = config.max_motor_speed_from_voltage();
        let no_load_rads = 1200.0 * 14.8 * std::f64::consts::PI / 30.0; // ~1860 rad/s
                                                                        // Torque-balance model: loaded RPM should be 80-95% of no-load for a
                                                                        // mid-KV motor with a reasonable prop load.
        let ratio = max_speed / no_load_rads;
        assert!(
            ratio > 0.80 && ratio < 0.95,
            "loaded/no-load ratio {:.3} outside expected [0.80, 0.95]",
            ratio
        );
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
