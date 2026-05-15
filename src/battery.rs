//! LiPo battery model with voltage curve and discharge tracking.

use crate::config::PhysicsConfig;
use crate::motor::compute_motor_current;

/// Estimate total internal resistance for a LiPo pack from cell count and C rating.
///
/// Empirical fit: `R_per_cell ≈ 200 / C` (milliohms). Real-world data:
/// - 75C 4S race LiPo: ~2.7 mΩ/cell → ~11 mΩ total
/// - 25C 4S standard: ~8 mΩ/cell → ~32 mΩ total
///
/// Cells in series sum. Returned value is in ohms.
pub fn estimate_r_internal_ohm(cell_count: u8, c_rating: f64) -> f64 {
    let r_per_cell_mohm = 200.0 / c_rating.max(1.0);
    cell_count as f64 * r_per_cell_mohm / 1000.0
}

/// Battery configuration derived from cell count and capacity.
#[derive(Debug, Clone)]
pub struct BatteryConfig {
    /// Number of cells in series
    pub cell_count: u8,
    /// Capacity in milliamp-hours
    pub capacity_mah: f64,
    /// Maximum continuous discharge rate (C rating)
    pub discharge_rate_c: f64,
    /// Total pack internal resistance in ohms (Phase 2b — used by `BatteryState::v_terminal`).
    pub internal_resistance_ohm: f64,
}

impl Default for BatteryConfig {
    fn default() -> Self {
        let cell_count = 4u8;
        let c_rating = 75.0;
        Self {
            cell_count,
            capacity_mah: 1500.0,
            discharge_rate_c: c_rating,
            internal_resistance_ohm: estimate_r_internal_ohm(cell_count, c_rating),
        }
    }
}

impl BatteryConfig {
    pub fn new(cell_count: u8, capacity_mah: f64, discharge_rate_c: f64) -> Self {
        Self {
            cell_count,
            capacity_mah,
            discharge_rate_c,
            internal_resistance_ohm: estimate_r_internal_ohm(cell_count, discharge_rate_c),
        }
    }

    /// Full charge voltage (4.2V per cell)
    pub fn full_voltage(&self) -> f64 {
        self.cell_count as f64 * 4.2
    }

    /// Nominal voltage (3.7V per cell)
    pub fn nominal_voltage(&self) -> f64 {
        self.cell_count as f64 * 3.7
    }

    /// Empty voltage (3.3V per cell — safe minimum for LiPo)
    pub fn empty_voltage(&self) -> f64 {
        self.cell_count as f64 * 3.3
    }

    /// Total energy capacity in amp-seconds (coulombs)
    pub fn capacity_as(&self) -> f64 {
        self.capacity_mah / 1000.0 * 3600.0
    }

    /// Maximum continuous current draw (amps)
    pub fn max_continuous_amps(&self) -> f64 {
        self.capacity_mah / 1000.0 * self.discharge_rate_c
    }
}

/// Runtime battery state tracking charge level.
#[derive(Debug, Clone)]
pub struct BatteryState {
    /// Remaining charge in amp-seconds (coulombs)
    pub charge_remaining_as: f64,
    /// Total capacity in amp-seconds
    pub capacity_as: f64,
    /// Cell count for voltage computation
    pub cell_count: u8,
}

impl BatteryState {
    /// Create a fully charged battery from config.
    pub fn fully_charged(config: &BatteryConfig) -> Self {
        let capacity_as = config.capacity_as();
        Self {
            charge_remaining_as: capacity_as,
            capacity_as,
            cell_count: config.cell_count,
        }
    }

    /// State of charge as fraction [0.0, 1.0]
    pub fn soc(&self) -> f64 {
        (self.charge_remaining_as / self.capacity_as).clamp(0.0, 1.0)
    }

    /// State of charge as percent [0, 100]
    pub fn percent(&self) -> u8 {
        (self.soc() * 100.0).round() as u8
    }

    /// Current voltage based on discharge curve (open-circuit / no-load).
    /// Uses a simplified LiPo curve: mostly flat at nominal, drops at extremes.
    pub fn voltage(&self) -> f64 {
        let soc = self.soc();
        let per_cell = lipo_voltage_curve(soc);
        per_cell * self.cell_count as f64
    }

    /// Terminal voltage under load (Phase 2b).
    ///
    /// `V_terminal = V_OCV(soc) - I · R_internal`, clamped to 0.
    /// Use this whenever you care about the voltage *the motors actually see*.
    /// The OCV `voltage()` method is appropriate for "battery percent" displays
    /// because it ignores transient sag.
    pub fn v_terminal(&self, load_amps: f64, r_internal_ohm: f64) -> f64 {
        let sag = load_amps * r_internal_ohm;
        (self.voltage() - sag).max(0.0)
    }

    /// Consume charge based on total current draw over dt seconds.
    /// Returns the actual current drawn (may be limited by remaining charge).
    pub fn discharge(&mut self, total_current_amps: f64, dt: f64) -> f64 {
        let charge_used = total_current_amps * dt;
        let actual = charge_used.min(self.charge_remaining_as);
        self.charge_remaining_as -= actual;
        if dt > 0.0 { actual / dt } else { 0.0 }
    }

    /// Reset to full charge.
    pub fn recharge(&mut self) {
        self.charge_remaining_as = self.capacity_as;
    }

    /// Check if battery is depleted (below safe threshold ~5%)
    pub fn is_depleted(&self) -> bool {
        self.soc() < 0.05
    }
}

/// Simplified LiPo per-cell voltage curve.
/// Input: state of charge [0.0, 1.0]
/// Output: per-cell voltage [3.3, 4.2]
fn lipo_voltage_curve(soc: f64) -> f64 {
    let s = soc.clamp(0.0, 1.0);
    // Piecewise linear approximation of LiPo discharge curve:
    // 100% → 4.20V, 90% → 4.05V, 20% → 3.70V, 10% → 3.50V, 0% → 3.30V
    if s >= 0.9 {
        4.05 + (s - 0.9) * (4.20 - 4.05) / 0.1
    } else if s >= 0.2 {
        3.70 + (s - 0.2) * (4.05 - 3.70) / 0.7
    } else if s >= 0.1 {
        3.50 + (s - 0.1) * (3.70 - 3.50) / 0.1
    } else {
        3.30 + s * (3.50 - 3.30) / 0.1
    }
}

/// Compute total current draw from all 4 motors.
pub fn total_motor_current(motor_speeds: &[f64; 4], config: &PhysicsConfig) -> f64 {
    motor_speeds
        .iter()
        .map(|&omega| compute_motor_current(omega, config))
        .sum()
}

/// Estimate hover flight time in minutes given battery config and physics config.
pub fn estimate_flight_time_min(battery: &BatteryConfig, physics: &PhysicsConfig) -> f64 {
    let hover_omega = physics.hover_motor_speed();
    let hover_current_per_motor = compute_motor_current(hover_omega, physics);
    let total_hover_current = 4.0 * hover_current_per_motor;

    if total_hover_current <= 0.0 {
        return 0.0;
    }

    let capacity_ah = battery.capacity_mah / 1000.0;
    // Use 80% of capacity (don't drain below 20% SoC for battery health)
    let usable_ah = capacity_ah * 0.80;
    (usable_ah / total_hover_current) * 60.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_config_defaults() {
        let config = BatteryConfig::default();
        assert_eq!(config.cell_count, 4);
        assert!((config.full_voltage() - 16.8).abs() < 1e-10);
        assert!((config.nominal_voltage() - 14.8).abs() < 1e-10);
        assert!((config.empty_voltage() - 13.2).abs() < 1e-10);
    }

    #[test]
    fn test_fully_charged_state() {
        let config = BatteryConfig::default();
        let state = BatteryState::fully_charged(&config);
        assert_eq!(state.percent(), 100);
        assert!((state.voltage() - 16.8).abs() < 0.01);
    }

    #[test]
    fn test_discharge() {
        let config = BatteryConfig::default(); // 1500mAh = 5400 As
        let mut state = BatteryState::fully_charged(&config);

        // Draw 30A for 60 seconds = 1800 As consumed (~33% of capacity)
        state.discharge(30.0, 60.0);
        assert!(state.soc() < 0.7);
        assert!(state.percent() < 70);
    }

    #[test]
    fn test_recharge() {
        let config = BatteryConfig::default();
        let mut state = BatteryState::fully_charged(&config);
        state.discharge(50.0, 60.0); // heavy draw
        assert!(state.percent() < 50);

        state.recharge();
        assert_eq!(state.percent(), 100);
    }

    #[test]
    fn test_voltage_curve_monotonic() {
        let mut prev = 0.0;
        for i in 0..=100 {
            let soc = i as f64 / 100.0;
            let v = lipo_voltage_curve(soc);
            assert!(v >= prev, "Voltage curve not monotonic at soc={soc}: {v} < {prev}");
            prev = v;
        }
    }

    #[test]
    fn test_voltage_curve_bounds() {
        assert!((lipo_voltage_curve(0.0) - 3.30).abs() < 0.01);
        assert!((lipo_voltage_curve(1.0) - 4.20).abs() < 0.01);
    }

    #[test]
    fn test_estimate_flight_time() {
        let battery = BatteryConfig::new(4, 1500.0, 75.0);
        let physics = PhysicsConfig::default();
        let time = estimate_flight_time_min(&battery, &physics);
        // Default config has low resistance → high hover current → short flight time
        // With 1500mAh this is a typical racing quad (short flight time expected)
        assert!(time > 0.3 && time < 15.0, "Unexpected flight time: {time} min");
    }

    #[test]
    fn test_depleted_stops_discharge() {
        let config = BatteryConfig::new(4, 100.0, 75.0); // tiny battery
        let mut state = BatteryState::fully_charged(&config);

        // Massive draw should deplete
        state.discharge(100.0, 100.0);
        assert!(state.is_depleted());
        assert!((state.charge_remaining_as).abs() < 1e-10);
    }
}
