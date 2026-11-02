//! LiPo battery model with voltage curve and discharge tracking.

use crate::config::PhysicsConfig;
use crate::motor::compute_motor_current;

// ---------------------------------------------------------------------------
// Fitted constants, measured from public PX4 flight logs.
//
// Source: the th3seus-tools calibration run 2026-11-02
// (log_validation/calibration/2026-11-02/ in that repository: battery-fit.json
// and report.md). Regenerate with the th3seus-tools tool
// `log_validation.battery_fit`; the values below are pasted from its generated
// output and are not edited by hand.
//
// Data: 35919 real logs, 49 simulation logs excluded; split seed 20260623.
// A log whose median voltage per cell is outside 3.0-4.4 V for its logged cell
// count is left out of the fit (2035 logs, counted in the run as
// cell_count_mismatch_count).
//
// R fit: rapid current-step regression (dV = -R * dI) over 842356 step events,
// per cell count. The step events come from the 11126 logs on the fit side of
// the vehicle-wise split that had at least one step event (per_log_r in
// battery-fit.json); the held-out logs and the logs without step events give
// none.
// OCV fit: samples taken below 3 A and inside 3.0-4.25 V per cell, binned by
// state of charge into 20 bins of width 0.05, the median per bin (a bin with
// fewer than 5 samples is interpolated from its neighbours), then made monotone
// by isotonic regression. State of charge is the logged `remaining` when it
// moves by more than 0.02 over the log. Otherwise (absent, or moving by 0.02 or
// less) it is the logged discharged mAh against a capacity estimate of the
// maximum discharged mAh divided by 0.8, when the discharged mAh is positive
// and rises; a log with neither gives no OCV samples.
//
// Held-out validation (the deployed tables, on held-out logs split by vehicle):
// median voltage error 0.0822 V/cell over 3276 scored logs, under the 0.10
// V/cell target.
// ---------------------------------------------------------------------------

// FITTED_R_TABLE: deployed per-cell R (mohm/cell), keyed by cell count.
// Canonical run, split seed=20260623, 35919 real logs, 49 SIM excluded.
/// Fitted per-cell internal resistance (mohm/cell) keyed by cell count.
///
/// Entry format: (cell_count, deployed_mohm_per_cell).
/// A cell count with at least 200 step events deploys its own median. A cell
/// count with fewer (here 1S and 9S) deploys the median pooled over vehicles
/// (each vehicle's median over all its step events, then the median over
/// vehicles), which is FITTED_R_DEFAULT_MOHM_PER_CELL. There is no plausibility
/// floor. A cell count not in this table also falls back to that default.
const FITTED_R_TABLE: &[(u8, f64)] = &[
    (1, 5.5699),  // n=169, fewer than 200 step events -> pooled over vehicles
    (2, 13.4364), // n=4732
    (3, 8.7361),  // n=13839
    (4, 6.0839),  // n=130805
    (5, 5.6962),  // n=1046
    (6, 3.2146),  // n=441198
    (7, 3.3744),  // n=1355
    (8, 3.2586),  // n=21839
    (9, 5.5699),  // n=7, fewer than 200 step events -> pooled over vehicles
    (10, 3.8934), // n=487
    (12, 2.1526), // n=195505
    (13, 3.7978), // n=17009
    (14, 2.3914), // n=14365
];

/// Median pooled over vehicles (mohm/cell): each vehicle's median over all its
/// step events, then the median over the 6380 vehicles. Used for cell counts
/// with fewer than 200 step events and for cell counts absent from
/// FITTED_R_TABLE.
const FITTED_R_DEFAULT_MOHM_PER_CELL: f64 = 5.5699; // pooled over 6380 vehicles

/// OCV curve: open-circuit voltage per cell vs state of charge.
/// 20-knot piecewise-linear table, monotone-enforced via isotonic regression.
///
/// Column 0: SoC (centre of 0.05-wide bin, range [0.025, 0.975]).
/// Column 1: V/cell (median of the low-current samples in the bin, isotonic).
const OCV_TABLE: &[(f64, f64)] = &[
    (0.0250, 3.5657),
    (0.0750, 3.6235),
    (0.1250, 3.6420),
    (0.1750, 3.6789),
    (0.2250, 3.7649),
    (0.2750, 3.7649),
    (0.3250, 3.7649),
    (0.3750, 3.7953),
    (0.4250, 3.7953),
    (0.4750, 3.7953),
    (0.5250, 3.8045),
    (0.5750, 3.8045),
    (0.6250, 3.8495),
    (0.6750, 3.8990),
    (0.7250, 3.9765),
    (0.7750, 3.9765),
    (0.8250, 3.9990),
    (0.8750, 4.0760),
    (0.9250, 4.0992),
    (0.9750, 4.1545),
];

/// Look up fitted per-cell internal resistance (mohm/cell) by cell count.
///
/// Returns the value deployed for the given cell count by the calibration run
/// 2026-11-02, or the pooled default if the cell count is not in the table.
fn fitted_r_mohm_per_cell(cell_count: u8) -> f64 {
    FITTED_R_TABLE
        .iter()
        .find(|(cc, _)| *cc == cell_count)
        .map(|(_, r)| *r)
        .unwrap_or(FITTED_R_DEFAULT_MOHM_PER_CELL)
}

/// Estimate total internal resistance for a LiPo pack from cell count and C rating.
///
/// Uses per-cell resistance fitted from public PX4 flight logs (the calibration
/// run 2026-11-02: 842356 step events, taken from the 11126 fit-side logs that
/// had step events, out of a corpus of 35919 real logs).  The values are keyed
/// by cell count; C-rating is not reliably present in PX4 logs, so the
/// `_c_rating` argument is ignored and is kept only for API compatibility.
/// There is no clamping: a cell count outside the table gets the pooled default.
///
/// Cells in series sum.  Returned value is in ohms.
pub fn estimate_r_internal_ohm(cell_count: u8, _c_rating: f64) -> f64 {
    // C-rating is not used: fitted median per cell is more accurate than 200/C.
    // The parameter is kept to avoid breaking callers (BatteryConfig::new, default()).
    let r_per_cell_mohm = fitted_r_mohm_per_cell(cell_count);
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
        if dt > 0.0 {
            actual / dt
        } else {
            0.0
        }
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

/// LiPo per-cell open-circuit voltage curve fitted from real PX4 flight logs.
///
/// Input:  state of charge [0.0, 1.0]
/// Output: per-cell voltage (V/cell)
///
/// Implements piecewise-linear interpolation over OCV_TABLE (20 knots).
/// Extrapolation clamps to the boundary knot values (soc < 0.025 -> 3.5657 V/cell,
/// soc > 0.975 -> 4.1545 V/cell).
///
/// Fitted from public PX4 flight logs (the calibration run 2026-11-02):
/// low-current observations, isotonic monotone enforcement.
fn lipo_voltage_curve(soc: f64) -> f64 {
    let s = soc.clamp(0.0, 1.0);

    // Clamp to table boundaries before searching
    if s <= OCV_TABLE[0].0 {
        return OCV_TABLE[0].1;
    }
    let last = OCV_TABLE[OCV_TABLE.len() - 1];
    if s >= last.0 {
        return last.1;
    }

    // Binary search for the enclosing knot interval
    let idx = OCV_TABLE
        .partition_point(|(knot_soc, _)| *knot_soc <= s)
        .saturating_sub(1);

    let (s0, v0) = OCV_TABLE[idx];
    let (s1, v1) = OCV_TABLE[idx + 1];

    // Linear interpolation within the interval
    let t = (s - s0) / (s1 - s0);
    v0 + t * (v1 - v0)
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
    fn fitted_resistance_comes_from_the_run() {
        // (cell count, deployed mohm per cell) from the calibration run 2026-11-02.
        let rows: [(u8, f64); 13] = [
            (1, 5.5699),
            (2, 13.4364),
            (3, 8.7361),
            (4, 6.0839),
            (5, 5.6962),
            (6, 3.2146),
            (7, 3.3744),
            (8, 3.2586),
            (9, 5.5699),
            (10, 3.8934),
            (12, 2.1526),
            (13, 3.7978),
            (14, 2.3914),
        ];
        for (cells, mohm) in rows {
            let expected = mohm * cells as f64 / 1000.0;
            let got = estimate_r_internal_ohm(cells, 75.0);
            assert!(
                (got - expected).abs() < 1e-12,
                "{cells}S: got {got}, expected {expected}"
            );
        }
        // A cell count absent from the table uses the pooled default.
        let got = estimate_r_internal_ohm(11, 75.0);
        assert!((got - 5.5699 * 11.0 / 1000.0).abs() < 1e-12);
    }

    #[test]
    fn ocv_table_comes_from_the_run() {
        // (state of charge, V/cell) knots from the calibration run 2026-11-02.
        let expected: [(f64, f64); 20] = [
            (0.0250, 3.5657),
            (0.0750, 3.6235),
            (0.1250, 3.6420),
            (0.1750, 3.6789),
            (0.2250, 3.7649),
            (0.2750, 3.7649),
            (0.3250, 3.7649),
            (0.3750, 3.7953),
            (0.4250, 3.7953),
            (0.4750, 3.7953),
            (0.5250, 3.8045),
            (0.5750, 3.8045),
            (0.6250, 3.8495),
            (0.6750, 3.8990),
            (0.7250, 3.9765),
            (0.7750, 3.9765),
            (0.8250, 3.9990),
            (0.8750, 4.0760),
            (0.9250, 4.0992),
            (0.9750, 4.1545),
        ];
        assert_eq!(OCV_TABLE.len(), expected.len());
        for (i, (got, want)) in OCV_TABLE.iter().zip(expected.iter()).enumerate() {
            assert_eq!(got, want, "OCV_TABLE knot {i}");
        }
    }

    #[test]
    fn four_cell_config_without_explicit_resistance_uses_the_run_value() {
        let config = BatteryConfig::new(4, 1500.0, 75.0);
        assert!((config.internal_resistance_ohm - 6.0839 * 4.0 / 1000.0).abs() < 1e-12);
        assert!(
            (BatteryConfig::default().internal_resistance_ohm - 6.0839 * 4.0 / 1000.0).abs()
                < 1e-12
        );
    }

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
        // Fitted OCV at soc=1.0 is 4.1545 V/cell -> 4S = 16.618 V
        // (previously hand-tuned 4.2 V/cell = 16.8 V)
        let v = state.voltage();
        assert!(
            v >= 16.0 && v <= 17.0,
            "Fully charged 4S voltage {v:.4} outside [16.0, 17.0]"
        );
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
            assert!(
                v >= prev,
                "Voltage curve not monotonic at soc={soc}: {v} < {prev}"
            );
            prev = v;
        }
    }

    #[test]
    fn test_voltage_curve_bounds() {
        // Fitted OCV endpoints from the calibration run 2026-11-02:
        //   soc=0.0  clamps to first knot  3.5657 V/cell
        //   soc=1.0  clamps to last knot   4.1545 V/cell (slightly below 4.20)
        // Physical LiPo range is 3.0–4.2 V/cell; the fitted range reflects
        // the actual discharge depth observed in the real-log corpus.
        let v_empty = lipo_voltage_curve(0.0);
        let v_full = lipo_voltage_curve(1.0);
        assert!((v_empty - 3.5657).abs() < 1e-12);
        assert!((v_full - 4.1545).abs() < 1e-12);
        assert!(
            v_empty >= 3.0 && v_empty <= 3.7,
            "lipo_voltage_curve(0.0) = {v_empty:.4} outside [3.0, 3.7]"
        );
        assert!(
            v_full >= 4.0 && v_full <= 4.25,
            "lipo_voltage_curve(1.0) = {v_full:.4} outside [4.0, 4.25]"
        );
    }

    #[test]
    fn test_estimate_flight_time() {
        let battery = BatteryConfig::new(4, 1500.0, 75.0);
        let physics = PhysicsConfig::default();
        let time = estimate_flight_time_min(&battery, &physics);
        // Default config has low resistance → high hover current → short flight time
        // With 1500mAh this is a typical racing quad (short flight time expected)
        assert!(
            time > 0.3 && time < 15.0,
            "Unexpected flight time: {time} min"
        );
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
