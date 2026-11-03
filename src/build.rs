//! `BuildSpec` — typed component aggregator that produces a [`PhysicsConfig`].
//!
//! This is the entry point for component-driven physics. Callers (typically the
//! daemon's WebSocket handler) parse API JSONB responses into the `*Spec` structs
//! below, then call [`BuildSpec::to_physics_config`] to derive the simulator
//! configuration.
//!
//! Phase 0 keeps parity with [`PhysicsConfig::from_build_specs`]; later phases
//! will replace the internals with richer per-component physics (frame-driven
//! inertia/drag, ESC dynamics, FC-selected sensor profiles, etc).

use crate::battery::BatteryConfig;
use crate::config::{EscConfig, PhysicsConfig};
use crate::prop_coefficients;

// =============================================================================
// Frame
// =============================================================================

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameMaterial {
    Carbon,
    Polymer,
    Aluminum,
    Other,
}

#[derive(Debug, Clone)]
pub struct FrameSpec {
    pub weight_g: f64,
    /// Motor-to-motor diagonal distance.
    pub wheelbase_mm: f64,
    pub width_mm: Option<f64>,
    pub length_mm: Option<f64>,
    pub height_mm: Option<f64>,
    pub material: Option<FrameMaterial>,
}

// =============================================================================
// Motor
// =============================================================================

#[derive(Debug, Clone, Copy)]
pub struct StatorSize {
    pub diameter_mm: f64,
    pub height_mm: f64,
}

#[derive(Debug, Clone)]
pub struct MotorSpec {
    pub kv: f64,
    pub weight_g: f64,
    pub stator_size: Option<StatorSize>,
    /// Idle current draw (typical 0.5-2 A). When `None`, the aggregator falls
    /// back to a library default.
    pub no_load_amps: Option<f64>,
    /// Winding resistance. When `None`, derived from stator size / KV.
    pub resistance_ohm: Option<f64>,
}

// =============================================================================
// ESC
// =============================================================================

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EscFirmware {
    BlHeli32,
    Am32,
    Bluejay,
    Other,
}

#[derive(Debug, Clone)]
pub struct EscSpec {
    pub continuous_amps: f64,
    pub burst_amps: Option<f64>,
    pub min_cell_count: u8,
    pub max_cell_count: u8,
    pub firmware: Option<EscFirmware>,
    /// Command response time constant (default ~3 ms).
    pub response_tau_ms: Option<f64>,
    pub weight_g: Option<f64>,
}

// =============================================================================
// Propeller
// =============================================================================

#[derive(Debug, Clone)]
pub struct PropellerSpec {
    pub diameter_in: f64,
    pub pitch_in: f64,
    pub blade_count: i32,
    pub weight_g: Option<f64>,
}

// =============================================================================
// Battery
// =============================================================================

#[derive(Debug, Clone)]
pub struct BatterySpec {
    pub cell_count: u8,
    pub capacity_mah: f64,
    pub c_rating: f64,
    pub weight_g: f64,
    /// Internal resistance in milli-ohms. When `None`, derived from cell count
    /// and C rating in the battery model.
    pub internal_resistance_mohm: Option<f64>,
}

impl BatterySpec {
    /// Nominal battery voltage (`cells * 3.7`).
    pub fn nominal_voltage(&self) -> f64 {
        self.cell_count as f64 * 3.7
    }
}

// =============================================================================
// Sensor noise profiles (Phase 3 — data only, daemon adapts to hitl-sensors)
// =============================================================================

/// IMU noise profile. The values map directly onto
/// `hitl_sensors::ImuConfig` field-by-field at the daemon boundary.
///
/// The gyro and accel noise densities are medians measured at rest on public
/// PX4 flight logs (calibration run 2026-11-02) where that run proposed a value
/// for the variant; every other field, and every variant without a proposal,
/// keeps its hitl-physics 0.13.1 table value. The comment above each
/// measured value in the generated `imu_profile` says what it replaced and how
/// many vehicles it is the median of.
#[derive(Debug, Clone, Copy)]
pub struct ImuProfile {
    /// Gyroscope noise density in rad/s/√Hz.
    pub gyro_noise_density: f64,
    /// Accelerometer noise density in m/s²/√Hz.
    pub accel_noise_density: f64,
    /// Gyroscope bias random-walk sigma in rad/s.
    pub gyro_bias_sigma: f64,
    /// Gyroscope bias time constant in seconds.
    pub gyro_bias_tau: f64,
}

/// Magnetometer noise profile.
///
/// `noise_sigma_gauss` is a median measured on public PX4 flight logs
/// (calibration run 2026-11-02) where that run proposed a value for the
/// variant; otherwise it keeps its hitl-physics 0.13.1 table value.
#[derive(Debug, Clone, Copy)]
pub struct MagProfile {
    /// Measurement noise standard deviation in Gauss.
    pub noise_sigma_gauss: f64,
}

/// Barometer noise profile.
///
/// `noise_sigma_m` is a median measured on public PX4 flight logs (calibration
/// run 2026-11-02) where that run proposed a value for the variant; otherwise
/// it keeps its hitl-physics 0.13.1 table value.
#[derive(Debug, Clone, Copy)]
pub struct BaroProfile {
    /// Altitude measurement noise standard deviation in meters.
    pub noise_sigma_m: f64,
}

/// Aggregate of sensor profiles for a given flight controller. Each profile is
/// measured where the run proposed a value and otherwise carries its hitl-physics
/// 0.13.1 table value, as described on its own type.
#[derive(Debug, Clone, Copy)]
pub struct SensorProfiles {
    pub imu: ImuProfile,
    pub mag: Option<MagProfile>,
    pub baro: BaroProfile,
}

/// GPS receiver noise + acquisition profile (Phase 4 — data only).
///
/// Mirrors `hitl_sensors::GpsConfig` plus fix-acquisition state machine inputs
/// that today's `GpsSensor` doesn't model. The daemon converts to `GpsConfig`
/// and (eventually) feeds the TTFF/sat-count fields into HIL_GPS injection.
///
/// The position, altitude and velocity sigmas and the update rate are medians
/// measured on public PX4 flight logs (calibration run 2026-11-02) where that
/// run proposed a value for the variant. The fix class of the measured values
/// is rtk_fixed for UbloxF9P and SeptentrioMosaic, and standalone_3d for
/// UbloxM8N, UbloxM9N, UbloxM10, Ubx7 and Other. Here3, Ubx6 and Ubx20 have no
/// proposal. Every other field, and every field or variant without a proposal,
/// keeps its hitl-physics 0.13.1 table value.
#[derive(Debug, Clone, Copy)]
pub struct GpsProfile {
    /// Horizontal position sigma in meters.
    pub horizontal_noise_sigma_m: f64,
    /// Vertical position σ in meters.
    pub altitude_noise_sigma_m: f64,
    /// Velocity σ in m/s.
    pub velocity_noise_sigma_mps: f64,
    /// Position drift random-walk σ (m/s equivalent).
    pub position_drift_sigma: f64,
    /// Position drift Gauss-Markov time constant in seconds.
    pub position_drift_tau: f64,
    /// Native update rate in Hz.
    pub update_rate_hz: f64,
    /// Total measurement latency in ms (modem + UART + protocol).
    pub delay_ms: f64,
    /// Cold-start time-to-first-fix in seconds.
    pub ttff_cold_s: f64,
    /// Warm-start TTFF (recent ephemeris) in seconds.
    pub ttff_warm_s: f64,
    /// Hot-start TTFF (almanac + position + time valid) in seconds.
    pub ttff_hot_s: f64,
    /// Maximum satellites the receiver can track concurrently.
    pub max_satellites: u8,
}

// =============================================================================
// Flight controller
// =============================================================================

// Generated by log_validation.part_tables, run 2026-11-02: ImuChip and
// ImuChip::imu_profile for hitl-physics src/build.rs, in the structure of
// hitl-physics 0.13.1. A "was" comment marks a value measured from
// real PX4 logs (median over vehicles).

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ImuChip {
    Mpu6000,
    Mpu6500,
    Icm20689,
    Icm42688p,
    Bmi270,
    Lsm6dso,
    Adis16470,
    Adis16607,
    Bmi055,
    Bmi088,
    Icm20602,
    Icm20608g,
    Icm20649,
    Icm20948,
    Icm42670p,
    Icm42686p,
    Icm45686,
    Iim42652,
    Iim42653,
    Mpu9250,
    Sch16t,
    Other,
}

impl ImuChip {
    /// Noise profile for this IMU chipset.
    ///
    /// A value with a "was" comment is the median over vehicles measured from real PX4 logs in
    /// run 2026-11-02; every other value is the hitl-physics 0.13.1 table value.
    ///
    /// hitl-physics 0.13.1 sources: InvenSense MPU-6000/6500/ICM-20689/ICM-42688P datasheets;
    /// Bosch BMI270; STMicroelectronics LSM6DSO. Bias drift values are
    /// empirical from PX4 EKF2 tuning recommendations.
    pub fn imu_profile(&self) -> ImuProfile {
        match self {
            // used in tests
            ImuChip::Mpu6000 => ImuProfile {
                // was 8.726646e-4 (hitl-physics 0.13.1); median of 70 vehicles, run 2026-11-02
                gyro_noise_density: 7.141e-5,
                // was 6.37e-3 (hitl-physics 0.13.1); median of 70 vehicles, run 2026-11-02
                accel_noise_density: 0.001504,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            ImuChip::Mpu6500 => ImuProfile {
                gyro_noise_density: 9.4e-4,
                accel_noise_density: 6.86e-3,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            ImuChip::Icm20689 => ImuProfile {
                // was 8.73e-5 (hitl-physics 0.13.1); median of 149 vehicles, run 2026-11-02
                gyro_noise_density: 9.295e-5,
                // was 7.85e-4 (hitl-physics 0.13.1); median of 149 vehicles, run 2026-11-02
                accel_noise_density: 0.001502,
                gyro_bias_sigma: 5e-5,
                gyro_bias_tau: 200.0,
            },
            // modern flagship
            ImuChip::Icm42688p => ImuProfile {
                // was 4.88e-5 (hitl-physics 0.13.1); median of 373 vehicles, run 2026-11-02
                gyro_noise_density: 7.997e-5,
                // was 6.86e-4 (hitl-physics 0.13.1); median of 373 vehicles, run 2026-11-02
                accel_noise_density: 0.001368,
                gyro_bias_sigma: 5e-5,
                gyro_bias_tau: 200.0,
            },
            ImuChip::Bmi270 => ImuProfile {
                // was 1.40e-4 (hitl-physics 0.13.1); median of 33 vehicles, run 2026-11-02
                gyro_noise_density: 0.0001092,
                // was 2.45e-3 (hitl-physics 0.13.1); median of 33 vehicles, run 2026-11-02
                accel_noise_density: 0.001326,
                gyro_bias_sigma: 8e-5,
                gyro_bias_tau: 150.0,
            },
            // 0.0035 deg/s/sqrt(Hz), 60 ug/sqrt(Hz)
            ImuChip::Lsm6dso => ImuProfile {
                gyro_noise_density: 6.11e-5,
                accel_noise_density: 5.88e-4,
                gyro_bias_sigma: 6e-5,
                gyro_bias_tau: 180.0,
            },
            // PX4 log names ADIS16470; new in run 2026-11-02
            ImuChip::Adis16470 => ImuProfile {
                // was 8.726646e-4 (Other); median of 17 vehicles, run 2026-11-02
                gyro_noise_density: 7.152e-5,
                // was 6.37e-3 (Other); median of 17 vehicles, run 2026-11-02
                accel_noise_density: 0.001139,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ADIS16607; new in run 2026-11-02
            ImuChip::Adis16607 => ImuProfile {
                // was 8.726646e-4 (Other); median of 10 vehicles, run 2026-11-02
                gyro_noise_density: 0.0001231,
                // was 6.37e-3 (Other); median of 10 vehicles, run 2026-11-02
                accel_noise_density: 0.003539,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names BMI055; new in run 2026-11-02
            ImuChip::Bmi055 => ImuProfile {
                // was 8.726646e-4 (Other); median of 131 vehicles, run 2026-11-02
                gyro_noise_density: 0.0002762,
                // was 6.37e-3 (Other); median of 131 vehicles, run 2026-11-02
                accel_noise_density: 0.001395,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names BMI088; new in run 2026-11-02
            ImuChip::Bmi088 => ImuProfile {
                // was 8.726646e-4 (Other); median of 392 vehicles, run 2026-11-02
                gyro_noise_density: 0.0002713,
                // was 6.37e-3 (Other); median of 392 vehicles, run 2026-11-02
                accel_noise_density: 0.001682,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM20602; new in run 2026-11-02
            ImuChip::Icm20602 => ImuProfile {
                // was 8.726646e-4 (Other); median of 681 vehicles, run 2026-11-02
                gyro_noise_density: 6.602e-5,
                // was 6.37e-3 (Other); median of 681 vehicles, run 2026-11-02
                accel_noise_density: 0.0007635,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM20608G; new in run 2026-11-02
            ImuChip::Icm20608g => ImuProfile {
                // was 8.726646e-4 (Other); median of 13 vehicles, run 2026-11-02
                gyro_noise_density: 9.93e-5,
                // was 6.37e-3 (Other); median of 13 vehicles, run 2026-11-02
                accel_noise_density: 0.002125,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM20649; new in run 2026-11-02
            ImuChip::Icm20649 => ImuProfile {
                // was 8.726646e-4 (Other); median of 28 vehicles, run 2026-11-02
                gyro_noise_density: 0.000185,
                // was 6.37e-3 (Other); median of 28 vehicles, run 2026-11-02
                accel_noise_density: 0.001699,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM20948; new in run 2026-11-02
            ImuChip::Icm20948 => ImuProfile {
                // was 8.726646e-4 (Other); median of 22 vehicles, run 2026-11-02
                gyro_noise_density: 0.0001125,
                // was 6.37e-3 (Other); median of 22 vehicles, run 2026-11-02
                accel_noise_density: 0.001322,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM42670P; new in run 2026-11-02
            ImuChip::Icm42670p => ImuProfile {
                // was 8.726646e-4 (Other); median of 79 vehicles, run 2026-11-02
                gyro_noise_density: 5.957e-5,
                // was 6.37e-3 (Other); median of 79 vehicles, run 2026-11-02
                accel_noise_density: 0.0006262,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM42686P; new in run 2026-11-02
            ImuChip::Icm42686p => ImuProfile {
                // was 8.726646e-4 (Other); median of 19 vehicles, run 2026-11-02
                gyro_noise_density: 7.024e-5,
                // was 6.37e-3 (Other); median of 19 vehicles, run 2026-11-02
                accel_noise_density: 0.001483,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names ICM45686; new in run 2026-11-02
            ImuChip::Icm45686 => ImuProfile {
                // was 8.726646e-4 (Other); median of 259 vehicles, run 2026-11-02
                gyro_noise_density: 5.727e-5,
                // was 6.37e-3 (Other); median of 259 vehicles, run 2026-11-02
                accel_noise_density: 0.001229,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names IIM42652; new in run 2026-11-02
            ImuChip::Iim42652 => ImuProfile {
                // was 8.726646e-4 (Other); median of 81 vehicles, run 2026-11-02
                gyro_noise_density: 0.0001204,
                // was 6.37e-3 (Other); median of 81 vehicles, run 2026-11-02
                accel_noise_density: 0.001308,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names IIM42653; new in run 2026-11-02
            ImuChip::Iim42653 => ImuProfile {
                // was 8.726646e-4 (Other); median of 31 vehicles, run 2026-11-02
                gyro_noise_density: 0.0001228,
                // was 6.37e-3 (Other); median of 31 vehicles, run 2026-11-02
                accel_noise_density: 0.002386,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names MPU9250; new in run 2026-11-02
            ImuChip::Mpu9250 => ImuProfile {
                // was 8.726646e-4 (Other); median of 31 vehicles, run 2026-11-02
                gyro_noise_density: 9.769e-5,
                // was 6.37e-3 (Other); median of 31 vehicles, run 2026-11-02
                accel_noise_density: 0.0009651,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // PX4 log names SCH16T; new in run 2026-11-02
            ImuChip::Sch16t => ImuProfile {
                // was 8.726646e-4 (Other); median of 61 vehicles, run 2026-11-02
                gyro_noise_density: 2.835e-5,
                // was 6.37e-3 (Other); median of 61 vehicles, run 2026-11-02
                accel_noise_density: 0.0007964,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
            // Unknown part - median over all vehicles of this part type (TH3SEUS's fallback entry)
            ImuChip::Other => ImuProfile {
                // was 8.726646e-4 (hitl-physics 0.13.1); median of 2434 vehicles, run 2026-11-02
                gyro_noise_density: 9.572e-5,
                // was 6.37e-3 (hitl-physics 0.13.1); median of 2434 vehicles, run 2026-11-02
                accel_noise_density: 0.001274,
                gyro_bias_sigma: 1e-4,
                gyro_bias_tau: 100.0,
            },
        }
    }
}

// Generated by log_validation.part_tables, run 2026-11-02: MagChip and
// MagChip::mag_profile for hitl-physics src/build.rs, in the structure of
// hitl-physics 0.13.1. A "was" comment marks a value measured from
// real PX4 logs (median over vehicles).

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MagChip {
    Hmc5883,
    Qmc5883,
    Ist8310,
    Rm3100,
    Lis3mdl,
    Ak09916,
    Bmm150,
    Bmm350,
    Iis2mdc,
    Qmc5883p,
    None,
    Other,
}

impl MagChip {
    /// Noise profile, or `None` for a build without a mag.
    ///
    /// A value with a "was" comment is the median over vehicles measured from real PX4 logs in
    /// run 2026-11-02; every other value is the hitl-physics 0.13.1 table value.
    pub fn mag_profile(&self) -> Option<MagProfile> {
        match self {
            MagChip::Hmc5883 => Some(MagProfile {
                // was 0.0006 (hitl-physics 0.13.1); median of 15 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.003405,
            }),
            // cheap, temperature-unstable
            MagChip::Qmc5883 => Some(MagProfile {
                // was 0.0005 (hitl-physics 0.13.1); median of 98 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.003216,
            }),
            MagChip::Ist8310 => Some(MagProfile {
                // was 0.0003 (hitl-physics 0.13.1); median of 410 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.002705,
            }),
            MagChip::Rm3100 => Some(MagProfile {
                // was 0.000013 (hitl-physics 0.13.1); median of 104 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.001544,
            }),
            MagChip::Lis3mdl => Some(MagProfile {
                // was 0.00032 (hitl-physics 0.13.1); median of 26 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.008952,
            }),
            // PX4 log names AK09916; new in run 2026-11-02
            MagChip::Ak09916 => Some(MagProfile {
                // was 0.0006 (Other); median of 113 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.009422,
            }),
            // PX4 log names BMM150; new in run 2026-11-02
            MagChip::Bmm150 => Some(MagProfile {
                // was 0.0006 (Other); median of 239 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.003614,
            }),
            // PX4 log names BMM350; new in run 2026-11-02
            MagChip::Bmm350 => Some(MagProfile {
                // was 0.0006 (Other); median of 10 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.002767,
            }),
            // PX4 log names IIS2MDC; new in run 2026-11-02
            MagChip::Iis2mdc => Some(MagProfile {
                // was 0.0006 (Other); median of 31 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.003449,
            }),
            // PX4 log names QMC5883P; new in run 2026-11-02
            MagChip::Qmc5883p => Some(MagProfile {
                // was 0.0006 (Other); median of 19 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.01207,
            }),
            MagChip::None => None,
            // Unknown part - median over all vehicles of this part type (TH3SEUS's fallback entry)
            MagChip::Other => Some(MagProfile {
                // was 0.0006 (hitl-physics 0.13.1); median of 1122 vehicles, run 2026-11-02
                noise_sigma_gauss: 0.003485,
            }),
        }
    }
}

// Generated by log_validation.part_tables, run 2026-11-02: BaroChip and
// BaroChip::baro_profile for hitl-physics src/build.rs, in the structure of
// hitl-physics 0.13.1. A "was" comment marks a value measured from
// real PX4 logs (median over vehicles).

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BaroChip {
    Bmp280,
    Bmp388,
    Dps310,
    Spl06,
    Ms5611,
    Bmp581,
    Icp201xx,
    Other,
}

impl BaroChip {
    /// Altitude noise standard deviation.
    ///
    /// A value with a "was" comment is the median over vehicles measured from real PX4 logs in
    /// run 2026-11-02; every other value is the hitl-physics 0.13.1 table value.
    pub fn baro_profile(&self) -> BaroProfile {
        match self {
            BaroChip::Bmp280 => BaroProfile {
                noise_sigma_m: 0.16,
            },
            BaroChip::Bmp388 => BaroProfile {
                // was 0.08 (hitl-physics 0.13.1); median of 78 vehicles, run 2026-11-02
                noise_sigma_m: 0.1059,
            },
            BaroChip::Dps310 => BaroProfile {
                noise_sigma_m: 0.05,
            },
            BaroChip::Spl06 => BaroProfile {
                // was 0.10 (hitl-physics 0.13.1); median of 18 vehicles, run 2026-11-02
                noise_sigma_m: 0.02095,
            },
            BaroChip::Ms5611 => BaroProfile {
                // was 0.10 (hitl-physics 0.13.1); median of 646 vehicles, run 2026-11-02
                noise_sigma_m: 0.2276,
            },
            // PX4 log names BMP581; new in run 2026-11-02
            BaroChip::Bmp581 => BaroProfile {
                // was 0.15 (Other); median of 15 vehicles, run 2026-11-02
                noise_sigma_m: 0.02184,
            },
            // PX4 log names ICP201XX; new in run 2026-11-02
            BaroChip::Icp201xx => BaroProfile {
                // was 0.15 (Other); median of 117 vehicles, run 2026-11-02
                noise_sigma_m: 0.1903,
            },
            // Unknown part - median over all vehicles of this part type (TH3SEUS's fallback entry)
            BaroChip::Other => BaroProfile {
                // was 0.15 (hitl-physics 0.13.1); median of 915 vehicles, run 2026-11-02
                noise_sigma_m: 0.2198,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct FlightControllerSpec {
    pub mcu: Option<String>,
    pub imu_chip: Option<ImuChip>,
    pub mag_chip: Option<MagChip>,
    pub baro_chip: Option<BaroChip>,
    pub loop_rate_hz: u32,
    pub gyro_filter_hz: u32,
    pub weight_g: f64,
}

// =============================================================================
// GPS
// =============================================================================

// Generated by log_validation.part_tables, run 2026-11-02: GpsChipset and
// GpsChipset::gps_profile for hitl-physics src/build.rs, in the structure of
// hitl-physics 0.13.1. A "was" comment marks a value measured from
// real PX4 logs (median over vehicles).

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpsChipset {
    UbloxM8N,
    UbloxM9N,
    UbloxM10,
    UbloxF9P,
    SeptentrioMosaic,
    Here3,
    Ubx7,
    Other,
}

impl GpsChipset {
    /// GPS receiver profile.
    ///
    /// A value with a "was" comment is the median over vehicles measured from real PX4 logs in
    /// run 2026-11-02; every other value is the hitl-physics 0.13.1 table value.
    ///
    /// hitl-physics 0.13.1 sources: u-blox M8/M9/M10/F9P product summaries; Septentrio Mosaic-X5
    /// brochure; Here3 (CubePilot) docs. CEP values are typical autonomous
    /// performance - RTK figures are post-base-station-correction (F9P,
    /// Mosaic). TTFF values are from the respective product specs.
    pub fn gps_profile(&self) -> GpsProfile {
        match self {
            // u-blox NEO-M8N; GPS+GLONASS; measured over standalone 3D fixes
            GpsChipset::UbloxM8N => GpsProfile {
                // was 1.5 (hitl-physics 0.13.1); median of 302 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 1.198,
                // was 3.0 (hitl-physics 0.13.1); median of 302 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 1.87,
                // was 0.10 (hitl-physics 0.13.1); median of 302 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.268,
                position_drift_sigma: 0.06,
                position_drift_tau: 30.0,
                // was 5.0 (hitl-physics 0.13.1); median of 23 vehicles, run 2026-11-02
                update_rate_hz: 5.0,
                delay_ms: 120.0,
                ttff_cold_s: 26.0,
                ttff_warm_s: 5.0,
                ttff_hot_s: 1.0,
                max_satellites: 22,
            },
            // u-blox NEO-M9N; 4-constellation; measured over standalone 3D fixes
            GpsChipset::UbloxM9N => GpsProfile {
                // was 0.9 (hitl-physics 0.13.1); median of 1100 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 0.8705,
                // was 1.5 (hitl-physics 0.13.1); median of 1100 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 1.43,
                // was 0.05 (hitl-physics 0.13.1); median of 1100 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.2344,
                position_drift_sigma: 0.04,
                position_drift_tau: 40.0,
                // was 10.0 (hitl-physics 0.13.1); median of 78 vehicles, run 2026-11-02
                update_rate_hz: 8.001,
                delay_ms: 80.0,
                ttff_cold_s: 24.0,
                ttff_warm_s: 2.0,
                ttff_hot_s: 1.0,
                max_satellites: 33,
            },
            // u-blox M10; single-band; measured over standalone 3D fixes
            GpsChipset::UbloxM10 => GpsProfile {
                // was 1.0 (hitl-physics 0.13.1); median of 47 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 2.074,
                // was 1.8 (hitl-physics 0.13.1); median of 47 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 2.609,
                // was 0.06 (hitl-physics 0.13.1); median of 37 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.328,
                position_drift_sigma: 0.04,
                position_drift_tau: 40.0,
                update_rate_hz: 10.0,
                delay_ms: 90.0,
                ttff_cold_s: 24.0,
                ttff_warm_s: 2.0,
                ttff_hot_s: 1.0,
                max_satellites: 32,
            },
            // u-blox ZED-F9P; measured over RTK fixed solutions
            GpsChipset::UbloxF9P => GpsProfile {
                // was 0.014 (hitl-physics 0.13.1); median of 359 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 0.0141,
                // was 0.02 (hitl-physics 0.13.1); median of 359 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 0.01,
                // was 0.015 (hitl-physics 0.13.1); median of 359 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.094,
                position_drift_sigma: 0.005,
                position_drift_tau: 60.0,
                // was 8.0 (hitl-physics 0.13.1); median of 33 vehicles, run 2026-11-02
                update_rate_hz: 5.051,
                delay_ms: 60.0,
                ttff_cold_s: 25.0,
                ttff_warm_s: 2.0,
                ttff_hot_s: 1.0,
                max_satellites: 60,
            },
            // Septentrio Mosaic-X5; 100 Hz; survey-grade; measured over RTK fixed solutions
            GpsChipset::SeptentrioMosaic => GpsProfile {
                // was 0.006 (hitl-physics 0.13.1); median of 11 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 0.01,
                // was 0.012 (hitl-physics 0.13.1); median of 11 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 0.02,
                // was 0.010 (hitl-physics 0.13.1); median of 11 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.01457,
                position_drift_sigma: 0.003,
                position_drift_tau: 60.0,
                update_rate_hz: 100.0,
                delay_ms: 40.0,
                ttff_cold_s: 45.0,
                ttff_warm_s: 20.0,
                ttff_hot_s: 5.0,
                max_satellites: 80,
            },
            // CubePilot Here3 (CAN GPS): built on u-blox M9N; delay_ms 100.0 is CAN bus latency
            GpsChipset::Here3 => GpsProfile {
                horizontal_noise_sigma_m: 0.9,
                altitude_noise_sigma_m: 1.5,
                velocity_noise_sigma_mps: 0.05,
                position_drift_sigma: 0.04,
                position_drift_tau: 40.0,
                update_rate_hz: 8.0,
                delay_ms: 100.0,
                ttff_cold_s: 24.0,
                ttff_warm_s: 2.0,
                ttff_hot_s: 1.0,
                max_satellites: 33,
            },
            // PX4 log names UBX_7; new in run 2026-11-02; measured over standalone 3D fixes
            GpsChipset::Ubx7 => GpsProfile {
                // was 1.5 (Other); median of 16 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 2.042,
                // was 3.0 (Other); median of 16 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 3.373,
                // was 0.10 (Other); median of 16 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.4266,
                position_drift_sigma: 0.06,
                position_drift_tau: 30.0,
                update_rate_hz: 5.0,
                delay_ms: 120.0,
                ttff_cold_s: 26.0,
                ttff_warm_s: 5.0,
                ttff_hot_s: 1.0,
                max_satellites: 22,
            },
            // Unknown part - median over all vehicles of this part type, standalone 3D fix only (TH3SEUS's fallback entry)
            GpsChipset::Other => GpsProfile {
                // was 1.5 (hitl-physics 0.13.1); median of 5855 vehicles, run 2026-11-02
                horizontal_noise_sigma_m: 0.812,
                // was 3.0 (hitl-physics 0.13.1); median of 5854 vehicles, run 2026-11-02
                altitude_noise_sigma_m: 1.25,
                // was 0.10 (hitl-physics 0.13.1); median of 5748 vehicles, run 2026-11-02
                velocity_noise_sigma_mps: 0.1391,
                position_drift_sigma: 0.06,
                position_drift_tau: 30.0,
                // was 5.0 (hitl-physics 0.13.1); median of 700 vehicles, run 2026-11-02
                update_rate_hz: 5.0,
                delay_ms: 120.0,
                ttff_cold_s: 26.0,
                ttff_warm_s: 5.0,
                ttff_hot_s: 1.0,
                max_satellites: 22,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct GpsSpec {
    pub chipset: GpsChipset,
    pub update_rate_hz: f64,
    pub has_compass: bool,
    pub weight_g: f64,
}

// =============================================================================
// BuildSpec — top-level aggregator
// =============================================================================

#[derive(Debug, Clone)]
pub struct BuildSpec {
    pub frame: FrameSpec,
    /// All four motors are assumed identical.
    pub motors: MotorSpec,
    pub escs: EscSpec,
    pub propellers: PropellerSpec,
    pub battery: BatterySpec,
    pub flight_controller: FlightControllerSpec,
    pub gps: Option<GpsSpec>,
}

impl BuildSpec {
    /// Derive a [`PhysicsConfig`] from this build specification.
    ///
    /// **Default / safe path.** Delegates to [`PhysicsConfig::from_build_specs`]
    /// for behavioral parity with the pre-component code path, including the
    /// `0.012`/`0.020` inertia floors that PX4's default rate PIDs require.
    /// Use this until Phase 6 ships per-build PID auto-tuning.
    ///
    /// **Note:** This method overrides `mass_kg` with `total_mass_kg()` which
    /// sums all components (frame, motors, ESCs, props, battery, FC, GPS).
    /// The legacy `from_build_specs` only counted frame + motors.
    pub fn to_physics_config(&self) -> PhysicsConfig {
        let mut physics = PhysicsConfig::from_build_specs(
            self.motors.kv,
            self.propellers.diameter_in,
            self.propellers.pitch_in,
            self.propellers.blade_count,
            self.frame.weight_g,
            self.motors.weight_g,
            self.battery.nominal_voltage(),
        );

        // Override mass with total from all components (frame, motors, ESCs,
        // props, battery, FC, GPS). from_build_specs only counts frame + motors.
        physics.mass_kg = self.total_mass_kg();

        // Phase 5: derive kt / kq from the propeller coefficient lookup so the
        // thrust curve no longer conflates prop with motor KV. Anchor entries
        // in `data/prop_coefficients.csv` preserve current hover behavior for
        // reference builds; other (D, P, B) combinations now scale per
        // bench-test ratios instead of the empirical linear multipliers.
        let coef = prop_coefficients::lookup(&self.propellers);
        let (kt, kq) = prop_coefficients::coefficients_to_kt_kq(coef, self.propellers.diameter_in);
        physics.kt = kt;
        physics.kq = kq;

        // Phase 2a: pipe motor/ESC fields when specified on the BuildSpec.
        if let Some(no_load) = self.motors.no_load_amps {
            physics.motor_no_load_amps = no_load;
        }
        if let Some(r_ohm) = self.motors.resistance_ohm {
            physics.motor_resistance_ohm = r_ohm;
        }
        physics.esc = EscConfig {
            continuous_amps: self.escs.continuous_amps,
            burst_amps: self
                .escs
                .burst_amps
                .unwrap_or(self.escs.continuous_amps * 1.2),
            response_tau_s: self.escs.response_tau_ms.unwrap_or(3.0) / 1000.0,
            ..EscConfig::default()
        };

        // The operating point is capped by whichever runs out first: the ESC's
        // burst rating, or the pack's C-rating shared across four motors.
        // Burst rather than continuous, because a static full-throttle figure
        // is a momentary number — sustained draw is the thermal model's job.
        let esc_limit = physics.esc.burst_amps;
        let pack_limit = self.battery.capacity_mah / 1000.0 * self.battery.c_rating / 4.0;
        physics.max_current_per_motor_a = esc_limit.min(pack_limit);

        physics
    }

    /// Derive a GPS profile from the build's GPS chipset selection.
    /// Returns `None` if the build doesn't include a GPS module.
    pub fn to_gps_profile(&self) -> Option<GpsProfile> {
        self.gps.as_ref().map(|g| g.chipset.gps_profile())
    }

    /// Derive sensor noise profiles from the build's flight controller chipset
    /// selection. Returns a `SensorProfiles` bundle that the daemon converts
    /// into `hitl_sensors::{ImuConfig,MagConfig,BaroConfig}`.
    ///
    /// A missing IMU or barometer selection falls back to the `Other` variant
    /// of that chip enum. Since 0.14.0 `Other` holds the pooled median of the
    /// calibration run 2026-11-02 over every vehicle of the part type (2434 for
    /// the IMU, 915 for the barometer), not a datasheet part. A build with no
    /// magnetometer selection gets `None`, not `MagChip::Other`.
    pub fn to_sensor_profiles(&self) -> SensorProfiles {
        let imu = self
            .flight_controller
            .imu_chip
            .as_ref()
            .map(|c| c.imu_profile())
            .unwrap_or_else(|| ImuChip::Other.imu_profile());

        let mag = self
            .flight_controller
            .mag_chip
            .as_ref()
            .and_then(|c| c.mag_profile());

        let baro = self
            .flight_controller
            .baro_chip
            .as_ref()
            .map(|c| c.baro_profile())
            .unwrap_or_else(|| BaroChip::Other.baro_profile());

        SensorProfiles { imu, mag, baro }
    }

    /// Derive a [`BatteryConfig`] from this build specification.
    ///
    /// Includes Phase 2b internal-resistance computation (defaulted from
    /// `c_rating`/`cell_count`, overridden by `BatterySpec.internal_resistance_mohm`
    /// when present).
    pub fn to_battery_config(&self) -> BatteryConfig {
        let mut cfg = BatteryConfig::new(
            self.battery.cell_count,
            self.battery.capacity_mah,
            self.battery.c_rating,
        );
        if let Some(r_mohm) = self.battery.internal_resistance_mohm {
            cfg.internal_resistance_ohm = r_mohm / 1000.0;
        }
        cfg
    }

    /// Derive a [`PhysicsConfig`] using **real frame and component geometry** —
    /// no inertia floor, no prop-diameter arm-length hack, no prop-area drag hack.
    ///
    /// Mass, arm length, inertia tensor, and drag coefficients are recomputed
    /// from the build's actual components. Everything else (`kt`, `kq`,
    /// `tau_motor`, electrical/thermal parameters) is inherited from the safe
    /// path; Phase 2/5 will replace those too.
    ///
    /// **WARNING:** Real inertia is typically 2-4× smaller than the floor-enforced
    /// values PX4's default PIDs were tuned for. Using this path without Phase 6
    /// PID auto-tune *will* produce attitude oscillation on light builds.
    pub fn to_physics_config_physical(&self) -> PhysicsConfig {
        let mut physics = self.to_physics_config();
        physics.mass_kg = self.total_mass_kg();
        physics.arm_length_m = self.arm_length_m();
        physics.inertia = self.compute_inertia();
        physics.drag_coeffs = self.compute_drag_coeffs();
        physics
    }

    // ---------------------------------------------------------------------
    // Geometry helpers (Phase 1)
    // ---------------------------------------------------------------------

    /// Total airframe mass in kg, summed across all configured components.
    ///
    /// `escs.weight_g` is treated as the **total** ESC system mass (whether 4
    /// individual ESCs or a 4-in-1 stack). Prop weight defaults to 3 g per blade
    /// assembly when missing.
    pub fn total_mass_kg(&self) -> f64 {
        let prop_each = self.propellers.weight_g.unwrap_or(3.0);
        let esc_total = self.escs.weight_g.unwrap_or(12.0);
        let gps = self.gps.as_ref().map(|g| g.weight_g).unwrap_or(0.0);
        let grams = self.frame.weight_g
            + 4.0 * self.motors.weight_g
            + 4.0 * prop_each
            + esc_total
            + self.battery.weight_g
            + self.flight_controller.weight_g
            + gps;
        grams / 1000.0
    }

    /// Arm length (center → motor) in meters.
    ///
    /// `wheelbase_mm` is the motor-to-motor diagonal, so arm = wheelbase / 2
    /// for any symmetric Quad-X frame.
    pub fn arm_length_m(&self) -> f64 {
        (self.frame.wheelbase_mm / 1000.0) / 2.0
    }

    /// Frame outer dimensions (width_x, length_y, height_z) in meters.
    ///
    /// Falls back to a square plate with side `wheelbase / √2` and a 30 mm
    /// height when the DB row lacks explicit dimensions. Logged at debug, not
    /// warn — most stock DB rows won't have these populated yet.
    pub fn frame_bounding_box_m(&self) -> (f64, f64, f64) {
        let l_default = (self.frame.wheelbase_mm / 1000.0) / 2f64.sqrt();
        (
            self.frame.width_mm.map(|w| w / 1000.0).unwrap_or(l_default),
            self.frame
                .length_mm
                .map(|l| l / 1000.0)
                .unwrap_or(l_default),
            self.frame.height_mm.map(|h| h / 1000.0).unwrap_or(0.030),
        )
    }

    /// Moment of inertia tensor `[Ixx, Iyy, Izz]` in kg·m², computed from
    /// component point-masses and a hybrid frame model (70% mass in 4 arms
    /// modelled as rods from origin along the X diagonals, 30% in a small
    /// central plate). No floor enforcement.
    ///
    /// Returned values reflect actual airframe physics — typically 2-4× smaller
    /// than the legacy floor-enforced values.
    pub fn compute_inertia(&self) -> [f64; 3] {
        let l = self.arm_length_m();
        let arm_offset_diag = l / 2f64.sqrt();

        // Motor + prop + 1/4 of the ESC stack as a point mass at each arm tip.
        let m_arm_tip_kg = (self.motors.weight_g
            + self.propellers.weight_g.unwrap_or(3.0)
            + self.escs.weight_g.unwrap_or(12.0) / 4.0)
            / 1000.0;
        let ixx_motors = 4.0 * m_arm_tip_kg * arm_offset_diag.powi(2);
        let iyy_motors = ixx_motors;
        let izz_motors = 4.0 * m_arm_tip_kg * l.powi(2);

        // Frame mass split: 70% in 4 arms (carbon-tube rods), 30% in central plate.
        let m_frame_kg = self.frame.weight_g / 1000.0;
        let m_arms_total = m_frame_kg * 0.70;
        let m_plate = m_frame_kg * 0.30;

        // 4 uniform rods from origin along ±x±y diagonals, each of length l.
        // For a single rod: Ixx-contribution = m·l²/6, Izz-contribution = m·l²/3.
        // Summed over 4 rods of mass m_arms_total/4 each:
        let ixx_arms = m_arms_total * l.powi(2) / 6.0;
        let iyy_arms = ixx_arms;
        let izz_arms = m_arms_total * l.powi(2) / 3.0;

        // Central plate ~80 mm × 80 mm × 20 mm.
        let plate_side: f64 = 0.080;
        let plate_thick: f64 = 0.020;
        let ixx_plate = m_plate * (plate_side.powi(2) + plate_thick.powi(2)) / 12.0;
        let iyy_plate = ixx_plate;
        let izz_plate = m_plate * 2.0 * plate_side.powi(2) / 12.0;

        // Battery near CG, ~15 mm below the centerline.
        let m_bat = self.battery.weight_g / 1000.0;
        let d_bat: f64 = 0.015;
        let ixx_bat = m_bat * d_bat.powi(2);
        let iyy_bat = ixx_bat;
        // Battery is roughly centered in x-y → no Izz contribution.

        // FC + GPS contributions are negligible (gram-scale at sub-cm offsets).
        [
            ixx_motors + ixx_arms + ixx_plate + ixx_bat,
            iyy_motors + iyy_arms + iyy_plate + iyy_bat,
            izz_motors + izz_arms + izz_plate,
        ]
    }

    /// Quadratic drag coefficients `[c_x, c_y, c_z]` such that
    /// `F_drag_axis = -c_axis · v · |v|`.
    ///
    /// Computed as `0.5·ρ·Cd·A` where A is the effective frontal area:
    /// - Lateral: frame side cross-section + 30% of total prop-disc area
    ///   (spinning-prop edge-on contribution).
    /// - Vertical: frame top cross-section + 80% of total prop-disc area.
    ///
    /// Cd values: 1.1 lateral (open-frame quad), 1.5 vertical (flat bottom).
    pub fn compute_drag_coeffs(&self) -> [f64; 3] {
        const RHO: f64 = 1.225;
        let prop_d_m = self.propellers.diameter_in * 0.0254;
        let a_disc = std::f64::consts::PI * (prop_d_m / 2.0).powi(2);
        let (w, l_y, h) = self.frame_bounding_box_m();

        let a_lat_x = w * h + 4.0 * a_disc * 0.30;
        let a_lat_y = l_y * h + 4.0 * a_disc * 0.30;
        let a_top = w * l_y + 4.0 * a_disc * 0.80;

        [
            0.5 * RHO * 1.1 * a_lat_x,
            0.5 * RHO * 1.1 * a_lat_y,
            0.5 * RHO * 1.5 * a_top,
        ]
    }
}

impl BuildSpec {
    // =========================================================================
    // Reference builds — well-documented real quads for Phase 7 validation.
    // Each includes a comment citing the source of measured/spec data.
    // =========================================================================

    // ALL FOUR reference builds were audited against manufacturer sources on
    // 2026-08-29 and every one of them was materially wrong; several described
    // a different aircraft than their name claimed. The corrections below each
    // carry the page they came from.
    //
    // `frame.weight_g` is a CATCH-ALL, not the bare frame kit. Published dry
    // weights are for the whole aircraft — including camera, VTX, antennas,
    // receiver, wiring and hardware — and `BuildSpec` has no field for any of
    // that. Putting the remainder in `frame` is what makes each build's dry sum
    // match its published figure. The alternative, modelling only the parts
    // that have fields, left `impulse_apex_5` 93 g short of a measured build
    // and inflated its thrust-to-weight by about 23%.

    /// EMAX Tinyhawk III — 1S 76 mm whoop, ~44.5 g all-up.
    ///
    /// Sources: EMAX BNF instruction manual v2.0 (32 g dry, TH0802 II 15000KV,
    /// 76 mm wheelbase, 5 A 4-in-1 AIO), NextFPV Avia TH listing (40 mm
    /// four-blade, 31 mm pitch, 0.61 g), ReadyMadeRC (1S HV 450 mAh, 12.5 g).
    ///
    /// The previous record described a different machine: 2S, 12000KV, 3-inch
    /// three-blade props and 75 g. The prop diameter mattered most — `kt`
    /// scales with D⁴, so 3.0" against a real 1.57" is a factor of 13.3, and
    /// every thrust figure this build produced was meaningless.
    pub fn tinyhawk_3() -> Self {
        Self {
            frame: FrameSpec {
                // 32 g published dry, less motors (10.0) and props (2.44).
                // Covers the AIO, canopy, camera, VTX and wiring.
                weight_g: 19.56,
                wheelbase_mm: 76.0,
                width_mm: Some(105.0),
                length_mm: Some(105.0),
                height_mm: Some(45.0),
                material: Some(FrameMaterial::Polymer),
            },
            motors: MotorSpec {
                kv: 15_000.0, // TH0802 II
                weight_g: 2.5,
                stator_size: Some(StatorSize {
                    diameter_mm: 8.0,
                    height_mm: 2.0,
                }),
                no_load_amps: None,
                resistance_ohm: None,
            },
            escs: EscSpec {
                continuous_amps: 5.0,
                // Not published for the base model's 5 A AIO. The 6.7 A figure
                // in circulation belongs to the Tinyhawk III PLUS, a different
                // product, so the honest ceiling is the published continuous
                // rating rather than an invented burst.
                burst_amps: Some(5.0),
                min_cell_count: 1,
                max_cell_count: 2,
                firmware: Some(EscFirmware::Bluejay),
                response_tau_ms: Some(2.0),
                weight_g: Some(0.0), // On the AIO, counted in `frame`
            },
            propellers: PropellerSpec {
                diameter_in: 1.57, // Avia TH, 40 mm
                pitch_in: 1.22,    // 31 mm
                blade_count: 4,
                weight_g: Some(0.61),
            },
            battery: BatterySpec {
                cell_count: 1,
                capacity_mah: 450.0,
                // PRINTED 80C. 80C on 450 mAh is 36 A from a 12.5 g cell, which
                // is not credible — this is a label, not a measurement, and it
                // sits above the 5 A ESC so the ESC is what actually binds.
                c_rating: 80.0,
                weight_g: 12.5,
                internal_resistance_mohm: Some(35.0),
            },
            flight_controller: FlightControllerSpec {
                mcu: Some("F4".into()),
                imu_chip: Some(ImuChip::Mpu6000),
                mag_chip: None,
                baro_chip: None,
                loop_rate_hz: 8000,
                gyro_filter_hz: 200,
                weight_g: 0.0, // On the AIO, counted in `frame`
            },
            gps: None,
        }
    }

    /// ImpulseRC Apex 5" — 5" freestyle, ~542 g all-up.
    ///
    /// Sources: frame 115 g and ESC 45/55 A from retailer listings (ImpulseRC
    /// ceased trading in 2026 and publishes nothing); 378 g dry / 542 g all-up
    /// from a documented build of this exact frame with a 4S 1300 pack
    /// (rotorbuilds.com/build/26613).
    ///
    /// The previous record computed 440 g all-up while its own doc comment
    /// claimed 680 g. 440 g allocated nothing to camera, VTX, antennas,
    /// receiver or wiring — 93 g short of the measured dry weight.
    pub fn impulse_apex_5() -> Self {
        Self {
            frame: FrameSpec {
                // 378 g measured dry, less motors (132), props (18),
                // ESC (13.6) and FC (8). The frame kit alone is 115 g; the
                // remaining ~91 g is the FPV gear the model has no field for.
                weight_g: 206.4,
                wheelbase_mm: 220.0,
                width_mm: Some(180.0),
                length_mm: Some(180.0),
                height_mm: Some(35.0),
                material: Some(FrameMaterial::Carbon),
            },
            motors: MotorSpec {
                kv: 1700.0,
                weight_g: 33.0,
                stator_size: Some(StatorSize {
                    diameter_mm: 23.0,
                    height_mm: 6.0,
                }),
                no_load_amps: Some(0.5),
                resistance_ohm: Some(0.065),
            },
            escs: EscSpec {
                continuous_amps: 45.0,
                burst_amps: Some(55.0),
                min_cell_count: 4,
                max_cell_count: 6,
                firmware: Some(EscFirmware::BlHeli32),
                response_tau_ms: Some(3.0),
                weight_g: Some(13.6), // Holybro Tekko32 F3 45A 4-in-1
            },
            propellers: PropellerSpec {
                diameter_in: 5.0,
                pitch_in: 4.3,
                blade_count: 3,
                weight_g: Some(4.5),
            },
            battery: BatterySpec {
                cell_count: 4,
                capacity_mah: 1300.0,
                c_rating: 75.0, // Printed
                weight_g: 164.0,
                internal_resistance_mohm: None,
            },
            flight_controller: FlightControllerSpec {
                mcu: Some("F7".into()),
                imu_chip: Some(ImuChip::Icm42688p),
                mag_chip: None,
                baro_chip: Some(BaroChip::Bmp280),
                loop_rate_hz: 8000,
                gyro_filter_hz: 250,
                weight_g: 8.0,
            },
            gps: None,
        }
    }

    /// GEPRC MARK5 — 5" freestyle on 6S, ~611 g all-up.
    ///
    /// **This is a five-inch quad.** The previous name and record said six.
    /// GEPRC's MARK5 Series is ten products on the same 225 mm five-inch
    /// airframe; the "6" in every listing title is 6S, the cell count. GEPRC's
    /// actual six-inch freestyle quads are the Vapor-X6 and Vapor-D6.
    ///
    /// Sources: GEPRC MARK5 Analog 6S TBS (385.0 g dry, 225 mm), SPEEDX2
    /// 2107.5 (1960KV on 6S, 29.5 g, 21.0 x 7.5 mm stator), GEP-BL32 50A 4-in-1
    /// (50 A / 55 A burst, 14 g), HQ Ethix S5 5x4x3 (3.7 g), and GEPRC's own
    /// recommended pack, the Storm 6S 1400 mAh (226 g, printed 120C).
    pub fn geprc_mark5() -> Self {
        Self {
            frame: FrameSpec {
                // 385.0 g published dry, less motors (118), props (14.8),
                // ESC (14) and FC (10).
                weight_g: 228.2,
                wheelbase_mm: 225.0,
                width_mm: Some(185.0),
                length_mm: Some(185.0),
                height_mm: Some(40.0),
                material: Some(FrameMaterial::Carbon),
            },
            motors: MotorSpec {
                kv: 1960.0, // SPEEDX2 2107.5, 6S winding
                weight_g: 29.5,
                stator_size: Some(StatorSize {
                    diameter_mm: 21.0,
                    height_mm: 7.5,
                }),
                no_load_amps: Some(0.6),
                resistance_ohm: None,
            },
            escs: EscSpec {
                continuous_amps: 50.0,
                burst_amps: Some(55.0), // "55A (5 second)"
                min_cell_count: 2,
                max_cell_count: 6,
                firmware: Some(EscFirmware::BlHeli32),
                response_tau_ms: Some(3.0),
                weight_g: Some(14.0),
            },
            propellers: PropellerSpec {
                diameter_in: 5.0,
                pitch_in: 4.0,
                blade_count: 3,
                weight_g: Some(3.7),
            },
            battery: BatterySpec {
                cell_count: 6,
                capacity_mah: 1400.0,
                c_rating: 120.0, // Printed on the Storm pack
                weight_g: 226.0,
                internal_resistance_mohm: None,
            },
            flight_controller: FlightControllerSpec {
                mcu: Some("F7".into()),
                imu_chip: Some(ImuChip::Icm42688p),
                mag_chip: None,
                baro_chip: Some(BaroChip::Bmp280),
                loop_rate_hz: 8000,
                gyro_filter_hz: 200,
                weight_g: 10.0,
            },
            gps: None,
        }
    }

    /// iFlight Chimera7 Pro V2 — 7.5" long-range on 6S Li-ion, ~1545 g all-up.
    ///
    /// Sources: iFlight Chimera7 Pro V2 6S Analog (705 g dry, 327 mm, deadcat
    /// geometry), XING2 2809 1250KV (60.7 g), BLITZ E55 4-in-1 (55 A constant /
    /// 65 A burst, 15.3 g), Fullsend 6S2P 8000 mAh Li-ion (840 g, EVE
    /// INR21700-40PL).
    ///
    /// **The pack is Li-ion, and its C-rating is the important correction.**
    /// The record said 50C; the current revision is printed 17.5C, and the
    /// history on iFlight's own page runs 10C -> 12.5C -> 17.5C as the cells
    /// changed. Current limiting makes that ceiling load-bearing: 8 Ah at 17.5C
    /// shared four ways is 35 A per motor, well under the ESC's 65 A, so the
    /// pack is what binds. Recording a Li-ion pack at LiPo C-ratings hands the
    /// build a current ceiling it does not have.
    ///
    /// The propeller pitch is genuinely ambiguous: iFlight ship HQ 7.5x3.7x3
    /// and 7.5x3x3 interchangeably ("will be shipped randomly", 2025-02-13).
    /// 3.7 is taken here; treat any pitch-sensitive result from this build as
    /// carrying that uncertainty.
    pub fn iflight_chimera_7() -> Self {
        Self {
            frame: FrameSpec {
                // 705 g published dry, less motors (242.8), props (32),
                // ESC (15.3), FC (12) and GPS (23).
                weight_g: 379.9,
                wheelbase_mm: 327.0,
                width_mm: Some(270.0),
                length_mm: Some(199.0),
                height_mm: Some(34.0),
                material: Some(FrameMaterial::Carbon),
            },
            motors: MotorSpec {
                kv: 1250.0, // XING2 2809
                weight_g: 60.7,
                stator_size: Some(StatorSize {
                    diameter_mm: 28.0,
                    height_mm: 9.0,
                }),
                no_load_amps: Some(0.8),
                resistance_ohm: Some(0.06546), // 65.46 mOhm interphase
            },
            escs: EscSpec {
                continuous_amps: 55.0,
                burst_amps: Some(65.0),
                min_cell_count: 2,
                max_cell_count: 6,
                firmware: Some(EscFirmware::BlHeli32),
                response_tau_ms: Some(3.0),
                weight_g: Some(15.3),
            },
            propellers: PropellerSpec {
                diameter_in: 7.5,
                pitch_in: 3.7,
                blade_count: 3,
                weight_g: Some(8.0),
            },
            battery: BatterySpec {
                cell_count: 6,
                capacity_mah: 8000.0,
                c_rating: 17.5, // Li-ion, printed
                weight_g: 840.0,
                internal_resistance_mohm: None,
            },
            flight_controller: FlightControllerSpec {
                mcu: Some("H7".into()),
                imu_chip: Some(ImuChip::Bmi270),
                mag_chip: Some(MagChip::Ist8310),
                baro_chip: Some(BaroChip::Bmp388),
                loop_rate_hz: 4000,
                gyro_filter_hz: 150,
                weight_g: 12.0,
            },
            gps: Some(GpsSpec {
                chipset: GpsChipset::UbloxM9N,
                update_rate_hz: 10.0,
                has_compass: true,
                weight_g: 23.0,
            }),
        }
    }
}

impl Default for BuildSpec {
    /// Canonical 5" race-quad reference build, sized to match the existing
    /// `PhysicsConfig::from_build_specs(1700.0, 5.0, 4.5, 3, 350.0, 33.0, 14.8)`
    /// invocation.
    fn default() -> Self {
        Self {
            frame: FrameSpec {
                weight_g: 350.0,
                wheelbase_mm: 220.0,
                width_mm: None,
                length_mm: None,
                height_mm: None,
                material: Some(FrameMaterial::Carbon),
            },
            motors: MotorSpec {
                kv: 1700.0,
                weight_g: 33.0,
                stator_size: Some(StatorSize {
                    diameter_mm: 23.0,
                    height_mm: 6.0,
                }),
                no_load_amps: None,
                resistance_ohm: None,
            },
            escs: EscSpec {
                continuous_amps: 45.0,
                burst_amps: Some(55.0),
                min_cell_count: 3,
                max_cell_count: 6,
                firmware: Some(EscFirmware::BlHeli32),
                response_tau_ms: None,
                weight_g: Some(12.0),
            },
            propellers: PropellerSpec {
                diameter_in: 5.0,
                pitch_in: 4.5,
                blade_count: 3,
                weight_g: Some(3.0),
            },
            battery: BatterySpec {
                cell_count: 4,
                capacity_mah: 1500.0,
                c_rating: 75.0,
                weight_g: 180.0,
                internal_resistance_mohm: None,
            },
            flight_controller: FlightControllerSpec {
                mcu: Some("F7".into()),
                imu_chip: Some(ImuChip::Mpu6000),
                mag_chip: None,
                baro_chip: Some(BaroChip::Bmp280),
                loop_rate_hz: 8000,
                gyro_filter_hz: 250,
                weight_g: 8.0,
            },
            gps: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Post-recalibration gate: `BuildSpec::default().to_physics_config()`
    /// preserves geometry / electrical / thermal fields, and produces kt/kq
    /// from the physical-CT prop lookup (deliberately ~3.5-4× lower than the
    /// legacy inflated `from_build_specs` values).
    #[test]
    fn default_build_spec_matches_from_build_specs_except_mass() {
        let via_build = BuildSpec::default().to_physics_config();
        let direct = PhysicsConfig::from_build_specs(1700.0, 5.0, 4.5, 3, 350.0, 33.0, 14.8);

        // Mass differs: to_physics_config uses total_mass_kg() (all components),
        // from_build_specs uses only frame + 4×motor
        assert!(
            via_build.mass_kg > direct.mass_kg,
            "BuildSpec mass should include battery, ESCs, props, FC"
        );

        // Geometry / inertia / drag inherit from the legacy path unchanged.
        assert_eq!(via_build.arm_length_m, direct.arm_length_m);
        assert_eq!(via_build.inertia, direct.inertia);
        assert_eq!(via_build.tau_motor, direct.tau_motor);
        assert_eq!(via_build.drag_coeffs, direct.drag_coeffs);
        assert_eq!(via_build.motor_kv, direct.motor_kv);
        assert_eq!(via_build.battery_voltage, direct.battery_voltage);
        assert_eq!(via_build.motor_kt_electrical, direct.motor_kt_electrical);

        // The lookup gives the measurement-anchored value; from_build_specs
        // retains the legacy formula, which over-produced thrust. The gap
        // narrowed at the 2026-08-29 re-anchor (CT_aero 0.109 -> 0.186), so the
        // band moved with it: 0.15-0.40 -> 0.26-0.69. Deliberate; see task 5.7.
        let kt_ratio = via_build.kt / direct.kt;
        assert!(
            kt_ratio > 0.26 && kt_ratio < 0.69,
            "kt de-inflation ratio {:.3} outside [0.15, 0.40] (got {:.4e}, legacy {:.4e})",
            kt_ratio,
            via_build.kt,
            direct.kt,
        );

        // kt from the lookup should be physically reasonable: CT_aero in the measured tri-blade range [0.15, 0.22]
        let d_m: f64 = 5.0 * 0.0254;
        let ct_aero = via_build.kt / (1.225 * d_m.powi(4)) * (2.0 * std::f64::consts::PI).powi(2);
        assert!(
            ct_aero > 0.15 && ct_aero < 0.22,
            "CT_aero {:.4} outside the measured tri-blade range [0.15, 0.22]",
            ct_aero,
        );
    }

    #[test]
    fn battery_spec_nominal_voltage() {
        let four_s = BatterySpec {
            cell_count: 4,
            capacity_mah: 1500.0,
            c_rating: 75.0,
            weight_g: 180.0,
            internal_resistance_mohm: None,
        };
        assert!((four_s.nominal_voltage() - 14.8).abs() < 1e-9);

        let six_s = BatterySpec {
            cell_count: 6,
            ..four_s
        };
        assert!((six_s.nominal_voltage() - 22.2).abs() < 1e-9);
    }

    #[test]
    fn lightweight_3inch_build_to_physics() {
        // Manually override default to a 3" build and check the physics is sane.
        let mut spec = BuildSpec::default();
        spec.frame.weight_g = 80.0;
        spec.frame.wheelbase_mm = 130.0;
        spec.motors.kv = 4500.0;
        spec.motors.weight_g = 9.0;
        spec.propellers.diameter_in = 3.0;
        spec.propellers.pitch_in = 2.5;
        spec.propellers.blade_count = 3;
        spec.battery.cell_count = 4;

        let physics = spec.to_physics_config();
        // Mass now uses total_mass_kg() which includes battery, ESCs, props, FC.
        // Default battery = 180g, ESCs = 12g, props = 4×3g = 12g, FC = 8g
        // Total = 80 + 36 + 180 + 12 + 12 + 8 = 328g = 0.328 kg
        let expected_mass = spec.total_mass_kg();
        assert!(
            (physics.mass_kg - expected_mass).abs() < 1e-6,
            "physics.mass_kg {} should match total_mass_kg {}",
            physics.mass_kg,
            expected_mass
        );
        // Hover throttle must remain achievable (< 80% of max omega)
        let hover_ratio = physics.hover_motor_speed() / physics.max_motor_speed_from_voltage();
        assert!(
            hover_ratio > 0.05 && hover_ratio < 0.8,
            "Hover/max ratio {} out of physical range",
            hover_ratio
        );
    }

    #[test]
    fn high_cell_count_propagates_to_battery_voltage() {
        let mut spec = BuildSpec::default();
        spec.battery.cell_count = 6;
        let physics = spec.to_physics_config();
        assert!((physics.battery_voltage - 22.2).abs() < 1e-9);
    }

    // -----------------------------------------------------------------
    // Phase 1: geometry helpers
    // -----------------------------------------------------------------

    /// Construct a representative 3" toothpick build (~210 g) for testing.
    fn toothpick_3inch() -> BuildSpec {
        let mut spec = BuildSpec::default();
        spec.frame.weight_g = 60.0;
        spec.frame.wheelbase_mm = 130.0;
        spec.motors.kv = 4500.0;
        spec.motors.weight_g = 14.0;
        spec.propellers.diameter_in = 3.0;
        spec.propellers.pitch_in = 2.5;
        spec.propellers.blade_count = 3;
        spec.propellers.weight_g = Some(1.5);
        spec.escs.weight_g = Some(8.0);
        spec.battery.cell_count = 4;
        spec.battery.capacity_mah = 450.0;
        spec.battery.weight_g = 80.0;
        spec.flight_controller.weight_g = 7.0;
        spec
    }

    /// Construct a representative 7" cinelifter build (~1250 g) for testing.
    fn cinelifter_7inch() -> BuildSpec {
        let mut spec = BuildSpec::default();
        spec.frame.weight_g = 250.0;
        spec.frame.wheelbase_mm = 295.0;
        spec.motors.kv = 1300.0;
        spec.motors.weight_g = 65.0;
        spec.propellers.diameter_in = 7.0;
        spec.propellers.pitch_in = 4.0;
        spec.propellers.blade_count = 3;
        spec.propellers.weight_g = Some(12.0);
        spec.escs.weight_g = Some(50.0);
        spec.battery.cell_count = 6;
        spec.battery.capacity_mah = 4500.0;
        spec.battery.weight_g = 600.0;
        spec.flight_controller.weight_g = 10.0;
        spec.gps = Some(GpsSpec {
            chipset: GpsChipset::UbloxM9N,
            update_rate_hz: 10.0,
            has_compass: true,
            weight_g: 30.0,
        });
        spec
    }

    #[test]
    fn total_mass_sums_all_components() {
        let m = BuildSpec::default().total_mass_kg();
        // frame 350 + 4×motor 132 + 4×prop 12 + esc 12 + bat 180 + fc 8 = 694 g
        assert!((m - 0.694).abs() < 1e-9, "got {} kg", m);
    }

    #[test]
    fn cinelifter_total_mass_includes_gps() {
        let m = cinelifter_7inch().total_mass_kg();
        // 250 + 260 + 48 + 50 + 600 + 10 + 30 = 1248 g
        assert!((m - 1.248).abs() < 1e-9, "got {} kg", m);
    }

    #[test]
    fn arm_length_is_half_wheelbase() {
        let spec = BuildSpec::default(); // 220 mm wheelbase
        assert!((spec.arm_length_m() - 0.110).abs() < 1e-9);
    }

    #[test]
    fn inertia_5inch_reference_is_physically_realistic() {
        let [ixx, iyy, izz] = BuildSpec::default().compute_inertia();
        // Real 5" race quads measured in flight logs: Ixx ≈ 0.0015-0.0035,
        // Izz ≈ 0.003-0.007. The point-mass+rod model lands on the low end;
        // CAD-driven per-frame inertia (future work) would tighten this.
        assert!(ixx > 0.0010 && ixx < 0.0050, "Ixx={} out of range", ixx);
        assert!(iyy > 0.0010 && iyy < 0.0050, "Iyy={} out of range", iyy);
        assert!(izz > 0.0020 && izz < 0.0100, "Izz={} out of range", izz);
        assert!(izz > ixx, "Izz must exceed Ixx");
        // Symmetric quad-X → Ixx == Iyy.
        assert!((ixx - iyy).abs() < 1e-12);
    }

    #[test]
    fn inertia_3inch_toothpick_is_lighter_than_5inch() {
        let [ixx_3, _, izz_3] = toothpick_3inch().compute_inertia();
        let [ixx_5, _, izz_5] = BuildSpec::default().compute_inertia();
        assert!(ixx_3 < ixx_5);
        assert!(izz_3 < izz_5);
        // Magnitude: should be small but nonzero.
        assert!(
            ixx_3 > 1e-5 && ixx_3 < 1e-3,
            "3\" Ixx={} unreasonable",
            ixx_3
        );
        assert!(izz_3 > ixx_3, "Izz must exceed Ixx for 3\" too");
    }

    #[test]
    fn inertia_7inch_cinelifter_is_heavier_than_5inch() {
        let [ixx_7, _, izz_7] = cinelifter_7inch().compute_inertia();
        let [ixx_5, _, izz_5] = BuildSpec::default().compute_inertia();
        assert!(ixx_7 > ixx_5);
        assert!(izz_7 > izz_5);
        // 7" 1.25kg builds: Ixx 0.003-0.020. Heavier builds (1.8kg with GoPro)
        // reach 0.015-0.030 — outside this test's reference build.
        assert!(
            ixx_7 > 0.003 && ixx_7 < 0.020,
            "7\" Ixx={} out of range",
            ixx_7
        );
        assert!(izz_7 > ixx_7);
    }

    #[test]
    fn inertia_no_longer_clamped_to_legacy_floor() {
        // Physical inertia for a 5" must be **below** the legacy 0.012 floor.
        let [ixx, _, izz] = BuildSpec::default().compute_inertia();
        let legacy = BuildSpec::default().to_physics_config();
        assert!(
            ixx < legacy.inertia[0],
            "physical Ixx {} should be below legacy {}",
            ixx,
            legacy.inertia[0]
        );
        // Izz may or may not be below the floor depending on geometry; just
        // confirm we're not anywhere near the legacy enforcement.
        assert!(izz > 0.0);
    }

    #[test]
    fn drag_coeffs_are_positive_and_vertical_largest() {
        let drag = BuildSpec::default().compute_drag_coeffs();
        assert!(drag.iter().all(|&c| c > 0.0));
        assert!(drag[2] > drag[0], "vertical drag should exceed lateral");
        // Symmetric quad → c_x == c_y when frame falls back to square plate.
        assert!((drag[0] - drag[1]).abs() < 1e-12);
    }

    #[test]
    fn drag_explicit_frame_dims_used_when_present() {
        let mut spec = BuildSpec::default();
        spec.frame.width_mm = Some(200.0);
        spec.frame.length_mm = Some(100.0);
        spec.frame.height_mm = Some(40.0);
        let drag = spec.compute_drag_coeffs();
        // Asymmetric width/length → c_x ≠ c_y.
        assert!((drag[0] - drag[1]).abs() > 1e-6);
        // Lateral X uses width × height = 0.008 m², larger than lateral Y's
        // length × height = 0.004 m². So c_x > c_y.
        assert!(drag[0] > drag[1]);
    }

    // -----------------------------------------------------------------
    // Phase 2: electrical chain
    // -----------------------------------------------------------------

    #[test]
    fn motor_no_load_amps_default_05() {
        let physics = BuildSpec::default().to_physics_config();
        assert!((physics.motor_no_load_amps - 0.5).abs() < 1e-9);
    }

    #[test]
    fn motor_no_load_amps_override_from_spec() {
        let mut spec = BuildSpec::default();
        spec.motors.no_load_amps = Some(1.2);
        let physics = spec.to_physics_config();
        assert!((physics.motor_no_load_amps - 1.2).abs() < 1e-9);
    }

    #[test]
    fn motor_resistance_override_from_spec() {
        let mut spec = BuildSpec::default();
        spec.motors.resistance_ohm = Some(0.090);
        let physics = spec.to_physics_config();
        assert!((physics.motor_resistance_ohm - 0.090).abs() < 1e-9);
    }

    #[test]
    fn esc_config_populated_from_spec() {
        let mut spec = BuildSpec::default();
        spec.escs.continuous_amps = 60.0;
        spec.escs.burst_amps = Some(80.0);
        spec.escs.response_tau_ms = Some(4.5);
        let physics = spec.to_physics_config();
        assert!((physics.esc.continuous_amps - 60.0).abs() < 1e-9);
        assert!((physics.esc.burst_amps - 80.0).abs() < 1e-9);
        assert!((physics.esc.response_tau_s - 0.0045).abs() < 1e-9);
    }

    #[test]
    fn esc_burst_defaults_to_120_percent_of_continuous() {
        let mut spec = BuildSpec::default();
        spec.escs.continuous_amps = 50.0;
        spec.escs.burst_amps = None;
        let physics = spec.to_physics_config();
        assert!((physics.esc.burst_amps - 60.0).abs() < 1e-9);
    }

    #[test]
    fn to_battery_config_uses_spec_c_rating() {
        let mut spec = BuildSpec::default();
        spec.battery.cell_count = 6;
        spec.battery.capacity_mah = 4500.0;
        spec.battery.c_rating = 100.0;
        let cfg = spec.to_battery_config();
        assert_eq!(cfg.cell_count, 6);
        assert!((cfg.capacity_mah - 4500.0).abs() < 1e-9);
        assert!((cfg.discharge_rate_c - 100.0).abs() < 1e-9);
        // R_int for 6S: deployed 3.2146 mohm/cell x 6 cells = 19.29 mohm
        // (calibration run 2026-11-02; previously 6 x 200/100 / 1000 = 0.012 ohm from the 200/C heuristic)
        let expected_r = 6.0 * 3.2146 / 1000.0;
        assert!(
            (cfg.internal_resistance_ohm - expected_r).abs() < 1e-6,
            "Expected R_int {expected_r:.6} Ω, got {:.6}",
            cfg.internal_resistance_ohm
        );
    }

    #[test]
    fn to_battery_config_internal_resistance_override() {
        let mut spec = BuildSpec::default();
        spec.battery.internal_resistance_mohm = Some(8.5);
        let cfg = spec.to_battery_config();
        assert!((cfg.internal_resistance_ohm - 0.0085).abs() < 1e-9);
    }

    #[test]
    fn battery_v_terminal_sags_under_load() {
        use crate::battery::BatteryState;
        let cfg = BuildSpec::default().to_battery_config();
        let state = BatteryState::fully_charged(&cfg);
        let v_ocv = state.voltage();
        // A 4S config built without an explicit resistance uses the run's 4S value:
        // 6.0839 mohm/cell x 4 = 24.34 mohm.
        let expected_r = 4.0 * 6.0839 / 1000.0;
        assert!(
            (cfg.internal_resistance_ohm - expected_r).abs() < 1e-9,
            "expected 4S R_int {expected_r:.6} ohm, got {:.6}",
            cfg.internal_resistance_ohm
        );
        // 80 A burst draw: sag = 80 A x 0.02434 ohm = 1.95 V
        let v_loaded = state.v_terminal(80.0, cfg.internal_resistance_ohm);
        assert!(v_loaded < v_ocv, "loaded voltage must be below OCV");
        let sag = v_ocv - v_loaded;
        // The fitted R yields about 2 V sag at 80 A for a 4S pack
        assert!(
            sag > 1.0 && sag < 5.0,
            "expected 1–5 V sag at 80 A, got {sag}"
        );
    }

    #[test]
    fn battery_v_terminal_clamps_to_zero() {
        use crate::battery::BatteryState;
        let cfg = BatteryConfig::new(4, 100.0, 5.0); // tiny low-C pack → high R_int
        let state = BatteryState::fully_charged(&cfg);
        // Absurd draw forces V_terminal negative without the clamp
        let v_loaded = state.v_terminal(10_000.0, cfg.internal_resistance_ohm);
        assert!(v_loaded >= 0.0);
    }

    // -----------------------------------------------------------------
    // Phase 3: FC-driven sensor profiles
    // -----------------------------------------------------------------

    #[test]
    fn icm42688p_gyro_is_not_quieter_than_mpu6000_in_the_run() {
        let mpu = ImuChip::Mpu6000.imu_profile();
        let icm = ImuChip::Icm42688p.imu_profile();
        // Median over vehicles in the calibration run: the gyro of the
        // ICM-42688-P is slightly noisier than the MPU-6000 gyro.
        assert_eq!(mpu.gyro_noise_density, 7.141e-5);
        assert_eq!(icm.gyro_noise_density, 7.997e-5);
        // The accel is still quieter.
        assert_eq!(mpu.accel_noise_density, 0.001504);
        assert_eq!(icm.accel_noise_density, 0.001368);
    }

    #[test]
    fn imu_other_holds_the_pooled_value() {
        // Other is the median over all vehicles, not a copy of the MPU-6000.
        let other = ImuChip::Other.imu_profile();
        assert_eq!(other.gyro_noise_density, 9.572e-5);
        assert_eq!(other.accel_noise_density, 0.001274);
        assert_ne!(
            other.gyro_noise_density,
            ImuChip::Mpu6000.imu_profile().gyro_noise_density
        );
    }

    #[test]
    fn imu_bias_drift_smaller_on_modern_chips() {
        let mpu = ImuChip::Mpu6000.imu_profile();
        let icm = ImuChip::Icm42688p.imu_profile();
        assert!(icm.gyro_bias_sigma < mpu.gyro_bias_sigma);
        assert!(icm.gyro_bias_tau > mpu.gyro_bias_tau);
    }

    #[test]
    fn rm3100_is_quieter_than_hmc5883_by_about_two_in_the_run() {
        let hmc = MagChip::Hmc5883.mag_profile().unwrap();
        let rm = MagChip::Rm3100.mag_profile().unwrap();
        assert_eq!(hmc.noise_sigma_gauss, 0.003405);
        assert_eq!(rm.noise_sigma_gauss, 0.001544);
    }

    #[test]
    fn mag_none_returns_no_profile() {
        assert!(MagChip::None.mag_profile().is_none());
    }

    #[test]
    fn spl06_has_the_lowest_noise_of_the_five_older_baros_in_the_run() {
        // DPS310 has no measured value and keeps its table value.
        assert_eq!(BaroChip::Dps310.baro_profile().noise_sigma_m, 0.05);
        assert_eq!(BaroChip::Spl06.baro_profile().noise_sigma_m, 0.02095);
        assert_eq!(BaroChip::Bmp388.baro_profile().noise_sigma_m, 0.1059);
        assert_eq!(BaroChip::Bmp280.baro_profile().noise_sigma_m, 0.16);
        assert_eq!(BaroChip::Ms5611.baro_profile().noise_sigma_m, 0.2276);
    }

    #[test]
    fn to_sensor_profiles_uses_fc_selections() {
        let mut spec = BuildSpec::default();
        spec.flight_controller.imu_chip = Some(ImuChip::Icm42688p);
        spec.flight_controller.mag_chip = Some(MagChip::Rm3100);
        spec.flight_controller.baro_chip = Some(BaroChip::Dps310);

        let profiles = spec.to_sensor_profiles();
        assert_eq!(
            profiles.imu.gyro_noise_density,
            ImuChip::Icm42688p.imu_profile().gyro_noise_density
        );
        assert_eq!(
            profiles.mag.unwrap().noise_sigma_gauss,
            MagChip::Rm3100.mag_profile().unwrap().noise_sigma_gauss
        );
        assert_eq!(profiles.baro.noise_sigma_m, 0.05);
    }

    #[test]
    fn to_sensor_profiles_handles_missing_chips() {
        let mut spec = BuildSpec::default();
        spec.flight_controller.imu_chip = None;
        spec.flight_controller.mag_chip = None;
        spec.flight_controller.baro_chip = None;
        let profiles = spec.to_sensor_profiles();
        // Falls back to "Other" variant
        assert_eq!(
            profiles.imu.gyro_noise_density,
            ImuChip::Other.imu_profile().gyro_noise_density
        );
        assert!(profiles.mag.is_none());
    }

    // -----------------------------------------------------------------
    // Phase 4: GPS chipset profiles
    // -----------------------------------------------------------------

    #[test]
    fn f9p_far_more_precise_than_m8n() {
        let m8 = GpsChipset::UbloxM8N.gps_profile();
        let f9 = GpsChipset::UbloxF9P.gps_profile();
        let ratio = m8.horizontal_noise_sigma_m / f9.horizontal_noise_sigma_m;
        // M8N: 1.198 m, F9P RTK: 0.0141 m: a ratio of about 85
        assert!(ratio > 50.0, "F9P expected ≥50× tighter, got {}×", ratio);
        assert!(f9.altitude_noise_sigma_m < m8.altitude_noise_sigma_m);
    }

    #[test]
    fn septentrio_mosaic_is_best_horizontal() {
        let candidates = [
            GpsChipset::UbloxM8N,
            GpsChipset::UbloxM9N,
            GpsChipset::UbloxM10,
            GpsChipset::UbloxF9P,
            GpsChipset::Here3,
        ];
        let mosaic = GpsChipset::SeptentrioMosaic
            .gps_profile()
            .horizontal_noise_sigma_m;
        for chip in &candidates {
            let n = chip.gps_profile().horizontal_noise_sigma_m;
            assert!(
                mosaic <= n,
                "Mosaic should beat {:?}: {} vs {}",
                chip,
                mosaic,
                n
            );
        }
    }

    #[test]
    fn m9n_has_higher_update_rate_than_m8n() {
        assert!(
            GpsChipset::UbloxM9N.gps_profile().update_rate_hz
                > GpsChipset::UbloxM8N.gps_profile().update_rate_hz
        );
    }

    #[test]
    fn here3_keeps_its_table_accuracy_and_higher_latency() {
        let here = GpsChipset::Here3.gps_profile();
        let m9 = GpsChipset::UbloxM9N.gps_profile();
        // Here3 has no measured value; the M9N now does, so they differ.
        assert_eq!(here.horizontal_noise_sigma_m, 0.9);
        assert_eq!(m9.horizontal_noise_sigma_m, 0.8705);
        // CAN bus: extra latency
        assert_eq!(here.delay_ms, 100.0);
        assert_eq!(m9.delay_ms, 80.0);
    }

    #[test]
    fn ttff_warm_under_5_seconds_for_consumer_chips() {
        for chip in [
            GpsChipset::UbloxM8N,
            GpsChipset::UbloxM9N,
            GpsChipset::UbloxM10,
            GpsChipset::UbloxF9P,
            GpsChipset::Here3,
        ] {
            assert!(
                chip.gps_profile().ttff_warm_s <= 5.0,
                "{:?} warm TTFF should be ≤5s",
                chip
            );
        }
    }

    #[test]
    fn gps_other_holds_the_pooled_value() {
        // Other is the median over all vehicles, not a copy of the M8N.
        let other = GpsChipset::Other.gps_profile();
        assert_eq!(other.horizontal_noise_sigma_m, 0.812);
        assert_eq!(other.altitude_noise_sigma_m, 1.25);
        assert_eq!(other.velocity_noise_sigma_mps, 0.1391);
        assert_eq!(other.update_rate_hz, 5.0);
        // Unmeasured fields still come from the M8N.
        assert_eq!(other.max_satellites, 22);
        assert_eq!(other.delay_ms, 120.0);
    }

    #[test]
    fn to_gps_profile_none_when_build_has_no_gps() {
        let mut spec = BuildSpec::default();
        spec.gps = None;
        assert!(spec.to_gps_profile().is_none());
    }

    #[test]
    fn to_gps_profile_matches_chipset() {
        let mut spec = BuildSpec::default();
        spec.gps = Some(GpsSpec {
            chipset: GpsChipset::UbloxF9P,
            update_rate_hz: 8.0,
            has_compass: false,
            weight_g: 20.0,
        });
        let profile = spec.to_gps_profile().unwrap();
        assert_eq!(
            profile.horizontal_noise_sigma_m,
            GpsChipset::UbloxF9P.gps_profile().horizontal_noise_sigma_m
        );
    }

    #[test]
    fn to_physics_config_physical_overrides_geometry() {
        let spec = BuildSpec::default();
        let safe = spec.to_physics_config();
        let physical = spec.to_physics_config_physical();

        // Both paths now use total_mass_kg(), so mass is the same
        assert_eq!(safe.mass_kg, physical.mass_kg);

        // Inertia differs: safe path uses legacy floor-enforced values,
        // physical path uses compute_inertia() (no floors)
        assert_ne!(safe.inertia, physical.inertia);
        // Physical inertia should be lower (no floors)
        assert!(
            physical.inertia[0] < safe.inertia[0],
            "physical Ixx {} should be below safe {}",
            physical.inertia[0],
            safe.inertia[0]
        );

        // Drag differs: safe uses legacy formula, physical uses compute_drag_coeffs()
        assert_ne!(safe.drag_coeffs, physical.drag_coeffs);

        // Arm length differs: safe uses prop-diameter hack, physical uses wheelbase/2
        assert_ne!(safe.arm_length_m, physical.arm_length_m);

        // Non-geometric fields preserved (these come from from_build_specs):
        assert_eq!(safe.kt, physical.kt);
        assert_eq!(safe.kq, physical.kq);
        assert_eq!(safe.tau_motor, physical.tau_motor);
        assert_eq!(safe.motor_kv, physical.motor_kv);
        assert_eq!(safe.battery_voltage, physical.battery_voltage);
        assert_eq!(safe.motor_kt_electrical, physical.motor_kt_electrical);
    }
}
