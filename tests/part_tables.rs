//! Part profiles measured from public PX4 flight logs (0.14.0).
//!
//! The literals below are the values of the calibration run's part-tables.json
//! rounded to four significant digits, as written into `src/build.rs`.

#![allow(clippy::excessive_precision)]

use hitl_physics::{BaroChip, GpsChipset, ImuChip, MagChip};

fn assert_good(name: &str, v: f64) {
    assert!(
        v.is_finite() && v > 0.0,
        "{name} must be finite and > 0, got {v}"
    );
}

/// The profile field equals the generated literal, and the literal is the
/// run's proposed value to within four-digit rounding (relative 5e-4).
fn check_row(row: &str, actual: f64, literal: f64, proposed: f64) {
    assert_eq!(
        actual, literal,
        "{row}: profile value differs from the generated literal"
    );
    let rel = ((literal - proposed) / proposed).abs();
    assert!(
        rel <= 5e-4,
        "{row}: literal {literal} is {rel:e} from proposed {proposed}"
    );
}

#[test]
fn every_profile_value_is_finite_and_positive() {
    for chip in [
        ImuChip::Mpu6000,
        ImuChip::Mpu6500,
        ImuChip::Icm20689,
        ImuChip::Icm42688p,
        ImuChip::Bmi270,
        ImuChip::Lsm6dso,
        ImuChip::Adis16470,
        ImuChip::Adis16607,
        ImuChip::Bmi055,
        ImuChip::Bmi088,
        ImuChip::Icm20602,
        ImuChip::Icm20608g,
        ImuChip::Icm20649,
        ImuChip::Icm20948,
        ImuChip::Icm42670p,
        ImuChip::Icm42686p,
        ImuChip::Icm45686,
        ImuChip::Iim42652,
        ImuChip::Iim42653,
        ImuChip::Mpu9250,
        ImuChip::Sch16t,
        ImuChip::Other,
    ] {
        let p = chip.imu_profile();
        let n = format!("{chip:?}");
        assert_good(&format!("{n} gyro_noise_density"), p.gyro_noise_density);
        assert_good(&format!("{n} accel_noise_density"), p.accel_noise_density);
        assert_good(&format!("{n} gyro_bias_sigma"), p.gyro_bias_sigma);
        assert_good(&format!("{n} gyro_bias_tau"), p.gyro_bias_tau);
    }
    for chip in [
        MagChip::Hmc5883,
        MagChip::Qmc5883,
        MagChip::Ist8310,
        MagChip::Rm3100,
        MagChip::Lis3mdl,
        MagChip::Ak09916,
        MagChip::Bmm150,
        MagChip::Bmm350,
        MagChip::Iis2mdc,
        MagChip::Qmc5883p,
        MagChip::None,
        MagChip::Other,
    ] {
        let n = format!("{chip:?}");
        match chip.mag_profile() {
            Some(p) => assert_good(&format!("{n} noise_sigma_gauss"), p.noise_sigma_gauss),
            None => assert_eq!(chip, MagChip::None, "only MagChip::None has no profile"),
        }
    }
    assert!(MagChip::None.mag_profile().is_none());
    for chip in [
        BaroChip::Bmp280,
        BaroChip::Bmp388,
        BaroChip::Dps310,
        BaroChip::Spl06,
        BaroChip::Ms5611,
        BaroChip::Bmp581,
        BaroChip::Icp201xx,
        BaroChip::Other,
    ] {
        let n = format!("{chip:?}");
        assert_good(
            &format!("{n} noise_sigma_m"),
            chip.baro_profile().noise_sigma_m,
        );
    }
    for chip in [
        GpsChipset::UbloxM8N,
        GpsChipset::UbloxM9N,
        GpsChipset::UbloxM10,
        GpsChipset::UbloxF9P,
        GpsChipset::SeptentrioMosaic,
        GpsChipset::Here3,
        GpsChipset::Ubx7,
        GpsChipset::Other,
    ] {
        let p = chip.gps_profile();
        let n = format!("{chip:?}");
        assert_good(
            &format!("{n} horizontal_noise_sigma_m"),
            p.horizontal_noise_sigma_m,
        );
        assert_good(
            &format!("{n} altitude_noise_sigma_m"),
            p.altitude_noise_sigma_m,
        );
        assert_good(
            &format!("{n} velocity_noise_sigma_mps"),
            p.velocity_noise_sigma_mps,
        );
        assert_good(&format!("{n} position_drift_sigma"), p.position_drift_sigma);
        assert_good(&format!("{n} position_drift_tau"), p.position_drift_tau);
        assert_good(&format!("{n} update_rate_hz"), p.update_rate_hz);
        assert_good(&format!("{n} delay_ms"), p.delay_ms);
        assert_good(&format!("{n} ttff_cold_s"), p.ttff_cold_s);
        assert_good(&format!("{n} ttff_warm_s"), p.ttff_warm_s);
        assert_good(&format!("{n} ttff_hot_s"), p.ttff_hot_s);
        assert_good(&format!("{n} max_satellites"), f64::from(p.max_satellites));
    }
}

#[test]
fn existing_variants_keep_unmeasured_fields() {
    let p = ImuChip::Mpu6000.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (1e-4, 100.0),
        "Mpu6000"
    );
    let p = ImuChip::Mpu6500.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (1e-4, 100.0),
        "Mpu6500"
    );
    let p = ImuChip::Icm20689.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (5e-5, 200.0),
        "Icm20689"
    );
    let p = ImuChip::Icm42688p.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (5e-5, 200.0),
        "Icm42688p"
    );
    let p = ImuChip::Bmi270.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (8e-5, 150.0),
        "Bmi270"
    );
    let p = ImuChip::Lsm6dso.imu_profile();
    assert_eq!(
        (p.gyro_bias_sigma, p.gyro_bias_tau),
        (6e-5, 180.0),
        "Lsm6dso"
    );
    let p = GpsChipset::UbloxM8N.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.06, 30.0, 120.0, 26.0, 5.0, 1.0, 22),
        "UbloxM8N"
    );
    let p = GpsChipset::UbloxM9N.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.04, 40.0, 80.0, 24.0, 2.0, 1.0, 33),
        "UbloxM9N"
    );
    let p = GpsChipset::UbloxM10.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.04, 40.0, 90.0, 24.0, 2.0, 1.0, 32),
        "UbloxM10"
    );
    let p = GpsChipset::UbloxF9P.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.005, 60.0, 60.0, 25.0, 2.0, 1.0, 60),
        "UbloxF9P"
    );
    let p = GpsChipset::SeptentrioMosaic.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.003, 60.0, 40.0, 45.0, 20.0, 5.0, 80),
        "SeptentrioMosaic"
    );
    let p = GpsChipset::Here3.gps_profile();
    assert_eq!(
        (
            p.position_drift_sigma,
            p.position_drift_tau,
            p.delay_ms,
            p.ttff_cold_s,
            p.ttff_warm_s,
            p.ttff_hot_s,
            p.max_satellites
        ),
        (0.04, 40.0, 100.0, 24.0, 2.0, 1.0, 33),
        "Here3"
    );
}

#[test]
fn run_values_are_used() {
    check_row(
        "imu Mpu6000 gyro_noise_density",
        ImuChip::Mpu6000.imu_profile().gyro_noise_density,
        7.141e-5,
        7.140908091604083e-05,
    );
    check_row(
        "imu Mpu6000 accel_noise_density",
        ImuChip::Mpu6000.imu_profile().accel_noise_density,
        0.001504,
        0.0015043799654325812,
    );
    check_row(
        "imu Icm20689 gyro_noise_density",
        ImuChip::Icm20689.imu_profile().gyro_noise_density,
        9.295e-5,
        9.295497128675086e-05,
    );
    check_row(
        "imu Icm20689 accel_noise_density",
        ImuChip::Icm20689.imu_profile().accel_noise_density,
        0.001502,
        0.001501713764715635,
    );
    check_row(
        "imu Icm42688p gyro_noise_density",
        ImuChip::Icm42688p.imu_profile().gyro_noise_density,
        7.997e-5,
        7.997254608679627e-05,
    );
    check_row(
        "imu Icm42688p accel_noise_density",
        ImuChip::Icm42688p.imu_profile().accel_noise_density,
        0.001368,
        0.001367802646982865,
    );
    check_row(
        "imu Bmi270 gyro_noise_density",
        ImuChip::Bmi270.imu_profile().gyro_noise_density,
        0.0001092,
        0.00010924056733493293,
    );
    check_row(
        "imu Bmi270 accel_noise_density",
        ImuChip::Bmi270.imu_profile().accel_noise_density,
        0.001326,
        0.0013263532066443986,
    );
    check_row(
        "imu Adis16470 gyro_noise_density",
        ImuChip::Adis16470.imu_profile().gyro_noise_density,
        7.152e-5,
        7.152366645376546e-05,
    );
    check_row(
        "imu Adis16470 accel_noise_density",
        ImuChip::Adis16470.imu_profile().accel_noise_density,
        0.001139,
        0.001138673462483743,
    );
    check_row(
        "imu Adis16607 gyro_noise_density",
        ImuChip::Adis16607.imu_profile().gyro_noise_density,
        0.0001231,
        0.0001230780980468071,
    );
    check_row(
        "imu Adis16607 accel_noise_density",
        ImuChip::Adis16607.imu_profile().accel_noise_density,
        0.003539,
        0.00353900498432258,
    );
    check_row(
        "imu Bmi055 gyro_noise_density",
        ImuChip::Bmi055.imu_profile().gyro_noise_density,
        0.0002762,
        0.0002761874472646714,
    );
    check_row(
        "imu Bmi055 accel_noise_density",
        ImuChip::Bmi055.imu_profile().accel_noise_density,
        0.001395,
        0.0013947928498405527,
    );
    check_row(
        "imu Bmi088 gyro_noise_density",
        ImuChip::Bmi088.imu_profile().gyro_noise_density,
        0.0002713,
        0.0002713115367834173,
    );
    check_row(
        "imu Bmi088 accel_noise_density",
        ImuChip::Bmi088.imu_profile().accel_noise_density,
        0.001682,
        0.0016815679405819356,
    );
    check_row(
        "imu Icm20602 gyro_noise_density",
        ImuChip::Icm20602.imu_profile().gyro_noise_density,
        6.602e-5,
        6.602211108661011e-05,
    );
    check_row(
        "imu Icm20602 accel_noise_density",
        ImuChip::Icm20602.imu_profile().accel_noise_density,
        0.0007635,
        0.0007635432931729475,
    );
    check_row(
        "imu Icm20608g gyro_noise_density",
        ImuChip::Icm20608g.imu_profile().gyro_noise_density,
        9.93e-5,
        9.93021342745974e-05,
    );
    check_row(
        "imu Icm20608g accel_noise_density",
        ImuChip::Icm20608g.imu_profile().accel_noise_density,
        0.002125,
        0.0021247957940980287,
    );
    check_row(
        "imu Icm20649 gyro_noise_density",
        ImuChip::Icm20649.imu_profile().gyro_noise_density,
        0.000185,
        0.00018503267461727818,
    );
    check_row(
        "imu Icm20649 accel_noise_density",
        ImuChip::Icm20649.imu_profile().accel_noise_density,
        0.001699,
        0.0016986429287514002,
    );
    check_row(
        "imu Icm20948 gyro_noise_density",
        ImuChip::Icm20948.imu_profile().gyro_noise_density,
        0.0001125,
        0.00011251739509580042,
    );
    check_row(
        "imu Icm20948 accel_noise_density",
        ImuChip::Icm20948.imu_profile().accel_noise_density,
        0.001322,
        0.001322021579562853,
    );
    check_row(
        "imu Icm42670p gyro_noise_density",
        ImuChip::Icm42670p.imu_profile().gyro_noise_density,
        5.957e-5,
        5.956739319686754e-05,
    );
    check_row(
        "imu Icm42670p accel_noise_density",
        ImuChip::Icm42670p.imu_profile().accel_noise_density,
        0.0006262,
        0.0006262417636586085,
    );
    check_row(
        "imu Icm42686p gyro_noise_density",
        ImuChip::Icm42686p.imu_profile().gyro_noise_density,
        7.024e-5,
        7.024018564138463e-05,
    );
    check_row(
        "imu Icm42686p accel_noise_density",
        ImuChip::Icm42686p.imu_profile().accel_noise_density,
        0.001483,
        0.0014830849145556553,
    );
    check_row(
        "imu Icm45686 gyro_noise_density",
        ImuChip::Icm45686.imu_profile().gyro_noise_density,
        5.727e-5,
        5.727239436600404e-05,
    );
    check_row(
        "imu Icm45686 accel_noise_density",
        ImuChip::Icm45686.imu_profile().accel_noise_density,
        0.001229,
        0.0012290112195895604,
    );
    check_row(
        "imu Iim42652 gyro_noise_density",
        ImuChip::Iim42652.imu_profile().gyro_noise_density,
        0.0001204,
        0.00012040130368280728,
    );
    check_row(
        "imu Iim42652 accel_noise_density",
        ImuChip::Iim42652.imu_profile().accel_noise_density,
        0.001308,
        0.0013079570081427087,
    );
    check_row(
        "imu Iim42653 gyro_noise_density",
        ImuChip::Iim42653.imu_profile().gyro_noise_density,
        0.0001228,
        0.0001227525996332161,
    );
    check_row(
        "imu Iim42653 accel_noise_density",
        ImuChip::Iim42653.imu_profile().accel_noise_density,
        0.002386,
        0.0023863500879107258,
    );
    check_row(
        "imu Mpu9250 gyro_noise_density",
        ImuChip::Mpu9250.imu_profile().gyro_noise_density,
        9.769e-5,
        9.768805923935437e-05,
    );
    check_row(
        "imu Mpu9250 accel_noise_density",
        ImuChip::Mpu9250.imu_profile().accel_noise_density,
        0.0009651,
        0.000965118235554749,
    );
    check_row(
        "imu Sch16t gyro_noise_density",
        ImuChip::Sch16t.imu_profile().gyro_noise_density,
        2.835e-5,
        2.834833544461242e-05,
    );
    check_row(
        "imu Sch16t accel_noise_density",
        ImuChip::Sch16t.imu_profile().accel_noise_density,
        0.0007964,
        0.0007963638507268023,
    );
    check_row(
        "imu Other gyro_noise_density",
        ImuChip::Other.imu_profile().gyro_noise_density,
        9.572e-5,
        9.572112803112154e-05,
    );
    check_row(
        "imu Other accel_noise_density",
        ImuChip::Other.imu_profile().accel_noise_density,
        0.001274,
        0.0012736021115917536,
    );
    check_row(
        "baro Bmp388 noise_sigma_m",
        BaroChip::Bmp388.baro_profile().noise_sigma_m,
        0.1059,
        0.10592946548479132,
    );
    check_row(
        "baro Spl06 noise_sigma_m",
        BaroChip::Spl06.baro_profile().noise_sigma_m,
        0.02095,
        0.02094525624205359,
    );
    check_row(
        "baro Ms5611 noise_sigma_m",
        BaroChip::Ms5611.baro_profile().noise_sigma_m,
        0.2276,
        0.22760992271955854,
    );
    check_row(
        "baro Bmp581 noise_sigma_m",
        BaroChip::Bmp581.baro_profile().noise_sigma_m,
        0.02184,
        0.02183690569155209,
    );
    check_row(
        "baro Icp201xx noise_sigma_m",
        BaroChip::Icp201xx.baro_profile().noise_sigma_m,
        0.1903,
        0.19029414154570407,
    );
    check_row(
        "baro Other noise_sigma_m",
        BaroChip::Other.baro_profile().noise_sigma_m,
        0.2198,
        0.2198055954595785,
    );
    check_row(
        "mag Hmc5883 noise_sigma_gauss",
        MagChip::Hmc5883.mag_profile().unwrap().noise_sigma_gauss,
        0.003405,
        0.0034054306832522717,
    );
    check_row(
        "mag Qmc5883 noise_sigma_gauss",
        MagChip::Qmc5883.mag_profile().unwrap().noise_sigma_gauss,
        0.003216,
        0.0032155177195758258,
    );
    check_row(
        "mag Ist8310 noise_sigma_gauss",
        MagChip::Ist8310.mag_profile().unwrap().noise_sigma_gauss,
        0.002705,
        0.002704991452152781,
    );
    check_row(
        "mag Rm3100 noise_sigma_gauss",
        MagChip::Rm3100.mag_profile().unwrap().noise_sigma_gauss,
        0.001544,
        0.0015442013427500554,
    );
    check_row(
        "mag Lis3mdl noise_sigma_gauss",
        MagChip::Lis3mdl.mag_profile().unwrap().noise_sigma_gauss,
        0.008952,
        0.008952435600846489,
    );
    check_row(
        "mag Ak09916 noise_sigma_gauss",
        MagChip::Ak09916.mag_profile().unwrap().noise_sigma_gauss,
        0.009422,
        0.009421596096709572,
    );
    check_row(
        "mag Bmm150 noise_sigma_gauss",
        MagChip::Bmm150.mag_profile().unwrap().noise_sigma_gauss,
        0.003614,
        0.0036142667440546505,
    );
    check_row(
        "mag Bmm350 noise_sigma_gauss",
        MagChip::Bmm350.mag_profile().unwrap().noise_sigma_gauss,
        0.002767,
        0.0027665101642512235,
    );
    check_row(
        "mag Iis2mdc noise_sigma_gauss",
        MagChip::Iis2mdc.mag_profile().unwrap().noise_sigma_gauss,
        0.003449,
        0.0034494980603023704,
    );
    check_row(
        "mag Qmc5883p noise_sigma_gauss",
        MagChip::Qmc5883p.mag_profile().unwrap().noise_sigma_gauss,
        0.01207,
        0.012070296873476173,
    );
    check_row(
        "mag Other noise_sigma_gauss",
        MagChip::Other.mag_profile().unwrap().noise_sigma_gauss,
        0.003485,
        0.003484920524468419,
    );
    check_row(
        "gps UbloxM8N horizontal_noise_sigma_m",
        GpsChipset::UbloxM8N.gps_profile().horizontal_noise_sigma_m,
        1.198,
        1.1978750377893448,
    );
    check_row(
        "gps UbloxM8N altitude_noise_sigma_m",
        GpsChipset::UbloxM8N.gps_profile().altitude_noise_sigma_m,
        1.87,
        1.8700000643730164,
    );
    check_row(
        "gps UbloxM8N velocity_noise_sigma_mps",
        GpsChipset::UbloxM8N.gps_profile().velocity_noise_sigma_mps,
        0.268,
        0.2680000066757202,
    );
    check_row(
        "gps UbloxM8N update_rate_hz",
        GpsChipset::UbloxM8N.gps_profile().update_rate_hz,
        5.0,
        4.999987500060506,
    );
    check_row(
        "gps UbloxM9N horizontal_noise_sigma_m",
        GpsChipset::UbloxM9N.gps_profile().horizontal_noise_sigma_m,
        0.8705,
        0.8705000579357147,
    );
    check_row(
        "gps UbloxM9N altitude_noise_sigma_m",
        GpsChipset::UbloxM9N.gps_profile().altitude_noise_sigma_m,
        1.43,
        1.4302500784397125,
    );
    check_row(
        "gps UbloxM9N velocity_noise_sigma_mps",
        GpsChipset::UbloxM9N.gps_profile().velocity_noise_sigma_mps,
        0.2344,
        0.2343750149011612,
    );
    check_row(
        "gps UbloxM9N update_rate_hz",
        GpsChipset::UbloxM9N.gps_profile().update_rate_hz,
        8.001,
        8.000656053825399,
    );
    check_row(
        "gps UbloxM10 horizontal_noise_sigma_m",
        GpsChipset::UbloxM10.gps_profile().horizontal_noise_sigma_m,
        2.074,
        2.0739734172821045,
    );
    check_row(
        "gps UbloxM10 altitude_noise_sigma_m",
        GpsChipset::UbloxM10.gps_profile().altitude_noise_sigma_m,
        2.609,
        2.6091760993003845,
    );
    check_row(
        "gps UbloxM10 velocity_noise_sigma_mps",
        GpsChipset::UbloxM10.gps_profile().velocity_noise_sigma_mps,
        0.328,
        0.328000009059906,
    );
    check_row(
        "gps UbloxF9P horizontal_noise_sigma_m",
        GpsChipset::UbloxF9P.gps_profile().horizontal_noise_sigma_m,
        0.0141,
        0.014099999330937862,
    );
    check_row(
        "gps UbloxF9P altitude_noise_sigma_m",
        GpsChipset::UbloxF9P.gps_profile().altitude_noise_sigma_m,
        0.01,
        0.010000000707805157,
    );
    check_row(
        "gps UbloxF9P velocity_noise_sigma_mps",
        GpsChipset::UbloxF9P.gps_profile().velocity_noise_sigma_mps,
        0.094,
        0.09400000423192978,
    );
    check_row(
        "gps UbloxF9P update_rate_hz",
        GpsChipset::UbloxF9P.gps_profile().update_rate_hz,
        5.051,
        5.0509132050982855,
    );
    check_row(
        "gps SeptentrioMosaic horizontal_noise_sigma_m",
        GpsChipset::SeptentrioMosaic
            .gps_profile()
            .horizontal_noise_sigma_m,
        0.01,
        0.009999999776482582,
    );
    check_row(
        "gps SeptentrioMosaic altitude_noise_sigma_m",
        GpsChipset::SeptentrioMosaic
            .gps_profile()
            .altitude_noise_sigma_m,
        0.02,
        0.019999999552965164,
    );
    check_row(
        "gps SeptentrioMosaic velocity_noise_sigma_mps",
        GpsChipset::SeptentrioMosaic
            .gps_profile()
            .velocity_noise_sigma_mps,
        0.01457,
        0.014573520049452782,
    );
    check_row(
        "gps Ubx7 horizontal_noise_sigma_m",
        GpsChipset::Ubx7.gps_profile().horizontal_noise_sigma_m,
        2.042,
        2.042375072836876,
    );
    check_row(
        "gps Ubx7 altitude_noise_sigma_m",
        GpsChipset::Ubx7.gps_profile().altitude_noise_sigma_m,
        3.373,
        3.3725001215934753,
    );
    check_row(
        "gps Ubx7 velocity_noise_sigma_mps",
        GpsChipset::Ubx7.gps_profile().velocity_noise_sigma_mps,
        0.4266,
        0.4266250282526016,
    );
    check_row(
        "gps Other horizontal_noise_sigma_m",
        GpsChipset::Other.gps_profile().horizontal_noise_sigma_m,
        0.812,
        0.812000036239624,
    );
    check_row(
        "gps Other altitude_noise_sigma_m",
        GpsChipset::Other.gps_profile().altitude_noise_sigma_m,
        1.25,
        1.2496250718832016,
    );
    check_row(
        "gps Other velocity_noise_sigma_mps",
        GpsChipset::Other.gps_profile().velocity_noise_sigma_mps,
        0.1391,
        0.13912500999867916,
    );
    check_row(
        "gps Other update_rate_hz",
        GpsChipset::Other.gps_profile().update_rate_hz,
        5.0,
        5.000100003890855,
    );
}

#[test]
fn new_variants_exist() {
    let _ = ImuChip::Adis16470.imu_profile();
    let _ = ImuChip::Adis16607.imu_profile();
    let _ = ImuChip::Bmi055.imu_profile();
    let _ = ImuChip::Bmi088.imu_profile();
    let _ = ImuChip::Icm20602.imu_profile();
    let _ = ImuChip::Icm20608g.imu_profile();
    let _ = ImuChip::Icm20649.imu_profile();
    let _ = ImuChip::Icm20948.imu_profile();
    let _ = ImuChip::Icm42670p.imu_profile();
    let _ = ImuChip::Icm42686p.imu_profile();
    let _ = ImuChip::Icm45686.imu_profile();
    let _ = ImuChip::Iim42652.imu_profile();
    let _ = ImuChip::Iim42653.imu_profile();
    let _ = ImuChip::Mpu9250.imu_profile();
    let _ = ImuChip::Sch16t.imu_profile();
    let _ = BaroChip::Bmp581.baro_profile();
    let _ = BaroChip::Icp201xx.baro_profile();
    let _ = MagChip::Ak09916.mag_profile();
    let _ = MagChip::Bmm150.mag_profile();
    let _ = MagChip::Bmm350.mag_profile();
    let _ = MagChip::Iis2mdc.mag_profile();
    let _ = MagChip::Qmc5883p.mag_profile();
    let _ = GpsChipset::Ubx7.gps_profile();
}

/// Existing entries and fields the run proposed no value for (part-tables.json
/// rows with a null `proposed`) keep their hitl-physics 0.13.1 literals. They
/// are separate from `existing_variants_keep_unmeasured_fields`, which pins the
/// fields the run has no row for (bias, drift, delay, time to first fix,
/// satellites); this test pins the noise and rate fields that have a row but no
/// proposal. Several of them now sit far from their measured siblings (for
/// example the MPU-6500 gyro 9.4e-4 against 7.141e-5 for the MPU-6000), which
/// is the specification's rule, not an oversight.
#[test]
fn unproposed_entries_keep_their_0_13_1_values() {
    // Mpu6500: 9 vehicles, below the 10-vehicle threshold.
    let p = ImuChip::Mpu6500.imu_profile();
    assert_eq!(p.gyro_noise_density, 9.4e-4, "Mpu6500 gyro_noise_density");
    assert_eq!(
        p.accel_noise_density, 6.86e-3,
        "Mpu6500 accel_noise_density"
    );
    // Lsm6dso: no eligible log.
    let p = ImuChip::Lsm6dso.imu_profile();
    assert_eq!(p.gyro_noise_density, 6.11e-5, "Lsm6dso gyro_noise_density");
    assert_eq!(
        p.accel_noise_density, 5.88e-4,
        "Lsm6dso accel_noise_density"
    );
    // Bmp280: too few logs pass the white-noise check. Dps310: likewise.
    assert_eq!(
        BaroChip::Bmp280.baro_profile().noise_sigma_m,
        0.16,
        "Bmp280 noise_sigma_m"
    );
    assert_eq!(
        BaroChip::Dps310.baro_profile().noise_sigma_m,
        0.05,
        "Dps310 noise_sigma_m"
    );
    // UbloxM10 update rate: 6 vehicles, below the threshold. Septentrio: no
    // eligible log.
    assert_eq!(
        GpsChipset::UbloxM10.gps_profile().update_rate_hz,
        10.0,
        "UbloxM10 update_rate_hz"
    );
    assert_eq!(
        GpsChipset::SeptentrioMosaic.gps_profile().update_rate_hz,
        100.0,
        "SeptentrioMosaic update_rate_hz"
    );
    // Here3: no eligible log for any of its four measured fields.
    let p = GpsChipset::Here3.gps_profile();
    assert_eq!(p.horizontal_noise_sigma_m, 0.9, "Here3 horizontal");
    assert_eq!(p.altitude_noise_sigma_m, 1.5, "Here3 altitude");
    assert_eq!(p.velocity_noise_sigma_mps, 0.05, "Here3 velocity");
    assert_eq!(p.update_rate_hz, 8.0, "Here3 update_rate_hz");
}

/// A new variant takes the same type's `Other` value for every field the run
/// proposed no value for it. The IMU variants are measured for the two noise
/// densities only, so the bias sigma and time constant come from `Other`; the
/// new barometer and magnetometer variants are measured for their one field, so
/// nothing is inherited there. `Ubx7` is measured for the three GPS sigmas and
/// inherits the update rate and every other field.
#[test]
fn new_variants_take_other_for_unmeasured_fields() {
    let other = ImuChip::Other.imu_profile();
    for chip in [
        ImuChip::Adis16470,
        ImuChip::Adis16607,
        ImuChip::Bmi055,
        ImuChip::Bmi088,
        ImuChip::Icm20602,
        ImuChip::Icm20608g,
        ImuChip::Icm20649,
        ImuChip::Icm20948,
        ImuChip::Icm42670p,
        ImuChip::Icm42686p,
        ImuChip::Icm45686,
        ImuChip::Iim42652,
        ImuChip::Iim42653,
        ImuChip::Mpu9250,
        ImuChip::Sch16t,
    ] {
        let p = chip.imu_profile();
        let n = format!("{chip:?}");
        assert_eq!(p.gyro_bias_sigma, other.gyro_bias_sigma, "{n} bias sigma");
        assert_eq!(p.gyro_bias_tau, other.gyro_bias_tau, "{n} bias tau");
    }
    let other = GpsChipset::Other.gps_profile();
    let p = GpsChipset::Ubx7.gps_profile();
    assert_eq!(
        p.update_rate_hz, other.update_rate_hz,
        "Ubx7 update_rate_hz"
    );
    assert_eq!(
        p.position_drift_sigma, other.position_drift_sigma,
        "Ubx7 position_drift_sigma"
    );
    assert_eq!(
        p.position_drift_tau, other.position_drift_tau,
        "Ubx7 position_drift_tau"
    );
    assert_eq!(p.delay_ms, other.delay_ms, "Ubx7 delay_ms");
    assert_eq!(p.ttff_cold_s, other.ttff_cold_s, "Ubx7 ttff_cold_s");
    assert_eq!(p.ttff_warm_s, other.ttff_warm_s, "Ubx7 ttff_warm_s");
    assert_eq!(p.ttff_hot_s, other.ttff_hot_s, "Ubx7 ttff_hot_s");
    assert_eq!(
        p.max_satellites, other.max_satellites,
        "Ubx7 max_satellites"
    );
}
