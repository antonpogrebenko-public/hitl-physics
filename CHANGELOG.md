# Changelog

## 0.14.0 - 2026-11-03

Sensor noise values and battery constants now come from a calibration run dated 2026-11-02 on public PX4 flight logs (the run and its tooling are in the th3seus-tools repository). Only sensor noise values, GPS update rates (for example UbloxM9N 10.0 to 8.001 Hz and UbloxF9P 8.0 to 5.051 Hz) and battery values change; the rigid-body, motor, propeller and drag models are unchanged. Every changed value is listed below, as it ships (the generated Rust rounds each measured value to 4 significant digits).

### Sensor values that changed

Existing variants and `Other`. "Old" is the 0.13.1 value; "Vehicles" is the number of vehicles behind the new median. GPS rows carry the fix class of the measurement. Two GPS `update_rate_hz` values were re-measured and stay at 5.0 (`UbloxM8N`, `Other`); they are not listed. Fields not listed (bias sigma and tau, drift, time to first fix, delays, and the values with no proposal) are unchanged. Units are those of the field names: IMU noise densities in rad/s/sqrt(Hz) (gyro) and m/s^2/sqrt(Hz) (accel), magnetometer noise in gauss, barometer noise in m, GPS noise in m and m/s.

| Part | Variant | Field | Old (0.13.1) | New | Vehicles |
|---|---|---|---|---|---|
| imu | Mpu6000 | `gyro_noise_density` | 8.726646e-4 | 7.141e-5 | 70 |
| imu | Mpu6000 | `accel_noise_density` | 6.37e-3 | 0.001504 | 70 |
| imu | Icm20689 | `gyro_noise_density` | 8.73e-5 | 9.295e-5 | 149 |
| imu | Icm20689 | `accel_noise_density` | 7.85e-4 | 0.001502 | 149 |
| imu | Icm42688p | `gyro_noise_density` | 4.88e-5 | 7.997e-5 | 373 |
| imu | Icm42688p | `accel_noise_density` | 6.86e-4 | 0.001368 | 373 |
| imu | Bmi270 | `gyro_noise_density` | 1.40e-4 | 0.0001092 | 33 |
| imu | Bmi270 | `accel_noise_density` | 2.45e-3 | 0.001326 | 33 |
| imu | Other | `gyro_noise_density` | 8.726646e-4 | 9.572e-5 | 2434 |
| imu | Other | `accel_noise_density` | 6.37e-3 | 0.001274 | 2434 |
| baro | Bmp388 | `noise_sigma_m` | 0.08 | 0.1059 | 78 |
| baro | Spl06 | `noise_sigma_m` | 0.10 | 0.02095 | 18 |
| baro | Ms5611 | `noise_sigma_m` | 0.10 | 0.2276 | 646 |
| baro | Other | `noise_sigma_m` | 0.15 | 0.2198 | 915 |
| mag | Hmc5883 | `noise_sigma_gauss` | 0.0006 | 0.003405 | 15 |
| mag | Qmc5883 | `noise_sigma_gauss` | 0.0005 | 0.003216 | 98 |
| mag | Ist8310 | `noise_sigma_gauss` | 0.0003 | 0.002705 | 410 |
| mag | Rm3100 | `noise_sigma_gauss` | 0.000013 | 0.001544 | 104 |
| mag | Lis3mdl | `noise_sigma_gauss` | 0.00032 | 0.008952 | 26 |
| mag | Other | `noise_sigma_gauss` | 0.0006 | 0.003485 | 1122 |
| gps | UbloxM8N (standalone_3d) | `horizontal_noise_sigma_m` | 1.5 | 1.198 | 302 |
| gps | UbloxM8N (standalone_3d) | `altitude_noise_sigma_m` | 3.0 | 1.87 | 302 |
| gps | UbloxM8N (standalone_3d) | `velocity_noise_sigma_mps` | 0.10 | 0.268 | 302 |
| gps | UbloxM9N (standalone_3d) | `horizontal_noise_sigma_m` | 0.9 | 0.8705 | 1100 |
| gps | UbloxM9N (standalone_3d) | `altitude_noise_sigma_m` | 1.5 | 1.43 | 1100 |
| gps | UbloxM9N (standalone_3d) | `velocity_noise_sigma_mps` | 0.05 | 0.2344 | 1100 |
| gps | UbloxM9N (standalone_3d) | `update_rate_hz` | 10.0 | 8.001 | 78 |
| gps | UbloxM10 (standalone_3d) | `horizontal_noise_sigma_m` | 1.0 | 2.074 | 47 |
| gps | UbloxM10 (standalone_3d) | `altitude_noise_sigma_m` | 1.8 | 2.609 | 47 |
| gps | UbloxM10 (standalone_3d) | `velocity_noise_sigma_mps` | 0.06 | 0.328 | 37 |
| gps | UbloxF9P (rtk_fixed) | `horizontal_noise_sigma_m` | 0.014 | 0.0141 | 359 |
| gps | UbloxF9P (rtk_fixed) | `altitude_noise_sigma_m` | 0.02 | 0.01 | 359 |
| gps | UbloxF9P (rtk_fixed) | `velocity_noise_sigma_mps` | 0.015 | 0.094 | 359 |
| gps | UbloxF9P (rtk_fixed) | `update_rate_hz` | 8.0 | 5.051 | 33 |
| gps | SeptentrioMosaic (rtk_fixed) | `horizontal_noise_sigma_m` | 0.006 | 0.01 | 11 |
| gps | SeptentrioMosaic (rtk_fixed) | `altitude_noise_sigma_m` | 0.012 | 0.02 | 11 |
| gps | SeptentrioMosaic (rtk_fixed) | `velocity_noise_sigma_mps` | 0.010 | 0.01457 | 11 |
| gps | Other (standalone_3d) | `horizontal_noise_sigma_m` | 1.5 | 0.812 | 5855 |
| gps | Other (standalone_3d) | `altitude_noise_sigma_m` | 3.0 | 1.25 | 5854 |
| gps | Other (standalone_3d) | `velocity_noise_sigma_mps` | 0.10 | 0.1391 | 5748 |

The `Other` profiles for IMU, magnetometer, barometer and GPS now hold the pooled medians above. They no longer equal `Mpu6000` (IMU) or `UbloxM8N` (GPS), which `Other` followed in 0.13.1.

### New variants

23 variants are new. Only the measured fields are listed; their other fields (bias sigma and tau, drift, delays and so on) are the `Other` values of 0.13.1.

| Part | Variant | Field | Value | Vehicles |
|---|---|---|---|---|
| imu | Adis16470 | `gyro_noise_density` | 7.152e-5 | 17 |
| imu | Adis16470 | `accel_noise_density` | 0.001139 | 17 |
| imu | Adis16607 | `gyro_noise_density` | 0.0001231 | 10 |
| imu | Adis16607 | `accel_noise_density` | 0.003539 | 10 |
| imu | Bmi055 | `gyro_noise_density` | 0.0002762 | 131 |
| imu | Bmi055 | `accel_noise_density` | 0.001395 | 131 |
| imu | Bmi088 | `gyro_noise_density` | 0.0002713 | 392 |
| imu | Bmi088 | `accel_noise_density` | 0.001682 | 392 |
| imu | Icm20602 | `gyro_noise_density` | 6.602e-5 | 681 |
| imu | Icm20602 | `accel_noise_density` | 0.0007635 | 681 |
| imu | Icm20608g | `gyro_noise_density` | 9.93e-5 | 13 |
| imu | Icm20608g | `accel_noise_density` | 0.002125 | 13 |
| imu | Icm20649 | `gyro_noise_density` | 0.000185 | 28 |
| imu | Icm20649 | `accel_noise_density` | 0.001699 | 28 |
| imu | Icm20948 | `gyro_noise_density` | 0.0001125 | 22 |
| imu | Icm20948 | `accel_noise_density` | 0.001322 | 22 |
| imu | Icm42670p | `gyro_noise_density` | 5.957e-5 | 79 |
| imu | Icm42670p | `accel_noise_density` | 0.0006262 | 79 |
| imu | Icm42686p | `gyro_noise_density` | 7.024e-5 | 19 |
| imu | Icm42686p | `accel_noise_density` | 0.001483 | 19 |
| imu | Icm45686 | `gyro_noise_density` | 5.727e-5 | 259 |
| imu | Icm45686 | `accel_noise_density` | 0.001229 | 259 |
| imu | Iim42652 | `gyro_noise_density` | 0.0001204 | 81 |
| imu | Iim42652 | `accel_noise_density` | 0.001308 | 81 |
| imu | Iim42653 | `gyro_noise_density` | 0.0001228 | 31 |
| imu | Iim42653 | `accel_noise_density` | 0.002386 | 31 |
| imu | Mpu9250 | `gyro_noise_density` | 9.769e-5 | 31 |
| imu | Mpu9250 | `accel_noise_density` | 0.0009651 | 31 |
| imu | Sch16t | `gyro_noise_density` | 2.835e-5 | 61 |
| imu | Sch16t | `accel_noise_density` | 0.0007964 | 61 |
| baro | Bmp581 | `noise_sigma_m` | 0.02184 | 15 |
| baro | Icp201xx | `noise_sigma_m` | 0.1903 | 117 |
| mag | Ak09916 | `noise_sigma_gauss` | 0.009422 | 113 |
| mag | Bmm150 | `noise_sigma_gauss` | 0.003614 | 239 |
| mag | Bmm350 | `noise_sigma_gauss` | 0.002767 | 10 |
| mag | Iis2mdc | `noise_sigma_gauss` | 0.003449 | 31 |
| mag | Qmc5883p | `noise_sigma_gauss` | 0.01207 | 19 |
| gps | Ubx7 (standalone_3d) | `horizontal_noise_sigma_m` | 2.042 | 16 |
| gps | Ubx7 (standalone_3d) | `altitude_noise_sigma_m` | 3.373 | 16 |
| gps | Ubx7 (standalone_3d) | `velocity_noise_sigma_mps` | 0.4266 | 16 |

### Battery

Per-cell internal resistance by cell count, from 842356 current steps. The steps come from the 11126 logs on the fit side of the vehicle-wise split that had at least one step event, out of a corpus of 35919 real logs (49 simulated logs excluded, split seed 20260623); held-out logs and logs without step events give none. The table now has rows for 1S to 10S and 12S to 14S; 0.13.1 had rows for 1S, 3S, 4S, 6S and 12S, and used its default of 8.3279 mohm per cell for every other cell count.

| Cells | Old (0.13.1) mohm per cell | New mohm per cell | Step events (n) or fallback |
|---|---|---|---|
| 1S | 8.3279 | 5.5699 | n=169, fewer than 200 step events: pooled over vehicles |
| 2S | 8.3279 (default) | 13.4364 | n=4732 |
| 3S | 9.0889 | 8.7361 | n=13839 |
| 4S | 9.2875 | 6.0839 | n=130805 |
| 5S | 8.3279 (default) | 5.6962 | n=1046 |
| 6S | 7.9288 | 3.2146 | n=441198 |
| 7S | 8.3279 (default) | 3.3744 | n=1355 |
| 8S | 8.3279 (default) | 3.2586 | n=21839 |
| 9S | 8.3279 (default) | 5.5699 | n=7, fewer than 200 step events: pooled over vehicles |
| 10S | 8.3279 (default) | 3.8934 | n=487 |
| 12S | 8.3279 (fallback) | 2.1526 | n=195505 |
| 13S | 8.3279 (default) | 3.7978 | n=17009 |
| 14S | 8.3279 (default) | 2.3914 | n=14365 |
| any other count (for example 11S) | 8.3279 (default) | 5.5699 (default) | no row; default pooled over 6380 vehicles |

- The default for a cell count with no row is now 5.5699 mohm per cell, the median over vehicles (6380 vehicles), replacing 8.3279.
- The 20-knot open-circuit voltage table was replaced. First knot (state of charge 0.025): 3.3784 V per cell before, 3.5657 now. Last knot (0.975): 4.0805 before, 4.1545 now. Every knot is in the table below.
- The plausibility floor of 4 mohm per cell is gone. In 0.13.1 a class whose median fell under it (12S, 1.99 mohm) was replaced by the pooled value; 6S, 7S, 8S, 10S, 12S, 13S and 14S now ship their own measured values, all under 4 mohm per cell.

Open-circuit voltage table, V per cell by state of charge (bin centre), old (0.13.1) and new:

| State of charge | Old (0.13.1) | New |
|---|---|---|
| 0.025 | 3.3784 | 3.5657 |
| 0.075 | 3.5724 | 3.6235 |
| 0.125 | 3.5724 | 3.6420 |
| 0.175 | 3.6721 | 3.6789 |
| 0.225 | 3.7079 | 3.7649 |
| 0.275 | 3.7079 | 3.7649 |
| 0.325 | 3.7079 | 3.7649 |
| 0.375 | 3.7228 | 3.7953 |
| 0.425 | 3.7241 | 3.7953 |
| 0.475 | 3.7379 | 3.7953 |
| 0.525 | 3.7424 | 3.8045 |
| 0.575 | 3.7864 | 3.8045 |
| 0.625 | 3.8196 | 3.8495 |
| 0.675 | 3.8388 | 3.8990 |
| 0.725 | 3.8388 | 3.9765 |
| 0.775 | 3.8875 | 3.9765 |
| 0.825 | 3.8955 | 3.9990 |
| 0.875 | 3.9185 | 4.0760 |
| 0.925 | 3.9794 | 4.0992 |
| 0.975 | 4.0805 | 4.1545 |

Deployment notes:

- The run's held-out check per cell count (median over held-out logs scored for that count) is thin for two counts: 10S has only 6 held-out logs and 7S only 11. 10S is also the worst cell count: its median signed error (predicted minus measured voltage per cell) is +0.3524 V per cell over those 6 logs. Per the report, a positive signed error means that the model predicts less sag than the packs show, or that the open-circuit curve sits above these packs. The run does not say which, so the 0.35 V per cell may be missing sag, a curve that sits too high for these packs, or a mix.
- 1S and 9S (the pooled fallback, 5.5699 mohm per cell) have no scored held-out logs: the report's held-out table and `holdout_by_cell_count` in battery-fit.json have no 1S or 9S row, so those two values had no held-out check.
- The fallback is not monotone in cell count. 1S, 9S (both under 200 step events), 11S, and 15S and above use the pooled 5.5699 mohm per cell, while 8S is 3.2586, 10S 3.8934 and 12S 2.1526. An 11S pack therefore gets about 2.6x the resistance per cell of a 12S pack (5.5699 against 2.1526), about 2.4x in total (61.3 mohm against 25.8 mohm).
- 2S and 7S also have a positive median signed error (predicted minus measured voltage per cell): +0.1240 V for 2S and +0.1226 V for 7S, about 0.12 V per cell. As for 10S, the run does not say whether the model predicts less sag than these packs show or the open-circuit curve sits above them.
- For 12S, a comparison made in the calibration tool's review before the cell-count check was added found the old 4 mohm/cell floor slightly better than the measured median (0.0940 vs 0.0957 V per cell held-out error). This figure comes from that review, not from the published run files.

### Read before deploying

IMU noise is at rest, not in flight. The IMU densities are medians of the noise measured while the vehicle was disarmed. The run's own report (section "Gaps TH3SEUS does not model") puts the in-flight gyro noise density at 0.0008893 rad/s/sqrt(Hz) against 0.0001012 at rest, a ratio of 8.78 (987 flight logs, 3671 at-rest logs), and says vibration is not modelled; it also says the ratio is biased low. The 0.13.1 `Mpu6000` and `Other` gyro value, 8.726646e-4, sat near the in-flight level. 0.14.0 lowers it to 7.141e-5 for `Mpu6000` (12.2 times lower) and 9.572e-5 for `Other` (9.1 times lower); the accel density falls from 6.37e-3 to 0.001504 (4.2 times) and 0.001274 (5.0 times). In HITL flight the estimator therefore sees gyro noise well below what these vehicles show in real flight. The run quotes no in-flight accel figure.

What the other values measure:

- GPS rows: the `measurement` fields are `gps.eph_median_m`, `gps.epv_median_m`, `gps.speed_accuracy_median_m_s` and `gps.update_rate_hz`: medians over vehicles of the logged horizontal accuracy (eph), vertical accuracy (epv), speed accuracy and update rate, for the fix class named in the table (`standalone_3d` or `rtk_fixed`).
- Barometer and magnetometer sigmas: the run's rules are an Allan slope within 0.15 of -0.5 and simulator rates of 50.0 Hz for both. A value is scaled by sqrt(simulator rate / logged rate) only when the log is at the simulator rate or above; below it the logged value is kept (the th3seus-tools log_validation README: the PX4 logger keeps one sample per interval instead of averaging, so a series logged below the simulator rate holds samples at the driver's own, unknown rate). For all 17 proposed barometer and magnetometer rows the median factor is 1; the median logged rate is about 1 Hz for 16 of them and 10.36 Hz for `Spl06`. So each sigma is the per-sample noise of samples taken at the driver's own, unknown rate and logged at about 1 Hz (10.36 Hz for `Spl06`). It is not scaled to the 50 Hz the simulator runs these sensors at, and its relation to that rate is unknown, so do not rescale it by sqrt(50/1) or any other factor.

Entries the run did not measure keep their 0.13.1 values (the rule is at least 10 vehicles per proposed value) and now sit far from their measured siblings. They are candidates for a later run:

- `Mpu6500` gyro 9.4e-4 against 7.141e-5 for `Mpu6000` and 9.572e-5 for `Other`; accel 6.86e-3 against 0.001504 and 0.001274. The run's own medians for it, on 9 vehicles and so not proposed, were 1.175e-4 (gyro) and 1.169e-3 (accel).
- `Here3` velocity sigma 0.05 against 0.2344 for `UbloxM9N` (no eligible log for `Here3`, 0 vehicles); its other three measured fields and `Lsm6dso` also have 0 vehicles.
- `Dps310` 0.05 m against 0.2198 m for the barometer `Other`; too few `Dps310` logs pass the white-noise check (2 vehicles).
- `UbloxM10` update rate 10.0 Hz (the run's median was 2.0 Hz on 6 vehicles, not proposed) and `SeptentrioMosaic` update rate 100.0 Hz (no eligible log).

### Breaking change

`ImuChip`, `BaroChip`, `MagChip` and `GpsChipset` gain variants (23 in all), which breaks an exhaustive `match` on any of them in a dependent crate. hitl-daemon has none on these enums: it builds them from strings with `_` arms. Those `_` arms send every new chip string to `Other` until the daemon gets the generated parser arms (`rust/daemon_aliases.rs` in the run's directory in th3seus-tools, for hitl-daemon 0.27.0); without them the new measured profiles are not selected.

### No longer true in measured data

Statements that earlier tests and comments made about the part tables, and that the measured values do not support:

- ICM-42688-P gyro noise is not 10x to 25x lower than MPU-6000: the measured gyro noise density is 7.997e-5 against 7.141e-5, so the ICM-42688-P is about 1.1x noisier. Its accel noise is still lower (0.001368 against 0.001504).
- `ImuChip::Other` no longer equals `Mpu6000`: `Other` gyro is 9.572e-5 and accel 0.001274, against 7.141e-5 for `Mpu6000` gyro.
- RM3100 is not 30x or more quieter than HMC5883: the measured ratio is about 2.2 (0.003405 gauss against 0.001544).
- DPS310 is not the lowest-noise barometer of the older five: SPL06 measures 0.02095 m against the unmeasured DPS310 value of 0.05 m (DPS310 has no proposal and keeps 0.05).
- Here3 no longer has the same horizontal accuracy as the M9N to within 0.01 m: Here3 keeps 0.9 m while `UbloxM9N` is now 0.8705 m (its latency is still higher, 100 ms against 80 ms).
- `GpsChipset::Other` no longer equals `UbloxM8N` for the three noise sigmas: `Other` horizontal is 0.812 m against 1.198 m, altitude 1.25 m against 1.87 m, velocity 0.1391 m/s against 0.268 m/s. The measured `update_rate_hz` is 5.0 for both, and the drift, delay, time-to-first-fix and `max_satellites` fields still match.

### Comments in the code

In the four generated sections of `src/build.rs` (`ImuChip`, `MagChip`, `BaroChip` and `GpsChipset`), the arm comments of the variants that existed in 0.13.1, other than `Other`, are built from their 0.13.1 comments by one rule:

- What a 0.13.1 comment says about the part itself is kept, for example "u-blox ZED-F9P", "single-band" or "cheap, temperature-unstable".
- A 0.13.1 figure is kept only while no field it describes has a measured value. A re-measured field counts even when its value did not change, so the M8N's "18 Hz max (5 Hz default)" is gone although its `update_rate_hz` is still 5.0.
- A claim that compares the part with another is dropped. The ICM-42688-P comment is now "modern flagship", without "~17x quieter than MPU6000", and the DPS310 and MPU-6500 arms have no comment, because "best in class" and "Slightly noisier than MPU6000" were their whole comments.

A comment with nothing dropped keeps its 0.13.1 wording, converted to ASCII. The one 0.13.1 field comment in these sections, "CAN bus latency" on the Here3 `delay_ms`, is now part of the Here3 arm comment as "delay_ms 100.0 is CAN bus latency". th3seus-tools classifies each 0.13.1 phrase in `log_validation/part_notes.py`, and it does not generate a section whose 0.13.1 comments hold a phrase it has not classified.

The 23 new variants carry "PX4 log names X; new in run 2026-11-02", and each `Other` arm says it is the median over all vehicles of its part type. Every GPS arm with a measured value names the fix class of its medians: the named receivers end with "measured over standalone 3D fixes" or "measured over RTK fixed solutions", and the `Other` comment says "standalone 3D fix only".

The `///` docs of `imu_profile`, `mag_profile`, `baro_profile` and `gps_profile` no longer say "Datasheet-derived". Each says that a value with a "was" comment is the median over vehicles measured from real PX4 logs in run 2026-11-02, and that every other value is the hitl-physics 0.13.1 table value. The IMU and GPS docs quote their sources as "hitl-physics 0.13.1 sources:". The comment above each measured value says what it replaced and how many vehicles it is the median of: existing variants carry "was X (hitl-physics 0.13.1); median of N vehicles, run 2026-11-02", and the 23 new variants carry "was X (Other); median of N vehicles, run 2026-11-02". The hand-written docs of `ImuProfile`, `MagProfile`, `BaroProfile`, `GpsProfile` and `to_sensor_profiles` were corrected to match.

## 0.13.1 — 2026-09-04

Performance only. No simulated quantity changes; the 148-test suite passes
unchanged.

- `state_derivative` is 28.7 % faster in isolation (29.36 ns to 20.94 ns,
  measured against a same-session control arm). **`rk4_step` did not move** —
  166.25 ns before, 171.25 ns after, and `rk4_step_x400` flat at ~68 us. The
  standalone benchmark black-boxes its inputs per iteration, which defeats
  hoisting the compiler already performs across the four derivative calls inside
  a step, so what this removes was already gone in the real call path. Recorded
  as measured rather than claimed as a win.

- `apply_thermal_derating_with_max` takes the current ceiling as a parameter
  instead of recomputing it. Deliberately **not** cached on `PhysicsConfig`:
  `BuildSpec::to_physics_config` mutates `kt`, `kq` and
  `max_current_per_motor_a` after construction, so a cached value would be stale
  for exactly the builds that need it.

- Propeller coefficient rows hold `&'static str` and are parsed once through a
  `OnceLock`, folded rather than collected.

- Criterion benches added under `benches/rk4.rs`. Read them with a control arm:
  a stored baseline from a loaded machine measured pristine code at 93.6 ns
  against a 46.9 ns baseline, and criterion will happily report that recovery as
  a 69 % improvement.

## 0.13.0 — 2026-08-29

### Changed

- **BREAKING: `geprc_mark5_6` is renamed `geprc_mark5`.** There is no six-inch
  GEPRC MARK5. All ten products in GEPRC's MARK5 Series are the same 225 mm
  five-inch airframe; the "6" in every listing title is 6S, the cell count.

- **All four reference builds corrected against manufacturer sources.** Every
  one was materially wrong, and two described a different aircraft than their
  name claimed. Reference builds are what every other test is measured against,
  so these errors propagated silently.

  | build | was | is | worst error |
  |---|---|---|---|
  | `tinyhawk_3` | 2S, 12000KV, 3" tri-blade, 75 g | 1S, 15000KV, 40 mm four-blade, 44.5 g | prop diameter: `kt` scales with D⁴, so 3.0" against a real 1.57" is a factor of 13.3 |
  | `impulse_apex_5` | 440 g all-up | 542 g | nothing allocated to camera, VTX, receiver or wiring — 93 g short of a measured build |
  | `geprc_mark5` | "6-inch", 1500KV, 518 g | 5-inch, 1960KV, 611 g | wrong aircraft class |
  | `iflight_chimera_7` | 7" two-blade, 1055 g, pack at 50C | 7.5" three-blade, 1545 g, Li-ion at 17.5C | the pack is Li-ion; 50C handed it a current ceiling it does not have |

  Resulting thrust-to-weight: 2.70, 6.55, 11.78, 6.19 — every one plausible,
  against 8.03, 8.07, 21.36 and 14.56 before this change and the two before it.
  Figures of merit are 0.29, 0.67, 0.59, 0.67, all at or under the 0.68 that is
  the best measured on real hardware.

- `frame.weight_g` is now documented as a CATCH-ALL. Published dry weights are
  whole-aircraft — camera, VTX, antennas, receiver, wiring — and `BuildSpec` has
  no field for any of it. Putting the remainder in `frame` is what makes each
  build's dry sum match its published figure.

### Added

- Table rows for the EMAX Avia TH 40 mm four-blade and the HQ 7.5x3.7x3, so
  both corrected reference builds resolve by exact match rather than by
  nearest-neighbour rescaling.

## 0.12.0 — 2026-08-29

### Changed

- **Blade-count model corrected against measured control pairs.**
  `0.85 + 0.05*blades` made a three-blade only 1.053x a two-blade. The UIUC
  Propeller Data Site published the decisive experiment: the DA4022 and DA4052
  in 5x3.75 and 9x6.75, each run as 2-, 3- and 4-blade on the SAME physical
  blades with only the hub swapped, isolating blade count from every other
  variable. Measured static ratios: 1.373, 1.350, 1.264, 1.280. APC's own files
  give 1.298 and 1.424; CSIR-NAL measured 30% on a six-inch pair.

  Now `(blades/2)^0.7`, giving 1.328. `K_CT` is re-normalised 0.00485 ->
  0.003651555 so the six-inch three-blade lab anchor is preserved at CT_aero
  0.180 exactly; what changes is the relationship between blade counts, not the
  absolute level. Two-blade propellers therefore read about 21% lower than
  before, which is the correction.

- `examples/regenerate_prop_table` now refuses to run against an empty table.
  It reads the file it is normally redirected into, and a shell `>` truncates
  the target before the program starts — so a stale build destroyed the table
  and the next run emitted a header with no rows.

## 0.11.0 — 2026-08-29

### Changed

- **Thrust basis re-anchored to lab bench data.** `K_CT` 0.00283 -> 0.00485 and
  `KQ_KT_SCALE` 0.348 -> 0.47625, giving a six-inch three-blade CT_aero 0.180 /
  CQ_aero 0.0135. Two unrelated sources agree on the target: Cobra CM-2213/12 on
  an HQ 6x4.5x3 (1032.4 g at 17,253 rpm on 273.66 W -> CT 0.1853) and APC's own
  6x4.2E-3 performance file (static CT 0.1850). The previous values were 1.76x
  and 2.41x low respectively.

  The two errors pointed opposite ways, which is why the result looked
  plausible: too little drag torque let the torque balance settle at 98% of
  no-load, and the excess shaft speed made up most of the missing thrust. They
  must be changed together.

- **BREAKING: the full-throttle operating point is now capped by available
  current.** `max_motor_speed_from_voltage()` returns
  `min(voltage-limited, current-limited)`. Previously the torque balance had no
  upper bound and a 2306 on a 6" tri-blade settled at 49.7 A per motor, 199 A
  for the airframe, with nothing objecting. `EscConfig` had carried
  `continuous_amps` and `burst_amps` since it was written and nothing read them.

- **Calibration source changed from a manufacturer figure to a lab bench.** The
  iFlight XING2 2207 anchor (1576 g at 40.35 A) is retired as a gate and kept
  as a bounded comparison. Under current limiting thrust is
  `(kt/kq)·Kt·(I−I0)`, so the thrust-to-torque ratio decides everything: Cobra's
  instrumented ladder gives 87.5 m^-1, the model now gives 82.4, and the iFlight
  figure implies 111.9.

- `data/prop_coefficients.csv` regenerated, and its header now states plainly
  that every value is model output rather than measurement. The manufacturer and
  source columns record which product page fixed each row's geometry, not the
  provenance of its coefficients.

### Added

- `PhysicsConfig::prop_diameter_m`, `disc_area_m2()`, `full_throttle_current_a()`,
  `full_throttle_power_w()`, `momentum_theory_thrust_ceiling_n()`,
  `max_thrust_per_motor_n()` and `is_current_limited()`. The momentum-theory
  ceiling bounds thrust by the power the operating point actually implies, which
  is what catches compensating ct/cq errors.
- `PhysicsConfig::max_current_per_motor_a`, set from
  `min(esc.burst_amps, pack_C_rating/4)`.
- `prop_coefficients::derive_from_basis` and `examples/regenerate_prop_table`,
  so the table is reproducible from the basis instead of hand-edited. Verified
  to regenerate the committed file byte-for-byte.
- `bench_anchor_cobra_cm2213_on_6x45x3`, which validates thrust AND rpm
  together. Thrust alone would pass a model with compensating errors.

### Note

Reference-build figures moved substantially and several are still not
trustworthy — their pack C-ratings and masses need auditing against the real
products (tasks 5.13/5.14 of `fix-hitl-sim-flight-regression`). The
`tinyhawk_3` hover gate was retired as a precision gate rather than re-fitted:
its old band was drawn around the model's own output.

## 0.10.3 — 2026-08-28

### Added

- Reference build for the reported flight regression (EMAX GT-II 2212C 1000KV,
  Gemfan 6040 three-blade, 4S 10 Ah, 600 g frame). Pins today's
  `mass_kg = 2.5445` and `twr = 0.6542` so the thrust basis cannot move without
  a test failing, records the per-term mass breakdown, and asserts that an
  unachievable hover throttle is produced.
- `correcting_the_mass_alone_does_not_make_this_build_fly` — correcting the
  battery (1400 g estimate to a real ~850 g) and the propellers (3 g default to
  a real ~8 g) takes the build to 2014.5 g and TWR 0.8263, still below 1.0.
  The mass defects are real and fixed; they are not what stopped it flying.

### Note

Whether 416 g per motor is correct for this pairing is still unmeasured. The
resulting `CT_aero` of 0.105 sits inside the 0.09-0.12 band `prop_coefficients.rs`
itself calls physical, so the pre-0.10.0 value may have been the wrong one. Do
not move `K_CT` without a bench figure.

## 0.10.0 — 2026-05-30

### Changed

- **Thrust model full recalibration** — fixes the ~14 Hz pitch/roll limit cycle
  (sess111/112) caused by the sim over-producing thrust ~3×. Two root causes fixed:
  1. `max_motor_speed_from_voltage` replaced constant `LOADED_RPM_FRACTION=0.88`
     with a **torque-balance model** that solves
     `Kt·(V−Kt·ω)/R = kq·ω²` per build. Correctly predicts ~0.88× no-load for
     mid-KV 5" builds AND the heavier ~0.5× bog on tiny high-KV motors (Tinyhawk).
  2. `prop_coefficients.csv` regenerated on a **physical-CT basis** (CT_aero ≈ 0.11,
     size-independent). Anchored to iFlight XING2 2207 bench data (1576g @16V).

- User's build (1855KV/5x3/4S/~900g) now hovers at ~65-71% (was ~17%), matching
  real flight. TWR drops from ~8 to ~2. PID authority scaling and per-build PIDs
  handle the new realistic plant response — the limit cycle's gone.

### Removed

- `PhysicsConfig::LOADED_RPM_FRACTION` constant (replaced by torque-balance).
- Armed ESC idle (`MOTOR_IDLE_FRACTION`) — reverted; the real fix is the thrust
  recalibration, not a braking-authority workaround.

## 0.9.5 — 2026-05-29

### Fixed

- **Restored armed ESC idle to kill the ~14 Hz rate-loop limit cycle** (sess111 logs: high-TWR iFlight XING2 2207 1855 kv / 4S build hovering at ~17% throttle). With `MIN_MOTOR_SPEED = 0` (0.9.2) a *braking* motor commanded toward zero coasted to ω≈0, where its control authority `dThrust/dω = 2·kt·ω → 0` — the motor went deaf on the braking half-cycle, producing a one-sided-authority limit cycle (pitch-rate ±115°/s at 14.0 Hz; motors pinned at zero ~20% of the flight, confirmed against the 181 Hz `sensor_combined` gyro). Reintroduced a small armed ESC idle via `MOTOR_IDLE_FRACTION = 0.035` (≈ the prior 955 RPM, now scaling per build), applied as a **linear offset** in `throttle_to_omega_with_config` so `dω/dcmd` stays constant (no return of the 0.9.0 near-zero-slope trembling). The idle models an *armed* ESC — the daemon zeroes motor speeds when disarmed. `MIN_MOTOR_SPEED` stays `0` as the *off* floor (disarmed / depleted / thermal shutdown). The 0.9.2 web-UI RPM "jump" is now fixed at its real source — the daemon's physics-step gate — instead of by removing the idle (see hitl-daemon 0.8.4).

## 0.9.2 — 2026-05-28

### Fixed

- **Motors no longer "jump" while armed-on-ground.** `MIN_MOTOR_SPEED` constant dropped from 100 rad/s (≈955 RPM) to 0. Combined with the daemon's `motors_active = any(cmd > 0.01)` physics-step gate in `loop_runner.rs`, the prior 100 rad/s floor produced this pattern: PX4's rate controller fires tiny micro-corrections at 400 Hz when armed, occasionally crossing 0.01 → physics step runs → all four motors snap from 0 to the 955 RPM floor → cmds dip back under 0.01 → step skipped → motors stay at 955 RPM. Visible in the web UI as repeated RPM jumps. With the floor at 0, `throttle_to_omega_with_config(0, _)` returns 0, matching real PX4 + DSHOT/BLHeli ESC behavior (no controller-side idle unless the user explicitly enables an ESC idle setting). Side-effect: `apply_thermal_derating` at extreme heat now derates to 0 RPM instead of 955 — also more physical (thermal shutdown stops motors).

## 0.9.1 — 2026-05-17

### Reverted

- **Throttle-to-ω is linear again** — undo 0.9.0's ω²-space interpolation. The sqrt has a near-vertical slope at cmd=0 (`dω/dcmd ≈ MAX²/(2·MIN)`, ~16× steeper than linear), which amplified tiny rate-controller PID outputs into 2000+ RPM motor swings at idle and caused visible pre-takeoff trembling. The right place to compensate for the resulting quadratic cmd→thrust is PX4's `THR_MDL_FAC=1` — see `hitl-daemon` 0.7.4 for the matching push. Linear cmd→ω also matches real ESC behavior (BLHeli, Bluejay, etc.), so the simulator is now more physically realistic too.

## 0.9.0 — 2026-05-17 [SUPERSEDED — see 0.9.1]

### Changed

- Throttle-to-thrust contract changed to linear via ω²-space interpolation. Was meant to make `THR_MDL_FAC=0` work cleanly but introduced rate-loop oscillation at idle. Reverted in 0.9.1.

## 0.8.1

- Phase 6: per-build PID push via MAVLink PARAM_SET
- Phase 5: propeller coefficient CSV lookup (ct/cq)
- Phase 1: inertia floor removal (PID scaling compensates)
