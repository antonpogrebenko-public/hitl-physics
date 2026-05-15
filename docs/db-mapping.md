# `BuildSpec` ↔ DB `specs` JSONB mapping

Contract between `th3seus-api`'s `components.specs JSONB` column and the typed
[`BuildSpec`](../src/build.rs) struct in `hitl-physics`. The daemon
(`hitl-daemon/crates/websocket/src/build_config.rs`) is responsible for parsing
JSONB into these structs.

All `Option<T>` fields fall back to library defaults when missing. Required
fields must be present or the daemon should refuse the config with a
descriptive error.

## Conventions

- JSONB keys are camelCase (Web API convention; PostgreSQL JSONB is case-sensitive).
- Units are explicit in the field name where ambiguous (`weight_g`, `wheelbase_mm`, `diameter_in`).
- Unknown enum values map to the `Other` variant — never a hard error.

---

## Frame (`category = 'frame'`)

| `BuildSpec.frame.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `weight_g` | `weightG` | f64 | grams | ✅ | total frame mass |
| `wheelbase_mm` | `wheelbaseMm` | f64 | mm | ✅ | motor-to-motor diagonal |
| `width_mm` | `widthMm` | f64 | mm | – | Phase 1 fallback: `wheelbase_mm / √2` |
| `length_mm` | `lengthMm` | f64 | mm | – | Phase 1 fallback: `wheelbase_mm / √2` |
| `height_mm` | `heightMm` | f64 | mm | – | typical 30 mm |
| `material` | `material` | string | – | – | `"carbon" \| "polymer" \| "aluminum"` |

## Motor (`category = 'motor'`)

| `BuildSpec.motors.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `kv` | `kvRating` | f64 | RPM/V | ✅ | |
| `weight_g` | `weightG` | f64 | grams | ✅ | per-motor mass |
| `stator_size.diameter_mm` | `statorDiameterMm` | f64 | mm | – | e.g. 23 for a 2306 |
| `stator_size.height_mm` | `statorHeightMm` | f64 | mm | – | e.g. 6 for a 2306 |
| `no_load_amps` | `noLoadAmps` | f64 | A | – | Phase 2 — defaults to 0.7 |
| `resistance_ohm` | `resistanceOhm` | f64 | Ω | – | Phase 2 — derived from KV if absent |

## ESC (`category = 'esc'`)

| `BuildSpec.escs.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `continuous_amps` | `continuousCurrentA` | f64 | A | ✅ | |
| `burst_amps` | `burstCurrentA` | f64 | A | – | typically 1.2× continuous |
| `min_cell_count` | `minCellCount` | u8 | – | ✅ | |
| `max_cell_count` | `maxCellCount` | u8 | – | ✅ | |
| `firmware` | `firmware` | string | – | – | `"BLHeli32" \| "AM32" \| "Bluejay"` |
| `response_tau_ms` | `responseTauMs` | f64 | ms | – | Phase 2 — defaults to 3.0 |
| `weight_g` | `weightG` | f64 | grams | – | per-unit mass (4-in-1 reports total/4) |

## Propeller (`category = 'propeller'`)

| `BuildSpec.propellers.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `diameter_in` | `diameterIn` | f64 | inches | ✅ | |
| `pitch_in` | `pitchIn` | f64 | inches | ✅ | fallback in daemon: `diameter * 0.9` |
| `blade_count` | `bladeCount` | i32 | – | ✅ | |
| `weight_g` | `weightG` | f64 | grams | – | per-prop |

## Battery (`category = 'battery'`)

| `BuildSpec.battery.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `cell_count` | `cellCount` | u8 | – | ✅ | |
| `capacity_mah` | `capacityMah` | f64 | mAh | ✅ | |
| `c_rating` | `cRating` | f64 | – | ✅ | continuous discharge |
| `weight_g` | `weightG` | f64 | grams | ✅ | |
| `internal_resistance_mohm` | `internalResistanceMohm` | f64 | mΩ | – | Phase 2 |

## Flight controller (`category = 'flight_controller'`)

| `BuildSpec.flight_controller.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `mcu` | `mcu` | string | – | – | `"F4" \| "F7" \| "H7"` |
| `imu_chip` | `imuChip` | string | – | – | `"MPU6000" \| "ICM42688P" \| "BMI270" \| "LSM6DSO" \| ...` |
| `mag_chip` | `magChip` | string | – | – | `"HMC5883" \| "QMC5883" \| "IST8310" \| "RM3100" \| "LIS3MDL" \| null` |
| `baro_chip` | `baroChip` | string | – | – | `"BMP280" \| "BMP388" \| "DPS310" \| "SPL06" \| "MS5611"` |
| `loop_rate_hz` | `loopRateHz` | u32 | Hz | – | default 8000 |
| `gyro_filter_hz` | `gyroFilterHz` | u32 | Hz | – | default 250 |
| `weight_g` | `weightG` | f64 | grams | ✅ | |

## GPS (`category = 'gps'`, optional)

| `BuildSpec.gps.*` | JSONB key | Type | Unit | Required | Notes |
|---|---|---|---|---|---|
| `chipset` | `chipset` | string | – | ✅ | `"UbloxM8N" \| "UbloxM9N" \| "UbloxM10" \| "UbloxF9P" \| "SeptentrioMosaic" \| "Here3"` |
| `update_rate_hz` | `updateRateHz` | f64 | Hz | – | typical 5-10 |
| `has_compass` | `hasCompass` | bool | – | – | true if the GPS module includes a mag chip |
| `weight_g` | `weightG` | f64 | grams | ✅ | |

---

## Versioning policy

- **Adding fields**: always allowed, must be `Option<T>` or have a default; old DB rows continue to deserialize.
- **Renaming JSONB keys**: requires a coordinated DB backfill + daemon release. Avoid.
- **Changing units**: never. Add a new field and deprecate the old one.

## Parser tolerance

The daemon MUST:
- Treat missing optional keys as `None` — log a `debug!` not `warn!`.
- Treat unknown enum values as `Other` and `warn!` once per unknown value per session.
- Treat malformed numeric values (e.g. string `"5.0"` instead of `5.0`) as parse errors and reject the entire `ConfigureBuild` message with a clear error.
