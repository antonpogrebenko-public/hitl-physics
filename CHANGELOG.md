# Changelog

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
