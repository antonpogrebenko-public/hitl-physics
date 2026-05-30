# Changelog

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
