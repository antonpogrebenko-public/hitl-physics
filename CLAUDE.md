# hitl-physics

Pure Rust quadrotor physics library for HITL simulation. No I/O, no async — just math.

## Overview

Provides rigid body dynamics for quadrotor simulation:
- State vector (position, velocity, quaternion, angular velocity, motor speeds, motor temps)
- Force/torque computation (thrust, drag, gravity)
- RK4 numerical integration
- Motor dynamics (first-order response, electrical model, thermal model)
- Battery model (LiPo discharge curve, flight time estimation)
- Build-spec-based configuration (KV, prop, frame weight → physics params)

## Usage

```rust
use hitl_physics::{PhysicsConfig, QuadrotorState, rk4_step, DEFAULT_DT};

let config = PhysicsConfig::default();  // 5" racing quad
let state = QuadrotorState::at_hover(&config);
let hover_omega = config.hover_motor_speed();

// Step simulation with hover thrust
let motor_commands = [hover_omega; 4];
let new_state = rk4_step(&state, &config, motor_commands, DEFAULT_DT);
```

## Motor Layout

```
    Front (nose)
  3(CW)   1(CCW)
     \   /
       X
     /   \
  2(CCW) 4(CW)
    Back
```

PX4 Standard Quad X — identity mapping (no remapping). Motors 1,2 are CCW; motors 3,4 are CW.

## Configuration

Default values match a typical 5" FPV racing quad (~2kg AUW):

| Parameter | Default | Description |
|-----------|---------|-------------|
| mass_kg | 2.0 | Total mass |
| arm_length_m | 0.11 | Center to motor |
| inertia | [0.015, 0.015, 0.025] | Ixx, Iyy, Izz (kg·m²) |
| kt | 1.9e-6 | Thrust coefficient |
| kq | 5.0e-8 | Torque coefficient |
| tau_motor | 0.025 | Motor time constant (s) |
| drag_coeffs | [0.016, 0.016, 0.022] | Quadratic drag (0.5·ρ·Cd·A) |
| motor_kv | 1200.0 | Motor KV rating |
| battery_voltage | 14.8 | 4S LiPo nominal |

### `from_build_specs` constructor

Creates config from real-world build parameters:
```rust
PhysicsConfig::from_build_specs(
    kv: 1800.0,              // Motor KV
    prop_diameter_inches: 5.0,
    prop_pitch_inches: 3.0,
    blade_count: 2,
    frame_weight_g: 350.0,
    motor_weight_g: 33.0,
    battery_voltage: 14.8,
)
```

Derives kt, kq, inertia, drag, tau_motor, and electrical/thermal parameters from these inputs.

## Key Files

- `src/config.rs` — PhysicsConfig struct, `from_build_specs`, hover/max speed calculations
- `src/quadrotor.rs` — QuadrotorState, force/torque computation, Euler's equations
- `src/motor.rs` — Motor model (throttle_to_omega, thrust, torque, electrical current, thermal)
- `src/battery.rs` — BatteryConfig, BatteryState, LiPo voltage curve, flight time estimation
- `src/integrator.rs` — RK4 numerical integration
- `src/wasm_bindings.rs` — WASM bindings for browser use

## Building

```bash
cargo build
cargo test

# For WASM (browser use)
wasm-pack build --target web
```

## Coordinate Frame

- **NED** (North-East-Down) for position/velocity
- **FRD** (Forward-Right-Down) for body frame
- Quaternion rotates body → NED

## Physics Models

### Motor Current (torque-based)
```
I = kq × ω² / Kt_electrical + I_no_load
```
Where `Kt_electrical = 60 / (2π × KV)`. More accurate than voltage-drop model at all speed ranges.

### kq/kt ratio (prop-dependent)
```
kq_kt_ratio = 0.010 + 0.002 × pitch_inches + 0.002 × blade_count
```
Low-pitch 2-blade ≈ 0.020, aggressive 3-blade 5x4.5 ≈ 0.025.

### Drag (area-based)
```
F = -c × v × |v|
c = 0.5 × ρ × Cd × A_frontal
```
Frontal area scales with prop_diameter². Lateral Cd≈1.1, vertical Cd≈1.5.

### Battery
LiPo piecewise-linear voltage curve (4.2V→3.3V per cell). Depleted at 5% SoC — motors cut to zero.

## Gotchas

### Yaw torque formula
```rust
let tau_yaw = q1 + q2 - q3 - q4;
```
CCW motors (1=FR, 2=BL) produce positive yaw torque. CW motors (3=FL, 4=BR) produce negative.

### Inertia tuning for PX4
`from_build_specs` enforces inertia floors for PX4's default rate PIDs:
- Ixx/Iyy >= 0.012 kg·m² (prevents roll/pitch oscillation on lightweight builds)
- Izz >= max(Ixx*1.7, 0.020) (prevents yaw hunting, keeps Izz/Ixx ratio physical)

Without the Ixx floor, builds under ~500g compute inertias so low that PX4's rate controller overshoots, causing visible motor RPM oscillation and frame trembling.

### Drag coefficients must be physically derived
Hardcoded drag values (e.g., 0.25) give unrealistic max speeds (~4 m/s). Always compute from `0.5 × ρ × Cd × A_frontal`. A 5" quad should achieve 15-20 m/s in position mode.

### Battery depletion kills motors
When `BatteryState::is_depleted()` returns true (SoC < 5%), the simulation loop must zero motor commands. The physics library doesn't enforce this — the daemon loop does.

### Thermal derating is applied inside `state_derivative` (not externally)
`apply_thermal_derating()` is called in `quadrotor.rs` inside the per-motor derivative computation. A motor that reaches shutdown temperature coasts smoothly to zero thrust — the daemon loop does **not** need to apply derating separately. Do not double-apply from outside.

### Motor speeds are clamped to `>= 0.0` in the integrator
`integrator.rs` clamps motor speeds to non-negative values both in the `rk4_step` final assembly and in `apply_derivative` intermediate states. This prevents negative RPM artifacts from numerical overshoot (e.g., after a hard cutoff). Any code that post-processes motor state should expect values `>= 0.0`.

### `compute_electrical_power` requires explicit terminal voltage
The signature is `compute_electrical_power(terminal_voltage: f64, ...)` — it no longer reads `config.battery_voltage`. Callers must pass the actual terminal voltage from the current `BatteryState`. Pass nominal voltage (`config.battery_voltage`) only as a fallback when no battery state is available (e.g., unit tests).

### `fingerprint()` in `px4_pids.rs` uses deterministic FNV-1a
The PID config fingerprint replaced `DefaultHasher` with FNV-1a over f32 bit patterns. The hash is now stable across process restarts and platforms. If you serialize fingerprints (e.g., to localStorage or logs), old values computed with `DefaultHasher` are invalidated and will trigger a config resend on the next daemon start.
