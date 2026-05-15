# hitl-physics

Pure Rust quadrotor physics library for HITL simulation. No I/O, no async — just math.

## Overview

Provides rigid body dynamics for quadrotor simulation:
- State vector (position, velocity, quaternion, angular velocity, motor speeds)
- Force/torque computation (thrust, drag, gravity)
- RK4 numerical integration
- Motor dynamics (first-order response)

## Usage

```rust
use hitl_physics::{PhysicsConfig, QuadrotorState, integrate_rk4};

let config = PhysicsConfig::default();  // 5" racing quad
let mut state = QuadrotorState::at_hover(&config);
let motor_commands = [1600.0, 1600.0, 1600.0, 1600.0];  // rad/s

// Integrate one timestep
state = integrate_rk4(&state, &config, motor_commands, 0.0025);  // 400 Hz
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

## Key Files

- `src/config.rs` — PhysicsConfig struct and defaults
- `src/quadrotor.rs` — QuadrotorState, force/torque computation
- `src/motor.rs` — Motor model (throttle_to_omega, thrust, torque)
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

## Gotchas

### Yaw torque formula
```rust
let tau_yaw = q1 + q2 - q3 - q4;
```
CCW motors (1=FR, 2=BL) produce positive yaw torque. CW motors (3=FL, 4=BR) produce negative.

### Inertia tuning for PX4
Izz must be >= 0.020 for stable yaw with PX4's default PIDs. `from_build_specs` enforces this floor. Lower values cause rapid yaw hunting (same-direction motors oscillating high/low cyclically).
