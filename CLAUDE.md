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
  1(CW)   2(CCW)
     \   /
       X
     /   \
  4(CCW) 3(CW)
    Back
```

After PX4→Sim remapping, motors 1 and 3 spin CW, motors 2 and 4 spin CCW.

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
let tau_yaw = -q1 + q2 - q3 + q4;
```
Motors 1,3 spin CW (negative reaction), motors 2,4 spin CCW (positive reaction).

### Inertia tuning for PX4
Higher Izz (0.025) reduces yaw oscillation with PX4's default PIDs. Lower values cause rapid yaw hunting.
