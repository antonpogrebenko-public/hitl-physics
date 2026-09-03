//! Integration-loop benchmarks.
//!
//! These exist because nothing in this crate measured itself. Every
//! performance claim made about the physics engine to date has been an
//! argument from code structure — an instruction count, an allocation
//! tally — and an argument cannot notice when it stops being true.
//!
//! The daemon runs `rk4_step` at 400 Hz, so `state_derivative` runs 1600
//! times a second. The tick budget is 2500 µs; these two figures are what
//! the physics engine spends out of it.
//!
//! Run: cargo bench -p hitl-physics

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use hitl_physics::{rk4_step, PhysicsConfig, QuadrotorState, DEFAULT_DT};

/// The hover operating point.
///
/// Benched here rather than at rest because hover is what the vehicle
/// spends its time doing, and because it is the point where the motor
/// model is fully exercised — a state with zero motor speed skips work
/// the real loop does on every tick.
fn hover() -> (PhysicsConfig, QuadrotorState, [f64; 4]) {
    let config = PhysicsConfig::default();
    let state = QuadrotorState::at_hover(&config);
    let commands = [config.hover_motor_speed(); 4];
    (config, state, commands)
}

fn bench_state_derivative(c: &mut Criterion) {
    let (config, state, commands) = hover();
    c.bench_function("state_derivative", |b| {
        b.iter(|| black_box(&state).state_derivative(black_box(&config), black_box(commands)))
    });
}

fn bench_rk4_step(c: &mut Criterion) {
    let (config, state, commands) = hover();
    c.bench_function("rk4_step", |b| {
        b.iter(|| {
            rk4_step(
                black_box(&state),
                black_box(&config),
                black_box(commands),
                black_box(DEFAULT_DT),
            )
        })
    });
}

/// One second of simulated flight at the daemon's rate.
///
/// The per-call figures above are what regressions show up in; this one is
/// what the number actually means. 400 steps is one second of wall time in
/// the daemon, so this result read as milliseconds is the percentage of a
/// core the physics engine costs.
fn bench_four_hundred_steps(c: &mut Criterion) {
    let (config, state, commands) = hover();
    c.bench_function("rk4_step_x400", |b| {
        b.iter(|| {
            let mut s = state.clone();
            for _ in 0..400 {
                s = rk4_step(&s, black_box(&config), black_box(commands), DEFAULT_DT);
            }
            black_box(s)
        })
    });
}

criterion_group!(
    benches,
    bench_state_derivative,
    bench_rk4_step,
    bench_four_hundred_steps
);
criterion_main!(benches);
