//! Bisects pitch-vs-roll asymmetry in the bare physics path.
//!
//! Background: ULOG `/Users/tonypo/Downloads/log1.ulg` shows a 25-35×
//! amplitude asymmetry between pitch and roll axes during HITL hover
//! (pitch_rate RMS 5°/s vs roll_rate RMS 0.2°/s at 10-30 Hz). The asymmetry
//! is motor-speed dependent (collapses to 1.3× at motor floor). Code review
//! of inertia, drag, mixer, and PIDs found no structural asymmetry.
//!
//! This harness drives the bare physics with two motor patterns of identical
//! magnitude — one producing pure +τ_pitch, the other pure +τ_roll — both
//! starting from the same hover state. If the angular-velocity responses are
//! mirror-symmetric, the bug is in PX4 / closed-loop coupling, not physics.
//! If they differ, the bug is in this crate.
//!
//! Run: `cargo run --example axis_symmetry --release`

use hitl_physics::{rk4_step, BuildSpec, PhysicsConfig, QuadrotorState};

const DT: f64 = 1.0 / 400.0; // 400 Hz, matches daemon
const STEPS: usize = 400; // 1 second
const PERTURBATION: f64 = 0.05; // ±5% of hover speed — small, linear regime

fn build_config() -> PhysicsConfig {
    let mut spec = BuildSpec::default();
    spec.motors.kv = 1800.0;
    spec.motors.weight_g = 33.0;
    spec.propellers.diameter_in = 5.0;
    spec.propellers.pitch_in = 3.0;
    spec.propellers.blade_count = 2;
    spec.frame.weight_g = 350.0;
    spec.battery.cell_count = 4;
    spec.battery.capacity_mah = 1000.0;
    spec.to_physics_config()
}

/// Returns `[m0=FR, m1=BL, m2=FL, m3=BR]` motor commands.
/// Mapping per CLAUDE.md: ch0=FR(CCW), ch1=BL(CCW), ch2=FL(CW), ch3=BR(CW).
fn pitch_pattern(hover_omega: f64) -> [f64; 4] {
    // +τ_pitch = (FR + FL) - (BL + BR) > 0 → front faster than back.
    let delta = hover_omega * PERTURBATION;
    [
        hover_omega + delta, // m0 FR  (front)
        hover_omega - delta, // m1 BL  (back)
        hover_omega + delta, // m2 FL  (front)
        hover_omega - delta, // m3 BR  (back)
    ]
}

fn roll_pattern(hover_omega: f64) -> [f64; 4] {
    // +τ_roll = (BL + FL) - (FR + BR) > 0 → left faster than right.
    let delta = hover_omega * PERTURBATION;
    [
        hover_omega - delta, // m0 FR  (right)
        hover_omega + delta, // m1 BL  (left)
        hover_omega + delta, // m2 FL  (left)
        hover_omega - delta, // m3 BR  (right)
    ]
}

struct Trace {
    label: &'static str,
    /// (sample_time, body_gyro_x, body_gyro_y, body_gyro_z)
    samples: Vec<(f64, f64, f64, f64)>,
    /// Torque produced by the motor pattern at t=0 (sanity check).
    initial_torque: [f64; 3],
}

fn run(label: &'static str, config: &PhysicsConfig, pattern: [f64; 4]) -> Trace {
    let mut state = QuadrotorState::at_hover(config);

    // Sanity probe: confirm the motor pattern actually produces the intended
    // torque if motors WERE already at the perturbed speeds. The simulation
    // below feeds the pattern as motor commands, so the first-order motor
    // dynamics ramp from hover to the commanded speed (matches the daemon).
    let mut probe = state.clone();
    probe.motor_speeds = pattern;
    let (_, tau0) = probe.compute_motor_forces(config);

    let mut trace = Trace {
        label,
        samples: Vec::with_capacity(STEPS),
        initial_torque: [tau0.x, tau0.y, tau0.z],
    };

    for i in 0..STEPS {
        let t = i as f64 * DT;
        trace.samples.push((
            t,
            state.angular_velocity[0],
            state.angular_velocity[1],
            state.angular_velocity[2],
        ));
        state = rk4_step(&state, config, pattern, DT);
    }
    trace
}

fn report(t: &Trace) {
    let n = t.samples.len();
    let gx: Vec<f64> = t.samples.iter().map(|s| s.1).collect();
    let gy: Vec<f64> = t.samples.iter().map(|s| s.2).collect();
    let gz: Vec<f64> = t.samples.iter().map(|s| s.3).collect();

    let max = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = |v: &[f64]| v.iter().cloned().fold(f64::INFINITY, f64::min);
    let abs_max = |v: &[f64]| v.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
    let final_val = |v: &[f64]| v[n - 1];

    println!("--- {} ---", t.label);
    println!(
        "  τ at t=0      : roll={:+.4e}  pitch={:+.4e}  yaw={:+.4e}  N·m",
        t.initial_torque[0], t.initial_torque[1], t.initial_torque[2]
    );
    println!(
        "  ω_x (roll)    : min={:+8.4} max={:+8.4} |max|={:8.4} final={:+8.4} rad/s",
        min(&gx), max(&gx), abs_max(&gx), final_val(&gx),
    );
    println!(
        "  ω_y (pitch)   : min={:+8.4} max={:+8.4} |max|={:8.4} final={:+8.4} rad/s",
        min(&gy), max(&gy), abs_max(&gy), final_val(&gy),
    );
    println!(
        "  ω_z (yaw)     : min={:+8.4} max={:+8.4} |max|={:8.4} final={:+8.4} rad/s",
        min(&gz), max(&gz), abs_max(&gz), final_val(&gz),
    );
}

fn compare(pitch_run: &Trace, roll_run: &Trace) {
    // Symmetry hypothesis:
    //   pitch_run.ω_y (response to +τ_pitch) ≈ roll_run.ω_x (response to +τ_roll)
    let n = pitch_run.samples.len();
    let mut max_abs_diff: f64 = 0.0;
    let mut sum_sq_diff = 0.0;
    let mut pitch_axis_norm = 0.0;
    let mut leak_pitch_into_roll: f64 = 0.0;
    let mut leak_roll_into_pitch: f64 = 0.0;
    for i in 0..n {
        let oy_pitch = pitch_run.samples[i].2; // on-axis
        let ox_roll = roll_run.samples[i].1;   // on-axis
        let diff = oy_pitch - ox_roll;
        max_abs_diff = max_abs_diff.max(diff.abs());
        sum_sq_diff += diff * diff;
        pitch_axis_norm += oy_pitch * oy_pitch;

        leak_pitch_into_roll = leak_pitch_into_roll.max(pitch_run.samples[i].1.abs());
        leak_roll_into_pitch = leak_roll_into_pitch.max(roll_run.samples[i].2.abs());
    }
    let rms_diff = (sum_sq_diff / n as f64).sqrt();
    let rms_pitch = (pitch_axis_norm / n as f64).sqrt();
    let rel_diff_pct = if rms_pitch > 0.0 {
        100.0 * rms_diff / rms_pitch
    } else {
        f64::NAN
    };

    println!();
    println!("=== SYMMETRY COMPARISON ===");
    println!(
        "  on-axis  max |ω_pitch.y − ω_roll.x| : {:.6e} rad/s   ({:.4}% of pitch RMS)",
        max_abs_diff, rel_diff_pct
    );
    println!("  cross-axis leakage in pitch run (max |ω_x|) : {:.6e} rad/s",
        leak_pitch_into_roll);
    println!("  cross-axis leakage in roll  run (max |ω_y|) : {:.6e} rad/s",
        leak_roll_into_pitch);
    println!();
    if rel_diff_pct.is_finite() && rel_diff_pct < 1.0 {
        println!("→ PHYSICS IS SYMMETRIC. Asymmetry must be in PX4 / closed-loop side.");
    } else {
        println!("→ PHYSICS IS ASYMMETRIC. Bug lives in hitl-physics.");
    }
}

fn main() {
    let config = build_config();
    let hover_omega = config.hover_motor_speed();

    println!("Config: mass={:.3}kg  Ixx={:.5}  Iyy={:.5}  Izz={:.5}  hover_ω={:.1}rad/s",
        config.mass_kg, config.inertia[0], config.inertia[1], config.inertia[2], hover_omega);
    println!("Perturbation: ±{:.1}% of hover_ω, applied as motor command for {:.2}s @ 400Hz",
        PERTURBATION * 100.0, DT * STEPS as f64);
    println!();

    let pitch_run = run("+τ_pitch  (front motors faster)", &config, pitch_pattern(hover_omega));
    let roll_run  = run("+τ_roll   (left motors faster)",  &config, roll_pattern(hover_omega));
    report(&pitch_run);
    report(&roll_run);
    compare(&pitch_run, &roll_run);
}
