use hitl_physics::BuildSpec;

fn main() {
    let mut spec = BuildSpec::default();
    spec.motors.kv = 1800.0;
    spec.motors.weight_g = 33.0;
    spec.propellers.diameter_in = 5.0;
    spec.propellers.pitch_in = 3.0;
    spec.propellers.blade_count = 2;
    spec.frame.weight_g = 350.0;
    spec.battery.cell_count = 4;
    spec.battery.capacity_mah = 1000.0;

    let physics = spec.to_physics_config();
    let hover_omega = physics.hover_motor_speed();
    let max_omega = physics.max_motor_speed_from_voltage();
    let hover_throttle = hover_omega / max_omega;

    println!("Mass: {:.3} kg", physics.mass_kg);
    println!("kt: {:.6e}", physics.kt);
    println!("Inertia: [{:.5}, {:.5}, {:.5}]", physics.inertia[0], physics.inertia[1], physics.inertia[2]);
    println!("Hover omega: {:.1} rad/s", hover_omega);
    println!("Max omega: {:.1} rad/s", max_omega);
    println!("Hover throttle: {:.3} ({:.1}%)", hover_throttle, hover_throttle * 100.0);
    println!("TWR: {:.2}", (4.0 * physics.kt * max_omega * max_omega) / (physics.mass_kg * 9.81));
}
