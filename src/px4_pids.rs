//! Per-build PX4 rate-controller gains (Phase 6).
//!
//! Computes the PX4 multirotor rate-controller PIDs that match a given
//! airframe's moments of inertia. Pure math — no MAVLink or I/O.
//!
//! ## Why
//!
//! PX4's default rate PIDs are tuned for a ~700 g 5"-class race quad with
//! `I ≈ [0.005, 0.005, 0.009] kg·m²`. The legacy `from_build_specs` enforces
//! a `0.012`/`0.020` inertia floor specifically to keep light airframes
//! inside that tuned region, otherwise PX4's controller produces too much
//! torque per unit error and the airframe oscillates.
//!
//! Phase 6 generates per-airframe PIDs so the inertia floor can be removed
//! (Phase 1). The daemon will eventually push these via MAVLink `PARAM_SET`
//! on `ConfigureBuild`; that integration is deferred until PX4-SITL
//! validation lands. This module supplies the math.
//!
//! ## Reference build
//!
//! Anchored on PX4 v1.14 defaults for a 5"-class build:
//!   - `MC_ROLLRATE_P = MC_PITCHRATE_P = 0.15`
//!   - `MC_ROLLRATE_I = MC_PITCHRATE_I = 0.20`
//!   - `MC_ROLLRATE_D = MC_PITCHRATE_D = 0.003`
//!   - `MC_YAWRATE_P  = 0.20`
//!   - `MC_YAWRATE_I  = 0.10`
//!   - `MC_YAWRATE_D  = 0.0`
//! at reference inertia `[Ixx, Iyy, Izz] = [0.005, 0.005, 0.009]`.
//!
//! ## Scaling rules
//!
//! The rate controller produces commanded torque `τ = P · ω_err + ...`.
//! To produce the same angular response on a different airframe, `P` must
//! scale linearly with the axis inertia:
//!
//! - `P(I)  = P_ref  · I / I_ref` (linear — proportional response)
//! - `D(I)  = D_ref  · I / I_ref` (linear — same physics as P, acts on `d(ω_err)/dt`)
//! - `I(I)  = I_ref  · sqrt(I / I_ref)` (sub-linear — integral is less sensitive
//!   because it accumulates over time and is bounded by anti-windup)
//! - `FF(I) = FF_ref · I / I_ref` (linear — feed-forward torque)
//!
//! These are heuristics, not derived from formal stability analysis. The
//! Phase 6 plan says the reference 5" build should match PX4 defaults within
//! 5% and lighter / heavier builds should scale proportionally; this module
//! satisfies both.

use crate::config::PhysicsConfig;

/// Reference moments of inertia (kg·m²) at which PX4 v1.14 defaults are tuned.
/// Corresponds to a typical 5"-class race quad without the legacy inertia floor.
pub const REF_INERTIA: [f64; 3] = [0.005, 0.005, 0.009];

/// PX4 v1.14 default rate-controller gains at [`REF_INERTIA`].
pub const REF_PIDS: Px4Pids = Px4Pids {
    roll_p: 0.15,  roll_i: 0.20,  roll_d: 0.003,  roll_ff: 0.0,
    pitch_p: 0.15, pitch_i: 0.20, pitch_d: 0.003, pitch_ff: 0.0,
    yaw_p: 0.20,   yaw_i: 0.10,   yaw_d: 0.0,     yaw_ff: 0.0,
};

/// Rate-controller gains corresponding to PX4's `MC_{ROLL,PITCH,YAW}RATE_*`
/// parameters. `f32` because PX4 stores params as 32-bit floats.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Px4Pids {
    pub roll_p: f32,
    pub roll_i: f32,
    pub roll_d: f32,
    pub roll_ff: f32,
    pub pitch_p: f32,
    pub pitch_i: f32,
    pub pitch_d: f32,
    pub pitch_ff: f32,
    pub yaw_p: f32,
    pub yaw_i: f32,
    pub yaw_d: f32,
    pub yaw_ff: f32,
}

/// Reference hover command for the 5"-class build at which REF_PIDS are tuned.
/// Corresponds to TWR ≈ 3.6 (1 / 0.28 ≈ 3.6).
const REF_HOVER_CMD: f64 = 0.28;

/// Compute rate-controller gains for the given airframe.
///
/// `hover_cmd` is the normalized hover throttle (0–1). High-TWR builds hover
/// at low throttle (e.g. 0.14), meaning motors saturate at zero before the
/// controller can fully brake a rotation. Without attenuation, the asymmetric
/// authority creates a limit cycle where one motor pair saturates at zero
/// every half-period.
///
/// The reference build hovers at ~0.28 (TWR ≈ 3.6). When `hover_cmd` is lower,
/// P/I/D gains are scaled by the braking authority ratio: at low hover_cmd the
/// motors saturate at zero long before the full correction is delivered. The
/// available braking torque is `hover/(1-hover)` normalized to the reference
/// build's `ref/(1-ref)`. This ratio is used directly (no sqrt) because the
/// gain margin before motor saturation is linear in braking authority — sqrt
/// was insufficient for extreme-TWR builds (>8:1) where the limit cycle
/// persisted. Feed-forward is not attenuated.
pub fn compute_pids(physics: &PhysicsConfig, hover_cmd: f32) -> Px4Pids {
    let safe = |i: f64, axis_ref: f64| -> f64 {
        if i.is_finite() && i > 0.0 {
            i
        } else {
            axis_ref
        }
    };
    let i_roll = safe(physics.inertia[0], REF_INERTIA[0]);
    let i_pitch = safe(physics.inertia[1], REF_INERTIA[1]);
    let i_yaw = safe(physics.inertia[2], REF_INERTIA[2]);

    let r_roll = i_roll / REF_INERTIA[0];
    let r_pitch = i_pitch / REF_INERTIA[1];
    let r_yaw = i_yaw / REF_INERTIA[2];

    let i_scale = |r: f64| r.sqrt();

    // Attenuate gains for high-TWR builds where braking authority is limited.
    // Braking ratio = (hover/(1-hover)) / (ref/(1-ref)): how much braking torque
    // this build has relative to the reference. Linear (no sqrt) because the
    // gain margin before saturation scales linearly with braking authority.
    let hover = (hover_cmd as f64).clamp(0.05, 1.0);
    let ref_braking = REF_HOVER_CMD / (1.0 - REF_HOVER_CMD);
    let build_braking = hover / (1.0 - hover);
    let authority_scale = (build_braking / ref_braking).min(1.0);

    Px4Pids {
        roll_p:   (REF_PIDS.roll_p   as f64 * r_roll * authority_scale) as f32,
        roll_i:   (REF_PIDS.roll_i   as f64 * i_scale(r_roll) * authority_scale) as f32,
        roll_d:   (REF_PIDS.roll_d   as f64 * r_roll * authority_scale) as f32,
        roll_ff:  (REF_PIDS.roll_ff  as f64 * r_roll) as f32,
        pitch_p:  (REF_PIDS.pitch_p  as f64 * r_pitch * authority_scale) as f32,
        pitch_i:  (REF_PIDS.pitch_i  as f64 * i_scale(r_pitch) * authority_scale) as f32,
        pitch_d:  (REF_PIDS.pitch_d  as f64 * r_pitch * authority_scale) as f32,
        pitch_ff: (REF_PIDS.pitch_ff as f64 * r_pitch) as f32,
        yaw_p:    (REF_PIDS.yaw_p    as f64 * r_yaw * authority_scale) as f32,
        yaw_i:    (REF_PIDS.yaw_i    as f64 * i_scale(r_yaw) * authority_scale) as f32,
        yaw_d:    (REF_PIDS.yaw_d    as f64 * r_yaw * authority_scale) as f32,
        yaw_ff:   (REF_PIDS.yaw_ff   as f64 * r_yaw) as f32,
    }
}

/// Stable fingerprint of computed PIDs. Useful for caching: the daemon can
/// compare two fingerprints and skip the `PARAM_SET` sequence on reconfigure
/// when the gains haven't actually changed.
///
/// Uses FNV-1a over the raw f32 bit patterns — fully deterministic across
/// process restarts (unlike `DefaultHasher`, which uses a random seed).
pub fn fingerprint(pids: &Px4Pids) -> u64 {
    const FNV_OFFSET: u64 = 0xcbf29ce484222325;
    const FNV_PRIME: u64 = 0x00000100000001b3;

    let mut hash = FNV_OFFSET;
    for &val in &[
        pids.roll_p, pids.roll_i, pids.roll_d, pids.roll_ff,
        pids.pitch_p, pids.pitch_i, pids.pitch_d, pids.pitch_ff,
        pids.yaw_p, pids.yaw_i, pids.yaw_d, pids.yaw_ff,
    ] {
        for byte in val.to_bits().to_le_bytes() {
            hash ^= byte as u64;
            hash = hash.wrapping_mul(FNV_PRIME);
        }
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config_with_inertia(inertia: [f64; 3]) -> PhysicsConfig {
        let mut c = PhysicsConfig::default();
        c.inertia = inertia;
        c
    }

    fn rel_diff(a: f64, b: f64) -> f64 {
        if b.abs() < 1e-12 {
            (a - b).abs()
        } else {
            (a - b).abs() / b.abs()
        }
    }

    #[test]
    fn reference_inertia_returns_px4_defaults_within_5_percent() {
        let physics = config_with_inertia(REF_INERTIA);
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        assert!(rel_diff(pids.roll_p as f64, 0.15) < 0.05);
        assert!(rel_diff(pids.pitch_p as f64, 0.15) < 0.05);
        assert!(rel_diff(pids.yaw_p as f64, 0.20) < 0.05);
        assert!(rel_diff(pids.roll_i as f64, 0.20) < 0.05);
        assert!(rel_diff(pids.pitch_d as f64, 0.003) < 0.05);
    }

    #[test]
    fn light_airframe_scales_gains_down() {
        // Tinyhawk-class ~0.3x reference inertia
        let physics = config_with_inertia([0.0015, 0.0015, 0.003]);
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // P should be roughly 0.3x
        let expected_p = 0.15 * 0.3;
        assert!(
            (pids.roll_p as f64 - expected_p).abs() < 0.02,
            "roll_p {:.4} not near {:.4}",
            pids.roll_p, expected_p
        );
        // I scales with sqrt(0.3) ≈ 0.547
        let expected_i = 0.20 * (0.3_f64).sqrt();
        assert!(
            (pids.roll_i as f64 - expected_i).abs() < 0.02,
            "roll_i {:.4} not near {:.4}",
            pids.roll_i, expected_i
        );
    }

    #[test]
    fn heavy_airframe_scales_gains_up() {
        // 7" cinelifter-class ~3x reference inertia
        let physics = config_with_inertia([0.015, 0.015, 0.027]);
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        let expected_p = 0.15 * 3.0;
        assert!(
            (pids.roll_p as f64 - expected_p).abs() < 0.02,
            "roll_p {:.4} not near {:.4}",
            pids.roll_p, expected_p
        );
        // I scales sub-linearly
        let expected_i = 0.20 * (3.0_f64).sqrt();
        assert!(
            (pids.roll_i as f64 - expected_i).abs() < 0.02,
            "roll_i {:.4} not near {:.4}",
            pids.roll_i, expected_i
        );
    }

    #[test]
    fn yaw_axis_scales_with_izz_not_ixx() {
        // High-Izz / low-Ixx airframe (e.g., elongated frame)
        let physics = config_with_inertia([0.005, 0.005, 0.018]);
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // Yaw_p = 0.20 * (0.018 / 0.009) = 0.40
        assert!((pids.yaw_p as f64 - 0.40).abs() < 0.01);
        // Roll_p stays at reference
        assert!((pids.roll_p as f64 - 0.15).abs() < 0.01);
    }

    #[test]
    fn d_term_scales_linearly_with_inertia() {
        let physics = config_with_inertia([0.0025, 0.0025, 0.0045]);
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // Half the reference inertia → half the D term
        let expected_d = 0.003 * 0.5;
        assert!((pids.roll_d as f64 - expected_d).abs() < 1e-4);
        assert!((pids.pitch_d as f64 - expected_d).abs() < 1e-4);
    }

    #[test]
    fn fingerprint_is_stable_and_changes_with_inertia() {
        let a = compute_pids(&config_with_inertia([0.005, 0.005, 0.009]), REF_HOVER_CMD as f32);
        let b = compute_pids(&config_with_inertia([0.005, 0.005, 0.009]), REF_HOVER_CMD as f32);
        let c = compute_pids(&config_with_inertia([0.006, 0.005, 0.009]), REF_HOVER_CMD as f32);
        assert_eq!(fingerprint(&a), fingerprint(&b));
        assert_ne!(fingerprint(&a), fingerprint(&c));
    }

    #[test]
    fn pathological_inertia_does_not_produce_nan() {
        // Zero / NaN inertia → fall back to reference defaults
        let mut physics = PhysicsConfig::default();
        physics.inertia = [0.0, f64::NAN, -1.0];
        let pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        assert!(pids.roll_p.is_finite() && pids.roll_p > 0.0);
        assert!(pids.pitch_p.is_finite() && pids.pitch_p > 0.0);
        assert!(pids.yaw_p.is_finite() && pids.yaw_p > 0.0);
    }

    #[test]
    fn high_twr_attenuates_gains() {
        let physics = config_with_inertia(REF_INERTIA);
        let ref_pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // TWR=7.2 → hover_cmd=0.14
        let high_twr_pids = compute_pids(&physics, 0.14);
        // authority_scale = braking_ratio (no sqrt)
        // braking_ratio = (0.14/0.86) / (0.28/0.72) = 0.1628 / 0.3889 = 0.4186
        let ref_braking = REF_HOVER_CMD / (1.0 - REF_HOVER_CMD);
        let build_braking = 0.14 / (1.0 - 0.14);
        let expected_scale = build_braking / ref_braking;
        let actual_scale = high_twr_pids.roll_p as f64 / ref_pids.roll_p as f64;
        assert!(
            (actual_scale - expected_scale).abs() < 0.01,
            "expected scale {:.4}, got {:.4}", expected_scale, actual_scale
        );
        // D-term also attenuated
        let d_scale = high_twr_pids.pitch_d as f64 / ref_pids.pitch_d as f64;
        assert!((d_scale - expected_scale).abs() < 0.01);
        // FF not attenuated
        assert_eq!(high_twr_pids.roll_ff, ref_pids.roll_ff);
    }

    #[test]
    fn low_twr_does_not_boost_gains() {
        let physics = config_with_inertia(REF_INERTIA);
        let ref_pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // TWR=2 → hover_cmd=0.50 (above reference)
        let low_twr_pids = compute_pids(&physics, 0.50);
        // braking_ratio = (0.50/0.50) / (0.28/0.72) > 1.0 → clamped to 1.0
        assert_eq!(low_twr_pids.roll_p, ref_pids.roll_p);
        assert_eq!(low_twr_pids.pitch_d, ref_pids.pitch_d);
    }

    #[test]
    fn extreme_twr_attenuates_aggressively() {
        let physics = config_with_inertia(REF_INERTIA);
        let ref_pids = compute_pids(&physics, REF_HOVER_CMD as f32);
        // TWR~10 → hover_cmd=0.12 (extreme racing build)
        let extreme_pids = compute_pids(&physics, 0.12);
        let ref_braking = REF_HOVER_CMD / (1.0 - REF_HOVER_CMD);
        let build_braking = 0.12 / (1.0 - 0.12);
        let expected_scale = build_braking / ref_braking;
        let actual_scale = extreme_pids.roll_p as f64 / ref_pids.roll_p as f64;
        // Expected: (0.12/0.88)/(0.28/0.72) ≈ 0.351
        assert!(
            expected_scale < 0.36,
            "extreme TWR should cut gains aggressively: {:.4}", expected_scale
        );
        assert!(
            (actual_scale - expected_scale).abs() < 0.01,
            "expected {:.4}, got {:.4}", expected_scale, actual_scale
        );
    }
}
