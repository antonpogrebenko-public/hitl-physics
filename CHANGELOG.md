# Changelog

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
