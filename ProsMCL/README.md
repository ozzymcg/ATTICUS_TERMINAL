# Atticus MCL + EKF Export

This package is a practical localization stack for VEX/PROS:
- `MCL` (particle filter) anchors pose using map geometry + semi-absolute sensors.
- `EKF` smooths/filters pose updates for control loops.
- `ProsMCL` runtime provides a single API for odometry deltas, sensor updates, and fused pose output.

## Generated outputs
- `include/`: generated headers (`mcl_config.hpp`, `mcl_localizer.h`, `mcl_runtime.h`, `mcl_map_data.h`)
- `src/`: generated implementation (`mcl_localizer.cpp`, `mcl_runtime.cpp`, `mcl_map_data.cpp`)
- `include/src` tuning helpers: `mcl_tune_packets.h`, `mcl_tune_log.*`, `mcl_tuning.*`
- `docs/`: setup, tuning, convention, and API references
- `examples/`: templates for minimal odom, distance sensors, and LemLib integration

## Current export profile
- Particle budget: `300` (`n_min=200`, `n_max=400`)
- Runtime cadence targets: motion `20 ms`, sensor `50 ms`
- Distance sensors: enabled (`3` configured)
- EKF: enabled

## Fast-start integration checklist
1. Create one global `ProsMCL` instance with your IMU + distance sensor ports.
2. In `initialize()`, call `startEasy(seed, initial_heading_deg, x, y, theta)` so IMU calibration does not consume autonomous time.
3. Choose one odometry feed mode:
   - manual: call `setOdomDelta(...)` in your 10 ms control loop
   - automatic: set `setFieldPoseProvider(...)` once (best for LemLib)
   Runtime updates happen in the built-in background task (no `update()` call needed).
   No global 15s timeout loop is required; competition mode exits autonomous automatically.
4. Use `getFusedPose()` for auton/pathing decisions (not raw odometry pose).
5. If using LemLib, keep LemLib as base odometry and periodically blend/correct using `applyOdomCorrection(...)`.

## Performance intent
- Tune drivetrain/pathing PID first; localization cannot compensate for unstable base motion.
- Keep odom delta updates frequent and low-latency.
- For long skills routes, avoid odom-only setups; include distance and/or vision corrections.
- Keep units consistent (inches, degrees) and sign conventions correct.
- Tune in this order: PID -> sign checks -> geometry -> motion noise -> sensor gating -> correction alpha.
