# API Reference: ProsMCL Runtime

This describes runtime methods in `mcl_runtime.h` and how to use them correctly.

For practical tuning order (including PID-first workflow), see `docs/CHECKLIST.md`.

## Constructor
### `ProsMCL(int imu_port, const std::vector<int>& dist_ports)`
- `imu_port`: V5 IMU smart port.
- `dist_ports`: distance sensor smart ports in the same order as your configured distance sensor list.
- Keep this as a long-lived singleton (global/static), not recreated in loops.

## Lifecycle
### `start(unsigned seed, double initial_heading_deg)`
- Starts background update task and calibrates/uses IMU heading.
- Prefer calling once in `initialize()` so IMU calibration does not consume autonomous time.
- `seed`: deterministic test runs use a fixed value; comp runs can use `pros::millis()`.
- `initial_heading_deg`: Atticus heading at robot boot pose.
- If started with global initialization, EKF remains unlocked until MCL confidence passes threshold (or you call `setPose`).

### `startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg)`
- One-call startup for teams: starts runtime and sets known start pose.
- Recommended default entry point for most users.

### `startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg)`
- Same as `startEasy`, but inputs are interpreted in external convention (`mcl.interop`).
- Recommended for conventional VEX `0 deg = forward` integrations.

### `startExternal(unsigned seed, double initial_heading_deg)`
- Starts runtime where the initial heading is interpreted in external convention.
- Useful when your odom stack already works in external frame and you want consistent startup semantics.

### `stop()`
- Stops runtime task cleanly (usually optional for normal PROS lifecycle).

## State input
### `setPose(double x_in, double y_in, double theta_deg)`
- Re-initializes the particle cloud around the pose in field frame (not an exact hard lock unless set-pose sigmas are 0).
- Use at init, explicit re-localization, or known reset points.

### `setPoseExternal(double x_in, double y_in, double theta_deg)`
- Same as `setPose`, but input pose is external-frame and converted by `mcl.interop`.

### `setOdomDelta(double dx_in, double dy_in, double dtheta_deg)`
- Robot-frame motion delta since the previous call.
- `dx_in`: forward inches; `dy_in`: left inches; `dtheta_deg`: clockwise degrees.
- Feed every control cycle (~10 ms) in manual mode.
- Ignored while `setFieldPoseProvider(...)` mode is active.
- Prefer encoder-derived `dtheta`; runtime already incorporates IMU heading updates.

### `setFieldPoseProvider(FieldPoseProvider provider, void* user)` / `clearFieldPoseProvider()`
- Auto-feed mode: provide field pose samples and runtime computes deltas internally.
- Best low-code path for most VEX teams using existing odometry frameworks.
- Callback output must be internal MCL frame; for external library poses call `ProsMCL::externalPoseToMCL(...)` in your provider.
- `mcl.interop` controls those external conversion helpers (`pose_convention`, `swap_xy`, `invert_x`, `invert_y`).
- If provider heading already uses IMU, set `mcl.ekf.use_imu_update=0` to avoid double-counting heading.

### `runFrameSanityCheck(start, after_forward, after_turn, forward_target_in, turn_target_deg)`
- Static helper that scores current adapter settings and suggests better ones.
- Use after a quick on-robot test: set known start pose with heading near 0 external, drive forward ~12 in, then turn CW ~90 deg.
- Inputs are external-frame poses (same convention as your provider/library).
- Returns pass/fail for current mapping plus recommended `pose_convention`/axis flags.

### `updateVision(double x_in, double y_in, double theta_deg, double confidence)`
- Optional external absolute pose measurement (vision/camera/landmark).
- `confidence` is [0..1] and should reflect measurement trust.

### `setSegmentBand(const MCLPose* pts, int n, double radius_in)` / `clearSegmentBand()`
- Optional temporary corridor constraint (for known route segments).
- Use sparingly; overly tight bands can block recovery from collisions.

### `freezeLocalization(bool freeze)` / `isLocalizationFrozen()`
- Debug control: when frozen, runtime still updates baselines/timestamps but does not update particles/EKF.
- Intended for deterministic troubleshooting without injecting stale odom deltas on unfreeze.

## State output
- Runtime updates happen automatically in the internal task.
### `getPose()` vs `getFusedPose()`
- `getPose()`: raw MCL estimate.
- `getFusedPose()`: output after EKF smoothing/blending (usually best for control).

### `getPoseExternal()` / `getFusedPoseExternal()`
- Same pose outputs mapped into external convention (`mcl.interop`).
- Recommended for direct drop-in with LemLib/JAR-style external field frames.

### `applyOdomCorrection(MCLPose& odom_pose, double* out_alpha)`
- Blends odom pose toward fused pose when EKF is enabled; otherwise toward raw MCL pose.
- Returns `true` if correction is applied.
- Safe-window guard can block correction during aggressive motion (`safe_max_speed_in_s`, `safe_max_turn_deg_s`).
- Use in LemLib bridging loops to gently remove drift instead of hard snaps.

### `getDebugSnapshot()`
- Returns a lock-safe telemetry snapshot for tuning/logging.
- Includes odom/MCL/fused poses, EKF covariance terms, peakedness confidence, ESS ratio, Neff, and distance expected/measured arrays.

### `requestRelocalize()`
- Requests runtime to re-localize using configured strategy.
- Use when the robot was physically moved unexpectedly.

## Configuration map (what major params mean)
### Particle settings (`mcl.particles`)
- `n`: nominal particle count.
- `n_min`, `n_max`: adaptive bounds (used with KLD).
- Higher counts improve global robustness but cost CPU.

### Motion model (`mcl.motion`)
- `sigma_x_in`, `sigma_y_in`, `sigma_theta_deg`: additive noise scale.
- `use_alpha_model`, `alpha1..alpha4`: odom-motion dependent noise model.
- `enabled`: allows turning motion model on/off (normally on).
- `motion_model` / `motion_source` are normalized to `drive` / `encoders` for deterministic runtime behavior.

### Distance sensor model (`mcl.sensors.distance`)
- `enabled`: distance update active.
- `model`: `likelihood_field` (fast, robust default) or beam model.
- Multi-ray FOV is LF-only; beam mode is constrained to single-ray for bounded runtime.
- Core weighting knobs: `sigma_hit_mm`, `sigma_far_scale`, `sigma_min_mm`, `sigma_max_mm`, `conf_sigma_scale`, `min_sensor_weight`.
- Validity/gating knobs: `max_range_mm`, `min_range_mm`, `confidence_min`, `object_size_min/max`, `gate_mm`, `innovation_gate_mm`.
- Throughput knobs: `batch_size`, `model`, LF resolution, and `raycast_bucket_in` for beam expected-distance acceleration.
- `gate_mode`, `innovation_gate_mm`: reject outliers hard/soft.
- Per-sensor fields: `x_in`, `y_in`, `angle_deg`, `bias_mm`, range limits, object-size filters, map mode.

### IMU / Vision (`mcl.sensors.imu`, `mcl.sensors.vision`)
- IMU: heading noise and gating behavior.
- Vision: confidence threshold and influence of absolute fixes.

### Resampling (`mcl.resample`, `mcl.kld`, `mcl.augmented`)
- `method`: systematic/stratified/multinomial.
- `threshold`: effective sample size trigger.
- KLD (`epsilon`, `delta`, bin sizes): adaptive particle sizing.
- Injection policy is deterministic: augmented injection when `mcl.augmented.enabled=1`, otherwise fixed `random_injection`.
- Recovery LOST mode can force temporary injection (`lost_injection_fraction`) and exit after sustained confidence (`lost_exit_confidence`, `lost_exit_streak`).

### CGR-lite (`mcl.cgr_lite`)
- Optional bounded top-K local refinement on finalized posterior estimate.
- Budget controls: `top_k`, `max_iters`, `budget_ms`.
- Designed to be watchdog-safe: bounded iterations with per-call time cap.

### Region + confidence + correction + EKF
- `mcl.region`: hard/soft geographic constraints and penalties.
- `mcl.confidence`: thresholding and auto-reinit policy (metric is locked to `peakedness`).
- `mcl.correction`: odom blend safety limits and alpha bounds.
- `mcl.ekf`: smoothing noise, trust bounds, and initialization sigmas.

## Tuning helper APIs (microSD workflow)
- `mcl_tune_packets.h`: binary log header/frame/footer format.
- `MCLTuneLogger` (`mcl_tune_log.h`): background snapshot logger to `/usd/...mcllog`.
- `MCLTuningWizard` (`mcl_tuning.h`): simple step-marker wrapper for tuning sessions.
