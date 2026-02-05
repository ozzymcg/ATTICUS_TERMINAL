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

### `stop()`
- Stops runtime task cleanly (usually optional for normal PROS lifecycle).

## State input
### `setPose(double x_in, double y_in, double theta_deg)`
- Re-initializes the particle cloud around the pose in field frame (not an exact hard lock unless set-pose sigmas are 0).
- Use at init, explicit re-localization, or known reset points.

### `setOdomDelta(double dx_in, double dy_in, double dtheta_deg)`
- Robot-frame motion delta since the previous call.
- `dx_in`: forward inches; `dy_in`: left inches; `dtheta_deg`: clockwise degrees.
- Feed every control cycle (~10 ms) in manual mode.
- Prefer encoder-derived `dtheta`; runtime already incorporates IMU heading updates.

### `setFieldPoseProvider(FieldPoseProvider provider, void* user)` / `clearFieldPoseProvider()`
- Auto-feed mode: provide field pose samples (e.g., LemLib `chassis.getPose`) and runtime computes deltas internally.
- Best low-code path for most VEX teams using existing odometry frameworks.
- Add an explicit pose adapter if your provider convention differs from MCL (`+x forward, +y left, CW+ heading`).
- If provider heading already uses IMU, set `mcl.ekf.use_imu_update=0` to avoid double-counting heading.

### `updateVision(double x_in, double y_in, double theta_deg, double confidence)`
- Optional external absolute pose measurement (vision/camera/landmark).
- `confidence` is [0..1] and should reflect measurement trust.

### `setSegmentBand(const MCLPose* pts, int n, double radius_in)` / `clearSegmentBand()`
- Optional temporary corridor constraint (for known route segments).
- Use sparingly; overly tight bands can block recovery from collisions.

## State output
- Runtime updates happen automatically in the internal task.
### `getPose()` vs `getFusedPose()`
- `getPose()`: raw MCL estimate.
- `getFusedPose()`: output after EKF smoothing/blending (usually best for control).

### `applyOdomCorrection(MCLPose& odom_pose, double* out_alpha)`
- Blends odom pose toward fused pose when EKF is enabled; otherwise toward raw MCL pose.
- Returns `true` if correction is applied.
- Use in LemLib bridging loops to gently remove drift instead of hard snaps.

### `getDebugSnapshot()`
- Returns a lock-safe telemetry snapshot for tuning/logging.
- Includes odom/MCL/fused poses, EKF covariance terms, confidence/Neff, and distance expected/measured arrays.

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

### Distance sensor model (`mcl.sensors.distance`)
- `enabled`: distance update active.
- `model`: `likelihood_field` (fast, robust default) or beam model.
- `sigma_mm`, `max_range_mm`, `min_confidence`: measurement weighting/gating controls.
- `gate_mode`, `innovation_gate_mm`: reject outliers hard/soft.
- Per-sensor fields: `x_in`, `y_in`, `angle_deg`, `bias_mm`, range limits, object-size filters, map mode.

### IMU / Vision (`mcl.sensors.imu`, `mcl.sensors.vision`)
- IMU: heading noise and gating behavior.
- Vision: confidence threshold and influence of absolute fixes.

### Resampling (`mcl.resample`, `mcl.kld`, `mcl.augmented`)
- `method`: systematic/stratified/multinomial.
- `threshold`: effective sample size trigger.
- KLD (`epsilon`, `delta`, bin sizes): adaptive particle sizing.
- Augmented/random injection: recovery when filter degenerates.

### Region + confidence + correction + EKF
- `mcl.region`: hard/soft geographic constraints and penalties.
- `mcl.confidence`: thresholding and auto-reinit policy.
- `mcl.correction`: odom blend safety limits and alpha bounds.
- `mcl.ekf`: smoothing noise, trust bounds, and initialization sigmas.

## Tuning helper APIs (microSD workflow)
- `mcl_tune_packets.h`: binary log header/frame/footer format.
- `MCLTuneLogger` (`mcl_tune_log.h`): background snapshot logger to `/usd/...mcllog`.
- `MCLTuningWizard` (`mcl_tuning.h`): simple step-marker wrapper for tuning sessions.
