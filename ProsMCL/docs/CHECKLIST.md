# Practical Configuration + Usage Checklist

This is an exact tuning workflow for VEX teams. Run steps in order.

## 1) PID first (required before localization tuning)
1. Tune drive/turn/path PID with MCL corrections effectively off (`alpha_min=0`, `alpha_max=0`).
2. Pass criteria before moving on:
   - Drive 72 in straight, 5 reps: end error <= 2 in.
   - Turn 90 deg, 5 reps: end error <= 2 deg.
   - No sustained oscillation after settling.
3. If PID is unstable, stop here; MCL/EKF tuning will not be trustworthy.

## 2) Wiring + geometry calibration (hard requirements)
1. IMU: verify calibration is complete before first movement.
2. Distance sensors: verify port order matches `mcl.sensor_geometry.distance_sensors` exactly.
3. Odom signs: forward => `dx>0`, left => `dy>0`, right turn => `dtheta>0`.
   - Standard diff-drive encoder signs (both sides forward-positive): `dtheta_cw = (dL - dR) / track_width * 180/pi`.
4. Track width: run 360 deg in-place turn, compare measured vs expected heading.
   - If under-rotating, decrease track width by 2-5%.
   - If over-rotating, increase track width by 2-5%.
   - Repeat until 360 deg test error <= 3 deg.

## 3) Runtime integration baseline
1. Create one global `ProsMCL` instance.
2. Call `startEasy(...)` once with correct start tile pose.
3. Feed odom one way only:
   - Manual `setOdomDelta(...)` every 10 ms, or
   - LemLib provider via `setFieldPoseProvider(...)`.
   - If provider heading already comes from IMU, set `mcl.ekf.use_imu_update=0` to avoid IMU double-counting.
4. Use `getFusedPose()` for autonomous decisions.

## 4) Baseline values (good first pass)
- Particles: `n=300`, `n_min=200`, `n_max=500`.
- Motion: `sigma_x_in=0.12`, `sigma_y_in=0.12`, `sigma_theta_deg=1.0`.
- Distance: `sigma_hit_mm=8.5`, `gate_mm=120`, `innovation_gate_mm=0` (enable later after stable lock).
- Correction: `alpha_min=0.03`, `alpha_max=0.12`, `min_confidence=0.6`.

## 5) Exact tuning procedure
1. **Motion noise first** (vision off during this step):
   - Run a 30-45 second mixed route (straight + turns).
   - If estimate is too jittery, reduce `sigma_x/y` by 0.02 and `sigma_theta` by 0.2.
   - If estimate lags/sticks after collisions, increase `sigma_x/y` by 0.02 and `sigma_theta` by 0.2.
2. **Distance gating second**:
   - First tune with `innovation_gate_mm=0`.
   - If false wall snaps occur: reduce `gate_mm` by 15 mm.
   - If real readings are ignored: increase `gate_mm` by 15 mm.
   - After stable runs, enable `innovation_gate_mm` in 20 mm steps.
   - Keep `gate_mm` usually in 80-180 mm.
3. **Particle count third**:
   - If kidnapped recovery is slow, increase `n` by +50.
   - If CPU load is high, decrease `n` by -50.
   - Keep skills configs typically in 250-450.
4. **Correction alpha last**:
   - Start at `0.03/0.12`.
   - If recovery after drift is too slow, raise both by +0.01.
   - If oscillation appears, lower `alpha_max` by -0.02 first.
   - Keep `alpha_max <= 0.18` for most robots.
5. **Vision integration (optional final step)**:
   - Feed only trusted fixes (`confidence >= 0.7` recommended).
   - If vision causes jumps, raise confidence threshold or reduce vision update frequency.

## 6) Quick symptom map
- Mirrored/rotated map behavior: coordinate sign/convention mismatch.
- Fast heading drift: track width or `dtheta` sign error.
- Snapping to wrong geometry: sensor pose wrong or gates too loose.
- Slow recovery after bump: motion noise too low or alpha too conservative.
- Oscillation/over-correction: alpha too high or base PID too aggressive.

## 7) No-USB tuning wizard flow (microSD, idiot-proof)
### Before you run
1. Run regression checks: `python mod/mcl_regression_checks.py` (must pass).
2. Put a microSD card in the V5 Brain.
3. Enable tuning mode in config (`mcl.tuning.enabled=1`) and export/build.
4. Deploy `examples/04_tuning_wizard_session.cpp` (or same flow in your project).
5. Confirm robot starts at known pose and field is clear.

### Robot-side run steps
1. Start the routine, wait for IMU calibration to finish.
2. Call `tuning.begin("skills")` at routine start.
3. Mark checkpoints with:
   - `markStep()` at each major phase boundary.
   - `markKidnappedStart()` when you pick up/relocate robot.
   - `markKidnappedPlaced()` right after placing robot down.
4. Run at least: still phase, straight phase, turn phase, kidnapped phase.
5. End with `tuning.end()` and verify LCD prints saved `.mcllog` path.

### Import + apply in Atticus Terminal
1. Remove microSD and open Tuning panel.
2. Import latest `.mcllog` file.
3. Confirm checks show PASS (duration, step markers, sensor coverage).
4. Click Apply Recommended.
5. Re-export MCL package, rebuild, rerun same session.

### Acceptance targets (repeat until met)
- Kidnapped recovery <= 8.0 s.
- Distance dropout < 25%.
- No repeated wrong-wall snaps in route replay.
- Auton endpoints within your team tolerance on 3 back-to-back runs.
