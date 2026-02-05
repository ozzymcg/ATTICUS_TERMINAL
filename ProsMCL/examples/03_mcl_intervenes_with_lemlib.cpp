// 03_mcl_intervenes_with_lemlib.cpp

// LemLib + MCL practical autonomous example (distance-sensor based).
// Flow:
// 1) Use LemLib as normal.
// 2) Set pose provider so MCL auto-feeds odometry from LemLib.
// 3) Start MCL once at autonomous start pose.
// 4) Run LemLib commands normally while a parallel correction task applies
//    small safe nudges back into LemLib pose.
//
// IMPORTANT: alpha comes from mcl.correction.alpha_min/max in config.
// Generated defaults are tuned conservative for skills:
//   alpha_min = 0.03, alpha_max = 0.12
// Put larger values only if your robot recovers too slowly and remains stable.
// In this example, correction runs in parallel with blocking LemLib commands.

extern lemlib::Chassis chassis;

static constexpr int IMU_PORT = 10; // Change to your IMU smart port.
// REQUIRED: set your real distance sensor smart ports in configured order.
static ProsMCL localizer(IMU_PORT, { 1, 2, 3 });
static std::atomic_bool correction_running{false};
static pros::Task* correction_task = nullptr;
// Teams can toggle this to quickly enable/disable live tuning telemetry.
static constexpr bool MCL_DEBUG_TELEMETRY = true;

// Pose adapters:
// Edit these two helpers if your LemLib convention differs from MCL
// (+x forward, +y left, theta CW+ with 0=left, 90=forward).
// If LemLib already matches, leave identity mapping as-is.
static MCLPose lemlib_to_mcl(const lemlib::Pose& p) {
  return MCLPose{p.x, p.y, p.theta};
}

static void mcl_to_lemlib(const MCLPose& p, double& x, double& y, double& theta) {
  x = p.x;
  y = p.y;
  theta = p.theta;
}

// Input adapter: runtime reads LemLib pose through this callback.
static bool lemlib_pose_provider(MCLPose* out_pose, void*) {
  const auto p = chassis.getPose(false, false);
  *out_pose = lemlib_to_mcl(p);
  return true;
}

static double wrap_err_deg(double target_deg, double current_deg) {
  double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static void correction_task_fn(void*) {
  std::uint32_t last_debug_ms = 0;
  while (correction_running.load()) {
    const auto p = chassis.getPose(false, false);
    const MCLPose odom_raw = lemlib_to_mcl(p);

    // odom_pose is the current LemLib odom estimate.
    MCLPose odom_pose = odom_raw;

    // alpha = blend strength selected by MCL using confidence and safety gates:
    // - near alpha_min for small safe corrections
    // - toward alpha_max only when confidence is strong
    // - 0 when no correction is applied
    double alpha = 0.0;
    if (localizer.applyOdomCorrection(odom_pose, &alpha)) {
      // Push corrected pose back into LemLib so pathing uses the improved estimate.
      double sx = 0.0, sy = 0.0, sth = 0.0;
      mcl_to_lemlib(odom_pose, sx, sy, sth);
      chassis.setPose(sx, sy, sth);
    }

    const MCLPose fused = localizer.getFusedPose();
    if (MCL_DEBUG_TELEMETRY) {
      const std::uint32_t now = pros::millis();
      if (now - last_debug_ms >= 50) {
        last_debug_ms = now;
        const double ex = fused.x - odom_raw.x;
        const double ey = fused.y - odom_raw.y;
        const double eth = wrap_err_deg(fused.theta, odom_raw.theta);
        pros::lcd::print(0, "ODOM x=%.1f y=%.1f t=%.1f", odom_raw.x, odom_raw.y, odom_raw.theta);
        pros::lcd::print(1, "FUSE x=%.1f y=%.1f t=%.1f", fused.x, fused.y, fused.theta);
        pros::lcd::print(2, "ERR  dx=%.2f dy=%.2f dt=%.2f", ex, ey, eth);
        pros::lcd::print(3, "MCL alpha=%.2f", alpha);
        // CSV-style serial log for offline tuning review:
        // time_ms, odom_x, odom_y, odom_theta, fused_x, fused_y, fused_theta, err_x, err_y, err_theta, alpha
        std::printf("MCLDBG,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    (unsigned long)now, odom_raw.x, odom_raw.y, odom_raw.theta, fused.x, fused.y, fused.theta, ex, ey, eth, alpha);
      }
    }
    pros::delay(10);
  }
}

void initialize() {
  pros::lcd::initialize();
  // Start here so IMU calibration completes pre-autonomous.
  localizer.start((int)pros::millis(), 90.0);
}

void autonomous() {
  // Start pose from LemLib should match real robot placement before first move.
  const auto p0 = chassis.getPose(false, false);
  const MCLPose p0_mcl = lemlib_to_mcl(p0);

  // Auto-feed odometry from LemLib into MCL runtime.
  // If LemLib heading already comes from IMU, disable MCL/EKF IMU updates in config
  // to avoid double-counting heading.
  localizer.setFieldPoseProvider(lemlib_pose_provider, nullptr);

  // Runtime is already running from initialize(); just lock belief to true start pose.
  localizer.setPose(p0_mcl.x, p0_mcl.y, p0_mcl.theta);

  // Parallel correction task: practical for long skills routes.
  correction_running.store(true);
  correction_task = new pros::Task(correction_task_fn, nullptr, "MCL-Corr");

  // Normal blocking LemLib auton commands.
  // Timeout values are per-command safety bounds, not a full-auton timer.
  // Replace coordinates/headings with your route.
  chassis.moveToPoint(24.0, 0.0, 1200);
  chassis.turnToHeading(45.0, 700);
  chassis.moveToPoint(42.0, 18.0, 1500);

  // Clean task shutdown at end of autonomous.
  correction_running.store(false);
  pros::delay(30);
  delete correction_task;
  correction_task = nullptr;
}

void opcontrol() {
  if (correction_task != nullptr) {
    correction_running.store(false);
    pros::delay(30);
    delete correction_task;
    correction_task = nullptr;
  }
}
