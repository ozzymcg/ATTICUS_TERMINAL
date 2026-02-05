// 01_minimal_motor_encoders_imu.cpp

// Recommended baseline for accurate autonomous skills runs.
// Why this pattern is practical in VEX:
// - Keeps your drivetrain control loop simple.
// - Feeds odometry continuously.
// - Uses distance sensors as semi-absolute anchors to reduce long-run drift.
// - Lets runtime fuse MCL + EKF in the background task automatically.
// - Runs in normal autonomous flow (no custom 15s timer loop).

// -------------------------- REQUIRED USER INPUTS --------------------------
static constexpr int IMU_PORT = 10;               // Change to your IMU smart port.
static constexpr double TRACK_WIDTH_IN = 12.0;    // Change to your measured left-right tracking width.
// REQUIRED: keep this non-empty for real matches.
// Order must match mcl.distance_sensors order (front/left/right/etc in config).
// If you see placeholder ports here, replace them with your real smart ports.
static const std::vector<int> DIST_PORTS = { 1, 2, 3 };

ProsMCL localizer(IMU_PORT, DIST_PORTS);

// Odom cache for delta computation. Seed in initialize() from your real sensors.
static double prev_left_in = 0.0;
static double prev_right_in = 0.0;

// REQUIRED: replace with your real encoder-to-inch conversions.
static double read_left_inches() { return 0.0; }
static double read_right_inches() { return 0.0; }

// REQUIRED: replace with your drivetrain command call.
static void set_drive_arcade_mv(int forward_mv, int turn_mv) {
  (void)forward_mv;
  (void)turn_mv;
}

// Optional external absolute fix (camera/AprilTag/etc).
// Return true only when the fix is trustworthy.
struct VisionFix { double x; double y; double theta; double confidence; };
static bool read_vision_fix(VisionFix* out_fix) {
  (void)out_fix;
  return false;
}

static double angle_error_deg(double target_deg, double current_deg) {
  double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static void feed_manual_odom_once() {
  const double left_in = read_left_inches();
  const double right_in = read_right_inches();

  const double dL = left_in - prev_left_in;
  const double dR = right_in - prev_right_in;
  prev_left_in = left_in;
  prev_right_in = right_in;

  // Differential-drive robot-frame delta expected by ProsMCL.
  // NOTE: for standard encoder signs (both sides forward-positive), CW+ is (dL-dR).
  const double dx_in = 0.5 * (dL + dR);
  const double dy_in = 0.0;
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / M_PI;

  // Background runtime consumes these deltas; no explicit localizer.update() call is needed.
  localizer.setOdomDelta(dx_in, dy_in, dtheta_deg_cw);
}

void initialize() {
  pros::lcd::initialize();

  // One-call startup:
  // - starts runtime task
  // - sets known start pose
  //
  // REQUIRED INPUTS:
  // start_x_in, start_y_in, start_theta_deg must match your real starting tile.
  // Atticus convention: +x forward, +y left, theta CW+.
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);

  // Seed odom cache with current sensor readings to avoid first-loop jump.
  prev_left_in = read_left_inches();
  prev_right_in = read_right_inches();
}

void autonomous() {
  // No fixed 15s timeout loop needed; competition mode exits autonomous automatically.
  int stage = 0;
  while (pros::competition::is_autonomous()) {
    feed_manual_odom_once();

    // Optional vision update. Confidence is 0..1 (1 = highest trust).
    VisionFix fix{};
    if (read_vision_fix(&fix)) {
      localizer.updateVision(fix.x, fix.y, fix.theta, fix.confidence);
    }

    const MCLPose fused = localizer.getFusedPose();

    // Practical pose-driven autonomous state machine.
    // Replace thresholds and commands with your routine.
    if (stage == 0) {
      set_drive_arcade_mv(9000, 0);
      if (fused.x >= 24.0) stage = 1;
    } else if (stage == 1) {
      set_drive_arcade_mv(0, 7000);
      if (std::fabs(angle_error_deg(45.0, fused.theta)) <= 3.0) stage = 2;
    } else if (stage == 2) {
      set_drive_arcade_mv(8500, 0);
      if (fused.x >= 42.0) stage = 3;
    } else {
      set_drive_arcade_mv(0, 0);
    }

    pros::lcd::print(0, "AUTO fused x=%.1f y=%.1f th=%.1f", fused.x, fused.y, fused.theta);
    pros::delay(10);
  }
  set_drive_arcade_mv(0, 0);
}

void opcontrol() {}
