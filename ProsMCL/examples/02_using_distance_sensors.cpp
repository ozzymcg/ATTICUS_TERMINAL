// 02_using_distance_sensors.cpp

// Vision-assisted autonomous example.
// This is the practical pattern when you have:
// - wheel odometry + IMU
// - distance sensors for semi-absolute anchoring
// - optional vision pose fixes for long-run drift cleanup

// -------------------------- REQUIRED USER INPUTS --------------------------
static constexpr int IMU_PORT = 10;            // Change to your IMU port.
static constexpr double TRACK_WIDTH_IN = 12.0; // Change to measured track width in inches.
// REQUIRED: set your real distance sensor ports in configured order.
static const std::vector<int> DIST_PORTS = { 1, 2, 3 };

ProsMCL localizer(IMU_PORT, DIST_PORTS);

static double prev_left_in = 0.0;
static double prev_right_in = 0.0;

// REQUIRED: replace with your real encoder-to-inch conversions.
static double read_left_inches() { return 0.0; }
static double read_right_inches() { return 0.0; }

// REQUIRED: replace with your real drivetrain command.
static void set_drive_arcade_mv(int forward_mv, int turn_mv) {
  (void)forward_mv;
  (void)turn_mv;
}

// Vision pose adapter:
// Return true only when your camera-based pose is trustworthy.
// confidence should be 0..1 (1 = highest trust).
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

  // Differential drive robot-frame deltas expected by runtime.
  // NOTE: for standard encoder signs (both sides forward-positive), CW+ is (dL-dR).
  const double dx_in = 0.5 * (dL + dR);
  const double dy_in = 0.0;
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / M_PI;
  localizer.setOdomDelta(dx_in, dy_in, dtheta_deg_cw);
}

void initialize() {
  pros::lcd::initialize();

  // Start runtime once and set known start pose.
  // Edit these values to your actual start tile.
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);

  prev_left_in = read_left_inches();
  prev_right_in = read_right_inches();
}

void autonomous() {
  // Autonomous runs until competition framework exits the mode.
  int stage = 0;
  while (pros::competition::is_autonomous()) {
    feed_manual_odom_once();

    // Optional vision update on trusted fixes.
    // Bad vision fixes can hurt, so gate tightly in your adapter.
    VisionFix fix{};
    if (read_vision_fix(&fix)) {
      localizer.updateVision(fix.x, fix.y, fix.theta, fix.confidence);
    }

    // raw = raw MCL estimate, fused = MCL+EKF output (use fused for control).
    const MCLPose raw = localizer.getPose();
    const MCLPose fused = localizer.getFusedPose();

    // Practical state machine example.
    // Replace with your own autonomous routine logic.
    if (stage == 0) {
      set_drive_arcade_mv(9000, 0);
      if (fused.x >= 24.0) stage = 1;
    } else if (stage == 1) {
      set_drive_arcade_mv(0, 6500);
      if (std::fabs(angle_error_deg(35.0, fused.theta)) <= 3.0) stage = 2;
    } else if (stage == 2) {
      set_drive_arcade_mv(8500, 0);
      if (fused.x >= 40.0) stage = 3;
    } else {
      set_drive_arcade_mv(0, 0);
    }

    // Optional safety hook after obvious displacement events.
    // if (/* detected major collision / human touch */) localizer.requestRelocalize();

    pros::lcd::print(0, "AUTO raw  x=%.1f y=%.1f th=%.1f", raw.x, raw.y, raw.theta);
    pros::lcd::print(1, "AUTO fuse x=%.1f y=%.1f th=%.1f", fused.x, fused.y, fused.theta);
    pros::delay(10);
  }
  set_drive_arcade_mv(0, 0);
}

void opcontrol() {}
