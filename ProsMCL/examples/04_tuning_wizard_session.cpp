// 04_tuning_wizard_session.cpp

// Completed no-USB tuning session example:
// - Logs compact telemetry to microSD (.mcllog)
// - Uses explicit wizard markers for Atticus Terminal analysis
// - Runs a full still + motion + kidnapped sequence
// - Feeds odometry through FieldPoseProvider so odom logs are meaningful
// - Requires mcl.tuning.enabled = 1 in config export

static constexpr int IMU_PORT = 10;
static constexpr int LEFT_DRIVE_PORT = 11;   // Update to your robot.
static constexpr int RIGHT_DRIVE_PORT = -12; // Negative reverses direction.
static constexpr double WHEEL_DIAMETER_IN = 2.75; // Update to your wheel diameter.
static constexpr double TRACK_WIDTH_IN = 12.0;    // Update to your measured track width.
static constexpr double kPi = 3.14159265358979323846;
static ProsMCL localizer(IMU_PORT, { 1, 2, 3 });
static MCLTuningWizard tuning(&localizer);
static pros::Motor left_drive(LEFT_DRIVE_PORT);
static pros::Motor right_drive(RIGHT_DRIVE_PORT);
static MCLPose odom_pose{0.0, 0.0, 90.0};
static double odom_prev_left_deg = 0.0;
static double odom_prev_right_deg = 0.0;
static bool odom_seeded = false;

static double wrap_deg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0) deg += 360.0;
  return deg;
}

static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {
  const double th = heading_deg * (kPi / 180.0);
  const double s = std::sin(th);
  const double c = std::cos(th);
  wx = lx * s - ly * c;
  wy = lx * c + ly * s;
}

static double motor_deg_to_in(double deg) {
  return (deg / 360.0) * (kPi * WHEEL_DIAMETER_IN);
}

static void reset_odom_seed(double x_in, double y_in, double theta_deg) {
  odom_pose = {x_in, y_in, wrap_deg(theta_deg)};
  odom_prev_left_deg = left_drive.get_position();
  odom_prev_right_deg = right_drive.get_position();
  odom_seeded = true;
}

// Runtime provider used by ProsMCL:
// - gives field pose each motion tick
// - lets runtime compute odom deltas + log odom_pose in .mcllog
static bool diff_drive_pose_provider(MCLPose* out_pose, void*) {
  const double left_deg = left_drive.get_position();
  const double right_deg = right_drive.get_position();
  if (!odom_seeded) {
    odom_prev_left_deg = left_deg;
    odom_prev_right_deg = right_deg;
    odom_seeded = true;
  }
  const double dL_in = motor_deg_to_in(left_deg - odom_prev_left_deg);
  const double dR_in = motor_deg_to_in(right_deg - odom_prev_right_deg);
  odom_prev_left_deg = left_deg;
  odom_prev_right_deg = right_deg;

  const double dx_robot = 0.5 * (dL_in + dR_in);
  const double dtheta_deg = ((dL_in - dR_in) / TRACK_WIDTH_IN) * (180.0 / kPi);

  double wx = 0.0, wy = 0.0;
  rotate_local_to_world(dx_robot, 0.0, odom_pose.theta, wx, wy);
  odom_pose.x += wx;
  odom_pose.y += wy;
  odom_pose.theta = wrap_deg(odom_pose.theta + dtheta_deg);

  *out_pose = odom_pose;
  return true;
}

static void set_drive(int left, int right) {
  left_drive.move(left);
  right_drive.move(right);
}

static void stop_drive() {
  set_drive(0, 0);
}

static void drive_for(int left, int right, int ms) {
  set_drive(left, right);
  pros::delay(ms);
  stop_drive();
  pros::delay(200);
}

void initialize() {
  pros::lcd::initialize();
  left_drive.tare_position();
  right_drive.tare_position();
  reset_odom_seed(0.0, 0.0, 90.0);
  localizer.setFieldPoseProvider(diff_drive_pose_provider, nullptr);
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);
}

void autonomous() {
  if (!tuning.begin("skills")) {
    pros::lcd::print(0, "Tune start failed (SD?)");
    return;
  }

  // STEP 1: Mark A confirmation.
  tuning.markStep();
  pros::lcd::print(0, "STEP1 Mark A confirm");
  stop_drive();
  pros::delay(1000);

  // STEP 2: Stillness window (noise floor).
  tuning.markStep();
  pros::lcd::print(0, "STEP2 stillness 8s");
  pros::delay(8000);

  // STEP 3: Straight out/back profile.
  tuning.markStep();
  pros::lcd::print(0, "STEP3 straight out/back");
  drive_for(70, 70, 2000);
  drive_for(-70, -70, 2000);

  // STEP 4: In-place spin profile.
  tuning.markStep();
  pros::lcd::print(0, "STEP4 spin profile");
  drive_for(65, -65, 1700);
  drive_for(-65, 65, 1700);

  // STEP 5: Kidnapped recovery profile.
  tuning.markKidnappedStart();
  pros::lcd::print(0, "STEP5 Pick+place robot");
  stop_drive();
  pros::delay(2000);  // Operator relocates robot.
  tuning.markKidnappedPlaced();
  pros::lcd::print(0, "STEP5 recovery settle");
  pros::delay(5000);

  tuning.end();
  pros::lcd::print(1, "Saved: %s", tuning.filePath());
}

void opcontrol() {
  stop_drive();
}
