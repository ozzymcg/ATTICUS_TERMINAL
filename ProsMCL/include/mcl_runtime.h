#pragma once
#include "api.h"
#include "mcl_localizer.h"
#include <vector>
#include <atomic>
#include <cstdint>
#include <memory>

// Internal MCL frame: +x forward, +y left; heading CW+ with 0=left, 90=forward.
// Use external* adapters below for exported conventions (e.g., 0=forward).
class ProsMCL {
 public:
  using FieldPoseProvider = bool (*)(MCLPose* out_pose, void* user);
  ProsMCL(int imu_port, const std::vector<int>& dist_ports);
  void start(unsigned seed, double initial_heading_deg);
  void startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg);
  void stop();
  // Robot-frame deltas: +x forward, +y left, +theta clockwise (deg).
  // Do not pass field-frame deltas here.
  // Ignored while a FieldPoseProvider is active (feed modes are exclusive).
  void setOdomDelta(double dx_in, double dy_in, double dtheta_deg);
  // Optional: provide field pose samples and let runtime compute odom deltas automatically.
  // Callback must return internal MCL frame pose (use externalPoseToMCL for library poses).
  void setFieldPoseProvider(FieldPoseProvider provider, void* user = nullptr);
  void clearFieldPoseProvider();
  void setPose(double x_in, double y_in, double theta_deg);
  // External convention adapters (selected by mcl.interop.pose_convention).
  // - atticus: heading CW+, 0=left, 90=forward
  // - cw_zero_forward: heading CW+, 0=forward
  // - ccw_zero_forward: heading CCW+, 0=forward
  static double externalHeadingToMCL(double heading_deg);
  static double mclHeadingToExternal(double heading_deg);
  static MCLPose externalPoseToMCL(const MCLPose& pose);
  static MCLPose mclPoseToExternal(const MCLPose& pose);
  struct FrameSanityResult {
    bool current_pass;
    bool translation_pass;
    bool heading_pass;
    double current_dx_in;
    double current_dy_in;
    double current_dtheta_deg;
    int recommended_pose_convention;
    int recommended_swap_xy;
    int recommended_invert_x;
    int recommended_invert_y;
    double recommended_score;
  };
  // Evaluate adapter settings from a simple on-robot test:
  // start(heading ~0 external) -> drive forward -> turn clockwise.
  // Inputs are external-frame poses.
  static FrameSanityResult runFrameSanityCheck(
      const MCLPose& start_external,
      const MCLPose& after_forward_external,
      const MCLPose& after_turn_external,
      double forward_target_in = 12.0,
      double turn_target_deg = 90.0);
  void startExternal(unsigned seed, double initial_heading_deg);
  void startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg);
  void setPoseExternal(double x_in, double y_in, double theta_deg);
  MCLPose getPoseExternal() const;
  MCLPose getFusedPoseExternal() const;
  void set_pose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }
  void resetPose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }
  void reset_pose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }
  void kidnap() { requestRelocalize(); }
  MCLPose getPose() const;
  MCLPose getFusedPose() const;
  void freezeLocalization(bool freeze);
  bool isLocalizationFrozen() const;
  void updateVision(double x_in, double y_in, double theta_deg, double confidence);
  void setSegmentBand(const MCLPose* pts, int n, double radius_in);
  void clearSegmentBand();
  bool applyOdomCorrection(MCLPose& odom_pose, double* out_alpha = nullptr) const;
  void requestRelocalize();
  struct DebugSnapshot {
    std::uint32_t time_ms;
    MCLPose odom_pose;
    MCLPose mcl_pose;
    MCLPose fused_pose;
    double mcl_confidence;
    double mcl_peakedness;
    double mcl_ess_ratio;
    double mcl_neff;
    double ekf_pxx;
    double ekf_pyy;
    double ekf_pxy;
    double ekf_ptt;
    std::uint32_t dist_used_mask;
    std::uint32_t event_flags;
    double dist_meas_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    double dist_exp_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    int dist_errno[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  };
  DebugSnapshot getDebugSnapshot() const;
  // Exposed so helper math functions in runtime can use the type without access issues.
  struct EKFState {
    double x;
    double y;
    double theta;
    double P[3][3];
    bool initialized;
  };

 private:
  void loop();

  std::unique_ptr<pros::Imu> imu_;
  std::vector<pros::Distance> dists_;
  MCLLocalizer mcl_;

  mutable pros::Mutex mu_;
  pros::Task* task_;
  std::atomic_bool running_;
  std::atomic_bool task_done_;
  bool map_blob_valid_;
  std::atomic_bool relocalize_requested_;
  std::atomic_bool localization_frozen_;
  FieldPoseProvider pose_provider_;
  void* pose_provider_user_;
  MCLPose provider_last_pose_;
  bool provider_last_valid_;
  double odom_dx_;
  double odom_dy_;
  double odom_dth_;
  int odom_fault_noise_cycles_;
  double last_speed_in_s_;
  double last_turn_deg_s_;
  double imu_last_rotation_;
  bool imu_last_rotation_valid_;
  MCLPose pose_;
  MCLPose fused_pose_;
  MCLPose pose_buf_[2];
  MCLPose fused_pose_buf_[2];
  std::atomic_int pose_buf_idx_;
  EKFState ekf_;
  bool mcl_ambiguous_;
  int mcl_recover_good_count_;
  int recovery_low_ess_streak_;
  int recovery_gate_reject_streak_;
  std::uint32_t recovery_cooldown_until_ms_;
  bool recovery_lost_;
  int recovery_exit_streak_;
  std::uint32_t recovery_lost_since_ms_;
  int sign_selftest_samples_;
  int sign_selftest_mismatch_;
  bool sign_selftest_done_;
  std::uint32_t ekf_intervene_cooldown_until_ms_;
  mutable std::uint32_t last_event_flags_;
  bool sensor_batch_active_;
  int sensor_batch_count_;
  int sensor_batch_cursor_;
  bool sensor_batch_have_imu_heading_;
  double sensor_batch_imu_heading_;
  bool sensor_batch_imu_applied_;
  double sensor_batch_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  double sensor_batch_conf_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  double sensor_batch_obj_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int sensor_batch_errno_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int sensor_batch_conf_meaningful_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int sensor_batch_obj_valid_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  double sensor_batch_weights_backup_[MCL_PARTICLE_CAPACITY];
  static constexpr int DIST_MEDIAN_WINDOW = (MCL_DIST_MEDIAN_WINDOW > 1 ? MCL_DIST_MEDIAN_WINDOW : 1);
  double dist_hist_[MCL_DISTANCE_SENSOR_COUNT_SAFE][DIST_MEDIAN_WINDOW];
  int dist_hist_count_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int dist_hist_idx_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
};
