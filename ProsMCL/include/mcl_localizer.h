#pragma once
#include "mcl_config.hpp"
#include <cstdint>
#include <vector>

// Field-centric coordinates: +x forward, +y left; heading 0=left, 90=forward (clockwise).
struct MCLPose {
  double x;
  double y;
  double theta;
};

class MCLLocalizer {
 public:
  MCLLocalizer();
  void seed(unsigned int seed);
  void init(double start_x, double start_y, double start_theta);
  void initGlobal();
  void predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale = 1.0);
  void updateDistance(const double* dist_mm, const double* dist_conf, const int* conf_meaningful, const double* dist_obj_size, const int* obj_size_valid, const int* dist_errno, int count);
  void updateIMU(double heading_deg);
  void updateVision(double x_in, double y_in, double theta_deg, double confidence);
  void estimateCovariance(MCLPose* pose_out, double cov_out[3][3]);
  void copyWeights(double* out_weights, int capacity) const;
  void setWeights(const double* in_weights, int capacity);
  void setForcedInjectionFraction(double frac);
  double forcedInjectionFraction() const { return forced_injection_fraction_; }
  void setSegmentBand(const MCLPose* pts, int n, double radius_in);
  void clearSegmentBand();
  void normalize();
  void resample();
  MCLPose estimate();
  // Localization peakedness: 0 = uniform weights, 1 = highly peaked weights.
  double confidence() const { return confidence_; }
  // ESS ratio: 1 = uniform weights, 0 = maximally concentrated.
  double essRatio() const { return ess_ratio_; }
  double neff() const { return effectiveN(); }
  int particleCount() const { return count_; }
  void getLastDistanceDebug(double* measured_mm, double* expected_mm, int* errno_codes, int capacity, uint32_t* used_mask) const;

 private:
  struct Particle {
    double x;
    double y;
    double theta;
    double w;
  };
  Particle particles_[MCL_PARTICLE_CAPACITY];
  Particle resample_buf_[MCL_PARTICLE_CAPACITY];
  double tmp_weights_[MCL_PARTICLE_CAPACITY];
  double cdf_buf_[MCL_PARTICLE_CAPACITY];
  uint64_t bin_buf_[MCL_PARTICLE_CAPACITY];
  char skip_sensor_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int count_;
  double w_slow_;
  double w_fast_;
  double forced_injection_fraction_;
  double confidence_;
  double ess_ratio_;
  MCLPose estimate_;
  bool estimate_valid_;
  bool have_estimate_ever_;
  double last_dist_measured_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  double last_dist_expected_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  int last_dist_errno_[MCL_DISTANCE_SENSOR_COUNT_SAFE];
  uint32_t last_dist_used_mask_;
  std::vector<MCLPose> segment_band_pts_;
  double segment_band_radius_;
  bool segment_band_active_;
  double regionWeight(double x, double y, double heading_deg) const;
  void cgrLiteRefineEstimate();
  double effectiveN() const;
};
