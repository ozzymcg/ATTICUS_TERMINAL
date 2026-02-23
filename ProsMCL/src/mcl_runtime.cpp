#include "mcl_runtime.h"
#include "mcl_config.hpp"
#include "mcl_map_data.h"
#include <algorithm>
#include <cstdint>
#include <cerrno>
#include <cmath>

namespace {
constexpr std::uint32_t MCL_EVENT_MCL_EKF_APPLIED = 1u << 0;
constexpr std::uint32_t MCL_EVENT_ODOM_CORR_APPLIED = 1u << 1;
constexpr std::uint32_t MCL_EVENT_RECOVERY_ACTIVE = 1u << 3;
constexpr std::uint32_t MCL_EVENT_FRAME_SIGN_MISMATCH = 1u << 4;
constexpr std::uint32_t MCL_EVENT_MAP_BLOB_MISMATCH = 1u << 5;
struct ScopedMutex {
  explicit ScopedMutex(pros::Mutex& m) : m_(m) { m_.take(); }
  ~ScopedMutex() { m_.give(); }
  pros::Mutex& m_;
};

static double wrap_deg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0) deg += 360.0;
  return deg;
}

static bool map_blob_sanity_ok() {
  if (MCL_MAP_GRID_W <= 0 || MCL_MAP_GRID_H <= 0) return false;
  const long expected = static_cast<long>(MCL_MAP_GRID_W) * static_cast<long>(MCL_MAP_GRID_H);
  if (expected <= 0) return false;
  if (MCL_MAP_DIST_FIELD_LEN < expected) return false;
  if (MCL_EXPECTED_MAP_GRID_W > 0 && MCL_MAP_GRID_W != MCL_EXPECTED_MAP_GRID_W) return false;
  if (MCL_EXPECTED_MAP_GRID_H > 0 && MCL_MAP_GRID_H != MCL_EXPECTED_MAP_GRID_H) return false;
  if (MCL_EXPECTED_MAP_DIST_LEN > 0 && MCL_MAP_DIST_FIELD_LEN != MCL_EXPECTED_MAP_DIST_LEN) return false;
  if (MCL_EXPECTED_MAP_HASH32 != 0u && MCL_MAP_DATA_HASH32 != MCL_EXPECTED_MAP_HASH32) return false;
  if (MCL_EXPECTED_MAP_PROJECT_HASH32 != 0u && MCL_MAP_PROJECT_HASH32 != MCL_EXPECTED_MAP_PROJECT_HASH32) return false;
  return true;
}

static double external_heading_to_mcl_mode(double heading_deg, int pose_convention) {
  double h = wrap_deg(heading_deg);
  if (pose_convention == MCL_POSE_CONVENTION_CW_ZERO_FORWARD) {
    return wrap_deg(h + 90.0);
  }
  if (pose_convention == MCL_POSE_CONVENTION_CCW_ZERO_FORWARD) {
    return wrap_deg(90.0 - h);
  }
  return h;
}

static double mcl_heading_to_external_mode(double heading_deg, int pose_convention) {
  double h = wrap_deg(heading_deg);
  if (pose_convention == MCL_POSE_CONVENTION_CW_ZERO_FORWARD) {
    return wrap_deg(h - 90.0);
  }
  if (pose_convention == MCL_POSE_CONVENTION_CCW_ZERO_FORWARD) {
    return wrap_deg(90.0 - h);
  }
  return h;
}

static void heading_to_unit(double heading_deg, double* dx, double* dy) {
  double th = heading_deg * (3.14159265358979323846 / 180.0);
  if (dx) *dx = std::sin(th);
  if (dy) *dy = std::cos(th);
}

static double unit_to_heading(double dx, double dy) {
  if (!std::isfinite(dx) || !std::isfinite(dy)) return 0.0;
  if (std::fabs(dx) < 1e-12 && std::fabs(dy) < 1e-12) return 0.0;
  double h = std::atan2(dx, dy) * (180.0 / 3.14159265358979323846);
  return wrap_deg(h);
}

static void external_xy_to_mcl_mode(double ex, double ey, int swap_xy, int invert_x, int invert_y, double* mx, double* my) {
  double x = ex;
  double y = ey;
  if (swap_xy != 0) {
    double t = x;
    x = y;
    y = t;
  }
  if (invert_x != 0) x = -x;
  if (invert_y != 0) y = -y;
  if (mx) *mx = x;
  if (my) *my = y;
}

static void mcl_xy_to_external_mode(double mx, double my, int swap_xy, int invert_x, int invert_y, double* ex, double* ey) {
  double x = mx;
  double y = my;
  if (invert_y != 0) y = -y;
  if (invert_x != 0) x = -x;
  if (swap_xy != 0) {
    double t = x;
    x = y;
    y = t;
  }
  if (ex) *ex = x;
  if (ey) *ey = y;
}

static double external_heading_to_mcl_full(double heading_deg, int pose_convention, int swap_xy, int invert_x, int invert_y) {
  double hx = 0.0, hy = 0.0;
  heading_to_unit(external_heading_to_mcl_mode(heading_deg, pose_convention), &hx, &hy);
  external_xy_to_mcl_mode(hx, hy, swap_xy, invert_x, invert_y, &hx, &hy);
  return unit_to_heading(hx, hy);
}

static double mcl_heading_to_external_full(double heading_deg, int pose_convention, int swap_xy, int invert_x, int invert_y) {
  double hx = 0.0, hy = 0.0;
  heading_to_unit(heading_deg, &hx, &hy);
  mcl_xy_to_external_mode(hx, hy, swap_xy, invert_x, invert_y, &hx, &hy);
  return mcl_heading_to_external_mode(unit_to_heading(hx, hy), pose_convention);
}

static double external_heading_to_mcl(double heading_deg) {
  return external_heading_to_mcl_full(
      heading_deg,
      MCL_EXPORT_POSE_CONVENTION,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0);
}

static double mcl_heading_to_external(double heading_deg) {
  return mcl_heading_to_external_full(
      heading_deg,
      MCL_EXPORT_POSE_CONVENTION,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0);
}

static double angle_diff_deg(double a, double b) {
  double d = std::fmod(a - b + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static double clamp_sigma_pos(double v) {
  return std::max(1e-4, std::fabs(v));
}

static double clamp_sigma_ang(double v) {
  return std::max(1e-3, std::fabs(v));
}

static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {
  double th = heading_deg * (3.14159265358979323846 / 180.0);
  double s = std::sin(th);
  double c = std::cos(th);
  wx = lx * s - ly * c;
  wy = lx * c + ly * s;
}

static void rotate_world_to_local(double wx, double wy, double heading_deg, double& lx, double& ly) {
  double th = heading_deg * (3.14159265358979323846 / 180.0);
  double s = std::sin(th);
  double c = std::cos(th);
  lx = s * wx + c * wy;
  ly = -c * wx + s * wy;
}

template <typename DistanceT>
static auto distance_read_mm_impl(DistanceT& d, int) -> decltype(static_cast<int>(d.get())) {
  return d.get();
}

template <typename DistanceT>
static auto distance_read_mm_impl(DistanceT& d, long) -> decltype(static_cast<int>(d.get_distance())) {
  return d.get_distance();
}

static int distance_read_mm_impl(...) {
  return PROS_ERR;
}

static int distance_read_mm(pros::Distance& d) {
  return distance_read_mm_impl(d, 0);
}

static bool imu_read_heading_deg(pros::Imu& imu, double* out_heading) {
  if (!out_heading) return false;
  errno = 0;
  double h = static_cast<double>(imu.get_heading());
  if (h == PROS_ERR_F || !std::isfinite(h)) return false;
  *out_heading = wrap_deg(h);
  return true;
}

static bool imu_read_rotation_deg(pros::Imu& imu, double* out_rotation) {
  if (!out_rotation) return false;
  errno = 0;
  double rot = static_cast<double>(imu.get_rotation());
  if (rot == PROS_ERR_F || !std::isfinite(rot)) return false;
  *out_rotation = rot;
  return true;
}

template <typename ImuT>
static auto imu_is_calibrating_impl(ImuT& imu, int) -> decltype(static_cast<bool>(imu.is_calibrating())) {
  return imu.is_calibrating();
}

static bool imu_is_calibrating_impl(...) {
  return false;
}

static bool imu_is_calibrating(pros::Imu& imu) {
  if (!MCL_IMU_CHECK_CALIBRATING) return false;
  return imu_is_calibrating_impl(imu, 0);
}

static void mat3_mul(const double A[3][3], const double B[3][3], double out[3][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += A[i][k] * B[k][j];
      }
      out[i][j] = sum;
    }
  }
}

static void mat3_transpose(const double A[3][3], double out[3][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i][j] = A[j][i];
    }
  }
}

static bool mat3_inv(const double A[3][3], double out[3][3]) {
  double det =
      A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
      A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
      A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  if (std::fabs(det) < 1e-12) return false;
  double inv_det = 1.0 / det;
  out[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
  out[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
  out[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
  out[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
  out[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
  out[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
  out[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
  out[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
  out[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
  return true;
}

static void ekf_reset(ProsMCL::EKFState& ekf, double x, double y, double theta) {
  ekf.x = x;
  ekf.y = y;
  ekf.theta = wrap_deg(theta);
  double sxy = MCL_EKF_INIT_SIGMA_XY_IN;
  double sth = MCL_EKF_INIT_SIGMA_THETA_DEG;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ekf.P[i][j] = 0.0;
    }
  }
  ekf.P[0][0] = sxy * sxy;
  ekf.P[1][1] = sxy * sxy;
  ekf.P[2][2] = sth * sth;
  ekf.initialized = true;
}

static void ekf_predict(ProsMCL::EKFState& ekf, double dx_in, double dy_in, double dtheta_deg, double noise_scale) {
  if (!ekf.initialized) return;
  double ns = std::isfinite(noise_scale) ? std::fabs(noise_scale) : 1.0;
  if (ns < 1.0) ns = 1.0;
  const double kDegToRad = 3.14159265358979323846 / 180.0;
  double th = ekf.theta * kDegToRad;
  double s = std::sin(th);
  double c = std::cos(th);
  ekf.x += dx_in * s - dy_in * c;
  ekf.y += dx_in * c + dy_in * s;
  ekf.theta = wrap_deg(ekf.theta + dtheta_deg);
  double rad_per_deg = kDegToRad;
  double F[3][3] = {
    {1.0, 0.0, (dx_in * c + dy_in * s) * rad_per_deg},
    {0.0, 1.0, (-dx_in * s + dy_in * c) * rad_per_deg},
    {0.0, 0.0, 1.0}
  };
  double L[3][3] = {
    {s, -c, 0.0},
    {c,  s, 0.0},
    {0.0, 0.0, 1.0}
  };
  double qdx = clamp_sigma_pos(MCL_EKF_SIGMA_DX_IN);
  double qdy = clamp_sigma_pos(MCL_EKF_SIGMA_DY_IN);
  double qth = clamp_sigma_ang(MCL_EKF_SIGMA_DTHETA_DEG);
  double qscale = ns * ns;
  double Q[3][3] = {
    {qdx * qdx * qscale, 0.0, 0.0},
    {0.0, qdy * qdy * qscale, 0.0},
    {0.0, 0.0, qth * qth * qscale}
  };
  double FP[3][3];
  double Ft[3][3];
  double temp[3][3];
  mat3_mul(F, ekf.P, FP);
  mat3_transpose(F, Ft);
  mat3_mul(FP, Ft, temp);
  double LQ[3][3];
  double Lt[3][3];
  double LQLt[3][3];
  mat3_mul(L, Q, LQ);
  mat3_transpose(L, Lt);
  mat3_mul(LQ, Lt, LQLt);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ekf.P[i][j] = temp[i][j] + LQLt[i][j];
    }
  }
}

static void ekf_update_imu(ProsMCL::EKFState& ekf, double heading_deg) {
  if (!ekf.initialized) return;
  double nu = angle_diff_deg(heading_deg, ekf.theta);
  double imu_sigma = clamp_sigma_ang(MCL_EKF_IMU_SIGMA_DEG);
  double R = imu_sigma * imu_sigma;
  double S = ekf.P[2][2] + R;
  if (S < 1e-12) return;
  double K[3] = {ekf.P[0][2] / S, ekf.P[1][2] / S, ekf.P[2][2] / S};
  ekf.x += K[0] * nu;
  ekf.y += K[1] * nu;
  ekf.theta = wrap_deg(ekf.theta + K[2] * nu);
  double I_KH[3][3] = {
    {1.0, 0.0, -K[0]},
    {0.0, 1.0, -K[1]},
    {0.0, 0.0, 1.0 - K[2]}
  };
  double temp[3][3];
  double IKHt[3][3];
  mat3_mul(I_KH, ekf.P, temp);
  mat3_transpose(I_KH, IKHt);
  double Pnew[3][3];
  mat3_mul(temp, IKHt, Pnew);
  double KRKt[3][3] = {
    {K[0] * K[0] * R, K[0] * K[1] * R, K[0] * K[2] * R},
    {K[1] * K[0] * R, K[1] * K[1] * R, K[1] * K[2] * R},
    {K[2] * K[0] * R, K[2] * K[1] * R, K[2] * K[2] * R}
  };
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ekf.P[i][j] = Pnew[i][j] + KRKt[i][j];
    }
  }
}

static bool ekf_update_mcl(ProsMCL::EKFState& ekf, const MCLPose& pose, const double mcl_cov[3][3], double confidence, bool* out_gate_reject = nullptr) {
  if (out_gate_reject) *out_gate_reject = false;
  if (!ekf.initialized) return false;
  if (confidence < MCL_EKF_MCL_MIN_CONF) return false;
  double nu_xy = std::sqrt((pose.x - ekf.x) * (pose.x - ekf.x) + (pose.y - ekf.y) * (pose.y - ekf.y));
  double nu_th = std::fabs(angle_diff_deg(pose.theta, ekf.theta));
  if (MCL_EKF_MCL_INNOVATION_GATE_XY_IN > 0.0 && nu_xy > MCL_EKF_MCL_INNOVATION_GATE_XY_IN) return false;
  if (MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG > 0.0 && nu_th > MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG) return false;
  double sx = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[0][0] : 0.0));
  double sy = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[1][1] : 0.0));
  double sth = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[2][2] : 0.0));
  if (!std::isfinite(sx) || sx <= 0.0) sx = MCL_EKF_MCL_SIGMA_X_MAX_IN;
  if (!std::isfinite(sy) || sy <= 0.0) sy = MCL_EKF_MCL_SIGMA_Y_MAX_IN;
  if (!std::isfinite(sth) || sth <= 0.0) sth = MCL_EKF_MCL_SIGMA_THETA_MAX_DEG;
  if (sx < MCL_EKF_MCL_SIGMA_X_MIN_IN) sx = MCL_EKF_MCL_SIGMA_X_MIN_IN;
  if (sx > MCL_EKF_MCL_SIGMA_X_MAX_IN) sx = MCL_EKF_MCL_SIGMA_X_MAX_IN;
  if (sy < MCL_EKF_MCL_SIGMA_Y_MIN_IN) sy = MCL_EKF_MCL_SIGMA_Y_MIN_IN;
  if (sy > MCL_EKF_MCL_SIGMA_Y_MAX_IN) sy = MCL_EKF_MCL_SIGMA_Y_MAX_IN;
  if (sth < MCL_EKF_MCL_SIGMA_THETA_MIN_DEG) sth = MCL_EKF_MCL_SIGMA_THETA_MIN_DEG;
  if (sth > MCL_EKF_MCL_SIGMA_THETA_MAX_DEG) sth = MCL_EKF_MCL_SIGMA_THETA_MAX_DEG;
  sx = clamp_sigma_pos(sx);
  sy = clamp_sigma_pos(sy);
  sth = clamp_sigma_ang(sth);
  double R[3][3] = {
    {sx * sx, 0.0, 0.0},
    {0.0, sy * sy, 0.0},
    {0.0, 0.0, sth * sth}
  };
  double S[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      S[i][j] = ekf.P[i][j] + R[i][j];
    }
  }
  double S_inv[3][3];
  if (!mat3_inv(S, S_inv)) return false;
  double nu[3] = {pose.x - ekf.x, pose.y - ekf.y, angle_diff_deg(pose.theta, ekf.theta)};
  if (MCL_EKF_MCL_MAHALANOBIS_GATE > 0.0) {
    double md2 =
      nu[0] * (S_inv[0][0] * nu[0] + S_inv[0][1] * nu[1] + S_inv[0][2] * nu[2]) +
      nu[1] * (S_inv[1][0] * nu[0] + S_inv[1][1] * nu[1] + S_inv[1][2] * nu[2]) +
      nu[2] * (S_inv[2][0] * nu[0] + S_inv[2][1] * nu[1] + S_inv[2][2] * nu[2]);
    if (md2 > MCL_EKF_MCL_MAHALANOBIS_GATE) {
      if (out_gate_reject) *out_gate_reject = true;
      return false;
    }
  }
  double K[3][3];
  mat3_mul(ekf.P, S_inv, K);
  double dx = K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2];
  double dy = K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2];
  double dth = K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2];
  ekf.x += dx;
  ekf.y += dy;
  ekf.theta = wrap_deg(ekf.theta + dth);
  double I_K[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      I_K[i][j] = (i == j ? 1.0 : 0.0) - K[i][j];
    }
  }
  double temp[3][3];
  double IKt[3][3];
  mat3_mul(I_K, ekf.P, temp);
  mat3_transpose(I_K, IKt);
  double Pnew[3][3];
  mat3_mul(temp, IKt, Pnew);
  double KR[3][3];
  mat3_mul(K, R, KR);
  double Kt[3][3];
  mat3_transpose(K, Kt);
  double KRKt[3][3];
  mat3_mul(KR, Kt, KRKt);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ekf.P[i][j] = Pnew[i][j] + KRKt[i][j];
    }
  }
  return true;
}

static bool ekf_covariance_needs_intervention(const ProsMCL::EKFState& ekf) {
  if (!ekf.initialized) return false;
  if (MCL_EKF_INTERVENE_P_XY_IN2 > 0.0) {
    if (ekf.P[0][0] >= MCL_EKF_INTERVENE_P_XY_IN2 || ekf.P[1][1] >= MCL_EKF_INTERVENE_P_XY_IN2) return true;
  }
  if (MCL_EKF_INTERVENE_P_THETA_DEG2 > 0.0 && ekf.P[2][2] >= MCL_EKF_INTERVENE_P_THETA_DEG2) return true;
  return false;
}

static bool ekf_fusion_update(ProsMCL::EKFState& ekf, const MCLPose& pose, const double mcl_cov[3][3], double conf, std::uint32_t now_ms,
                              bool& mcl_ambiguous, int& mcl_recover_good_count,
                              std::uint32_t& cooldown_until_ms, bool* out_gate_reject = nullptr) {
  if (out_gate_reject) *out_gate_reject = false;
  bool was_ambiguous = mcl_ambiguous;
  if (MCL_EKF_AMBIGUOUS_CONF > 0.0 && conf < MCL_EKF_AMBIGUOUS_CONF) {
    mcl_ambiguous = true;
    mcl_recover_good_count = 0;
  }
  if (mcl_ambiguous) {
    if (conf >= MCL_EKF_RECOVER_CONF) {
      if (mcl_recover_good_count < MCL_EKF_RECOVER_STABLE_UPDATES) mcl_recover_good_count++;
      if (mcl_recover_good_count >= MCL_EKF_RECOVER_STABLE_UPDATES) mcl_ambiguous = false;
    } else {
      mcl_recover_good_count = 0;
    }
  }
  bool recovered_now = was_ambiguous && !mcl_ambiguous;
  if (!ekf.initialized) {
    if (!mcl_ambiguous && conf >= MCL_EKF_MCL_MIN_CONF) {
      ekf_reset(ekf, pose.x, pose.y, pose.theta);
    }
    return false;
  }
  if (mcl_ambiguous) return false;
  double nu_xy = std::sqrt((pose.x - ekf.x) * (pose.x - ekf.x) + (pose.y - ekf.y) * (pose.y - ekf.y));
  double nu_th = std::fabs(angle_diff_deg(pose.theta, ekf.theta));
  bool cooldown_active = now_ms < cooldown_until_ms;
  bool intervene_delta = false;
  if (MCL_EKF_INTERVENE_TRANS_IN > 0.0 && nu_xy >= MCL_EKF_INTERVENE_TRANS_IN) intervene_delta = true;
  if (MCL_EKF_INTERVENE_THETA_DEG > 0.0 && nu_th >= MCL_EKF_INTERVENE_THETA_DEG) intervene_delta = true;
  bool intervene = recovered_now || intervene_delta || ekf_covariance_needs_intervention(ekf);
  bool track_ok = MCL_EKF_TRACK_CORRECTION_ENABLED && conf >= MCL_EKF_TRACK_MIN_CONF;
  if (track_ok && MCL_EKF_TRACK_MAX_TRANS_IN > 0.0 && nu_xy > MCL_EKF_TRACK_MAX_TRANS_IN) track_ok = false;
  if (track_ok && MCL_EKF_TRACK_MAX_THETA_DEG > 0.0 && nu_th > MCL_EKF_TRACK_MAX_THETA_DEG) track_ok = false;
  bool intervene_ok = intervene && !cooldown_active && conf >= MCL_EKF_INTERVENE_MIN_CONF;
  bool use_reset = recovered_now ? MCL_EKF_RECOVER_USE_RESET : MCL_EKF_INTERVENE_USE_RESET;
  bool hard_jump = false;
  if (MCL_EKF_INTERVENE_HARD_RESET_TRANS_IN > 0.0 && nu_xy >= MCL_EKF_INTERVENE_HARD_RESET_TRANS_IN) hard_jump = true;
  if (MCL_EKF_INTERVENE_HARD_RESET_THETA_DEG > 0.0 && nu_th >= MCL_EKF_INTERVENE_HARD_RESET_THETA_DEG) hard_jump = true;
  int fusion_mode = MCL_EKF_FUSION_MODE;
  if (fusion_mode < 0 || fusion_mode > 2) fusion_mode = 2;
  if (fusion_mode == 0) {
    return ekf_update_mcl(ekf, pose, mcl_cov, conf, out_gate_reject);
  }
  if (fusion_mode == 1 || fusion_mode == 2) {
    if (intervene_ok) {
      if (use_reset && hard_jump) {
        ekf_reset(ekf, pose.x, pose.y, pose.theta);
        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);
        return true;
      }
      bool gate_reject = false;
      if (ekf_update_mcl(ekf, pose, mcl_cov, conf, &gate_reject)) {
        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);
        return true;
      }
      if (out_gate_reject && gate_reject) *out_gate_reject = true;
    }
    if (fusion_mode == 2 && track_ok) {
      return ekf_update_mcl(ekf, pose, mcl_cov, conf, out_gate_reject);
    }
    return false;
  }
  return false;
}
}  // namespace

ProsMCL::ProsMCL(int imu_port, const std::vector<int>& dist_ports)
  : task_(nullptr), running_(false), task_done_(true), map_blob_valid_(true), relocalize_requested_(false), localization_frozen_(false), pose_provider_(nullptr), pose_provider_user_(nullptr), provider_last_pose_{0.0, 0.0, 0.0}, provider_last_valid_(false), odom_dx_(0.0), odom_dy_(0.0), odom_dth_(0.0), odom_fault_noise_cycles_(0), last_speed_in_s_(0.0), last_turn_deg_s_(0.0), imu_last_rotation_(0.0), imu_last_rotation_valid_(false), pose_{0.0, 0.0, 0.0}, fused_pose_{0.0, 0.0, 0.0}, pose_buf_{{0.0,0.0,0.0},{0.0,0.0,0.0}}, fused_pose_buf_{{0.0,0.0,0.0},{0.0,0.0,0.0}}, pose_buf_idx_(0), mcl_ambiguous_(false), mcl_recover_good_count_(0), recovery_low_ess_streak_(0), recovery_gate_reject_streak_(0), recovery_cooldown_until_ms_(0), recovery_lost_(false), recovery_exit_streak_(0), recovery_lost_since_ms_(0), sign_selftest_samples_(0), sign_selftest_mismatch_(0), sign_selftest_done_(false), ekf_intervene_cooldown_until_ms_(0), last_event_flags_(0), sensor_batch_active_(false), sensor_batch_count_(0), sensor_batch_cursor_(0), sensor_batch_have_imu_heading_(false), sensor_batch_imu_heading_(0.0), sensor_batch_imu_applied_(false) {
  if (imu_port > 0) imu_.reset(new pros::Imu(imu_port));
  int max_dist_cfg = std::max(0, MCL_DISTANCE_SENSOR_COUNT);
  for (int p : dist_ports) {
    if (static_cast<int>(dists_.size()) >= max_dist_cfg) break;
    if (p > 0) dists_.emplace_back(p);
  }
  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {
    dist_hist_count_[i] = 0;
    dist_hist_idx_[i] = 0;
    for (int j = 0; j < DIST_MEDIAN_WINDOW; ++j) dist_hist_[i][j] = 0.0;
    sensor_batch_mm_[i] = -1.0;
    sensor_batch_conf_[i] = 0.0;
    sensor_batch_obj_[i] = 0.0;
    sensor_batch_errno_[i] = 0;
    sensor_batch_conf_meaningful_[i] = 0;
    sensor_batch_obj_valid_[i] = 0;
  }
  for (int i = 0; i < MCL_PARTICLE_CAPACITY; ++i) sensor_batch_weights_backup_[i] = 0.0;
  map_blob_valid_ = map_blob_sanity_ok();
  if (!map_blob_valid_) last_event_flags_ |= MCL_EVENT_MAP_BLOB_MISMATCH;
  ekf_.initialized = false;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ekf_.P[i][j] = 0.0;
    }
  }
}

void ProsMCL::start(unsigned seed, double initial_heading_deg) {
  if (running_.load()) return;
  mcl_.seed(seed);
  imu_last_rotation_valid_ = false;
  if (imu_) {
    imu_->reset();
    errno = 0;
    imu_->set_heading(initial_heading_deg);
    double rot0 = 0.0;
    if (imu_read_rotation_deg(*imu_, &rot0)) {
      imu_last_rotation_ = rot0;
      imu_last_rotation_valid_ = true;
    }
  }
  mcl_.initGlobal();
  {
    ScopedMutex lk(mu_);
    pose_ = mcl_.estimate();
    ekf_.initialized = false;
    fused_pose_ = pose_;
    mcl_ambiguous_ = false;
    mcl_recover_good_count_ = 0;
    recovery_low_ess_streak_ = 0;
    recovery_gate_reject_streak_ = 0;
    recovery_cooldown_until_ms_ = 0;
    recovery_lost_ = false;
    recovery_exit_streak_ = 0;
    recovery_lost_since_ms_ = 0;
    mcl_.setForcedInjectionFraction(0.0);
    sign_selftest_samples_ = 0;
    sign_selftest_mismatch_ = 0;
    sign_selftest_done_ = false;
    ekf_intervene_cooldown_until_ms_ = 0;
    last_speed_in_s_ = 0.0;
    last_turn_deg_s_ = 0.0;
    localization_frozen_.store(false);
    int next_idx = 1 - pose_buf_idx_.load();
    pose_buf_[next_idx] = pose_;
    fused_pose_buf_[next_idx] = fused_pose_;
    pose_buf_idx_.store(next_idx);
  }
  task_done_.store(false);
  running_.store(true);
  task_ = new pros::Task([this] { loop(); });
}

double ProsMCL::externalHeadingToMCL(double heading_deg) {
  return external_heading_to_mcl(heading_deg);
}

double ProsMCL::mclHeadingToExternal(double heading_deg) {
  return mcl_heading_to_external(heading_deg);
}

MCLPose ProsMCL::externalPoseToMCL(const MCLPose& pose) {
  double mx = pose.x;
  double my = pose.y;
  external_xy_to_mcl_mode(
      pose.x, pose.y,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0,
      &mx, &my);
  return {mx, my, external_heading_to_mcl(pose.theta)};
}

MCLPose ProsMCL::mclPoseToExternal(const MCLPose& pose) {
  double ex = pose.x;
  double ey = pose.y;
  mcl_xy_to_external_mode(
      pose.x, pose.y,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0,
      &ex, &ey);
  return {ex, ey, mcl_heading_to_external(pose.theta)};
}

ProsMCL::FrameSanityResult ProsMCL::runFrameSanityCheck(
    const MCLPose& start_external,
    const MCLPose& after_forward_external,
    const MCLPose& after_turn_external,
    double forward_target_in,
    double turn_target_deg) {
  FrameSanityResult out{};
  double target_fwd = std::fabs(forward_target_in);
  if (!std::isfinite(target_fwd) || target_fwd < 1e-3) target_fwd = 12.0;
  double target_turn = std::fabs(turn_target_deg);
  if (!std::isfinite(target_turn) || target_turn < 1e-3) target_turn = 90.0;

  double dx_ext = after_forward_external.x - start_external.x;
  double dy_ext = after_forward_external.y - start_external.y;
  double h1_ext = after_forward_external.theta;
  double h2_ext = after_turn_external.theta;

  external_xy_to_mcl_mode(
      dx_ext, dy_ext,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0,
      &out.current_dx_in, &out.current_dy_in);
  double h1_cur = external_heading_to_mcl_full(
      h1_ext, MCL_EXPORT_POSE_CONVENTION,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0);
  double h2_cur = external_heading_to_mcl_full(
      h2_ext, MCL_EXPORT_POSE_CONVENTION,
      MCL_EXPORT_SWAP_XY ? 1 : 0,
      MCL_EXPORT_INVERT_X ? 1 : 0,
      MCL_EXPORT_INVERT_Y ? 1 : 0);
  out.current_dtheta_deg = angle_diff_deg(h2_cur, h1_cur);

  const double trans_tol = std::max(2.0, target_fwd * 0.35);
  const double heading_tol = 20.0;
  out.translation_pass = (out.current_dx_in > 0.0) && (std::fabs(out.current_dy_in) <= trans_tol);
  out.heading_pass = (out.current_dtheta_deg > 0.0) && (std::fabs(out.current_dtheta_deg - target_turn) <= heading_tol);
  out.current_pass = out.translation_pass && out.heading_pass;

  double best_score = 1e300;
  int best_conv = MCL_EXPORT_POSE_CONVENTION;
  int best_swap = MCL_EXPORT_SWAP_XY ? 1 : 0;
  int best_inv_x = MCL_EXPORT_INVERT_X ? 1 : 0;
  int best_inv_y = MCL_EXPORT_INVERT_Y ? 1 : 0;
  for (int conv = 0; conv <= 2; ++conv) {
    for (int swap_xy = 0; swap_xy <= 1; ++swap_xy) {
      for (int inv_x = 0; inv_x <= 1; ++inv_x) {
        for (int inv_y = 0; inv_y <= 1; ++inv_y) {
          double dx_m = 0.0, dy_m = 0.0;
          external_xy_to_mcl_mode(dx_ext, dy_ext, swap_xy, inv_x, inv_y, &dx_m, &dy_m);
          double hh1 = external_heading_to_mcl_full(h1_ext, conv, swap_xy, inv_x, inv_y);
          double hh2 = external_heading_to_mcl_full(h2_ext, conv, swap_xy, inv_x, inv_y);
          double dth = angle_diff_deg(hh2, hh1);
          double mag = std::sqrt(dx_m * dx_m + dy_m * dy_m);
          double score = std::fabs(dy_m);
          if (dx_m < 0.0) score += 3.0 * std::fabs(dx_m);
          score += std::fabs(mag - target_fwd);
          if (dth < 0.0) score += 3.0 * std::fabs(dth);
          score += 0.25 * std::fabs(dth - target_turn);
          if (score < best_score) {
            best_score = score;
            best_conv = conv;
            best_swap = swap_xy;
            best_inv_x = inv_x;
            best_inv_y = inv_y;
          }
        }
      }
    }
  }

  out.recommended_pose_convention = best_conv;
  out.recommended_swap_xy = best_swap;
  out.recommended_invert_x = best_inv_x;
  out.recommended_invert_y = best_inv_y;
  out.recommended_score = best_score;
  return out;
}

void ProsMCL::startExternal(unsigned seed, double initial_heading_deg) {
  start(seed, external_heading_to_mcl(initial_heading_deg));
}

void ProsMCL::startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg) {
  start(seed, external_heading_to_mcl(initial_heading_deg));
  MCLPose p = externalPoseToMCL({start_x_in, start_y_in, start_theta_deg});
  setPose(p.x, p.y, p.theta);
}

void ProsMCL::startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg) {
  start(seed, initial_heading_deg);
  setPose(start_x_in, start_y_in, start_theta_deg);
}

void ProsMCL::stop() {
  running_.store(false);
  if (task_) {
    uint32_t t0 = pros::millis();
    while (!task_done_.load() && (pros::millis() - t0) < 250) {
      pros::delay(10);
    }
    if (!task_done_.load()) {
      task_->remove();
    }
    delete task_;
    task_ = nullptr;
  }
  task_done_.store(true);
}

void ProsMCL::setOdomDelta(double dx_in, double dy_in, double dtheta_deg) {
  ScopedMutex lk(mu_);
  if (pose_provider_ != nullptr) return;
  odom_dx_ += dx_in;
  odom_dy_ += dy_in;
  odom_dth_ += dtheta_deg;
}

void ProsMCL::setFieldPoseProvider(FieldPoseProvider provider, void* user) {
  ScopedMutex lk(mu_);
  pose_provider_ = provider;
  pose_provider_user_ = user;
  odom_dx_ = 0.0;
  odom_dy_ = 0.0;
  odom_dth_ = 0.0;
  odom_fault_noise_cycles_ = 0;
  last_speed_in_s_ = 0.0;
  last_turn_deg_s_ = 0.0;
  provider_last_valid_ = false;
}

void ProsMCL::clearFieldPoseProvider() {
  ScopedMutex lk(mu_);
  pose_provider_ = nullptr;
  pose_provider_user_ = nullptr;
  provider_last_valid_ = false;
  odom_dx_ = 0.0;
  odom_dy_ = 0.0;
  odom_dth_ = 0.0;
  odom_fault_noise_cycles_ = 0;
  last_speed_in_s_ = 0.0;
  last_turn_deg_s_ = 0.0;
}

void ProsMCL::freezeLocalization(bool freeze) {
  localization_frozen_.store(freeze);
}

bool ProsMCL::isLocalizationFrozen() const {
  return localization_frozen_.load();
}

void ProsMCL::setPose(double x_in, double y_in, double theta_deg) {
  ScopedMutex lk(mu_);
  mcl_.init(x_in, y_in, theta_deg);
  pose_ = mcl_.estimate();
  if (MCL_EKF_ENABLED) {
    ekf_reset(ekf_, pose_.x, pose_.y, pose_.theta);
    fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};
    mcl_ambiguous_ = false;
    mcl_recover_good_count_ = 0;
    recovery_low_ess_streak_ = 0;
    recovery_gate_reject_streak_ = 0;
    recovery_cooldown_until_ms_ = 0;
    recovery_lost_ = false;
    recovery_exit_streak_ = 0;
    recovery_lost_since_ms_ = 0;
    mcl_.setForcedInjectionFraction(0.0);
    sign_selftest_samples_ = 0;
    sign_selftest_mismatch_ = 0;
    sign_selftest_done_ = false;
    ekf_intervene_cooldown_until_ms_ = 0;
    localization_frozen_.store(false);
  } else {
    fused_pose_ = pose_;
  }
  mcl_ambiguous_ = false;
  mcl_recover_good_count_ = 0;
  recovery_low_ess_streak_ = 0;
  recovery_gate_reject_streak_ = 0;
  recovery_cooldown_until_ms_ = 0;
  recovery_lost_ = false;
  recovery_exit_streak_ = 0;
  recovery_lost_since_ms_ = 0;
  mcl_.setForcedInjectionFraction(0.0);
  sign_selftest_samples_ = 0;
  sign_selftest_mismatch_ = 0;
  sign_selftest_done_ = false;
  ekf_intervene_cooldown_until_ms_ = 0;
  localization_frozen_.store(false);
  odom_dx_ = 0.0;
  odom_dy_ = 0.0;
  odom_dth_ = 0.0;
  odom_fault_noise_cycles_ = 0;
  last_speed_in_s_ = 0.0;
  last_turn_deg_s_ = 0.0;
  imu_last_rotation_valid_ = false;
  provider_last_pose_ = {x_in, y_in, theta_deg};
  provider_last_valid_ = false;
  int next_idx = 1 - pose_buf_idx_.load();
  pose_buf_[next_idx] = pose_;
  fused_pose_buf_[next_idx] = fused_pose_;
  pose_buf_idx_.store(next_idx);
}

void ProsMCL::setPoseExternal(double x_in, double y_in, double theta_deg) {
  MCLPose p = externalPoseToMCL({x_in, y_in, theta_deg});
  setPose(p.x, p.y, p.theta);
}

MCLPose ProsMCL::getPose() const {
  int idx = pose_buf_idx_.load();
  if (idx < 0 || idx > 1) idx = 0;
  return pose_buf_[idx];
}

MCLPose ProsMCL::getPoseExternal() const {
  return mclPoseToExternal(getPose());
}

MCLPose ProsMCL::getFusedPose() const {
  int idx = pose_buf_idx_.load();
  if (idx < 0 || idx > 1) idx = 0;
  return fused_pose_buf_[idx];
}

MCLPose ProsMCL::getFusedPoseExternal() const {
  return mclPoseToExternal(getFusedPose());
}

ProsMCL::DebugSnapshot ProsMCL::getDebugSnapshot() const {
  ScopedMutex lk(mu_);
  DebugSnapshot s{};
  s.time_ms = pros::millis();
  s.odom_pose = provider_last_pose_;
  s.mcl_pose = pose_;
  s.fused_pose = fused_pose_;
  s.mcl_confidence = mcl_.confidence();
  s.mcl_peakedness = mcl_.confidence();
  s.mcl_ess_ratio = mcl_.essRatio();
  s.mcl_neff = mcl_.neff();
  s.ekf_pxx = ekf_.P[0][0];
  s.ekf_pyy = ekf_.P[1][1];
  s.ekf_pxy = ekf_.P[0][1];
  s.ekf_ptt = ekf_.P[2][2];
  s.event_flags = last_event_flags_;
  last_event_flags_ = 0;
  mcl_.getLastDistanceDebug(s.dist_meas_mm, s.dist_exp_mm, s.dist_errno, MCL_DISTANCE_SENSOR_COUNT_SAFE, &s.dist_used_mask);
  return s;
}

void ProsMCL::updateVision(double x_in, double y_in, double theta_deg, double confidence) {
  ScopedMutex lk(mu_);
  mcl_.updateVision(x_in, y_in, theta_deg, confidence);
  mcl_.normalize();
  double mcl_cov[3][3];
  MCLPose posterior_pose = pose_;
  mcl_.estimateCovariance(&posterior_pose, mcl_cov);
  pose_ = posterior_pose;
  if (MCL_EKF_ENABLED) {
    double conf = mcl_.confidence();
    bool gate_reject = false;
    bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf, pros::millis(), mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);
    recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;
    if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;
    if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
  } else {
    recovery_gate_reject_streak_ = 0;
  }
  mcl_.resample();
  pose_ = posterior_pose;
  if (MCL_EKF_ENABLED) {
    if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};
    else fused_pose_ = pose_;
  } else {
    fused_pose_ = pose_;
  }
  int next_idx = 1 - pose_buf_idx_.load();
  pose_buf_[next_idx] = pose_;
  fused_pose_buf_[next_idx] = fused_pose_;
  pose_buf_idx_.store(next_idx);
}

void ProsMCL::setSegmentBand(const MCLPose* pts, int n, double radius_in) {
  ScopedMutex lk(mu_);
  mcl_.setSegmentBand(pts, n, radius_in);
}

void ProsMCL::clearSegmentBand() {
  ScopedMutex lk(mu_);
  mcl_.clearSegmentBand();
}

bool ProsMCL::applyOdomCorrection(MCLPose& odom_pose, double* out_alpha) const {
  if (!MCL_CORR_ENABLED) return false;
  ScopedMutex lk(mu_);
  double conf = mcl_.confidence();
  if (conf < MCL_CORR_MIN_CONF) return false;
  if (MCL_EKF_ENABLED && mcl_ambiguous_) return false;
  if (MCL_CORR_SAFE_WINDOW_ENABLED) {
    if (MCL_CORR_SAFE_MAX_SPEED_IN_S > 0.0 && last_speed_in_s_ > MCL_CORR_SAFE_MAX_SPEED_IN_S) return false;
    if (MCL_CORR_SAFE_MAX_TURN_DEG_S > 0.0 && last_turn_deg_s_ > MCL_CORR_SAFE_MAX_TURN_DEG_S) return false;
  }
  const MCLPose& target = (MCL_EKF_ENABLED ? fused_pose_ : pose_);
  double dx = target.x - odom_pose.x;
  double dy = target.y - odom_pose.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double dth = angle_diff_deg(target.theta, odom_pose.theta);
  if (MCL_CORR_MAX_TRANS_JUMP_IN > 0.0 && dist > MCL_CORR_MAX_TRANS_JUMP_IN) return false;
  if (MCL_CORR_MAX_THETA_JUMP_DEG > 0.0 && std::fabs(dth) > MCL_CORR_MAX_THETA_JUMP_DEG) return false;
  double alpha = MCL_CORR_ALPHA_MIN;
  if (MCL_CORR_ALPHA_MAX > MCL_CORR_ALPHA_MIN && conf > MCL_CORR_MIN_CONF) {
    double t = (conf - MCL_CORR_MIN_CONF) / std::max(1e-6, (1.0 - MCL_CORR_MIN_CONF));
    t = std::max(0.0, std::min(1.0, t));
    alpha = MCL_CORR_ALPHA_MIN + (MCL_CORR_ALPHA_MAX - MCL_CORR_ALPHA_MIN) * t;
  }
  double dlx = 0.0, dly = 0.0;
  rotate_world_to_local(dx, dy, odom_pose.theta, dlx, dly);
  dlx *= alpha;
  dly *= alpha;
  double dwx = 0.0, dwy = 0.0;
  rotate_local_to_world(dlx, dly, odom_pose.theta, dwx, dwy);
  odom_pose.x += dwx;
  odom_pose.y += dwy;
  odom_pose.theta = wrap_deg(odom_pose.theta + dth * alpha);
  last_event_flags_ |= MCL_EVENT_ODOM_CORR_APPLIED;
  if (out_alpha) *out_alpha = alpha;
  return true;
}

void ProsMCL::requestRelocalize() {
  relocalize_requested_.store(true);
}

void ProsMCL::loop() {
  task_done_.store(false);
  uint32_t wake_ms = pros::millis();
  uint32_t last_motion = wake_ms;
  uint32_t last_sensor = wake_ms;
  const std::uint32_t loop_ms = static_cast<std::uint32_t>(MCL_LOOP_MS > 0 ? MCL_LOOP_MS : 1);
  while (running_.load()) {
    const uint32_t now = pros::millis();
    bool do_motion = (MCL_MOTION_UPDATE_MS <= 0 || now - last_motion >= static_cast<uint32_t>(MCL_MOTION_UPDATE_MS));
    bool do_sensor = (MCL_SENSOR_UPDATE_MS <= 0 || now - last_sensor >= static_cast<uint32_t>(MCL_SENSOR_UPDATE_MS));
    const bool distance_updates_allowed = MCL_USE_DISTANCE && map_blob_valid_;
    if (!map_blob_valid_) last_event_flags_ |= MCL_EVENT_MAP_BLOB_MISMATCH;
    double mm_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    double conf_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    double obj_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    int errno_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    int conf_meaningful_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    int obj_valid_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
    int mm_count = 0;
    FieldPoseProvider provider_cb = nullptr;
    void* provider_user = nullptr;
    MCLPose provider_prev{0.0, 0.0, 0.0};
    bool provider_prev_valid = false;
    MCLPose provider_sample{0.0, 0.0, 0.0};
    bool provider_sample_ok = false;
    bool have_imu_heading = false;
    double imu_heading = 0.0;
    bool have_imu_dtheta = false;
    double imu_dtheta = 0.0;
    bool imu_unreliable = false;
    bool imu_calibrating = false;
    if (do_sensor && !sensor_batch_active_ && distance_updates_allowed && !dists_.empty()) {
      int n = std::min(static_cast<int>(dists_.size()), MCL_DISTANCE_SENSOR_COUNT_SAFE);
      mm_count = 0;
      for (int i = 0; i < n; ++i) {
        auto& d = dists_[i];
        const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[i];
        mm_buf[mm_count] = -1.0;
        conf_buf[mm_count] = 0.0;
        obj_buf[mm_count] = 0.0;
        errno_buf[mm_count] = 0;
        conf_meaningful_buf[mm_count] = 0;
        obj_valid_buf[mm_count] = 0;
        errno = 0;
        int meas_raw_i = distance_read_mm(d);
        int meas_errno = errno;
        int sample_errno = 0;
        if (meas_raw_i == PROS_ERR) {
          sample_errno = meas_errno;
          errno_buf[mm_count] = sample_errno;
          mm_count++;
          continue;
        }
        double meas = static_cast<double>(meas_raw_i);
        bool valid = true;
        bool no_object = (meas >= 9999.0);
        double meas_bias = meas - cfg.bias_mm;
        double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;
        double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;
        if (s_min <= 0.0) s_min = 20.0;
        if (s_max <= 0.0) s_max = 2000.0;
        if (no_object) {
          if (MCL_DIST_USE_NO_OBJECT_INFO) {
            meas = s_max + cfg.bias_mm;
            meas_bias = s_max;
          } else {
            valid = false;
          }
        }
        if (valid && meas_bias > s_max) valid = false;
        if (valid && meas_bias < s_min) valid = false;
        bool conf_meaningful = valid && !no_object && (meas_bias > 200.0);
        int conf_i = 63;
        if (valid && conf_meaningful) {
          errno = 0;
          conf_i = d.get_confidence();
          int conf_errno = errno;
          if (conf_i == PROS_ERR) {
            if (sample_errno == 0) sample_errno = conf_errno;
            conf_i = 0;
            conf_meaningful = false;
          }
        }
        double conf_min = (cfg.min_confidence > 0.0) ? cfg.min_confidence : MCL_DIST_CONFIDENCE_MIN;
        if (valid && conf_meaningful && conf_min > 0.0) {
          double conf_thresh = conf_min;
          if (conf_thresh <= 1.0) conf_thresh *= 63.0;
          if (conf_thresh < 0.0) conf_thresh = 0.0;
          if (conf_thresh > 63.0) conf_thresh = 63.0;
          if (static_cast<double>(conf_i) < conf_thresh) valid = false;
        }
        int obj_i = PROS_ERR;
        bool obj_valid = false;
        if (valid && !no_object) {
          errno = 0;
          obj_i = d.get_object_size();
          int obj_errno = errno;
          obj_valid = (obj_i != PROS_ERR);
          if (!obj_valid && sample_errno == 0) sample_errno = obj_errno;
        }
        double obj_min = (cfg.min_object_size > 0.0) ? cfg.min_object_size : MCL_DIST_OBJECT_SIZE_MIN;
        double obj_max = (cfg.max_object_size > 0.0) ? cfg.max_object_size : MCL_DIST_OBJECT_SIZE_MAX;
        if (valid && obj_valid && (obj_min > 0.0 || obj_max > 0.0)) {
          if ((obj_min > 0.0 && obj_i < obj_min) || (obj_max > 0.0 && obj_i > obj_max)) valid = false;
        }
        conf_buf[mm_count] = static_cast<double>(conf_i);
        conf_meaningful_buf[mm_count] = conf_meaningful ? 1 : 0;
        obj_buf[mm_count] = static_cast<double>(obj_i);
        errno_buf[mm_count] = sample_errno;
        obj_valid_buf[mm_count] = obj_valid ? 1 : 0;
        if (!valid) {
          mm_count++;
          continue;
        }
        if (MCL_DIST_MEDIAN_WINDOW > 1) {
          int idx = dist_hist_idx_[i];
          dist_hist_[i][idx] = meas;
          idx = (idx + 1) % DIST_MEDIAN_WINDOW;
          dist_hist_idx_[i] = idx;
          if (dist_hist_count_[i] < DIST_MEDIAN_WINDOW) dist_hist_count_[i]++;
          double temp[DIST_MEDIAN_WINDOW];
          int cnt = dist_hist_count_[i];
          for (int j = 0; j < cnt; ++j) temp[j] = dist_hist_[i][j];
          std::sort(temp, temp + cnt);
          meas = temp[cnt / 2];
        }
        mm_buf[mm_count] = meas;
        mm_count++;
      }
    }
    if (do_motion) {
      {
        ScopedMutex lk(mu_);
        provider_cb = pose_provider_;
        provider_user = pose_provider_user_;
        provider_prev = provider_last_pose_;
        provider_prev_valid = provider_last_valid_;
      }
      if (provider_cb) {
        provider_sample = provider_prev;
        provider_sample_ok = provider_cb(&provider_sample, provider_user);
      }
    }
    if (imu_ && imu_is_calibrating(*imu_)) {
      imu_calibrating = true;
      imu_unreliable = true;
    }
    if (do_sensor && imu_ && !sensor_batch_active_ && !imu_calibrating) {
      double h = 0.0;
      if (imu_read_heading_deg(*imu_, &h)) {
        imu_heading = h;
        have_imu_heading = true;
      } else {
        imu_unreliable = true;
      }
    }
    if (do_motion && imu_) {
      double rot = 0.0;
      if (!imu_calibrating && imu_read_rotation_deg(*imu_, &rot)) {
        if (imu_last_rotation_valid_) {
          imu_dtheta = rot - imu_last_rotation_;
          have_imu_dtheta = true;
        }
        imu_last_rotation_ = rot;
        imu_last_rotation_valid_ = true;
      } else {
        imu_last_rotation_valid_ = false;
        imu_unreliable = true;
      }
    }
    {
      ScopedMutex lk(mu_);
      if (relocalize_requested_.load()) {
        if (sensor_batch_active_) {
          mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);
          sensor_batch_active_ = false;
          sensor_batch_count_ = 0;
          sensor_batch_cursor_ = 0;
          sensor_batch_have_imu_heading_ = false;
          sensor_batch_imu_applied_ = false;
        }
        mcl_.initGlobal();
        pose_ = mcl_.estimate();
        ekf_.initialized = false;
        fused_pose_ = pose_;
        odom_dx_ = 0.0;
        odom_dy_ = 0.0;
        odom_dth_ = 0.0;
        odom_fault_noise_cycles_ = 0;
        last_speed_in_s_ = 0.0;
        last_turn_deg_s_ = 0.0;
        imu_last_rotation_valid_ = false;
        provider_last_valid_ = false;
        mcl_ambiguous_ = false;
        mcl_recover_good_count_ = 0;
        recovery_low_ess_streak_ = 0;
        recovery_gate_reject_streak_ = 0;
        recovery_cooldown_until_ms_ = 0;
        recovery_exit_streak_ = 0;
        if (recovery_lost_ && recovery_lost_since_ms_ == 0) recovery_lost_since_ms_ = now;
        if (!recovery_lost_) {
          recovery_lost_since_ms_ = 0;
          mcl_.setForcedInjectionFraction(0.0);
        }
        sign_selftest_samples_ = 0;
        sign_selftest_mismatch_ = 0;
        sign_selftest_done_ = false;
        ekf_intervene_cooldown_until_ms_ = 0;
        relocalize_requested_.store(false);
      }
      if (localization_frozen_.load()) {
        if (sensor_batch_active_) {
          mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);
          sensor_batch_active_ = false;
          sensor_batch_count_ = 0;
          sensor_batch_cursor_ = 0;
          sensor_batch_have_imu_heading_ = false;
          sensor_batch_imu_applied_ = false;
        }
        if (do_motion) {
          if (provider_cb == pose_provider_ && provider_sample_ok) {
            provider_last_pose_ = provider_sample;
            provider_last_valid_ = true;
          }
          odom_dx_ = 0.0;
          odom_dy_ = 0.0;
          odom_dth_ = 0.0;
          odom_fault_noise_cycles_ = 0;
          last_speed_in_s_ = 0.0;
          last_turn_deg_s_ = 0.0;
          last_motion = now;
        }
        if (do_sensor) {
          last_sensor = now;
        }
      } else if (do_motion) {
        if (provider_cb == pose_provider_ && provider_sample_ok) {
          if (provider_prev_valid) {
            double dX = provider_sample.x - provider_prev.x;
            double dY = provider_sample.y - provider_prev.y;
            double dx_auto = 0.0, dy_auto = 0.0;
            rotate_world_to_local(dX, dY, provider_prev.theta, dx_auto, dy_auto);
            odom_dx_ += dx_auto;
            odom_dy_ += dy_auto;
            odom_dth_ += angle_diff_deg(provider_sample.theta, provider_prev.theta);
          }
          provider_last_pose_ = provider_sample;
          provider_last_valid_ = true;
        }
        double dx = odom_dx_;
        double dy = odom_dy_;
        double dth_odom = odom_dth_;
        double dth = dth_odom;
        if (MCL_FRAME_SIGN_SELF_TEST_ENABLED && !sign_selftest_done_ && have_imu_dtheta) {
          double min_delta = (MCL_FRAME_SIGN_SELF_TEST_MIN_DELTA_DEG > 0.0) ?
            MCL_FRAME_SIGN_SELF_TEST_MIN_DELTA_DEG : 2.0;
          if (std::fabs(imu_dtheta) >= min_delta && std::fabs(dth_odom) >= min_delta) {
            sign_selftest_samples_++;
            if ((imu_dtheta * dth_odom) < 0.0) sign_selftest_mismatch_++;
            int target = std::max(1, MCL_FRAME_SIGN_SELF_TEST_SAMPLES);
            int mismatch_th = std::max(1, MCL_FRAME_SIGN_SELF_TEST_MISMATCH);
            if (sign_selftest_samples_ >= target) {
              sign_selftest_done_ = true;
              if (sign_selftest_mismatch_ >= mismatch_th) {
                last_event_flags_ |= MCL_EVENT_FRAME_SIGN_MISMATCH;
              }
            }
          }
        }
        if (have_imu_dtheta) dth = imu_dtheta;
        odom_dx_ = 0.0;
        odom_dy_ = 0.0;
        odom_dth_ = 0.0;
        double motion_noise_scale = 1.0;
        if (MCL_ODOM_DELTA_GUARD_ENABLED) {
          bool odom_fault = false;
          if (MCL_ODOM_MAX_DX_IN_PER_TICK > 0.0 && std::fabs(dx) > MCL_ODOM_MAX_DX_IN_PER_TICK) odom_fault = true;
          if (MCL_ODOM_MAX_DY_IN_PER_TICK > 0.0 && std::fabs(dy) > MCL_ODOM_MAX_DY_IN_PER_TICK) odom_fault = true;
          if (MCL_ODOM_MAX_DTHETA_DEG_PER_TICK > 0.0 && std::fabs(dth) > MCL_ODOM_MAX_DTHETA_DEG_PER_TICK) odom_fault = true;
          if (odom_fault) {
            dx = 0.0;
            dy = 0.0;
            dth = 0.0;
            if (MCL_ODOM_FAULT_INFLATE_CYCLES > 0) odom_fault_noise_cycles_ = std::max(odom_fault_noise_cycles_, MCL_ODOM_FAULT_INFLATE_CYCLES);
          }
        }
        if (odom_fault_noise_cycles_ > 0) {
          if (MCL_ODOM_FAULT_NOISE_SCALE > 1.0) motion_noise_scale = MCL_ODOM_FAULT_NOISE_SCALE;
          odom_fault_noise_cycles_--;
        }
        if (imu_ && imu_unreliable && MCL_IMU_FALLBACK_NOISE_SCALE > 1.0) {
          motion_noise_scale = std::max(motion_noise_scale, MCL_IMU_FALLBACK_NOISE_SCALE);
        }
        double dt_s = static_cast<double>((MCL_MOTION_UPDATE_MS > 0) ? MCL_MOTION_UPDATE_MS : MCL_LOOP_MS) / 1000.0;
        if (dt_s < 1e-3) dt_s = 1e-3;
        last_speed_in_s_ = std::sqrt(dx * dx + dy * dy) / dt_s;
        last_turn_deg_s_ = std::fabs(dth) / dt_s;
        mcl_.predict(dx, dy, dth, motion_noise_scale);
        pose_ = mcl_.estimate();
        if (MCL_EKF_ENABLED) {
          if (ekf_.initialized) {
            ekf_predict(ekf_, dx, dy, dth, motion_noise_scale);
            fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};
          } else {
            fused_pose_ = pose_;
          }
        } else {
          fused_pose_ = pose_;
        }
        last_motion = now;
      }
      if (!localization_frozen_.load()) {
        if (do_sensor && !sensor_batch_active_) {
          if (distance_updates_allowed && mm_count > 0) {
            mcl_.copyWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);
            sensor_batch_count_ = std::min(mm_count, MCL_DISTANCE_SENSOR_COUNT_SAFE);
            for (int i = 0; i < sensor_batch_count_; ++i) {
              sensor_batch_mm_[i] = mm_buf[i];
              sensor_batch_conf_[i] = conf_buf[i];
              sensor_batch_obj_[i] = obj_buf[i];
              sensor_batch_errno_[i] = errno_buf[i];
              sensor_batch_conf_meaningful_[i] = conf_meaningful_buf[i];
              sensor_batch_obj_valid_[i] = obj_valid_buf[i];
            }
            for (int i = sensor_batch_count_; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {
              sensor_batch_mm_[i] = -1.0;
              sensor_batch_conf_[i] = 0.0;
              sensor_batch_obj_[i] = 0.0;
              sensor_batch_errno_[i] = 0;
              sensor_batch_conf_meaningful_[i] = 0;
              sensor_batch_obj_valid_[i] = 0;
            }
            sensor_batch_cursor_ = 0;
            sensor_batch_have_imu_heading_ = have_imu_heading;
            sensor_batch_imu_heading_ = imu_heading;
            sensor_batch_imu_applied_ = false;
            sensor_batch_active_ = true;
          } else if (have_imu_heading) {
            if (MCL_USE_IMU) mcl_.updateIMU(imu_heading);
            if (MCL_EKF_ENABLED && MCL_EKF_USE_IMU) ekf_update_imu(ekf_, imu_heading);
            mcl_.normalize();
            double mcl_cov[3][3];
            MCLPose posterior_pose = pose_;
            mcl_.estimateCovariance(&posterior_pose, mcl_cov);
            pose_ = posterior_pose;
            bool gate_reject = false;
            double conf_now = mcl_.confidence();
            if (MCL_EKF_ENABLED) {
              bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf_now, now, mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);
              if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;
              if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
              recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;
            } else {
              recovery_gate_reject_streak_ = 0;
            }
            double ess_ratio = mcl_.neff() / std::max(1.0, static_cast<double>(mcl_.particleCount()));
            if (MCL_RECOVERY_ESS_RATIO_MIN > 0.0 && ess_ratio < MCL_RECOVERY_ESS_RATIO_MIN) recovery_low_ess_streak_++;
            else recovery_low_ess_streak_ = 0;
            bool recovery_trigger = false;
            if (MCL_RECOVERY_ENABLED) {
              if (MCL_RECOVERY_ESS_STREAK > 0 && recovery_low_ess_streak_ >= MCL_RECOVERY_ESS_STREAK) recovery_trigger = true;
              if (MCL_EKF_ENABLED && MCL_RECOVERY_EKF_GATE_REJECT_STREAK > 0 &&
                  recovery_gate_reject_streak_ >= MCL_RECOVERY_EKF_GATE_REJECT_STREAK) recovery_trigger = true;
            }
            if (recovery_trigger && now >= recovery_cooldown_until_ms_) {
              recovery_lost_ = true;
              recovery_exit_streak_ = 0;
              recovery_lost_since_ms_ = now;
              double lost_inj = MCL_RECOVERY_LOST_INJECTION_FRACTION;
              if (lost_inj < 0.0) lost_inj = 0.0;
              if (lost_inj > 1.0) lost_inj = 1.0;
              mcl_.setForcedInjectionFraction(lost_inj);
              relocalize_requested_.store(true);
              recovery_low_ess_streak_ = 0;
              recovery_gate_reject_streak_ = 0;
              if (MCL_RECOVERY_COOLDOWN_MS > 0) recovery_cooldown_until_ms_ = now + static_cast<uint32_t>(MCL_RECOVERY_COOLDOWN_MS);
              last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
            }
            if (recovery_lost_) {
              bool lost_good = (conf_now >= MCL_RECOVERY_LOST_EXIT_CONFIDENCE) && !gate_reject && !mcl_ambiguous_;
              if (lost_good) recovery_exit_streak_++;
              else recovery_exit_streak_ = 0;
              if (MCL_RECOVERY_LOST_FORCE_REINIT_MS > 0 && now > recovery_lost_since_ms_ &&
                  (now - recovery_lost_since_ms_) >= static_cast<uint32_t>(MCL_RECOVERY_LOST_FORCE_REINIT_MS)) {
                relocalize_requested_.store(true);
                recovery_lost_since_ms_ = now;
              }
              if (recovery_exit_streak_ >= std::max(1, MCL_RECOVERY_LOST_EXIT_STREAK)) {
                recovery_lost_ = false;
                recovery_exit_streak_ = 0;
                recovery_lost_since_ms_ = 0;
                mcl_.setForcedInjectionFraction(0.0);
              } else {
                last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
              }
            }
            mcl_.resample();
            pose_ = posterior_pose;
            if (MCL_EKF_ENABLED) {
              if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};
              else fused_pose_ = pose_;
            } else {
              fused_pose_ = pose_;
            }
            last_sensor = now;
          } else {
            last_sensor = now;
          }
        }
        if (sensor_batch_active_) {
          int n = sensor_batch_count_;
          int step = MCL_DIST_BATCH_SIZE;
          if (step <= 0) step = n;
          if (step <= 0) step = MCL_DISTANCE_SENSOR_COUNT_SAFE;
          int begin = sensor_batch_cursor_;
          if (begin < 0) begin = 0;
          if (begin > n) begin = n;
          int end = begin + step;
          if (end > n) end = n;
          if (distance_updates_allowed && n > 0 && end > begin) {
            double chunk_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            double chunk_conf[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            double chunk_obj[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            int chunk_errno[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            int chunk_conf_meaningful[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            int chunk_obj_valid[MCL_DISTANCE_SENSOR_COUNT_SAFE];
            for (int i = 0; i < n; ++i) {
              chunk_mm[i] = -1.0;
              chunk_conf[i] = 0.0;
              chunk_obj[i] = 0.0;
              chunk_errno[i] = 0;
              chunk_conf_meaningful[i] = 0;
              chunk_obj_valid[i] = 0;
            }
            for (int i = begin; i < end; ++i) {
              chunk_mm[i] = sensor_batch_mm_[i];
              chunk_conf[i] = sensor_batch_conf_[i];
              chunk_obj[i] = sensor_batch_obj_[i];
              chunk_errno[i] = sensor_batch_errno_[i];
              chunk_conf_meaningful[i] = sensor_batch_conf_meaningful_[i];
              chunk_obj_valid[i] = sensor_batch_obj_valid_[i];
            }
            mcl_.updateDistance(chunk_mm, chunk_conf, chunk_conf_meaningful, chunk_obj, chunk_obj_valid, chunk_errno, n);
          }
          sensor_batch_cursor_ = end;
          bool batch_done = (sensor_batch_cursor_ >= n);
          if (batch_done) {
            if (sensor_batch_have_imu_heading_ && !sensor_batch_imu_applied_) {
              if (MCL_USE_IMU) mcl_.updateIMU(sensor_batch_imu_heading_);
              if (MCL_EKF_ENABLED && MCL_EKF_USE_IMU) ekf_update_imu(ekf_, sensor_batch_imu_heading_);
              sensor_batch_imu_applied_ = true;
            }
            mcl_.normalize();
            double mcl_cov[3][3];
            MCLPose posterior_pose = pose_;
            mcl_.estimateCovariance(&posterior_pose, mcl_cov);
            pose_ = posterior_pose;
            bool gate_reject = false;
            double conf_now = mcl_.confidence();
            if (MCL_EKF_ENABLED) {
              bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf_now, now, mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);
              if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;
              if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
              recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;
            } else {
              recovery_gate_reject_streak_ = 0;
            }
            double ess_ratio = mcl_.neff() / std::max(1.0, static_cast<double>(mcl_.particleCount()));
            if (MCL_RECOVERY_ESS_RATIO_MIN > 0.0 && ess_ratio < MCL_RECOVERY_ESS_RATIO_MIN) recovery_low_ess_streak_++;
            else recovery_low_ess_streak_ = 0;
            bool recovery_trigger = false;
            if (MCL_RECOVERY_ENABLED) {
              if (MCL_RECOVERY_ESS_STREAK > 0 && recovery_low_ess_streak_ >= MCL_RECOVERY_ESS_STREAK) recovery_trigger = true;
              if (MCL_EKF_ENABLED && MCL_RECOVERY_EKF_GATE_REJECT_STREAK > 0 &&
                  recovery_gate_reject_streak_ >= MCL_RECOVERY_EKF_GATE_REJECT_STREAK) recovery_trigger = true;
            }
            if (recovery_trigger && now >= recovery_cooldown_until_ms_) {
              recovery_lost_ = true;
              recovery_exit_streak_ = 0;
              recovery_lost_since_ms_ = now;
              double lost_inj = MCL_RECOVERY_LOST_INJECTION_FRACTION;
              if (lost_inj < 0.0) lost_inj = 0.0;
              if (lost_inj > 1.0) lost_inj = 1.0;
              mcl_.setForcedInjectionFraction(lost_inj);
              relocalize_requested_.store(true);
              recovery_low_ess_streak_ = 0;
              recovery_gate_reject_streak_ = 0;
              if (MCL_RECOVERY_COOLDOWN_MS > 0) recovery_cooldown_until_ms_ = now + static_cast<uint32_t>(MCL_RECOVERY_COOLDOWN_MS);
              last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
            }
            if (recovery_lost_) {
              bool lost_good = (conf_now >= MCL_RECOVERY_LOST_EXIT_CONFIDENCE) && !gate_reject && !mcl_ambiguous_;
              if (lost_good) recovery_exit_streak_++;
              else recovery_exit_streak_ = 0;
              if (MCL_RECOVERY_LOST_FORCE_REINIT_MS > 0 && now > recovery_lost_since_ms_ &&
                  (now - recovery_lost_since_ms_) >= static_cast<uint32_t>(MCL_RECOVERY_LOST_FORCE_REINIT_MS)) {
                relocalize_requested_.store(true);
                recovery_lost_since_ms_ = now;
              }
              if (recovery_exit_streak_ >= std::max(1, MCL_RECOVERY_LOST_EXIT_STREAK)) {
                recovery_lost_ = false;
                recovery_exit_streak_ = 0;
                recovery_lost_since_ms_ = 0;
                mcl_.setForcedInjectionFraction(0.0);
              } else {
                last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;
              }
            }
            mcl_.resample();
            pose_ = posterior_pose;
            if (MCL_EKF_ENABLED) {
              if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};
              else fused_pose_ = pose_;
            } else {
              fused_pose_ = pose_;
            }
            sensor_batch_active_ = false;
            sensor_batch_count_ = 0;
            sensor_batch_cursor_ = 0;
            sensor_batch_have_imu_heading_ = false;
            sensor_batch_imu_applied_ = false;
            last_sensor = now;
          }
        }
      }
      int next_idx = 1 - pose_buf_idx_.load();
      pose_buf_[next_idx] = pose_;
      fused_pose_buf_[next_idx] = fused_pose_;
      pose_buf_idx_.store(next_idx);
    }
    const uint32_t now_end = pros::millis();
    const uint32_t late_ms = (now_end > wake_ms) ? (now_end - wake_ms) : 0u;
    if (MCL_STALL_MS > 0 && late_ms > static_cast<uint32_t>(MCL_STALL_MS)) {
      ScopedMutex lk(mu_);
      if (sensor_batch_active_) {
        mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);
        sensor_batch_active_ = false;
        sensor_batch_count_ = 0;
        sensor_batch_cursor_ = 0;
        sensor_batch_have_imu_heading_ = false;
        sensor_batch_imu_applied_ = false;
      }
      last_sensor = now_end;
    }
    pros::Task::delay_until(&wake_ms, loop_ms);
  }
  task_done_.store(true);
}
