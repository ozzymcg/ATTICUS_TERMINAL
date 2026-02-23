#pragma once

#include "mcl_config.hpp"

#include <cstdint>

namespace mcl_tune {
constexpr std::uint32_t kMagic = 0x31434C4Du;  // "MLC1"
constexpr std::uint16_t kVersion = 1;
constexpr int kMaxSensors = MCL_DISTANCE_SENSOR_COUNT_SAFE;

enum EventFlags : std::uint32_t {
  EVENT_NONE = 0,
  EVENT_MCL_TO_EKF_APPLIED = 1u << 0,
  EVENT_ODOM_CORR_APPLIED = 1u << 1,
  EVENT_STEP_MARK = 1u << 2,
  EVENT_RECOVERY_ACTIVE = 1u << 3
};

struct LogHeader {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t header_size;
  std::uint32_t config_hash32;
  std::uint16_t sensor_count;
  std::uint16_t reserved0;
  std::int32_t sensor_angle_mdeg[kMaxSensors];
};

struct LogFrame {
  std::uint32_t t_ms;
  std::uint32_t event_flags;
  std::uint32_t dist_used_mask;
  std::int32_t odom_x_milin;
  std::int32_t odom_y_milin;
  std::int32_t odom_th_mdeg;
  std::int32_t mcl_x_milin;
  std::int32_t mcl_y_milin;
  std::int32_t mcl_th_mdeg;
  std::int32_t fused_x_milin;
  std::int32_t fused_y_milin;
  std::int32_t fused_th_mdeg;
  std::int32_t ekf_pxx_uin2;
  std::int32_t ekf_pyy_uin2;
  std::int32_t ekf_pxy_uin2;
  std::int32_t ekf_ptt_umdeg2;
  std::int32_t mcl_conf_milli;
  std::int32_t mcl_neff_milli;
  std::int32_t dist_meas_mm[kMaxSensors];
  std::int32_t dist_exp_mm[kMaxSensors];
};

struct LogFooter {
  std::uint32_t frame_count;
  std::uint32_t crc32;
};
}  // namespace mcl_tune
