#include "mcl_tune_log.h"
#include "mcl_runtime.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
static std::int32_t to_milin(double v_in) { return static_cast<std::int32_t>(std::lround(v_in * 1000.0)); }
static std::int32_t to_mdeg(double v_deg) { return static_cast<std::int32_t>(std::lround(v_deg * 1000.0)); }
static std::int32_t to_uin2(double v) { return static_cast<std::int32_t>(std::lround(v * 1000000.0)); }
static std::int32_t to_umdeg2(double v) { return static_cast<std::int32_t>(std::lround(v * 1000000.0)); }
static std::uint32_t crc32_update(std::uint32_t crc, const void* data, std::size_t len) {
  const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
  crc = ~crc;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
  }
  return ~crc;
}
}

MCLTuneLogger::MCLTuneLogger(ProsMCL* runtime)
  : runtime_(runtime), fp_(nullptr), running_(false), sampler_done_(true), flush_done_(true),
    pending_event_flags_(0), frame_count_(0), crc32_(0), ring_head_(0), ring_tail_(0),
    sampler_task_(nullptr), flush_task_(nullptr) {
  file_path_[0] = '\0';
}

bool MCLTuneLogger::start(const char* file_prefix) {
  if (!runtime_ || running_.load()) return false;
  if (!MCL_TUNING_ENABLED) return false;
  std::snprintf(file_path_, sizeof(file_path_), "/usd/atticus/mcl_tune/%s_%lu.mcllog",
                (file_prefix && file_prefix[0]) ? file_prefix : "session",
                static_cast<unsigned long>(pros::millis()));
  fp_ = std::fopen(file_path_, "wb");
  if (!fp_) {
    std::snprintf(file_path_, sizeof(file_path_), "/usd/%s_%lu.mcllog",
                  (file_prefix && file_prefix[0]) ? file_prefix : "session",
                  static_cast<unsigned long>(pros::millis()));
    fp_ = std::fopen(file_path_, "wb");
    if (!fp_) return false;
  }

  mcl_tune::LogHeader header{};
  header.magic = mcl_tune::kMagic;
  header.version = mcl_tune::kVersion;
  header.header_size = static_cast<std::uint16_t>(sizeof(mcl_tune::LogHeader));
  header.config_hash32 = MCL_CONFIG_HASH32;
  header.sensor_count = static_cast<std::uint16_t>(MCL_DISTANCE_SENSOR_COUNT);
  for (int i = 0; i < mcl_tune::kMaxSensors; ++i) {
    double deg = (i < MCL_DISTANCE_SENSOR_COUNT) ? MCL_DISTANCE_SENSORS[i].angle_deg : 0.0;
    header.sensor_angle_mdeg[i] = to_mdeg(deg);
  }
  std::fwrite(&header, sizeof(header), 1, fp_);

  frame_count_ = 0;
  crc32_ = 0;
  ring_head_ = 0;
  ring_tail_ = 0;
  pending_event_flags_.store(0);
  sampler_done_.store(false);
  flush_done_.store(false);
  running_.store(true);

  sampler_task_ = new pros::Task([this] { sampleLoop(); });
  flush_task_ = new pros::Task([this] { flushLoop(); });
  return true;
}

void MCLTuneLogger::stop() {
  if (!running_.load()) return;
  running_.store(false);
  while (!sampler_done_.load() || !flush_done_.load()) pros::delay(5);
  if (sampler_task_) { delete sampler_task_; sampler_task_ = nullptr; }
  if (flush_task_) { delete flush_task_; flush_task_ = nullptr; }
  if (fp_) {
    mcl_tune::LogFooter footer{};
    footer.frame_count = frame_count_;
    footer.crc32 = crc32_;
    std::fwrite(&footer, sizeof(footer), 1, fp_);
    std::fclose(fp_);
    fp_ = nullptr;
  }
}

void MCLTuneLogger::addEvent(std::uint32_t flags) {
  pending_event_flags_.fetch_or(flags);
}

bool MCLTuneLogger::pushFrame(const mcl_tune::LogFrame& f) {
  ring_mu_.take();
  int next = (ring_head_ + 1) % kRingSize;
  if (next == ring_tail_) {
    ring_mu_.give();
    return false;
  }
  ring_[ring_head_] = f;
  ring_head_ = next;
  ring_mu_.give();
  return true;
}

bool MCLTuneLogger::popFrame(mcl_tune::LogFrame* out) {
  ring_mu_.take();
  if (ring_tail_ == ring_head_) {
    ring_mu_.give();
    return false;
  }
  *out = ring_[ring_tail_];
  ring_tail_ = (ring_tail_ + 1) % kRingSize;
  ring_mu_.give();
  return true;
}

void MCLTuneLogger::sampleLoop() {
  int period_ms = 50;
  if (MCL_TUNING_LOG_RATE_HZ > 0) period_ms = std::max(10, 1000 / MCL_TUNING_LOG_RATE_HZ);
  while (running_.load()) {
    ProsMCL::DebugSnapshot s = runtime_->getDebugSnapshot();
    mcl_tune::LogFrame f{};
    f.t_ms = s.time_ms;
    f.event_flags = s.event_flags | pending_event_flags_.exchange(0);
    f.dist_used_mask = s.dist_used_mask;
    f.odom_x_milin = to_milin(s.odom_pose.x);
    f.odom_y_milin = to_milin(s.odom_pose.y);
    f.odom_th_mdeg = to_mdeg(s.odom_pose.theta);
    f.mcl_x_milin = to_milin(s.mcl_pose.x);
    f.mcl_y_milin = to_milin(s.mcl_pose.y);
    f.mcl_th_mdeg = to_mdeg(s.mcl_pose.theta);
    f.fused_x_milin = to_milin(s.fused_pose.x);
    f.fused_y_milin = to_milin(s.fused_pose.y);
    f.fused_th_mdeg = to_mdeg(s.fused_pose.theta);
    f.ekf_pxx_uin2 = to_uin2(s.ekf_pxx);
    f.ekf_pyy_uin2 = to_uin2(s.ekf_pyy);
    f.ekf_pxy_uin2 = to_uin2(s.ekf_pxy);
    f.ekf_ptt_umdeg2 = to_umdeg2(s.ekf_ptt);
    f.mcl_conf_milli = static_cast<std::int32_t>(std::lround(s.mcl_confidence * 1000.0));
    f.mcl_neff_milli = static_cast<std::int32_t>(std::lround(s.mcl_neff * 1000.0));
    for (int i = 0; i < mcl_tune::kMaxSensors; ++i) {
      f.dist_meas_mm[i] = static_cast<std::int32_t>(std::lround(s.dist_meas_mm[i]));
      f.dist_exp_mm[i] = static_cast<std::int32_t>(std::lround(s.dist_exp_mm[i]));
    }
    pushFrame(f);
    pros::delay(period_ms);
  }
  sampler_done_.store(true);
}

void MCLTuneLogger::flushLoop() {
  while (true) {
    if (!fp_) break;
    mcl_tune::LogFrame chunk[kFlushChunk];
    int n = 0;
    while (n < kFlushChunk && popFrame(&chunk[n])) n++;
    if (n > 0) {
      std::fwrite(chunk, sizeof(mcl_tune::LogFrame), static_cast<size_t>(n), fp_);
      frame_count_ += static_cast<std::uint32_t>(n);
      crc32_ = crc32_update(crc32_, chunk, static_cast<std::size_t>(n) * sizeof(mcl_tune::LogFrame));
      std::fflush(fp_);
    } else {
      if (!running_.load()) break;
      pros::delay(10);
    }
  }
  flush_done_.store(true);
}
