#pragma once

#include "api.h"
#include "mcl_tune_packets.h"

#include <atomic>
#include <cstdint>
#include <cstdio>

class ProsMCL;

class MCLTuneLogger {
 public:
  explicit MCLTuneLogger(ProsMCL* runtime);
  bool start(const char* file_prefix = "session");
  void stop();
  bool isRunning() const { return running_.load(); }
  void addEvent(std::uint32_t flags);
  const char* filePath() const { return file_path_; }

 private:
  static constexpr int kRingSize = 512;
  static constexpr int kFlushChunk = 64;
  void sampleLoop();
  void flushLoop();
  bool pushFrame(const mcl_tune::LogFrame& f);
  bool popFrame(mcl_tune::LogFrame* out);

  ProsMCL* runtime_;
  FILE* fp_;
  char file_path_[96];
  std::atomic_bool running_;
  std::atomic_bool sampler_done_;
  std::atomic_bool flush_done_;
  std::atomic_uint32_t pending_event_flags_;
  std::uint32_t frame_count_;
  std::uint32_t crc32_;
  pros::Mutex ring_mu_;
  mcl_tune::LogFrame ring_[kRingSize];
  int ring_head_;
  int ring_tail_;
  pros::Task* sampler_task_;
  pros::Task* flush_task_;
};
