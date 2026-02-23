#pragma once

#include "mcl_tune_log.h"
#include "mcl_runtime.h"

class MCLTuningWizard {
 public:
  explicit MCLTuningWizard(ProsMCL* runtime) : logger_(runtime), recovery_active_(false) {}
  bool begin(const char* file_prefix = "session") { return logger_.start(file_prefix); }
  void end() { logger_.stop(); recovery_active_ = false; }
  void markStep() { logger_.addEvent(mcl_tune::EVENT_STEP_MARK); }
  void markKidnappedStart() { recovery_active_ = true; logger_.addEvent(mcl_tune::EVENT_STEP_MARK | mcl_tune::EVENT_RECOVERY_ACTIVE); }
  void markKidnappedPlaced() { recovery_active_ = false; logger_.addEvent(mcl_tune::EVENT_STEP_MARK); }
  bool recoveryActive() const { return recovery_active_; }
  const char* filePath() const { return logger_.filePath(); }

 private:
  MCLTuneLogger logger_;
  bool recovery_active_;
};
