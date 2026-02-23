from __future__ import annotations

import json
import math
import pathlib
import sys


def _must_contain(text: str, needle: str, label: str, errors: list[str]) -> None:
    """Handle must contain."""
    if needle not in text:
        errors.append(f"missing [{label}]")


def _must_not_contain(text: str, needle: str, label: str, errors: list[str]) -> None:
    """Handle must not contain."""
    if needle in text:
        errors.append(f"forbidden [{label}]")


def _must_contain_in_order(text: str, first: str, second: str, label: str, errors: list[str]) -> None:
    """Handle must contain in order."""
    i = text.find(first)
    j = text.find(second, i + len(first) if i >= 0 else 0)
    if i < 0 or j < 0 or j < i:
        errors.append(f"order-fail [{label}]")


def _ang_err_deg(a: float, b: float) -> float:
    """Smallest signed angular error in degrees."""
    return ((a - b + 180.0) % 360.0) - 180.0


def main() -> int:
    """Handle main."""
    root = pathlib.Path(__file__).resolve().parents[1]
    gen = (root / "mod" / "mcl_codegen.py").read_text(encoding="utf-8")
    tuning = (root / "mod" / "mcl_tuning.py").read_text(encoding="utf-8")
    config_py = (root / "mod" / "config.py").read_text(encoding="utf-8")
    mcl_py = (root / "mod" / "mcl.py").read_text(encoding="utf-8")
    export_validator = (root / "mod" / "atticus_export_validation.py").read_text(encoding="utf-8")
    pack_codegen = (root / "mod" / "atticus_pack_codegen.py").read_text(encoding="utf-8")
    deep_report = (root / "deep-research-report.md").read_text(encoding="utf-8")
    main_py = (root / "main.py").read_text(encoding="utf-8")
    runtime_path = root / "pros" / "src" / "mcl_runtime.cpp"
    localizer_path = root / "pros" / "src" / "mcl_localizer.cpp"
    tune_log_h_path = root / "pros" / "include" / "mcl_tune_log.h"
    tune_packets_h_path = root / "pros" / "include" / "mcl_tune_packets.h"
    ex1_path = root / "pros" / "examples" / "01_minimal_motor_encoders_imu.cpp"
    ex2_path = root / "pros" / "examples" / "02_using_distance_sensors.cpp"
    ex3_path = root / "pros" / "examples" / "03_mcl_intervenes_with_lemlib.cpp"
    ex4_path = root / "pros" / "examples" / "04_tuning_wizard_session.cpp"
    ex_readme_path = root / "pros" / "examples" / "README.md"
    api_doc_path = root / "pros" / "docs" / "API_REFERENCE.md"
    checklist_doc_path = root / "pros" / "docs" / "CHECKLIST.md"
    term_doc_path = root / "pros" / "docs" / "TERMINOLOGY.md"
    pros_readme_path = root / "pros" / "README.md"
    config_root_path = root / "config.json"
    config_mod_path = root / "mod" / "config.json"
    runtime = runtime_path.read_text(encoding="utf-8") if runtime_path.exists() else ""
    localizer = localizer_path.read_text(encoding="utf-8") if localizer_path.exists() else ""
    tune_log_h = tune_log_h_path.read_text(encoding="utf-8") if tune_log_h_path.exists() else ""
    tune_packets_h = tune_packets_h_path.read_text(encoding="utf-8") if tune_packets_h_path.exists() else ""
    ex1 = ex1_path.read_text(encoding="utf-8") if ex1_path.exists() else ""
    ex2 = ex2_path.read_text(encoding="utf-8") if ex2_path.exists() else ""
    ex3 = ex3_path.read_text(encoding="utf-8") if ex3_path.exists() else ""
    ex4 = ex4_path.read_text(encoding="utf-8") if ex4_path.exists() else ""
    ex_readme = ex_readme_path.read_text(encoding="utf-8") if ex_readme_path.exists() else ""
    api_doc = api_doc_path.read_text(encoding="utf-8") if api_doc_path.exists() else ""
    checklist_doc = checklist_doc_path.read_text(encoding="utf-8") if checklist_doc_path.exists() else ""
    term_doc = term_doc_path.read_text(encoding="utf-8") if term_doc_path.exists() else ""
    pros_readme = pros_readme_path.read_text(encoding="utf-8") if pros_readme_path.exists() else ""
    config_root = json.loads(config_root_path.read_text(encoding="utf-8")) if config_root_path.exists() else None
    config_mod = json.loads(config_mod_path.read_text(encoding="utf-8")) if config_mod_path.exists() else None

    errors: list[str] = []

    if config_root is not None and config_mod is not None and config_root != config_mod:
        errors.append("config mismatch [config.json vs mod/config.json]")

    _must_contain(gen, "MCLPose dbg_pose = est;", "gen dbg prior", errors)
    _must_not_contain(gen, "MCLPose dbg_pose = estimate();", "gen dbg stale", errors)
    if localizer:
        _must_contain(localizer, "MCLPose dbg_pose = est;", "localizer dbg prior", errors)
        _must_not_contain(localizer, "MCLPose dbg_pose = estimate();", "localizer dbg stale", errors)

    _must_contain(gen, "s.event_flags = last_event_flags_;", "gen snapshot read", errors)
    _must_contain_in_order(gen, "s.event_flags = last_event_flags_;", "last_event_flags_ = 0;", "gen clear-after-read", errors)
    if runtime:
        _must_contain(runtime, "s.event_flags = last_event_flags_;", "runtime snapshot read", errors)
        _must_contain_in_order(runtime, "s.event_flags = last_event_flags_;", "last_event_flags_ = 0;", "runtime clear-after-read", errors)

    if localizer:
        _must_contain(localizer, "particles_[i].x = new_x;", "perimeter apply pose", errors)
        _must_contain(localizer, "particles_[i].w = 0.0;", "perimeter zero weight", errors)
        _must_contain(localizer, "continue;", "perimeter continue", errors)

    _must_contain_in_order(gen, "mcl_.predict(dx, dy, dth, motion_noise_scale);", "pose_ = mcl_.estimate();", "gen motion pose refresh", errors)
    if runtime:
        _must_contain_in_order(runtime, "mcl_.predict(dx, dy, dth, motion_noise_scale);", "pose_ = mcl_.estimate();", "runtime motion pose refresh", errors)

    _must_contain(gen, "odom_dx_ = 0.0;", "gen relocalize clear odom dx", errors)
    _must_contain(gen, "odom_dy_ = 0.0;", "gen relocalize clear odom dy", errors)
    _must_contain(gen, "odom_dth_ = 0.0;", "gen relocalize clear odom dth", errors)
    _must_contain(gen, "provider_last_valid_ = false;", "gen relocalize clear provider", errors)
    if runtime:
        _must_contain(runtime, "odom_dx_ = 0.0;", "runtime relocalize clear odom dx", errors)
        _must_contain(runtime, "odom_dy_ = 0.0;", "runtime relocalize clear odom dy", errors)
        _must_contain(runtime, "odom_dth_ = 0.0;", "runtime relocalize clear odom dth", errors)
        _must_contain(runtime, "provider_last_valid_ = false;", "runtime relocalize clear provider", errors)

    _must_contain(gen, "MCLPose est = estimate_;", "gen prior est snapshot", errors)
    _must_contain(gen, "if (have_est) est = estimate();", "gen prior est compute", errors)
    if localizer:
        _must_contain(localizer, "MCLPose est = estimate_;", "localizer prior est snapshot", errors)
        _must_contain(localizer, "if (have_est) est = estimate();", "localizer prior est compute", errors)

    _must_contain_in_order(gen, "provider_cb = pose_provider_;", "provider_sample_ok = provider_cb(&provider_sample, provider_user);", "gen provider sampled after unlocked copy", errors)
    if runtime:
        _must_contain_in_order(runtime, "provider_cb = pose_provider_;", "provider_sample_ok = provider_cb(&provider_sample, provider_user);", "runtime provider sampled after unlocked copy", errors)

    _must_contain(gen, "if (do_sensor && imu_ && !sensor_batch_active_ && !imu_calibrating) {", "gen imu read pre-lock", errors)
    _must_contain(gen, "if (have_imu_heading) {", "gen imu apply captured", errors)
    _must_not_contain(gen, "if (imu_) {\n          double imu_heading = static_cast<double>(imu_->get_heading());", "gen imu read in lock", errors)
    if runtime:
        _must_contain(runtime, "if (do_sensor && imu_ && !sensor_batch_active_ && !imu_calibrating) {", "runtime imu read pre-lock", errors)
        _must_contain(runtime, "if (have_imu_heading) {", "runtime imu apply captured", errors)
        _must_not_contain(runtime, "if (imu_) {\n          double imu_heading = static_cast<double>(imu_->get_heading());", "runtime imu read in lock", errors)

    _must_contain(tuning, "def _edge_indices(", "edge helper", errors)
    _must_contain(tuning, "mcl_ekf_apply_count = len(_edge_indices(", "edge mcl->ekf count", errors)
    _must_contain(tuning, "odom_corr_apply_count = len(_edge_indices(", "edge odom count", errors)

    _must_contain(tuning, "crc_calc = zlib.crc32(frame_blob)", "crc compute", errors)
    _must_contain(tuning, "if crc32_footer != 0 and crc_calc != crc32_footer:", "crc mismatch gate", errors)
    _must_contain(tuning, "CRC mismatch", "crc error message", errors)
    _must_contain(gen, "pros::Task::delay_until(&wake_ms, loop_ms);", "gen delay_until loop", errors)
    _must_contain(gen, "const uint32_t late_ms = (now_end > wake_ms) ? (now_end - wake_ms) : 0u;", "gen late_ms end-loop", errors)
    _must_contain(gen, "if (conf_min_map >= 63.0) conf_min_map = 62.0;", "gen confidence denominator guard", errors)
    _must_contain(gen, "if (do_sensor && !sensor_batch_active_) {", "gen batch start gate", errors)
    _must_contain(gen, "if (sensor_batch_active_) {", "gen batch active process", errors)
    _must_contain(gen, "mcl_.updateDistance(chunk_mm, chunk_conf, chunk_conf_meaningful, chunk_obj, chunk_obj_valid, chunk_errno, n);", "gen chunked distance update", errors)
    _must_contain(gen, "bool batch_done = (sensor_batch_cursor_ >= n);", "gen batch completion check", errors)
    _must_contain(gen, "mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);", "gen stall/freeze/relocalize batch rollback", errors)
    _must_contain(gen, "MCLPose posterior_pose = pose_;", "gen posterior pose capture", errors)
    _must_contain(gen, "mcl_.estimateCovariance(&posterior_pose, mcl_cov);", "gen posterior covariance target", errors)
    _must_contain_in_order(gen, "mcl_.resample();", "pose_ = posterior_pose;", "gen publish pre-resample posterior", errors)
    _must_not_contain(gen, "mcl_.resample();\n            pose_ = mcl_.estimate();", "gen forbid post-resample pose overwrite", errors)
    _must_contain(gen, "double logp_rays[9];", "gen fov multi-ray buffer", errors)
    _must_contain(gen, "log_w += logmeanexp(logp_rays, ray_used);", "gen fov logmeanexp combine", errors)
    _must_contain(gen, "static double distance_field_lookup(const uint16_t* field", "gen uint16 likelihood-field lookup", errors)
    _must_contain(gen, "const int* obj_size_valid, const int* dist_errno, int count)", "gen updateDistance signature errno channel", errors)
    _must_not_contain(gen, "while (imu_->is_calibrating()) pros::delay(10);", "gen imu blocking wait", errors)
    _must_contain(gen, "double max_log_w = -1e300;", "gen log-domain distance weights", errors)
    _must_contain(gen, "double w = std::exp(tmp_weights_[i] - max_log_w);", "gen log-domain max normalization", errors)
    _must_contain(gen, "double sigma_eff = (meas <= 200.0) ? MCL_DIST_SIGMA_HIT_MM : (MCL_DIST_SIGMA_FAR_SCALE * meas);", "gen sigma model uses config knobs", errors)
    _must_contain(gen, "double essRatio() const { return ess_ratio_; }", "gen explicit ess-ratio API", errors)
    _must_contain(gen, "double ess_ratio_;", "gen ess-ratio member", errors)
    _must_contain(gen, "ess_ratio_ = std::max(0.0, std::min(1.0, frac));", "gen ess-ratio tracking", errors)
    _must_contain(gen, "confidence_ = std::max(0.0, std::min(1.0, 1.0 - ess_ratio_));", "gen peakedness confidence mapping", errors)
    _must_contain(gen, "s.mcl_peakedness = mcl_.confidence();", "gen debug peakedness field", errors)
    _must_contain(gen, "s.mcl_ess_ratio = mcl_.essRatio();", "gen debug ess-ratio field", errors)
    _must_contain(gen, "constexpr double MCL_RESAMPLE_ROUGHEN_XY_IN", "gen roughening config xy", errors)
    _must_contain(gen, "constexpr double MCL_RESAMPLE_ROUGHEN_THETA_DEG", "gen roughening config theta", errors)
    _must_contain(gen, "if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0 || MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {", "gen roughening apply", errors)
    _must_contain(gen, "constexpr double MCL_EKF_MCL_MAHALANOBIS_GATE", "gen ekf mahal gate config", errors)
    _must_contain(gen, "if (md2 > MCL_EKF_MCL_MAHALANOBIS_GATE) {", "gen ekf mahal gate apply", errors)
    _must_contain(gen, "if (out_gate_reject) *out_gate_reject = true;", "gen ekf gate reject signal", errors)
    _must_contain(gen, "if (valid && meas_bias > s_max) valid = false;", "gen distance high-range invalid", errors)
    _must_not_contain(gen, "if (valid && meas_bias > s_max) meas = s_max + cfg.bias_mm;", "gen distance high-range clamp removed", errors)
    _must_contain(gen, "if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0)", "gen finite check distance inputs", errors)
    _must_contain(gen, "if (MCL_DIST_USE_NO_OBJECT_INFO) {", "gen no-object info optional path", errors)
    _must_contain(gen, "LF_COARSEN_LADDER_IN = (0.5, 1.0, 1.5, 2.0, 3.0)", "gen fixed lf coarsen ladder", errors)
    _must_contain(gen, "MCL_EXPECTED_MAP_HASH32", "gen expected map hash config", errors)
    _must_contain(gen, "MCL_EXPECTED_MAP_PROJECT_HASH32", "gen expected map project hash config", errors)
    _must_contain(gen, "MCL_EXPECTED_MAP_GRID_W", "gen expected map width config", errors)
    _must_contain(gen, "MCL_EXPECTED_MAP_GRID_H", "gen expected map height config", errors)
    _must_contain(gen, "MCL_EXPECTED_MAP_DIST_LEN", "gen expected map len config", errors)
    _must_contain(gen, "map_blob_sanity_ok()", "gen runtime map sanity helper", errors)
    _must_contain(gen, "distance_updates_allowed = MCL_USE_DISTANCE && map_blob_valid_;", "gen map-gated distance updates", errors)
    _must_contain(gen, "MCL_EVENT_MAP_BLOB_MISMATCH", "gen runtime map mismatch event", errors)
    _must_contain(gen, "MCL_CGR_LITE_ENABLED", "gen cgr-lite enable config", errors)
    _must_contain(gen, "MCL_CGR_LITE_TOP_K", "gen cgr-lite top-k config", errors)
    _must_contain(gen, "MCL_CGR_LITE_MAX_ITERS", "gen cgr-lite iter config", errors)
    _must_contain(gen, "MCL_CGR_LITE_BUDGET_MS", "gen cgr-lite budget config", errors)
    _must_contain(gen, "void cgrLiteRefineEstimate();", "gen cgr-lite refine declaration", errors)
    _must_contain(gen, "void MCLLocalizer::cgrLiteRefineEstimate() {", "gen cgr-lite refine impl", errors)
    _must_contain(gen, "if (MCL_CGR_LITE_ENABLED) cgrLiteRefineEstimate();", "gen cgr-lite refine call", errors)
    _must_contain(gen, "template <typename DistanceT>", "gen distance shim template helper", errors)
    _must_contain(gen, "static auto distance_read_mm_impl(DistanceT& d, int) -> decltype(static_cast<int>(d.get())) {", "gen distance shim get() path", errors)
    _must_contain(gen, "static auto distance_read_mm_impl(DistanceT& d, long) -> decltype(static_cast<int>(d.get_distance())) {", "gen distance shim get_distance() path", errors)
    _must_contain(gen, "static int distance_read_mm(pros::Distance& d) {", "gen distance api shim", errors)
    _must_contain(gen, "int meas_raw_i = distance_read_mm(d);", "gen distance read via shim", errors)
    _must_not_contain(gen, "requires(", "gen avoid c++20 requires in shim", errors)
    _must_contain(gen, "static bool imu_read_heading_deg(pros::Imu& imu, double* out_heading) {", "gen imu heading helper", errors)
    _must_contain(gen, "static bool imu_read_rotation_deg(pros::Imu& imu, double* out_rotation) {", "gen imu rotation helper", errors)
    _must_contain(gen, "static bool imu_is_calibrating(pros::Imu& imu) {", "gen imu calibrating helper", errors)
    _must_not_contain(gen, "imu_->is_calibrating()", "gen avoid is_calibrating dependency", errors)
    _must_contain(gen, "static std::mt19937& rng_engine() {", "gen mt19937 engine", errors)
    _must_contain(gen, "rng_engine().seed(seed);", "gen mt19937 seed", errors)
    _must_not_contain(gen, "std::srand(seed);", "gen legacy srand removed", errors)
    _must_not_contain(gen, "std::rand()", "gen legacy rand removed", errors)
    _must_contain(gen, "constexpr bool MCL_ODOM_DELTA_GUARD_ENABLED", "gen odom guard config", errors)
    _must_contain(gen, "constexpr int MCL_POSE_CONVENTION_CW_ZERO_FORWARD = 1;", "gen pose convention cw const", errors)
    _must_contain(gen, "constexpr int MCL_POSE_CONVENTION_CCW_ZERO_FORWARD = 2;", "gen pose convention ccw const", errors)
    _must_contain(gen, "constexpr bool MCL_EXPORT_SWAP_XY", "gen pose adapter swap const", errors)
    _must_contain(gen, "constexpr bool MCL_EXPORT_INVERT_X", "gen pose adapter invert x const", errors)
    _must_contain(gen, "constexpr bool MCL_EXPORT_INVERT_Y", "gen pose adapter invert y const", errors)
    _must_contain(gen, "static double external_heading_to_mcl_full(", "gen full external heading adapter", errors)
    _must_contain(gen, "static double mcl_heading_to_external_full(", "gen full reverse heading adapter", errors)
    _must_contain(gen, "static MCLPose externalPoseToMCL(const MCLPose& pose);", "gen external pose adapter api", errors)
    _must_contain(gen, "void startExternal(unsigned seed, double initial_heading_deg);", "gen external start api", errors)
    _must_contain(gen, "static FrameSanityResult runFrameSanityCheck(", "gen frame sanity api", errors)
    _must_contain(gen, "MCLPose p = externalPoseToMCL({start_x_in, start_y_in, start_theta_deg});", "gen startEasyExternal pose convert", errors)
    _must_contain(gen, "localizer.startEasyExternal((int)pros::millis(), 0.0, 0.0, 0.0, 0.0);", "gen examples external easy start", errors)
    _must_contain(gen, "localizer.startExternal((int)pros::millis(), 0.0);", "gen examples external heading start", errors)
    _must_contain(gen, "  if (pose_provider_ != nullptr) return;", "gen exclusive odom feed guard", errors)
    _must_contain(gen, "  odom_fault_noise_cycles_ = 0;", "gen provider mode clears odom fault inflation", errors)
    _must_contain(gen, "if (MCL_AUGMENTED_ENABLED) {", "gen deterministic injection branch augmented", errors)
    _must_contain(gen, "inj = MCL_RANDOM_INJECTION;", "gen deterministic injection branch fixed", errors)
    _must_not_contain(gen, "inj = std::max(inj, std::max(0.0, 1.0 - w_fast_ / w_slow_));", "gen remove mixed injection precedence", errors)
    _must_not_contain(gen, "if (PERIM_SEGMENTS_COUNT > 0) {\n      double p = raycast_distance_segments(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);", "gen duplicate perimeter raycast removed", errors)
    _must_contain(gen, "void predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale = 1.0);", "gen predict noise-scale signature", errors)
    _must_contain(gen, "double ns = std::isfinite(noise_scale) ? noise_scale : 1.0;", "gen motion noise scale clamp", errors)
    _must_contain(gen, "mcl_.predict(dx, dy, dth, motion_noise_scale);", "gen runtime motion noise-scale predict", errors)
    _must_contain(gen, "if (MCL_ODOM_MAX_DX_IN_PER_TICK > 0.0 && std::fabs(dx) > MCL_ODOM_MAX_DX_IN_PER_TICK) odom_fault = true;", "gen odom dx clamp", errors)
    _must_contain(gen, "if (imu_ && imu_unreliable && MCL_IMU_FALLBACK_NOISE_SCALE > 1.0) {", "gen imu fallback noise inflation", errors)
    _must_contain(gen, "if (MCL_FRAME_SIGN_SELF_TEST_ENABLED && !sign_selftest_done_ && have_imu_dtheta) {", "gen frame sign self-test hook", errors)
    _must_contain(gen, "if (MCL_RECOVERY_ESS_STREAK > 0 && recovery_low_ess_streak_ >= MCL_RECOVERY_ESS_STREAK) recovery_trigger = true;", "gen recovery ess trigger", errors)
    _must_contain(gen, "recovery_gate_reject_streak_ >= MCL_RECOVERY_EKF_GATE_REJECT_STREAK", "gen recovery gate reject trigger", errors)
    _must_contain(gen, "tune_log_h = r'''#pragma once\n\n#include \"api.h\"\n#include \"mcl_tune_packets.h\"", "gen tune header includes", errors)
    _must_contain(gen, "tune_packets_h = r'''#pragma once\n\n#include \"mcl_config.hpp\"\n\n#include <cstdint>", "gen tune packets includes", errors)
    _must_contain(gen, "tune_log_cpp = r'''#include \"mcl_tune_log.h\"\n#include \"mcl_runtime.h\"\n\n#include <algorithm>", "gen tune cpp includes", errors)
    if runtime:
        _must_not_contain(runtime, "while (imu_->is_calibrating()) pros::delay(10);", "runtime imu blocking wait", errors)
        _must_not_contain(runtime, "imu_->is_calibrating()", "runtime avoid is_calibrating dependency", errors)
        _must_contain(runtime, "external_xy_to_mcl_mode(", "runtime external->mcl xy adapter helper", errors)
        _must_contain(runtime, "mcl_xy_to_external_mode(", "runtime mcl->external xy adapter helper", errors)
        _must_contain(runtime, "external_heading_to_mcl_full(", "runtime full external heading adapter", errors)
        _must_contain(runtime, "mcl_heading_to_external_full(", "runtime full reverse heading adapter", errors)
        _must_contain(runtime, "ProsMCL::FrameSanityResult ProsMCL::runFrameSanityCheck(", "runtime frame sanity impl", errors)
        _must_contain(runtime, "MCLPose p = externalPoseToMCL({start_x_in, start_y_in, start_theta_deg});", "runtime startEasyExternal pose convert", errors)
        _must_contain(runtime, "if (pose_provider_ != nullptr) return;", "runtime exclusive odom feed guard", errors)
        _must_contain(runtime, "odom_fault_noise_cycles_ = 0;", "runtime provider mode clears odom fault inflation", errors)
        _must_not_contain(runtime, "inj = std::max(inj, std::max(0.0, 1.0 - w_fast_ / w_slow_));", "runtime remove mixed injection precedence", errors)
        _must_not_contain(runtime, "if (PERIM_SEGMENTS_COUNT > 0) {\n      double p = raycast_distance_segments(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);", "runtime duplicate perimeter raycast removed", errors)
        _must_contain(runtime, "int meas_raw_i = distance_read_mm(d);", "runtime distance read via shim", errors)
        _must_contain(runtime, "mcl_.updateDistance(chunk_mm, chunk_conf, chunk_conf_meaningful, chunk_obj, chunk_obj_valid, chunk_errno, n);", "runtime distance errno batch update", errors)
        _must_contain(runtime, "mcl_.getLastDistanceDebug(s.dist_meas_mm, s.dist_exp_mm, s.dist_errno, MCL_DISTANCE_SENSOR_COUNT_SAFE, &s.dist_used_mask);", "runtime debug errno snapshot", errors)
        _must_not_contain(runtime, "mcl_.resample();\n            pose_ = mcl_.estimate();", "runtime forbid post-resample pose overwrite", errors)
        _must_not_contain(runtime, "requires(", "runtime avoid c++20 requires in shim", errors)
        _must_contain(runtime, "mcl_.predict(dx, dy, dth, motion_noise_scale);", "runtime motion noise-scale predict", errors)
        _must_contain(runtime, "if (MCL_ODOM_MAX_DX_IN_PER_TICK > 0.0 && std::fabs(dx) > MCL_ODOM_MAX_DX_IN_PER_TICK) odom_fault = true;", "runtime odom dx clamp", errors)
    if localizer:
        _must_contain(localizer, "double max_log_w = -1e300;", "localizer log-domain distance weights", errors)
        _must_contain(localizer, "double w = std::exp(tmp_weights_[i] - max_log_w);", "localizer log-domain max normalization", errors)
        _must_contain(localizer, "if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0 || MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {", "localizer roughening apply", errors)
        _must_contain(localizer, "static std::mt19937& rng_engine() {", "localizer mt19937 engine", errors)
        _must_contain(localizer, "rng_engine().seed(seed);", "localizer mt19937 seed", errors)
        _must_not_contain(localizer, "requires(", "localizer avoid c++20 requires in shim", errors)
        _must_not_contain(localizer, "std::srand(seed);", "localizer legacy srand removed", errors)
        _must_not_contain(localizer, "std::rand()", "localizer legacy rand removed", errors)
        _must_contain(localizer, "void MCLLocalizer::predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale) {", "localizer predict noise-scale impl", errors)
        _must_contain(localizer, "double ns = std::isfinite(noise_scale) ? noise_scale : 1.0;", "localizer motion noise scale clamp", errors)
    if tune_log_h:
        _must_contain(tune_log_h, "#include \"api.h\"", "tune log header api include", errors)
        _must_contain(tune_log_h, "#include \"mcl_tune_packets.h\"", "tune log header packets include", errors)
        _must_contain(tune_log_h, "class ProsMCL;", "tune log header ProsMCL fwd", errors)
        _must_contain(tune_log_h, "bool pushFrame(const mcl_tune::LogFrame& f);", "tune log header frame signature", errors)
    if tune_packets_h:
        _must_contain(tune_packets_h, "#include \"mcl_config.hpp\"", "tune packets config include", errors)
        _must_contain(tune_packets_h, "#include <cstdint>", "tune packets cstdint include", errors)

    _must_contain(config_py, "motion_cfg[\"motion_model\"] = \"drive\"", "config lock motion model", errors)
    _must_contain(config_py, "motion_cfg[\"motion_source\"] = \"encoders\"", "config lock motion source", errors)
    _must_contain(config_py, "if dist_model not in (\"likelihood_field\", \"beam\"):", "config lock distance model enum", errors)
    _must_contain(config_py, "if dist_model != \"likelihood_field\":", "config beam single-ray policy", errors)
    _must_contain(config_py, "dist_cfg[\"fov_multi_ray\"] = 0", "config disable beam multi-ray", errors)
    _must_contain(config_py, "dist_cfg[\"rays_per_sensor\"] = 1", "config force beam single-ray count", errors)
    _must_contain(config_py, "conf_cfg[\"metric\"] = \"peakedness\"", "config lock confidence metric semantics", errors)
    _must_contain(config_py, "\"cgr_lite\": {", "config cgr-lite defaults", errors)
    _must_contain(main_py, "\"motion_model\": \"drive\"", "main save fixed motion model", errors)
    _must_contain(main_py, "\"motion_source\": \"encoders\"", "main save fixed motion source", errors)
    _must_contain(main_py, "_set_row_visible(mcl_row_random_injection, not aug_on)", "main hide random injection under augmented", errors)
    _must_contain(main_py, "_set_row_visible(mcl_row_dist_w_short, dist_on and not dist_lf)", "main hide beam-only w_short in lf", errors)
    _must_contain(main_py, "_set_row_visible(mcl_row_dist_lambda_short, dist_on and not dist_lf)", "main hide beam-only lambda in lf", errors)
    _must_contain(main_py, "if dist_model_save != \"likelihood_field\":", "main save beam single-ray policy", errors)
    _must_contain(main_py, "\"fov_multi_ray\": dist_fov_multi", "main save fov policy field", errors)
    _must_contain(main_py, "\"rays_per_sensor\": dist_rays_per_sensor", "main save ray-count policy field", errors)
    _must_contain(main_py, "mcl_cgr_enabled_var", "main cgr-lite ui var", errors)
    _must_contain(main_py, "mcl_row_cgr_top_k", "main cgr-lite ui row", errors)
    _must_contain(main_py, "mcl[\"cgr_lite\"] = {", "main save cgr-lite block", errors)
    _must_contain(mcl_py, "if augmented_enabled:", "sim deterministic injection augmented branch", errors)
    _must_contain(mcl_py, "else:\n            inj = float(mcl.get(\"random_injection\", 0.0))", "sim deterministic injection fixed branch", errors)
    _must_not_contain(mcl_py, "inj = max(inj, max(0.0, 1.0 - state.w_fast / state.w_slow))", "sim remove mixed injection precedence", errors)
    _must_contain(mcl_py, "if model not in (\"likelihood_field\", \"beam\"):", "sim lock distance model enum", errors)
    _must_contain(mcl_py, "fov_multi_ray = int(dist_cfg.get(\"fov_multi_ray\", 0)) == 1 and model == \"likelihood_field\"", "sim lf-only multi-ray policy", errors)
    _must_contain(mcl_py, "if model != \"likelihood_field\":\n        rays_per_sensor = 1", "sim beam single-ray count", errors)
    _must_contain(gen, "dist_fov_multi = _bool(dist.get(\"fov_multi_ray\", 0)) and dist_model_val == 0", "gen lf-only multi-ray policy", errors)
    _must_contain(gen, "if dist_model_val != 0:\n        dist_rays = 1", "gen beam single-ray count", errors)
    _must_contain(pack_codegen, "class ProsMclLocalizationSource final : public atticus::ILocalizationSource", "pack localizer source adapter", errors)
    _must_contain(pack_codegen, "buildLocalizationSchedulePolicy", "pack scheduling policy export", errors)
    _must_contain(pack_codegen, "buildCgrLitePolicy", "pack cgr-lite policy export", errors)
    _must_contain(pack_codegen, "buildCorridorDefaults", "pack corridor defaults export", errors)
    _must_not_contain(pack_codegen, "TriggerUnit::Fraction", "pack runtime fraction triggers removed", errors)
    _must_contain(pack_codegen, "if st in {\"pose\"}:", "pack path no longer treated as pose segment", errors)
    _must_contain(pack_codegen, "flags = _segment_flags(seg, seg_type)", "pack segment flags export path", errors)
    _must_contain(pack_codegen, "auto_convert_unsupported: bool = False", "pack safe default disable unsupported auto-convert", errors)
    _must_contain(pack_codegen, "out.correction_suggested =", "pack adapter correction-suggested semantics", errors)
    _must_contain(pack_codegen, "out.lost =", "pack adapter lost semantics", errors)
    _must_contain(pack_codegen, "atticus_drive_ff.hpp", "pack drive ff export", errors)
    _must_contain(pack_codegen, "atticus_drive_lut.bin", "pack optional drive lut export", errors)
    _must_not_contain(pack_codegen, "ILocalizer", "pack legacy localizer naming removed", errors)
    _must_not_contain(deep_report, "ILocalizer", "report legacy localizer naming removed", errors)
    _must_contain(main_py, "auto_convert_unsupported\"), 0)", "main safe default auto-convert off", errors)
    _must_contain(config_py, "\"auto_convert_unsupported\": {\"value\": 0}", "config safe default path conversion off", errors)
    _must_contain(export_validator, "path_not_supported_in_efficiency_pack", "validator explicit path unsupported code", errors)
    _must_contain(export_validator, "trigger_order_nonmonotonic", "validator trigger order monotonic check", errors)

    if ex1:
        _must_contain(ex1, "localizer.startEasyExternal((int)pros::millis(), 0.0, 0.0, 0.0, 0.0);", "example01 external startup", errors)
    if ex2:
        _must_contain(ex2, "localizer.startEasyExternal((int)pros::millis(), 0.0, 0.0, 0.0, 0.0);", "example02 external startup", errors)
    if ex3:
        _must_contain(ex3, "localizer.startExternal((int)pros::millis(), 0.0);", "example03 external heading startup", errors)
        _must_contain(ex3, "return ProsMCL::externalPoseToMCL(MCLPose{p.x, p.y, p.theta});", "example03 provider conversion", errors)
    if ex4:
        _must_contain(ex4, "This provider emits internal MCL-frame pose directly", "example04 internal frame note", errors)
        _must_contain(ex4, "localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);", "example04 internal startup", errors)
    if ex_readme:
        _must_contain(ex_readme, "These examples are practical autonomous templates", "examples readme present", errors)
    if pros_readme:
        _must_contain(pros_readme, "startEasyExternal", "pros readme external startup guidance", errors)
        _must_contain(pros_readme, "Runtime treats these modes as exclusive", "pros readme feed exclusivity guidance", errors)
    if api_doc:
        _must_contain(api_doc, "setPoseExternal", "api docs setPoseExternal", errors)
        _must_contain(api_doc, "getPoseExternal", "api docs getPoseExternal", errors)
        _must_contain(api_doc, "freezeLocalization", "api docs freezeLocalization", errors)
        _must_contain(api_doc, "`sigma_hit_mm`", "api docs sigma_hit_mm naming", errors)
        _must_not_contain(api_doc, "`sigma_mm`", "api docs stale sigma_mm naming", errors)
        _must_contain(api_doc, "Injection policy is deterministic", "api docs deterministic injection policy", errors)
    if checklist_doc:
        _must_contain(checklist_doc, "Manual deltas are ignored while provider mode is active", "checklist feed exclusivity note", errors)
        _must_contain(checklist_doc, "runFrameSanityCheck", "checklist frame sanity guidance", errors)
    if term_doc:
        _must_contain(term_doc, "**Confidence (peakedness):** `1 - ESS/N`", "terminology peakedness confidence", errors)

    # Numeric invariant checks for full pose adapter math.
    # Keep deterministic and lightweight.
    ATTICUS = 0
    CW0 = 1
    CCW0 = 2

    def _wrap(d: float) -> float:
        while d >= 360.0:
            d -= 360.0
        while d < 0.0:
            d += 360.0
        return d

    def _ex_h_to_mcl_mode(h: float, conv: int) -> float:
        h = _wrap(h)
        if conv == CW0:
            return _wrap(h + 90.0)
        if conv == CCW0:
            return _wrap(90.0 - h)
        return h

    def _mcl_h_to_ex_mode(h: float, conv: int) -> float:
        h = _wrap(h)
        if conv == CW0:
            return _wrap(h - 90.0)
        if conv == CCW0:
            return _wrap(90.0 - h)
        return h

    def _ex_xy_to_mcl(x: float, y: float, swap_xy: int, inv_x: int, inv_y: int) -> tuple[float, float]:
        if swap_xy:
            x, y = y, x
        if inv_x:
            x = -x
        if inv_y:
            y = -y
        return x, y

    def _mcl_xy_to_ex(x: float, y: float, swap_xy: int, inv_x: int, inv_y: int) -> tuple[float, float]:
        if inv_y:
            y = -y
        if inv_x:
            x = -x
        if swap_xy:
            x, y = y, x
        return x, y

    def _h_to_unit(h: float) -> tuple[float, float]:
        th = math.radians(h)
        return math.sin(th), math.cos(th)

    def _unit_to_h(dx: float, dy: float) -> float:
        if abs(dx) < 1e-12 and abs(dy) < 1e-12:
            return 0.0
        return _wrap(math.degrees(math.atan2(dx, dy)))

    def _ex_h_to_mcl_full(h: float, conv: int, swap_xy: int, inv_x: int, inv_y: int) -> float:
        hx, hy = _h_to_unit(_ex_h_to_mcl_mode(h, conv))
        hx, hy = _ex_xy_to_mcl(hx, hy, swap_xy, inv_x, inv_y)
        return _unit_to_h(hx, hy)

    def _mcl_h_to_ex_full(h: float, conv: int, swap_xy: int, inv_x: int, inv_y: int) -> float:
        hx, hy = _h_to_unit(h)
        hx, hy = _mcl_xy_to_ex(hx, hy, swap_xy, inv_x, inv_y)
        return _mcl_h_to_ex_mode(_unit_to_h(hx, hy), conv)

    sample_xy = [(-144.0, -96.0), (-12.5, 8.75), (0.0, 0.0), (25.5, -33.25), (143.5, 143.5)]
    sample_h = [0.0, 15.0, 89.5, 180.0, 271.0, 359.9]
    for conv in (ATTICUS, CW0, CCW0):
        for swap_xy in (0, 1):
            for inv_x in (0, 1):
                for inv_y in (0, 1):
                    for x, y in sample_xy:
                        mx, my = _ex_xy_to_mcl(x, y, swap_xy, inv_x, inv_y)
                        rx, ry = _mcl_xy_to_ex(mx, my, swap_xy, inv_x, inv_y)
                        if abs(x - rx) > 1e-9 or abs(y - ry) > 1e-9:
                            errors.append(f"math-fail [xy-roundtrip conv={conv} s={swap_xy} ix={inv_x} iy={inv_y}]")
                            break
                    for h in sample_h:
                        mh = _ex_h_to_mcl_full(h, conv, swap_xy, inv_x, inv_y)
                        rh = _mcl_h_to_ex_full(mh, conv, swap_xy, inv_x, inv_y)
                        if abs(_ang_err_deg(rh, h)) > 1e-8:
                            errors.append(f"math-fail [heading-roundtrip conv={conv} s={swap_xy} ix={inv_x} iy={inv_y}]")
                            break

    if errors:
        print("MCL regression checks FAILED:")
        for e in errors:
            print(f"- {e}")
        return 1
    print("MCL regression checks PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
