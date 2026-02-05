from __future__ import annotations

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


def main() -> int:
    """Handle main."""
    root = pathlib.Path(__file__).resolve().parents[1]
    gen = (root / "mod" / "mcl_codegen.py").read_text(encoding="utf-8")
    tuning = (root / "mod" / "mcl_tuning.py").read_text(encoding="utf-8")
    runtime = (root / "pros" / "src" / "mcl_runtime.cpp").read_text(encoding="utf-8")
    localizer = (root / "pros" / "src" / "mcl_localizer.cpp").read_text(encoding="utf-8")

    errors: list[str] = []

    _must_contain(gen, "MCLPose dbg_pose = est;", "gen dbg prior", errors)
    _must_not_contain(gen, "MCLPose dbg_pose = estimate();", "gen dbg stale", errors)
    _must_contain(localizer, "MCLPose dbg_pose = est;", "localizer dbg prior", errors)
    _must_not_contain(localizer, "MCLPose dbg_pose = estimate();", "localizer dbg stale", errors)

    _must_contain(gen, "s.event_flags = last_event_flags_;", "gen snapshot read", errors)
    _must_contain_in_order(gen, "s.event_flags = last_event_flags_;", "last_event_flags_ = 0;", "gen clear-after-read", errors)
    _must_contain(runtime, "s.event_flags = last_event_flags_;", "runtime snapshot read", errors)
    _must_contain_in_order(runtime, "s.event_flags = last_event_flags_;", "last_event_flags_ = 0;", "runtime clear-after-read", errors)

    _must_contain(localizer, "particles_[i].x = new_x;", "perimeter apply pose", errors)
    _must_contain(localizer, "particles_[i].w = 0.0;", "perimeter zero weight", errors)
    _must_contain(localizer, "continue;", "perimeter continue", errors)

    _must_contain_in_order(gen, "mcl_.predict(dx, dy, dth);", "pose_ = mcl_.estimate();", "gen motion pose refresh", errors)
    _must_contain_in_order(runtime, "mcl_.predict(dx, dy, dth);", "pose_ = mcl_.estimate();", "runtime motion pose refresh", errors)

    _must_contain(gen, "odom_dx_ = 0.0;", "gen relocalize clear odom dx", errors)
    _must_contain(gen, "odom_dy_ = 0.0;", "gen relocalize clear odom dy", errors)
    _must_contain(gen, "odom_dth_ = 0.0;", "gen relocalize clear odom dth", errors)
    _must_contain(gen, "provider_last_valid_ = false;", "gen relocalize clear provider", errors)
    _must_contain(runtime, "odom_dx_ = 0.0;", "runtime relocalize clear odom dx", errors)
    _must_contain(runtime, "odom_dy_ = 0.0;", "runtime relocalize clear odom dy", errors)
    _must_contain(runtime, "odom_dth_ = 0.0;", "runtime relocalize clear odom dth", errors)
    _must_contain(runtime, "provider_last_valid_ = false;", "runtime relocalize clear provider", errors)

    _must_contain(gen, "MCLPose est = estimate_;", "gen prior est snapshot", errors)
    _must_contain(gen, "if (have_est) est = estimate();", "gen prior est compute", errors)
    _must_contain(localizer, "MCLPose est = estimate_;", "localizer prior est snapshot", errors)
    _must_contain(localizer, "if (have_est) est = estimate();", "localizer prior est compute", errors)

    _must_contain_in_order(gen, "provider_cb = pose_provider_;", "provider_sample_ok = provider_cb(&provider_sample, provider_user);", "gen provider sampled after unlocked copy", errors)
    _must_contain_in_order(runtime, "provider_cb = pose_provider_;", "provider_sample_ok = provider_cb(&provider_sample, provider_user);", "runtime provider sampled after unlocked copy", errors)

    _must_contain(gen, "if (do_sensor && imu_) {", "gen imu read pre-lock", errors)
    _must_contain(gen, "if (have_imu_heading) {", "gen imu apply captured", errors)
    _must_not_contain(gen, "if (imu_) {\n          double imu_heading = static_cast<double>(imu_->get_heading());", "gen imu read in lock", errors)
    _must_contain(runtime, "if (do_sensor && imu_) {", "runtime imu read pre-lock", errors)
    _must_contain(runtime, "if (have_imu_heading) {", "runtime imu apply captured", errors)
    _must_not_contain(runtime, "if (imu_) {\n          double imu_heading = static_cast<double>(imu_->get_heading());", "runtime imu read in lock", errors)

    _must_contain(tuning, "def _edge_indices(", "edge helper", errors)
    _must_contain(tuning, "mcl_ekf_apply_count = len(_edge_indices(", "edge mcl->ekf count", errors)
    _must_contain(tuning, "odom_corr_apply_count = len(_edge_indices(", "edge odom count", errors)

    _must_contain(tuning, "crc_calc = zlib.crc32(frame_blob)", "crc compute", errors)
    _must_contain(tuning, "if crc32_footer != 0 and crc_calc != crc32_footer:", "crc mismatch gate", errors)
    _must_contain(tuning, "CRC mismatch", "crc error message", errors)

    if errors:
        print("MCL regression checks FAILED:")
        for e in errors:
            print(f"- {e}")
        return 1
    print("MCL regression checks PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
