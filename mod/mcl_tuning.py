from __future__ import annotations

import math
import os
import json
import struct
import zlib
from dataclasses import dataclass
from statistics import fmean, pstdev
from typing import Dict, List, Optional


MAGIC = 0x31434C4D  # "MLC1"
VERSION = 1

EVENT_MCL_TO_EKF_APPLIED = 1 << 0
EVENT_ODOM_CORR_APPLIED = 1 << 1
EVENT_STEP_MARK = 1 << 2
EVENT_RECOVERY_ACTIVE = 1 << 3


class TuningLogError(ValueError):
    pass


@dataclass
class LogHeader:
    version: int
    header_size: int
    config_hash32: int
    sensor_count: int
    sensor_angles_deg: List[float]
    max_sensors: int


def _clamp(v: float, lo: float, hi: float) -> float:
    """Handle clamp."""
    return lo if v < lo else hi if v > hi else v


def _angle_diff_deg(a: float, b: float) -> float:
    """Handle angle diff deg."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def _stdev(vals: List[float]) -> float:
    """Handle stdev."""
    if len(vals) < 2:
        return 0.0
    return float(pstdev(vals))

def _circular_std_deg(vals: List[float]) -> float:
    """Handle circular std deg."""
    if len(vals) < 2:
        return 0.0
    s = 0.0
    c = 0.0
    for v in vals:
        r = math.radians(v)
        s += math.sin(r)
        c += math.cos(r)
    mean_deg = math.degrees(math.atan2(s, c))
    diffs = [_angle_diff_deg(v, mean_deg) for v in vals]
    return _stdev(diffs)


def _percentile(vals: List[float], p: float, default: float = 0.0) -> float:
    """Handle percentile."""
    if not vals:
        return float(default)
    s = sorted(vals)
    if len(s) == 1:
        return float(s[0])
    idx = _clamp(p, 0.0, 100.0) / 100.0 * (len(s) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(s[lo])
    t = idx - lo
    return float(s[lo] * (1.0 - t) + s[hi] * t)


def _edge_indices(frames: List[Dict], flag: int) -> List[int]:
    """Handle edge indices."""
    idx: List[int] = []
    prev = False
    for i, f in enumerate(frames):
        cur = bool(int(f.get("event_flags", 0)) & flag)
        if cur and not prev:
            idx.append(i)
        prev = cur
    return idx


def parse_mcl_log(path: str) -> Dict:
    """Handle parse mcl log."""
    if not path or not os.path.isfile(path):
        raise TuningLogError("Log file does not exist.")
    data = open(path, "rb").read()
    if len(data) < 24:
        raise TuningLogError("Log file is too small.")

    base = struct.Struct("<IHHIHH")
    magic, version, header_size, config_hash32, sensor_count, _ = base.unpack_from(data, 0)
    if magic != MAGIC:
        raise TuningLogError("Invalid log magic (expected MLC1).")
    if version != VERSION:
        raise TuningLogError(f"Unsupported log version: {version}.")
    if header_size < base.size or header_size > len(data) - 8:
        raise TuningLogError("Invalid log header size.")
    rem = header_size - base.size
    if rem % 4 != 0:
        raise TuningLogError("Corrupt header sensor angle payload.")
    max_sensors = rem // 4
    sensor_angles_mdeg = list(struct.unpack_from(f"<{max_sensors}i", data, base.size)) if max_sensors > 0 else []
    sensor_angles_deg = [v / 1000.0 for v in sensor_angles_mdeg]

    footer_struct = struct.Struct("<II")
    frame_count_footer, crc32_footer = footer_struct.unpack_from(data, len(data) - footer_struct.size)

    frame_struct = struct.Struct(f"<III15i{max_sensors}i{max_sensors}i")
    frame_blob = data[header_size:len(data) - footer_struct.size]
    if len(frame_blob) < 0 or (frame_struct.size > 0 and len(frame_blob) % frame_struct.size != 0):
        raise TuningLogError("Frame payload size mismatch.")
    frame_count = 0 if frame_struct.size == 0 else len(frame_blob) // frame_struct.size
    if frame_count_footer not in (0, frame_count):
        raise TuningLogError("Footer frame count does not match payload.")
    crc_calc = zlib.crc32(frame_blob) & 0xFFFFFFFF
    if crc32_footer != 0 and crc_calc != crc32_footer:
        raise TuningLogError(
            f"CRC mismatch (footer=0x{crc32_footer:08x}, computed=0x{crc_calc:08x})."
        )

    frames: List[Dict] = []
    off = 0
    for _ in range(frame_count):
        vals = frame_struct.unpack_from(frame_blob, off)
        off += frame_struct.size
        t_ms = vals[0]
        event_flags = vals[1]
        dist_used_mask = vals[2]
        ints = vals[3:18]
        dist_meas = vals[18:18 + max_sensors]
        dist_exp = vals[18 + max_sensors:18 + (2 * max_sensors)]
        frames.append({
            "t_ms": int(t_ms),
            "event_flags": int(event_flags),
            "dist_used_mask": int(dist_used_mask),
            "odom_x_in": ints[0] / 1000.0,
            "odom_y_in": ints[1] / 1000.0,
            "odom_th_deg": ints[2] / 1000.0,
            "mcl_x_in": ints[3] / 1000.0,
            "mcl_y_in": ints[4] / 1000.0,
            "mcl_th_deg": ints[5] / 1000.0,
            "fused_x_in": ints[6] / 1000.0,
            "fused_y_in": ints[7] / 1000.0,
            "fused_th_deg": ints[8] / 1000.0,
            "ekf_pxx": ints[9] / 1_000_000.0,
            "ekf_pyy": ints[10] / 1_000_000.0,
            "ekf_pxy": ints[11] / 1_000_000.0,
            "ekf_ptt": ints[12] / 1_000_000.0,
            "mcl_conf": ints[13] / 1000.0,
            "mcl_neff": ints[14] / 1000.0,
            "dist_meas_mm": [float(v) for v in dist_meas],
            "dist_exp_mm": [float(v) for v in dist_exp],
        })

    return {
        "header": LogHeader(
            version=int(version),
            header_size=int(header_size),
            config_hash32=int(config_hash32),
            sensor_count=max(0, min(int(sensor_count), max_sensors)),
            sensor_angles_deg=sensor_angles_deg,
            max_sensors=max_sensors,
        ),
        "frames": frames,
        "frame_count_footer": int(frame_count_footer),
        "crc32_footer": int(crc32_footer),
        "crc32_computed": int(crc_calc),
    }


def analyze_mcl_log(path: str, expected_config_hash32: Optional[int] = None) -> Dict:
    """Handle analyze mcl log."""
    parsed = parse_mcl_log(path)
    header: LogHeader = parsed["header"]
    frames: List[Dict] = parsed["frames"]
    if not frames:
        raise TuningLogError("Log has no frames.")
    if expected_config_hash32 is not None and int(header.config_hash32) != int(expected_config_hash32):
        raise TuningLogError(
            f"Config hash mismatch (log=0x{header.config_hash32:08x}, current=0x{int(expected_config_hash32):08x})."
        )

    sensor_count = header.sensor_count
    start_ms = frames[0]["t_ms"]
    end_ms = frames[-1]["t_ms"]
    duration_s = max(0.0, (end_ms - start_ms) / 1000.0)

    step_idxs = _edge_indices(frames, EVENT_STEP_MARK)
    recovery_idxs = _edge_indices(frames, EVENT_RECOVERY_ACTIVE)

    still_a = step_idxs[1] if len(step_idxs) >= 3 else 0
    still_b = step_idxs[2] if len(step_idxs) >= 3 else min(len(frames), still_a + max(1, len(frames) // 5))
    still = frames[still_a:still_b] if still_b > still_a else frames[:max(2, len(frames) // 5)]

    still_headings = [float(f["odom_th_deg"]) for f in still]
    still_heading_jitter_deg = _circular_std_deg(still_headings)

    dist_valid = 0
    dist_total = 0
    used_residuals: List[float] = []
    all_residuals: List[float] = []
    for f in frames:
        for s in range(sensor_count):
            dist_total += 1
            meas = f["dist_meas_mm"][s]
            exp = f["dist_exp_mm"][s]
            if meas < 0.0 or exp < 0.0:
                continue
            dist_valid += 1
            resid = meas - exp
            all_residuals.append(resid)
            if f["dist_used_mask"] & (1 << s):
                used_residuals.append(resid)

    residuals = used_residuals if used_residuals else all_residuals
    dist_resid_sigma = _stdev(residuals)
    dist_resid_mean = float(fmean(residuals)) if residuals else 0.0
    outlier_thr = max(40.0, 3.0 * max(1.0, dist_resid_sigma))
    outlier_frac = (sum(1 for r in residuals if abs(r) > outlier_thr) / len(residuals)) if residuals else 0.0
    dropout_frac = 1.0 - (float(dist_valid) / float(dist_total)) if dist_total > 0 else 1.0

    mcl_conf_vals = [float(f["mcl_conf"]) for f in frames]
    mcl_conf_med = _percentile(mcl_conf_vals, 50, 0.0)
    mcl_conf_p10 = _percentile(mcl_conf_vals, 10, 0.0)
    mcl_conf_p90 = _percentile(mcl_conf_vals, 90, 0.0)

    mcl_fused_xy = []
    mcl_fused_th = []
    odom_fused_xy = []
    odom_fused_th = []
    for f in frames:
        dx_mf = f["mcl_x_in"] - f["fused_x_in"]
        dy_mf = f["mcl_y_in"] - f["fused_y_in"]
        mcl_fused_xy.append(math.hypot(dx_mf, dy_mf))
        mcl_fused_th.append(abs(_angle_diff_deg(f["mcl_th_deg"], f["fused_th_deg"])))
        dx_of = f["odom_x_in"] - f["fused_x_in"]
        dy_of = f["odom_y_in"] - f["fused_y_in"]
        odom_fused_xy.append(math.hypot(dx_of, dy_of))
        odom_fused_th.append(abs(_angle_diff_deg(f["odom_th_deg"], f["fused_th_deg"])))

    mcl_ekf_apply_count = len(_edge_indices(frames, EVENT_MCL_TO_EKF_APPLIED))
    odom_corr_apply_count = len(_edge_indices(frames, EVENT_ODOM_CORR_APPLIED))

    kidnapped_start = recovery_idxs[0] if recovery_idxs else None
    kidnapped_placed = None
    if kidnapped_start is not None:
        for idx in step_idxs:
            if idx > kidnapped_start:
                kidnapped_placed = idx
                break
    conv_time_s: Optional[float] = None
    if kidnapped_placed is not None:
        target_conf = 0.6
        target_xy = max(3.0, _percentile(mcl_fused_xy, 75, 3.0))
        target_th = max(8.0, _percentile(mcl_fused_th, 75, 8.0))
        good = 0
        t0 = frames[kidnapped_placed]["t_ms"]
        for f in frames[kidnapped_placed:]:
            ok = (
                f["mcl_conf"] >= target_conf
                and math.hypot(f["mcl_x_in"] - f["fused_x_in"], f["mcl_y_in"] - f["fused_y_in"]) <= target_xy
                and abs(_angle_diff_deg(f["mcl_th_deg"], f["fused_th_deg"])) <= target_th
            )
            good = good + 1 if ok else 0
            if good >= 10:
                conv_time_s = max(0.0, (f["t_ms"] - t0) / 1000.0)
                break

    imu_sigma = _clamp(max(0.6, still_heading_jitter_deg * 3.0), 0.6, 3.0)
    dist_sigma = _clamp(max(4.0, dist_resid_sigma), 4.0, 35.0)
    if dropout_frac > 0.20 or outlier_frac > 0.25:
        w_rand = 0.12
    elif dropout_frac > 0.10 or outlier_frac > 0.15:
        w_rand = 0.08
    else:
        w_rand = 0.05
    w_hit = _clamp(1.0 - w_rand, 0.70, 0.97)
    gate_mm = _clamp(max(80.0, dist_sigma * 4.5), 80.0, 260.0)
    stable_lock = (mcl_conf_med >= 0.70 and dropout_frac < 0.20 and outlier_frac < 0.15)
    innov_mm = _clamp(dist_sigma * 3.0, 0.0, 200.0) if stable_lock else 0.0

    mcl_min_conf = 0.60
    if conv_time_s is None and kidnapped_start is not None:
        mcl_min_conf = 0.55
    elif conv_time_s is not None and conv_time_s < 2.5 and outlier_frac < 0.12:
        mcl_min_conf = 0.65
    elif conv_time_s is not None and conv_time_s > 6.0:
        mcl_min_conf = 0.58

    sigma_dx = _clamp(0.08 + _percentile(odom_fused_xy, 75, 1.0) * 0.03, 0.08, 0.60)
    sigma_dy = _clamp(0.08 + _percentile(odom_fused_xy, 75, 1.0) * 0.03, 0.08, 0.60)
    sigma_dth = _clamp(0.70 + _percentile(odom_fused_th, 75, 5.0) * 0.06, 0.60, 4.00)
    mcl_gate_xy = _clamp(max(2.0, _percentile(mcl_fused_xy, 95, 4.0) * 1.25), 2.0, 14.0)
    mcl_gate_th = _clamp(max(4.0, _percentile(mcl_fused_th, 95, 10.0) * 1.25), 4.0, 30.0)
    track_min_conf = _clamp(mcl_min_conf + 0.12, 0.65, 0.9)
    track_max_xy = _clamp(_percentile(mcl_fused_xy, 70, 1.8) * 1.15, 1.0, 4.0)
    track_max_th = _clamp(_percentile(mcl_fused_th, 70, 6.0) * 1.2, 3.0, 12.0)
    intervene_xy = _clamp(max(track_max_xy * 1.8, _percentile(mcl_fused_xy, 90, 4.0)), 2.0, 16.0)
    intervene_th = _clamp(max(track_max_th * 1.8, _percentile(mcl_fused_th, 90, 10.0)), 4.0, 35.0)
    pxx_vals = [float(f["ekf_pxx"]) for f in frames]
    pyy_vals = [float(f["ekf_pyy"]) for f in frames]
    ptt_vals = [float(f["ekf_ptt"]) for f in frames]
    intervene_p_xy = _clamp(max(_percentile(pxx_vals, 90, 4.0), _percentile(pyy_vals, 90, 4.0)), 1.0, 100.0)
    intervene_p_th = _clamp(_percentile(ptt_vals, 90, 60.0), 10.0, 900.0)
    hard_xy = _clamp(intervene_xy * 2.5, intervene_xy + 2.0, 30.0)
    hard_th = _clamp(intervene_th * 2.5, intervene_th + 6.0, 80.0)
    ambiguous_conf = _clamp(mcl_min_conf * 0.7, 0.2, 0.7)
    recover_conf = _clamp(mcl_min_conf, ambiguous_conf + 0.05, 0.95)

    recommendations = {
        "imu_sigma_deg": round(imu_sigma, 3),
        "dist_sigma_hit_mm": round(dist_sigma, 3),
        "dist_w_hit": round(w_hit, 3),
        "dist_w_rand": round(w_rand, 3),
        "dist_gate_mm": round(gate_mm, 3),
        "dist_innovation_gate_mm": round(innov_mm, 3),
        "ekf_sigma_dx_in": round(sigma_dx, 4),
        "ekf_sigma_dy_in": round(sigma_dy, 4),
        "ekf_sigma_dtheta_deg": round(sigma_dth, 4),
        "ekf_mcl_min_conf": round(mcl_min_conf, 3),
        "ekf_mcl_innovation_gate_xy_in": round(mcl_gate_xy, 3),
        "ekf_mcl_innovation_gate_theta_deg": round(mcl_gate_th, 3),
        "ekf_ambiguous_conf": round(ambiguous_conf, 3),
        "ekf_recover_conf": round(recover_conf, 3),
        "ekf_recover_stable_updates": 3,
        "ekf_fusion_mode": 2,
        "ekf_track_correction_enabled": 1,
        "ekf_track_min_conf": round(track_min_conf, 3),
        "ekf_track_max_trans_in": round(track_max_xy, 3),
        "ekf_track_max_theta_deg": round(track_max_th, 3),
        "ekf_intervene_min_conf": round(recover_conf, 3),
        "ekf_intervene_trans_in": round(intervene_xy, 3),
        "ekf_intervene_theta_deg": round(intervene_th, 3),
        "ekf_intervene_p_xy_in2": round(intervene_p_xy, 3),
        "ekf_intervene_p_theta_deg2": round(intervene_p_th, 3),
        "ekf_intervene_hard_reset_trans_in": round(hard_xy, 3),
        "ekf_intervene_hard_reset_theta_deg": round(hard_th, 3),
        "ekf_intervene_cooldown_ms": 250,
        "ekf_intervene_use_reset": 1,
        "ekf_recover_use_reset": 1,
        "corr_alpha_min": 0.03,
        "corr_alpha_max": 0.12,
        "corr_min_conf": round(mcl_min_conf, 3),
        "tuning_enabled": 1,
        "tuning_log_rate_hz": 20,
        "tuning_particle_subsample": 0,
    }

    score = 100.0
    score -= min(35.0, dropout_frac * 120.0)
    score -= min(25.0, outlier_frac * 100.0)
    score -= min(15.0, still_heading_jitter_deg * 6.0)
    if kidnapped_start is not None:
        score -= 20.0 if conv_time_s is None else min(20.0, max(0.0, conv_time_s - 2.5) * 4.0)
    score = _clamp(score, 0.0, 100.0)
    grade = "A" if score >= 90 else "B" if score >= 80 else "C" if score >= 70 else "D"

    checks = {
        "crc_ok": True if parsed.get("crc32_footer", 0) != 0 else False,
        "session_duration_ok": duration_s >= 20.0,
        "step_markers_ok": len(step_idxs) >= 3,
        "sensor_coverage_ok": dropout_frac < 0.25,
        "residual_quality_ok": abs(dist_resid_mean) < 25.0 and dist_resid_sigma < 60.0,
        "kidnapped_recovery_ok": True if kidnapped_start is None else (conv_time_s is not None and conv_time_s <= 8.0),
    }

    return {
        "path": path,
        "header": {
            "version": header.version,
            "config_hash32": header.config_hash32,
            "sensor_count": sensor_count,
            "sensor_angles_deg": header.sensor_angles_deg[:sensor_count],
        },
        "summary": {
            "frames": len(frames),
            "duration_s": duration_s,
            "score": round(score, 1),
            "grade": grade,
            "step_markers": len(step_idxs),
            "dropout_frac": dropout_frac,
            "dist_residual_mean_mm": dist_resid_mean,
            "dist_residual_sigma_mm": dist_resid_sigma,
            "outlier_frac": outlier_frac,
            "still_heading_jitter_deg": still_heading_jitter_deg,
            "mcl_conf_p10": mcl_conf_p10,
            "mcl_conf_med": mcl_conf_med,
            "mcl_conf_p90": mcl_conf_p90,
            "mcl_ekf_apply_count": mcl_ekf_apply_count,
            "odom_corr_apply_count": odom_corr_apply_count,
            "kidnapped_convergence_s": conv_time_s,
        },
        "checks": checks,
        "recommendations": recommendations,
    }


def format_analysis_text(analysis: Dict) -> str:
    """Handle format analysis text."""
    s = analysis["summary"]
    conv = s["kidnapped_convergence_s"]
    conv_text = "n/a" if conv is None else f"{conv:.2f}s"
    lines = [
        f"File: {os.path.basename(analysis['path'])}",
        f"Score: {s['score']:.1f}/100 ({s['grade']})",
        f"Duration: {s['duration_s']:.1f}s, Frames: {s['frames']}, Step markers: {s['step_markers']}",
        f"Distance residual: mean {s['dist_residual_mean_mm']:.1f} mm, sigma {s['dist_residual_sigma_mm']:.1f} mm",
        f"Dropout: {s['dropout_frac'] * 100.0:.1f}%, Outliers: {s['outlier_frac'] * 100.0:.1f}%",
        f"Still heading jitter: {s['still_heading_jitter_deg']:.3f} deg/tick",
        f"MCL confidence p10/med/p90: {s['mcl_conf_p10']:.2f}/{s['mcl_conf_med']:.2f}/{s['mcl_conf_p90']:.2f}",
        f"Kidnapped convergence: {conv_text}",
    ]
    return "\n".join(lines)


def build_tuning_report(analysis: Dict) -> str:
    """Build tuning report."""
    s = analysis["summary"]
    checks = analysis["checks"]
    rec = analysis["recommendations"]
    conv = s["kidnapped_convergence_s"]
    conv_text = "n/a" if conv is None else f"{conv:.2f}s"
    return (
        "# MCL/EKF Tuning Report\n\n"
        f"- Score: **{s['score']:.1f}/100 ({s['grade']})**\n"
        f"- Duration: {s['duration_s']:.1f}s ({s['frames']} frames)\n"
        f"- Step markers: {s['step_markers']}\n"
        f"- Config hash32: `0x{analysis['header']['config_hash32']:08x}`\n\n"
        "## Session Metrics\n"
        f"- Distance residual mean: {s['dist_residual_mean_mm']:.2f} mm\n"
        f"- Distance residual sigma: {s['dist_residual_sigma_mm']:.2f} mm\n"
        f"- Distance dropout fraction: {s['dropout_frac']:.3f}\n"
        f"- Distance outlier fraction: {s['outlier_frac']:.3f}\n"
        f"- Still heading jitter: {s['still_heading_jitter_deg']:.4f} deg/tick\n"
        f"- MCL confidence p10/med/p90: {s['mcl_conf_p10']:.3f} / {s['mcl_conf_med']:.3f} / {s['mcl_conf_p90']:.3f}\n"
        f"- MCL->EKF updates applied: {s['mcl_ekf_apply_count']}\n"
        f"- Odom corrections applied: {s['odom_corr_apply_count']}\n"
        f"- Kidnapped convergence: {conv_text}\n\n"
        "## Check Status\n"
        f"- Session duration: {'PASS' if checks['session_duration_ok'] else 'FAIL'}\n"
        f"- Log CRC: {'PASS' if checks['crc_ok'] else 'WARN (legacy/no CRC)'}\n"
        f"- Step markers: {'PASS' if checks['step_markers_ok'] else 'FAIL'}\n"
        f"- Sensor coverage: {'PASS' if checks['sensor_coverage_ok'] else 'FAIL'}\n"
        f"- Residual quality: {'PASS' if checks['residual_quality_ok'] else 'FAIL'}\n"
        f"- Kidnapped recovery: {'PASS' if checks['kidnapped_recovery_ok'] else 'FAIL'}\n\n"
        "## Recommended Config Updates\n"
        "```json\n"
        f"{_json_like_block(rec)}\n"
        "```\n"
    )


def _json_like_block(d: Dict) -> str:
    """Handle json like block."""
    return json.dumps(d, indent=2, sort_keys=True)


def apply_recommendations(cfg: Dict, rec: Dict) -> None:
    """Handle apply recommendations."""
    mcl = cfg.setdefault("mcl", {})
    sensors = mcl.setdefault("sensors", {})
    dist = sensors.setdefault("distance", {})
    imu = sensors.setdefault("imu", {})
    ekf = mcl.setdefault("ekf", {})
    corr = mcl.setdefault("correction", {})
    tuning = mcl.setdefault("tuning", {})

    imu["sigma_deg"] = float(rec.get("imu_sigma_deg", imu.get("sigma_deg", 1.0)))
    dist["sigma_hit_mm"] = float(rec.get("dist_sigma_hit_mm", dist.get("sigma_hit_mm", 8.5)))
    dist["w_hit"] = float(rec.get("dist_w_hit", dist.get("w_hit", 0.9)))
    dist["w_rand"] = float(rec.get("dist_w_rand", dist.get("w_rand", 0.1)))
    dist["gate_mm"] = float(rec.get("dist_gate_mm", dist.get("gate_mm", 150.0)))
    dist["innovation_gate_mm"] = float(rec.get("dist_innovation_gate_mm", dist.get("innovation_gate_mm", 0.0)))

    ekf["sigma_dx_in"] = float(rec.get("ekf_sigma_dx_in", ekf.get("sigma_dx_in", 0.1275)))
    ekf["sigma_dy_in"] = float(rec.get("ekf_sigma_dy_in", ekf.get("sigma_dy_in", 0.1275)))
    ekf["sigma_dtheta_deg"] = float(rec.get("ekf_sigma_dtheta_deg", ekf.get("sigma_dtheta_deg", 1.0)))
    ekf["mcl_min_conf"] = float(rec.get("ekf_mcl_min_conf", ekf.get("mcl_min_conf", 0.6)))
    ekf["mcl_innovation_gate_xy_in"] = float(
        rec.get("ekf_mcl_innovation_gate_xy_in", ekf.get("mcl_innovation_gate_xy_in", 0.0))
    )
    ekf["mcl_innovation_gate_theta_deg"] = float(
        rec.get("ekf_mcl_innovation_gate_theta_deg", ekf.get("mcl_innovation_gate_theta_deg", 0.0))
    )
    ekf["ambiguous_conf"] = float(rec.get("ekf_ambiguous_conf", ekf.get("ambiguous_conf", 0.35)))
    ekf["recover_conf"] = float(rec.get("ekf_recover_conf", ekf.get("recover_conf", 0.6)))
    ekf["recover_stable_updates"] = int(rec.get("ekf_recover_stable_updates", ekf.get("recover_stable_updates", 3)))
    ekf["fusion_mode"] = int(rec.get("ekf_fusion_mode", ekf.get("fusion_mode", 2)))
    ekf["track_correction_enabled"] = int(rec.get("ekf_track_correction_enabled", ekf.get("track_correction_enabled", 1)))
    ekf["track_min_conf"] = float(rec.get("ekf_track_min_conf", ekf.get("track_min_conf", 0.75)))
    ekf["track_max_trans_in"] = float(rec.get("ekf_track_max_trans_in", ekf.get("track_max_trans_in", 2.0)))
    ekf["track_max_theta_deg"] = float(rec.get("ekf_track_max_theta_deg", ekf.get("track_max_theta_deg", 5.0)))
    ekf["intervene_min_conf"] = float(rec.get("ekf_intervene_min_conf", ekf.get("intervene_min_conf", 0.7)))
    ekf["intervene_trans_in"] = float(rec.get("ekf_intervene_trans_in", ekf.get("intervene_trans_in", 4.0)))
    ekf["intervene_theta_deg"] = float(rec.get("ekf_intervene_theta_deg", ekf.get("intervene_theta_deg", 8.0)))
    ekf["intervene_p_xy_in2"] = float(rec.get("ekf_intervene_p_xy_in2", ekf.get("intervene_p_xy_in2", 9.0)))
    ekf["intervene_p_theta_deg2"] = float(rec.get("ekf_intervene_p_theta_deg2", ekf.get("intervene_p_theta_deg2", 100.0)))
    ekf["intervene_hard_reset_trans_in"] = float(
        rec.get("ekf_intervene_hard_reset_trans_in", ekf.get("intervene_hard_reset_trans_in", 12.0))
    )
    ekf["intervene_hard_reset_theta_deg"] = float(
        rec.get("ekf_intervene_hard_reset_theta_deg", ekf.get("intervene_hard_reset_theta_deg", 25.0))
    )
    ekf["intervene_cooldown_ms"] = int(rec.get("ekf_intervene_cooldown_ms", ekf.get("intervene_cooldown_ms", 250)))
    ekf["intervene_use_reset"] = int(rec.get("ekf_intervene_use_reset", ekf.get("intervene_use_reset", 1)))
    ekf["recover_use_reset"] = int(rec.get("ekf_recover_use_reset", ekf.get("recover_use_reset", 1)))

    corr["min_confidence"] = float(rec.get("corr_min_conf", corr.get("min_confidence", 0.6)))
    corr["alpha_min"] = float(rec.get("corr_alpha_min", corr.get("alpha_min", 0.03)))
    corr["alpha_max"] = float(rec.get("corr_alpha_max", corr.get("alpha_max", 0.12)))

    tuning["enabled"] = int(rec.get("tuning_enabled", tuning.get("enabled", 1)))
    tuning["log_rate_hz"] = int(rec.get("tuning_log_rate_hz", tuning.get("log_rate_hz", 20)))
    tuning["particle_subsample"] = int(
        rec.get("tuning_particle_subsample", tuning.get("particle_subsample", 0))
    )
