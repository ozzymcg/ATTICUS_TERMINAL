"""
Code generation module for PROS + LemLib with path file export.
Handles both straight segments and curved path segments.
"""

import os
import math
import re
from typing import List, Tuple, Dict
from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT, DEFAULT_CONFIG
from .pathing import export_lemlib_path, generate_path_asset_name
from .sim import _move_total_time, turn_time, path_time_with_curvature, vmax_straight, turn_rate, PROFILE_SPEED_SCALE, _cmd_to_ips, _sample_quad_bezier
from .geom import convert_heading_input, field_coords_in, interpret_input_angle
from .util import pros_convert_inches

SETTLE_BASE = {
    "drive": {
        "precise": {"err_min": 0.5, "err_max": 1.2, "t_min": 220, "t_max": 400},
        "normal":  {"err_min": 0.8, "err_max": 1.6, "t_min": 180, "t_max": 320},
        "custom":  {"err_min": 0.8, "err_max": 1.6, "t_min": 180, "t_max": 320},
        "fast":    {"err_min": 1.0, "err_max": 2.0, "t_min": 150, "t_max": 260},
        "slam":    {"err_min": 1.5, "err_max": 2.0, "t_min": 120, "t_max": 200},
    },
    "turn": {
        "precise": {"err_min": 0.5, "err_max": 1.0, "t_min": 200, "t_max": 380},
        "normal":  {"err_min": 0.8, "err_max": 1.5, "t_min": 170, "t_max": 320},
        "custom":  {"err_min": 0.8, "err_max": 1.5, "t_min": 170, "t_max": 320},
        "fast":    {"err_min": 1.0, "err_max": 2.0, "t_min": 150, "t_max": 280},
        "slam":    {"err_min": 1.5, "err_max": 2.5, "t_min": 120, "t_max": 220},
    },
    "swing": {
        "precise": {"err_min": 0.6, "err_max": 1.2, "t_min": 170, "t_max": 320},
        "normal":  {"err_min": 0.9, "err_max": 1.6, "t_min": 150, "t_max": 260},
        "custom":  {"err_min": 0.9, "err_max": 1.6, "t_min": 150, "t_max": 260},
        "fast":    {"err_min": 1.2, "err_max": 2.0, "t_min": 120, "t_max": 220},
        "slam":    {"err_min": 1.6, "err_max": 2.6, "t_min": 100, "t_max": 180},
    }
}

VOLTAGE_SHAPES = {
    "drive": {
        "precise": (4.0, 6.0, 8.0),
        "normal":  (5.0, 7.0, 10.0),
        "custom":  (5.0, 7.0, 10.0),
        "fast":    (6.0, 8.5, 12.0),
        "slam":    (7.0, 9.0, 12.0),
    },
    "turn": {
        "precise": (3.5, 5.5, 8.0),
        "normal":  (4.5, 6.5, 10.0),
        "custom":  (4.5, 6.5, 10.0),
        "fast":    (5.0, 7.5, 11.0),
        "slam":    (6.0, 9.0, 12.0),
    },
    "swing": {
        "precise": (3.0, 5.0, 7.0),
        "normal":  (4.0, 6.0, 9.0),
        "custom":  (4.0, 6.0, 9.0),
        "fast":    (4.5, 7.0, 10.0),
        "slam":    (5.5, 8.5, 12.0),
    }
}

DEFAULT_SWING_MIN_SPEED = 40
DEFAULT_SWING_EARLY_EXIT = 7.0


def _timeout_tokens(duration_s: float, pad_factor: float, min_s: float):
    """Calculate timeout tokens for code export."""
    raw_s = max(0.0, float(duration_s))
    pad_s = raw_s * pad_factor
    fin_s = max(min_s, pad_s)
    fin_ms = math.ceil(fin_s * 1000.0)
    return int(fin_ms), round(fin_s, 6)

def _profile_speed_scale(name: str) -> float:
    """Handle profile speed scale."""
    if name is None:
        return 1.0
    try:
        key = str(name).strip().lower()
    except Exception:
        return 1.0
    return float(PROFILE_SPEED_SCALE.get(key, 1.0))

def _pose_curve_time_s(seg: dict, cfg: dict, profile: str) -> float:
    """Handle pose curve time s."""
    p0 = seg.get("p0")
    p1 = seg.get("p1")
    if p0 is None or p1 is None:
        return 0.0
    try:
        dist_px = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    except Exception:
        return 0.0
    if dist_px <= 1e-6:
        return 0.0
    pose_heading = None
    if seg.get("pose_heading") is not None:
        try:
            pose_heading = float(seg.get("pose_heading"))
        except Exception:
            pose_heading = None
        if pose_heading is not None and seg.get("reverse", False):
            pose_heading = (pose_heading + 180.0) % 360.0
    if pose_heading is None:
        pose_heading = seg.get("facing", seg.get("target_heading"))
    if pose_heading is None:
        return 0.0
    try:
        lead_in = float(seg.get("pose_lead_in", 0.0) or 0.0)
    except Exception:
        lead_in = 0.0
    lead_in = max(0.0, lead_in)
    disp_heading = convert_heading_input(pose_heading, None)
    th = math.radians(disp_heading)
    carrot = (
        p1[0] + math.cos(th) * dist_px * lead_in,
        p1[1] + math.sin(th) * dist_px * lead_in
    )
    samples = max(20, int((dist_px / PPI) * 4.0))
    path_points = _sample_quad_bezier(p0, carrot, p1, num=samples)
    speed_mult = _profile_speed_scale(profile)
    max_override = seg.get("drive_speed_cmd")
    if max_override is None and seg.get("drive_speed_ips") is not None:
        try:
            vmax_cfg = vmax_straight(cfg)
            if vmax_cfg > 1e-6:
                max_override = float(seg.get("drive_speed_ips")) / vmax_cfg * 127.0
        except Exception:
            max_override = None
    try:
        _, t_move, *_rest = path_time_with_curvature(
            path_points,
            cfg,
            speed_mult=speed_mult,
            min_speed_override=None,
            max_speed_override=max_override
        )
        return float(t_move or 0.0)
    except Exception:
        return 0.0

def _physics_timeout_s(seg: dict, move_type: str, cfg: dict, motion_mode: str = None) -> float:
    """Compute a conservative physics-based travel time for timeout."""
    if move_type in ("wait", "reshape"):
        return 0.0
    if move_type == "path":
        path_points = seg.get("path_points")
        if not path_points:
            return 0.0
        profile = seg.get("profile_override") or pick_profile("drive", abs(float(seg.get("path_length_in", 0.0) or 0.0)), cfg=cfg)
        speed_mult = _profile_speed_scale(profile)
        min_override = seg.get("min_speed_cmd")
        max_override = seg.get("max_speed_cmd")
        if max_override is None:
            max_override = seg.get("drive_speed_cmd")
        try:
            _, t_move, *_rest = path_time_with_curvature(
                path_points,
                cfg,
                speed_mult=speed_mult,
                min_speed_override=min_override,
                max_speed_override=max_override
            )
            return float(t_move or 0.0)
        except Exception:
            return 0.0
    if move_type in ("move", "pose"):
        length_in = seg.get("length_in")
        if length_in is None:
            p0 = seg.get("p0") or seg.get("start_pos")
            p1 = seg.get("p1") or seg.get("end_pos")
            if p0 is not None and p1 is not None:
                try:
                    length_in = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / PPI
                except Exception:
                    length_in = None
        if length_in is None:
            return 0.0
        profile = seg.get("profile_override") or pick_profile("drive", abs(float(length_in)), cfg=cfg)
        use_pose_curve = (move_type == "pose") or (move_type == "move" and str(motion_mode).lower() == "pose")
        if use_pose_curve:
            curve_t = _pose_curve_time_s(seg, cfg, profile)
            if curve_t > 0.0:
                return float(curve_t)
        v_override = None
        if seg.get("drive_speed_ips") is not None:
            try:
                v_override = float(seg.get("drive_speed_ips"))
            except Exception:
                v_override = None
        if v_override is None and seg.get("drive_speed_cmd") is not None:
            try:
                v_override = _cmd_to_ips(float(seg.get("drive_speed_cmd")), cfg)
            except Exception:
                v_override = None
        if v_override is None:
            v_override = vmax_straight(cfg) * _profile_speed_scale(profile)
        try:
            return float(_move_total_time(float(length_in), cfg, v_override=v_override))
        except Exception:
            return 0.0
    if move_type in ("face", "turn"):
        h0 = seg.get("start_heading", 0.0)
        h1 = seg.get("target_heading", seg.get("heading", 0.0))
        try:
            delta = ((float(h1) - float(h0) + 180.0) % 360.0) - 180.0
        except Exception:
            delta = 0.0
        profile = seg.get("profile_override") or pick_profile("turn", abs(delta), cfg=cfg)
        rate_override = seg.get("turn_speed_dps")
        if rate_override is None:
            rate_override = turn_rate(cfg) * _profile_speed_scale(profile)
        try:
            return float(turn_time(delta, cfg, rate_override=rate_override))
        except Exception:
            return 0.0
    if move_type == "swing":
        length_in = seg.get("length_in")
        if length_in is None:
            length_in = 0.0
        profile = seg.get("profile_override") or pick_profile("swing", abs(float(seg.get("delta_heading", 0.0) or 0.0)), cfg=cfg)
        v_override = None
        if seg.get("drive_speed_ips") is not None:
            try:
                v_override = float(seg.get("drive_speed_ips"))
            except Exception:
                v_override = None
        if v_override is None and seg.get("drive_speed_cmd") is not None:
            try:
                v_override = _cmd_to_ips(float(seg.get("drive_speed_cmd")), cfg)
            except Exception:
                v_override = None
        if v_override is None:
            v_override = vmax_straight(cfg) * _profile_speed_scale(profile)
        try:
            return float(_move_total_time(float(length_in), cfg, v_override=v_override))
        except Exception:
            return 0.0
    return 0.0

def _reshape_state_token(cfg, state) -> str:
    """Handle reshape state token."""
    mode = str(cfg.get("codegen", {}).get("opts", {}).get("reshape_output", "1/2")).strip().lower()
    if isinstance(state, bool):
        enabled = state
    elif isinstance(state, str):
        state_norm = state.strip().lower()
        if state_norm in ("true", "on", "yes", "enabled", "reshape", "reshaped"):
            enabled = True
        elif state_norm in ("false", "off", "no", "disabled", "normal"):
            enabled = False
        else:
            try:
                enabled = int(float(state_norm)) >= 2
            except Exception:
                enabled = False
    else:
        try:
            enabled = int(state) >= 2
        except Exception:
            enabled = False
    if mode in ("bool", "true/false", "truefalse", "true"):
        return "true" if enabled else "false"
    return "2" if enabled else "1"

MIN_CMD_TIMEOUT_S = 0.15
SETTLE_TIMEOUT_PAD_MS = 50


def _clamp(x, lo, hi):
    """Handle clamp."""
    return max(lo, min(hi, x))


def _lerp(a, b, t):
    """Handle lerp."""
    return a + (b - a) * t


def _map_piecewise(mag, x1, x2, v_small, v_mid, v_large):
    """Handle map piecewise."""
    if mag <= 0:
        return v_small
    if mag < x1:
        return _lerp(v_small, v_mid, mag / x1)
    if mag < x2:
        return _lerp(v_mid, v_large, (mag - x1) / (x2 - x1))
    return v_large


def _motion_profiles_cfg(cfg) -> dict:
    """Handle motion profiles cfg."""
    mp = cfg.get("codegen", {}).get("motion_profiles", {})
    return mp if isinstance(mp, dict) else {}


def _profile_override(section: dict, move_type: str, profile: str):
    """Handle profile override."""
    if not isinstance(section, dict):
        return None
    move_cfg = None
    for k, v in section.items():
        if str(k).lower() == str(move_type).lower():
            move_cfg = v
            break
    if not isinstance(move_cfg, dict):
        return None
    prof_cfg = None
    for k, v in move_cfg.items():
        if str(k).lower() == str(profile).lower():
            prof_cfg = v
            break
    if prof_cfg is None:
        for k, v in move_cfg.items():
            if str(k).lower() == "default":
                prof_cfg = v
                break
    return prof_cfg


def _settle_base_for(cfg, move_type: str, profile: str) -> dict:
    """Handle settle base for."""
    base = SETTLE_BASE[move_type][profile]
    mp = _motion_profiles_cfg(cfg)
    settle_cfg = mp.get("settle_base")
    prof_cfg = _profile_override(settle_cfg, move_type, profile)
    if not isinstance(prof_cfg, dict):
        return base
    out = dict(base)
    for key in ("err_min", "err_max", "t_min", "t_max"):
        if key in prof_cfg:
            try:
                out[key] = float(prof_cfg[key])
            except Exception:
                pass
    return out


def _voltage_shape_for(cfg, move_type: str, profile: str):
    """Handle voltage shape for."""
    base = VOLTAGE_SHAPES[move_type][profile]
    mp = _motion_profiles_cfg(cfg)
    volt_cfg = mp.get("voltage_shapes")
    prof_cfg = _profile_override(volt_cfg, move_type, profile)
    if prof_cfg is None:
        return base
    if isinstance(prof_cfg, (list, tuple)) and len(prof_cfg) >= 3:
        try:
            return (float(prof_cfg[0]), float(prof_cfg[1]), float(prof_cfg[2]))
        except Exception:
            return base
    if isinstance(prof_cfg, dict):
        def _val(keys, fallback):
            """Handle val."""
            for key in keys:
                if key in prof_cfg:
                    try:
                        return float(prof_cfg[key])
                    except Exception:
                        return fallback
            return fallback
        return (
            _val(("v_small", "small"), base[0]),
            _val(("v_mid", "mid"), base[1]),
            _val(("v_large", "large"), base[2]),
        )
    return base


def _compute_settle(cfg, move_type: str, magnitude: float, voltage_cap: float, profile: str):
    """Compute settle."""
    b = _settle_base_for(cfg, move_type, profile)
    if move_type == "drive":
        m = _clamp(magnitude / 48.0, 0.0, 1.0)
    else:
        m = _clamp(magnitude / 180.0, 0.0, 1.0)
    v = _clamp(voltage_cap / 12.0, 0.0, 1.0)
    err_t = _clamp(0.9 * m + 0.1 * v, 0.0, 1.0)
    err = _lerp(b["err_min"], b["err_max"], err_t)
    t = _lerp(b["t_min"], b["t_max"], 0.85 * m + 0.15 * v)
    if (move_type == "drive" and magnitude < 6.0) or (move_type != "drive" and magnitude < 15.0):
        t = max(t, b["t_min"])
    return (round(err, 2), int(round(t)))

def _calibration_active(cfg) -> bool:
    """Handle calibration active."""
    cal = cfg.get("codegen", {}).get("calibration", {})
    if not isinstance(cal, dict):
        return False
    enabled = cal.get("enabled", 0)
    if isinstance(enabled, dict):
        enabled = enabled.get("value", 0)
    return bool(enabled)

def _bucket_by_magnitude(move_type: str, magnitude: float) -> str:
    """Handle bucket by magnitude."""
    mag = abs(float(magnitude))
    if move_type == "drive":
        if mag < 12.0:
            return "small"
        if mag < 36.0:
            return "medium"
        return "large"
    if mag < 25.0:
        return "small"
    if mag < 120.0:
        return "medium"
    return "large"

def _match_key(d: dict, key: str):
    """Handle match key."""
    if not isinstance(d, dict):
        return None
    for k in d.keys():
        if str(k).lower() == str(key).lower():
            return k
    return None

def _nearest_numeric_key(d: dict, value: float):
    """Handle nearest numeric key."""
    best_key = None
    best_dist = None
    for k in d.keys():
        try:
            kval = float(k)
        except Exception:
            continue
        dist = abs(kval - value)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_key = k
    if best_key is not None:
        return best_key
    return _match_key(d, "default")

def _cal_section(cal: dict, move_type: str):
    """Handle cal section."""
    profiles = cal.get("profiles")
    if isinstance(profiles, dict) and move_type in profiles:
        return profiles.get(move_type)
    return cal.get(move_type)

def _calibration_bucket(cal: dict, move_type: str, profile: str, cap_frac: float, mag_bucket: str):
    """Handle calibration bucket."""
    sect = _cal_section(cal, move_type)
    if not isinstance(sect, dict):
        return None
    if any(k in sect for k in ("err_p90", "settle_ms_p90", "err", "settle_ms")):
        return sect
    prof_key = _match_key(sect, profile) or _match_key(sect, "default")
    prof = sect.get(prof_key) if prof_key else sect
    if not isinstance(prof, dict):
        return None
    if any(k in prof for k in ("err_p90", "settle_ms_p90", "err", "settle_ms")):
        return prof
    caps = prof.get("caps") if isinstance(prof.get("caps"), dict) else None
    if caps:
        cap_key = _nearest_numeric_key(caps, cap_frac)
        cap_bucket = caps.get(cap_key) if cap_key else caps
    else:
        cap_bucket = prof
    if not isinstance(cap_bucket, dict):
        return None
    if mag_bucket in cap_bucket:
        return cap_bucket.get(mag_bucket)
    if "default" in cap_bucket:
        return cap_bucket.get("default")
    if any(k in cap_bucket for k in ("err_p90", "settle_ms_p90", "err", "settle_ms")):
        return cap_bucket
    return None

def _compute_settle_calibrated(cfg, move_type: str, magnitude: float, voltage_cap: float, profile: str):
    """Compute settle calibrated."""
    cal = cfg.get("codegen", {}).get("calibration", {})
    if not isinstance(cal, dict):
        return None
    intent_err, intent_time = _compute_settle(cfg, move_type, magnitude, voltage_cap, profile)
    cap_frac = _clamp(voltage_cap / 12.0, 0.0, 1.0)
    mag_bucket = _bucket_by_magnitude(move_type, magnitude)
    bucket = _calibration_bucket(cal, move_type, profile, cap_frac, mag_bucket) or {}
    err_p90 = bucket.get("err_p90", bucket.get("err"))
    settle_ms_p90 = bucket.get("settle_ms_p90", bucket.get("settle_ms"))
    noise = cal.get("noise", {}) if isinstance(cal.get("noise", {}), dict) else {}
    if move_type == "drive":
        noise_floor = float(noise.get("drive_in", cal.get("noise_drive_in", 0.0)) or 0.0)
    else:
        noise_floor = float(noise.get("turn_deg", cal.get("noise_turn_deg", 0.0)) or 0.0)
    k_noise = float(cal.get("k_noise", 4.0) or 4.0)
    err_scale = float(cal.get("err_scale", 1.1) or 1.1)
    time_scale = float(cal.get("time_scale", 1.1) or 1.1)
    settle_err = float(intent_err)
    if err_p90 is not None:
        try:
            settle_err = max(settle_err, err_scale * float(err_p90))
        except Exception:
            pass
    if noise_floor > 0.0:
        settle_err = max(settle_err, k_noise * noise_floor)
    settle_time = float(intent_time)
    if settle_ms_p90 is not None:
        try:
            settle_time = max(settle_time, time_scale * float(settle_ms_p90))
        except Exception:
            pass
    return round(settle_err, 2), int(round(settle_time))

def _sensor_settle_scale(cfg, move_type: str):
    """Handle sensor settle scale."""
    rp = cfg.get("robot_physics", {})
    omni_raw = rp.get("all_omni", 0)
    if isinstance(omni_raw, dict):
        omni_raw = omni_raw.get("value", 0)
    omni = float(omni_raw)
    adv_raw = rp.get("advanced_motion", 0)
    if isinstance(adv_raw, dict):
        adv_raw = adv_raw.get("value", 0)
    adv_motion = bool(float(adv_raw))
    if not adv_motion:
        return 1.0, 1.0
    try:
        tracking_raw = rp.get("tracking_wheels", 0)
        if isinstance(tracking_raw, dict):
            tracking_raw = tracking_raw.get("value", 0)
        tracking = int(tracking_raw)
    except Exception:
        tracking = 0
    tracking = max(0, min(2, tracking))
    if not adv_motion:
        tracking = 0

    err_scale = 1.0
    time_scale = 1.0

    if omni:
        err_scale *= 1.06
        time_scale *= 1.05

    track_err = {0: 1.0, 1: 0.96, 2: 0.92}
    track_time = {0: 1.0, 1: 1.0, 2: 1.0}
    err_scale *= track_err.get(tracking, 0.92)
    time_scale *= track_time.get(tracking, 0.94)

    if move_type in ("turn", "swing"):
        err_scale = 1.0 + (err_scale - 1.0) * 0.5
        time_scale = 1.0 + (time_scale - 1.0) * 0.5

    err_scale = _clamp(err_scale, 0.80, 1.20)
    time_scale = _clamp(time_scale, 0.85, 1.20)
    return err_scale, time_scale

def _apply_sensor_settle(cfg, move_type: str, settle_err: float, settle_time: int):
    """Handle apply sensor settle."""
    err_scale, time_scale = _sensor_settle_scale(cfg, move_type)
    if err_scale == 1.0 and time_scale == 1.0:
        return settle_err, settle_time
    adj_err = max(0.05, float(settle_err) * err_scale)
    adj_time = max(0, int(round(float(settle_time) * time_scale)))
    return (round(adj_err, 2), adj_time)

def _apply_settle_adjustments(cfg, move_type: str, settle_err: float, settle_time: int):
    """Handle apply settle adjustments."""
    base_err = float(settle_err)
    base_time = float(settle_time)
    settle_err, settle_time = _apply_sensor_settle(cfg, move_type, settle_err, settle_time)
    min_err = max(0.05, base_err * 0.70)
    max_err = base_err * 1.25
    min_time = max(0.0, base_time * 0.75)
    max_time = base_time * 1.25
    settle_err = round(_clamp(float(settle_err), min_err, max_err), 2)
    settle_time = int(round(_clamp(float(settle_time), min_time, max_time)))
    return settle_err, settle_time

def _compute_settle_final(cfg, move_type: str, magnitude: float, voltage_cap: float, profile: str):
    """Compute settle final."""
    if _calibration_active(cfg):
        calibrated = _compute_settle_calibrated(cfg, move_type, magnitude, voltage_cap, profile)
        if calibrated:
            return calibrated
    settle_err, settle_time = _compute_settle(cfg, move_type, magnitude, voltage_cap, profile)
    return _apply_settle_adjustments(cfg, move_type, settle_err, settle_time)

def _ensure_timeout(tokens: dict, min_ms: int):
    """Handle ensure timeout."""
    try:
        min_ms = int(min_ms)
    except Exception:
        return
    if min_ms <= 0:
        return
    cur = int(tokens.get("TIMEOUT_MS", 0) or 0)
    if cur >= min_ms:
        return
    tokens["TIMEOUT_MS"] = min_ms
    tokens["MS"] = min_ms
    tokens["TIMEOUT_S"] = round(min_ms / 1000.0, 6)
    tokens["S"] = tokens["TIMEOUT_S"]

def _add_settle_timeout(tokens: dict, settle_ms: int, extra_ms: int):
    """Handle add settle timeout."""
    try:
        settle_ms = int(settle_ms)
        extra_ms = int(extra_ms)
    except Exception:
        return
    base_ms = int(tokens.get("TIMEOUT_MS", 0) or 0)
    target = base_ms + max(0, settle_ms) + max(0, extra_ms)
    _ensure_timeout(tokens, target)

def _scale_timeout(tokens: dict, speed_scale: float):
    """Handle scale timeout."""
    try:
        scale = float(speed_scale)
    except Exception:
        return
    if scale <= 0.0 or scale >= 1.0:
        return
    base_ms = int(tokens.get("TIMEOUT_MS", 0) or 0)
    if base_ms <= 0:
        return
    target = int(math.ceil(base_ms / max(1e-6, scale)))
    _ensure_timeout(tokens, target)


def _compute_voltage_cap(cfg, move_type: str, magnitude: float, profile: str):
    """Compute voltage cap."""
    v_small, v_mid, v_large = _voltage_shape_for(cfg, move_type, profile)
    if move_type == "drive":
        return round(_map_piecewise(magnitude, x1=6.0, x2=48.0, v_small=v_small, v_mid=v_mid, v_large=v_large), 2)
    return round(_map_piecewise(magnitude, x1=15.0, x2=90.0, v_small=v_small, v_mid=v_mid, v_large=v_large), 2)


def _compute_heading_cap(drive_cap: float, profile: str):
    """Compute heading cap."""
    frac = {"precise": 0.50, "normal": 0.60, "custom": 0.60, "fast": 0.65, "slam": 0.70}[profile]
    return round(_clamp(drive_cap * frac, 3.0, 9.0), 2)

def _num_value(v, default):
    """Handle num value."""
    if isinstance(v, dict):
        v = v.get("value", default)
    try:
        return float(v)
    except Exception:
        return float(default)

def _wheel_max_ips_from(rpm, diameter):
    """Handle wheel max ips from."""
    rpm_f = _num_value(rpm, 200.0)
    diam_f = max(0.01, _num_value(diameter, 4.0))
    return max(1e-6, rpm_f * math.pi * diam_f / 60.0)

def _default_speed_ref_ips():
    """Handle default speed ref ips."""
    try:
        rp = DEFAULT_CONFIG.get("robot_physics", {})
        return _wheel_max_ips_from(rp.get("rpm", 200.0), rp.get("diameter", 4.0))
    except Exception:
        return 60.0

_DEFAULT_SPEED_REF_IPS = _default_speed_ref_ips()

def _wheel_speed_scale(cfg) -> float:
    """Handle wheel speed scale."""
    rp = cfg.get("robot_physics", {})
    vmax_ips = _wheel_max_ips_from(rp.get("rpm", 200.0), rp.get("diameter", 4.0))
    opts = cfg.get("codegen", {}).get("opts", {})
    ref_ips = _num_value(opts.get("speed_ref_ips", _DEFAULT_SPEED_REF_IPS), _DEFAULT_SPEED_REF_IPS)
    ref_ips = max(1e-6, ref_ips)
    return vmax_ips / ref_ips


def _omni_drive_scale(cfg) -> float:
    """Handle omni drive scale."""
    rp = cfg.get("robot_physics", {})
    omni = float(rp.get("all_omni", 0))
    scale = 1.0
    if omni:
        scale *= 0.9
    return max(0.2, min(1.0, scale))


def _omni_turn_scale(cfg) -> float:
    """Handle omni turn scale."""
    rp = cfg.get("robot_physics", {})
    omni = float(rp.get("all_omni", 0))
    scale = 1.0
    if omni:
        scale *= 0.9
    return max(0.2, min(1.0, scale))


_PROFILE_RULES_DEFAULT = {
    "drive": {"precise_max": 12.0, "fast_min": 36.0},
    "turn": {"precise_max": 25.0, "fast_min": 120.0},
    "swing": {"precise_max": 25.0, "fast_min": 120.0},
}


def _profile_rules_for(cfg, move_type: str) -> dict:
    """Handle profile rules for."""
    base = dict(_PROFILE_RULES_DEFAULT.get(move_type, {}))
    mp = _motion_profiles_cfg(cfg)
    rules = mp.get("profile_rules")
    if not isinstance(rules, dict):
        return base
    user_rules = None
    for k, v in rules.items():
        if str(k).lower() == str(move_type).lower():
            user_rules = v
            break
    if not isinstance(user_rules, dict):
        return base
    for key in ("precise_max", "fast_min", "slam_min"):
        if key in user_rules:
            try:
                base[key] = float(user_rules[key])
            except Exception:
                pass
    return base


def pick_profile(move_type: str, magnitude: float, default_profile: str = "normal", cfg: dict = None) -> str:
    """
    Choose a motion profile label based on move size.
    
    Short moves favor "precise", long moves favor "fast"; otherwise defaults.
    """
    try:
        rules = _PROFILE_RULES_DEFAULT.get(move_type, {})
        if cfg is not None:
            rules = _profile_rules_for(cfg, move_type)
        precise_max = rules.get("precise_max")
        fast_min = rules.get("fast_min")
        slam_min = rules.get("slam_min")
        if precise_max is not None and magnitude < float(precise_max):
            return "precise"
        if slam_min is not None and magnitude >= float(slam_min):
            return "slam"
        if fast_min is not None and magnitude > float(fast_min):
            return "fast"
    except Exception:
        pass
    return default_profile


def build_export_lines_with_paths(cfg, timeline, routine_name="autonomous", initial_heading=None):
    """
    Build code export lines from timeline, including path file generation.
    
    For curved segments (type="path"), generates .path files and emits
    LemLib path-following calls. For straight segments, uses original logic.
    
    Args:
        cfg: Configuration dictionary
        timeline: Compiled timeline from compile_timeline()
        routine_name: Name for path file generation
    
    Returns:
        List of code lines, or None for "Action List" style
    """
    style_raw = cfg.get("codegen", {}).get("style", "Action List")
    if isinstance(style_raw, dict):
        style_raw = style_raw.get("value", "Action List")
    style = str(style_raw)
    if style == "EZ-Template":
        style = "JAR"
    if style.strip().lower() in ("action list", "actionlist", "list"):
        return None

    try:
        routine_name = str(routine_name)
    except Exception:
        routine_name = "autonomous"
    
    defaults = {
        "LemLib": {
            "wait": "task::sleep({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}});",
            "turn_global": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {{.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}});",
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {{.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}});",
            "swing": "chassis.swingToHeading({HEADING_DEG}, lemlib::DriveSide::{SIDE}, {TIMEOUT_MS}, {{.minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}});",
            "path_follow": "chassis.follow({PATH_ASSET}, {LOOKAHEAD}, {TIMEOUT_MS}, {FORWARDS});",
            "reshape_on": "// RESHAPE ON state={STATE}",
            "reshape_off": "// RESHAPE OFF state={STATE}",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled inline",
            "reverse_off": "// reverse handled inline",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "chassis.waitUntilDone();",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "JAR": {
            "wait": "pros::delay({MS});",
            "move": "chassis.drive_timeout = {TIMEOUT_MS};\nchassis.holonomic_drive_to_point({X_IN}, {Y_IN});",
            "turn_global": "chassis.turn_timeout = {TIMEOUT_MS};\nchassis.turn_to_angle({HEADING_DEG});",
            "turn_local": "chassis.turn_timeout = {TIMEOUT_MS};\nchassis.turn_to_angle({HEADING_DEG});",
            "pose": "chassis.drive_timeout = {TIMEOUT_MS};\nchassis.holonomic_drive_to_point({X_IN}, {Y_IN});\nchassis.turn_timeout = {TIMEOUT_MS};\nchassis.turn_to_angle({HEADING_DEG});",
            "pose_angle": "chassis.drive_timeout = {TIMEOUT_MS};\nchassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG});",
            "swing": "chassis.{SIDE}_swing_to_angle({HEADING_DEG});",
            "path_follow": 'followPath("{PATH_FILE}", {TIMEOUT_MS});',
            "reshape_on": "// RESHAPE ON state={STATE}",
            "reshape_off": "// RESHAPE OFF state={STATE}",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled inline",
            "reverse_off": "// reverse handled inline",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "",
            "marker_wait_done": "",
            "setpose": "chassis.set_coordinates({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "PROS": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS});",
            "turn_global": "turn_to({HEADING_DEG});",
            "turn_local": "turn_angle({TURN_DELTA_DEG});",
            "pose": "// move_to_pose x={X_IN}, y={Y_IN}, h={HEADING_DEG}, lead={LEAD_IN}",
            "swing": "swing_to({HEADING_DEG}, {DIR});",
            "path_follow": 'follow_path("{PATH_FILE}", {LOOKAHEAD});',
            "reshape_on": "// RESHAPE ON state={STATE}",
            "reshape_off": "// RESHAPE OFF state={STATE}",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "",
            "marker_wait_done": "",
            "setpose": "// set pose {X_IN},{Y_IN},{HEADING_DEG}"
        },
        "Custom": {
            "wait": "pros::delay({MS});",
            "move": "move({X_IN}, {Y_IN}, {HEADING_DEG});",
            "turn_global": "turn_to_heading({HEADING_DEG});",
            "turn_local": "turn_relative({TURN_DELTA_DEG});",
            "pose": "pose({X_IN}, {Y_IN}, {HEADING_DEG}, {LEAD_IN});",
            "swing": "swing_to_heading({HEADING_DEG}, {DIR});",
            "path_follow": 'follow_path("{PATH_NAME}", {TIMEOUT_MS}, {LOOKAHEAD});',
            "reshape_on": "// RESHAPE ON state={STATE}",
            "reshape_off": "// RESHAPE OFF state={STATE}",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "waitUntilDone();",
            "setpose": "setpose({X_IN},{Y_IN},{HEADING_DEG});"
        }
    }
    defaults["JAR (advanced)"] = dict(defaults["JAR"])
    defaults["JAR (advanced)"].update({
        "move": "chassis.drive_distance({DIST_IN}, {HEADING_DEG},{DRIVE_MAX_V},{HEADING_MAX_V},{DRIVE_SETTLE_ERR},{DRIVE_SETTLE_TIME}, {TIMEOUT_MS});",
        "turn_global": "chassis.turn_to_angle({HEADING_DEG}, {TURN_MAX_V}, {TIMEOUT_MS});",
        "turn_local": "turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS});",
        "pose": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "pose_angle": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "swing": "chassis.{SIDE}_swing_to_angle({HEADING_DEG}, {SWING_MAX_V}, {SWING_SETTLE_ERR}, {SWING_SETTLE_TIME}, {TIMEOUT_MS});",
        "path_follow": "followPath(\"{PATH_FILE}\", {TIMEOUT_MS});",
    })
    jar_like_style = style in ("JAR", "JAR (advanced)")
    tpls = dict(defaults.get(style, defaults["Custom"]))
    stored_tpl = cfg.get("codegen", {}).get("templates", {}).get(style, {})
    tpls.update(stored_tpl)
    if style == "LemLib" and "swing" not in tpls:
        tpls["swing"] = defaults["LemLib"]["swing"]
    optional_keys = ["reverse_on", "reverse_off", "reshape_on", "reshape_off", "setpose", "path_follow", "marker_wait", "marker_wait_done"]
    if style != "LemLib":
        optional_keys += ["pose", "swing"]
    active_opt = stored_tpl.get("__optional__", None)
    if isinstance(active_opt, list) and "reshape" in active_opt:
        active_opt = [opt for opt in active_opt if opt != "reshape"]
        if "reshape_on" not in active_opt:
            active_opt.append("reshape_on")
        if "reshape_off" not in active_opt:
            active_opt.append("reshape_off")
    if active_opt is not None:
        active_opt = [opt for opt in active_opt if opt in optional_keys]
        for opt in optional_keys:
            if opt not in active_opt and opt in tpls:
                tpls.pop(opt, None)
    legacy_reshape_tpl = stored_tpl.get("reshape") if isinstance(stored_tpl, dict) else None
    if legacy_reshape_tpl:
        if not tpls.get("reshape_on"):
            tpls["reshape_on"] = legacy_reshape_tpl
        if not tpls.get("reshape_off"):
            tpls["reshape_off"] = legacy_reshape_tpl
    modes = stored_tpl.get("__modes__", {}) if isinstance(stored_tpl, dict) else {}
    
    opts = cfg.get("codegen", {}).get("opts", {})
    ticks_per_rotation = float(opts.get("ticks_per_rotation", 360))
    pad_factor = float(opts.get("pad_factor", 1.0) or 1.0)
    min_s = float(opts.get("min_timeout_s", 0.0) or 0.0)
    motion_mode = modes.get("motion", "move")
    turn_mode = modes.get("turn", "turn_global" if style == "LemLib" else "turn_local")
    
    path_cfg = cfg.get("path_config", {})
    lookahead_in = float(path_cfg.get("lookahead_in", 15.0))
    min_speed_cmd = float(path_cfg.get("min_speed_cmd", path_cfg.get("min_speed_ips", 0.0)))
    max_speed_cmd = float(path_cfg.get("max_speed_cmd", path_cfg.get("max_speed_ips", 127.0)))
    
    def dist_conversions(p0, p1, reverse=False):
        """Handle dist conversions."""
        dist_in = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
        if reverse: 
            dist_in = -dist_in
        wheel_diam = float(cfg["robot_physics"].get("diameter", 2.75))
        rotations = 0.0 if wheel_diam == 0 else dist_in / (math.pi * wheel_diam)
        deg = rotations * 360.0
        ticks = rotations * ticks_per_rotation
        return dist_in, rotations, deg, ticks
    
    lines = [f"/* CODEGEN EXPORT: {style} */", ""]
    asset_declarations = []  # Track ASSET() declarations
    generated_paths = set()  # Track which paths we've generated
    jar_state = {
        "drive_max_voltage": None,
        "heading_max_voltage": None,
        "turn_max_voltage": None,
        "swing_max_voltage": None,
        "drive_settle_err": None,
        "drive_settle_time": None,
        "turn_settle_err": None,
        "turn_settle_time": None,
        "swing_settle_err": None,
        "swing_settle_time": None,
    }
    
    pose_pos = None
    for seg in timeline:
        if "p0" in seg:
            pose_pos = seg["p0"]
            break
        if "pos" in seg:
            pose_pos = seg["pos"]
            break
    pose_heading_internal = initial_heading
    if pose_heading_internal is None:
        for seg in timeline:
            pose_heading_internal = seg.get("start_heading") or seg.get("heading")
            if pose_heading_internal is not None:
                break
    initial_setpose_tokens = None
    if pose_heading_internal is None:
        try:
            pose_heading_internal = interpret_input_angle(float(cfg.get("initial_heading_deg", 0.0) or 0.0))
        except Exception:
            pose_heading_internal = float(cfg.get("initial_heading_deg", 0.0) or 0.0)
    pose_heading_disp = convert_heading_input(pose_heading_internal, None)
    angle_units = int(cfg.get("angle_units", 0))
    use_radians = angle_units == 1

    def _angle_out(val):
        """Handle angle out."""
        try:
            val_f = float(val)
        except Exception:
            return val
        return val_f * (math.pi / 180.0) if use_radians else val_f

    def _apply_angle_units(tokens: dict):
        """Handle apply angle units."""
        if not use_radians:
            return
        for key in (
            "HEADING_DEG",
            "TURN_DELTA_DEG",
            "TURN_SPEED",
            "TURN_EARLY_EXIT",
            "SWING_EARLY_EXIT",
            "TURN_SETTLE_ERR",
            "SWING_SETTLE_ERR",
        ):
            if key not in tokens:
                continue
            val = tokens.get(key)
            if val in ("", None):
                continue
            tokens[key] = _angle_out(val)
    pose_heading_out = _angle_out(pose_heading_disp)
    if pose_pos is not None and "setpose" in tpls:
        x0_in, y0_in = field_coords_in(pose_pos)
        initial_setpose_tokens = {
            "X_IN": round(x0_in, 6),
            "Y_IN": round(y0_in, 6),
            "HEADING_DEG": round(pose_heading_out, 6),
            "HEADING_RAD": round(pose_heading_disp * math.pi / 180.0, 9),
        }
    
    def _normalize_tpl(val):
        """Handle normalize tpl."""
        if isinstance(val, (list, tuple)):
            return [str(v) for v in val if str(v).strip()]
        if isinstance(val, str):
            parts = [p.strip().replace("\\n", "\n").replace("\\t", "\t") for p in val.split("||")]
            return [p for p in parts if p]
        return []

    def _escape_struct_braces(part: str) -> str:
        """Handle escape struct braces."""
        out = []
        in_field = False
        in_struct = False
        i = 0
        while i < len(part):
            ch = part[i]
            if ch == "{":
                if i + 1 < len(part) and part[i + 1] == "{":
                    out.append("{{")
                    i += 2
                    continue
                if (not in_field) and (not in_struct) and i + 1 < len(part) and part[i + 1] == ".":
                    out.append("{{.")
                    in_struct = True
                    i += 2
                    continue
                out.append("{")
                in_field = True
                i += 1
                continue
            if ch == "}":
                if in_field:
                    out.append("}")
                    in_field = False
                    i += 1
                    continue
                if in_struct:
                    out.append("}}")
                    in_struct = False
                    i += 1
                    continue
                if i + 1 < len(part) and part[i + 1] == "}":
                    out.append("}}")
                    i += 2
                    continue
                out.append("}")
                i += 1
                continue
            out.append(ch)
            i += 1
        return "".join(out)

    OMIT_SENTINEL = "__OMIT__"
    TOKEN_RE = re.compile(r"\{([A-Z0-9_]+)\}")
    omit_defaults = bool(cfg.get("codegen", {}).get("opts", {}).get("omit_defaults", 1))
    cleanup_defaults = {}
    if omit_defaults:
        cleanup_defaults = {
            "FORWARDS": True,
            "LEAD_IN": 0.0,
            "MOVE_SPEED": "",
            "TURN_SPEED": "",
            "PATH_MIN_SPEED": min_speed_cmd,
            "PATH_MAX_SPEED": max_speed_cmd,
            "DRIVE_MIN_SPEED": 0,
            "TURN_MIN_SPEED": 0,
            "SWING_MIN_SPEED": 0,
            "DRIVE_EARLY_EXIT": 0.0,
            "TURN_EARLY_EXIT": 0.0,
            "SWING_EARLY_EXIT": 0.0,
            "DRIVE_MAX_V": 12.0,
            "HEADING_MAX_V": 12.0,
            "TURN_MAX_V": 12.0,
            "SWING_MAX_V": 12.0,
            "DRIVE_SETTLE_ERR": 0.0,
            "DRIVE_SETTLE_TIME": 0,
            "TURN_SETTLE_ERR": 0.0,
            "TURN_SETTLE_TIME": 0,
            "SWING_SETTLE_ERR": 0.0,
            "SWING_SETTLE_TIME": 0,
            "DIR": "AUTO",
            "SIDE": "AUTO",
            "LOCKED_SIDE": "AUTO",
            "LOCKED_SIDE_": "",
        }

    def _boolish(val):
        """Handle boolish."""
        if isinstance(val, bool):
            return val
        s = str(val).strip().lower()
        if s in ("true", "1", "yes", "on"):
            return True
        if s in ("false", "0", "no", "off"):
            return False
        return None

    def _is_default_value(val, default):
        """Check whether is default value."""
        if isinstance(default, (list, tuple, set)):
            return any(_is_default_value(val, d) for d in default)
        if default is None or default == "":
            return val is None or str(val).strip() == ""
        if isinstance(default, bool):
            bool_val = _boolish(val)
            return bool_val is not None and bool_val == default
        if isinstance(default, (int, float)):
            try:
                return abs(float(val) - float(default)) <= 1e-6
            except Exception:
                return False
        return str(val).strip().lower() == str(default).strip().lower()

    def _token_is_named_segment(part: str, start_idx: int) -> bool:
        """Handle token is named segment."""
        prefix = part[:start_idx]
        seg_start = max(prefix.rfind(","), prefix.rfind("{"), prefix.rfind("("))
        segment = prefix[seg_start + 1:]
        return "=" in segment

    ADV_MOTION_TOKENS = {
        "DRIVE_MAX_V", "HEADING_MAX_V", "TURN_MAX_V", "SWING_MAX_V",
        "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
        "TURN_SETTLE_ERR", "TURN_SETTLE_TIME",
        "SWING_SETTLE_ERR", "SWING_SETTLE_TIME",
    }

    def _tokens_with_omit(part: str, tokens: dict) -> dict:
        """Handle tokens with omit."""
        if not part or not cleanup_defaults:
            return tokens
        contexts = {}
        for match in TOKEN_RE.finditer(part):
            key = match.group(1)
            if key not in cleanup_defaults:
                continue
            ctx = "named" if _token_is_named_segment(part, match.start()) else "positional"
            contexts.setdefault(key, set()).add(ctx)
        if not contexts:
            return tokens
        out = dict(tokens)
        for key, ctxs in contexts.items():
            default_hit = _is_default_value(tokens.get(key), cleanup_defaults[key])
            if not default_hit:
                continue
            if "named" in ctxs and "positional" not in ctxs:
                out[key] = OMIT_SENTINEL
                continue
        return out

    def _split_top_level_commas(content: str) -> list:
        """Handle split top level commas."""
        parts = []
        buf = []
        depth_paren = depth_brace = depth_bracket = 0
        for ch in content:
            if ch == "(":
                depth_paren += 1
            elif ch == ")":
                depth_paren = max(0, depth_paren - 1)
            elif ch == "{":
                depth_brace += 1
            elif ch == "}":
                depth_brace = max(0, depth_brace - 1)
            elif ch == "[":
                depth_bracket += 1
            elif ch == "]":
                depth_bracket = max(0, depth_bracket - 1)
            if ch == "," and depth_paren == depth_brace == depth_bracket == 0:
                parts.append("".join(buf))
                buf = []
                continue
            buf.append(ch)
        parts.append("".join(buf))
        return parts

    def _drop_in_delimited(line: str, open_ch: str, close_ch: str) -> str:
        """Handle drop in delimited."""
        out = []
        i = 0
        while i < len(line):
            if line[i] == open_ch:
                depth = 1
                j = i + 1
                while j < len(line) and depth > 0:
                    if line[j] == open_ch:
                        depth += 1
                    elif line[j] == close_ch:
                        depth -= 1
                    j += 1
                if depth != 0:
                    out.append(line[i:])
                    break
                content = line[i + 1:j - 1]
                if OMIT_SENTINEL in content:
                    parts = _split_top_level_commas(content)
                    kept = [p.strip() for p in parts if OMIT_SENTINEL not in p and p.strip()]
                    content = ", ".join(kept)
                out.append(open_ch + content + close_ch)
                i = j
                continue
            out.append(line[i])
            i += 1
        return "".join(out)

    def _drop_sentinel_segments(line: str) -> str:
        """Handle drop sentinel segments."""
        if OMIT_SENTINEL not in line:
            return line
        line = _drop_in_delimited(line, "{", "}")
        line = _drop_in_delimited(line, "(", ")")
        if OMIT_SENTINEL in line:
            line = re.sub(r"\s*=\s*__OMIT__", "", line)
            line = line.replace(OMIT_SENTINEL, "")
        line = re.sub(r",\s*\{\s*\}", "", line)
        line = re.sub(r",\s*,", ", ", line)
        line = re.sub(r"\(\s*,", "(", line)
        line = re.sub(r",\s*\)", ")", line)
        line = re.sub(r"\{\s*,", "{", line)
        line = re.sub(r",\s*\}", "}", line)
        line = re.sub(r"\s{2,}", " ", line)
        kept_lines = []
        for raw in line.splitlines():
            s = raw.strip()
            if re.search(r"\bset_(?:drive|turn|swing)_exit_conditions\s*\(\s*\)\s*;", s):
                continue
            kept_lines.append(raw)
        line = "\n".join(kept_lines)
        return line

    def _format_tpl_part(part: str, tokens: dict, template_key: str = None):
        """Handle format tpl part."""
        tokens_fmt = _tokens_with_omit(part, tokens)
        if not adv_motion_enabled and style not in ("JAR", "JAR (advanced)"):
            for key in ADV_MOTION_TOKENS:
                if key in tokens_fmt:
                    tokens_fmt[key] = OMIT_SENTINEL
        try:
            line = part.format(**tokens_fmt)
        except Exception as e:
            part_fixed = _escape_struct_braces(part)
            try:
                line = part_fixed.format(**tokens_fmt)
            except Exception:
                if template_key:
                    lines.append(f"// template error in {template_key}: {e}")
                    return None
                return part
        if OMIT_SENTINEL in line:
            line = _drop_sentinel_segments(line)
        return line
    
    def emit(template_key, tokens):
        """Handle emit."""
        tpl_val = tpls.get(template_key, "")
        for part in _normalize_tpl(tpl_val):
            line = _format_tpl_part(part, tokens, template_key=template_key)
            if line is None:
                continue
            if line.strip():
                lines.append(line)

    mech_presets = cfg.get("codegen", {}).get("mech_presets", [])
    if not isinstance(mech_presets, list):
        mech_presets = []
    preset_map = {}
    for p in mech_presets:
        if not isinstance(p, dict):
            continue
        name = str(p.get("name", "")).strip()
        if not name:
            continue
        preset_map[name.lower()] = p
    preset_state = {"reshape": False}

    def _render_tpl_lines(tpl_val, tokens):
        """Handle render tpl lines."""
        out_lines = []
        for part in _normalize_tpl(tpl_val):
            line = _format_tpl_part(part, tokens)
            if line is None:
                continue
            if line.strip():
                out_lines.append(line)
        return out_lines

    def _render_marker_actions(actions):
        """Handle render marker actions."""
        out_lines = []
        if not isinstance(actions, list):
            return out_lines
        for act in actions:
            if not isinstance(act, dict):
                continue
            kind = str(act.get("kind", "code")).lower()
            if kind == "preset":
                name = str(act.get("name", "")).strip()
                key = name.lower()
                preset = preset_map.get(key)
                if not preset:
                    out_lines.append(f"// marker: unknown preset '{name}'")
                    continue
                mode = str(preset.get("mode", "action")).strip().lower()
                if mode not in ("action", "toggle", "cases"):
                    mode = "action"
                value = str(act.get("value", "") or "").strip()
                values = act.get("values", [])
                if not isinstance(values, list):
                    values = []
                if not value and values:
                    value = " ".join(str(v).strip() for v in values if str(v).strip() != "")
                if not values and value:
                    values = value.split()
                values = [str(v).strip() for v in values if str(v).strip() != ""]
                value1 = values[0] if len(values) > 0 else ""
                value2 = values[1] if len(values) > 1 else ""
                value3 = values[2] if len(values) > 2 else ""
                state = str(act.get("state", "") or "").strip().lower()
                tokens = {
                    "VALUE": value,
                    "VALUE1": value1,
                    "VALUE2": value2,
                    "VALUE3": value3,
                    "CASE_KEY": "",
                    "STATE": state
                }
                if mode == "toggle":
                    if state not in ("on", "off", "toggle", ""):
                        state = "toggle"
                    if state == "on":
                        next_state = True
                    elif state == "off":
                        next_state = False
                    else:
                        current = bool(preset_state.get(key, preset.get("default", False)))
                        next_state = not current
                    preset_state[key] = next_state
                    tpl_key = "on" if next_state else "off"
                    tpl_val = preset.get(tpl_key, "")
                    if not tpl_val and key == "reshape":
                        tpl_val = tpls.get("reshape_on" if next_state else "reshape_off", "")
                        if not tpl_val:
                            tpl_val = tpls.get("reshape", "")
                        tokens["STATE"] = _reshape_state_token(cfg, 2 if next_state else 1)
                    else:
                        tokens["STATE"] = "on" if next_state else "off"
                    if not str(tpl_val or "").strip():
                        fallback = preset.get("template", preset.get("action", ""))
                        if str(fallback or "").strip():
                            tpl_val = fallback
                        else:
                            out_lines.append(f"// marker preset '{name}' has empty {tpl_key} template")
                            continue
                    out_lines.extend(_render_tpl_lines(tpl_val, tokens))
                elif mode == "cases":
                    case_list = preset.get("cases", [])
                    if isinstance(case_list, dict):
                        case_list = [{"key": k, "template": v} for k, v in case_list.items()]
                    if not isinstance(case_list, list):
                        case_list = []

                    def _case_match(case_key: str, full_val: str, first_val: str):
                        """Handle case match."""
                        key_cmp = str(case_key or "").strip()
                        if not key_cmp:
                            return False
                        if key_cmp in ("*", "_", "default"):
                            return False
                        full_cf = str(full_val or "").strip().casefold()
                        first_cf = str(first_val or "").strip().casefold()
                        for cand in key_cmp.split("|"):
                            c = cand.strip().casefold()
                            if not c:
                                continue
                            if c == full_cf or c == first_cf:
                                return True
                        return False

                    chosen_key = ""
                    tpl_val = ""
                    wildcard_tpl = ""
                    for case_item in case_list:
                        if not isinstance(case_item, dict):
                            continue
                        ckey = str(case_item.get("key", "")).strip()
                        ctpl = str(case_item.get("template", "") or "")
                        if not ckey:
                            continue
                        if ckey.strip().casefold() in ("*", "_", "default"):
                            if not wildcard_tpl:
                                wildcard_tpl = ctpl
                            continue
                        if _case_match(ckey, value, value1):
                            chosen_key = ckey
                            tpl_val = ctpl
                            break
                    if not tpl_val:
                        tpl_val = wildcard_tpl
                        if tpl_val:
                            chosen_key = "*"
                    if not tpl_val:
                        tpl_val = str(preset.get("case_default", "") or "")
                    if not tpl_val:
                        tpl_val = str(preset.get("template", preset.get("action", "")) or "")
                    if not str(tpl_val).strip():
                        out_lines.append(f"// marker preset '{name}' has no matching case for '{value or value1}'")
                        continue
                    tokens["CASE_KEY"] = chosen_key
                    out_lines.extend(_render_tpl_lines(tpl_val, tokens))
                else:
                    tpl_val = preset.get("template", preset.get("action", ""))
                    if not tpl_val and key == "reshape":
                        tpl_val = tpls.get("reshape_on", "")
                        if not tpl_val:
                            tpl_val = tpls.get("reshape", "")
                        tokens["STATE"] = _reshape_state_token(cfg, 2)
                    if not str(tpl_val or "").strip():
                        out_lines.append(f"// marker preset '{name}' has empty template")
                        continue
                    out_lines.extend(_render_tpl_lines(tpl_val, tokens))
            else:
                tpl_val = str(act.get("code", "")).strip()
                for part in _normalize_tpl(tpl_val):
                    if part.strip():
                        out_lines.append(part)
        return out_lines

    def _emit_edge_markers(seg, total_len_in, tokens_base, path_points=None):
        """Handle emit edge markers."""
        events = seg.get("edge_events", [])
        if not events:
            return
        total_len = 0.0
        if path_points and len(path_points) >= 2:
            try:
                plen = 0.0
                for i_pp in range(len(path_points) - 1):
                    dx = path_points[i_pp + 1][0] - path_points[i_pp][0]
                    dy = path_points[i_pp + 1][1] - path_points[i_pp][1]
                    plen += (math.hypot(dx, dy) / PPI)
                total_len = max(total_len, plen)
            except Exception:
                total_len = 0.0
        if total_len <= 1e-6:
            try:
                total_len = abs(float(total_len_in))
            except Exception:
                total_len = 0.0
        wait_tpl = tpls.get("marker_wait", "")
        done_tpl = tpls.get("marker_wait_done", "")
        events_sorted = []
        for ev in events:
            if not isinstance(ev, dict):
                continue
            if not ev.get("enabled", True):
                continue
            try:
                t_val = float(ev.get("t", 0.0))
            except Exception:
                t_val = 0.0
            events_sorted.append((max(0.0, min(1.0, t_val)), ev))
        events_sorted.sort(key=lambda item: item[0])
        for idx, (t_val, ev) in enumerate(events_sorted):
            dist_in = total_len * t_val
            tokens = dict(tokens_base)
            tokens.update({
                "MARKER_DIST_IN": round(dist_in, 6),
                "MARKER_FRAC": round(t_val, 6),
                "MARKER_INDEX": idx
            })
            if wait_tpl:
                emit("marker_wait", tokens)
            out_lines = _render_marker_actions(ev.get("actions", []))
            if out_lines:
                lines.extend(out_lines)
        if done_tpl:
            emit("marker_wait_done", tokens_base)

    def _turn_chain_settings(idx: int, seg: dict):
        """Handle turn chain settings."""
        turn_min = seg.get("chain_min_speed") if seg.get("chain_to_next") else None
        turn_exit = seg.get("chain_early_exit") if seg.get("chain_to_next") else None
        if seg.get("role") == "explicit" and (turn_min is None or turn_exit is None) and (idx + 1) < len(timeline):
            nxt = timeline[idx + 1]
            if isinstance(nxt, dict) and nxt.get("chain_to_next"):
                node_i = seg.get("node_i")
                nxt_i0 = nxt.get("i0", None)
                same_node = (node_i is not None and nxt_i0 is not None and node_i == nxt_i0)
                if same_node:
                    if turn_min is None:
                        turn_min = nxt.get("chain_min_speed")
                    if turn_exit is None:
                        turn_exit = nxt.get("chain_early_exit")
        return turn_min, turn_exit
    
    def emit_first(keys, tokens):
        """Handle emit first."""
        for k in keys:
            if k in tpls:
                emit(k, tokens)
                return
    
    i = 0
    rp = cfg.get("robot_physics", {})
    tbuf = float(rp.get("t_buffer", 0.0) or 0.0)
    adv_raw = rp.get("advanced_motion", 0)
    if isinstance(adv_raw, dict):
        adv_raw = adv_raw.get("value", 0)
    adv_motion_enabled = bool(float(adv_raw))
    if initial_setpose_tokens is not None:
        emit("setpose", initial_setpose_tokens)
        lines.append("")

    def emit_jar_constants(kind: str, tokens: dict, voltage: float, settle_err: float, settle_time: int):
        """No-op placeholder; JAR now uses inline placeholders only."""
        return False
        return False

    def _emit_jar_defaults(kind: str, tokens: dict):
        """Handle emit jar defaults."""
        return

    _exit_call_re = re.compile(
        r"^\s*(?:[\w:]+\s*\.\s*)?set_(drive|turn|swing)_exit_conditions\s*\(\s*([^()]*)\s*\)\s*;\s*$",
        re.IGNORECASE,
    )

    def _norm_exit_value(raw: str) -> str:
        """Handle norm exit value."""
        s = str(raw).strip()
        try:
            v = float(s.rstrip("fF"))
            if abs(v) < 1e-9:
                v = 0.0
            return f"{v:.6f}"
        except Exception:
            return re.sub(r"\s+", "", s).lower()

    def _dedupe_exit_condition_lines(lines_in):
        """Handle dedupe exit condition lines."""
        out = []
        last_val = {}
        for entry in lines_in:
            text = str(entry)
            parts = text.splitlines() or [text]
            kept_parts = []
            for line in parts:
                m = _exit_call_re.match(line)
                if not m:
                    kept_parts.append(line)
                    continue
                kind = m.group(1).lower()
                value = _norm_exit_value(m.group(2))
                if kind in last_val and last_val[kind] == value:
                    continue
                last_val[kind] = value
                kept_parts.append(line)
            if kept_parts:
                out.append("\n".join(kept_parts))
        return out

    def _first_tpl_key(keys):
        """Handle first tpl key."""
        for k in keys:
            if k in tpls:
                return k
        return None

    def _apply_settle(
        tokens: dict,
        move_type: str,
        magnitude: float,
        profile: str,
        settle_mag: float = None,
        t_speed_frac: float = None,
        drive_min_speed: int = None,
        drive_early_exit: float = None,
        turn_min_speed: int = None,
        turn_early_exit: float = None,
        swing_min_speed: int = None,
        swing_early_exit: float = None
    ):
        """Handle apply settle."""
        mag = abs(float(magnitude))
        cap = 12.0
        settle_err = 0.0
        settle_time_adj = 0
        if adv_motion_enabled:
            cap = _compute_voltage_cap(cfg, move_type, mag, profile)
            cap = min(12.0, max(0.0, cap))
            if jar_like_style:
                cap_frac = _clamp(cap / 12.0, 0.0, 1.0)
                eff_frac = cap_frac
                if t_speed_frac is not None:
                    try:
                        t_frac = float(t_speed_frac)
                    except Exception:
                        t_frac = None
                    if t_frac is not None and t_frac > 0.0:
                        t_frac = _clamp(t_frac, 0.2, 1.0)
                        eff_frac = _clamp(cap_frac / max(1e-6, t_frac), 0.2, 1.0)
                _scale_timeout(tokens, eff_frac)
            settle_mag_val = mag if settle_mag is None else abs(float(settle_mag))
            settle_cap = cap if jar_like_style else 12.0
            settle_err, settle_time = _compute_settle_final(cfg, move_type, settle_mag_val, settle_cap, profile)
            settle_time_adj = int(round(settle_time))
        drive_early_exit = 0.0 if drive_early_exit is None else drive_early_exit
        turn_early_exit = 0.0 if turn_early_exit is None else turn_early_exit
        swing_early_exit = 0.0 if swing_early_exit is None else swing_early_exit
        drive_min_speed = 0 if drive_min_speed is None else drive_min_speed
        turn_min_speed = 0 if turn_min_speed is None else turn_min_speed
        swing_min_speed = 0 if swing_min_speed is None else swing_min_speed
        drive_min_speed = int(max(0, min(127, drive_min_speed)))
        turn_min_speed = int(max(0, min(127, turn_min_speed)))
        swing_min_speed = int(max(0, min(127, swing_min_speed)))
        cap_out = cap
        heading_cap_out = _compute_heading_cap(cap, profile)
        if move_type == "drive":
            tokens.update({
                "DRIVE_MAX_V": cap_out,
                "HEADING_MAX_V": heading_cap_out,
                "DRIVE_SETTLE_ERR": round(float(settle_err), 2),
                "DRIVE_SETTLE_TIME": int(settle_time_adj),
                "DRIVE_EARLY_EXIT": round(float(drive_early_exit), 2),
                "DRIVE_MIN_SPEED": int(drive_min_speed),
            })
        elif move_type == "turn":
            tokens.update({
                "TURN_MAX_V": cap_out,
                "TURN_SETTLE_ERR": round(float(settle_err), 2),
                "TURN_SETTLE_TIME": int(settle_time_adj),
                "TURN_EARLY_EXIT": round(float(turn_early_exit), 2),
                "TURN_MIN_SPEED": int(turn_min_speed),
            })
        elif move_type == "swing":
            tokens.update({
                "SWING_MAX_V": cap_out,
                "SWING_SETTLE_ERR": round(float(settle_err), 2),
                "SWING_SETTLE_TIME": int(settle_time_adj),
                "SWING_EARLY_EXIT": round(float(swing_early_exit), 2),
                "SWING_MIN_SPEED": int(swing_min_speed),
            })
        if adv_motion_enabled:
            emit_jar_constants(move_type, tokens, cap, settle_err, settle_time_adj)
            _add_settle_timeout(tokens, settle_time_adj, SETTLE_TIMEOUT_PAD_MS)
        return cap, settle_err, settle_time_adj
    
    while i < len(timeline):
        seg = timeline[i]
        T = float(seg.get("T", 0.0))
        st = seg.get("type")
        if st == "turn":
            st = "face"
        if st in ("move", "pose", "path", "face", "swing"):
            phys_T = _physics_timeout_s(seg, st, cfg, motion_mode=motion_mode)
            if phys_T > 0.0:
                T = max(T, phys_T)
        eff_pad = pad_factor
        if st == "wait" or seg.get("role") == "buffer":
            eff_pad = 1.0
        min_s_eff = min_s
        if st in ("wait",) or seg.get("role") == "buffer":
            min_s_eff = 0.0
        elif st in ("move", "pose", "path", "face", "swing"):
            min_s_eff = max(min_s, MIN_CMD_TIMEOUT_S)
        to_ms, to_s = _timeout_tokens(T, eff_pad, min_s_eff)
        tokens_base = {
            "MS": to_ms, "S": to_s, "TIMEOUT_MS": to_ms, "TIMEOUT_S": to_s,
            "STATE": _reshape_state_token(cfg, seg.get("state", 0)), "NAME": seg.get("name", ""),
            "LOOKAHEAD": int(lookahead_in)
        }
        
        if st == "path":
            if seg.get("move_to_pose"):
                p0, p1 = seg["p0"], seg["p1"]
                x_in, y_in = field_coords_in(p1)
                dist_in, rotations, deg, ticks = dist_conversions(p0, p1, reverse=seg.get("reverse", False))
                path_len_in = float(seg.get("path_length_in") or 0.0)
                if path_len_in <= 0.0:
                    pts = seg.get("path_points") or []
                    if len(pts) >= 2:
                        try:
                            for idx_pp in range(len(pts) - 1):
                                dx = pts[idx_pp + 1][0] - pts[idx_pp][0]
                                dy = pts[idx_pp + 1][1] - pts[idx_pp][1]
                                path_len_in += (math.hypot(dx, dy) / PPI)
                        except Exception:
                            path_len_in = 0.0
                pose_h = seg.get("pose_heading", seg.get("facing", 0.0))
                pose_h = convert_heading_input(pose_h, None)
                if seg.get("reverse", False):
                    pose_h = (pose_h + 180.0) % 360.0
                curve_T = None
                if path_len_in > 0.0:
                    profile_curve = seg.get("profile_override") or pick_profile("drive", abs(path_len_in), cfg=cfg)
                    v_override = None
                    if seg.get("drive_speed_ips") is not None:
                        try:
                            v_override = float(seg.get("drive_speed_ips"))
                        except Exception:
                            v_override = None
                    if v_override is None and seg.get("drive_speed_cmd") is not None:
                        try:
                            v_override = _cmd_to_ips(float(seg.get("drive_speed_cmd")), cfg)
                        except Exception:
                            v_override = None
                    if v_override is None:
                        v_override = vmax_straight(cfg) * _profile_speed_scale(profile_curve)
                    try:
                        curve_T = float(_move_total_time(path_len_in, cfg, v_override=v_override))
                    except Exception:
                        curve_T = None
                to_ms = tokens_base["TIMEOUT_MS"]
                to_s = tokens_base["TIMEOUT_S"]
                if curve_T is not None and curve_T > 0.0 and curve_T > T:
                    to_ms, to_s = _timeout_tokens(curve_T, eff_pad, min_s_eff)
                tokens = dict(tokens_base)
                if to_ms != tokens_base["TIMEOUT_MS"]:
                    tokens.update({"MS": to_ms, "S": to_s, "TIMEOUT_MS": to_ms, "TIMEOUT_S": to_s})
                tokens.update({
                    "X_IN": round(x_in, 6), "Y_IN": round(y_in, 6),
                    "DIST_IN": round(dist_in, 6), "DIST_ROT": round(rotations, 6),
                    "DIST_DEG": round(deg, 6), "DIST_TICKS": round(ticks, 6),
                    "FORWARDS": "false" if seg.get("reverse") else "true",
                    "FORWARD_PARAM": "{.forwards = " + ("false" if seg.get("reverse") else "true") + "}",
                    "MOVE_SPEED": "" if seg.get("drive_speed_cmd") is None else round(max(0.0, min(127.0, float(seg.get("drive_speed_cmd")))), 3),
                    "HEADING_DEG": round(pose_h, 6),
                    "HEADING_RAD": round(pose_h * math.pi / 180.0, 9),
                    "LEAD_IN": round(float(seg.get("pose_lead_in", 0.0) or 0.0), 6),
                    "DRIVE_MAX_V": "",
                    "HEADING_MAX_V": "",
                    "DRIVE_SETTLE_ERR": "",
                    "DRIVE_SETTLE_TIME": "",
                })
                prof_len = path_len_in if path_len_in > 0.0 else abs(dist_in)
                profile = seg.get("profile_override") or pick_profile("drive", abs(prof_len), cfg=cfg)
                chain_min = seg.get("chain_min_speed") if seg.get("chain_to_next") else None
                chain_exit = seg.get("chain_early_exit") if seg.get("chain_to_next") else None
                _apply_settle(
                    tokens,
                    "drive",
                    prof_len,
                    profile,
                    settle_mag=prof_len,
                    t_speed_frac=seg.get("T_speed_frac"),
                    drive_min_speed=chain_min,
                    drive_early_exit=chain_exit
                )
                _apply_angle_units(tokens)
                _emit_jar_defaults("drive", tokens)
                emit("pose", tokens)
                _emit_edge_markers(seg, path_len_in if path_len_in > 0.0 else abs(dist_in), tokens_base, path_points=seg.get("path_points"))
                i += 1
                continue
            segment_idx = seg.get("segment_idx", i)
            path_asset_name = generate_path_asset_name(routine_name, segment_idx)
            path_points = seg.get("path_points", [])
            vels = seg.get("path_speeds_export") or seg.get("path_speeds") or []
            tokens_min = max(0.0, min(127.0, float(seg.get("min_speed_cmd", seg.get("min_speed_ips", min_speed_cmd)))))
            tokens_max = max(0.0, min(127.0, float(seg.get("max_speed_cmd", seg.get("max_speed_ips", max_speed_cmd)))))
            
            if path_points and len(path_points) >= 2 and path_asset_name not in generated_paths:
                try:
                    init_v = vels[0] if vels else min_speed_cmd
                    end_v = vels[-1] if vels else min_speed_cmd
                    export_lemlib_path(
                        path_points,
                        path_asset_name,
                        cfg,
                        initial_velocity=init_v,
                        end_velocity=end_v,
                        velocities=vels if vels else None,
                desired_min_speed=tokens_min,
                desired_max_speed=tokens_max
                    )
                    generated_paths.add(path_asset_name)
                    
                    asset_var = path_asset_name.replace(".txt", "").replace("-", "_")
                    asset_declarations.append(f'ASSET({asset_var}_txt);')
                except Exception as e:
                    print(f"Warning: Failed to export path {path_asset_name}: {e}")
            
            tokens = dict(tokens_base)
            base_name = path_asset_name.replace(".txt", "")
            asset_var = base_name.replace("-", "_")
            forward_flag = "false" if seg.get("reverse") else "true"
            la_in_seg = lookahead_in
            if seg.get("lookahead_in_override") is not None:
                try:
                    la_in_seg = float(seg["lookahead_in_override"])
                except Exception:
                    la_in_seg = lookahead_in
            elif seg.get("lookahead_px") is not None:
                try:
                    la_in_seg = float(seg["lookahead_px"]) / PPI
                except Exception:
                    la_in_seg = lookahead_in
            tokens.update({
                "PATH_ASSET": f"{asset_var}_txt",            # ASSET symbol (C identifier)
                "PATH_NAME": f"{asset_var}_txt",             # String name to match ASSET
                "PATH_FILE": path_asset_name,                # Filename with .txt
                "FORWARDS": forward_flag,
                "FORWARD_PARAM": f"{{.forwards = {forward_flag}}}",
                "PATH_MIN_SPEED": round(float(tokens_min), 3),
                "PATH_MAX_SPEED": round(float(tokens_max), 3),
                "LOOKAHEAD": round(la_in_seg, 6),
                "DRIVE_MAX_V": "",
                "HEADING_MAX_V": "",
                "DRIVE_SETTLE_ERR": "",
                "DRIVE_SETTLE_TIME": "",
            })
            path_len_in = 0.0
            try:
                for idx_pp in range(len(path_points) - 1):
                    dx = path_points[idx_pp + 1][0] - path_points[idx_pp][0]
                    dy = path_points[idx_pp + 1][1] - path_points[idx_pp][1]
                    path_len_in += (math.hypot(dx, dy) / PPI)
            except Exception:
                path_len_in = 0.0
            profile = seg.get("profile_override") or pick_profile("drive", abs(path_len_in), cfg=cfg)
            settle_mag = abs(path_len_in)
            curvatures = seg.get("path_curvatures") or []
            if curvatures:
                curv_in_vals = [abs(c) * PPI for c in curvatures]
                avg_curv = sum(curv_in_vals) / max(1, len(curv_in_vals))
                max_curv = max(curv_in_vals)
                curv_metric = max(avg_curv, max_curv * 0.75)
                curv_scale = min(0.35, curv_metric * 4.0)
                settle_mag = settle_mag * (1.0 + curv_scale)
            path_tpl_key = _first_tpl_key(("path_follow", "path"))
            chain_min = seg.get("chain_min_speed") if seg.get("chain_to_next") else None
            chain_exit = seg.get("chain_early_exit") if seg.get("chain_to_next") else None
            _apply_settle(
                tokens,
                "drive",
                path_len_in,
                profile,
                settle_mag=settle_mag,
                t_speed_frac=seg.get("T_speed_frac"),
                drive_min_speed=chain_min,
                drive_early_exit=chain_exit
            )
            _apply_angle_units(tokens)
            _emit_jar_defaults("drive", tokens)
            if path_tpl_key:
                emit(path_tpl_key, tokens)
                _emit_edge_markers(seg, path_len_in, tokens_base, path_points=seg.get("path_points"))
            
            i += 1
        elif st == "move":
            p0, p1 = seg["p0"], seg["p1"]
            x_in, y_in = field_coords_in(p1)
            dist_in, rotations, deg, ticks = dist_conversions(p0, p1, reverse=seg.get("reverse", False))
            facing_heading = seg.get("facing", seg.get("target_heading", 0.0))
            face_disp = convert_heading_input(facing_heading, None)
            tokens = dict(tokens_base)
            tokens.update({
                "X_IN": round(x_in, 6), "Y_IN": round(y_in, 6),
                "DIST_IN": round(dist_in, 6), "DIST_ROT": round(rotations, 6),
                "DIST_DEG": round(deg, 6), "DIST_TICKS": round(ticks, 6),
                "FORWARDS": "false" if seg.get("reverse") else "true",
                "FORWARD_PARAM": "{.forwards = " + ("false" if seg.get("reverse") else "true") + "}",
                "MOVE_SPEED": "" if seg.get("drive_speed_cmd") is None else round(max(0.0, min(127.0, float(seg.get("drive_speed_cmd")))), 3),
                "HEADING_DEG": round(face_disp, 6),
                "HEADING_RAD": round(face_disp * math.pi / 180.0, 9),
                "LEAD_IN": round(float(seg.get("pose_lead_in", 0.0) or 0.0), 6),
                "DRIVE_MAX_V": "",
                "HEADING_MAX_V": "",
                "DRIVE_SETTLE_ERR": "",
                "DRIVE_SETTLE_TIME": "",
            })
            profile = seg.get("profile_override") or pick_profile("drive", abs(dist_in), cfg=cfg)
            move_tpl_key = "pose" if motion_mode == "pose" else "move"
            chain_min = seg.get("chain_min_speed") if seg.get("chain_to_next") else None
            chain_exit = seg.get("chain_early_exit") if seg.get("chain_to_next") else None
            _apply_settle(
                tokens,
                "drive",
                dist_in,
                profile,
                t_speed_frac=seg.get("T_speed_frac"),
                drive_min_speed=chain_min,
                drive_early_exit=chain_exit
            )
            _apply_angle_units(tokens)
            _emit_jar_defaults("drive", tokens)
            emit(move_tpl_key, tokens)
            _emit_edge_markers(seg, dist_in, tokens_base)
            i += 1
        
        elif st == "pose":
            p0, p1 = seg["p0"], seg["p1"]
            x_in, y_in = field_coords_in(p1)
            dist_in, rotations, deg, ticks = dist_conversions(p0, p1, reverse=seg.get("reverse", False))
            pose_h = convert_heading_input(seg.get("facing", 0.0), None)
            if seg.get("reverse", False):
                pose_h = (pose_h + 180.0) % 360.0
            tokens = dict(tokens_base)
            tokens.update({
                "X_IN": round(x_in, 6), "Y_IN": round(y_in, 6),
                "DIST_IN": round(dist_in, 6), "DIST_ROT": round(rotations, 6),
                "DIST_DEG": round(deg, 6), "DIST_TICKS": round(ticks, 6),
                "FORWARDS": "false" if seg.get("reverse") else "true",
                "FORWARD_PARAM": "{.forwards = " + ("false" if seg.get("reverse") else "true") + "}",
                "MOVE_SPEED": "" if seg.get("drive_speed_cmd") is None else round(max(0.0, min(127.0, float(seg.get("drive_speed_cmd")))), 3),
                "HEADING_DEG": round(pose_h, 6),
                "HEADING_RAD": round(pose_h * math.pi / 180.0, 9),
                "LEAD_IN": round(float(seg.get("pose_lead_in", 0.0) or 0.0), 6),
                "DRIVE_MAX_V": "",
                "HEADING_MAX_V": "",
                "DRIVE_SETTLE_ERR": "",
                "DRIVE_SETTLE_TIME": "",
            })
            profile = seg.get("profile_override") or pick_profile("drive", abs(dist_in), cfg=cfg)
            pose_tpl_key = "pose"
            if jar_like_style:
                jar_pose_opt = opts.get("jar_pose_angle_overload", 0)
                if isinstance(jar_pose_opt, dict):
                    jar_pose_opt = jar_pose_opt.get("value", 0)
                try:
                    jar_pose_angle = bool(int(jar_pose_opt))
                except Exception:
                    jar_pose_angle = bool(jar_pose_opt)
                if jar_pose_angle and "pose_angle" in tpls:
                    pose_tpl_key = "pose_angle"
            chain_min = seg.get("chain_min_speed") if seg.get("chain_to_next") else None
            chain_exit = seg.get("chain_early_exit") if seg.get("chain_to_next") else None
            _apply_settle(
                tokens,
                "drive",
                dist_in,
                profile,
                t_speed_frac=seg.get("T_speed_frac"),
                drive_min_speed=chain_min,
                drive_early_exit=chain_exit
            )
            _apply_angle_units(tokens)
            _emit_jar_defaults("drive", tokens)
            emit(pose_tpl_key, tokens)
            _emit_edge_markers(seg, dist_in, tokens_base)
            i += 1
        
        elif st == "face":
            role = seg.get("role")
            h_disp = convert_heading_input(seg["target_heading"], None)
            delta_internal = ((seg["target_heading"] - seg.get("start_heading", 0.0) + 180.0) % 360.0) - 180.0
            delta_heading = -delta_internal
            target_pos = seg.get("target_pos")
            if not (isinstance(target_pos, (list, tuple)) and len(target_pos) >= 2):
                target_pos = seg.get("p1", seg.get("pos", (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2)))
            target_x_in, target_y_in = field_coords_in(target_pos)
            tokens = dict(tokens_base)
            tokens.update({
                "HEADING_DEG": round(h_disp, 6),
                "HEADING_RAD": round(h_disp * math.pi / 180.0, 9),
                "TURN_DELTA_DEG": round(delta_heading, 6),
                "TURN_DELTA_RAD": round(delta_heading * math.pi / 180.0, 9),
                "TARGET_X_IN": round(target_x_in, 6),
                "TARGET_Y_IN": round(target_y_in, 6),
                "TURN_SPEED": "" if seg.get("turn_speed_dps") is None else round(float(seg.get("turn_speed_dps")), 6),
                "TURN_MAX_V": "",
                "TURN_SETTLE_ERR": "",
                "TURN_SETTLE_TIME": "",
            })
            chosen_turn = "turn_global"
            if turn_mode == "turn_local":
                chosen_turn = "turn_local"
            profile = seg.get("profile_override") or pick_profile("turn", abs(delta_heading), cfg=cfg)
            turn_min, turn_exit = _turn_chain_settings(i, seg)
            _apply_settle(
                tokens,
                "turn",
                abs(delta_heading),
                profile,
                t_speed_frac=seg.get("T_speed_frac"),
                turn_min_speed=turn_min,
                turn_early_exit=turn_exit
            )
            _apply_angle_units(tokens)
            _emit_jar_defaults("turn", tokens)
            emit(chosen_turn, tokens)
            i += 1
        
        elif st == "wait":
            if not ((i == len(timeline)-1) and abs(T - tbuf) <= 1e-6 and seg.get("role") == "buffer"):
                temp_key = "tbuffer" if seg.get("role") == "buffer" else "wait"
                emit(temp_key, tokens_base)
            i += 1
        
        elif st == "marker":
            out_lines = _render_marker_actions(seg.get("actions", []))
            if out_lines:
                lines.extend(out_lines)
            i += 1
        
        elif st == "swing":
            h_disp = convert_heading_input(seg["target_heading"], None)
            delta_internal = ((seg["target_heading"] - seg.get("start_heading", 0.0) + 180.0) % 360.0) - 180.0
            delta_heading = -delta_internal
            target_pos = seg.get("target_pos")
            if not (isinstance(target_pos, (list, tuple)) and len(target_pos) >= 2):
                target_pos = seg.get("p1", seg.get("end_pos", seg.get("pos", (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2))))
            target_x_in, target_y_in = field_coords_in(target_pos)
            dir_raw = str(seg.get("swing_dir", "auto")).strip().lower()
            if dir_raw in ("cw", "clockwise", "cw_clockwise"):
                dir_key = "cw"
            elif dir_raw in ("ccw", "counterclockwise", "ccw_counterclockwise"):
                dir_key = "ccw"
            else:
                dir_key = "auto"

            dir_effective = dir_key
            if dir_effective == "auto":
                dir_effective = "ccw" if delta_internal >= 0.0 else "cw"

            dir_token = {"cw": "CW_CLOCKWISE", "ccw": "CCW_COUNTERCLOCKWISE"}[dir_effective]

            if dir_token == "CW_CLOCKWISE":
                side_token = "LEFT"
            elif dir_token == "CCW_COUNTERCLOCKWISE":
                side_token = "RIGHT"

            def _opp_side(side: str) -> str:
                """Handle opp side."""
                s = (side or "").strip().upper()
                if s == "LEFT":  return "RIGHT"
                if s == "RIGHT": return "LEFT"

            locked_side = _opp_side(side_token)

            tokens = dict(tokens_base)
            tokens.update({
                "HEADING_DEG": round(h_disp, 6),
                "HEADING_RAD": round(h_disp * math.pi / 180.0, 9),
                "TURN_DELTA_DEG": round(delta_heading, 6),
                "TURN_DELTA_RAD": round(delta_heading * math.pi / 180.0, 9),
                "TARGET_X_IN": round(target_x_in, 6),
                "TARGET_Y_IN": round(target_y_in, 6),
                "DIR": dir_token,
                "SIDE": side_token,
                "SWING_MAX_V": "",
                "SWING_SETTLE_ERR": "",
                "SWING_SETTLE_TIME": "",
                "LOCKED_SIDE": locked_side,
            })
            profile = seg.get("profile_override") or pick_profile("swing", abs(delta_heading), cfg=cfg)
            swing_tpl_key = "swing"
            swing_settle = bool(seg.get("swing_settle", False))
            swing_min_speed = 0 if swing_settle else DEFAULT_SWING_MIN_SPEED
            swing_early_exit = 0.0 if swing_settle else DEFAULT_SWING_EARLY_EXIT
            _apply_settle(
                tokens,
                "swing",
                abs(delta_heading),
                profile,
                t_speed_frac=seg.get("T_speed_frac"),
                swing_min_speed=swing_min_speed,
                swing_early_exit=swing_early_exit
            )
            _apply_angle_units(tokens)
            _emit_jar_defaults("swing", tokens)
            emit("swing", tokens)
            i += 1
        
        elif st == "reshape":
            state_raw = seg.get("state", 0)
            try:
                state_num = int(state_raw)
            except Exception:
                state_num = 2 if str(state_raw).strip().lower() in ("2", "on", "true", "yes", "reshaped") else 1
            next_state = (state_num == 2)
            tokens = dict(tokens_base)
            tokens["STATE"] = _reshape_state_token(cfg, 2 if next_state else 1)
            reshape_key = "reshape_on" if next_state else "reshape_off"
            if tpls.get(reshape_key, ""):
                emit(reshape_key, tokens)
            else:
                emit("reshape", tokens)
            preset_state["reshape"] = next_state
            i += 1
        
        else:
            i += 1
    
    lines = _dedupe_exit_condition_lines(lines)

    total_t = sum(float(sg.get("T", 0.0)) for sg in timeline)
    if timeline and timeline[-1].get("type") == "wait" and timeline[-1].get("role") == "buffer":
        total_t = max(0.0, total_t - float(tbuf))
    
    lines.append("")
    lines.append(f"// Estimated total time: {total_t:.2f} s")
    
    if asset_declarations:
        header = [
            "// Path asset declarations",
            "// Add these near the top of your file:",
            ""
        ] + asset_declarations + ["", "// Autonomous routine:"]
        lines = header + lines
    
    return lines


def export_action_list(cfg, timeline, log_lines):
    """
    Generate action list output (original format) with path indicators.
    
    Args:
        cfg: Configuration dictionary
        timeline: Compiled timeline
        log_lines: List to append log output to
    """
    from .util import pros_convert_inches
    from .geom import convert_heading_input, coords_str
    
    tbuf = float(cfg.get("robot_physics", {}).get("t_buffer", 0.0) or 0.0)
    last_turn = None
    reshape_label = str(cfg.get("reshape_label", "Reshape"))

    def _marker_actions_label(actions):
        """Handle marker actions label."""
        if not isinstance(actions, list):
            return ""
        parts = []
        for act in actions:
            if not isinstance(act, dict):
                continue
            kind = str(act.get("kind", "code")).lower()
            if kind == "preset":
                name = str(act.get("name", "")).strip()
                if not name:
                    continue
                text = name
                state_raw = act.get("state", None)
                if isinstance(state_raw, bool):
                    state = "on" if state_raw else "off"
                elif state_raw is None:
                    state = ""
                else:
                    state = str(state_raw).strip().lower()
                value_raw = act.get("value", None)
                if value_raw is None:
                    values = act.get("values", [])
                    if isinstance(values, list):
                        value = " ".join(str(v).strip() for v in values if str(v).strip() != "")
                    else:
                        value = ""
                else:
                    value = str(value_raw).strip()
                if state:
                    text = f"{text} {state}"
                if value:
                    text = f"{text} {value}"
                parts.append(text)
            else:
                code = str(act.get("code", "")).strip()
                if code:
                    parts.append(f"code {code}")
        return ", ".join(parts)
    
    for idx, seg in enumerate(timeline):
        T = float(seg.get("T", 0.0))
        st = seg.get("type")
        if st == "marker":
            label = _marker_actions_label(seg.get("actions", []))
            if label:
                log_lines.append(f"  Marker: {label}")
            else:
                log_lines.append("  Marker")
            continue
        if T <= 0.0:
            continue
        
        if st == "path":
            p0, p1 = seg["p0"], seg["p1"]
            segment_idx = seg.get("segment_idx", idx)
            path_file = generate_path_asset_name("routine", segment_idx)
            
            log_lines.append(f"\n --- Path Segment {segment_idx} ---")
            log_lines.append(f"  Start: {coords_str(p0, cfg, (WINDOW_WIDTH//2, WINDOW_HEIGHT//2))}")
            log_lines.append(f"  End: {coords_str(p1, cfg, (WINDOW_WIDTH//2, WINDOW_HEIGHT//2))}")
            log_lines.append(f"  Path File: {path_file}")
            log_lines.append(f"  Duration: {T:.3f} s")
            
            if "path_points" in seg:
                path_points = seg["path_points"]
                total_dist_px = 0.0
                for i in range(len(path_points) - 1):
                    dx = path_points[i+1][0] - path_points[i][0]
                    dy = path_points[i+1][1] - path_points[i][1]
                    total_dist_px += math.hypot(dx, dy)
                dist_in = total_dist_px / PPI
                conv, unit = pros_convert_inches(cfg, dist_in)
                log_lines.append(f"  Path Length: {conv:.3f} {unit}")
        
        elif st == "move":
            p0, p1 = seg["p0"], seg["p1"]
            log_lines.append(f"\n --- Node {seg.get('i0', '?')} to Node {seg.get('i1', '?')} ---")
            inches = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
            if seg.get("reverse", False):
                inches = -inches
            conv, unit = pros_convert_inches(cfg, inches)
            log_lines.append(f"  Distance: {conv:.3f} {unit}")
        
        elif st == "face":
            h0, h1 = seg["start_heading"], seg["target_heading"]
            _key = (round(h0, 6), round(h1, 6), seg.get("role"))
            if _key != last_turn:
                delta_ccw = (h1 - h0 + 360.0) % 360.0
                if delta_ccw > 180.0:
                    delta_ccw -= 360.0
                chosen = float(-delta_ccw)
                dir_tag = "CCW" if chosen >= 0 else "CW"
                to_face_deg = convert_heading_input(h1, None)
                
                if int(cfg.get("angle_units", 0)) == 1:
                    chosen_val = chosen * (math.pi/180.0)
                    to_face_val = to_face_deg * (math.pi/180.0)
                    log_lines.append(f"  Turn: {chosen_val:.6f} rad ({dir_tag} +) | To face: {to_face_val:.6f} rad")
                else:
                    log_lines.append(f"  Turn: {chosen:.3f}° ({dir_tag} +) | To face: {to_face_deg:.3f}°")
                last_turn = _key
        
        elif st == "wait":
            if not (idx == len(timeline)-1 and abs(T - tbuf) <= 1e-6 and seg.get("role") == "buffer"):
                log_lines.append(f"  Wait: {T:.3f} s")
        
        elif st == "reshape":
            log_lines.append(f"  {reshape_label}: state {seg.get('state', 0)}")
        
        elif st == "swing":
            h0, h1 = seg["start_heading"], seg["target_heading"]
            delta_ccw = (h1 - h0 + 360.0) % 360.0
            if delta_ccw > 180.0:
                delta_ccw -= 360.0
            chosen = float(-delta_ccw)
            dir_tag = seg.get("swing_dir", "AUTO").upper()
            to_face_deg = convert_heading_input(h1, None)
            if int(cfg.get("angle_units", 0)) == 1:
                chosen_val = chosen * (math.pi/180.0)
                to_face_val = to_face_deg * (math.pi/180.0)
                log_lines.append(f"  Swing: {chosen_val:.6f} rad dir={dir_tag} to {to_face_val:.6f} rad")
            else:
                log_lines.append(f"  Swing: {chosen:.3f}° dir={dir_tag} to {to_face_deg:.3f}°")
    
    total_t = sum(float(sg.get("T", 0.0)) for sg in timeline)
    log_lines.append(f"\nEstimated total time: {total_t:.2f} s")

def build_export_lines(cfg, timeline, routine_name="autonomous", initial_heading=None):
    """Build code export lines from timeline (path-aware wrapper)."""
    return build_export_lines_with_paths(cfg, timeline, routine_name, initial_heading)
