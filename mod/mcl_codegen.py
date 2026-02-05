from __future__ import annotations

import hashlib
import json
import math
import os
from typing import List, Tuple

from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT
from .draw import get_field_object_entries
from .mcl import build_map_segments, get_distance_sensors, _entry_enabled


def _extract_value(val):
    """Handle extract value."""
    for _ in range(8):
        if isinstance(val, dict) and "value" in val:
            val = val.get("value")
            continue
        break
    return val


def _fmt(val: float) -> str:
    """Handle fmt."""
    val = _extract_value(val)
    try:
        return f"{float(val):.6g}"
    except Exception:
        return "0"


def _bool(val) -> str:
    """Handle bool."""
    val = _extract_value(val)
    try:
        return "true" if int(val) != 0 else "false"
    except Exception:
        return "true" if bool(val) else "false"


def _config_hash(payload: dict) -> str:
    """Handle config hash."""
    try:
        raw = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    except Exception:
        raw = str(payload)
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def _unwrap_values(obj):
    """Handle unwrap values."""
    if isinstance(obj, dict):
        if set(obj.keys()) == {"value"}:
            return obj.get("value")
        return {k: _unwrap_values(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_unwrap_values(v) for v in obj]
    return obj


def _px_to_field_in(x_px: float, y_px: float, field_w: float, field_h: float) -> Tuple[float, float]:
    """Handle px to field in."""
    x_in = x_px / PPI
    y_in = y_px / PPI
    x_fwd = field_h * 0.5 - y_in
    y_left = field_w * 0.5 - x_in
    return x_fwd, y_left


def _screen_in_to_field_in(x_in: float, y_in: float, field_w: float, field_h: float) -> Tuple[float, float]:
    """Handle screen in to field in."""
    x_fwd = field_h * 0.5 - y_in
    y_left = field_w * 0.5 - x_in
    return x_fwd, y_left


def _segments_in_field_in(cfg: dict, field_w: float, field_h: float) -> List[Tuple[float, float, float, float]]:
    """Handle segments in field in."""
    segs_px = build_map_segments(cfg)
    out = []
    for x0, y0, x1, y1 in segs_px:
        fx0, fy0 = _px_to_field_in(x0, y0, field_w, field_h)
        fx1, fy1 = _px_to_field_in(x1, y1, field_w, field_h)
        out.append((fx0, fy0, fx1, fy1))
    return out


def _perimeter_segments_in_field_in(cfg: dict, field_w: float, field_h: float) -> List[Tuple[float, float, float, float]]:
    """Handle perimeter segments in field in."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    map_defaults = mcl.get("map_objects", {}) if isinstance(mcl, dict) else {}
    if int(map_defaults.get("perimeter", 1)) != 1:
        return []
    hx = field_h * 0.5
    hy = field_w * 0.5
    return [
        (-hx, -hy, hx, -hy),
        (hx, -hy, hx, hy),
        (hx, hy, -hx, hy),
        (-hx, hy, -hx, -hy),
    ]


def _object_segments_in_field_in(cfg: dict, field_w: float, field_h: float) -> List[Tuple[float, float, float, float]]:
    """Handle object segments in field in."""
    segs: List[Tuple[float, float, float, float]] = []
    for entry in _enabled_object_entries(cfg):
        poly = entry.get("poly", [])
        if not poly:
            continue
        pts = [_px_to_field_in(p[0], p[1], field_w, field_h) for p in poly]
        if len(pts) < 2:
            continue
        for i in range(len(pts)):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % len(pts)]
            segs.append((x0, y0, x1, y1))
    return segs


def _object_polys_in_field_in(cfg: dict, field_w: float, field_h: float) -> List[List[Tuple[float, float]]]:
    """Handle object polys in field in."""
    polys = []
    for entry in _enabled_object_entries(cfg):
        poly = entry.get("poly", [])
        if not poly:
            continue
        pts = [_px_to_field_in(p[0], p[1], field_w, field_h) for p in poly]
        polys.append(pts)
    return polys


def _enabled_object_entries(cfg: dict) -> List[dict]:
    """Handle enabled object entries."""
    out = []
    for entry in get_field_object_entries(cfg):
        if _entry_enabled(cfg, entry):
            out.append(entry)
    return out


def _emit_segments(segs: List[Tuple[float, float, float, float]]) -> List[str]:
    """Handle emit segments."""
    if not segs:
        return [
            "static const Segment MAP_SEGMENTS[] = {{0.0, 0.0, 0.0, 0.0}};",
            "static const int MAP_SEGMENT_COUNT = 0;",
        ]
    lines = ["static const Segment MAP_SEGMENTS[] = {"]
    for x0, y0, x1, y1 in segs:
        lines.append(f"  {{{_fmt(x0)}, {_fmt(y0)}, {_fmt(x1)}, {_fmt(y1)}}},")
    lines.append("};")
    lines.append(f"static const int MAP_SEGMENT_COUNT = {len(segs)};")
    return lines


def _emit_polys(polys: List[List[Tuple[float, float]]]) -> List[str]:
    """Handle emit polys."""
    if not polys:
        return [
            "static const double OBJECT_POINTS[][2] = {{0.0, 0.0}};",
            "static const PolyIndex OBJECT_POLYS[] = {{0, 0}};",
            "static const int OBJECT_POLY_COUNT = 0;",
        ]
    points: List[Tuple[float, float]] = []
    indices: List[Tuple[int, int]] = []
    for poly in polys:
        offset = len(points)
        for pt in poly:
            points.append(pt)
        indices.append((offset, len(poly)))
    lines = ["static const double OBJECT_POINTS[][2] = {"]
    for x, y in points:
        lines.append(f"  {{{_fmt(x)}, {_fmt(y)}}},")
    lines.append("};")
    lines.append("static const PolyIndex OBJECT_POLYS[] = {")
    for offset, count in indices:
        lines.append(f"  {{{offset}, {count}}},")
    lines.append("};")
    lines.append(f"static const int OBJECT_POLY_COUNT = {len(indices)};")
    return lines


def _dist_point_seg_sq(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    """Handle dist point seg sq."""
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        ex = px - x0
        ey = py - y0
        return ex * ex + ey * ey
    t = ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)
    if t < 0.0:
        t = 0.0
    if t > 1.0:
        t = 1.0
    cx = x0 + t * dx
    cy = y0 + t * dy
    ex = px - cx
    ey = py - cy
    return ex * ex + ey * ey


def _build_distance_field(segs: List[Tuple[float, float, float, float]], field_w: float, field_h: float,
                          res_in: float):
    """Build distance field."""
    if res_in <= 1e-6 or not segs:
        return None
    nx = int(math.floor(field_h / res_in)) + 1
    ny = int(math.floor(field_w / res_in)) + 1
    x0 = -field_h * 0.5
    y0 = -field_w * 0.5
    grid: List[float] = []
    for j in range(ny):
        y = y0 + j * res_in
        for i in range(nx):
            x = x0 + i * res_in
            best = 1e18
            for sx0, sy0, sx1, sy1 in segs:
                d2 = _dist_point_seg_sq(x, y, sx0, sy0, sx1, sy1)
                if d2 < best:
                    best = d2
            grid.append(math.sqrt(best))
    return {
        "nx": nx,
        "ny": ny,
        "res": res_in,
        "x0": x0,
        "y0": y0,
        "grid": grid,
    }


def build_mcl_config_hpp(cfg: dict) -> str:
    """Build mcl config hpp."""
    cfg = _unwrap_values(cfg or {})
    mcl = cfg.get("mcl", {})
    parts = mcl.get("particles", {})
    n = int(parts.get("n", parts.get("n_min", 200)))
    n_min = int(parts.get("n_min", n))
    n_max = int(parts.get("n_max", max(n, n_min)))
    kld = mcl.get("kld", {})
    kld_enabled = int(kld.get("enabled", 0)) == 1
    particle_count = n
    particle_capacity = max(n, n_max)
    motion = mcl.get("motion", {})
    sensors = mcl.get("sensors", {})
    dist = sensors.get("distance", {})
    imu = sensors.get("imu", {})
    vision = sensors.get("vision", {})
    resample = mcl.get("resample", {})
    region = mcl.get("region", {})
    conf = mcl.get("confidence", {})
    ekf = mcl.get("ekf", {})
    tuning = mcl.get("tuning", {}) if isinstance(mcl, dict) else {}
    aug = mcl.get("augmented", {})

    def _to_float(v, default=0.0):
        """Handle to float."""
        v = _extract_value(v)
        try:
            return float(v)
        except Exception:
            return float(default)

    def _to_int(v, default=0):
        """Handle to int."""
        v = _extract_value(v)
        try:
            return int(v)
        except Exception:
            try:
                return int(float(v))
            except Exception:
                return int(default)

    ekf_mcl_min_conf = _to_float(ekf.get("mcl_min_conf", mcl.get("correction", {}).get("min_confidence", 0.6)), 0.6)
    ekf_ambiguous_conf = _to_float(ekf.get("ambiguous_conf", max(0.0, ekf_mcl_min_conf * 0.75)), max(0.0, ekf_mcl_min_conf * 0.75))
    ekf_recover_conf = _to_float(ekf.get("recover_conf", ekf_mcl_min_conf), ekf_mcl_min_conf)
    ekf_recover_stable_updates = max(1, _to_int(ekf.get("recover_stable_updates", 3), 3))
    ekf_fusion_mode = max(0, _to_int(ekf.get("fusion_mode", 2), 2))
    ekf_track_enabled = _to_int(ekf.get("track_correction_enabled", 1), 1)
    ekf_track_min_conf = _to_float(ekf.get("track_min_conf", ekf_mcl_min_conf), ekf_mcl_min_conf)
    ekf_track_max_trans = _to_float(ekf.get("track_max_trans_in", 0.0), 0.0)
    ekf_track_max_theta = _to_float(ekf.get("track_max_theta_deg", 0.0), 0.0)
    ekf_intervene_min_conf = _to_float(ekf.get("intervene_min_conf", ekf_recover_conf), ekf_recover_conf)
    ekf_intervene_trans = _to_float(ekf.get("intervene_trans_in", 0.0), 0.0)
    ekf_intervene_theta = _to_float(ekf.get("intervene_theta_deg", 0.0), 0.0)
    ekf_intervene_p_xy = _to_float(ekf.get("intervene_p_xy_in2", 0.0), 0.0)
    ekf_intervene_p_theta = _to_float(ekf.get("intervene_p_theta_deg2", 0.0), 0.0)
    ekf_intervene_hard_trans = _to_float(ekf.get("intervene_hard_reset_trans_in", 0.0), 0.0)
    ekf_intervene_hard_theta = _to_float(ekf.get("intervene_hard_reset_theta_deg", 0.0), 0.0)
    ekf_intervene_cooldown_ms = max(0, _to_int(ekf.get("intervene_cooldown_ms", 250), 250))
    ekf_intervene_use_reset = _to_int(ekf.get("intervene_use_reset", 0), 0)
    ekf_recover_use_reset = _to_int(ekf.get("recover_use_reset", 0), 0)
    dist_sensors = get_distance_sensors(cfg)
    enabled_object_entries = _enabled_object_entries(cfg)
    enabled_object_ids = [str(e.get("id", "")) for e in enabled_object_entries if str(e.get("id", ""))]
    sensor_obj_vis = mcl.get("sensor_object_visibility", {}) if isinstance(mcl, dict) else {}
    if not isinstance(sensor_obj_vis, dict):
        sensor_obj_vis = {}

    def _sensor_object_mask(sensor: dict, sensor_idx: int) -> int:
        """Handle sensor object mask."""
        count = len(enabled_object_ids)
        if count <= 0:
            return 0
        if count > 63:
            return 0xffffffffffffffff
        full_mask = (1 << count) - 1
        name = str(sensor.get("name", "")).strip()
        vis_raw = sensor_obj_vis.get(str(sensor_idx), {})
        if not isinstance(vis_raw, dict):
            vis_raw = sensor_obj_vis.get(name, {})
        if not isinstance(vis_raw, dict):
            vis_raw = sensor_obj_vis.get(f"sensor_{sensor_idx + 1}", {})
        if not isinstance(vis_raw, dict):
            return full_mask
        mask = 0
        for bit, obj_id in enumerate(enabled_object_ids):
            try:
                on = int(vis_raw.get(obj_id, 1))
            except Exception:
                on = 1
            if on == 1:
                mask |= (1 << bit)
        return mask
    bd = cfg.get("bot_dimensions", {})
    bot_width = float(bd.get("width", 14.0) or 14.0)
    bot_length = float(bd.get("length", 14.0) or 14.0)
    bot_off_x = float(bd.get("full_offset_x_in", 0.0) or 0.0)
    bot_off_y = float(bd.get("full_offset_y_in", 0.0) or 0.0)
    field_w = WINDOW_WIDTH / PPI
    field_h = WINDOW_HEIGHT / PPI
    dist_model = str(dist.get("model", "likelihood_field")).strip().lower()
    dist_model_val = 0 if dist_model == "likelihood_field" else 1
    dist_gate_mode = str(dist.get("gate_mode", "hard")).strip().lower()
    dist_gate_mode_val = 1 if dist_gate_mode == "soft" else 0
    res_method = str(resample.get("method", "systematic")).strip().lower()
    res_method_val = {"systematic": 0, "stratified": 1, "multinomial": 2}.get(res_method, 0)
    region_mode = str(region.get("mode", "hard")).strip().lower()
    region_mode_val = 0 if region_mode == "hard" else 1
    obj_mode_raw = region.get("object_mode", None)
    if obj_mode_raw is None:
        try:
            obj_mode = 2 if int(region.get("object_gate", 0)) == 1 else 0
        except Exception:
            obj_mode = 0
    else:
        if isinstance(obj_mode_raw, str):
            obj_mode_str = obj_mode_raw.strip().lower()
            if obj_mode_str in ("off", "0", "false", "none"):
                obj_mode = 0
            elif obj_mode_str in ("soft", "1", "penalty"):
                obj_mode = 1
            elif obj_mode_str in ("hard", "2", "gate", "reject"):
                obj_mode = 2
            else:
                try:
                    obj_mode = int(obj_mode_raw)
                except Exception:
                    obj_mode = 0
        else:
            try:
                obj_mode = int(obj_mode_raw)
            except Exception:
                obj_mode = 0
    obj_mode = max(0, min(2, obj_mode))
    obj_clip_free = float(region.get("object_clip_free_in", 0.5))
    obj_clip_max = float(region.get("object_clip_max_in", 2.0))
    obj_clip_sigma = float(region.get("object_clip_sigma_in", 0.75))
    conf_mode = str(conf.get("reinit_mode", "global")).strip().lower()
    conf_mode_val = 1 if conf_mode == "estimate" else 0
    region_x_min = float(region.get("x_min_in", 0.0))
    region_x_max = float(region.get("x_max_in", field_w))
    region_y_min = float(region.get("y_min_in", 0.0))
    region_y_max = float(region.get("y_max_in", field_h))
    x_min_f = field_h * 0.5 - region_y_min
    x_max_f = field_h * 0.5 - region_y_max
    y_min_f = field_w * 0.5 - region_x_min
    y_max_f = field_w * 0.5 - region_x_max
    region_x_min_f = min(x_min_f, x_max_f)
    region_x_max_f = max(x_min_f, x_max_f)
    region_y_min_f = min(y_min_f, y_max_f)
    region_y_max_f = max(y_min_f, y_max_f)
    seg_hash = _segments_in_field_in(cfg, field_w, field_h)
    perim_hash = _perimeter_segments_in_field_in(cfg, field_w, field_h)
    obj_hash = _object_segments_in_field_in(cfg, field_w, field_h)
    poly_hash = _object_polys_in_field_in(cfg, field_w, field_h)
    def _round_geom(v):
        """Handle round geom."""
        if isinstance(v, (list, tuple)):
            return [_round_geom(x) for x in v]
        try:
            return round(float(v), 4)
        except Exception:
            return v
    hash_payload = {
        "mcl": mcl,
        "bot_dimensions": {
            "width": bot_width,
            "length": bot_length,
            "offset_x_in": bot_off_x,
            "offset_y_in": bot_off_y,
        },
        "field": {"width_in": field_w, "height_in": field_h},
        "distance_sensors": dist_sensors,
        "geometry": {
            "segments": _round_geom(seg_hash),
            "perimeter": _round_geom(perim_hash),
            "objects": _round_geom(obj_hash),
            "polys": _round_geom(poly_hash),
        },
    }
    cfg_hash = _config_hash(hash_payload)
    cfg_hash32 = cfg_hash[:8]
    rates = mcl.get("rates", {}) if isinstance(mcl, dict) else {}
    motion_ms = int(mcl.get("motion_ms", rates.get("motion_ms", 20)))
    sensor_ms = int(mcl.get("sensor_ms", rates.get("sensor_ms", 50)))
    object_count = len(enabled_object_ids)
    if object_count <= 0:
        object_mask_all_literal = "0x0ull"
    elif object_count >= 64:
        object_mask_all_literal = "0xffffffffffffffffull"
    else:
        object_mask_all_literal = f"{hex((1 << object_count) - 1)}ull"

    lines = [
        "#pragma once",
        "// Auto-generated by Atticus Terminal (MCL)",
        "#include <cstdint>",
        "",
        "constexpr double MCL_MM_PER_IN = 25.4;",
        f"constexpr double MCL_FIELD_WIDTH_IN = {_fmt(field_w)};",
        f"constexpr double MCL_FIELD_HEIGHT_IN = {_fmt(field_h)};",
        f"constexpr double MCL_FIELD_HALF_WIDTH_IN = {_fmt(field_w * 0.5)};",
        f"constexpr double MCL_FIELD_HALF_HEIGHT_IN = {_fmt(field_h * 0.5)};",
        f"constexpr double MCL_BOT_WIDTH_IN = {_fmt(bot_width)};",
        f"constexpr double MCL_BOT_LENGTH_IN = {_fmt(bot_length)};",
        f"constexpr double MCL_BOT_OFFSET_X_IN = {_fmt(bot_off_x)};",
        f"constexpr double MCL_BOT_OFFSET_Y_IN = {_fmt(bot_off_y)};",
        f"constexpr const char* MCL_CONFIG_HASH = \"{cfg_hash}\";",
        f"constexpr unsigned int MCL_CONFIG_HASH32 = 0x{cfg_hash32}u;",
        "",
        f"constexpr int MCL_PARTICLE_COUNT = {particle_count};",
        f"constexpr int MCL_PARTICLE_CAPACITY = {particle_capacity};",
        "static_assert(MCL_PARTICLE_COUNT > 0, \"MCL_PARTICLE_COUNT must be > 0\");",
        "static_assert(MCL_PARTICLE_COUNT <= MCL_PARTICLE_CAPACITY, \"MCL_PARTICLE_COUNT exceeds capacity\");",
        f"constexpr int MCL_N_MIN = {n_min};",
        f"constexpr int MCL_N_MAX = {n_max};",
        f"constexpr bool MCL_KLD_ENABLED = {_bool(kld_enabled)};",
        f"constexpr double MCL_KLD_EPSILON = {_fmt(kld.get('epsilon', 0.05))};",
        f"constexpr double MCL_KLD_DELTA = {_fmt(kld.get('delta', 0.99))};",
        f"constexpr double MCL_KLD_BIN_XY_IN = {_fmt(kld.get('bin_xy_in', 2.0))};",
        f"constexpr double MCL_KLD_BIN_THETA_DEG = {_fmt(kld.get('bin_theta_deg', 10.0))};",
        "",
        f"constexpr int MCL_MOTION_UPDATE_MS = {motion_ms};",
        f"constexpr int MCL_SENSOR_UPDATE_MS = {sensor_ms};",
        "",
        f"constexpr bool MCL_USE_MOTION = {_bool(motion.get('enabled', 1))};",
        f"constexpr bool MCL_USE_ALPHA_MODEL = {_bool(motion.get('use_alpha_model', 0))};",
        f"constexpr double MCL_MOTION_SIGMA_X_IN = {_fmt(motion.get('sigma_x_in', 0.1275))};",
        f"constexpr double MCL_MOTION_SIGMA_Y_IN = {_fmt(motion.get('sigma_y_in', 0.1275))};",
        f"constexpr double MCL_MOTION_SIGMA_THETA_DEG = {_fmt(motion.get('sigma_theta_deg', 1.0))};",
        f"constexpr double MCL_ALPHA1 = {_fmt(motion.get('alpha1', 0.05))};",
        f"constexpr double MCL_ALPHA2 = {_fmt(motion.get('alpha2', 0.05))};",
        f"constexpr double MCL_ALPHA3 = {_fmt(motion.get('alpha3', 0.05))};",
        f"constexpr double MCL_ALPHA4 = {_fmt(motion.get('alpha4', 0.05))};",
        f"constexpr double MCL_SETPOSE_SIGMA_XY_IN = {_fmt(mcl.get('set_pose_sigma_xy_in', 0.2))};",
        f"constexpr double MCL_SETPOSE_SIGMA_THETA_DEG = {_fmt(mcl.get('set_pose_sigma_theta_deg', 2.0))};",
        "",
        f"constexpr bool MCL_USE_DISTANCE = {_bool(dist.get('enabled', 1))};",
        f"constexpr int MCL_DISTANCE_MODEL = {dist_model_val};",
        f"constexpr double MCL_DIST_SIGMA_HIT_MM = {_fmt(dist.get('sigma_hit_mm', 8.5))};",
        f"constexpr double MCL_DIST_W_HIT = {_fmt(dist.get('w_hit', 0.9))};",
        f"constexpr double MCL_DIST_W_RAND = {_fmt(dist.get('w_rand', 0.1))};",
        f"constexpr double MCL_DIST_W_SHORT = {_fmt(dist.get('w_short', 0.0))};",
        f"constexpr double MCL_DIST_W_MAX = {_fmt(dist.get('w_max', 0.0))};",
        f"constexpr double MCL_DIST_LAMBDA_SHORT = {_fmt(dist.get('lambda_short', 0.1))};",
        f"constexpr double MCL_DIST_GATE_MM = {_fmt(dist.get('gate_mm', 150.0))};",
        f"constexpr int MCL_DIST_GATE_MODE = {dist_gate_mode_val};",
        f"constexpr double MCL_DIST_GATE_PENALTY = {_fmt(dist.get('gate_penalty', 0.05))};",
        f"constexpr double MCL_DIST_GATE_REJECT_RATIO = {_fmt(dist.get('gate_reject_ratio', 0.9))};",
        f"constexpr double MCL_DIST_MAX_RANGE_MM = {_fmt(dist.get('max_range_mm', 2000.0))};",
        f"constexpr double MCL_DIST_MIN_RANGE_MM = {_fmt(dist.get('min_range_mm', 0.0))};",
        f"constexpr double MCL_DIST_CONFIDENCE_MIN = {_fmt(dist.get('confidence_min', 0.0))};",
        f"constexpr double MCL_DIST_OBJECT_SIZE_MIN = {_fmt(dist.get('object_size_min', 0.0))};",
        f"constexpr double MCL_DIST_OBJECT_SIZE_MAX = {_fmt(dist.get('object_size_max', 0.0))};",
        f"constexpr double MCL_DIST_INNOVATION_GATE_MM = {_fmt(dist.get('innovation_gate_mm', 0.0))};",
        f"constexpr double MCL_DIST_INNOVATION_GATE_MIN_CONF = {_fmt(dist.get('innovation_gate_min_conf', max(_to_float(conf.get('threshold', 0.0), 0.0), 0.65)))};",
        f"constexpr int MCL_DIST_MEDIAN_WINDOW = {int(dist.get('median_window', 3) or 0)};",
        f"constexpr bool MCL_LF_IGNORE_MAX = {_bool(dist.get('lf_ignore_max', 0))};",
        "",
        f"constexpr bool MCL_USE_IMU = {_bool(imu.get('enabled', 1))};",
        f"constexpr double MCL_IMU_SIGMA_DEG = {_fmt(imu.get('sigma_deg', 1.0))};",
        "",
        f"constexpr bool MCL_USE_VISION = {_bool(vision.get('enabled', 0))};",
        f"constexpr double MCL_VISION_SIGMA_XY_IN = {_fmt(vision.get('sigma_xy_in', 2.0))};",
        f"constexpr double MCL_VISION_SIGMA_THETA_DEG = {_fmt(vision.get('sigma_theta_deg', 5.0))};",
        f"constexpr double MCL_VISION_CONFIDENCE_MIN = {_fmt(vision.get('confidence_min', 0.0))};",
        "",
        f"constexpr double MCL_RESAMPLE_THRESHOLD = {_fmt(resample.get('threshold', 0.5))};",
        f"constexpr bool MCL_RESAMPLE_ALWAYS = {_bool(resample.get('always', 0))};",
        f"constexpr int MCL_RESAMPLE_METHOD = {res_method_val};",
        f"constexpr double MCL_RANDOM_INJECTION = {_fmt(mcl.get('random_injection', 0.01))};",
        f"constexpr bool MCL_AUGMENTED_ENABLED = {_bool(aug.get('enabled', 0))};",
        f"constexpr double MCL_ALPHA_SLOW = {_fmt(aug.get('alpha_slow', 0.001))};",
        f"constexpr double MCL_ALPHA_FAST = {_fmt(aug.get('alpha_fast', 0.1))};",
        "",
        f"constexpr bool MCL_REGION_ENABLED = {_bool(region.get('enabled', 1))};",
        f"constexpr int MCL_REGION_MODE = {region_mode_val};",
        f"constexpr double MCL_REGION_PENALTY = {_fmt(region.get('penalty', 0.2))};",
        f"constexpr bool MCL_REGION_PERIMETER_GATE = {_bool(region.get('perimeter_gate', 1))};",
        f"constexpr int MCL_OBJECT_MODE = {obj_mode};",
        f"constexpr double MCL_OBJECT_CLIP_FREE_IN = {_fmt(obj_clip_free)};",
        f"constexpr double MCL_OBJECT_CLIP_MAX_IN = {_fmt(obj_clip_max)};",
        f"constexpr double MCL_OBJECT_CLIP_SIGMA_IN = {_fmt(obj_clip_sigma)};",
        f"constexpr bool MCL_REGION_OBJECT_GATE = {'true' if obj_mode == 2 else 'false'};",
        f"constexpr double MCL_REGION_X_MIN_IN = {_fmt(region_x_min_f)};",
        f"constexpr double MCL_REGION_X_MAX_IN = {_fmt(region_x_max_f)};",
        f"constexpr double MCL_REGION_Y_MIN_IN = {_fmt(region_y_min_f)};",
        f"constexpr double MCL_REGION_Y_MAX_IN = {_fmt(region_y_max_f)};",
        f"constexpr int MCL_REGION_SAMPLE_ATTEMPTS = {int(region.get('sample_attempts', 50))};",
        "",
        f"constexpr double MCL_CONFIDENCE_THRESHOLD = {_fmt(conf.get('threshold', 0.0))};",
        f"constexpr bool MCL_CONFIDENCE_AUTO_REINIT = {_bool(conf.get('auto_reinit', 0))};",
        f"constexpr int MCL_REINIT_MODE = {conf_mode_val};",
        "",
        f"constexpr bool MCL_CORR_ENABLED = {_bool(mcl.get('correction', {}).get('enabled', 1))};",
        f"constexpr double MCL_CORR_MIN_CONF = {_fmt(mcl.get('correction', {}).get('min_confidence', 0.6))};",
        f"constexpr double MCL_CORR_MAX_TRANS_JUMP_IN = {_fmt(mcl.get('correction', {}).get('max_trans_jump_in', 8.0))};",
        f"constexpr double MCL_CORR_MAX_THETA_JUMP_DEG = {_fmt(mcl.get('correction', {}).get('max_theta_jump_deg', 15.0))};",
        f"constexpr double MCL_CORR_ALPHA_MIN = {_fmt(mcl.get('correction', {}).get('alpha_min', 0.03))};",
        f"constexpr double MCL_CORR_ALPHA_MAX = {_fmt(mcl.get('correction', {}).get('alpha_max', 0.12))};",
        "",
        f"constexpr bool MCL_EKF_ENABLED = {_bool(ekf.get('enabled', 1))};",
        f"constexpr double MCL_EKF_MCL_MIN_CONF = {_fmt(ekf_mcl_min_conf)};",
        f"constexpr double MCL_EKF_SIGMA_DX_IN = {_fmt(ekf.get('sigma_dx_in', motion.get('sigma_x_in', 0.1275)))};",
        f"constexpr double MCL_EKF_SIGMA_DY_IN = {_fmt(ekf.get('sigma_dy_in', motion.get('sigma_y_in', 0.1275)))};",
        f"constexpr double MCL_EKF_SIGMA_DTHETA_DEG = {_fmt(ekf.get('sigma_dtheta_deg', motion.get('sigma_theta_deg', 1.0)))};",
        f"constexpr double MCL_EKF_IMU_SIGMA_DEG = {_fmt(ekf.get('imu_sigma_deg', imu.get('sigma_deg', 1.0)))};",
        f"constexpr bool MCL_EKF_USE_IMU = {_bool(ekf.get('use_imu_update', 1))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_X_MIN_IN = {_fmt(ekf.get('mcl_sigma_x_min', mcl.get('set_pose_sigma_xy_in', 0.2)))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_X_MAX_IN = {_fmt(ekf.get('mcl_sigma_x_max', 6.0))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_Y_MIN_IN = {_fmt(ekf.get('mcl_sigma_y_min', mcl.get('set_pose_sigma_xy_in', 0.2)))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_Y_MAX_IN = {_fmt(ekf.get('mcl_sigma_y_max', 6.0))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_THETA_MIN_DEG = {_fmt(ekf.get('mcl_sigma_theta_min', mcl.get('set_pose_sigma_theta_deg', 2.0)))};",
        f"constexpr double MCL_EKF_MCL_SIGMA_THETA_MAX_DEG = {_fmt(ekf.get('mcl_sigma_theta_max', 15.0))};",
        f"constexpr double MCL_EKF_INIT_SIGMA_XY_IN = {_fmt(ekf.get('init_sigma_xy_in', mcl.get('set_pose_sigma_xy_in', 0.2)))};",
        f"constexpr double MCL_EKF_INIT_SIGMA_THETA_DEG = {_fmt(ekf.get('init_sigma_theta_deg', mcl.get('set_pose_sigma_theta_deg', 2.0)))};",
        f"constexpr double MCL_EKF_MCL_INNOVATION_GATE_XY_IN = {_fmt(ekf.get('mcl_innovation_gate_xy_in', 0.0))};",
        f"constexpr double MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG = {_fmt(ekf.get('mcl_innovation_gate_theta_deg', 0.0))};",
        f"constexpr double MCL_EKF_AMBIGUOUS_CONF = {_fmt(ekf_ambiguous_conf)};",
        f"constexpr double MCL_EKF_RECOVER_CONF = {_fmt(ekf_recover_conf)};",
        f"constexpr int MCL_EKF_RECOVER_STABLE_UPDATES = {ekf_recover_stable_updates};",
        f"constexpr int MCL_EKF_FUSION_MODE = {ekf_fusion_mode};",
        f"constexpr bool MCL_EKF_TRACK_CORRECTION_ENABLED = {_bool(ekf_track_enabled)};",
        f"constexpr double MCL_EKF_TRACK_MIN_CONF = {_fmt(ekf_track_min_conf)};",
        f"constexpr double MCL_EKF_TRACK_MAX_TRANS_IN = {_fmt(ekf_track_max_trans)};",
        f"constexpr double MCL_EKF_TRACK_MAX_THETA_DEG = {_fmt(ekf_track_max_theta)};",
        f"constexpr double MCL_EKF_INTERVENE_MIN_CONF = {_fmt(ekf_intervene_min_conf)};",
        f"constexpr double MCL_EKF_INTERVENE_TRANS_IN = {_fmt(ekf_intervene_trans)};",
        f"constexpr double MCL_EKF_INTERVENE_THETA_DEG = {_fmt(ekf_intervene_theta)};",
        f"constexpr double MCL_EKF_INTERVENE_P_XY_IN2 = {_fmt(ekf_intervene_p_xy)};",
        f"constexpr double MCL_EKF_INTERVENE_P_THETA_DEG2 = {_fmt(ekf_intervene_p_theta)};",
        f"constexpr double MCL_EKF_INTERVENE_HARD_RESET_TRANS_IN = {_fmt(ekf_intervene_hard_trans)};",
        f"constexpr double MCL_EKF_INTERVENE_HARD_RESET_THETA_DEG = {_fmt(ekf_intervene_hard_theta)};",
        f"constexpr int MCL_EKF_INTERVENE_COOLDOWN_MS = {ekf_intervene_cooldown_ms};",
        f"constexpr bool MCL_EKF_INTERVENE_USE_RESET = {_bool(ekf_intervene_use_reset)};",
        f"constexpr bool MCL_EKF_RECOVER_USE_RESET = {_bool(ekf_recover_use_reset)};",
        "",
        f"constexpr bool MCL_TUNING_ENABLED = {_bool(tuning.get('enabled', 0))};",
        f"constexpr int MCL_TUNING_LOG_RATE_HZ = {int(tuning.get('log_rate_hz', 20) or 20)};",
        f"constexpr int MCL_TUNING_PARTICLE_SUBSAMPLE = {int(tuning.get('particle_subsample', 0) or 0)};",
        "",
        "struct MCLDistanceSensorConfig {",
        "  double x_in;",
        "  double y_in;",
        "  double angle_deg;",
        "  double bias_mm;",
        "  double angle_offset_deg;",
        "  double min_range_mm;",
        "  double max_range_mm;",
        "  double min_confidence;",
        "  double min_object_size;",
        "  double max_object_size;",
        "  double innovation_gate_mm;",
        "  int map_mode;",
        "  uint64_t object_mask;",
        "};",
        f"constexpr int MCL_OBJECT_COUNT = {len(enabled_object_ids)};",
        f"constexpr uint64_t MCL_OBJECT_MASK_ALL = {object_mask_all_literal};",
        f"constexpr int MCL_DISTANCE_SENSOR_COUNT = {len(dist_sensors)};",
        "constexpr int MCL_DISTANCE_SENSOR_COUNT_SAFE = "
        "(MCL_DISTANCE_SENSOR_COUNT > 0 ? MCL_DISTANCE_SENSOR_COUNT : 1);",
        "static constexpr MCLDistanceSensorConfig MCL_DISTANCE_SENSORS[MCL_DISTANCE_SENSOR_COUNT_SAFE] = {",
    ]
    def _map_mode(val: str) -> int:
        """Handle map mode."""
        mode = str(val or "both").strip().lower()
        if mode == "perimeter":
            return 1
        if mode == "objects":
            return 2
        return 0
    if dist_sensors:
        for idx, sensor in enumerate(dist_sensors):
            lines.append(
                "  {" +
                f"{_fmt(sensor.get('x_in', 0.0))}, "
                f"{_fmt(sensor.get('y_in', 0.0))}, "
                f"{_fmt(sensor.get('angle_deg', 0.0))}, "
                f"{_fmt(sensor.get('bias_mm', 0.0))}, "
                f"{_fmt(sensor.get('angle_offset_deg', 0.0))}, "
                f"{_fmt(sensor.get('min_range_mm', 0.0))}, "
                f"{_fmt(sensor.get('max_range_mm', 0.0))}, "
                f"{_fmt(sensor.get('min_confidence', 0.0))}, "
                f"{_fmt(sensor.get('min_object_size', 0.0))}, "
                f"{_fmt(sensor.get('max_object_size', 0.0))}, "
                f"{_fmt(sensor.get('innovation_gate_mm', 0.0))}, "
                f"{_map_mode(sensor.get('map_mode', 'both'))}, "
                f"{hex(_sensor_object_mask(sensor, idx))}ull"
                "},"
            )
    else:
        lines.append("  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0xffffffffffffffffull},")
    lines.append("};")
    lines.append("")
    lines.append("constexpr int MCL_MAP_MODE_BOTH = 0;")
    lines.append("constexpr int MCL_MAP_MODE_PERIMETER = 1;")
    lines.append("constexpr int MCL_MAP_MODE_OBJECTS = 2;")
    lines.append("")
    lines.append("constexpr int MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD = 0;")
    lines.append("constexpr int MCL_DISTANCE_MODEL_BEAM = 1;")
    lines.append("constexpr int MCL_RESAMPLE_SYSTEMATIC = 0;")
    lines.append("constexpr int MCL_RESAMPLE_STRATIFIED = 1;")
    lines.append("constexpr int MCL_RESAMPLE_MULTINOMIAL = 2;")
    lines.append("constexpr int MCL_REGION_MODE_HARD = 0;")
    lines.append("constexpr int MCL_REGION_MODE_SOFT = 1;")
    lines.append("constexpr int MCL_GATE_MODE_HARD = 0;")
    lines.append("constexpr int MCL_GATE_MODE_SOFT = 1;")
    lines.append("constexpr int MCL_REINIT_MODE_GLOBAL = 0;")
    lines.append("constexpr int MCL_REINIT_MODE_ESTIMATE = 1;")
    lines.append("")
    return "\n".join(lines)

def build_mcl_map_data_h(cfg: dict) -> str:
    """Build mcl map data h."""
    lines = []
    lines.append("// mcl_map_data.h (generated)")
    lines.append("#pragma once")
    lines.append("")
    lines.append("// Likelihood-field distance grids used by distance sensors.")
    lines.append("extern const double MAP_DIST_FIELD[];")
    lines.append("extern const double MAP_DIST_FIELD_PERIM[];")
    lines.append("extern const double MAP_DIST_FIELD_OBJ[];")
    lines.append("")
    return "\n".join(lines) + "\n"


def build_mcl_map_data_cpp(cfg: dict) -> str:
    """Build mcl map data cpp."""
    cfg = _unwrap_values(cfg or {})
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    lf_cfg = dist_cfg.get("likelihood_field", {}) if isinstance(dist_cfg, dict) else {}

    use_lf = True
    try:
        if isinstance(dist_cfg, dict) and dist_cfg.get("enabled", 1) in (0, False):
            use_lf = False
        if isinstance(lf_cfg, dict) and lf_cfg.get("enabled", 1) in (0, False):
            use_lf = False
        model = str(dist_cfg.get("model", "likelihood_field")).strip().lower() if isinstance(dist_cfg, dict) else "likelihood_field"
        if model != "likelihood_field":
            use_lf = False
    except Exception:
        pass

    if use_lf:
        try:
            res_in = float(lf_cfg.get("resolution_in", 2.0))
        except Exception:
            res_in = 2.0

        field_w = float(WINDOW_WIDTH) / float(PPI)
        field_h = float(WINDOW_HEIGHT) / float(PPI)

        segs = _segments_in_field_in(cfg, field_w, field_h)
        perim_segs = _perimeter_segments_in_field_in(cfg, field_w, field_h)
        obj_segs = _object_segments_in_field_in(cfg, field_w, field_h)

        df_all = _build_distance_field(segs, field_w, field_h, res_in)
        df_perim = _build_distance_field(perim_segs, field_w, field_h, res_in)
        df_obj = _build_distance_field(obj_segs, field_w, field_h, res_in)
    else:
        df_all = None
        df_perim = None
        df_obj = None

    lines = []
    lines.append("// mcl_map_data.cpp (generated)")
    lines.append('#include "mcl_map_data.h"')
    lines.append("")

    def emit(df, name: str):
        """Handle emit."""
        if not df or not df.get("grid"):
            lines.append(f"extern const double {name}[] = {{0.0}};")
            lines.append("")
            return
        lines.append(f"extern const double {name}[] = {{")
        per_line = 48
        buf = []
        for v in df["grid"]:
            buf.append(_fmt(v))
            if len(buf) >= per_line:
                lines.append("  " + ", ".join(buf) + ",")
                buf = []
        if buf:
            lines.append("  " + ", ".join(buf) + ",")
        lines.append("};")
        lines.append("")

    emit(df_all, "MAP_DIST_FIELD")
    emit(df_perim, "MAP_DIST_FIELD_PERIM")
    emit(df_obj, "MAP_DIST_FIELD_OBJ")
    return "\n".join(lines) + "\n"


def build_mcl_runtime_hpp(cfg: dict) -> str:
    """Build mcl runtime hpp."""
    return "\n".join([
        "#pragma once",
        "#include \"api.h\"",
        "#include \"mcl_localizer.h\"",
        "#include <vector>",
        "#include <atomic>",
        "#include <cstdint>",
        "#include <memory>",
        "",
        "// Field-centric coordinates: +x forward, +y left; heading 0=left, 90=forward (clockwise).",
        "class ProsMCL {",
        " public:",
        "  using FieldPoseProvider = bool (*)(MCLPose* out_pose, void* user);",
        "  ProsMCL(int imu_port, const std::vector<int>& dist_ports);",
        "  void start(unsigned seed, double initial_heading_deg);",
        "  void startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg);",
        "  void stop();",
        "  // Robot-frame deltas: +x forward, +y left, +theta clockwise (deg).",
        "  void setOdomDelta(double dx_in, double dy_in, double dtheta_deg);",
        "  // Optional: provide field pose samples (e.g., LemLib) and let runtime compute odom deltas automatically.",
        "  void setFieldPoseProvider(FieldPoseProvider provider, void* user = nullptr);",
        "  void clearFieldPoseProvider();",
        "  void setPose(double x_in, double y_in, double theta_deg);",
        "  MCLPose getPose() const;",
        "  MCLPose getFusedPose() const;",
        "  void updateVision(double x_in, double y_in, double theta_deg, double confidence);",
        "  void setSegmentBand(const MCLPose* pts, int n, double radius_in);",
        "  void clearSegmentBand();",
        "  bool applyOdomCorrection(MCLPose& odom_pose, double* out_alpha = nullptr) const;",
        "  void requestRelocalize();",
        "  struct DebugSnapshot {",
        "    std::uint32_t time_ms;",
        "    MCLPose odom_pose;",
        "    MCLPose mcl_pose;",
        "    MCLPose fused_pose;",
        "    double mcl_confidence;",
        "    double mcl_neff;",
        "    double ekf_pxx;",
        "    double ekf_pyy;",
        "    double ekf_pxy;",
        "    double ekf_ptt;",
        "    std::uint32_t dist_used_mask;",
        "    std::uint32_t event_flags;",
        "    double dist_meas_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "    double dist_exp_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  };",
        "  DebugSnapshot getDebugSnapshot() const;",
        "  // Exposed so helper math functions in runtime can use the type without access issues.",
        "  struct EKFState {",
        "    double x;",
        "    double y;",
        "    double theta;",
        "    double P[3][3];",
        "    bool initialized;",
        "  };",
        "",
        " private:",
        "  void loop();",
        "",
        "  std::unique_ptr<pros::Imu> imu_;",
        "  std::vector<pros::Distance> dists_;",
        "  MCLLocalizer mcl_;",
        "",
        "  mutable pros::Mutex mu_;",
        "  pros::Task* task_;",
        "  std::atomic_bool running_;",
        "  std::atomic_bool task_done_;",
        "  std::atomic_bool relocalize_requested_;",
        "  FieldPoseProvider pose_provider_;",
        "  void* pose_provider_user_;",
        "  MCLPose provider_last_pose_;",
        "  bool provider_last_valid_;",
        "  double odom_dx_;",
        "  double odom_dy_;",
        "  double odom_dth_;",
        "  MCLPose pose_;",
        "  MCLPose fused_pose_;",
        "  EKFState ekf_;",
        "  bool mcl_ambiguous_;",
        "  int mcl_recover_good_count_;",
        "  std::uint32_t ekf_intervene_cooldown_until_ms_;",
        "  mutable std::uint32_t last_event_flags_;",
        "  static constexpr int DIST_MEDIAN_WINDOW = (MCL_DIST_MEDIAN_WINDOW > 1 ? MCL_DIST_MEDIAN_WINDOW : 1);",
        "  double dist_hist_[MCL_DISTANCE_SENSOR_COUNT_SAFE][DIST_MEDIAN_WINDOW];",
        "  int dist_hist_count_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int dist_hist_idx_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "};",
        "",
    ])

def build_mcl_runtime_cpp(cfg: dict) -> str:
    """Build mcl runtime cpp."""
    lines: List[str] = []
    lines.append("#include \"mcl_runtime.h\"")
    lines.append("#include \"mcl_config.hpp\"")
    lines.append("#include \"mcl_map_data.h\"")
    lines.append("#include <algorithm>")
    lines.append("#include <cstdint>")
    lines.append("#include <cmath>")
    lines.append("")
    lines.append("namespace {")
    lines.append("constexpr std::uint32_t MCL_EVENT_MCL_EKF_APPLIED = 1u << 0;")
    lines.append("constexpr std::uint32_t MCL_EVENT_ODOM_CORR_APPLIED = 1u << 1;")
    lines.append("constexpr std::uint32_t MCL_EVENT_RECOVERY_ACTIVE = 1u << 3;")
    lines.append("struct ScopedMutex {")
    lines.append("  explicit ScopedMutex(pros::Mutex& m) : m_(m) { m_.take(); }")
    lines.append("  ~ScopedMutex() { m_.give(); }")
    lines.append("  pros::Mutex& m_;")
    lines.append("};")
    lines.append("")
    lines.append("static double wrap_deg(double deg) {")
    lines.append("  while (deg >= 360.0) deg -= 360.0;")
    lines.append("  while (deg < 0.0) deg += 360.0;")
    lines.append("  return deg;")
    lines.append("}")
    lines.append("")
    lines.append("static double angle_diff_deg(double a, double b) {")
    lines.append("  double d = std::fmod(a - b + 180.0, 360.0);")
    lines.append("  if (d < 0.0) d += 360.0;")
    lines.append("  return d - 180.0;")
    lines.append("}")
    lines.append("")
    lines.append("static double clamp_sigma_pos(double v) {")
    lines.append("  return std::max(1e-4, std::fabs(v));")
    lines.append("}")
    lines.append("")
    lines.append("static double clamp_sigma_ang(double v) {")
    lines.append("  return std::max(1e-3, std::fabs(v));")
    lines.append("}")
    lines.append("")
    lines.append("static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {")
    lines.append("  double th = heading_deg * (3.14159265358979323846 / 180.0);")
    lines.append("  double s = std::sin(th);")
    lines.append("  double c = std::cos(th);")
    lines.append("  wx = lx * s - ly * c;")
    lines.append("  wy = lx * c + ly * s;")
    lines.append("}")
    lines.append("")
    lines.append("static void rotate_world_to_local(double wx, double wy, double heading_deg, double& lx, double& ly) {")
    lines.append("  double th = heading_deg * (3.14159265358979323846 / 180.0);")
    lines.append("  double s = std::sin(th);")
    lines.append("  double c = std::cos(th);")
    lines.append("  lx = s * wx + c * wy;")
    lines.append("  ly = -c * wx + s * wy;")
    lines.append("}")
    lines.append("")
    lines.append("static void mat3_mul(const double A[3][3], const double B[3][3], double out[3][3]) {")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      double sum = 0.0;")
    lines.append("      for (int k = 0; k < 3; ++k) {")
    lines.append("        sum += A[i][k] * B[k][j];")
    lines.append("      }")
    lines.append("      out[i][j] = sum;")
    lines.append("    }")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("static void mat3_transpose(const double A[3][3], double out[3][3]) {")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      out[i][j] = A[j][i];")
    lines.append("    }")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("static bool mat3_inv(const double A[3][3], double out[3][3]) {")
    lines.append("  double det =")
    lines.append("      A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -")
    lines.append("      A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +")
    lines.append("      A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);")
    lines.append("  if (std::fabs(det) < 1e-12) return false;")
    lines.append("  double inv_det = 1.0 / det;")
    lines.append("  out[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;")
    lines.append("  out[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;")
    lines.append("  out[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;")
    lines.append("  out[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;")
    lines.append("  out[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;")
    lines.append("  out[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;")
    lines.append("  out[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;")
    lines.append("  out[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;")
    lines.append("  out[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("static void ekf_reset(ProsMCL::EKFState& ekf, double x, double y, double theta) {")
    lines.append("  ekf.x = x;")
    lines.append("  ekf.y = y;")
    lines.append("  ekf.theta = wrap_deg(theta);")
    lines.append("  double sxy = MCL_EKF_INIT_SIGMA_XY_IN;")
    lines.append("  double sth = MCL_EKF_INIT_SIGMA_THETA_DEG;")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      ekf.P[i][j] = 0.0;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  ekf.P[0][0] = sxy * sxy;")
    lines.append("  ekf.P[1][1] = sxy * sxy;")
    lines.append("  ekf.P[2][2] = sth * sth;")
    lines.append("  ekf.initialized = true;")
    lines.append("}")
    lines.append("")
    lines.append("static void ekf_predict(ProsMCL::EKFState& ekf, double dx_in, double dy_in, double dtheta_deg) {")
    lines.append("  if (!ekf.initialized) return;")
    lines.append("  const double kDegToRad = 3.14159265358979323846 / 180.0;")
    lines.append("  double th = ekf.theta * kDegToRad;")
    lines.append("  double s = std::sin(th);")
    lines.append("  double c = std::cos(th);")
    lines.append("  ekf.x += dx_in * s - dy_in * c;")
    lines.append("  ekf.y += dx_in * c + dy_in * s;")
    lines.append("  ekf.theta = wrap_deg(ekf.theta + dtheta_deg);")
    lines.append("  double rad_per_deg = kDegToRad;")
    lines.append("  double F[3][3] = {")
    lines.append("    {1.0, 0.0, (dx_in * c + dy_in * s) * rad_per_deg},")
    lines.append("    {0.0, 1.0, (-dx_in * s + dy_in * c) * rad_per_deg},")
    lines.append("    {0.0, 0.0, 1.0}")
    lines.append("  };")
    lines.append("  double L[3][3] = {")
    lines.append("    {s, -c, 0.0},")
    lines.append("    {c,  s, 0.0},")
    lines.append("    {0.0, 0.0, 1.0}")
    lines.append("  };")
    lines.append("  double qdx = clamp_sigma_pos(MCL_EKF_SIGMA_DX_IN);")
    lines.append("  double qdy = clamp_sigma_pos(MCL_EKF_SIGMA_DY_IN);")
    lines.append("  double qth = clamp_sigma_ang(MCL_EKF_SIGMA_DTHETA_DEG);")
    lines.append("  double Q[3][3] = {")
    lines.append("    {qdx * qdx, 0.0, 0.0},")
    lines.append("    {0.0, qdy * qdy, 0.0},")
    lines.append("    {0.0, 0.0, qth * qth}")
    lines.append("  };")
    lines.append("  double FP[3][3];")
    lines.append("  double Ft[3][3];")
    lines.append("  double temp[3][3];")
    lines.append("  mat3_mul(F, ekf.P, FP);")
    lines.append("  mat3_transpose(F, Ft);")
    lines.append("  mat3_mul(FP, Ft, temp);")
    lines.append("  double LQ[3][3];")
    lines.append("  double Lt[3][3];")
    lines.append("  double LQLt[3][3];")
    lines.append("  mat3_mul(L, Q, LQ);")
    lines.append("  mat3_transpose(L, Lt);")
    lines.append("  mat3_mul(LQ, Lt, LQLt);")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      ekf.P[i][j] = temp[i][j] + LQLt[i][j];")
    lines.append("    }")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("static void ekf_update_imu(ProsMCL::EKFState& ekf, double heading_deg) {")
    lines.append("  if (!ekf.initialized) return;")
    lines.append("  double nu = angle_diff_deg(heading_deg, ekf.theta);")
    lines.append("  double imu_sigma = clamp_sigma_ang(MCL_EKF_IMU_SIGMA_DEG);")
    lines.append("  double R = imu_sigma * imu_sigma;")
    lines.append("  double S = ekf.P[2][2] + R;")
    lines.append("  if (S < 1e-12) return;")
    lines.append("  double K[3] = {ekf.P[0][2] / S, ekf.P[1][2] / S, ekf.P[2][2] / S};")
    lines.append("  ekf.x += K[0] * nu;")
    lines.append("  ekf.y += K[1] * nu;")
    lines.append("  ekf.theta = wrap_deg(ekf.theta + K[2] * nu);")
    lines.append("  double I_KH[3][3] = {")
    lines.append("    {1.0, 0.0, -K[0]},")
    lines.append("    {0.0, 1.0, -K[1]},")
    lines.append("    {0.0, 0.0, 1.0 - K[2]}")
    lines.append("  };")
    lines.append("  double temp[3][3];")
    lines.append("  double IKHt[3][3];")
    lines.append("  mat3_mul(I_KH, ekf.P, temp);")
    lines.append("  mat3_transpose(I_KH, IKHt);")
    lines.append("  double Pnew[3][3];")
    lines.append("  mat3_mul(temp, IKHt, Pnew);")
    lines.append("  double KRKt[3][3] = {")
    lines.append("    {K[0] * K[0] * R, K[0] * K[1] * R, K[0] * K[2] * R},")
    lines.append("    {K[1] * K[0] * R, K[1] * K[1] * R, K[1] * K[2] * R},")
    lines.append("    {K[2] * K[0] * R, K[2] * K[1] * R, K[2] * K[2] * R}")
    lines.append("  };")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      ekf.P[i][j] = Pnew[i][j] + KRKt[i][j];")
    lines.append("    }")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("static bool ekf_update_mcl(ProsMCL::EKFState& ekf, const MCLPose& pose, double confidence) {")
    lines.append("  if (!ekf.initialized) return false;")
    lines.append("  if (confidence < MCL_EKF_MCL_MIN_CONF) return false;")
    lines.append("  double nu_xy = std::sqrt((pose.x - ekf.x) * (pose.x - ekf.x) + (pose.y - ekf.y) * (pose.y - ekf.y));")
    lines.append("  double nu_th = std::fabs(angle_diff_deg(pose.theta, ekf.theta));")
    lines.append("  if (MCL_EKF_MCL_INNOVATION_GATE_XY_IN > 0.0 && nu_xy > MCL_EKF_MCL_INNOVATION_GATE_XY_IN) return false;")
    lines.append("  if (MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG > 0.0 && nu_th > MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG) return false;")
    lines.append("  double sx = MCL_EKF_MCL_SIGMA_X_MAX_IN - confidence * (MCL_EKF_MCL_SIGMA_X_MAX_IN - MCL_EKF_MCL_SIGMA_X_MIN_IN);")
    lines.append("  double sy = MCL_EKF_MCL_SIGMA_Y_MAX_IN - confidence * (MCL_EKF_MCL_SIGMA_Y_MAX_IN - MCL_EKF_MCL_SIGMA_Y_MIN_IN);")
    lines.append("  double sth = MCL_EKF_MCL_SIGMA_THETA_MAX_DEG - confidence * (MCL_EKF_MCL_SIGMA_THETA_MAX_DEG - MCL_EKF_MCL_SIGMA_THETA_MIN_DEG);")
    lines.append("  sx = clamp_sigma_pos(sx);")
    lines.append("  sy = clamp_sigma_pos(sy);")
    lines.append("  sth = clamp_sigma_ang(sth);")
    lines.append("  double R[3][3] = {")
    lines.append("    {sx * sx, 0.0, 0.0},")
    lines.append("    {0.0, sy * sy, 0.0},")
    lines.append("    {0.0, 0.0, sth * sth}")
    lines.append("  };")
    lines.append("  double S[3][3];")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      S[i][j] = ekf.P[i][j] + R[i][j];")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double S_inv[3][3];")
    lines.append("  if (!mat3_inv(S, S_inv)) return false;")
    lines.append("  double K[3][3];")
    lines.append("  mat3_mul(ekf.P, S_inv, K);")
    lines.append("  double nu[3] = {pose.x - ekf.x, pose.y - ekf.y, angle_diff_deg(pose.theta, ekf.theta)};")
    lines.append("  double dx = K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2];")
    lines.append("  double dy = K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2];")
    lines.append("  double dth = K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2];")
    lines.append("  ekf.x += dx;")
    lines.append("  ekf.y += dy;")
    lines.append("  ekf.theta = wrap_deg(ekf.theta + dth);")
    lines.append("  double I_K[3][3];")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      I_K[i][j] = (i == j ? 1.0 : 0.0) - K[i][j];")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double temp[3][3];")
    lines.append("  double IKt[3][3];")
    lines.append("  mat3_mul(I_K, ekf.P, temp);")
    lines.append("  mat3_transpose(I_K, IKt);")
    lines.append("  double Pnew[3][3];")
    lines.append("  mat3_mul(temp, IKt, Pnew);")
    lines.append("  double KR[3][3];")
    lines.append("  mat3_mul(K, R, KR);")
    lines.append("  double Kt[3][3];")
    lines.append("  mat3_transpose(K, Kt);")
    lines.append("  double KRKt[3][3];")
    lines.append("  mat3_mul(KR, Kt, KRKt);")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      ekf.P[i][j] = Pnew[i][j] + KRKt[i][j];")
    lines.append("    }")
    lines.append("  }")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("static bool ekf_covariance_needs_intervention(const ProsMCL::EKFState& ekf) {")
    lines.append("  if (!ekf.initialized) return false;")
    lines.append("  if (MCL_EKF_INTERVENE_P_XY_IN2 > 0.0) {")
    lines.append("    if (ekf.P[0][0] >= MCL_EKF_INTERVENE_P_XY_IN2 || ekf.P[1][1] >= MCL_EKF_INTERVENE_P_XY_IN2) return true;")
    lines.append("  }")
    lines.append("  if (MCL_EKF_INTERVENE_P_THETA_DEG2 > 0.0 && ekf.P[2][2] >= MCL_EKF_INTERVENE_P_THETA_DEG2) return true;")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static bool ekf_fusion_update(ProsMCL::EKFState& ekf, const MCLPose& pose, double conf, std::uint32_t now_ms,")
    lines.append("                              bool& mcl_ambiguous, int& mcl_recover_good_count,")
    lines.append("                              std::uint32_t& cooldown_until_ms) {")
    lines.append("  bool was_ambiguous = mcl_ambiguous;")
    lines.append("  if (MCL_EKF_AMBIGUOUS_CONF > 0.0 && conf < MCL_EKF_AMBIGUOUS_CONF) {")
    lines.append("    mcl_ambiguous = true;")
    lines.append("    mcl_recover_good_count = 0;")
    lines.append("  }")
    lines.append("  if (mcl_ambiguous) {")
    lines.append("    if (conf >= MCL_EKF_RECOVER_CONF) {")
    lines.append("      if (mcl_recover_good_count < MCL_EKF_RECOVER_STABLE_UPDATES) mcl_recover_good_count++;")
    lines.append("      if (mcl_recover_good_count >= MCL_EKF_RECOVER_STABLE_UPDATES) mcl_ambiguous = false;")
    lines.append("    } else {")
    lines.append("      mcl_recover_good_count = 0;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  bool recovered_now = was_ambiguous && !mcl_ambiguous;")
    lines.append("  if (!ekf.initialized) {")
    lines.append("    if (!mcl_ambiguous && conf >= MCL_EKF_MCL_MIN_CONF) {")
    lines.append("      ekf_reset(ekf, pose.x, pose.y, pose.theta);")
    lines.append("    }")
    lines.append("    return false;")
    lines.append("  }")
    lines.append("  if (mcl_ambiguous) return false;")
    lines.append("  double nu_xy = std::sqrt((pose.x - ekf.x) * (pose.x - ekf.x) + (pose.y - ekf.y) * (pose.y - ekf.y));")
    lines.append("  double nu_th = std::fabs(angle_diff_deg(pose.theta, ekf.theta));")
    lines.append("  bool cooldown_active = now_ms < cooldown_until_ms;")
    lines.append("  bool intervene_delta = false;")
    lines.append("  if (MCL_EKF_INTERVENE_TRANS_IN > 0.0 && nu_xy >= MCL_EKF_INTERVENE_TRANS_IN) intervene_delta = true;")
    lines.append("  if (MCL_EKF_INTERVENE_THETA_DEG > 0.0 && nu_th >= MCL_EKF_INTERVENE_THETA_DEG) intervene_delta = true;")
    lines.append("  bool intervene = recovered_now || intervene_delta || ekf_covariance_needs_intervention(ekf);")
    lines.append("  bool track_ok = MCL_EKF_TRACK_CORRECTION_ENABLED && conf >= MCL_EKF_TRACK_MIN_CONF;")
    lines.append("  if (track_ok && MCL_EKF_TRACK_MAX_TRANS_IN > 0.0 && nu_xy > MCL_EKF_TRACK_MAX_TRANS_IN) track_ok = false;")
    lines.append("  if (track_ok && MCL_EKF_TRACK_MAX_THETA_DEG > 0.0 && nu_th > MCL_EKF_TRACK_MAX_THETA_DEG) track_ok = false;")
    lines.append("  bool intervene_ok = intervene && !cooldown_active && conf >= MCL_EKF_INTERVENE_MIN_CONF;")
    lines.append("  bool use_reset = recovered_now ? MCL_EKF_RECOVER_USE_RESET : MCL_EKF_INTERVENE_USE_RESET;")
    lines.append("  bool hard_jump = false;")
    lines.append("  if (MCL_EKF_INTERVENE_HARD_RESET_TRANS_IN > 0.0 && nu_xy >= MCL_EKF_INTERVENE_HARD_RESET_TRANS_IN) hard_jump = true;")
    lines.append("  if (MCL_EKF_INTERVENE_HARD_RESET_THETA_DEG > 0.0 && nu_th >= MCL_EKF_INTERVENE_HARD_RESET_THETA_DEG) hard_jump = true;")
    lines.append("  int fusion_mode = MCL_EKF_FUSION_MODE;")
    lines.append("  if (fusion_mode < 0 || fusion_mode > 2) fusion_mode = 2;")
    lines.append("  if (fusion_mode == 0) {")
    lines.append("    return ekf_update_mcl(ekf, pose, conf);")
    lines.append("  }")
    lines.append("  if (fusion_mode == 1 || fusion_mode == 2) {")
    lines.append("    if (intervene_ok) {")
    lines.append("      if (use_reset && hard_jump) {")
    lines.append("        ekf_reset(ekf, pose.x, pose.y, pose.theta);")
    lines.append("        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);")
    lines.append("        return true;")
    lines.append("      }")
    lines.append("      if (ekf_update_mcl(ekf, pose, conf)) {")
    lines.append("        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);")
    lines.append("        return true;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (fusion_mode == 2 && track_ok) {")
    lines.append("      return ekf_update_mcl(ekf, pose, conf);")
    lines.append("    }")
    lines.append("    return false;")
    lines.append("  }")
    lines.append("  return false;")
    lines.append("}")
    lines.append("}  // namespace")
    lines.append("")
    lines.append("ProsMCL::ProsMCL(int imu_port, const std::vector<int>& dist_ports)")
    lines.append("  : task_(nullptr), running_(false), task_done_(true), relocalize_requested_(false), pose_provider_(nullptr), pose_provider_user_(nullptr), provider_last_pose_{0.0, 0.0, 0.0}, provider_last_valid_(false), odom_dx_(0.0), odom_dy_(0.0), odom_dth_(0.0), pose_{0.0, 0.0, 0.0}, fused_pose_{0.0, 0.0, 0.0}, mcl_ambiguous_(false), mcl_recover_good_count_(0), ekf_intervene_cooldown_until_ms_(0), last_event_flags_(0) {")
    lines.append("  if (imu_port > 0) imu_.reset(new pros::Imu(imu_port));")
    lines.append("  int max_dist_cfg = std::max(0, MCL_DISTANCE_SENSOR_COUNT);")
    lines.append("  for (int p : dist_ports) {")
    lines.append("    if (static_cast<int>(dists_.size()) >= max_dist_cfg) break;")
    lines.append("    if (p > 0) dists_.emplace_back(p);")
    lines.append("  }")
    lines.append("  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("    dist_hist_count_[i] = 0;")
    lines.append("    dist_hist_idx_[i] = 0;")
    lines.append("    for (int j = 0; j < DIST_MEDIAN_WINDOW; ++j) dist_hist_[i][j] = 0.0;")
    lines.append("  }")
    lines.append("  ekf_.initialized = false;")
    lines.append("  for (int i = 0; i < 3; ++i) {")
    lines.append("    for (int j = 0; j < 3; ++j) {")
    lines.append("      ekf_.P[i][j] = 0.0;")
    lines.append("    }")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::start(unsigned seed, double initial_heading_deg) {")
    lines.append("  if (running_.load()) return;")
    lines.append("  mcl_.seed(seed);")
    lines.append("  if (imu_) {")
    lines.append("    imu_->reset();")
    lines.append("    while (imu_->is_calibrating()) pros::delay(10);")
    lines.append("    imu_->set_heading(initial_heading_deg);")
    lines.append("  }")
    lines.append("  mcl_.initGlobal();")
    lines.append("  {")
    lines.append("    ScopedMutex lk(mu_);")
    lines.append("    pose_ = mcl_.estimate();")
    lines.append("    ekf_.initialized = false;")
    lines.append("    fused_pose_ = pose_;")
    lines.append("    mcl_ambiguous_ = false;")
    lines.append("    mcl_recover_good_count_ = 0;")
    lines.append("    ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("  }")
    lines.append("  task_done_.store(false);")
    lines.append("  running_.store(true);")
    lines.append("  task_ = new pros::Task([this] { loop(); });")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg) {")
    lines.append("  start(seed, initial_heading_deg);")
    lines.append("  setPose(start_x_in, start_y_in, start_theta_deg);")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::stop() {")
    lines.append("  running_.store(false);")
    lines.append("  if (task_) {")
    lines.append("    uint32_t t0 = pros::millis();")
    lines.append("    while (!task_done_.load() && (pros::millis() - t0) < 250) {")
    lines.append("      pros::delay(10);")
    lines.append("    }")
    lines.append("    if (!task_done_.load()) {")
    lines.append("      task_->remove();")
    lines.append("    }")
    lines.append("    delete task_;")
    lines.append("    task_ = nullptr;")
    lines.append("  }")
    lines.append("  task_done_.store(true);")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setOdomDelta(double dx_in, double dy_in, double dtheta_deg) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  odom_dx_ += dx_in;")
    lines.append("  odom_dy_ += dy_in;")
    lines.append("  odom_dth_ += dtheta_deg;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setFieldPoseProvider(FieldPoseProvider provider, void* user) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  pose_provider_ = provider;")
    lines.append("  pose_provider_user_ = user;")
    lines.append("  provider_last_valid_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::clearFieldPoseProvider() {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  pose_provider_ = nullptr;")
    lines.append("  pose_provider_user_ = nullptr;")
    lines.append("  provider_last_valid_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setPose(double x_in, double y_in, double theta_deg) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  mcl_.init(x_in, y_in, theta_deg);")
    lines.append("  pose_ = mcl_.estimate();")
    lines.append("  if (MCL_EKF_ENABLED) {")
    lines.append("    ekf_reset(ekf_, pose_.x, pose_.y, pose_.theta);")
    lines.append("    fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("    mcl_ambiguous_ = false;")
    lines.append("    mcl_recover_good_count_ = 0;")
    lines.append("    ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("  } else {")
    lines.append("    fused_pose_ = pose_;")
    lines.append("  }")
    lines.append("  provider_last_pose_ = {x_in, y_in, theta_deg};")
    lines.append("  provider_last_valid_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getPose() const {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  return pose_;")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getFusedPose() const {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  return fused_pose_;")
    lines.append("}")
    lines.append("")
    lines.append("ProsMCL::DebugSnapshot ProsMCL::getDebugSnapshot() const {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  DebugSnapshot s{};")
    lines.append("  s.time_ms = pros::millis();")
    lines.append("  s.odom_pose = provider_last_pose_;")
    lines.append("  s.mcl_pose = pose_;")
    lines.append("  s.fused_pose = fused_pose_;")
    lines.append("  s.mcl_confidence = mcl_.confidence();")
    lines.append("  s.mcl_neff = mcl_.neff();")
    lines.append("  s.ekf_pxx = ekf_.P[0][0];")
    lines.append("  s.ekf_pyy = ekf_.P[1][1];")
    lines.append("  s.ekf_pxy = ekf_.P[0][1];")
    lines.append("  s.ekf_ptt = ekf_.P[2][2];")
    lines.append("  s.event_flags = last_event_flags_;")
    lines.append("  last_event_flags_ = 0;")
    lines.append("  mcl_.getLastDistanceDebug(s.dist_meas_mm, s.dist_exp_mm, MCL_DISTANCE_SENSOR_COUNT_SAFE, &s.dist_used_mask);")
    lines.append("  return s;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::updateVision(double x_in, double y_in, double theta_deg, double confidence) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  mcl_.updateVision(x_in, y_in, theta_deg, confidence);")
    lines.append("  mcl_.normalize();")
    lines.append("  mcl_.resample();")
    lines.append("  pose_ = mcl_.estimate();")
    lines.append("  if (MCL_EKF_ENABLED) {")
    lines.append("    double conf = mcl_.confidence();")
    lines.append("    bool applied = ekf_fusion_update(ekf_, pose_, conf, pros::millis(), mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_);")
    lines.append("    if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;")
    lines.append("    if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("    if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("    else fused_pose_ = pose_;")
    lines.append("  } else {")
    lines.append("    fused_pose_ = pose_;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setSegmentBand(const MCLPose* pts, int n, double radius_in) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  mcl_.setSegmentBand(pts, n, radius_in);")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::clearSegmentBand() {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  mcl_.clearSegmentBand();")
    lines.append("}")
    lines.append("")
    lines.append("bool ProsMCL::applyOdomCorrection(MCLPose& odom_pose, double* out_alpha) const {")
    lines.append("  if (!MCL_CORR_ENABLED) return false;")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  double conf = mcl_.confidence();")
    lines.append("  if (conf < MCL_CORR_MIN_CONF) return false;")
    lines.append("  if (MCL_EKF_ENABLED && mcl_ambiguous_) return false;")
    lines.append("  const MCLPose& target = (MCL_EKF_ENABLED ? fused_pose_ : pose_);")
    lines.append("  double dx = target.x - odom_pose.x;")
    lines.append("  double dy = target.y - odom_pose.y;")
    lines.append("  double dist = std::sqrt(dx * dx + dy * dy);")
    lines.append("  double dth = angle_diff_deg(target.theta, odom_pose.theta);")
    lines.append("  if (MCL_CORR_MAX_TRANS_JUMP_IN > 0.0 && dist > MCL_CORR_MAX_TRANS_JUMP_IN) return false;")
    lines.append("  if (MCL_CORR_MAX_THETA_JUMP_DEG > 0.0 && std::fabs(dth) > MCL_CORR_MAX_THETA_JUMP_DEG) return false;")
    lines.append("  double alpha = MCL_CORR_ALPHA_MIN;")
    lines.append("  if (MCL_CORR_ALPHA_MAX > MCL_CORR_ALPHA_MIN && conf > MCL_CORR_MIN_CONF) {")
    lines.append("    double t = (conf - MCL_CORR_MIN_CONF) / std::max(1e-6, (1.0 - MCL_CORR_MIN_CONF));")
    lines.append("    t = std::max(0.0, std::min(1.0, t));")
    lines.append("    alpha = MCL_CORR_ALPHA_MIN + (MCL_CORR_ALPHA_MAX - MCL_CORR_ALPHA_MIN) * t;")
    lines.append("  }")
    lines.append("  double dlx = 0.0, dly = 0.0;")
    lines.append("  rotate_world_to_local(dx, dy, odom_pose.theta, dlx, dly);")
    lines.append("  dlx *= alpha;")
    lines.append("  dly *= alpha;")
    lines.append("  double dwx = 0.0, dwy = 0.0;")
    lines.append("  rotate_local_to_world(dlx, dly, odom_pose.theta, dwx, dwy);")
    lines.append("  odom_pose.x += dwx;")
    lines.append("  odom_pose.y += dwy;")
    lines.append("  odom_pose.theta = wrap_deg(odom_pose.theta + dth * alpha);")
    lines.append("  last_event_flags_ |= MCL_EVENT_ODOM_CORR_APPLIED;")
    lines.append("  if (out_alpha) *out_alpha = alpha;")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::requestRelocalize() {")
    lines.append("  relocalize_requested_.store(true);")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::loop() {")
    lines.append("  task_done_.store(false);")
    lines.append("  uint32_t last_motion = pros::millis();")
    lines.append("  uint32_t last_sensor = last_motion;")
    lines.append("  while (running_.load()) {")
    lines.append("    const uint32_t now = pros::millis();")
    lines.append("    bool do_motion = (MCL_MOTION_UPDATE_MS <= 0 || now - last_motion >= static_cast<uint32_t>(MCL_MOTION_UPDATE_MS));")
    lines.append("    bool do_sensor = (MCL_SENSOR_UPDATE_MS <= 0 || now - last_sensor >= static_cast<uint32_t>(MCL_SENSOR_UPDATE_MS));")
    lines.append("    if (!do_motion && !do_sensor) {")
    lines.append("      pros::delay(2);")
    lines.append("      continue;")
    lines.append("    }")
    lines.append("    double mm_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    int mm_count = 0;")
    lines.append("    FieldPoseProvider provider_cb = nullptr;")
    lines.append("    void* provider_user = nullptr;")
    lines.append("    MCLPose provider_prev{0.0, 0.0, 0.0};")
    lines.append("    bool provider_prev_valid = false;")
    lines.append("    MCLPose provider_sample{0.0, 0.0, 0.0};")
    lines.append("    bool provider_sample_ok = false;")
    lines.append("    bool have_imu_heading = false;")
    lines.append("    double imu_heading = 0.0;")
    lines.append("    if (do_sensor && MCL_USE_DISTANCE && !dists_.empty()) {")
    lines.append("      int n = std::min(static_cast<int>(dists_.size()), MCL_DISTANCE_SENSOR_COUNT_SAFE);")
    lines.append("      mm_count = 0;")
    lines.append("      for (int i = 0; i < n; ++i) {")
    lines.append("        auto& d = dists_[i];")
    lines.append("        const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[i];")
    lines.append("        double meas = static_cast<double>(d.get());")
    lines.append("        bool valid = true;")
    lines.append("        if (meas >= 9000.0) valid = false;")
    lines.append("        double meas_bias = meas - cfg.bias_mm;")
    lines.append("        double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("        double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("        if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("        if (valid && s_max > 0.0 && meas_bias > s_max) meas = s_max + cfg.bias_mm;")
    lines.append("        if (valid && s_min > 0.0 && meas_bias < s_min) valid = false;")
    lines.append("        double conf_min = (cfg.min_confidence > 0.0) ? cfg.min_confidence : MCL_DIST_CONFIDENCE_MIN;")
    lines.append("        if (valid && conf_min > 0.0) {")
    lines.append("          double conf_thresh = conf_min;")
    lines.append("          if (conf_thresh <= 1.0) conf_thresh *= 63.0;")
    lines.append("          if (conf_thresh < 0.0) conf_thresh = 0.0;")
    lines.append("          if (conf_thresh > 63.0) conf_thresh = 63.0;")
    lines.append("          int conf = d.get_confidence();")
    lines.append("          if (static_cast<double>(conf) < conf_thresh) valid = false;")
    lines.append("        }")
    lines.append("        double obj_min = (cfg.min_object_size > 0.0) ? cfg.min_object_size : MCL_DIST_OBJECT_SIZE_MIN;")
    lines.append("        double obj_max = (cfg.max_object_size > 0.0) ? cfg.max_object_size : MCL_DIST_OBJECT_SIZE_MAX;")
    lines.append("        if (valid && (obj_min > 0.0 || obj_max > 0.0)) {")
    lines.append("          int obj = d.get_object_size();")
    lines.append("          if (obj < 0 || (obj_min > 0.0 && obj < obj_min) || (obj_max > 0.0 && obj > obj_max)) valid = false;")
    lines.append("        }")
    lines.append("        if (!valid) {")
    lines.append("          mm_buf[mm_count++] = -1.0;")
    lines.append("          continue;")
    lines.append("        }")
    lines.append("        if (MCL_DIST_MEDIAN_WINDOW > 1) {")
    lines.append("          int idx = dist_hist_idx_[i];")
    lines.append("          dist_hist_[i][idx] = meas;")
    lines.append("          idx = (idx + 1) % DIST_MEDIAN_WINDOW;")
    lines.append("          dist_hist_idx_[i] = idx;")
    lines.append("          if (dist_hist_count_[i] < DIST_MEDIAN_WINDOW) dist_hist_count_[i]++;")
    lines.append("          double temp[DIST_MEDIAN_WINDOW];")
    lines.append("          int cnt = dist_hist_count_[i];")
    lines.append("          for (int j = 0; j < cnt; ++j) temp[j] = dist_hist_[i][j];")
    lines.append("          std::sort(temp, temp + cnt);")
    lines.append("          double med = temp[cnt / 2];")
    lines.append("          mm_buf[mm_count++] = med;")
    lines.append("        } else {")
    lines.append("          mm_buf[mm_count++] = meas;")
    lines.append("        }")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (do_motion) {")
    lines.append("      {")
    lines.append("        ScopedMutex lk(mu_);")
    lines.append("        provider_cb = pose_provider_;")
    lines.append("        provider_user = pose_provider_user_;")
    lines.append("        provider_prev = provider_last_pose_;")
    lines.append("        provider_prev_valid = provider_last_valid_;")
    lines.append("      }")
    lines.append("      if (provider_cb) {")
    lines.append("        provider_sample = provider_prev;")
    lines.append("        provider_sample_ok = provider_cb(&provider_sample, provider_user);")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (do_sensor && imu_) {")
    lines.append("      imu_heading = static_cast<double>(imu_->get_heading());")
    lines.append("      have_imu_heading = true;")
    lines.append("    }")
    lines.append("    {")
    lines.append("      ScopedMutex lk(mu_);")
    lines.append("      if (relocalize_requested_.load()) {")
    lines.append("        mcl_.initGlobal();")
    lines.append("        pose_ = mcl_.estimate();")
    lines.append("        ekf_.initialized = false;")
    lines.append("        fused_pose_ = pose_;")
    lines.append("        odom_dx_ = 0.0;")
    lines.append("        odom_dy_ = 0.0;")
    lines.append("        odom_dth_ = 0.0;")
    lines.append("        provider_last_valid_ = false;")
    lines.append("        mcl_ambiguous_ = false;")
    lines.append("        mcl_recover_good_count_ = 0;")
    lines.append("        ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("        relocalize_requested_.store(false);")
    lines.append("      }")
    lines.append("      if (do_motion) {")
    lines.append("        if (provider_cb == pose_provider_ && provider_sample_ok) {")
    lines.append("          if (provider_prev_valid) {")
    lines.append("            double dX = provider_sample.x - provider_prev.x;")
    lines.append("            double dY = provider_sample.y - provider_prev.y;")
    lines.append("            double dx_auto = 0.0, dy_auto = 0.0;")
    lines.append("            rotate_world_to_local(dX, dY, provider_prev.theta, dx_auto, dy_auto);")
    lines.append("            odom_dx_ += dx_auto;")
    lines.append("            odom_dy_ += dy_auto;")
    lines.append("            odom_dth_ += angle_diff_deg(provider_sample.theta, provider_prev.theta);")
    lines.append("          }")
    lines.append("          provider_last_pose_ = provider_sample;")
    lines.append("          provider_last_valid_ = true;")
    lines.append("        }")
    lines.append("        double dx = odom_dx_;")
    lines.append("        double dy = odom_dy_;")
    lines.append("        double dth = odom_dth_;")
    lines.append("        odom_dx_ = 0.0;")
    lines.append("        odom_dy_ = 0.0;")
    lines.append("        odom_dth_ = 0.0;")
    lines.append("        mcl_.predict(dx, dy, dth);")
    lines.append("        pose_ = mcl_.estimate();")
    lines.append("        if (MCL_EKF_ENABLED) {")
    lines.append("          if (ekf_.initialized) {")
    lines.append("            ekf_predict(ekf_, dx, dy, dth);")
    lines.append("            fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("          } else {")
    lines.append("            fused_pose_ = pose_;")
    lines.append("          }")
    lines.append("        } else {")
    lines.append("          fused_pose_ = pose_;")
    lines.append("        }")
    lines.append("        last_motion = now;")
    lines.append("      }")
    lines.append("      if (do_sensor) {")
    lines.append("        if (MCL_USE_DISTANCE && mm_count > 0) {")
    lines.append("          mcl_.updateDistance(mm_buf, mm_count);")
    lines.append("        }")
    lines.append("        if (have_imu_heading) {")
    lines.append("          if (MCL_USE_IMU) mcl_.updateIMU(imu_heading);")
    lines.append("          if (MCL_EKF_ENABLED && MCL_EKF_USE_IMU) {")
    lines.append("            ekf_update_imu(ekf_, imu_heading);")
    lines.append("          }")
    lines.append("        }")
    lines.append("        mcl_.normalize();")
    lines.append("        mcl_.resample();")
    lines.append("        pose_ = mcl_.estimate();")
    lines.append("        if (MCL_EKF_ENABLED) {")
    lines.append("          double conf = mcl_.confidence();")
    lines.append("          bool applied = ekf_fusion_update(ekf_, pose_, conf, now, mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_);")
    lines.append("          if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;")
    lines.append("          if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("          if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("          else fused_pose_ = pose_;")
    lines.append("        } else {")
    lines.append("          fused_pose_ = pose_;")
    lines.append("        }")
    lines.append("        last_sensor = now;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    pros::delay(1);")
    lines.append("  }")
    lines.append("  task_done_.store(true);")
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def build_mcl_localizer_h(cfg: dict) -> str:
    """Build mcl localizer h."""
    return "\n".join([
        "#pragma once",
        "#include \"mcl_config.hpp\"",
        "#include <cstdint>",
        "#include <vector>",
        "",
        "// Field-centric coordinates: +x forward, +y left; heading 0=left, 90=forward (clockwise).",
        "struct MCLPose {",
        "  double x;",
        "  double y;",
        "  double theta;",
        "};",
        "",
        "class MCLLocalizer {",
        " public:",
        "  MCLLocalizer();",
        "  void seed(unsigned int seed);",
        "  void init(double start_x, double start_y, double start_theta);",
        "  void initGlobal();",
        "  void predict(double dx_in, double dy_in, double dtheta_deg);",
        "  void updateDistance(const double* dist_mm, int count);",
        "  void updateIMU(double heading_deg);",
        "  void updateVision(double x_in, double y_in, double theta_deg, double confidence);",
        "  void setSegmentBand(const MCLPose* pts, int n, double radius_in);",
        "  void clearSegmentBand();",
        "  void normalize();",
        "  void resample();",
        "  MCLPose estimate();",
        "  // 0 = uniform weights, 1 = highly peaked weights.",
        "  double confidence() const { return confidence_; }",
        "  double neff() const { return effectiveN(); }",
        "  void getLastDistanceDebug(double* measured_mm, double* expected_mm, int capacity, uint32_t* used_mask) const;",
        "",
        " private:",
        "  struct Particle {",
        "    double x;",
        "    double y;",
        "    double theta;",
        "    double w;",
        "  };",
        "  Particle particles_[MCL_PARTICLE_CAPACITY];",
        "  Particle resample_buf_[MCL_PARTICLE_CAPACITY];",
        "  double tmp_weights_[MCL_PARTICLE_CAPACITY];",
        "  double cdf_buf_[MCL_PARTICLE_CAPACITY];",
        "  uint64_t bin_buf_[MCL_PARTICLE_CAPACITY];",
        "  char skip_sensor_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int count_;",
        "  double w_slow_;",
        "  double w_fast_;",
        "  double confidence_;",
        "  MCLPose estimate_;",
        "  bool estimate_valid_;",
        "  bool have_estimate_ever_;",
        "  double last_dist_measured_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  double last_dist_expected_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  uint32_t last_dist_used_mask_;",
        "  std::vector<MCLPose> segment_band_pts_;",
        "  double segment_band_radius_;",
        "  bool segment_band_active_;",
        "  double regionWeight(double x, double y, double heading_deg) const;",
        "  double effectiveN() const;",
        "};",
        "",
    ])


def build_mcl_localizer_cpp(cfg: dict) -> str:
    """Build mcl localizer cpp."""
    cfg = _unwrap_values(cfg or {})
    field_w = WINDOW_WIDTH / PPI
    field_h = WINDOW_HEIGHT / PPI
    segs = _segments_in_field_in(cfg, field_w, field_h)
    perim_segs = _perimeter_segments_in_field_in(cfg, field_w, field_h)
    obj_segs = _object_segments_in_field_in(cfg, field_w, field_h)
    polys = _object_polys_in_field_in(cfg, field_w, field_h)
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors.get("distance", {}) if isinstance(sensors, dict) else {}
    lf_cfg = dist_cfg.get("likelihood_field", {}) if isinstance(dist_cfg, dict) else {}
    use_lf = True
    try:
        if isinstance(dist_cfg, dict) and dist_cfg.get("enabled", 1) in (0, False):
            use_lf = False
        if isinstance(lf_cfg, dict) and lf_cfg.get("enabled", 1) in (0, False):
            use_lf = False
        model = str(dist_cfg.get("model", "likelihood_field")).strip().lower() if isinstance(dist_cfg, dict) else "likelihood_field"
        if model != "likelihood_field":
            use_lf = False
    except Exception:
        pass

    if use_lf:
        try:
            res_in = float(lf_cfg.get("resolution_in", 2.0))
        except Exception:
            res_in = 2.0
        dist_field = _build_distance_field(segs, field_w, field_h, res_in)
        dist_field_perim = _build_distance_field(perim_segs, field_w, field_h, res_in)
        dist_field_obj = _build_distance_field(obj_segs, field_w, field_h, res_in)
    else:
        dist_field = None
        dist_field_perim = None
        dist_field_obj = None
    lines: List[str] = []
    lines.append("#include \"mcl_localizer.h\"")
    lines.append("#include \"mcl_config.hpp\"")
    lines.append("#include <algorithm>")
    lines.append("#include <cmath>")
    lines.append("#include <cstdint>")
    lines.append("#include <cstdlib>")
    lines.append("#include <vector>")
    lines.append("")
    lines.append("namespace {")
    lines.append("constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;")
    lines.append("constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;")
    lines.append("")
    lines.append("struct Segment { double x0; double y0; double x1; double y1; };")
    def _rename_segments(seg_lines, name):
        """Handle rename segments."""
        out = []
        for ln in seg_lines:
            ln = ln.replace("MAP_SEGMENTS", name)
            ln = ln.replace("MAP_SEGMENT_COUNT", f"{name}_COUNT")
            out.append(ln)
        return out
    lines.extend(_emit_segments(segs))
    lines.extend(_rename_segments(_emit_segments(perim_segs), "PERIM_SEGMENTS"))
    lines.extend(_rename_segments(_emit_segments(obj_segs), "OBJECT_SEGMENTS"))
    lines.append("struct PolyIndex { int offset; int count; };")
    lines.extend(_emit_polys(polys))
    def _emit_dist_field(df, name):
        """Handle emit dist field."""
        if df:
            lines.append(f"static const int {name}_W = {df['nx']};")
            lines.append(f"static const int {name}_H = {df['ny']};")
            lines.append(f"static const double {name}_RES_IN = {_fmt(df['res'])};")
            lines.append(f"static const double {name}_ORIGIN_X = {_fmt(df['x0'])};")
            lines.append(f"static const double {name}_ORIGIN_Y = {_fmt(df['y0'])};")
            lines.append("// Grid data declared in mcl_map_data.h; defined in mcl_map_data.cpp")
            lines.append(f"extern const double {name}[];")
        else:
            lines.append(f"static const int {name}_W = 1;")
            lines.append(f"static const int {name}_H = 1;")
            lines.append(f"static const double {name}_RES_IN = 1.0;")
            lines.append(f"static const double {name}_ORIGIN_X = 0.0;")
            lines.append(f"static const double {name}_ORIGIN_Y = 0.0;")
            lines.append(f"extern const double {name}[];")

    _emit_dist_field(dist_field, "MAP_DIST_FIELD")
    _emit_dist_field(dist_field_perim, "MAP_DIST_FIELD_PERIM")
    _emit_dist_field(dist_field_obj, "MAP_DIST_FIELD_OBJ")
    lines.append("")
    lines.append("static double rand_uniform() {")
    lines.append("  return static_cast<double>(std::rand()) / (static_cast<double>(RAND_MAX) + 1.0);")
    lines.append("}")
    lines.append("")
    lines.append("static double rand_gaussian(double mean, double stddev) {")
    lines.append("  double u1 = std::max(1e-12, rand_uniform());")
    lines.append("  double u2 = rand_uniform();")
    lines.append("  double z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);")
    lines.append("  return mean + z0 * stddev;")
    lines.append("}")
    lines.append("")
    lines.append("static double wrap_deg(double deg) {")
    lines.append("  while (deg >= 360.0) deg -= 360.0;")
    lines.append("  while (deg < 0.0) deg += 360.0;")
    lines.append("  return deg;")
    lines.append("}")
    lines.append("")
    lines.append("static double angle_diff_deg(double a, double b) {")
    lines.append("  double d = std::fmod(a - b + 180.0, 360.0);")
    lines.append("  if (d < 0.0) d += 360.0;")
    lines.append("  return d - 180.0;")
    lines.append("}")
    lines.append("")
    lines.append("static double gaussian(double x, double mu, double sigma) {")
    lines.append("  if (sigma <= 1e-9) return (std::fabs(x - mu) <= 1e-9) ? 1.0 : 0.0;")
    lines.append("  double z = (x - mu) / sigma;")
    lines.append("  return std::exp(-0.5 * z * z);")
    lines.append("}")
    lines.append("")
    lines.append("static void heading_unit(double heading_deg, double& dx, double& dy) {")
    lines.append("  double th = heading_deg * DEG_TO_RAD;")
    lines.append("  dx = std::sin(th);")
    lines.append("  dy = std::cos(th);")
    lines.append("}")
    lines.append("")
    lines.append("static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {")
    lines.append("  double th = heading_deg * DEG_TO_RAD;")
    lines.append("  double s = std::sin(th);")
    lines.append("  double c = std::cos(th);")
    lines.append("  wx = lx * s - ly * c;")
    lines.append("  wy = lx * c + ly * s;")
    lines.append("}")
    lines.append("")
    lines.append("static double dist_point_seg_sq(double px, double py, double x0, double y0, double x1, double y1) {")
    lines.append("  double dx = x1 - x0;")
    lines.append("  double dy = y1 - y0;")
    lines.append("  if (std::fabs(dx) < 1e-9 && std::fabs(dy) < 1e-9) {")
    lines.append("    double ex = px - x0;")
    lines.append("    double ey = py - y0;")
    lines.append("    return ex * ex + ey * ey;")
    lines.append("  }")
    lines.append("  double t = ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy);")
    lines.append("  if (t < 0.0) t = 0.0;")
    lines.append("  if (t > 1.0) t = 1.0;")
    lines.append("  double cx = x0 + t * dx;")
    lines.append("  double cy = y0 + t * dy;")
    lines.append("  double ex = px - cx;")
    lines.append("  double ey = py - cy;")
    lines.append("  return ex * ex + ey * ey;")
    lines.append("}")
    lines.append("")
    lines.append("static double distance_field_lookup(const double* field, int w, int h, double res,")
    lines.append("                                   double ox, double oy, double x, double y) {")
    lines.append("  if (!field || w <= 1 || h <= 1 || res <= 1e-9) return -1.0;")
    lines.append("  double fx = (x - ox) / res;")
    lines.append("  double fy = (y - oy) / res;")
    lines.append("  int ix = static_cast<int>(std::floor(fx));")
    lines.append("  int iy = static_cast<int>(std::floor(fy));")
    lines.append("  if (ix < 0) ix = 0;")
    lines.append("  if (iy < 0) iy = 0;")
    lines.append("  if (ix > w - 2) ix = w - 2;")
    lines.append("  if (iy > h - 2) iy = h - 2;")
    lines.append("  double tx = fx - ix;")
    lines.append("  double ty = fy - iy;")
    lines.append("  int idx = iy * w + ix;")
    lines.append("  double d00 = field[idx];")
    lines.append("  double d10 = field[idx + 1];")
    lines.append("  double d01 = field[idx + w];")
    lines.append("  double d11 = field[idx + w + 1];")
    lines.append("  double dx0 = d00 + (d10 - d00) * tx;")
    lines.append("  double dx1 = d01 + (d11 - d01) * tx;")
    lines.append("  return dx0 + (dx1 - dx0) * ty;")
    lines.append("}")
    lines.append("")
    lines.append("static bool sensor_allows_object(int sensor_idx, int obj_idx) {")
    lines.append("  if (obj_idx < 0 || obj_idx >= OBJECT_POLY_COUNT) return false;")
    lines.append("  if (sensor_idx < 0 || sensor_idx >= MCL_DISTANCE_SENSOR_COUNT) return true;")
    lines.append("  if (obj_idx >= 64) return true;")
    lines.append("  uint64_t mask = MCL_DISTANCE_SENSORS[sensor_idx].object_mask;")
    lines.append("  return ((mask >> obj_idx) & 1ull) != 0ull;")
    lines.append("}")
    lines.append("")
    lines.append("static bool sensor_has_full_object_mask(int sensor_idx) {")
    lines.append("  if (OBJECT_POLY_COUNT <= 0) return true;")
    lines.append("  if (sensor_idx < 0 || sensor_idx >= MCL_DISTANCE_SENSOR_COUNT) return true;")
    lines.append("  if (OBJECT_POLY_COUNT >= 64) return true;")
    lines.append("  uint64_t need = (1ull << OBJECT_POLY_COUNT) - 1ull;")
    lines.append("  uint64_t have = MCL_DISTANCE_SENSORS[sensor_idx].object_mask;")
    lines.append("  return (have & need) == need;")
    lines.append("}")
    lines.append("")
    lines.append("static double distance_to_objects_mask(double x, double y, int sensor_idx) {")
    lines.append("  if (OBJECT_POLY_COUNT <= 0) return -1.0;")
    lines.append("  bool have = false;")
    lines.append("  double best = 1e18;")
    lines.append("  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {")
    lines.append("    if (!sensor_allows_object(sensor_idx, i)) continue;")
    lines.append("    const PolyIndex& p = OBJECT_POLYS[i];")
    lines.append("    if (p.count < 2) continue;")
    lines.append("    have = true;")
    lines.append("    for (int j = 0; j < p.count; ++j) {")
    lines.append("      int k = (j + 1) % p.count;")
    lines.append("      double x0 = OBJECT_POINTS[p.offset + j][0];")
    lines.append("      double y0 = OBJECT_POINTS[p.offset + j][1];")
    lines.append("      double x1 = OBJECT_POINTS[p.offset + k][0];")
    lines.append("      double y1 = OBJECT_POINTS[p.offset + k][1];")
    lines.append("      double d2 = dist_point_seg_sq(x, y, x0, y0, x1, y1);")
    lines.append("      if (d2 < best) best = d2;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  if (!have || best >= 1e17) return -1.0;")
    lines.append("  return std::sqrt(best);")
    lines.append("}")
    lines.append("")
    lines.append("static double distance_to_map_mode(double x, double y, int mode, int sensor_idx) {")
    lines.append("  bool full_object_mask = sensor_has_full_object_mask(sensor_idx);")
    lines.append("  if (mode == MCL_MAP_MODE_PERIMETER) {")
    lines.append("    double d = distance_field_lookup(MAP_DIST_FIELD_PERIM, MAP_DIST_FIELD_PERIM_W, MAP_DIST_FIELD_PERIM_H,")
    lines.append("                                     MAP_DIST_FIELD_PERIM_RES_IN, MAP_DIST_FIELD_PERIM_ORIGIN_X, MAP_DIST_FIELD_PERIM_ORIGIN_Y, x, y);")
    lines.append("    if (d >= 0.0) return d;")
    lines.append("    const Segment* segs = PERIM_SEGMENTS;")
    lines.append("    int seg_count = PERIM_SEGMENTS_COUNT;")
    lines.append("    if (seg_count <= 0) return 0.0;")
    lines.append("    double best = 1e18;")
    lines.append("    for (int i = 0; i < seg_count; ++i) {")
    lines.append("      const Segment& s = segs[i];")
    lines.append("      double d2 = dist_point_seg_sq(x, y, s.x0, s.y0, s.x1, s.y1);")
    lines.append("      if (d2 < best) best = d2;")
    lines.append("    }")
    lines.append("    return std::sqrt(best);")
    lines.append("  } else if (mode == MCL_MAP_MODE_OBJECTS) {")
    lines.append("    if (full_object_mask) {")
    lines.append("      double d = distance_field_lookup(MAP_DIST_FIELD_OBJ, MAP_DIST_FIELD_OBJ_W, MAP_DIST_FIELD_OBJ_H,")
    lines.append("                                       MAP_DIST_FIELD_OBJ_RES_IN, MAP_DIST_FIELD_OBJ_ORIGIN_X, MAP_DIST_FIELD_OBJ_ORIGIN_Y, x, y);")
    lines.append("      if (d >= 0.0) return d;")
    lines.append("    }")
    lines.append("    double od = distance_to_objects_mask(x, y, sensor_idx);")
    lines.append("    return (od >= 0.0) ? od : 0.0;")
    lines.append("  }")
    lines.append("  if (full_object_mask) {")
    lines.append("    double d = distance_field_lookup(MAP_DIST_FIELD, MAP_DIST_FIELD_W, MAP_DIST_FIELD_H,")
    lines.append("                                     MAP_DIST_FIELD_RES_IN, MAP_DIST_FIELD_ORIGIN_X, MAP_DIST_FIELD_ORIGIN_Y, x, y);")
    lines.append("    if (d >= 0.0) return d;")
    lines.append("  }")
    lines.append("  double best = 1e18;")
    lines.append("  bool have = false;")
    lines.append("  if (PERIM_SEGMENTS_COUNT > 0) {")
    lines.append("    have = true;")
    lines.append("    for (int i = 0; i < PERIM_SEGMENTS_COUNT; ++i) {")
    lines.append("      const Segment& s = PERIM_SEGMENTS[i];")
    lines.append("      double d2 = dist_point_seg_sq(x, y, s.x0, s.y0, s.x1, s.y1);")
    lines.append("      if (d2 < best) best = d2;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double od = distance_to_objects_mask(x, y, sensor_idx);")
    lines.append("  if (od >= 0.0) {")
    lines.append("    have = true;")
    lines.append("    double od2 = od * od;")
    lines.append("    if (od2 < best) best = od2;")
    lines.append("  }")
    lines.append("  if (!have || best >= 1e17) return 0.0;")
    lines.append("  return std::sqrt(best);")
    lines.append("}")
    lines.append("")
    lines.append("static bool ray_segment_intersect(double ox, double oy, double dx, double dy,")
    lines.append("                                  double x0, double y0, double x1, double y1, double& t_out) {")
    lines.append("  double rx = dx, ry = dy;")
    lines.append("  double sx = x1 - x0, sy = y1 - y0;")
    lines.append("  double rxs = rx * sy - ry * sx;")
    lines.append("  if (std::fabs(rxs) < 1e-9) return false;")
    lines.append("  double qpx = x0 - ox, qpy = y0 - oy;")
    lines.append("  double t = (qpx * sy - qpy * sx) / rxs;")
    lines.append("  double u = (qpx * ry - qpy * rx) / rxs;")
    lines.append("  if (t >= 0.0 && u >= 0.0 && u <= 1.0) {")
    lines.append("    t_out = t;")
    lines.append("    return true;")
    lines.append("  }")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static double raycast_distance_segments(const Segment* segs, int count, double ox, double oy,")
    lines.append("                                        double dx, double dy, double max_dist_in) {")
    lines.append("  double best = -1.0;")
    lines.append("  for (int i = 0; i < count; ++i) {")
    lines.append("    double t = 0.0;")
    lines.append("    if (!ray_segment_intersect(ox, oy, dx, dy, segs[i].x0, segs[i].y0, segs[i].x1, segs[i].y1, t)) {")
    lines.append("      continue;")
    lines.append("    }")
    lines.append("    if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;")
    lines.append("  }")
    lines.append("  return best;")
    lines.append("}")
    lines.append("")
    lines.append("static double raycast_distance_objects_mask(double ox, double oy, double dx, double dy, double max_dist_in, int sensor_idx) {")
    lines.append("  if (OBJECT_POLY_COUNT <= 0) return -1.0;")
    lines.append("  double best = -1.0;")
    lines.append("  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {")
    lines.append("    if (!sensor_allows_object(sensor_idx, i)) continue;")
    lines.append("    const PolyIndex& p = OBJECT_POLYS[i];")
    lines.append("    if (p.count < 2) continue;")
    lines.append("    for (int j = 0; j < p.count; ++j) {")
    lines.append("      int k = (j + 1) % p.count;")
    lines.append("      double x0 = OBJECT_POINTS[p.offset + j][0];")
    lines.append("      double y0 = OBJECT_POINTS[p.offset + j][1];")
    lines.append("      double x1 = OBJECT_POINTS[p.offset + k][0];")
    lines.append("      double y1 = OBJECT_POINTS[p.offset + k][1];")
    lines.append("      double t = 0.0;")
    lines.append("      if (!ray_segment_intersect(ox, oy, dx, dy, x0, y0, x1, y1, t)) continue;")
    lines.append("      if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  return best;")
    lines.append("}")
    lines.append("")
    lines.append("static double raycast_distance_mode(double ox, double oy, double heading_deg, double max_dist_in, int mode, int sensor_idx) {")
    lines.append("  double dx = 0.0, dy = 0.0;")
    lines.append("  heading_unit(heading_deg, dx, dy);")
    lines.append("  bool full_object_mask = sensor_has_full_object_mask(sensor_idx);")
    lines.append("  double best = -1.0;")
    lines.append("  if (mode == MCL_MAP_MODE_PERIMETER) {")
    lines.append("    best = raycast_distance_segments(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);")
    lines.append("  } else if (mode == MCL_MAP_MODE_OBJECTS) {")
    lines.append("    if (full_object_mask) {")
    lines.append("      best = raycast_distance_segments(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);")
    lines.append("    } else {")
    lines.append("      best = raycast_distance_objects_mask(ox, oy, dx, dy, max_dist_in, sensor_idx);")
    lines.append("    }")
    lines.append("  } else {")
    lines.append("    best = raycast_distance_segments(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);")
    lines.append("    if (PERIM_SEGMENTS_COUNT > 0) {")
    lines.append("      double p = raycast_distance_segments(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in);")
    lines.append("      if (p >= 0.0 && (best < 0.0 || p < best)) best = p;")
    lines.append("    }")
    lines.append("    double o = full_object_mask ?")
    lines.append("      raycast_distance_segments(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, ox, oy, dx, dy, max_dist_in) :")
    lines.append("      raycast_distance_objects_mask(ox, oy, dx, dy, max_dist_in, sensor_idx);")
    lines.append("    if (o >= 0.0 && (best < 0.0 || o < best)) best = o;")
    lines.append("  }")
    lines.append("  return best;")
    lines.append("}")
    lines.append("")
    lines.append("static bool point_in_poly(double x, double y, int offset, int count) {")
    lines.append("  bool c = false;")
    lines.append("  for (int i = 0, j = count - 1; i < count; j = i++) {")
    lines.append("    double xi = OBJECT_POINTS[offset + i][0];")
    lines.append("    double yi = OBJECT_POINTS[offset + i][1];")
    lines.append("    double xj = OBJECT_POINTS[offset + j][0];")
    lines.append("    double yj = OBJECT_POINTS[offset + j][1];")
    lines.append("    bool intersect = ((yi > y) != (yj > y)) &&")
    lines.append("      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);")
    lines.append("    if (intersect) c = !c;")
    lines.append("  }")
    lines.append("  return c;")
    lines.append("}")
    lines.append("")
    lines.append("static bool in_object(double x, double y) {")
    lines.append("  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {")
    lines.append("    const PolyIndex& p = OBJECT_POLYS[i];")
    lines.append("    if (p.count <= 0) continue;")
    lines.append("    if (point_in_poly(x, y, p.offset, p.count)) return true;")
    lines.append("  }")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static double dist_sq_point_segment(double px, double py, double x0, double y0, double x1, double y1) {")
    lines.append("  double dx = x1 - x0;")
    lines.append("  double dy = y1 - y0;")
    lines.append("  double denom = dx * dx + dy * dy;")
    lines.append("  if (denom <= 1e-12) {")
    lines.append("    double ox = px - x0;")
    lines.append("    double oy = py - y0;")
    lines.append("    return ox * ox + oy * oy;")
    lines.append("  }")
    lines.append("  double t = ((px - x0) * dx + (py - y0) * dy) / denom;")
    lines.append("  if (t < 0.0) t = 0.0;")
    lines.append("  if (t > 1.0) t = 1.0;")
    lines.append("  double cx = x0 + t * dx;")
    lines.append("  double cy = y0 + t * dy;")
    lines.append("  double ox = px - cx;")
    lines.append("  double oy = py - cy;")
    lines.append("  return ox * ox + oy * oy;")
    lines.append("}")
    lines.append("")
    lines.append("static double poly_edge_dist(double x, double y, int offset, int count) {")
    lines.append("  if (count < 2) return 0.0;")
    lines.append("  double best = 1e18;")
    lines.append("  for (int i = 0; i < count; ++i) {")
    lines.append("    int j = (i + 1) % count;")
    lines.append("    double xi = OBJECT_POINTS[offset + i][0];")
    lines.append("    double yi = OBJECT_POINTS[offset + i][1];")
    lines.append("    double xj = OBJECT_POINTS[offset + j][0];")
    lines.append("    double yj = OBJECT_POINTS[offset + j][1];")
    lines.append("    double d2 = dist_sq_point_segment(x, y, xi, yi, xj, yj);")
    lines.append("    if (d2 < best) best = d2;")
    lines.append("  }")
    lines.append("  return (best <= 0.0) ? 0.0 : std::sqrt(best);")
    lines.append("}")
    lines.append("")
    lines.append("static void robot_corners(double x, double y, double heading_deg, double out[4][2]) {")
    lines.append("  double hw = MCL_BOT_WIDTH_IN * 0.5;")
    lines.append("  double hl = MCL_BOT_LENGTH_IN * 0.5;")
    lines.append("  double off_x = 0.0, off_y = 0.0;")
    lines.append("  rotate_local_to_world(MCL_BOT_OFFSET_X_IN, MCL_BOT_OFFSET_Y_IN, heading_deg, off_x, off_y);")
    lines.append("  const double local[4][2] = {{-hl, -hw}, {hl, -hw}, {hl, hw}, {-hl, hw}};")
    lines.append("  for (int i = 0; i < 4; ++i) {")
    lines.append("    double rx = 0.0, ry = 0.0;")
    lines.append("    rotate_local_to_world(local[i][0], local[i][1], heading_deg, rx, ry);")
    lines.append("    out[i][0] = x + off_x + rx;")
    lines.append("    out[i][1] = y + off_y + ry;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("static bool point_in_quad(double x, double y, const double q[4][2]) {")
    lines.append("  bool c = false;")
    lines.append("  for (int i = 0, j = 3; i < 4; j = i++) {")
    lines.append("    double xi = q[i][0], yi = q[i][1];")
    lines.append("    double xj = q[j][0], yj = q[j][1];")
    lines.append("    bool intersect = ((yi > y) != (yj > y)) &&")
    lines.append("      (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi);")
    lines.append("    if (intersect) c = !c;")
    lines.append("  }")
    lines.append("  return c;")
    lines.append("}")
    lines.append("")
    lines.append("static double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {")
    lines.append("  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);")
    lines.append("}")
    lines.append("")
    lines.append("static bool on_segment(double ax, double ay, double bx, double by, double px, double py) {")
    lines.append("  return px >= std::min(ax, bx) - 1e-9 && px <= std::max(ax, bx) + 1e-9 &&")
    lines.append("         py >= std::min(ay, by) - 1e-9 && py <= std::max(ay, by) + 1e-9;")
    lines.append("}")
    lines.append("")
    lines.append("static bool segments_intersect(double ax, double ay, double bx, double by,")
    lines.append("                               double cx, double cy, double dx, double dy) {")
    lines.append("  double o1 = orient2d(ax, ay, bx, by, cx, cy);")
    lines.append("  double o2 = orient2d(ax, ay, bx, by, dx, dy);")
    lines.append("  double o3 = orient2d(cx, cy, dx, dy, ax, ay);")
    lines.append("  double o4 = orient2d(cx, cy, dx, dy, bx, by);")
    lines.append("  if ((o1 > 0.0) != (o2 > 0.0) && (o3 > 0.0) != (o4 > 0.0)) return true;")
    lines.append("  if (std::fabs(o1) <= 1e-9 && on_segment(ax, ay, bx, by, cx, cy)) return true;")
    lines.append("  if (std::fabs(o2) <= 1e-9 && on_segment(ax, ay, bx, by, dx, dy)) return true;")
    lines.append("  if (std::fabs(o3) <= 1e-9 && on_segment(cx, cy, dx, dy, ax, ay)) return true;")
    lines.append("  if (std::fabs(o4) <= 1e-9 && on_segment(cx, cy, dx, dy, bx, by)) return true;")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static double object_clip_overlap(double x, double y, double heading_deg) {")
    lines.append("  if (OBJECT_POLY_COUNT <= 0) return 0.0;")
    lines.append("  double corners[4][2];")
    lines.append("  robot_corners(x, y, heading_deg, corners);")
    lines.append("  double max_pen = 0.0;")
    lines.append("  for (int i = 0; i < OBJECT_POLY_COUNT; ++i) {")
    lines.append("    const PolyIndex& p = OBJECT_POLYS[i];")
    lines.append("    if (p.count <= 0) continue;")
    lines.append("    bool overlap = false;")
    lines.append("    double poly_pen = 0.0;")
    lines.append("    for (int c = 0; c < 4; ++c) {")
    lines.append("      double cx = corners[c][0];")
    lines.append("      double cy = corners[c][1];")
    lines.append("      if (point_in_poly(cx, cy, p.offset, p.count)) {")
    lines.append("        overlap = true;")
    lines.append("        double dist = poly_edge_dist(cx, cy, p.offset, p.count);")
    lines.append("        if (dist > poly_pen) poly_pen = dist;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (!overlap) {")
    lines.append("      for (int v = 0; v < p.count; ++v) {")
    lines.append("        double vx = OBJECT_POINTS[p.offset + v][0];")
    lines.append("        double vy = OBJECT_POINTS[p.offset + v][1];")
    lines.append("        if (point_in_quad(vx, vy, corners)) {")
    lines.append("          overlap = true;")
    lines.append("          break;")
    lines.append("        }")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (!overlap) {")
    lines.append("      for (int e = 0; e < p.count && !overlap; ++e) {")
    lines.append("        int en = (e + 1) % p.count;")
    lines.append("        double ex0 = OBJECT_POINTS[p.offset + e][0];")
    lines.append("        double ey0 = OBJECT_POINTS[p.offset + e][1];")
    lines.append("        double ex1 = OBJECT_POINTS[p.offset + en][0];")
    lines.append("        double ey1 = OBJECT_POINTS[p.offset + en][1];")
    lines.append("        for (int r = 0; r < 4; ++r) {")
    lines.append("          int rn = (r + 1) % 4;")
    lines.append("          double rx0 = corners[r][0];")
    lines.append("          double ry0 = corners[r][1];")
    lines.append("          double rx1 = corners[rn][0];")
    lines.append("          double ry1 = corners[rn][1];")
    lines.append("          if (segments_intersect(ex0, ey0, ex1, ey1, rx0, ry0, rx1, ry1)) {")
    lines.append("            overlap = true;")
    lines.append("            break;")
    lines.append("          }")
    lines.append("        }")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (overlap && poly_pen < 1e-3) poly_pen = 0.01;")
    lines.append("    if (poly_pen > max_pen) max_pen = poly_pen;")
    lines.append("  }")
    lines.append("  return max_pen;")
    lines.append("}")
    lines.append("")
    lines.append("static double object_clip_weight(double x, double y, double heading_deg) {")
    lines.append("  if (MCL_OBJECT_MODE <= 0) return 1.0;")
    lines.append("  double overlap = object_clip_overlap(x, y, heading_deg);")
    lines.append("  if (overlap <= 0.0) return 1.0;")
    lines.append("  if (MCL_OBJECT_MODE >= 2) return 0.0;")
    lines.append("  double free_in = MCL_OBJECT_CLIP_FREE_IN;")
    lines.append("  double max_in = MCL_OBJECT_CLIP_MAX_IN;")
    lines.append("  double sigma_in = MCL_OBJECT_CLIP_SIGMA_IN;")
    lines.append("  if (overlap <= free_in) return 1.0;")
    lines.append("  if (max_in <= free_in || sigma_in <= 1e-6) return 0.0;")
    lines.append("  if (overlap >= max_in) return 0.0;")
    lines.append("  double z = (overlap - free_in) / sigma_in;")
    lines.append("  return std::exp(-0.5 * z * z);")
    lines.append("}")
    lines.append("")
    lines.append("static bool pose_outside_perimeter(double x, double y, double heading_deg) {")
    lines.append("  double corners[4][2];")
    lines.append("  robot_corners(x, y, heading_deg, corners);")
    lines.append("  for (int i = 0; i < 4; ++i) {")
    lines.append("    double cx = corners[i][0];")
    lines.append("    double cy = corners[i][1];")
    lines.append("    if (cx < -MCL_FIELD_HALF_HEIGHT_IN || cx > MCL_FIELD_HALF_HEIGHT_IN ||")
    lines.append("        cy < -MCL_FIELD_HALF_WIDTH_IN || cy > MCL_FIELD_HALF_WIDTH_IN) {")
    lines.append("      return true;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static double polyline_dist_sq(double x, double y, const std::vector<MCLPose>& pts) {")
    lines.append("  if (pts.size() < 2) {")
    lines.append("    if (pts.empty()) return 1e18;")
    lines.append("    double dx = x - pts[0].x;")
    lines.append("    double dy = y - pts[0].y;")
    lines.append("    return dx * dx + dy * dy;")
    lines.append("  }")
    lines.append("  double best = 1e18;")
    lines.append("  for (size_t i = 0; i + 1 < pts.size(); ++i) {")
    lines.append("    double d2 = dist_sq_point_segment(x, y, pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y);")
    lines.append("    if (d2 < best) best = d2;")
    lines.append("  }")
    lines.append("  return best;")
    lines.append("}")
    lines.append("")
    lines.append("struct PoseSample { double x; double y; double theta; };")
    lines.append("")
    lines.append("static PoseSample sample_random_pose() {")
    lines.append("  double x_min = MCL_REGION_X_MIN_IN;")
    lines.append("  double y_min = MCL_REGION_Y_MIN_IN;")
    lines.append("  double x_max = MCL_REGION_X_MAX_IN;")
    lines.append("  double y_max = MCL_REGION_Y_MAX_IN;")
    lines.append("  if (x_max <= x_min) { x_min = -MCL_FIELD_HALF_HEIGHT_IN; x_max = MCL_FIELD_HALF_HEIGHT_IN; }")
    lines.append("  if (y_max <= y_min) { y_min = -MCL_FIELD_HALF_WIDTH_IN; y_max = MCL_FIELD_HALF_WIDTH_IN; }")
    lines.append("  int attempts = std::max(1, MCL_REGION_SAMPLE_ATTEMPTS);")
    lines.append("  for (int i = 0; i < attempts; ++i) {")
    lines.append("    double x = x_min + rand_uniform() * (x_max - x_min);")
    lines.append("    double y = y_min + rand_uniform() * (y_max - y_min);")
    lines.append("    double th = rand_uniform() * 360.0;")
    lines.append("    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;")
    lines.append("    return {x, y, th};")
    lines.append("  }")
    lines.append("  for (int i = 0; i < attempts; ++i) {")
    lines.append("    double x = -MCL_FIELD_HALF_HEIGHT_IN + rand_uniform() * (MCL_FIELD_HALF_HEIGHT_IN * 2.0);")
    lines.append("    double y = -MCL_FIELD_HALF_WIDTH_IN + rand_uniform() * (MCL_FIELD_HALF_WIDTH_IN * 2.0);")
    lines.append("    double th = rand_uniform() * 360.0;")
    lines.append("    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;")
    lines.append("    return {x, y, th};")
    lines.append("  }")
    lines.append("  return {0.0, 0.0, rand_uniform() * 360.0};")
    lines.append("}")
    lines.append("")
    lines.append("")
    lines.append("static double inv_norm_cdf(double p) {")
    lines.append("  if (p <= 0.0) return -1e9;")
    lines.append("  if (p >= 1.0) return 1e9;")
    lines.append("  static const double a[] = {")
    lines.append("    -3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02,")
    lines.append("    1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00")
    lines.append("  };")
    lines.append("  static const double b[] = {")
    lines.append("    -5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02,")
    lines.append("    6.680131188771972e+01, -1.328068155288572e+01")
    lines.append("  };")
    lines.append("  static const double c[] = {")
    lines.append("    -7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00,")
    lines.append("    -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00")
    lines.append("  };")
    lines.append("  static const double d[] = {")
    lines.append("    7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00,")
    lines.append("    3.754408661907416e+00")
    lines.append("  };")
    lines.append("  double plow = 0.02425;")
    lines.append("  double phigh = 1.0 - plow;")
    lines.append("  if (p < plow) {")
    lines.append("    double q = std::sqrt(-2.0 * std::log(p));")
    lines.append("    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /")
    lines.append("      ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);")
    lines.append("  }")
    lines.append("  if (p > phigh) {")
    lines.append("    double q = std::sqrt(-2.0 * std::log(1.0 - p));")
    lines.append("    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /")
    lines.append("      ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);")
    lines.append("  }")
    lines.append("  double q = p - 0.5;")
    lines.append("  double r = q * q;")
    lines.append("  return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /")
    lines.append("    (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);")
    lines.append("}")
    lines.append("")
    lines.append("static int kld_required_particles(int k, double epsilon, double delta) {")
    lines.append("  if (k <= 1) return 1;")
    lines.append("  double p = (delta >= 0.5) ? delta : (1.0 - delta);")
    lines.append("  double z = inv_norm_cdf(p);")
    lines.append("  double km = k - 1.0;")
    lines.append("  double frac = 1.0 - 2.0 / (9.0 * km) + z * std::sqrt(2.0 / (9.0 * km));")
    lines.append("  double n = (km / (2.0 * epsilon)) * (frac * frac * frac);")
    lines.append("  return static_cast<int>(std::ceil(n));")
    lines.append("}")
    lines.append("")
    lines.append("static uint64_t mix_u64(uint64_t x) {")
    lines.append("  x += 0x9e3779b97f4a7c15ULL;")
    lines.append("  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;")
    lines.append("  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;")
    lines.append("  return x ^ (x >> 31);")
    lines.append("}")
    lines.append("")
    lines.append("static uint64_t bin_index(double x, double y, double theta) {")
    lines.append("  double bx = std::max(1e-6, MCL_KLD_BIN_XY_IN);")
    lines.append("  double by = bx;")
    lines.append("  double bt = std::max(1e-6, MCL_KLD_BIN_THETA_DEG);")
    lines.append("  int64_t ix = static_cast<int64_t>(std::floor(x / bx));")
    lines.append("  int64_t iy = static_cast<int64_t>(std::floor(y / by));")
    lines.append("  int64_t it = static_cast<int64_t>(std::floor(wrap_deg(theta) / bt));")
    lines.append("  uint64_t h = 0;")
    lines.append("  h ^= mix_u64(static_cast<uint64_t>(ix));")
    lines.append("  h ^= mix_u64(static_cast<uint64_t>(iy) + 0x9e3779b97f4a7c15ULL);")
    lines.append("  h ^= mix_u64(static_cast<uint64_t>(it) + 0xbf58476d1ce4e5b9ULL);")
    lines.append("  return h;")
    lines.append("}")
    lines.append("}  // namespace")
    lines.append("")
    lines.append("double MCLLocalizer::regionWeight(double x, double y, double heading_deg) const {")
    lines.append("  double weight = 1.0;")
    lines.append("  if (MCL_REGION_PERIMETER_GATE) {")
    lines.append("    if (pose_outside_perimeter(x, y, heading_deg)) return 0.0;")
    lines.append("  }")
    lines.append("  double obj_w = object_clip_weight(x, y, heading_deg);")
    lines.append("  if (obj_w <= 0.0) return 0.0;")
    lines.append("  weight *= obj_w;")
    lines.append("  if (!MCL_REGION_ENABLED) return weight;")
    lines.append("  if (segment_band_active_ && segment_band_radius_ > 0.0 && segment_band_pts_.size() >= 2) {")
    lines.append("    double d2 = polyline_dist_sq(x, y, segment_band_pts_);")
    lines.append("    if (d2 > segment_band_radius_ * segment_band_radius_) {")
    lines.append("      if (MCL_REGION_MODE == MCL_REGION_MODE_HARD) return 0.0;")
    lines.append("      weight *= MCL_REGION_PENALTY;")
    lines.append("    }")
    lines.append("    return weight;")
    lines.append("  }")
    lines.append("  if (x < MCL_REGION_X_MIN_IN || x > MCL_REGION_X_MAX_IN ||")
    lines.append("      y < MCL_REGION_Y_MIN_IN || y > MCL_REGION_Y_MAX_IN) {")
    lines.append("    if (MCL_REGION_MODE == MCL_REGION_MODE_HARD) return 0.0;")
    lines.append("    weight *= MCL_REGION_PENALTY;")
    lines.append("  }")
    lines.append("  return weight;")
    lines.append("}")
    lines.append("")
    lines.append("MCLLocalizer::MCLLocalizer()")
    lines.append("  : count_(MCL_PARTICLE_COUNT), w_slow_(0.0), w_fast_(0.0), confidence_(0.0),")
    lines.append("    estimate_valid_(false), have_estimate_ever_(false), last_dist_used_mask_(0), segment_band_radius_(0.0),")
    lines.append("    segment_band_active_(false) {")
    lines.append("  if (MCL_KLD_ENABLED) count_ = MCL_N_MIN;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    particles_[i] = {0.0, 0.0, 0.0, 1.0 / std::max(1, count_)};")
    lines.append("  }")
    lines.append("  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("    last_dist_measured_mm_[i] = -1.0;")
    lines.append("    last_dist_expected_mm_[i] = -1.0;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::seed(unsigned int seed) {")
    lines.append("  std::srand(seed);")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::init(double start_x, double start_y, double start_theta) {")
    lines.append("  count_ = MCL_KLD_ENABLED ? MCL_N_MIN : MCL_PARTICLE_COUNT;")
    lines.append("  double base_x = start_x;")
    lines.append("  double base_y = start_y;")
    lines.append("  double base_th = start_theta;")
    lines.append("  int attempts = std::max(1, MCL_REGION_SAMPLE_ATTEMPTS);")
    lines.append("  estimate_valid_ = false;")
    lines.append("  have_estimate_ever_ = false;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    bool placed = false;")
    lines.append("    int tries = MCL_REGION_PERIMETER_GATE ? attempts : 1;")
    lines.append("    for (int a = 0; a < tries; ++a) {")
    lines.append("      double x = base_x + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_XY_IN);")
    lines.append("      double y = base_y + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_XY_IN);")
    lines.append("      double th = wrap_deg(base_th + rand_gaussian(0.0, MCL_SETPOSE_SIGMA_THETA_DEG));")
    lines.append("      if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(x, y, th)) continue;")
    lines.append("      particles_[i].x = x;")
    lines.append("      particles_[i].y = y;")
    lines.append("      particles_[i].theta = th;")
    lines.append("      placed = true;")
    lines.append("      break;")
    lines.append("    }")
    lines.append("    if (!placed) {")
    lines.append("      PoseSample p = sample_random_pose();")
    lines.append("      particles_[i].x = p.x;")
    lines.append("      particles_[i].y = p.y;")
    lines.append("      particles_[i].theta = p.theta;")
    lines.append("    }")
    lines.append("    particles_[i].w = 1.0 / std::max(1, count_);")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::initGlobal() {")
    lines.append("  count_ = MCL_KLD_ENABLED ? MCL_N_MIN : MCL_PARTICLE_COUNT;")
    lines.append("  estimate_valid_ = false;")
    lines.append("  have_estimate_ever_ = false;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    PoseSample p = sample_random_pose();")
    lines.append("    particles_[i].x = p.x;")
    lines.append("    particles_[i].y = p.y;")
    lines.append("    particles_[i].theta = p.theta;")
    lines.append("    particles_[i].w = 1.0 / std::max(1, count_);")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::setSegmentBand(const MCLPose* pts, int n, double radius_in) {")
    lines.append("  segment_band_pts_.clear();")
    lines.append("  segment_band_radius_ = 0.0;")
    lines.append("  segment_band_active_ = false;")
    lines.append("  if (!pts || n <= 0 || radius_in <= 0.0) return;")
    lines.append("  segment_band_pts_.assign(pts, pts + n);")
    lines.append("  segment_band_radius_ = std::max(0.0, radius_in);")
    lines.append("  segment_band_active_ = (segment_band_pts_.size() >= 2 && segment_band_radius_ > 0.0);")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::clearSegmentBand() {")
    lines.append("  segment_band_pts_.clear();")
    lines.append("  segment_band_radius_ = 0.0;")
    lines.append("  segment_band_active_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::predict(double dx_in, double dy_in, double dtheta_deg) {")
    lines.append("  if (!MCL_USE_MOTION) return;")
    lines.append("  if (std::fabs(dx_in) + std::fabs(dy_in) + std::fabs(dtheta_deg) <= 1e-9) return;")
    lines.append("  estimate_valid_ = false;")
    lines.append("  double dist = std::sqrt(dx_in * dx_in + dy_in * dy_in);")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double base_th = particles_[i].theta;")
    lines.append("    double ndx = dx_in;")
    lines.append("    double ndy = dy_in;")
    lines.append("    double nth = dtheta_deg;")
    lines.append("    if (MCL_USE_ALPHA_MODEL) {")
    lines.append("      double sigma_trans = MCL_ALPHA1 * dist + MCL_ALPHA2 * std::fabs(dtheta_deg);")
    lines.append("      double sigma_rot = MCL_ALPHA3 * dist + MCL_ALPHA4 * std::fabs(dtheta_deg);")
    lines.append("      ndx += rand_gaussian(0.0, sigma_trans);")
    lines.append("      ndy += rand_gaussian(0.0, sigma_trans);")
    lines.append("      nth += rand_gaussian(0.0, sigma_rot);")
    lines.append("    } else {")
    lines.append("      ndx += rand_gaussian(0.0, MCL_MOTION_SIGMA_X_IN);")
    lines.append("      ndy += rand_gaussian(0.0, MCL_MOTION_SIGMA_Y_IN);")
    lines.append("      nth += rand_gaussian(0.0, MCL_MOTION_SIGMA_THETA_DEG);")
    lines.append("    }")
    lines.append("    double wx = 0.0, wy = 0.0;")
    lines.append("    rotate_local_to_world(ndx, ndy, base_th, wx, wy);")
    lines.append("    double new_x = particles_[i].x + wx;")
    lines.append("    double new_y = particles_[i].y + wy;")
    lines.append("    double new_th = wrap_deg(base_th + nth);")
    lines.append("    if (MCL_REGION_PERIMETER_GATE && pose_outside_perimeter(new_x, new_y, new_th)) {")
    lines.append("      particles_[i].x = new_x;")
    lines.append("      particles_[i].y = new_y;")
    lines.append("      particles_[i].theta = new_th;")
    lines.append("      particles_[i].w = 0.0;")
    lines.append("      continue;")
    lines.append("    }")
    lines.append("    particles_[i].x = new_x;")
    lines.append("    particles_[i].y = new_y;")
    lines.append("    particles_[i].theta = new_th;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::updateDistance(const double* dist_mm, int count) {")
    lines.append("  if (!MCL_USE_DISTANCE || !dist_mm || count <= 0) return;")
    lines.append("  bool have_est = have_estimate_ever_;")
    lines.append("  MCLPose est = estimate_;")
    lines.append("  if (have_est) est = estimate();")
    lines.append("  estimate_valid_ = false;")
    lines.append("  int used = std::min(count, MCL_DISTANCE_SENSOR_COUNT);")
    lines.append("  last_dist_used_mask_ = 0;")
    lines.append("  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("    last_dist_measured_mm_[i] = -1.0;")
    lines.append("    last_dist_expected_mm_[i] = -1.0;")
    lines.append("  }")
    lines.append("  for (int i = 0; i < used; ++i) {")
    lines.append("    if (dist_mm[i] >= 0.0 && dist_mm[i] < 9000.0) last_dist_measured_mm_[i] = dist_mm[i];")
    lines.append("  }")
    lines.append("  for (int i = 0; i < used; ++i) skip_sensor_[i] = 0;")
    lines.append("  double neff = effectiveN();")
    lines.append("  double localized = (count_ > 0)")
    lines.append("    ? (1.0 - (neff / std::max(1.0, static_cast<double>(count_))))")
    lines.append("    : 0.0;")
    lines.append("  double gate_conf = MCL_CONFIDENCE_THRESHOLD;")
    lines.append("  bool allow_skip = (MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_BEAM) &&")
    lines.append("    (MCL_DIST_GATE_MM > 0.0) && (MCL_DIST_GATE_REJECT_RATIO > 0.0) &&")
    lines.append("    (count_ > 0) && (gate_conf > 0.0) && (localized >= gate_conf);")
    lines.append("  bool allow_innovation_skip = have_est && (MCL_DIST_INNOVATION_GATE_MM > 0.0);")
    lines.append("  if (allow_innovation_skip && MCL_DIST_INNOVATION_GATE_MIN_CONF > 0.0) {")
    lines.append("    allow_innovation_skip = (confidence_ >= MCL_DIST_INNOVATION_GATE_MIN_CONF);")
    lines.append("  }")
    lines.append("  if (allow_innovation_skip) {")
    lines.append("    for (int s = 0; s < used; ++s) {")
    lines.append("      if (skip_sensor_[s]) continue;")
    lines.append("      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];")
    lines.append("      double meas_raw = dist_mm[s];")
    lines.append("      if (meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (MCL_LF_IGNORE_MAX && MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD &&")
    lines.append("          meas >= s_max) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double off_x = 0.0, off_y = 0.0;")
    lines.append("      rotate_local_to_world(cfg.x_in, cfg.y_in, est.theta, off_x, off_y);")
    lines.append("      double ox = est.x + off_x;")
    lines.append("      double oy = est.y + off_y;")
    lines.append("      double heading = wrap_deg(est.theta + cfg.angle_deg + cfg.angle_offset_deg);")
    lines.append("      double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);")
    lines.append("      if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;")
    lines.append("      double expected_mm = expected_in * MCL_MM_PER_IN;")
    lines.append("      double gate_mm = (cfg.innovation_gate_mm > 0.0) ? cfg.innovation_gate_mm : MCL_DIST_INNOVATION_GATE_MM;")
    lines.append("      if (gate_mm > 0.0 && std::fabs(meas - expected_mm) > gate_mm) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("      }")
    lines.append("    }")
    lines.append("  }")
    lines.append("  if (allow_skip) {")
    lines.append("    for (int s = 0; s < used; ++s) {")
    lines.append("      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];")
    lines.append("      double meas_raw = dist_mm[s];")
    lines.append("      if (meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (meas > s_max) meas = s_max;")
    lines.append("      int gated = 0;")
    lines.append("      for (int i = 0; i < count_; ++i) {")
    lines.append("        double off_x = 0.0, off_y = 0.0;")
    lines.append("        rotate_local_to_world(cfg.x_in, cfg.y_in, particles_[i].theta, off_x, off_y);")
    lines.append("        double ox = particles_[i].x + off_x;")
    lines.append("        double oy = particles_[i].y + off_y;")
    lines.append("        double heading = wrap_deg(particles_[i].theta + cfg.angle_deg + cfg.angle_offset_deg);")
    lines.append("        double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);")
    lines.append("        if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;")
    lines.append("        double expected_mm = expected_in * MCL_MM_PER_IN;")
    lines.append("        if (std::fabs(meas - expected_mm) > MCL_DIST_GATE_MM) {")
    lines.append("          gated++;")
    lines.append("        }")
    lines.append("      }")
    lines.append("      if (gated >= static_cast<int>(MCL_DIST_GATE_REJECT_RATIO * count_)) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("      }")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double total = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double w = particles_[i].w;")
    lines.append("    for (int s = 0; s < used; ++s) {")
    lines.append("      if (skip_sensor_[s]) continue;")
    lines.append("      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];")
    lines.append("      double meas_raw = dist_mm[s];")
    lines.append("      if (meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (MCL_LF_IGNORE_MAX && MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD &&")
    lines.append("          meas >= s_max) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (meas > s_max) meas = s_max;")
    lines.append("      double off_x = 0.0, off_y = 0.0;")
    lines.append("      rotate_local_to_world(cfg.x_in, cfg.y_in, particles_[i].theta, off_x, off_y);")
    lines.append("      double ox = particles_[i].x + off_x;")
    lines.append("      double oy = particles_[i].y + off_y;")
    lines.append("      double heading = wrap_deg(particles_[i].theta + cfg.angle_deg + cfg.angle_offset_deg);")
    lines.append("      double w_meas = 1.0;")
    lines.append("      if (MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD) {")
    lines.append("        double meas_in = meas / MCL_MM_PER_IN;")
    lines.append("        double dir_x = 0.0, dir_y = 0.0;")
    lines.append("        heading_unit(heading, dir_x, dir_y);")
    lines.append("        double end_x = ox + dir_x * meas_in;")
    lines.append("        double end_y = oy + dir_y * meas_in;")
    lines.append("        double dist_mm = distance_to_map_mode(end_x, end_y, cfg.map_mode, s) * MCL_MM_PER_IN;")
    lines.append("        double p_hit = gaussian(dist_mm, 0.0, MCL_DIST_SIGMA_HIT_MM);")
    lines.append("        double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;")
    lines.append("        double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;")
    lines.append("        w_meas = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_MAX * p_max;")
    lines.append("        if (MCL_DIST_GATE_MM > 0.0 && dist_mm > MCL_DIST_GATE_MM) {")
    lines.append("          if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {")
    lines.append("            w_meas *= MCL_DIST_GATE_PENALTY;")
    lines.append("          } else {")
    lines.append("            w_meas = 0.0;")
    lines.append("          }")
    lines.append("        }")
    lines.append("      } else {")
    lines.append("        double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);")
    lines.append("        if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;")
    lines.append("        double expected_mm = expected_in * MCL_MM_PER_IN;")
    lines.append("        bool gated = (MCL_DIST_GATE_MM > 0.0 && std::fabs(meas - expected_mm) > MCL_DIST_GATE_MM);")
    lines.append("        double error = meas - expected_mm;")
    lines.append("        double p_hit = gaussian(error, 0.0, MCL_DIST_SIGMA_HIT_MM);")
    lines.append("        double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;")
    lines.append("        double p_short = 0.0;")
    lines.append("        if (MCL_DIST_W_SHORT > 0.0 && meas >= 0.0 && meas <= expected_mm) {")
    lines.append("          p_short = MCL_DIST_LAMBDA_SHORT * std::exp(-MCL_DIST_LAMBDA_SHORT * meas);")
    lines.append("        }")
    lines.append("        double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;")
    lines.append("        w_meas = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_SHORT * p_short + MCL_DIST_W_MAX * p_max;")
    lines.append("        if (gated) {")
    lines.append("          if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {")
    lines.append("            w_meas *= MCL_DIST_GATE_PENALTY;")
    lines.append("          } else {")
    lines.append("            w_meas = 0.0;")
    lines.append("          }")
    lines.append("        }")
    lines.append("      }")
    lines.append("      w *= w_meas;")
    lines.append("    }")
    lines.append("    tmp_weights_[i] = w;")
    lines.append("    total += w;")
    lines.append("  }")
    lines.append("  if (total <= 1e-12) return;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    particles_[i].w = tmp_weights_[i];")
    lines.append("  }")
    lines.append("  // Debug expected distance is computed from the prior estimate (pre-normalize).")
    lines.append("  MCLPose dbg_pose = est;")
    lines.append("  uint32_t mask = 0;")
    lines.append("  for (int s = 0; s < used; ++s) {")
    lines.append("    const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];")
    lines.append("    double meas = last_dist_measured_mm_[s];")
    lines.append("    if (meas < 0.0) continue;")
    lines.append("    double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("    if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("    double off_x = 0.0, off_y = 0.0;")
    lines.append("    rotate_local_to_world(cfg.x_in, cfg.y_in, dbg_pose.theta, off_x, off_y);")
    lines.append("    double ox = dbg_pose.x + off_x;")
    lines.append("    double oy = dbg_pose.y + off_y;")
    lines.append("    double heading = wrap_deg(dbg_pose.theta + cfg.angle_deg + cfg.angle_offset_deg);")
    lines.append("    double expected_in = raycast_distance_mode(ox, oy, heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);")
    lines.append("    if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;")
    lines.append("    last_dist_expected_mm_[s] = expected_in * MCL_MM_PER_IN;")
    lines.append("    if (!skip_sensor_[s]) mask |= (1u << s);")
    lines.append("  }")
    lines.append("  last_dist_used_mask_ = mask;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::updateIMU(double heading_deg) {")
    lines.append("  if (!MCL_USE_IMU) return;")
    lines.append("  estimate_valid_ = false;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double diff = angle_diff_deg(particles_[i].theta, heading_deg);")
    lines.append("    particles_[i].w *= gaussian(diff, 0.0, MCL_IMU_SIGMA_DEG);")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::updateVision(double x_in, double y_in, double theta_deg, double confidence) {")
    lines.append("  if (!MCL_USE_VISION) return;")
    lines.append("  if (confidence < MCL_VISION_CONFIDENCE_MIN) return;")
    lines.append("  estimate_valid_ = false;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double dx = particles_[i].x - x_in;")
    lines.append("    double dy = particles_[i].y - y_in;")
    lines.append("    double dist = std::sqrt(dx * dx + dy * dy);")
    lines.append("    particles_[i].w *= gaussian(dist, 0.0, MCL_VISION_SIGMA_XY_IN);")
    lines.append("    double diff = angle_diff_deg(particles_[i].theta, theta_deg);")
    lines.append("    particles_[i].w *= gaussian(diff, 0.0, MCL_VISION_SIGMA_THETA_DEG);")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::normalize() {")
    lines.append("  double total = 0.0;")
    lines.append("  double total_sensor = 0.0;")
    lines.append("  double sumsq_sensor = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double ws = particles_[i].w;")
    lines.append("    total_sensor += ws;")
    lines.append("    sumsq_sensor += ws * ws;")
    lines.append("    double w = particles_[i].w * regionWeight(particles_[i].x, particles_[i].y, particles_[i].theta);")
    lines.append("    particles_[i].w = w;")
    lines.append("    total += w;")
    lines.append("  }")
    lines.append("  double w_avg = total / std::max(1, count_);")
    lines.append("  if (MCL_AUGMENTED_ENABLED) {")
    lines.append("    if (w_slow_ <= 0.0) w_slow_ = w_avg;")
    lines.append("    else w_slow_ += MCL_ALPHA_SLOW * (w_avg - w_slow_);")
    lines.append("    if (w_fast_ <= 0.0) w_fast_ = w_avg;")
    lines.append("    else w_fast_ += MCL_ALPHA_FAST * (w_avg - w_fast_);")
    lines.append("  }")
    lines.append("  if (total <= 1e-12) {")
    lines.append("    for (int i = 0; i < count_; ++i) particles_[i].w = 1.0 / std::max(1, count_);")
    lines.append("    confidence_ = 0.0;")
    lines.append("    estimate_valid_ = false;")
    lines.append("    return;")
    lines.append("  }")
    lines.append("  for (int i = 0; i < count_; ++i) particles_[i].w /= total;")
    lines.append("  double neff = 0.0;")
    lines.append("  if (sumsq_sensor > 1e-18 && total_sensor > 1e-12) {")
    lines.append("    neff = (total_sensor * total_sensor) / sumsq_sensor;")
    lines.append("  } else {")
    lines.append("    neff = effectiveN();")
    lines.append("  }")
    lines.append("  double frac = neff / std::max(1.0, static_cast<double>(count_));")
    lines.append("  confidence_ = std::max(0.0, std::min(1.0, 1.0 - frac));")
    lines.append("  if (MCL_CONFIDENCE_AUTO_REINIT && MCL_CONFIDENCE_THRESHOLD > 0.0 &&")
    lines.append("      confidence_ < MCL_CONFIDENCE_THRESHOLD) {")
    lines.append("    if (MCL_REINIT_MODE == MCL_REINIT_MODE_ESTIMATE) {")
    lines.append("      MCLPose est = estimate();")
    lines.append("      init(est.x, est.y, est.theta);")
    lines.append("    } else {")
    lines.append("      initGlobal();")
    lines.append("    }")
    lines.append("    estimate();")
    lines.append("    return;")
    lines.append("  }")
    lines.append("  double x = 0.0, y = 0.0;")
    lines.append("  double sin_sum = 0.0, cos_sum = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    x += particles_[i].x * particles_[i].w;")
    lines.append("    y += particles_[i].y * particles_[i].w;")
    lines.append("    double th = particles_[i].theta * DEG_TO_RAD;")
    lines.append("    sin_sum += std::sin(th) * particles_[i].w;")
    lines.append("    cos_sum += std::cos(th) * particles_[i].w;")
    lines.append("  }")
    lines.append("  estimate_.x = x;")
    lines.append("  estimate_.y = y;")
    lines.append("  estimate_.theta = wrap_deg(std::atan2(sin_sum, cos_sum) * RAD_TO_DEG);")
    lines.append("  estimate_valid_ = true;")
    lines.append("  have_estimate_ever_ = true;")
    lines.append("}")
    lines.append("")
    lines.append("double MCLLocalizer::effectiveN() const {")
    lines.append("  double denom = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) denom += particles_[i].w * particles_[i].w;")
    lines.append("  return (denom <= 1e-12) ? 0.0 : (1.0 / denom);")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::getLastDistanceDebug(double* measured_mm, double* expected_mm, int capacity, uint32_t* used_mask) const {")
    lines.append("  int n = std::min(capacity, MCL_DISTANCE_SENSOR_COUNT_SAFE);")
    lines.append("  for (int i = 0; i < n; ++i) {")
    lines.append("    if (measured_mm) measured_mm[i] = last_dist_measured_mm_[i];")
    lines.append("    if (expected_mm) expected_mm[i] = last_dist_expected_mm_[i];")
    lines.append("  }")
    lines.append("  if (used_mask) *used_mask = last_dist_used_mask_;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::resample() {")
    lines.append("  double neff = effectiveN();")
    lines.append("  if (!MCL_RESAMPLE_ALWAYS && neff >= MCL_RESAMPLE_THRESHOLD * std::max(1, count_)) return;")
    lines.append("  int target = count_;")
    lines.append("  if (MCL_KLD_ENABLED) {")
    lines.append("    for (int i = 0; i < count_; ++i) {")
    lines.append("      bin_buf_[i] = bin_index(particles_[i].x, particles_[i].y, particles_[i].theta);")
    lines.append("    }")
    lines.append("    std::sort(bin_buf_, bin_buf_ + count_);")
    lines.append("    int k = 1;")
    lines.append("    for (int i = 1; i < count_; ++i) {")
    lines.append("      if (bin_buf_[i] != bin_buf_[i - 1]) k++;")
    lines.append("    }")
    lines.append("    int n_req = kld_required_particles(k, MCL_KLD_EPSILON, MCL_KLD_DELTA);")
    lines.append("    target = std::max(MCL_N_MIN, std::min(MCL_N_MAX, n_req));")
    lines.append("  }")
    lines.append("  target = std::max(1, std::min(target, MCL_PARTICLE_CAPACITY));")
    lines.append("  if (MCL_RESAMPLE_METHOD == MCL_RESAMPLE_STRATIFIED) {")
    lines.append("    double c = particles_[0].w;")
    lines.append("    int i = 0;")
    lines.append("    for (int m = 0; m < target; ++m) {")
    lines.append("      double u = (m + rand_uniform()) / target;")
    lines.append("      while (u > c && i < count_ - 1) {")
    lines.append("        i++;")
    lines.append("        c += particles_[i].w;")
    lines.append("      }")
    lines.append("      Particle p = particles_[i];")
    lines.append("      p.w = 1.0 / std::max(1, target);")
    lines.append("      resample_buf_[m] = p;")
    lines.append("    }")
    lines.append("  } else if (MCL_RESAMPLE_METHOD == MCL_RESAMPLE_MULTINOMIAL) {")
    lines.append("    double total = 0.0;")
    lines.append("    for (int i = 0; i < count_; ++i) {")
    lines.append("      total += particles_[i].w;")
    lines.append("      cdf_buf_[i] = total;")
    lines.append("    }")
    lines.append("    for (int m = 0; m < target; ++m) {")
    lines.append("      double r = rand_uniform() * total;")
    lines.append("      int idx = 0;")
    lines.append("      while (idx < count_ - 1 && r > cdf_buf_[idx]) idx++;")
    lines.append("      Particle p = particles_[idx];")
    lines.append("      p.w = 1.0 / std::max(1, target);")
    lines.append("      resample_buf_[m] = p;")
    lines.append("    }")
    lines.append("  } else {")
    lines.append("    double step = 1.0 / target;")
    lines.append("    double r = rand_uniform() * step;")
    lines.append("    double c = particles_[0].w;")
    lines.append("    int i = 0;")
    lines.append("    for (int m = 0; m < target; ++m) {")
    lines.append("      double u = r + m * step;")
    lines.append("      while (u > c && i < count_ - 1) {")
    lines.append("        i++;")
    lines.append("        c += particles_[i].w;")
    lines.append("      }")
    lines.append("      Particle p = particles_[i];")
    lines.append("      p.w = 1.0 / std::max(1, target);")
    lines.append("      resample_buf_[m] = p;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double inj = MCL_RANDOM_INJECTION;")
    lines.append("  if (MCL_AUGMENTED_ENABLED && w_slow_ > 1e-9 && w_fast_ > 1e-9) {")
    lines.append("    inj = std::max(inj, std::max(0.0, 1.0 - w_fast_ / w_slow_));")
    lines.append("  }")
    lines.append("  for (int i = 0; i < target; ++i) {")
    lines.append("    if (inj > 0.0 && rand_uniform() < inj) {")
    lines.append("      PoseSample p = sample_random_pose();")
    lines.append("      resample_buf_[i].x = p.x;")
    lines.append("      resample_buf_[i].y = p.y;")
    lines.append("      resample_buf_[i].theta = p.theta;")
    lines.append("      resample_buf_[i].w = 1.0 / std::max(1, target);")
    lines.append("    }")
    lines.append("  }")
    lines.append("  count_ = target;")
    lines.append("  for (int i = 0; i < count_; ++i) particles_[i] = resample_buf_[i];")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose MCLLocalizer::estimate() {")
    lines.append("  if (!estimate_valid_) {")
    lines.append("    double x = 0.0, y = 0.0;")
    lines.append("    double sin_sum = 0.0, cos_sum = 0.0;")
    lines.append("    for (int i = 0; i < count_; ++i) {")
    lines.append("      x += particles_[i].x * particles_[i].w;")
    lines.append("      y += particles_[i].y * particles_[i].w;")
    lines.append("      double th = particles_[i].theta * DEG_TO_RAD;")
    lines.append("      sin_sum += std::sin(th) * particles_[i].w;")
    lines.append("      cos_sum += std::cos(th) * particles_[i].w;")
    lines.append("    }")
    lines.append("    estimate_.x = x;")
    lines.append("    estimate_.y = y;")
    lines.append("    estimate_.theta = wrap_deg(std::atan2(sin_sum, cos_sum) * RAD_TO_DEG);")
    lines.append("    estimate_valid_ = true;")
    lines.append("    have_estimate_ever_ = true;")
    lines.append("  }")
    lines.append("  return estimate_;")
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def write_mcl_files(cfg: dict, out_dir: str) -> None:
    """Write mcl files."""
    cfg = _unwrap_values(cfg or {})
    os.makedirs(out_dir, exist_ok=True)
    inc_dir = os.path.join(out_dir, "include")
    src_dir = os.path.join(out_dir, "src")
    docs_dir = os.path.join(out_dir, "docs")
    ex_dir = os.path.join(out_dir, "examples")
    os.makedirs(inc_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)
    os.makedirs(docs_dir, exist_ok=True)
    os.makedirs(ex_dir, exist_ok=True)

    with open(os.path.join(inc_dir, "mcl_config.hpp"), "w", encoding="utf-8") as f:
        f.write(build_mcl_config_hpp(cfg)); f.write("\n")
    with open(os.path.join(inc_dir, "mcl_localizer.h"), "w", encoding="utf-8") as f:
        f.write(build_mcl_localizer_h(cfg))
    with open(os.path.join(inc_dir, "mcl_runtime.h"), "w", encoding="utf-8") as f:
        f.write(build_mcl_runtime_hpp(cfg))
    with open(os.path.join(inc_dir, "mcl_map_data.h"), "w", encoding="utf-8") as f:
        f.write(build_mcl_map_data_h(cfg))

    with open(os.path.join(src_dir, "mcl_localizer.cpp"), "w", encoding="utf-8") as f:
        f.write(build_mcl_localizer_cpp(cfg))
    with open(os.path.join(src_dir, "mcl_runtime.cpp"), "w", encoding="utf-8") as f:
        f.write(build_mcl_runtime_cpp(cfg))
    with open(os.path.join(src_dir, "mcl_map_data.cpp"), "w", encoding="utf-8") as f:
        f.write(build_mcl_map_data_cpp(cfg))

    tune_packets_h = r'''#pragma once

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
'''

    tune_log_h = r'''#pragma once

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
'''

    tune_log_cpp = r'''#include "mcl_tune_log.h"

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
'''

    tune_wizard_h = r'''#pragma once

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
'''

    tune_wizard_cpp = r'''#include "mcl_tuning.h"
'''

    with open(os.path.join(inc_dir, "mcl_tune_packets.h"), "w", encoding="utf-8") as f:
        f.write(tune_packets_h)
    with open(os.path.join(inc_dir, "mcl_tune_log.h"), "w", encoding="utf-8") as f:
        f.write(tune_log_h)
    with open(os.path.join(inc_dir, "mcl_tuning.h"), "w", encoding="utf-8") as f:
        f.write(tune_wizard_h)
    with open(os.path.join(src_dir, "mcl_tune_log.cpp"), "w", encoding="utf-8") as f:
        f.write(tune_log_cpp)
    with open(os.path.join(src_dir, "mcl_tuning.cpp"), "w", encoding="utf-8") as f:
        f.write(tune_wizard_cpp)

    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors.get("distance", {}) if isinstance(sensors, dict) else {}
    dist_on = int(dist_cfg.get("enabled", 1)) == 1 if isinstance(dist_cfg, dict) else True
    ekf_cfg = mcl.get("ekf", {}) if isinstance(mcl, dict) else {}
    ekf_on = int(ekf_cfg.get("enabled", 1)) == 1 if isinstance(ekf_cfg, dict) else True
    parts = mcl.get("particles", {}) if isinstance(mcl, dict) else {}
    dist_sensors = get_distance_sensors(cfg)

    particle_n = int(parts.get("n", parts.get("n_min", 200)))
    particle_n_min = int(parts.get("n_min", particle_n))
    particle_n_max = int(parts.get("n_max", max(particle_n, particle_n_min)))
    rates_cfg = mcl.get("rates", {}) if isinstance(mcl.get("rates", {}), dict) else {}
    motion_ms = int(mcl.get("motion_ms", rates_cfg.get("motion_ms", 20)))
    sensor_ms = int(mcl.get("sensor_ms", rates_cfg.get("sensor_ms", 50)))
    dist_status = "enabled" if dist_on else "disabled"
    ekf_status = "enabled" if ekf_on else "disabled"
    dist_ports = []
    for sensor in dist_sensors:
        if not isinstance(sensor, dict):
            continue
        try:
            port = int(sensor.get("port", 0))
        except Exception:
            port = 0
        if port > 0:
            dist_ports.append(port)
    if dist_ports:
        dist_ports_literal = "{ " + ", ".join(str(p) for p in dist_ports) + " }"
    else:
        placeholder_count = len(dist_sensors) if isinstance(dist_sensors, list) else 0
        if placeholder_count <= 0:
            placeholder_count = 3
        placeholder_count = max(1, min(placeholder_count, 4))
        placeholder_ports = list(range(1, placeholder_count + 1))
        dist_ports_literal = "{ " + ", ".join(str(p) for p in placeholder_ports) + " }"

    conv = (
        "# Coordinate Convention\n\n"
        "This export uses a field-centric Atticus convention everywhere (config, API, and examples).\n\n"
        "## Axes and heading\n"
        "- +x is forward (toward the far side of the field).\n"
        "- +y is left (from your alliance perspective).\n"
        "- Heading is clockwise-positive.\n"
        "- 0 deg faces +y (left), 90 deg faces +x (forward), 180 deg faces -y, 270 deg faces -x.\n\n"
        "## Frame expectations by API\n"
        "- `setPose(x, y, theta)`: field frame pose in inches + degrees.\n"
        "- `setOdomDelta(dx, dy, dtheta)`: robot-frame delta since last update (`dx` forward, `dy` left, `dtheta` CW deg).\n"
        "- `getPose()` / `getFusedPose()`: field frame pose.\n\n"
        "## Practical sanity checks\n"
        "1. Drive straight forward: `dx` should be positive and `dy` near zero.\n"
        "2. Strafe/slide left (if possible): `dy` should be positive.\n"
        "3. Turn right in place: `dtheta` should be positive.\n"
        "4. If signs are wrong, fix signs in your odometry adapter first (do not tune noise to hide sign errors).\n"
    )

    readme = (
        "# Atticus MCL + EKF Export\n\n"
        "This package is a practical localization stack for VEX/PROS:\n"
        "- `MCL` (particle filter) anchors pose using map geometry + semi-absolute sensors.\n"
        "- `EKF` smooths/filters pose updates for control loops.\n"
        "- `ProsMCL` runtime provides a single API for odometry deltas, sensor updates, and fused pose output.\n\n"
        "## Generated outputs\n"
        "- `include/`: generated headers (`mcl_config.hpp`, `mcl_localizer.h`, `mcl_runtime.h`, `mcl_map_data.h`)\n"
        "- `src/`: generated implementation (`mcl_localizer.cpp`, `mcl_runtime.cpp`, `mcl_map_data.cpp`)\n"
        "- `include/src` tuning helpers: `mcl_tune_packets.h`, `mcl_tune_log.*`, `mcl_tuning.*`\n"
        "- `docs/`: setup, tuning, convention, and API references\n"
        "- `examples/`: templates for minimal odom, distance sensors, and LemLib integration\n\n"
        "## Current export profile\n"
        f"- Particle budget: `{particle_n}` (`n_min={particle_n_min}`, `n_max={particle_n_max}`)\n"
        f"- Runtime cadence targets: motion `{motion_ms} ms`, sensor `{sensor_ms} ms`\n"
        f"- Distance sensors: {dist_status} (`{len(dist_sensors)}` configured)\n"
        f"- EKF: {ekf_status}\n\n"
        "## Fast-start integration checklist\n"
        "1. Create one global `ProsMCL` instance with your IMU + distance sensor ports.\n"
        "2. In `initialize()`, call `startEasy(seed, initial_heading_deg, x, y, theta)` so IMU calibration does not consume autonomous time.\n"
        "3. Choose one odometry feed mode:\n"
        "   - manual: call `setOdomDelta(...)` in your 10 ms control loop\n"
        "   - automatic: set `setFieldPoseProvider(...)` once (best for LemLib)\n"
        "   Runtime updates happen in the built-in background task (no `update()` call needed).\n"
        "   No global 15s timeout loop is required; competition mode exits autonomous automatically.\n"
        "4. Use `getFusedPose()` for auton/pathing decisions (not raw odometry pose).\n"
        "5. If using LemLib, keep LemLib as base odometry and periodically blend/correct using `applyOdomCorrection(...)`.\n\n"
        "## Performance intent\n"
        "- Tune drivetrain/pathing PID first; localization cannot compensate for unstable base motion.\n"
        "- Keep odom delta updates frequent and low-latency.\n"
        "- For long skills routes, avoid odom-only setups; include distance and/or vision corrections.\n"
        "- Keep units consistent (inches, degrees) and sign conventions correct.\n"
        "- Tune in this order: PID -> sign checks -> geometry -> motion noise -> sensor gating -> correction alpha.\n"
    )

    checklist = (
        "# Practical Configuration + Usage Checklist\n\n"
        "This is an exact tuning workflow for VEX teams. Run steps in order.\n\n"
        "## 1) PID first (required before localization tuning)\n"
        "1. Tune drive/turn/path PID with MCL corrections effectively off (`alpha_min=0`, `alpha_max=0`).\n"
        "2. Pass criteria before moving on:\n"
        "   - Drive 72 in straight, 5 reps: end error <= 2 in.\n"
        "   - Turn 90 deg, 5 reps: end error <= 2 deg.\n"
        "   - No sustained oscillation after settling.\n"
        "3. If PID is unstable, stop here; MCL/EKF tuning will not be trustworthy.\n\n"
        "## 2) Wiring + geometry calibration (hard requirements)\n"
        "1. IMU: verify calibration is complete before first movement.\n"
        "2. Distance sensors: verify port order matches `mcl.sensor_geometry.distance_sensors` exactly.\n"
        "3. Odom signs: forward => `dx>0`, left => `dy>0`, right turn => `dtheta>0`.\n"
        "   - Standard diff-drive encoder signs (both sides forward-positive): `dtheta_cw = (dL - dR) / track_width * 180/pi`.\n"
        "4. Track width: run 360 deg in-place turn, compare measured vs expected heading.\n"
        "   - If under-rotating, decrease track width by 2-5%.\n"
        "   - If over-rotating, increase track width by 2-5%.\n"
        "   - Repeat until 360 deg test error <= 3 deg.\n\n"
        "## 3) Runtime integration baseline\n"
        "1. Create one global `ProsMCL` instance.\n"
        "2. Call `startEasy(...)` once with correct start tile pose.\n"
        "3. Feed odom one way only:\n"
        "   - Manual `setOdomDelta(...)` every 10 ms, or\n"
        "   - LemLib provider via `setFieldPoseProvider(...)`.\n"
        "   - If provider heading already comes from IMU, set `mcl.ekf.use_imu_update=0` to avoid IMU double-counting.\n"
        "4. Use `getFusedPose()` for autonomous decisions.\n\n"
        "## 4) Baseline values (good first pass)\n"
        "- Particles: `n=300`, `n_min=200`, `n_max=500`.\n"
        "- Motion: `sigma_x_in=0.12`, `sigma_y_in=0.12`, `sigma_theta_deg=1.0`.\n"
        "- Distance: `sigma_hit_mm=8.5`, `gate_mm=120`, `innovation_gate_mm=0` (enable later after stable lock).\n"
        "- Correction: `alpha_min=0.03`, `alpha_max=0.12`, `min_confidence=0.6`.\n\n"
        "## 5) Exact tuning procedure\n"
        "1. **Motion noise first** (vision off during this step):\n"
        "   - Run a 30-45 second mixed route (straight + turns).\n"
        "   - If estimate is too jittery, reduce `sigma_x/y` by 0.02 and `sigma_theta` by 0.2.\n"
        "   - If estimate lags/sticks after collisions, increase `sigma_x/y` by 0.02 and `sigma_theta` by 0.2.\n"
        "2. **Distance gating second**:\n"
        "   - First tune with `innovation_gate_mm=0`.\n"
        "   - If false wall snaps occur: reduce `gate_mm` by 15 mm.\n"
        "   - If real readings are ignored: increase `gate_mm` by 15 mm.\n"
        "   - After stable runs, enable `innovation_gate_mm` in 20 mm steps.\n"
        "   - Keep `gate_mm` usually in 80-180 mm.\n"
        "3. **Particle count third**:\n"
        "   - If kidnapped recovery is slow, increase `n` by +50.\n"
        "   - If CPU load is high, decrease `n` by -50.\n"
        "   - Keep skills configs typically in 250-450.\n"
        "4. **Correction alpha last**:\n"
        "   - Start at `0.03/0.12`.\n"
        "   - If recovery after drift is too slow, raise both by +0.01.\n"
        "   - If oscillation appears, lower `alpha_max` by -0.02 first.\n"
        "   - Keep `alpha_max <= 0.18` for most robots.\n"
        "5. **Vision integration (optional final step)**:\n"
        "   - Feed only trusted fixes (`confidence >= 0.7` recommended).\n"
        "   - If vision causes jumps, raise confidence threshold or reduce vision update frequency.\n\n"
        "## 6) Quick symptom map\n"
        "- Mirrored/rotated map behavior: coordinate sign/convention mismatch.\n"
        "- Fast heading drift: track width or `dtheta` sign error.\n"
        "- Snapping to wrong geometry: sensor pose wrong or gates too loose.\n"
        "- Slow recovery after bump: motion noise too low or alpha too conservative.\n"
        "- Oscillation/over-correction: alpha too high or base PID too aggressive.\n\n"
        "## 7) No-USB tuning wizard flow (microSD, idiot-proof)\n"
        "### Before you run\n"
        "1. Run regression checks: `python mod/mcl_regression_checks.py` (must pass).\n"
        "2. Put a microSD card in the V5 Brain.\n"
        "3. Enable tuning mode in config (`mcl.tuning.enabled=1`) and export/build.\n"
        "4. Deploy `examples/04_tuning_wizard_session.cpp` (or same flow in your project).\n"
        "5. Confirm robot starts at known pose and field is clear.\n\n"
        "### Robot-side run steps\n"
        "1. Start the routine, wait for IMU calibration to finish.\n"
        "2. Call `tuning.begin(\"skills\")` at routine start.\n"
        "3. Mark checkpoints with:\n"
        "   - `markStep()` at each major phase boundary.\n"
        "   - `markKidnappedStart()` when you pick up/relocate robot.\n"
        "   - `markKidnappedPlaced()` right after placing robot down.\n"
        "4. Run at least: still phase, straight phase, turn phase, kidnapped phase.\n"
        "5. End with `tuning.end()` and verify LCD prints saved `.mcllog` path.\n\n"
        "### Import + apply in Atticus Terminal\n"
        "1. Remove microSD and open Tuning panel.\n"
        "2. Import latest `.mcllog` file.\n"
        "3. Confirm checks show PASS (duration, step markers, sensor coverage).\n"
        "4. Click Apply Recommended.\n"
        "5. Re-export MCL package, rebuild, rerun same session.\n\n"
        "### Acceptance targets (repeat until met)\n"
        "- Kidnapped recovery <= 8.0 s.\n"
        "- Distance dropout < 25%.\n"
        "- No repeated wrong-wall snaps in route replay.\n"
        "- Auton endpoints within your team tolerance on 3 back-to-back runs.\n"
    )

    api = (
        "# API Reference: ProsMCL Runtime\n\n"
        "This describes runtime methods in `mcl_runtime.h` and how to use them correctly.\n\n"
        "For practical tuning order (including PID-first workflow), see `docs/CHECKLIST.md`.\n\n"
        "## Constructor\n"
        "### `ProsMCL(int imu_port, const std::vector<int>& dist_ports)`\n"
        "- `imu_port`: V5 IMU smart port.\n"
        "- `dist_ports`: distance sensor smart ports in the same order as your configured distance sensor list.\n"
        "- Keep this as a long-lived singleton (global/static), not recreated in loops.\n\n"
        "## Lifecycle\n"
        "### `start(unsigned seed, double initial_heading_deg)`\n"
        "- Starts background update task and calibrates/uses IMU heading.\n"
        "- Prefer calling once in `initialize()` so IMU calibration does not consume autonomous time.\n"
        "- `seed`: deterministic test runs use a fixed value; comp runs can use `pros::millis()`.\n"
        "- `initial_heading_deg`: Atticus heading at robot boot pose.\n"
        "- If started with global initialization, EKF remains unlocked until MCL confidence passes threshold (or you call `setPose`).\n\n"
        "### `startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg)`\n"
        "- One-call startup for teams: starts runtime and sets known start pose.\n"
        "- Recommended default entry point for most users.\n\n"
        "### `stop()`\n"
        "- Stops runtime task cleanly (usually optional for normal PROS lifecycle).\n\n"
        "## State input\n"
        "### `setPose(double x_in, double y_in, double theta_deg)`\n"
        "- Re-initializes the particle cloud around the pose in field frame (not an exact hard lock unless set-pose sigmas are 0).\n"
        "- Use at init, explicit re-localization, or known reset points.\n\n"
        "### `setOdomDelta(double dx_in, double dy_in, double dtheta_deg)`\n"
        "- Robot-frame motion delta since the previous call.\n"
        "- `dx_in`: forward inches; `dy_in`: left inches; `dtheta_deg`: clockwise degrees.\n"
        "- Feed every control cycle (~10 ms) in manual mode.\n"
        "- Prefer encoder-derived `dtheta`; runtime already incorporates IMU heading updates.\n\n"
        "### `setFieldPoseProvider(FieldPoseProvider provider, void* user)` / `clearFieldPoseProvider()`\n"
        "- Auto-feed mode: provide field pose samples (e.g., LemLib `chassis.getPose`) and runtime computes deltas internally.\n"
        "- Best low-code path for most VEX teams using existing odometry frameworks.\n"
        "- Add an explicit pose adapter if your provider convention differs from MCL (`+x forward, +y left, CW+ heading`).\n"
        "- If provider heading already uses IMU, set `mcl.ekf.use_imu_update=0` to avoid double-counting heading.\n\n"
        "### `updateVision(double x_in, double y_in, double theta_deg, double confidence)`\n"
        "- Optional external absolute pose measurement (vision/camera/landmark).\n"
        "- `confidence` is [0..1] and should reflect measurement trust.\n\n"
        "### `setSegmentBand(const MCLPose* pts, int n, double radius_in)` / `clearSegmentBand()`\n"
        "- Optional temporary corridor constraint (for known route segments).\n"
        "- Use sparingly; overly tight bands can block recovery from collisions.\n\n"
        "## State output\n"
        "- Runtime updates happen automatically in the internal task.\n"
        "### `getPose()` vs `getFusedPose()`\n"
        "- `getPose()`: raw MCL estimate.\n"
        "- `getFusedPose()`: output after EKF smoothing/blending (usually best for control).\n\n"
        "### `applyOdomCorrection(MCLPose& odom_pose, double* out_alpha)`\n"
        "- Blends odom pose toward fused pose when EKF is enabled; otherwise toward raw MCL pose.\n"
        "- Returns `true` if correction is applied.\n"
        "- Use in LemLib bridging loops to gently remove drift instead of hard snaps.\n\n"
        "### `getDebugSnapshot()`\n"
        "- Returns a lock-safe telemetry snapshot for tuning/logging.\n"
        "- Includes odom/MCL/fused poses, EKF covariance terms, confidence/Neff, and distance expected/measured arrays.\n\n"
        "### `requestRelocalize()`\n"
        "- Requests runtime to re-localize using configured strategy.\n"
        "- Use when the robot was physically moved unexpectedly.\n\n"
        "## Configuration map (what major params mean)\n"
        "### Particle settings (`mcl.particles`)\n"
        "- `n`: nominal particle count.\n"
        "- `n_min`, `n_max`: adaptive bounds (used with KLD).\n"
        "- Higher counts improve global robustness but cost CPU.\n\n"
        "### Motion model (`mcl.motion`)\n"
        "- `sigma_x_in`, `sigma_y_in`, `sigma_theta_deg`: additive noise scale.\n"
        "- `use_alpha_model`, `alpha1..alpha4`: odom-motion dependent noise model.\n"
        "- `enabled`: allows turning motion model on/off (normally on).\n\n"
        "### Distance sensor model (`mcl.sensors.distance`)\n"
        "- `enabled`: distance update active.\n"
        "- `model`: `likelihood_field` (fast, robust default) or beam model.\n"
        "- `sigma_mm`, `max_range_mm`, `min_confidence`: measurement weighting/gating controls.\n"
        "- `gate_mode`, `innovation_gate_mm`: reject outliers hard/soft.\n"
        "- Per-sensor fields: `x_in`, `y_in`, `angle_deg`, `bias_mm`, range limits, object-size filters, map mode.\n\n"
        "### IMU / Vision (`mcl.sensors.imu`, `mcl.sensors.vision`)\n"
        "- IMU: heading noise and gating behavior.\n"
        "- Vision: confidence threshold and influence of absolute fixes.\n\n"
        "### Resampling (`mcl.resample`, `mcl.kld`, `mcl.augmented`)\n"
        "- `method`: systematic/stratified/multinomial.\n"
        "- `threshold`: effective sample size trigger.\n"
        "- KLD (`epsilon`, `delta`, bin sizes): adaptive particle sizing.\n"
        "- Augmented/random injection: recovery when filter degenerates.\n\n"
        "### Region + confidence + correction + EKF\n"
        "- `mcl.region`: hard/soft geographic constraints and penalties.\n"
        "- `mcl.confidence`: thresholding and auto-reinit policy.\n"
        "- `mcl.correction`: odom blend safety limits and alpha bounds.\n"
        "- `mcl.ekf`: smoothing noise, trust bounds, and initialization sigmas.\n\n"
        "## Tuning helper APIs (microSD workflow)\n"
        "- `mcl_tune_packets.h`: binary log header/frame/footer format.\n"
        "- `MCLTuneLogger` (`mcl_tune_log.h`): background snapshot logger to `/usd/...mcllog`.\n"
        "- `MCLTuningWizard` (`mcl_tuning.h`): simple step-marker wrapper for tuning sessions.\n"
    )

    terminology = (
        "# Terminology\n\n"
        "## Core concepts\n"
        "- **MCL (Monte Carlo Localization):** particle filter representing many pose hypotheses.\n"
        "- **Particle:** one candidate pose `(x, y, theta)` with weight `w`.\n"
        "- **Weight normalization:** scaling all particle weights so they sum to 1.\n"
        "- **Resampling:** drawing a new particle set from weighted particles to focus on likely states.\n"
        "- **Effective N (`Neff`):** degeneracy metric; low values indicate few particles dominate.\n"
        "- **KLD adaptive sampling:** adjusts particle count to match estimated posterior complexity.\n"
        "- **Fused pose:** final pose used by control (typically MCL + EKF blend).\n"
        "- **Confidence:** concentration of weights; higher means particles agree more strongly.\n\n"
        "## Sensor + map terms\n"
        "- **Likelihood field:** precomputed nearest-obstacle distance grid used for fast sensor scoring.\n"
        "- **Beam model:** ray-style sensor likelihood model (usually more compute-heavy).\n"
        "- **Innovation:** residual between predicted and measured sensor value.\n"
        "- **Gate:** rule that rejects or downweights improbable innovations.\n"
        "- **Perimeter map mode:** compare sensor rays only against outer field boundary geometry.\n"
        "- **Objects map mode:** compare only against configured objects/obstacles.\n"
        "- **Both map mode:** compare against perimeter + objects.\n\n"
        "## Motion + fusion terms\n"
        "- **Odometry delta:** incremental robot-frame motion (`dx`, `dy`, `dtheta`) between updates.\n"
        "- **Alpha motion model:** noise scales with rotational/translational motion magnitude.\n"
        "- **Random injection:** periodically reintroduces random particles for kidnapping recovery.\n"
        "- **Re-localize:** force broad/global or estimate-centered reinitialization.\n"
        "- **EKF (Extended Kalman Filter):** smooth state estimator over pose + covariance.\n"
        "- **Covariance (`P`):** uncertainty matrix tracked by EKF.\n"
        "- **Odom correction alpha:** blend ratio when nudging odom toward fused estimate.\n"
        "- **Segment band:** temporary path corridor constraint to discourage off-route hypotheses.\n\n"
        "## Coordinate words used in docs\n"
        "- **Field frame:** global map frame (`x` forward, `y` left).\n"
        "- **Robot frame:** local body frame (`dx` forward, `dy` left).\n"
        "- **CW heading:** clockwise-positive angle in degrees.\n"
    )

    template_pros_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "pros"))
    template_docs_dir = os.path.join(template_pros_dir, "docs")
    copied_md_rel = []
    if os.path.isdir(template_pros_dir):
        template_readme = os.path.join(template_pros_dir, "README.md")
        if os.path.isfile(template_readme):
            dst_md = os.path.join(out_dir, "README.md")
            os.makedirs(os.path.dirname(dst_md), exist_ok=True)
            with open(template_readme, "r", encoding="utf-8") as sf:
                content = sf.read()
            with open(dst_md, "w", encoding="utf-8") as df:
                df.write(content)
            copied_md_rel.append("README.md")
        if os.path.isdir(template_docs_dir):
            for root, _, files in os.walk(template_docs_dir):
                for name in files:
                    if not name.lower().endswith(".md"):
                        continue
                    if name.strip().lower() == "convention.md":
                        continue
                    src_md = os.path.join(root, name)
                    rel_md = os.path.relpath(src_md, template_pros_dir).replace("\\", "/")
                    dst_md = os.path.join(out_dir, rel_md)
                    os.makedirs(os.path.dirname(dst_md), exist_ok=True)
                    with open(src_md, "r", encoding="utf-8") as sf:
                        content = sf.read()
                    with open(dst_md, "w", encoding="utf-8") as df:
                        df.write(content)
                    copied_md_rel.append(rel_md)

    if not copied_md_rel:
        with open(os.path.join(out_dir, "README.md"), "w", encoding="utf-8") as f:
            f.write(readme)
        with open(os.path.join(docs_dir, "CHECKLIST.md"), "w", encoding="utf-8") as f:
            f.write(checklist)
        with open(os.path.join(docs_dir, "API_REFERENCE.md"), "w", encoding="utf-8") as f:
            f.write(api)
        with open(os.path.join(docs_dir, "TERMINOLOGY.md"), "w", encoding="utf-8") as f:
            f.write(terminology)
        examples_readme = (
            "# Examples\n\n"
            "These examples are practical autonomous templates, not toy snippets.\n"
            "Important: odom + IMU only will drift over long runs; use distance and/or vision corrections.\n\n"
            "- `03_mcl_intervenes_with_lemlib.cpp`: full LemLib autonomous flow with distance sensors, auto-feed pose provider, parallel correction, and live tuning telemetry.\n"
            "- `02_using_distance_sensors.cpp`: vision-assisted autonomous flow that combines odometry + distance + optional vision fixes.\n"
            "- `01_minimal_motor_encoders_imu.cpp`: minimal baseline if you need to start from a simple manual-odom structure.\n"
            "- `04_tuning_wizard_session.cpp`: microSD tuning wizard session logger (no USB required).\n"
        )
        with open(os.path.join(ex_dir, "README.md"), "w", encoding="utf-8") as f:
            f.write(examples_readme)
    else:
        copied_set = set(copied_md_rel)
        root_readme = os.path.join(out_dir, "README.md")
        if "README.md" not in copied_set and os.path.exists(root_readme):
            try:
                os.remove(root_readme)
            except Exception:
                pass
        if os.path.isdir(docs_dir):
            for name in os.listdir(docs_dir):
                if not name.lower().endswith(".md"):
                    continue
                rel_md = f"docs/{name}"
                if rel_md in copied_set:
                    continue
                stale = os.path.join(docs_dir, name)
                if os.path.exists(stale):
                    try:
                        os.remove(stale)
                    except Exception:
                        pass
        stale_examples_readme = os.path.join(ex_dir, "README.md")
        if os.path.exists(stale_examples_readme):
            try:
                os.remove(stale_examples_readme)
            except Exception:
                pass

    ex1 = r'''// 01_minimal_motor_encoders_imu.cpp

// Recommended baseline for accurate autonomous skills runs.
// Why this pattern is practical in VEX:
// - Keeps your drivetrain control loop simple.
// - Feeds odometry continuously.
// - Uses distance sensors as semi-absolute anchors to reduce long-run drift.
// - Lets runtime fuse MCL + EKF in the background task automatically.
// - Runs in normal autonomous flow (no custom 15s timer loop).

// -------------------------- REQUIRED USER INPUTS --------------------------
static constexpr int IMU_PORT = 10;               // Change to your IMU smart port.
static constexpr double TRACK_WIDTH_IN = 12.0;    // Change to your measured left-right tracking width.
// REQUIRED: keep this non-empty for real matches.
// Order must match mcl.distance_sensors order (front/left/right/etc in config).
// If you see placeholder ports here, replace them with your real smart ports.
static const std::vector<int> DIST_PORTS = __DIST_PORTS__;

ProsMCL localizer(IMU_PORT, DIST_PORTS);

// Odom cache for delta computation. Seed in initialize() from your real sensors.
static double prev_left_in = 0.0;
static double prev_right_in = 0.0;

// REQUIRED: replace with your real encoder-to-inch conversions.
static double read_left_inches() { return 0.0; }
static double read_right_inches() { return 0.0; }

// REQUIRED: replace with your drivetrain command call.
static void set_drive_arcade_mv(int forward_mv, int turn_mv) {
  (void)forward_mv;
  (void)turn_mv;
}

// Optional external absolute fix (camera/AprilTag/etc).
// Return true only when the fix is trustworthy.
struct VisionFix { double x; double y; double theta; double confidence; };
static bool read_vision_fix(VisionFix* out_fix) {
  (void)out_fix;
  return false;
}

static double angle_error_deg(double target_deg, double current_deg) {
  double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static void feed_manual_odom_once() {
  const double left_in = read_left_inches();
  const double right_in = read_right_inches();

  const double dL = left_in - prev_left_in;
  const double dR = right_in - prev_right_in;
  prev_left_in = left_in;
  prev_right_in = right_in;

  // Differential-drive robot-frame delta expected by ProsMCL.
  // NOTE: for standard encoder signs (both sides forward-positive), CW+ is (dL-dR).
  const double dx_in = 0.5 * (dL + dR);
  const double dy_in = 0.0;
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / M_PI;

  // Background runtime consumes these deltas; no explicit localizer.update() call is needed.
  localizer.setOdomDelta(dx_in, dy_in, dtheta_deg_cw);
}

void initialize() {
  pros::lcd::initialize();

  // One-call startup:
  // - starts runtime task
  // - sets known start pose
  //
  // REQUIRED INPUTS:
  // start_x_in, start_y_in, start_theta_deg must match your real starting tile.
  // Atticus convention: +x forward, +y left, theta CW+.
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);

  // Seed odom cache with current sensor readings to avoid first-loop jump.
  prev_left_in = read_left_inches();
  prev_right_in = read_right_inches();
}

void autonomous() {
  // No fixed 15s timeout loop needed; competition mode exits autonomous automatically.
  int stage = 0;
  while (pros::competition::is_autonomous()) {
    feed_manual_odom_once();

    // Optional vision update. Confidence is 0..1 (1 = highest trust).
    VisionFix fix{};
    if (read_vision_fix(&fix)) {
      localizer.updateVision(fix.x, fix.y, fix.theta, fix.confidence);
    }

    const MCLPose fused = localizer.getFusedPose();

    // Practical pose-driven autonomous state machine.
    // Replace thresholds and commands with your routine.
    if (stage == 0) {
      set_drive_arcade_mv(9000, 0);
      if (fused.x >= 24.0) stage = 1;
    } else if (stage == 1) {
      set_drive_arcade_mv(0, 7000);
      if (std::fabs(angle_error_deg(45.0, fused.theta)) <= 3.0) stage = 2;
    } else if (stage == 2) {
      set_drive_arcade_mv(8500, 0);
      if (fused.x >= 42.0) stage = 3;
    } else {
      set_drive_arcade_mv(0, 0);
    }

    pros::lcd::print(0, "AUTO fused x=%.1f y=%.1f th=%.1f", fused.x, fused.y, fused.theta);
    pros::delay(10);
  }
  set_drive_arcade_mv(0, 0);
}

void opcontrol() {}
'''
    ex1 = ex1.replace("__DIST_PORTS__", dist_ports_literal)
    with open(os.path.join(ex_dir, "01_minimal_motor_encoders_imu.cpp"), "w", encoding="utf-8") as f:
        f.write(ex1)

    ex2 = r'''// 02_using_distance_sensors.cpp

// Vision-assisted autonomous example.
// This is the practical pattern when you have:
// - wheel odometry + IMU
// - distance sensors for semi-absolute anchoring
// - optional vision pose fixes for long-run drift cleanup

// -------------------------- REQUIRED USER INPUTS --------------------------
static constexpr int IMU_PORT = 10;            // Change to your IMU port.
static constexpr double TRACK_WIDTH_IN = 12.0; // Change to measured track width in inches.
// REQUIRED: set your real distance sensor ports in configured order.
static const std::vector<int> DIST_PORTS = __DIST_PORTS__;

ProsMCL localizer(IMU_PORT, DIST_PORTS);

static double prev_left_in = 0.0;
static double prev_right_in = 0.0;

// REQUIRED: replace with your real encoder-to-inch conversions.
static double read_left_inches() { return 0.0; }
static double read_right_inches() { return 0.0; }

// REQUIRED: replace with your real drivetrain command.
static void set_drive_arcade_mv(int forward_mv, int turn_mv) {
  (void)forward_mv;
  (void)turn_mv;
}

// Vision pose adapter:
// Return true only when your camera-based pose is trustworthy.
// confidence should be 0..1 (1 = highest trust).
struct VisionFix { double x; double y; double theta; double confidence; };
static bool read_vision_fix(VisionFix* out_fix) {
  (void)out_fix;
  return false;
}

static double angle_error_deg(double target_deg, double current_deg) {
  double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static void feed_manual_odom_once() {
  const double left_in = read_left_inches();
  const double right_in = read_right_inches();

  const double dL = left_in - prev_left_in;
  const double dR = right_in - prev_right_in;
  prev_left_in = left_in;
  prev_right_in = right_in;

  // Differential drive robot-frame deltas expected by runtime.
  // NOTE: for standard encoder signs (both sides forward-positive), CW+ is (dL-dR).
  const double dx_in = 0.5 * (dL + dR);
  const double dy_in = 0.0;
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / M_PI;
  localizer.setOdomDelta(dx_in, dy_in, dtheta_deg_cw);
}

void initialize() {
  pros::lcd::initialize();

  // Start runtime once and set known start pose.
  // Edit these values to your actual start tile.
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);

  prev_left_in = read_left_inches();
  prev_right_in = read_right_inches();
}

void autonomous() {
  // Autonomous runs until competition framework exits the mode.
  int stage = 0;
  while (pros::competition::is_autonomous()) {
    feed_manual_odom_once();

    // Optional vision update on trusted fixes.
    // Bad vision fixes can hurt, so gate tightly in your adapter.
    VisionFix fix{};
    if (read_vision_fix(&fix)) {
      localizer.updateVision(fix.x, fix.y, fix.theta, fix.confidence);
    }

    // raw = raw MCL estimate, fused = MCL+EKF output (use fused for control).
    const MCLPose raw = localizer.getPose();
    const MCLPose fused = localizer.getFusedPose();

    // Practical state machine example.
    // Replace with your own autonomous routine logic.
    if (stage == 0) {
      set_drive_arcade_mv(9000, 0);
      if (fused.x >= 24.0) stage = 1;
    } else if (stage == 1) {
      set_drive_arcade_mv(0, 6500);
      if (std::fabs(angle_error_deg(35.0, fused.theta)) <= 3.0) stage = 2;
    } else if (stage == 2) {
      set_drive_arcade_mv(8500, 0);
      if (fused.x >= 40.0) stage = 3;
    } else {
      set_drive_arcade_mv(0, 0);
    }

    // Optional safety hook after obvious displacement events.
    // if (/* detected major collision / human touch */) localizer.requestRelocalize();

    pros::lcd::print(0, "AUTO raw  x=%.1f y=%.1f th=%.1f", raw.x, raw.y, raw.theta);
    pros::lcd::print(1, "AUTO fuse x=%.1f y=%.1f th=%.1f", fused.x, fused.y, fused.theta);
    pros::delay(10);
  }
  set_drive_arcade_mv(0, 0);
}

void opcontrol() {}
'''
    ex2 = ex2.replace("__DIST_PORTS__", dist_ports_literal)
    with open(os.path.join(ex_dir, "02_using_distance_sensors.cpp"), "w", encoding="utf-8") as f:
        f.write(ex2)

    ex3 = r'''// 03_mcl_intervenes_with_lemlib.cpp

// LemLib + MCL practical autonomous example (distance-sensor based).
// Flow:
// 1) Use LemLib as normal.
// 2) Set pose provider so MCL auto-feeds odometry from LemLib.
// 3) Start MCL once at autonomous start pose.
// 4) Run LemLib commands normally while a parallel correction task applies
//    small safe nudges back into LemLib pose.
//
// IMPORTANT: alpha comes from mcl.correction.alpha_min/max in config.
// Generated defaults are tuned conservative for skills:
//   alpha_min = 0.03, alpha_max = 0.12
// Put larger values only if your robot recovers too slowly and remains stable.
// In this example, correction runs in parallel with blocking LemLib commands.

extern lemlib::Chassis chassis;

static constexpr int IMU_PORT = 10; // Change to your IMU smart port.
// REQUIRED: set your real distance sensor smart ports in configured order.
static ProsMCL localizer(IMU_PORT, __DIST_PORTS__);
static std::atomic_bool correction_running{false};
static pros::Task* correction_task = nullptr;
// Teams can toggle this to quickly enable/disable live tuning telemetry.
static constexpr bool MCL_DEBUG_TELEMETRY = true;

// Pose adapters:
// Edit these two helpers if your LemLib convention differs from MCL
// (+x forward, +y left, theta CW+ with 0=left, 90=forward).
// If LemLib already matches, leave identity mapping as-is.
static MCLPose lemlib_to_mcl(const lemlib::Pose& p) {
  return MCLPose{p.x, p.y, p.theta};
}

static void mcl_to_lemlib(const MCLPose& p, double& x, double& y, double& theta) {
  x = p.x;
  y = p.y;
  theta = p.theta;
}

// Input adapter: runtime reads LemLib pose through this callback.
static bool lemlib_pose_provider(MCLPose* out_pose, void*) {
  const auto p = chassis.getPose(false, false);
  *out_pose = lemlib_to_mcl(p);
  return true;
}

static double wrap_err_deg(double target_deg, double current_deg) {
  double d = std::fmod(target_deg - current_deg + 180.0, 360.0);
  if (d < 0.0) d += 360.0;
  return d - 180.0;
}

static void correction_task_fn(void*) {
  std::uint32_t last_debug_ms = 0;
  while (correction_running.load()) {
    const auto p = chassis.getPose(false, false);
    const MCLPose odom_raw = lemlib_to_mcl(p);

    // odom_pose is the current LemLib odom estimate.
    MCLPose odom_pose = odom_raw;

    // alpha = blend strength selected by MCL using confidence and safety gates:
    // - near alpha_min for small safe corrections
    // - toward alpha_max only when confidence is strong
    // - 0 when no correction is applied
    double alpha = 0.0;
    if (localizer.applyOdomCorrection(odom_pose, &alpha)) {
      // Push corrected pose back into LemLib so pathing uses the improved estimate.
      double sx = 0.0, sy = 0.0, sth = 0.0;
      mcl_to_lemlib(odom_pose, sx, sy, sth);
      chassis.setPose(sx, sy, sth);
    }

    const MCLPose fused = localizer.getFusedPose();
    if (MCL_DEBUG_TELEMETRY) {
      const std::uint32_t now = pros::millis();
      if (now - last_debug_ms >= 50) {
        last_debug_ms = now;
        const double ex = fused.x - odom_raw.x;
        const double ey = fused.y - odom_raw.y;
        const double eth = wrap_err_deg(fused.theta, odom_raw.theta);
        pros::lcd::print(0, "ODOM x=%.1f y=%.1f t=%.1f", odom_raw.x, odom_raw.y, odom_raw.theta);
        pros::lcd::print(1, "FUSE x=%.1f y=%.1f t=%.1f", fused.x, fused.y, fused.theta);
        pros::lcd::print(2, "ERR  dx=%.2f dy=%.2f dt=%.2f", ex, ey, eth);
        pros::lcd::print(3, "MCL alpha=%.2f", alpha);
        // CSV-style serial log for offline tuning review:
        // time_ms, odom_x, odom_y, odom_theta, fused_x, fused_y, fused_theta, err_x, err_y, err_theta, alpha
        std::printf("MCLDBG,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    (unsigned long)now, odom_raw.x, odom_raw.y, odom_raw.theta, fused.x, fused.y, fused.theta, ex, ey, eth, alpha);
      }
    }
    pros::delay(10);
  }
}

void initialize() {
  pros::lcd::initialize();
  // Start here so IMU calibration completes pre-autonomous.
  localizer.start((int)pros::millis(), 90.0);
}

void autonomous() {
  // Start pose from LemLib should match real robot placement before first move.
  const auto p0 = chassis.getPose(false, false);
  const MCLPose p0_mcl = lemlib_to_mcl(p0);

  // Auto-feed odometry from LemLib into MCL runtime.
  // If LemLib heading already comes from IMU, disable MCL/EKF IMU updates in config
  // to avoid double-counting heading.
  localizer.setFieldPoseProvider(lemlib_pose_provider, nullptr);

  // Runtime is already running from initialize(); just lock belief to true start pose.
  localizer.setPose(p0_mcl.x, p0_mcl.y, p0_mcl.theta);

  // Parallel correction task: practical for long skills routes.
  correction_running.store(true);
  correction_task = new pros::Task(correction_task_fn, nullptr, "MCL-Corr");

  // Normal blocking LemLib auton commands.
  // Timeout values are per-command safety bounds, not a full-auton timer.
  // Replace coordinates/headings with your route.
  chassis.moveToPoint(24.0, 0.0, 1200);
  chassis.turnToHeading(45.0, 700);
  chassis.moveToPoint(42.0, 18.0, 1500);

  // Clean task shutdown at end of autonomous.
  correction_running.store(false);
  pros::delay(30);
  delete correction_task;
  correction_task = nullptr;
}

void opcontrol() {
  if (correction_task != nullptr) {
    correction_running.store(false);
    pros::delay(30);
    delete correction_task;
    correction_task = nullptr;
  }
}
'''
    ex3 = ex3.replace("__DIST_PORTS__", dist_ports_literal)
    with open(os.path.join(ex_dir, "03_mcl_intervenes_with_lemlib.cpp"), "w", encoding="utf-8") as f:
        f.write(ex3)

    ex4 = r'''// 04_tuning_wizard_session.cpp

// Completed no-USB tuning session example:
// - Logs compact telemetry to microSD (.mcllog)
// - Uses explicit wizard markers for Atticus Terminal analysis
// - Runs a full still + motion + kidnapped sequence
// - Feeds odometry through FieldPoseProvider so odom logs are meaningful
// - Requires mcl.tuning.enabled = 1 in config export

static constexpr int IMU_PORT = 10;
static constexpr int LEFT_DRIVE_PORT = 11;   // Update to your robot.
static constexpr int RIGHT_DRIVE_PORT = -12; // Negative reverses direction.
static constexpr double WHEEL_DIAMETER_IN = 2.75; // Update to your wheel diameter.
static constexpr double TRACK_WIDTH_IN = 12.0;    // Update to your measured track width.
static constexpr double kPi = 3.14159265358979323846;
static ProsMCL localizer(IMU_PORT, __DIST_PORTS__);
static MCLTuningWizard tuning(&localizer);
static pros::Motor left_drive(LEFT_DRIVE_PORT);
static pros::Motor right_drive(RIGHT_DRIVE_PORT);
static MCLPose odom_pose{0.0, 0.0, 90.0};
static double odom_prev_left_deg = 0.0;
static double odom_prev_right_deg = 0.0;
static bool odom_seeded = false;

static double wrap_deg(double deg) {
  while (deg >= 360.0) deg -= 360.0;
  while (deg < 0.0) deg += 360.0;
  return deg;
}

static void rotate_local_to_world(double lx, double ly, double heading_deg, double& wx, double& wy) {
  const double th = heading_deg * (kPi / 180.0);
  const double s = std::sin(th);
  const double c = std::cos(th);
  wx = lx * s - ly * c;
  wy = lx * c + ly * s;
}

static double motor_deg_to_in(double deg) {
  return (deg / 360.0) * (kPi * WHEEL_DIAMETER_IN);
}

static void reset_odom_seed(double x_in, double y_in, double theta_deg) {
  odom_pose = {x_in, y_in, wrap_deg(theta_deg)};
  odom_prev_left_deg = left_drive.get_position();
  odom_prev_right_deg = right_drive.get_position();
  odom_seeded = true;
}

// Runtime provider used by ProsMCL:
// - gives field pose each motion tick
// - lets runtime compute odom deltas + log odom_pose in .mcllog
static bool diff_drive_pose_provider(MCLPose* out_pose, void*) {
  const double left_deg = left_drive.get_position();
  const double right_deg = right_drive.get_position();
  if (!odom_seeded) {
    odom_prev_left_deg = left_deg;
    odom_prev_right_deg = right_deg;
    odom_seeded = true;
  }
  const double dL_in = motor_deg_to_in(left_deg - odom_prev_left_deg);
  const double dR_in = motor_deg_to_in(right_deg - odom_prev_right_deg);
  odom_prev_left_deg = left_deg;
  odom_prev_right_deg = right_deg;

  const double dx_robot = 0.5 * (dL_in + dR_in);
  const double dtheta_deg = ((dL_in - dR_in) / TRACK_WIDTH_IN) * (180.0 / kPi);

  double wx = 0.0, wy = 0.0;
  rotate_local_to_world(dx_robot, 0.0, odom_pose.theta, wx, wy);
  odom_pose.x += wx;
  odom_pose.y += wy;
  odom_pose.theta = wrap_deg(odom_pose.theta + dtheta_deg);

  *out_pose = odom_pose;
  return true;
}

static void set_drive(int left, int right) {
  left_drive.move(left);
  right_drive.move(right);
}

static void stop_drive() {
  set_drive(0, 0);
}

static void drive_for(int left, int right, int ms) {
  set_drive(left, right);
  pros::delay(ms);
  stop_drive();
  pros::delay(200);
}

void initialize() {
  pros::lcd::initialize();
  left_drive.tare_position();
  right_drive.tare_position();
  reset_odom_seed(0.0, 0.0, 90.0);
  localizer.setFieldPoseProvider(diff_drive_pose_provider, nullptr);
  localizer.startEasy((int)pros::millis(), 90.0, 0.0, 0.0, 90.0);
}

void autonomous() {
  if (!tuning.begin("skills")) {
    pros::lcd::print(0, "Tune start failed (SD?)");
    return;
  }

  // STEP 1: Mark A confirmation.
  tuning.markStep();
  pros::lcd::print(0, "STEP1 Mark A confirm");
  stop_drive();
  pros::delay(1000);

  // STEP 2: Stillness window (noise floor).
  tuning.markStep();
  pros::lcd::print(0, "STEP2 stillness 8s");
  pros::delay(8000);

  // STEP 3: Straight out/back profile.
  tuning.markStep();
  pros::lcd::print(0, "STEP3 straight out/back");
  drive_for(70, 70, 2000);
  drive_for(-70, -70, 2000);

  // STEP 4: In-place spin profile.
  tuning.markStep();
  pros::lcd::print(0, "STEP4 spin profile");
  drive_for(65, -65, 1700);
  drive_for(-65, 65, 1700);

  // STEP 5: Kidnapped recovery profile.
  tuning.markKidnappedStart();
  pros::lcd::print(0, "STEP5 Pick+place robot");
  stop_drive();
  pros::delay(2000);  // Operator relocates robot.
  tuning.markKidnappedPlaced();
  pros::lcd::print(0, "STEP5 recovery settle");
  pros::delay(5000);

  tuning.end();
  pros::lcd::print(1, "Saved: %s", tuning.filePath());
}

void opcontrol() {
  stop_drive();
}
'''
    ex4 = ex4.replace("__DIST_PORTS__", dist_ports_literal)
    with open(os.path.join(ex_dir, "04_tuning_wizard_session.cpp"), "w", encoding="utf-8") as f:
        f.write(ex4)
