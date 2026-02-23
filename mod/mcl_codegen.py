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
    # Internal field frame: x-forward span uses field_h, y-left span uses field_w.
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


def _lf_dims(field_w: float, field_h: float, res_in: float) -> Tuple[int, int]:
    """Compute LF grid dimensions (x,y) for a given resolution."""
    # Internal field frame is +x forward, +y left.
    # With screen-derived dimensions this means:
    #   x-span uses field_h, y-span uses field_w.
    nx = int(math.floor(field_h / res_in)) + 1
    ny = int(math.floor(field_w / res_in)) + 1
    return nx, ny


def _lf_bytes_for_dims(nx: int, ny: int, bytes_per_cell: int = 2) -> int:
    """Compute contiguous LF bytes."""
    if nx <= 0 or ny <= 0:
        return 0
    return int(nx) * int(ny) * int(bytes_per_cell)


LF_COARSEN_LADDER_IN = (0.5, 1.0, 1.5, 2.0, 3.0)


def _choose_lf_resolution(field_w: float, field_h: float, base_res_in: float, max_bytes: int) -> float:
    """Deterministically coarsen LF resolution until within byte cap."""
    base = max(1e-6, float(base_res_in))
    cap = max(1024, int(max_bytes))
    max_span = max(field_w, field_h) * 2.0

    # Locked deterministic ladder for export parity across environments.
    candidates = [r for r in LF_COARSEN_LADDER_IN if r + 1e-9 >= base]
    if not candidates:
        candidates = [base]
    elif abs(candidates[0] - base) > 1e-9:
        candidates.insert(0, base)

    for res in candidates:
        nx, ny = _lf_dims(field_w, field_h, res)
        if _lf_bytes_for_dims(nx, ny, 2) <= cap:
            return res

    # If still over budget beyond the fixed ladder, continue deterministically.
    res = max(candidates[-1], base)
    for _ in range(24):
        nx, ny = _lf_dims(field_w, field_h, res)
        if _lf_bytes_for_dims(nx, ny, 2) <= cap:
            return res
        res *= 2.0
        if res > max_span:
            break
    return res


def _build_raycast_bucket_grid(
    segs: List[Tuple[float, float, float, float]],
    field_w: float,
    field_h: float,
    bucket_in: float,
):
    """Build a deterministic segment bucket grid for beam-mode raycast acceleration."""
    if not segs:
        return None
    try:
        cell = float(bucket_in)
    except Exception:
        cell = 12.0
    if not math.isfinite(cell) or cell <= 1e-6:
        cell = 12.0
    # Internal field frame: x span uses field_h, y span uses field_w.
    x0 = -field_h * 0.5
    y0 = -field_w * 0.5
    nx = max(1, int(math.ceil(field_h / cell)))
    ny = max(1, int(math.ceil(field_w / cell)))
    buckets: List[List[int]] = [[] for _ in range(nx * ny)]

    def _clamp_i(v: int, lo: int, hi: int) -> int:
        return lo if v < lo else hi if v > hi else v

    for idx, (sx0, sy0, sx1, sy1) in enumerate(segs):
        xmin = min(sx0, sx1)
        xmax = max(sx0, sx1)
        ymin = min(sy0, sy1)
        ymax = max(sy0, sy1)
        ix0 = _clamp_i(int(math.floor((xmin - x0) / cell)), 0, nx - 1)
        ix1 = _clamp_i(int(math.floor((xmax - x0) / cell)), 0, nx - 1)
        iy0 = _clamp_i(int(math.floor((ymin - y0) / cell)), 0, ny - 1)
        iy1 = _clamp_i(int(math.floor((ymax - y0) / cell)), 0, ny - 1)
        for iy in range(iy0, iy1 + 1):
            row = iy * nx
            for ix in range(ix0, ix1 + 1):
                buckets[row + ix].append(idx)

    offsets: List[int] = [0]
    flat: List[int] = []
    for b in buckets:
        if b:
            flat.extend(sorted(set(int(i) for i in b if i >= 0)))
        offsets.append(len(flat))
    return {
        "origin_x": x0,
        "origin_y": y0,
        "cell_in": cell,
        "nx": nx,
        "ny": ny,
        "offsets": offsets,
        "indices": flat,
    }


def _emit_bucket_grid(lines: List[str], grid, name: str) -> None:
    """Emit static arrays/constants for a precomputed raycast bucket grid."""
    if not grid:
        lines.append(f"static const int {name}_NX = 0;")
        lines.append(f"static const int {name}_NY = 0;")
        lines.append(f"static const double {name}_CELL_IN = 0.0;")
        lines.append(f"static const double {name}_ORIGIN_X = 0.0;")
        lines.append(f"static const double {name}_ORIGIN_Y = 0.0;")
        lines.append(f"static const uint32_t {name}_OFFSETS[] = {{0}};")
        lines.append(f"static const uint16_t {name}_INDICES[] = {{0}};")
        return
    nx = int(grid.get("nx", 0))
    ny = int(grid.get("ny", 0))
    cell = float(grid.get("cell_in", 0.0))
    ox = float(grid.get("origin_x", 0.0))
    oy = float(grid.get("origin_y", 0.0))
    offsets = [int(v) for v in (grid.get("offsets", []) or [0])]
    indices = [int(v) for v in (grid.get("indices", []) or [0])]
    if not offsets:
        offsets = [0]
    if not indices:
        indices = [0]
    lines.append(f"static const int {name}_NX = {nx};")
    lines.append(f"static const int {name}_NY = {ny};")
    lines.append(f"static const double {name}_CELL_IN = {_fmt(cell)};")
    lines.append(f"static const double {name}_ORIGIN_X = {_fmt(ox)};")
    lines.append(f"static const double {name}_ORIGIN_Y = {_fmt(oy)};")
    lines.append(f"static const uint32_t {name}_OFFSETS[] = {{")
    row = "  "
    for i, off in enumerate(offsets):
        row += f"{int(off)}u"
        if i != len(offsets) - 1:
            row += ", "
        if len(row) > 110 and i != len(offsets) - 1:
            lines.append(row)
            row = "  "
    if row.strip():
        lines.append(row)
    lines.append("};")
    lines.append(f"static const uint16_t {name}_INDICES[] = {{")
    row = "  "
    for i, idx in enumerate(indices):
        row += f"{int(max(0, min(65535, idx)))}u"
        if i != len(indices) - 1:
            row += ", "
        if len(row) > 110 and i != len(indices) - 1:
            lines.append(row)
            row = "  "
    if row.strip():
        lines.append(row)
    lines.append("};")


def _build_distance_field(segs: List[Tuple[float, float, float, float]], field_w: float, field_h: float,
                          res_in: float):
    """Build distance field."""
    if res_in <= 1e-6 or not segs:
        return None
    nx, ny = _lf_dims(field_w, field_h, res_in)
    # Origin matches internal field frame (+x forward uses field_h, +y left uses field_w).
    x0 = -field_h * 0.5
    y0 = -field_w * 0.5
    grid_mm_u16: List[int] = []
    for j in range(ny):
        y = y0 + j * res_in
        for i in range(nx):
            x = x0 + i * res_in
            best = 1e18
            for sx0, sy0, sx1, sy1 in segs:
                d2 = _dist_point_seg_sq(x, y, sx0, sy0, sx1, sy1)
                if d2 < best:
                    best = d2
            dist_in = math.sqrt(best)
            dist_mm = int(round(dist_in * 25.4))
            if dist_mm < 0:
                dist_mm = 0
            if dist_mm > 65535:
                dist_mm = 65535
            grid_mm_u16.append(dist_mm)
    return {
        "nx": nx,
        "ny": ny,
        "res": res_in,
        "x0": x0,
        "y0": y0,
        "grid_mm_u16": grid_mm_u16,
        "bytes": _lf_bytes_for_dims(nx, ny, 2),
    }


def _compute_map_blob_meta(cfg: dict) -> dict:
    """Compute map/raycast blob metadata and deterministic hashes."""
    cfg = _unwrap_values(cfg or {})
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    lf_cfg = dist_cfg.get("likelihood_field", {}) if isinstance(dist_cfg, dict) else {}

    field_w = float(WINDOW_WIDTH) / float(PPI)
    field_h = float(WINDOW_HEIGHT) / float(PPI)
    segs = _segments_in_field_in(cfg, field_w, field_h)
    perim_segs = _perimeter_segments_in_field_in(cfg, field_w, field_h)
    obj_segs = _object_segments_in_field_in(cfg, field_w, field_h)

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

    df_all = None
    df_perim = None
    df_obj = None
    if use_lf:
        try:
            res_in = float(lf_cfg.get("resolution_in", 1.0))
        except Exception:
            res_in = 1.0
        try:
            lf_max_bytes = int(lf_cfg.get("max_bytes", 262144))
        except Exception:
            lf_max_bytes = 262144
        res_in = _choose_lf_resolution(field_w, field_h, res_in, lf_max_bytes)
        df_all = _build_distance_field(segs, field_w, field_h, res_in)
        df_perim = _build_distance_field(perim_segs, field_w, field_h, res_in)
        df_obj = _build_distance_field(obj_segs, field_w, field_h, res_in)

    try:
        ray_bucket_in = float(dist_cfg.get("raycast_bucket_in", 12.0)) if isinstance(dist_cfg, dict) else 12.0
    except Exception:
        ray_bucket_in = 12.0
    bucket_all = _build_raycast_bucket_grid(segs, field_w, field_h, ray_bucket_in)
    bucket_perim = _build_raycast_bucket_grid(perim_segs, field_w, field_h, ray_bucket_in)
    bucket_obj = _build_raycast_bucket_grid(obj_segs, field_w, field_h, ray_bucket_in)

    hash_payload = {
        "field_w": round(field_w, 6),
        "field_h": round(field_h, 6),
        "use_lf": bool(use_lf),
        "df_all": df_all.get("grid_mm_u16", []) if isinstance(df_all, dict) else [],
        "df_perim": df_perim.get("grid_mm_u16", []) if isinstance(df_perim, dict) else [],
        "df_obj": df_obj.get("grid_mm_u16", []) if isinstance(df_obj, dict) else [],
        "bucket_all": bucket_all,
        "bucket_perim": bucket_perim,
        "bucket_obj": bucket_obj,
    }
    map_hash = _config_hash(hash_payload)[:8]
    project_hash = _config_hash(
        {
            "mcl": mcl,
            "segments": segs,
            "perimeter": perim_segs,
            "objects": obj_segs,
        }
    )[:8]
    grid_w = int(df_all["nx"]) if isinstance(df_all, dict) and "nx" in df_all else 1
    grid_h = int(df_all["ny"]) if isinstance(df_all, dict) and "ny" in df_all else 1
    dist_len = len(df_all.get("grid_mm_u16", [])) if isinstance(df_all, dict) else 1
    return {
        "df_all": df_all,
        "df_perim": df_perim,
        "df_obj": df_obj,
        "bucket_all": bucket_all,
        "bucket_perim": bucket_perim,
        "bucket_obj": bucket_obj,
        "map_hash32": map_hash,
        "project_hash32": project_hash,
        "grid_w": grid_w,
        "grid_h": grid_h,
        "dist_len": max(1, int(dist_len)),
    }


def build_mcl_config_hpp(cfg: dict) -> str:
    """Build mcl config hpp."""
    cfg = _unwrap_values(cfg or {})
    mcl = cfg.get("mcl", {})
    map_meta = _compute_map_blob_meta(cfg)
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
    mode_split = mcl.get("mode_split", {}) if isinstance(mcl, dict) else {}
    interop = mcl.get("interop", {}) if isinstance(mcl, dict) else {}
    ekf = mcl.get("ekf", {})
    tuning = mcl.get("tuning", {}) if isinstance(mcl, dict) else {}
    aug = mcl.get("augmented", {})
    cgr_lite = mcl.get("cgr_lite", {}) if isinstance(mcl, dict) else {}

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
    if dist_model not in ("likelihood_field", "beam"):
        dist_model = "likelihood_field"
    dist_model_val = 0 if dist_model == "likelihood_field" else 1
    dist_fov_multi = _bool(dist.get("fov_multi_ray", 0)) and dist_model_val == 0
    dist_rays = int(dist.get("rays_per_sensor", 3) or 0)
    if dist_model_val != 0:
        dist_rays = 1
    pose_conv = str(interop.get("pose_convention", "cw_zero_forward")).strip().lower()
    if pose_conv in ("cw_forward", "cw_zero_forward", "vex_cw_forward", "conventional_cw", "cw0forward"):
        pose_conv_val = 1
    elif pose_conv in ("ccw_forward", "ccw_zero_forward", "math_ccw_forward", "conventional_ccw", "ccw0forward"):
        pose_conv_val = 2
    else:
        pose_conv_val = 0
    pose_swap_xy = _bool(interop.get("swap_xy", 0))
    pose_invert_x = _bool(interop.get("invert_x", 0))
    pose_invert_y = _bool(interop.get("invert_y", 0))
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
    loop_ms = int(mcl.get("loop_ms", rates.get("loop_ms", 10)))
    if loop_ms <= 0:
        loop_ms = 10
    stall_ms = int(mcl.get("stall_ms", max(1, 2 * loop_ms)))
    motion_ms = int(mcl.get("motion_ms", rates.get("motion_ms", loop_ms)))
    sensor_ms = int(mcl.get("sensor_ms", rates.get("sensor_ms", 50)))
    guard_vmax_in_s = _to_float(motion.get("guard_vmax_in_s", 60.0), 60.0)
    guard_wmax_deg_s = _to_float(motion.get("guard_wmax_deg_s", 540.0), 540.0)
    guard_margin_in = _to_float(motion.get("guard_margin_in", 0.5), 0.5)
    guard_margin_deg = _to_float(motion.get("guard_margin_deg", 8.0), 8.0)
    max_dx_tick = _to_float(motion.get("max_dx_in_per_tick", 0.0), 0.0)
    max_dy_tick = _to_float(motion.get("max_dy_in_per_tick", 0.0), 0.0)
    max_dth_tick = _to_float(motion.get("max_dtheta_deg_per_tick", 0.0), 0.0)
    if max_dx_tick <= 0.0:
        max_dx_tick = max(0.0, guard_vmax_in_s * (loop_ms / 1000.0) + guard_margin_in)
    if max_dy_tick <= 0.0:
        max_dy_tick = max(0.0, guard_vmax_in_s * (loop_ms / 1000.0) + guard_margin_in)
    if max_dth_tick <= 0.0:
        max_dth_tick = max(0.0, guard_wmax_deg_s * (loop_ms / 1000.0) + guard_margin_deg)
    recovery = mcl.get("recovery", {}) if isinstance(mcl, dict) else {}
    correction = mcl.get("correction", {}) if isinstance(mcl, dict) else {}
    imu_check_cal = _bool(imu.get("check_calibrating", 1))
    imu_fallback_noise_scale = _to_float(imu.get("fallback_noise_scale", 2.0), 2.0)
    frame_test = mcl.get("frame_sign_self_test", {}) if isinstance(mcl, dict) else {}
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
        "constexpr int MCL_POSE_CONVENTION_ATTICUS = 0;",
        "constexpr int MCL_POSE_CONVENTION_CW_ZERO_FORWARD = 1;",
        "constexpr int MCL_POSE_CONVENTION_CCW_ZERO_FORWARD = 2;",
        f"constexpr int MCL_EXPORT_POSE_CONVENTION = {pose_conv_val};",
        f"constexpr bool MCL_EXPORT_SWAP_XY = {pose_swap_xy};",
        f"constexpr bool MCL_EXPORT_INVERT_X = {pose_invert_x};",
        f"constexpr bool MCL_EXPORT_INVERT_Y = {pose_invert_y};",
        f"constexpr double MCL_BOT_WIDTH_IN = {_fmt(bot_width)};",
        f"constexpr double MCL_BOT_LENGTH_IN = {_fmt(bot_length)};",
        f"constexpr double MCL_BOT_OFFSET_X_IN = {_fmt(bot_off_x)};",
        f"constexpr double MCL_BOT_OFFSET_Y_IN = {_fmt(bot_off_y)};",
        f"constexpr const char* MCL_CONFIG_HASH = \"{cfg_hash}\";",
        f"constexpr unsigned int MCL_CONFIG_HASH32 = 0x{cfg_hash32}u;",
        f"constexpr unsigned int MCL_EXPECTED_MAP_HASH32 = 0x{map_meta.get('map_hash32', '0')}u;",
        f"constexpr unsigned int MCL_EXPECTED_MAP_PROJECT_HASH32 = 0x{map_meta.get('project_hash32', '0')}u;",
        f"constexpr int MCL_EXPECTED_MAP_GRID_W = {int(map_meta.get('grid_w', 1))};",
        f"constexpr int MCL_EXPECTED_MAP_GRID_H = {int(map_meta.get('grid_h', 1))};",
        f"constexpr int MCL_EXPECTED_MAP_DIST_LEN = {int(map_meta.get('dist_len', 1))};",
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
        f"constexpr bool MCL_CGR_LITE_ENABLED = {_bool(cgr_lite.get('enabled', 0))};",
        f"constexpr int MCL_CGR_LITE_TOP_K = {max(1, int(cgr_lite.get('top_k', 8) or 8))};",
        f"constexpr int MCL_CGR_LITE_MAX_ITERS = {max(1, int(cgr_lite.get('max_iters', 2) or 2))};",
        f"constexpr double MCL_CGR_LITE_BUDGET_MS = {_fmt(cgr_lite.get('budget_ms', 1.5))};",
        "",
        f"constexpr int MCL_MOTION_UPDATE_MS = {motion_ms};",
        f"constexpr int MCL_SENSOR_UPDATE_MS = {sensor_ms};",
        f"constexpr int MCL_LOOP_MS = {loop_ms};",
        f"constexpr int MCL_STALL_MS = {stall_ms};",
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
        f"constexpr bool MCL_ODOM_DELTA_GUARD_ENABLED = {_bool(motion.get('delta_guard_enabled', 1))};",
        f"constexpr double MCL_ODOM_MAX_DX_IN_PER_TICK = {_fmt(max_dx_tick)};",
        f"constexpr double MCL_ODOM_MAX_DY_IN_PER_TICK = {_fmt(max_dy_tick)};",
        f"constexpr double MCL_ODOM_MAX_DTHETA_DEG_PER_TICK = {_fmt(max_dth_tick)};",
        f"constexpr int MCL_ODOM_FAULT_INFLATE_CYCLES = {int(motion.get('fault_inflate_cycles', 2) or 0)};",
        f"constexpr double MCL_ODOM_FAULT_NOISE_SCALE = {_fmt(motion.get('fault_noise_scale', 2.5))};",
        f"constexpr double MCL_SETPOSE_SIGMA_XY_IN = {_fmt(mcl.get('set_pose_sigma_xy_in', 0.2))};",
        f"constexpr double MCL_SETPOSE_SIGMA_THETA_DEG = {_fmt(mcl.get('set_pose_sigma_theta_deg', 2.0))};",
        "",
        f"constexpr bool MCL_USE_DISTANCE = {_bool(dist.get('enabled', 1))};",
        f"constexpr int MCL_DISTANCE_MODEL = {dist_model_val};",
        f"constexpr double MCL_DIST_SIGMA_HIT_MM = {_fmt(dist.get('sigma_hit_mm', 15.0))};",
        f"constexpr double MCL_DIST_SIGMA_FAR_SCALE = {_fmt(dist.get('sigma_far_scale', 0.05))};",
        f"constexpr double MCL_DIST_SIGMA_MIN_MM = {_fmt(dist.get('sigma_min_mm', 8.0))};",
        f"constexpr double MCL_DIST_SIGMA_MAX_MM = {_fmt(dist.get('sigma_max_mm', 120.0))};",
        f"constexpr double MCL_DIST_CONF_SIGMA_SCALE = {_fmt(dist.get('conf_sigma_scale', 1.0))};",
        f"constexpr double MCL_DIST_W_HIT = {_fmt(dist.get('w_hit', 0.9))};",
        f"constexpr double MCL_DIST_W_RAND = {_fmt(dist.get('w_rand', 0.1))};",
        f"constexpr double MCL_DIST_W_SHORT = {_fmt(dist.get('w_short', 0.0))};",
        f"constexpr double MCL_DIST_W_MAX = {_fmt(dist.get('w_max', 0.0))};",
        f"constexpr double MCL_DIST_LAMBDA_SHORT = {_fmt(dist.get('lambda_short', 0.1))};",
        f"constexpr double MCL_DIST_MIN_SENSOR_WEIGHT = {_fmt(dist.get('min_sensor_weight', 1e-6))};",
        f"constexpr double MCL_DIST_GATE_MM = {_fmt(dist.get('gate_mm', 150.0))};",
        f"constexpr int MCL_DIST_GATE_MODE = {dist_gate_mode_val};",
        f"constexpr double MCL_DIST_GATE_PENALTY = {_fmt(dist.get('gate_penalty', 0.05))};",
        f"constexpr double MCL_DIST_GATE_REJECT_RATIO = {_fmt(dist.get('gate_reject_ratio', 0.9))};",
        f"constexpr double MCL_DIST_MAX_RANGE_MM = {_fmt(dist.get('max_range_mm', 2000.0))};",
        f"constexpr double MCL_DIST_MIN_RANGE_MM = {_fmt(dist.get('min_range_mm', 20.0))};",
        f"constexpr double MCL_DIST_CONFIDENCE_MIN = {_fmt(dist.get('confidence_min', 0.0))};",
        f"constexpr double MCL_DIST_OBJECT_SIZE_MIN = {_fmt(dist.get('object_size_min', 0.0))};",
        f"constexpr double MCL_DIST_OBJECT_SIZE_MAX = {_fmt(dist.get('object_size_max', 0.0))};",
        f"constexpr double MCL_DIST_INNOVATION_GATE_MM = {_fmt(dist.get('innovation_gate_mm', 0.0))};",
        f"constexpr double MCL_DIST_INNOVATION_GATE_MIN_CONF = {_fmt(dist.get('innovation_gate_min_conf', max(_to_float(conf.get('threshold', 0.0), 0.0), 0.65)))};",
        f"constexpr int MCL_DIST_MEDIAN_WINDOW = {int(dist.get('median_window', 3) or 0)};",
        f"constexpr int MCL_DIST_BATCH_SIZE = {int(dist.get('batch_size', 3) or 0)};",
        f"constexpr bool MCL_LF_IGNORE_MAX = {_bool(dist.get('lf_ignore_max', 0))};",
        f"constexpr bool MCL_DIST_USE_NO_OBJECT_INFO = {_bool(dist.get('use_no_object_info', 0))};",
        f"constexpr bool MCL_DIST_FOV_MULTI_RAY = {_bool(dist_fov_multi)};",
        f"constexpr int MCL_DIST_RAYS_PER_SENSOR = {dist_rays};",
        f"constexpr double MCL_DIST_FOV_HALF_DEG_NEAR = {_fmt(dist.get('fov_half_deg_near', 18.0))};",
        f"constexpr double MCL_DIST_FOV_HALF_DEG_FAR = {_fmt(dist.get('fov_half_deg_far', 12.0))};",
        f"constexpr double MCL_DIST_FOV_SWITCH_MM = {_fmt(dist.get('fov_switch_mm', 203.0))};",
        f"constexpr double MCL_DIST_RAYCAST_BUCKET_IN = {_fmt(dist.get('raycast_bucket_in', 12.0))};",
        f"constexpr int MCL_LF_MAX_BYTES = {int((dist.get('likelihood_field', {}) if isinstance(dist.get('likelihood_field', {}), dict) else {}).get('max_bytes', 262144) or 262144)};",
        "",
        f"constexpr bool MCL_USE_IMU = {_bool(imu.get('enabled', 1))};",
        f"constexpr double MCL_IMU_SIGMA_DEG = {_fmt(imu.get('sigma_deg', 1.0))};",
        f"constexpr bool MCL_IMU_CHECK_CALIBRATING = {imu_check_cal};",
        f"constexpr double MCL_IMU_FALLBACK_NOISE_SCALE = {_fmt(imu_fallback_noise_scale)};",
        "",
        f"constexpr bool MCL_USE_VISION = {_bool(vision.get('enabled', 0))};",
        f"constexpr double MCL_VISION_SIGMA_XY_IN = {_fmt(vision.get('sigma_xy_in', 2.0))};",
        f"constexpr double MCL_VISION_SIGMA_THETA_DEG = {_fmt(vision.get('sigma_theta_deg', 5.0))};",
        f"constexpr double MCL_VISION_CONFIDENCE_MIN = {_fmt(vision.get('confidence_min', 0.0))};",
        "",
        f"constexpr double MCL_RESAMPLE_THRESHOLD = {_fmt(resample.get('threshold', 0.5))};",
        f"constexpr bool MCL_RESAMPLE_ALWAYS = {_bool(resample.get('always', 0))};",
        f"constexpr int MCL_RESAMPLE_METHOD = {res_method_val};",
        f"constexpr double MCL_RESAMPLE_ROUGHEN_XY_IN = {_fmt(resample.get('roughen_xy_in', 0.03))};",
        f"constexpr double MCL_RESAMPLE_ROUGHEN_THETA_DEG = {_fmt(resample.get('roughen_theta_deg', 0.4))};",
        f"constexpr double MCL_RANDOM_INJECTION = {_fmt(mcl.get('random_injection', 0.01))};",
        f"constexpr bool MCL_AUGMENTED_ENABLED = {_bool(aug.get('enabled', 0))};",
        f"constexpr double MCL_ALPHA_SLOW = {_fmt(aug.get('alpha_slow', 0.001))};",
        f"constexpr double MCL_ALPHA_FAST = {_fmt(aug.get('alpha_fast', 0.1))};",
        f"constexpr bool MCL_RECOVERY_ENABLED = {_bool(recovery.get('enabled', 1))};",
        f"constexpr double MCL_RECOVERY_ESS_RATIO_MIN = {_fmt(recovery.get('ess_ratio_min', 0.2))};",
        f"constexpr int MCL_RECOVERY_ESS_STREAK = {int(recovery.get('ess_streak', 3) or 0)};",
        f"constexpr int MCL_RECOVERY_EKF_GATE_REJECT_STREAK = {int(recovery.get('ekf_gate_reject_streak', 3) or 0)};",
        f"constexpr int MCL_RECOVERY_COOLDOWN_MS = {int(recovery.get('cooldown_ms', 500) or 0)};",
        f"constexpr double MCL_RECOVERY_LOST_EXIT_CONFIDENCE = {_fmt(recovery.get('lost_exit_confidence', 0.55))};",
        f"constexpr int MCL_RECOVERY_LOST_EXIT_STREAK = {int(recovery.get('lost_exit_streak', 3) or 0)};",
        f"constexpr double MCL_RECOVERY_LOST_INJECTION_FRACTION = {_fmt(recovery.get('lost_injection_fraction', 0.15))};",
        f"constexpr int MCL_RECOVERY_LOST_FORCE_REINIT_MS = {int(recovery.get('lost_force_reinit_ms', 0) or 0)};",
        f"constexpr bool MCL_FRAME_SIGN_SELF_TEST_ENABLED = {_bool(frame_test.get('enabled', 1))};",
        f"constexpr double MCL_FRAME_SIGN_SELF_TEST_MIN_DELTA_DEG = {_fmt(frame_test.get('min_delta_deg', 2.0))};",
        f"constexpr int MCL_FRAME_SIGN_SELF_TEST_SAMPLES = {int(frame_test.get('samples', 6) or 0)};",
        f"constexpr int MCL_FRAME_SIGN_SELF_TEST_MISMATCH = {int(frame_test.get('mismatch_threshold', 5) or 0)};",
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
        f"constexpr bool MCL_MODE_SPLIT_ENABLED = {_bool(mode_split.get('enabled', 1))};",
        f"constexpr double MCL_MODE_SPLIT_CONF_MAX = {_fmt(mode_split.get('conf_max', 0.55))};",
        f"constexpr double MCL_MODE_SPLIT_MIN_SEPARATION_IN = {_fmt(mode_split.get('min_separation_in', 8.0))};",
        f"constexpr double MCL_MODE_SPLIT_MIN_MASS = {_fmt(mode_split.get('min_mass', 0.15))};",
        "",
        f"constexpr bool MCL_CORR_ENABLED = {_bool(correction.get('enabled', 1))};",
        f"constexpr double MCL_CORR_MIN_CONF = {_fmt(correction.get('min_confidence', 0.6))};",
        f"constexpr double MCL_CORR_MAX_TRANS_JUMP_IN = {_fmt(correction.get('max_trans_jump_in', 8.0))};",
        f"constexpr double MCL_CORR_MAX_THETA_JUMP_DEG = {_fmt(correction.get('max_theta_jump_deg', 15.0))};",
        f"constexpr double MCL_CORR_ALPHA_MIN = {_fmt(correction.get('alpha_min', 0.03))};",
        f"constexpr double MCL_CORR_ALPHA_MAX = {_fmt(correction.get('alpha_max', 0.12))};",
        f"constexpr bool MCL_CORR_SAFE_WINDOW_ENABLED = {_bool(correction.get('safe_window_enabled', 1))};",
        f"constexpr double MCL_CORR_SAFE_MAX_SPEED_IN_S = {_fmt(correction.get('safe_max_speed_in_s', 8.0))};",
        f"constexpr double MCL_CORR_SAFE_MAX_TURN_DEG_S = {_fmt(correction.get('safe_max_turn_deg_s', 60.0))};",
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
        f"constexpr double MCL_EKF_MCL_MAHALANOBIS_GATE = {_fmt(ekf.get('mcl_mahalanobis_gate', 11.34))};",
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
    meta = _compute_map_blob_meta(cfg)
    lines = []
    lines.append("// mcl_map_data.h (generated)")
    lines.append("#pragma once")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("// Likelihood-field distance grids used by distance sensors.")
    lines.append("extern const uint16_t MAP_DIST_FIELD[];")
    lines.append("extern const uint16_t MAP_DIST_FIELD_PERIM[];")
    lines.append("extern const uint16_t MAP_DIST_FIELD_OBJ[];")
    lines.append("")
    lines.append(f"constexpr uint32_t MCL_MAP_DATA_HASH32 = 0x{meta.get('map_hash32', '0')}u;")
    lines.append(f"constexpr uint32_t MCL_MAP_PROJECT_HASH32 = 0x{meta.get('project_hash32', '0')}u;")
    lines.append(f"constexpr int MCL_MAP_GRID_W = {int(meta.get('grid_w', 1))};")
    lines.append(f"constexpr int MCL_MAP_GRID_H = {int(meta.get('grid_h', 1))};")
    lines.append(f"constexpr int MCL_MAP_DIST_FIELD_LEN = {int(meta.get('dist_len', 1))};")
    perim_len = len(meta.get("df_perim", {}).get("grid_mm_u16", [])) if isinstance(meta.get("df_perim"), dict) else 1
    obj_len = len(meta.get("df_obj", {}).get("grid_mm_u16", [])) if isinstance(meta.get("df_obj"), dict) else 1
    lines.append(f"constexpr int MCL_MAP_DIST_FIELD_PERIM_LEN = {max(1, int(perim_len))};")
    lines.append(f"constexpr int MCL_MAP_DIST_FIELD_OBJ_LEN = {max(1, int(obj_len))};")
    lines.append("")
    return "\n".join(lines) + "\n"


def build_mcl_map_data_cpp(cfg: dict) -> str:
    """Build mcl map data cpp."""
    meta = _compute_map_blob_meta(cfg)
    df_all = meta.get("df_all")
    df_perim = meta.get("df_perim")
    df_obj = meta.get("df_obj")

    lines = []
    lines.append("// mcl_map_data.cpp (generated)")
    lines.append('#include "mcl_map_data.h"')
    lines.append("")

    def emit(df, name: str):
        """Handle emit."""
        if not df or not df.get("grid_mm_u16"):
            lines.append(f"extern const uint16_t {name}[] = {{0}};")
            lines.append("")
            return
        lines.append(f"extern const uint16_t {name}[] = {{")
        per_line = 64
        buf = []
        for v in df["grid_mm_u16"]:
            buf.append(str(int(v)))
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
        "// Internal MCL frame: +x forward, +y left; heading CW+ with 0=left, 90=forward.",
        "// Use external* adapters below for exported conventions (e.g., 0=forward).",
        "class ProsMCL {",
        " public:",
        "  using FieldPoseProvider = bool (*)(MCLPose* out_pose, void* user);",
        "  ProsMCL(int imu_port, const std::vector<int>& dist_ports);",
        "  void start(unsigned seed, double initial_heading_deg);",
        "  void startEasy(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg);",
        "  void stop();",
        "  // Robot-frame deltas: +x forward, +y left, +theta clockwise (deg).",
        "  // Do not pass field-frame deltas here.",
        "  // Ignored while a FieldPoseProvider is active (feed modes are exclusive).",
        "  void setOdomDelta(double dx_in, double dy_in, double dtheta_deg);",
        "  // Optional: provide field pose samples and let runtime compute odom deltas automatically.",
        "  // Callback must return internal MCL frame pose (use externalPoseToMCL for library poses).",
        "  void setFieldPoseProvider(FieldPoseProvider provider, void* user = nullptr);",
        "  void clearFieldPoseProvider();",
        "  void setPose(double x_in, double y_in, double theta_deg);",
        "  // External convention adapters (selected by mcl.interop.pose_convention).",
        "  // - atticus: heading CW+, 0=left, 90=forward",
        "  // - cw_zero_forward: heading CW+, 0=forward",
        "  // - ccw_zero_forward: heading CCW+, 0=forward",
        "  static double externalHeadingToMCL(double heading_deg);",
        "  static double mclHeadingToExternal(double heading_deg);",
        "  static MCLPose externalPoseToMCL(const MCLPose& pose);",
        "  static MCLPose mclPoseToExternal(const MCLPose& pose);",
        "  struct FrameSanityResult {",
        "    bool current_pass;",
        "    bool translation_pass;",
        "    bool heading_pass;",
        "    double current_dx_in;",
        "    double current_dy_in;",
        "    double current_dtheta_deg;",
        "    int recommended_pose_convention;",
        "    int recommended_swap_xy;",
        "    int recommended_invert_x;",
        "    int recommended_invert_y;",
        "    double recommended_score;",
        "  };",
        "  // Evaluate adapter settings from a simple on-robot test:",
        "  // start(heading ~0 external) -> drive forward -> turn clockwise.",
        "  // Inputs are external-frame poses.",
        "  static FrameSanityResult runFrameSanityCheck(",
        "      const MCLPose& start_external,",
        "      const MCLPose& after_forward_external,",
        "      const MCLPose& after_turn_external,",
        "      double forward_target_in = 12.0,",
        "      double turn_target_deg = 90.0);",
        "  void startExternal(unsigned seed, double initial_heading_deg);",
        "  void startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg);",
        "  void setPoseExternal(double x_in, double y_in, double theta_deg);",
        "  MCLPose getPoseExternal() const;",
        "  MCLPose getFusedPoseExternal() const;",
        "  void set_pose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }",
        "  void resetPose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }",
        "  void reset_pose(double x_in, double y_in, double theta_deg) { setPose(x_in, y_in, theta_deg); }",
        "  void kidnap() { requestRelocalize(); }",
        "  MCLPose getPose() const;",
        "  MCLPose getFusedPose() const;",
        "  void freezeLocalization(bool freeze);",
        "  bool isLocalizationFrozen() const;",
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
        "    double mcl_peakedness;",
        "    double mcl_ess_ratio;",
        "    double mcl_neff;",
        "    double ekf_pxx;",
        "    double ekf_pyy;",
        "    double ekf_pxy;",
        "    double ekf_ptt;",
        "    std::uint32_t dist_used_mask;",
        "    std::uint32_t event_flags;",
        "    double dist_meas_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "    double dist_exp_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "    int dist_errno[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
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
        "  bool map_blob_valid_;",
        "  std::atomic_bool relocalize_requested_;",
        "  std::atomic_bool localization_frozen_;",
        "  FieldPoseProvider pose_provider_;",
        "  void* pose_provider_user_;",
        "  MCLPose provider_last_pose_;",
        "  bool provider_last_valid_;",
        "  double odom_dx_;",
        "  double odom_dy_;",
        "  double odom_dth_;",
        "  int odom_fault_noise_cycles_;",
        "  double last_speed_in_s_;",
        "  double last_turn_deg_s_;",
        "  double imu_last_rotation_;",
        "  bool imu_last_rotation_valid_;",
        "  MCLPose pose_;",
        "  MCLPose fused_pose_;",
        "  MCLPose pose_buf_[2];",
        "  MCLPose fused_pose_buf_[2];",
        "  std::atomic_int pose_buf_idx_;",
        "  EKFState ekf_;",
        "  bool mcl_ambiguous_;",
        "  int mcl_recover_good_count_;",
        "  int recovery_low_ess_streak_;",
        "  int recovery_gate_reject_streak_;",
        "  std::uint32_t recovery_cooldown_until_ms_;",
        "  bool recovery_lost_;",
        "  int recovery_exit_streak_;",
        "  std::uint32_t recovery_lost_since_ms_;",
        "  int sign_selftest_samples_;",
        "  int sign_selftest_mismatch_;",
        "  bool sign_selftest_done_;",
        "  std::uint32_t ekf_intervene_cooldown_until_ms_;",
        "  mutable std::uint32_t last_event_flags_;",
        "  bool sensor_batch_active_;",
        "  int sensor_batch_count_;",
        "  int sensor_batch_cursor_;",
        "  bool sensor_batch_have_imu_heading_;",
        "  double sensor_batch_imu_heading_;",
        "  bool sensor_batch_imu_applied_;",
        "  double sensor_batch_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  double sensor_batch_conf_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  double sensor_batch_obj_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int sensor_batch_errno_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int sensor_batch_conf_meaningful_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int sensor_batch_obj_valid_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  double sensor_batch_weights_backup_[MCL_PARTICLE_CAPACITY];",
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
    lines.append("#include <cerrno>")
    lines.append("#include <cmath>")
    lines.append("")
    lines.append("namespace {")
    lines.append("constexpr std::uint32_t MCL_EVENT_MCL_EKF_APPLIED = 1u << 0;")
    lines.append("constexpr std::uint32_t MCL_EVENT_ODOM_CORR_APPLIED = 1u << 1;")
    lines.append("constexpr std::uint32_t MCL_EVENT_RECOVERY_ACTIVE = 1u << 3;")
    lines.append("constexpr std::uint32_t MCL_EVENT_FRAME_SIGN_MISMATCH = 1u << 4;")
    lines.append("constexpr std::uint32_t MCL_EVENT_MAP_BLOB_MISMATCH = 1u << 5;")
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
    lines.append("static bool map_blob_sanity_ok() {")
    lines.append("  if (MCL_MAP_GRID_W <= 0 || MCL_MAP_GRID_H <= 0) return false;")
    lines.append("  const long expected = static_cast<long>(MCL_MAP_GRID_W) * static_cast<long>(MCL_MAP_GRID_H);")
    lines.append("  if (expected <= 0) return false;")
    lines.append("  if (MCL_MAP_DIST_FIELD_LEN < expected) return false;")
    lines.append("  if (MCL_EXPECTED_MAP_GRID_W > 0 && MCL_MAP_GRID_W != MCL_EXPECTED_MAP_GRID_W) return false;")
    lines.append("  if (MCL_EXPECTED_MAP_GRID_H > 0 && MCL_MAP_GRID_H != MCL_EXPECTED_MAP_GRID_H) return false;")
    lines.append("  if (MCL_EXPECTED_MAP_DIST_LEN > 0 && MCL_MAP_DIST_FIELD_LEN != MCL_EXPECTED_MAP_DIST_LEN) return false;")
    lines.append("  if (MCL_EXPECTED_MAP_HASH32 != 0u && MCL_MAP_DATA_HASH32 != MCL_EXPECTED_MAP_HASH32) return false;")
    lines.append("  if (MCL_EXPECTED_MAP_PROJECT_HASH32 != 0u && MCL_MAP_PROJECT_HASH32 != MCL_EXPECTED_MAP_PROJECT_HASH32) return false;")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("static double external_heading_to_mcl_mode(double heading_deg, int pose_convention) {")
    lines.append("  double h = wrap_deg(heading_deg);")
    lines.append("  if (pose_convention == MCL_POSE_CONVENTION_CW_ZERO_FORWARD) {")
    lines.append("    return wrap_deg(h + 90.0);")
    lines.append("  }")
    lines.append("  if (pose_convention == MCL_POSE_CONVENTION_CCW_ZERO_FORWARD) {")
    lines.append("    return wrap_deg(90.0 - h);")
    lines.append("  }")
    lines.append("  return h;")
    lines.append("}")
    lines.append("")
    lines.append("static double mcl_heading_to_external_mode(double heading_deg, int pose_convention) {")
    lines.append("  double h = wrap_deg(heading_deg);")
    lines.append("  if (pose_convention == MCL_POSE_CONVENTION_CW_ZERO_FORWARD) {")
    lines.append("    return wrap_deg(h - 90.0);")
    lines.append("  }")
    lines.append("  if (pose_convention == MCL_POSE_CONVENTION_CCW_ZERO_FORWARD) {")
    lines.append("    return wrap_deg(90.0 - h);")
    lines.append("  }")
    lines.append("  return h;")
    lines.append("}")
    lines.append("")
    lines.append("static void heading_to_unit(double heading_deg, double* dx, double* dy) {")
    lines.append("  double th = heading_deg * (3.14159265358979323846 / 180.0);")
    lines.append("  if (dx) *dx = std::sin(th);")
    lines.append("  if (dy) *dy = std::cos(th);")
    lines.append("}")
    lines.append("")
    lines.append("static double unit_to_heading(double dx, double dy) {")
    lines.append("  if (!std::isfinite(dx) || !std::isfinite(dy)) return 0.0;")
    lines.append("  if (std::fabs(dx) < 1e-12 && std::fabs(dy) < 1e-12) return 0.0;")
    lines.append("  double h = std::atan2(dx, dy) * (180.0 / 3.14159265358979323846);")
    lines.append("  return wrap_deg(h);")
    lines.append("}")
    lines.append("")
    lines.append("static void external_xy_to_mcl_mode(double ex, double ey, int swap_xy, int invert_x, int invert_y, double* mx, double* my) {")
    lines.append("  double x = ex;")
    lines.append("  double y = ey;")
    lines.append("  if (swap_xy != 0) {")
    lines.append("    double t = x;")
    lines.append("    x = y;")
    lines.append("    y = t;")
    lines.append("  }")
    lines.append("  if (invert_x != 0) x = -x;")
    lines.append("  if (invert_y != 0) y = -y;")
    lines.append("  if (mx) *mx = x;")
    lines.append("  if (my) *my = y;")
    lines.append("}")
    lines.append("")
    lines.append("static void mcl_xy_to_external_mode(double mx, double my, int swap_xy, int invert_x, int invert_y, double* ex, double* ey) {")
    lines.append("  double x = mx;")
    lines.append("  double y = my;")
    lines.append("  if (invert_y != 0) y = -y;")
    lines.append("  if (invert_x != 0) x = -x;")
    lines.append("  if (swap_xy != 0) {")
    lines.append("    double t = x;")
    lines.append("    x = y;")
    lines.append("    y = t;")
    lines.append("  }")
    lines.append("  if (ex) *ex = x;")
    lines.append("  if (ey) *ey = y;")
    lines.append("}")
    lines.append("")
    lines.append("static double external_heading_to_mcl_full(double heading_deg, int pose_convention, int swap_xy, int invert_x, int invert_y) {")
    lines.append("  double hx = 0.0, hy = 0.0;")
    lines.append("  heading_to_unit(external_heading_to_mcl_mode(heading_deg, pose_convention), &hx, &hy);")
    lines.append("  external_xy_to_mcl_mode(hx, hy, swap_xy, invert_x, invert_y, &hx, &hy);")
    lines.append("  return unit_to_heading(hx, hy);")
    lines.append("}")
    lines.append("")
    lines.append("static double mcl_heading_to_external_full(double heading_deg, int pose_convention, int swap_xy, int invert_x, int invert_y) {")
    lines.append("  double hx = 0.0, hy = 0.0;")
    lines.append("  heading_to_unit(heading_deg, &hx, &hy);")
    lines.append("  mcl_xy_to_external_mode(hx, hy, swap_xy, invert_x, invert_y, &hx, &hy);")
    lines.append("  return mcl_heading_to_external_mode(unit_to_heading(hx, hy), pose_convention);")
    lines.append("}")
    lines.append("")
    lines.append("static double external_heading_to_mcl(double heading_deg) {")
    lines.append("  return external_heading_to_mcl_full(")
    lines.append("      heading_deg,")
    lines.append("      MCL_EXPORT_POSE_CONVENTION,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0);")
    lines.append("}")
    lines.append("")
    lines.append("static double mcl_heading_to_external(double heading_deg) {")
    lines.append("  return mcl_heading_to_external_full(")
    lines.append("      heading_deg,")
    lines.append("      MCL_EXPORT_POSE_CONVENTION,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0);")
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
    lines.append("template <typename DistanceT>")
    lines.append("static auto distance_read_mm_impl(DistanceT& d, int) -> decltype(static_cast<int>(d.get())) {")
    lines.append("  return d.get();")
    lines.append("}")
    lines.append("")
    lines.append("template <typename DistanceT>")
    lines.append("static auto distance_read_mm_impl(DistanceT& d, long) -> decltype(static_cast<int>(d.get_distance())) {")
    lines.append("  return d.get_distance();")
    lines.append("}")
    lines.append("")
    lines.append("static int distance_read_mm_impl(...) {")
    lines.append("  return PROS_ERR;")
    lines.append("}")
    lines.append("")
    lines.append("static int distance_read_mm(pros::Distance& d) {")
    lines.append("  return distance_read_mm_impl(d, 0);")
    lines.append("}")
    lines.append("")
    lines.append("static bool imu_read_heading_deg(pros::Imu& imu, double* out_heading) {")
    lines.append("  if (!out_heading) return false;")
    lines.append("  errno = 0;")
    lines.append("  double h = static_cast<double>(imu.get_heading());")
    lines.append("  if (h == PROS_ERR_F || !std::isfinite(h)) return false;")
    lines.append("  *out_heading = wrap_deg(h);")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("static bool imu_read_rotation_deg(pros::Imu& imu, double* out_rotation) {")
    lines.append("  if (!out_rotation) return false;")
    lines.append("  errno = 0;")
    lines.append("  double rot = static_cast<double>(imu.get_rotation());")
    lines.append("  if (rot == PROS_ERR_F || !std::isfinite(rot)) return false;")
    lines.append("  *out_rotation = rot;")
    lines.append("  return true;")
    lines.append("}")
    lines.append("")
    lines.append("template <typename ImuT>")
    lines.append("static auto imu_is_calibrating_impl(ImuT& imu, int) -> decltype(static_cast<bool>(imu.is_calibrating())) {")
    lines.append("  return imu.is_calibrating();")
    lines.append("}")
    lines.append("")
    lines.append("static bool imu_is_calibrating_impl(...) {")
    lines.append("  return false;")
    lines.append("}")
    lines.append("")
    lines.append("static bool imu_is_calibrating(pros::Imu& imu) {")
    lines.append("  if (!MCL_IMU_CHECK_CALIBRATING) return false;")
    lines.append("  return imu_is_calibrating_impl(imu, 0);")
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
    lines.append("static void ekf_predict(ProsMCL::EKFState& ekf, double dx_in, double dy_in, double dtheta_deg, double noise_scale) {")
    lines.append("  if (!ekf.initialized) return;")
    lines.append("  double ns = std::isfinite(noise_scale) ? std::fabs(noise_scale) : 1.0;")
    lines.append("  if (ns < 1.0) ns = 1.0;")
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
    lines.append("  double qscale = ns * ns;")
    lines.append("  double Q[3][3] = {")
    lines.append("    {qdx * qdx * qscale, 0.0, 0.0},")
    lines.append("    {0.0, qdy * qdy * qscale, 0.0},")
    lines.append("    {0.0, 0.0, qth * qth * qscale}")
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
    lines.append("static bool ekf_update_mcl(ProsMCL::EKFState& ekf, const MCLPose& pose, const double mcl_cov[3][3], double confidence, bool* out_gate_reject = nullptr) {")
    lines.append("  if (out_gate_reject) *out_gate_reject = false;")
    lines.append("  if (!ekf.initialized) return false;")
    lines.append("  if (confidence < MCL_EKF_MCL_MIN_CONF) return false;")
    lines.append("  double nu_xy = std::sqrt((pose.x - ekf.x) * (pose.x - ekf.x) + (pose.y - ekf.y) * (pose.y - ekf.y));")
    lines.append("  double nu_th = std::fabs(angle_diff_deg(pose.theta, ekf.theta));")
    lines.append("  if (MCL_EKF_MCL_INNOVATION_GATE_XY_IN > 0.0 && nu_xy > MCL_EKF_MCL_INNOVATION_GATE_XY_IN) return false;")
    lines.append("  if (MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG > 0.0 && nu_th > MCL_EKF_MCL_INNOVATION_GATE_THETA_DEG) return false;")
    lines.append("  double sx = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[0][0] : 0.0));")
    lines.append("  double sy = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[1][1] : 0.0));")
    lines.append("  double sth = std::sqrt(std::max(0.0, mcl_cov ? mcl_cov[2][2] : 0.0));")
    lines.append("  if (!std::isfinite(sx) || sx <= 0.0) sx = MCL_EKF_MCL_SIGMA_X_MAX_IN;")
    lines.append("  if (!std::isfinite(sy) || sy <= 0.0) sy = MCL_EKF_MCL_SIGMA_Y_MAX_IN;")
    lines.append("  if (!std::isfinite(sth) || sth <= 0.0) sth = MCL_EKF_MCL_SIGMA_THETA_MAX_DEG;")
    lines.append("  if (sx < MCL_EKF_MCL_SIGMA_X_MIN_IN) sx = MCL_EKF_MCL_SIGMA_X_MIN_IN;")
    lines.append("  if (sx > MCL_EKF_MCL_SIGMA_X_MAX_IN) sx = MCL_EKF_MCL_SIGMA_X_MAX_IN;")
    lines.append("  if (sy < MCL_EKF_MCL_SIGMA_Y_MIN_IN) sy = MCL_EKF_MCL_SIGMA_Y_MIN_IN;")
    lines.append("  if (sy > MCL_EKF_MCL_SIGMA_Y_MAX_IN) sy = MCL_EKF_MCL_SIGMA_Y_MAX_IN;")
    lines.append("  if (sth < MCL_EKF_MCL_SIGMA_THETA_MIN_DEG) sth = MCL_EKF_MCL_SIGMA_THETA_MIN_DEG;")
    lines.append("  if (sth > MCL_EKF_MCL_SIGMA_THETA_MAX_DEG) sth = MCL_EKF_MCL_SIGMA_THETA_MAX_DEG;")
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
    lines.append("  double nu[3] = {pose.x - ekf.x, pose.y - ekf.y, angle_diff_deg(pose.theta, ekf.theta)};")
    lines.append("  if (MCL_EKF_MCL_MAHALANOBIS_GATE > 0.0) {")
    lines.append("    double md2 =")
    lines.append("      nu[0] * (S_inv[0][0] * nu[0] + S_inv[0][1] * nu[1] + S_inv[0][2] * nu[2]) +")
    lines.append("      nu[1] * (S_inv[1][0] * nu[0] + S_inv[1][1] * nu[1] + S_inv[1][2] * nu[2]) +")
    lines.append("      nu[2] * (S_inv[2][0] * nu[0] + S_inv[2][1] * nu[1] + S_inv[2][2] * nu[2]);")
    lines.append("    if (md2 > MCL_EKF_MCL_MAHALANOBIS_GATE) {")
    lines.append("      if (out_gate_reject) *out_gate_reject = true;")
    lines.append("      return false;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  double K[3][3];")
    lines.append("  mat3_mul(ekf.P, S_inv, K);")
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
    lines.append("static bool ekf_fusion_update(ProsMCL::EKFState& ekf, const MCLPose& pose, const double mcl_cov[3][3], double conf, std::uint32_t now_ms,")
    lines.append("                              bool& mcl_ambiguous, int& mcl_recover_good_count,")
    lines.append("                              std::uint32_t& cooldown_until_ms, bool* out_gate_reject = nullptr) {")
    lines.append("  if (out_gate_reject) *out_gate_reject = false;")
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
    lines.append("    return ekf_update_mcl(ekf, pose, mcl_cov, conf, out_gate_reject);")
    lines.append("  }")
    lines.append("  if (fusion_mode == 1 || fusion_mode == 2) {")
    lines.append("    if (intervene_ok) {")
    lines.append("      if (use_reset && hard_jump) {")
    lines.append("        ekf_reset(ekf, pose.x, pose.y, pose.theta);")
    lines.append("        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);")
    lines.append("        return true;")
    lines.append("      }")
    lines.append("      bool gate_reject = false;")
    lines.append("      if (ekf_update_mcl(ekf, pose, mcl_cov, conf, &gate_reject)) {")
    lines.append("        if (MCL_EKF_INTERVENE_COOLDOWN_MS > 0) cooldown_until_ms = now_ms + static_cast<std::uint32_t>(MCL_EKF_INTERVENE_COOLDOWN_MS);")
    lines.append("        return true;")
    lines.append("      }")
    lines.append("      if (out_gate_reject && gate_reject) *out_gate_reject = true;")
    lines.append("    }")
    lines.append("    if (fusion_mode == 2 && track_ok) {")
    lines.append("      return ekf_update_mcl(ekf, pose, mcl_cov, conf, out_gate_reject);")
    lines.append("    }")
    lines.append("    return false;")
    lines.append("  }")
    lines.append("  return false;")
    lines.append("}")
    lines.append("}  // namespace")
    lines.append("")
    lines.append("ProsMCL::ProsMCL(int imu_port, const std::vector<int>& dist_ports)")
    lines.append("  : task_(nullptr), running_(false), task_done_(true), map_blob_valid_(true), relocalize_requested_(false), localization_frozen_(false), pose_provider_(nullptr), pose_provider_user_(nullptr), provider_last_pose_{0.0, 0.0, 0.0}, provider_last_valid_(false), odom_dx_(0.0), odom_dy_(0.0), odom_dth_(0.0), odom_fault_noise_cycles_(0), last_speed_in_s_(0.0), last_turn_deg_s_(0.0), imu_last_rotation_(0.0), imu_last_rotation_valid_(false), pose_{0.0, 0.0, 0.0}, fused_pose_{0.0, 0.0, 0.0}, pose_buf_{{0.0,0.0,0.0},{0.0,0.0,0.0}}, fused_pose_buf_{{0.0,0.0,0.0},{0.0,0.0,0.0}}, pose_buf_idx_(0), mcl_ambiguous_(false), mcl_recover_good_count_(0), recovery_low_ess_streak_(0), recovery_gate_reject_streak_(0), recovery_cooldown_until_ms_(0), recovery_lost_(false), recovery_exit_streak_(0), recovery_lost_since_ms_(0), sign_selftest_samples_(0), sign_selftest_mismatch_(0), sign_selftest_done_(false), ekf_intervene_cooldown_until_ms_(0), last_event_flags_(0), sensor_batch_active_(false), sensor_batch_count_(0), sensor_batch_cursor_(0), sensor_batch_have_imu_heading_(false), sensor_batch_imu_heading_(0.0), sensor_batch_imu_applied_(false) {")
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
    lines.append("    sensor_batch_mm_[i] = -1.0;")
    lines.append("    sensor_batch_conf_[i] = 0.0;")
    lines.append("    sensor_batch_obj_[i] = 0.0;")
    lines.append("    sensor_batch_errno_[i] = 0;")
    lines.append("    sensor_batch_conf_meaningful_[i] = 0;")
    lines.append("    sensor_batch_obj_valid_[i] = 0;")
    lines.append("  }")
    lines.append("  for (int i = 0; i < MCL_PARTICLE_CAPACITY; ++i) sensor_batch_weights_backup_[i] = 0.0;")
    lines.append("  map_blob_valid_ = map_blob_sanity_ok();")
    lines.append("  if (!map_blob_valid_) last_event_flags_ |= MCL_EVENT_MAP_BLOB_MISMATCH;")
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
    lines.append("  imu_last_rotation_valid_ = false;")
    lines.append("  if (imu_) {")
    lines.append("    imu_->reset();")
    lines.append("    errno = 0;")
    lines.append("    imu_->set_heading(initial_heading_deg);")
    lines.append("    double rot0 = 0.0;")
    lines.append("    if (imu_read_rotation_deg(*imu_, &rot0)) {")
    lines.append("      imu_last_rotation_ = rot0;")
    lines.append("      imu_last_rotation_valid_ = true;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  mcl_.initGlobal();")
    lines.append("  {")
    lines.append("    ScopedMutex lk(mu_);")
    lines.append("    pose_ = mcl_.estimate();")
    lines.append("    ekf_.initialized = false;")
    lines.append("    fused_pose_ = pose_;")
    lines.append("    mcl_ambiguous_ = false;")
    lines.append("    mcl_recover_good_count_ = 0;")
    lines.append("    recovery_low_ess_streak_ = 0;")
    lines.append("    recovery_gate_reject_streak_ = 0;")
    lines.append("    recovery_cooldown_until_ms_ = 0;")
    lines.append("    recovery_lost_ = false;")
    lines.append("    recovery_exit_streak_ = 0;")
    lines.append("    recovery_lost_since_ms_ = 0;")
    lines.append("    mcl_.setForcedInjectionFraction(0.0);")
    lines.append("    sign_selftest_samples_ = 0;")
    lines.append("    sign_selftest_mismatch_ = 0;")
    lines.append("    sign_selftest_done_ = false;")
    lines.append("    ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("    last_speed_in_s_ = 0.0;")
    lines.append("    last_turn_deg_s_ = 0.0;")
    lines.append("    localization_frozen_.store(false);")
    lines.append("    int next_idx = 1 - pose_buf_idx_.load();")
    lines.append("    pose_buf_[next_idx] = pose_;")
    lines.append("    fused_pose_buf_[next_idx] = fused_pose_;")
    lines.append("    pose_buf_idx_.store(next_idx);")
    lines.append("  }")
    lines.append("  task_done_.store(false);")
    lines.append("  running_.store(true);")
    lines.append("  task_ = new pros::Task([this] { loop(); });")
    lines.append("}")
    lines.append("")
    lines.append("double ProsMCL::externalHeadingToMCL(double heading_deg) {")
    lines.append("  return external_heading_to_mcl(heading_deg);")
    lines.append("}")
    lines.append("")
    lines.append("double ProsMCL::mclHeadingToExternal(double heading_deg) {")
    lines.append("  return mcl_heading_to_external(heading_deg);")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::externalPoseToMCL(const MCLPose& pose) {")
    lines.append("  double mx = pose.x;")
    lines.append("  double my = pose.y;")
    lines.append("  external_xy_to_mcl_mode(")
    lines.append("      pose.x, pose.y,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0,")
    lines.append("      &mx, &my);")
    lines.append("  return {mx, my, external_heading_to_mcl(pose.theta)};")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::mclPoseToExternal(const MCLPose& pose) {")
    lines.append("  double ex = pose.x;")
    lines.append("  double ey = pose.y;")
    lines.append("  mcl_xy_to_external_mode(")
    lines.append("      pose.x, pose.y,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0,")
    lines.append("      &ex, &ey);")
    lines.append("  return {ex, ey, mcl_heading_to_external(pose.theta)};")
    lines.append("}")
    lines.append("")
    lines.append("ProsMCL::FrameSanityResult ProsMCL::runFrameSanityCheck(")
    lines.append("    const MCLPose& start_external,")
    lines.append("    const MCLPose& after_forward_external,")
    lines.append("    const MCLPose& after_turn_external,")
    lines.append("    double forward_target_in,")
    lines.append("    double turn_target_deg) {")
    lines.append("  FrameSanityResult out{};")
    lines.append("  double target_fwd = std::fabs(forward_target_in);")
    lines.append("  if (!std::isfinite(target_fwd) || target_fwd < 1e-3) target_fwd = 12.0;")
    lines.append("  double target_turn = std::fabs(turn_target_deg);")
    lines.append("  if (!std::isfinite(target_turn) || target_turn < 1e-3) target_turn = 90.0;")
    lines.append("")
    lines.append("  double dx_ext = after_forward_external.x - start_external.x;")
    lines.append("  double dy_ext = after_forward_external.y - start_external.y;")
    lines.append("  double h1_ext = after_forward_external.theta;")
    lines.append("  double h2_ext = after_turn_external.theta;")
    lines.append("")
    lines.append("  external_xy_to_mcl_mode(")
    lines.append("      dx_ext, dy_ext,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0,")
    lines.append("      &out.current_dx_in, &out.current_dy_in);")
    lines.append("  double h1_cur = external_heading_to_mcl_full(")
    lines.append("      h1_ext, MCL_EXPORT_POSE_CONVENTION,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0);")
    lines.append("  double h2_cur = external_heading_to_mcl_full(")
    lines.append("      h2_ext, MCL_EXPORT_POSE_CONVENTION,")
    lines.append("      MCL_EXPORT_SWAP_XY ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_X ? 1 : 0,")
    lines.append("      MCL_EXPORT_INVERT_Y ? 1 : 0);")
    lines.append("  out.current_dtheta_deg = angle_diff_deg(h2_cur, h1_cur);")
    lines.append("")
    lines.append("  const double trans_tol = std::max(2.0, target_fwd * 0.35);")
    lines.append("  const double heading_tol = 20.0;")
    lines.append("  out.translation_pass = (out.current_dx_in > 0.0) && (std::fabs(out.current_dy_in) <= trans_tol);")
    lines.append("  out.heading_pass = (out.current_dtheta_deg > 0.0) && (std::fabs(out.current_dtheta_deg - target_turn) <= heading_tol);")
    lines.append("  out.current_pass = out.translation_pass && out.heading_pass;")
    lines.append("")
    lines.append("  double best_score = 1e300;")
    lines.append("  int best_conv = MCL_EXPORT_POSE_CONVENTION;")
    lines.append("  int best_swap = MCL_EXPORT_SWAP_XY ? 1 : 0;")
    lines.append("  int best_inv_x = MCL_EXPORT_INVERT_X ? 1 : 0;")
    lines.append("  int best_inv_y = MCL_EXPORT_INVERT_Y ? 1 : 0;")
    lines.append("  for (int conv = 0; conv <= 2; ++conv) {")
    lines.append("    for (int swap_xy = 0; swap_xy <= 1; ++swap_xy) {")
    lines.append("      for (int inv_x = 0; inv_x <= 1; ++inv_x) {")
    lines.append("        for (int inv_y = 0; inv_y <= 1; ++inv_y) {")
    lines.append("          double dx_m = 0.0, dy_m = 0.0;")
    lines.append("          external_xy_to_mcl_mode(dx_ext, dy_ext, swap_xy, inv_x, inv_y, &dx_m, &dy_m);")
    lines.append("          double hh1 = external_heading_to_mcl_full(h1_ext, conv, swap_xy, inv_x, inv_y);")
    lines.append("          double hh2 = external_heading_to_mcl_full(h2_ext, conv, swap_xy, inv_x, inv_y);")
    lines.append("          double dth = angle_diff_deg(hh2, hh1);")
    lines.append("          double mag = std::sqrt(dx_m * dx_m + dy_m * dy_m);")
    lines.append("          double score = std::fabs(dy_m);")
    lines.append("          if (dx_m < 0.0) score += 3.0 * std::fabs(dx_m);")
    lines.append("          score += std::fabs(mag - target_fwd);")
    lines.append("          if (dth < 0.0) score += 3.0 * std::fabs(dth);")
    lines.append("          score += 0.25 * std::fabs(dth - target_turn);")
    lines.append("          if (score < best_score) {")
    lines.append("            best_score = score;")
    lines.append("            best_conv = conv;")
    lines.append("            best_swap = swap_xy;")
    lines.append("            best_inv_x = inv_x;")
    lines.append("            best_inv_y = inv_y;")
    lines.append("          }")
    lines.append("        }")
    lines.append("      }")
    lines.append("    }")
    lines.append("  }")
    lines.append("")
    lines.append("  out.recommended_pose_convention = best_conv;")
    lines.append("  out.recommended_swap_xy = best_swap;")
    lines.append("  out.recommended_invert_x = best_inv_x;")
    lines.append("  out.recommended_invert_y = best_inv_y;")
    lines.append("  out.recommended_score = best_score;")
    lines.append("  return out;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::startExternal(unsigned seed, double initial_heading_deg) {")
    lines.append("  start(seed, external_heading_to_mcl(initial_heading_deg));")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg) {")
    lines.append("  start(seed, external_heading_to_mcl(initial_heading_deg));")
    lines.append("  MCLPose p = externalPoseToMCL({start_x_in, start_y_in, start_theta_deg});")
    lines.append("  setPose(p.x, p.y, p.theta);")
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
    lines.append("  if (pose_provider_ != nullptr) return;")
    lines.append("  odom_dx_ += dx_in;")
    lines.append("  odom_dy_ += dy_in;")
    lines.append("  odom_dth_ += dtheta_deg;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setFieldPoseProvider(FieldPoseProvider provider, void* user) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  pose_provider_ = provider;")
    lines.append("  pose_provider_user_ = user;")
    lines.append("  odom_dx_ = 0.0;")
    lines.append("  odom_dy_ = 0.0;")
    lines.append("  odom_dth_ = 0.0;")
    lines.append("  odom_fault_noise_cycles_ = 0;")
    lines.append("  last_speed_in_s_ = 0.0;")
    lines.append("  last_turn_deg_s_ = 0.0;")
    lines.append("  provider_last_valid_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::clearFieldPoseProvider() {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  pose_provider_ = nullptr;")
    lines.append("  pose_provider_user_ = nullptr;")
    lines.append("  provider_last_valid_ = false;")
    lines.append("  odom_dx_ = 0.0;")
    lines.append("  odom_dy_ = 0.0;")
    lines.append("  odom_dth_ = 0.0;")
    lines.append("  odom_fault_noise_cycles_ = 0;")
    lines.append("  last_speed_in_s_ = 0.0;")
    lines.append("  last_turn_deg_s_ = 0.0;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::freezeLocalization(bool freeze) {")
    lines.append("  localization_frozen_.store(freeze);")
    lines.append("}")
    lines.append("")
    lines.append("bool ProsMCL::isLocalizationFrozen() const {")
    lines.append("  return localization_frozen_.load();")
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
    lines.append("    recovery_low_ess_streak_ = 0;")
    lines.append("    recovery_gate_reject_streak_ = 0;")
    lines.append("    recovery_cooldown_until_ms_ = 0;")
    lines.append("    recovery_lost_ = false;")
    lines.append("    recovery_exit_streak_ = 0;")
    lines.append("    recovery_lost_since_ms_ = 0;")
    lines.append("    mcl_.setForcedInjectionFraction(0.0);")
    lines.append("    sign_selftest_samples_ = 0;")
    lines.append("    sign_selftest_mismatch_ = 0;")
    lines.append("    sign_selftest_done_ = false;")
    lines.append("    ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("    localization_frozen_.store(false);")
    lines.append("  } else {")
    lines.append("    fused_pose_ = pose_;")
    lines.append("  }")
    lines.append("  mcl_ambiguous_ = false;")
    lines.append("  mcl_recover_good_count_ = 0;")
    lines.append("  recovery_low_ess_streak_ = 0;")
    lines.append("  recovery_gate_reject_streak_ = 0;")
    lines.append("  recovery_cooldown_until_ms_ = 0;")
    lines.append("  recovery_lost_ = false;")
    lines.append("  recovery_exit_streak_ = 0;")
    lines.append("  recovery_lost_since_ms_ = 0;")
    lines.append("  mcl_.setForcedInjectionFraction(0.0);")
    lines.append("  sign_selftest_samples_ = 0;")
    lines.append("  sign_selftest_mismatch_ = 0;")
    lines.append("  sign_selftest_done_ = false;")
    lines.append("  ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("  localization_frozen_.store(false);")
    lines.append("  odom_dx_ = 0.0;")
    lines.append("  odom_dy_ = 0.0;")
    lines.append("  odom_dth_ = 0.0;")
    lines.append("  odom_fault_noise_cycles_ = 0;")
    lines.append("  last_speed_in_s_ = 0.0;")
    lines.append("  last_turn_deg_s_ = 0.0;")
    lines.append("  imu_last_rotation_valid_ = false;")
    lines.append("  provider_last_pose_ = {x_in, y_in, theta_deg};")
    lines.append("  provider_last_valid_ = false;")
    lines.append("  int next_idx = 1 - pose_buf_idx_.load();")
    lines.append("  pose_buf_[next_idx] = pose_;")
    lines.append("  fused_pose_buf_[next_idx] = fused_pose_;")
    lines.append("  pose_buf_idx_.store(next_idx);")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::setPoseExternal(double x_in, double y_in, double theta_deg) {")
    lines.append("  MCLPose p = externalPoseToMCL({x_in, y_in, theta_deg});")
    lines.append("  setPose(p.x, p.y, p.theta);")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getPose() const {")
    lines.append("  int idx = pose_buf_idx_.load();")
    lines.append("  if (idx < 0 || idx > 1) idx = 0;")
    lines.append("  return pose_buf_[idx];")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getPoseExternal() const {")
    lines.append("  return mclPoseToExternal(getPose());")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getFusedPose() const {")
    lines.append("  int idx = pose_buf_idx_.load();")
    lines.append("  if (idx < 0 || idx > 1) idx = 0;")
    lines.append("  return fused_pose_buf_[idx];")
    lines.append("}")
    lines.append("")
    lines.append("MCLPose ProsMCL::getFusedPoseExternal() const {")
    lines.append("  return mclPoseToExternal(getFusedPose());")
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
    lines.append("  s.mcl_peakedness = mcl_.confidence();")
    lines.append("  s.mcl_ess_ratio = mcl_.essRatio();")
    lines.append("  s.mcl_neff = mcl_.neff();")
    lines.append("  s.ekf_pxx = ekf_.P[0][0];")
    lines.append("  s.ekf_pyy = ekf_.P[1][1];")
    lines.append("  s.ekf_pxy = ekf_.P[0][1];")
    lines.append("  s.ekf_ptt = ekf_.P[2][2];")
    lines.append("  s.event_flags = last_event_flags_;")
    lines.append("  last_event_flags_ = 0;")
    lines.append("  mcl_.getLastDistanceDebug(s.dist_meas_mm, s.dist_exp_mm, s.dist_errno, MCL_DISTANCE_SENSOR_COUNT_SAFE, &s.dist_used_mask);")
    lines.append("  return s;")
    lines.append("}")
    lines.append("")
    lines.append("void ProsMCL::updateVision(double x_in, double y_in, double theta_deg, double confidence) {")
    lines.append("  ScopedMutex lk(mu_);")
    lines.append("  mcl_.updateVision(x_in, y_in, theta_deg, confidence);")
    lines.append("  mcl_.normalize();")
    lines.append("  double mcl_cov[3][3];")
    lines.append("  MCLPose posterior_pose = pose_;")
    lines.append("  mcl_.estimateCovariance(&posterior_pose, mcl_cov);")
    lines.append("  pose_ = posterior_pose;")
    lines.append("  if (MCL_EKF_ENABLED) {")
    lines.append("    double conf = mcl_.confidence();")
    lines.append("    bool gate_reject = false;")
    lines.append("    bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf, pros::millis(), mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);")
    lines.append("    recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;")
    lines.append("    if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;")
    lines.append("    if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("  } else {")
    lines.append("    recovery_gate_reject_streak_ = 0;")
    lines.append("  }")
    lines.append("  mcl_.resample();")
    lines.append("  pose_ = posterior_pose;")
    lines.append("  if (MCL_EKF_ENABLED) {")
    lines.append("    if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("    else fused_pose_ = pose_;")
    lines.append("  } else {")
    lines.append("    fused_pose_ = pose_;")
    lines.append("  }")
    lines.append("  int next_idx = 1 - pose_buf_idx_.load();")
    lines.append("  pose_buf_[next_idx] = pose_;")
    lines.append("  fused_pose_buf_[next_idx] = fused_pose_;")
    lines.append("  pose_buf_idx_.store(next_idx);")
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
    lines.append("  if (MCL_CORR_SAFE_WINDOW_ENABLED) {")
    lines.append("    if (MCL_CORR_SAFE_MAX_SPEED_IN_S > 0.0 && last_speed_in_s_ > MCL_CORR_SAFE_MAX_SPEED_IN_S) return false;")
    lines.append("    if (MCL_CORR_SAFE_MAX_TURN_DEG_S > 0.0 && last_turn_deg_s_ > MCL_CORR_SAFE_MAX_TURN_DEG_S) return false;")
    lines.append("  }")
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
    lines.append("  uint32_t wake_ms = pros::millis();")
    lines.append("  uint32_t last_motion = wake_ms;")
    lines.append("  uint32_t last_sensor = wake_ms;")
    lines.append("  const std::uint32_t loop_ms = static_cast<std::uint32_t>(MCL_LOOP_MS > 0 ? MCL_LOOP_MS : 1);")
    lines.append("  while (running_.load()) {")
    lines.append("    const uint32_t now = pros::millis();")
    lines.append("    bool do_motion = (MCL_MOTION_UPDATE_MS <= 0 || now - last_motion >= static_cast<uint32_t>(MCL_MOTION_UPDATE_MS));")
    lines.append("    bool do_sensor = (MCL_SENSOR_UPDATE_MS <= 0 || now - last_sensor >= static_cast<uint32_t>(MCL_SENSOR_UPDATE_MS));")
    lines.append("    const bool distance_updates_allowed = MCL_USE_DISTANCE && map_blob_valid_;")
    lines.append("    if (!map_blob_valid_) last_event_flags_ |= MCL_EVENT_MAP_BLOB_MISMATCH;")
    lines.append("    double mm_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    double conf_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    double obj_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    int errno_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    int conf_meaningful_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    int obj_valid_buf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("    int mm_count = 0;")
    lines.append("    FieldPoseProvider provider_cb = nullptr;")
    lines.append("    void* provider_user = nullptr;")
    lines.append("    MCLPose provider_prev{0.0, 0.0, 0.0};")
    lines.append("    bool provider_prev_valid = false;")
    lines.append("    MCLPose provider_sample{0.0, 0.0, 0.0};")
    lines.append("    bool provider_sample_ok = false;")
    lines.append("    bool have_imu_heading = false;")
    lines.append("    double imu_heading = 0.0;")
    lines.append("    bool have_imu_dtheta = false;")
    lines.append("    double imu_dtheta = 0.0;")
    lines.append("    bool imu_unreliable = false;")
    lines.append("    bool imu_calibrating = false;")
    lines.append("    if (do_sensor && !sensor_batch_active_ && distance_updates_allowed && !dists_.empty()) {")
    lines.append("      int n = std::min(static_cast<int>(dists_.size()), MCL_DISTANCE_SENSOR_COUNT_SAFE);")
    lines.append("      mm_count = 0;")
    lines.append("      for (int i = 0; i < n; ++i) {")
    lines.append("        auto& d = dists_[i];")
    lines.append("        const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[i];")
    lines.append("        mm_buf[mm_count] = -1.0;")
    lines.append("        conf_buf[mm_count] = 0.0;")
    lines.append("        obj_buf[mm_count] = 0.0;")
    lines.append("        errno_buf[mm_count] = 0;")
    lines.append("        conf_meaningful_buf[mm_count] = 0;")
    lines.append("        obj_valid_buf[mm_count] = 0;")
    lines.append("        errno = 0;")
    lines.append("        int meas_raw_i = distance_read_mm(d);")
    lines.append("        int meas_errno = errno;")
    lines.append("        int sample_errno = 0;")
    lines.append("        if (meas_raw_i == PROS_ERR) {")
    lines.append("          sample_errno = meas_errno;")
    lines.append("          errno_buf[mm_count] = sample_errno;")
    lines.append("          mm_count++;")
    lines.append("          continue;")
    lines.append("        }")
    lines.append("        double meas = static_cast<double>(meas_raw_i);")
    lines.append("        bool valid = true;")
    lines.append("        bool no_object = (meas >= 9999.0);")
    lines.append("        double meas_bias = meas - cfg.bias_mm;")
    lines.append("        double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("        double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("        if (s_min <= 0.0) s_min = 20.0;")
    lines.append("        if (s_max <= 0.0) s_max = 2000.0;")
    lines.append("        if (no_object) {")
    lines.append("          if (MCL_DIST_USE_NO_OBJECT_INFO) {")
    lines.append("            meas = s_max + cfg.bias_mm;")
    lines.append("            meas_bias = s_max;")
    lines.append("          } else {")
    lines.append("            valid = false;")
    lines.append("          }")
    lines.append("        }")
    lines.append("        if (valid && meas_bias > s_max) valid = false;")
    lines.append("        if (valid && meas_bias < s_min) valid = false;")
    lines.append("        bool conf_meaningful = valid && !no_object && (meas_bias > 200.0);")
    lines.append("        int conf_i = 63;")
    lines.append("        if (valid && conf_meaningful) {")
    lines.append("          errno = 0;")
    lines.append("          conf_i = d.get_confidence();")
    lines.append("          int conf_errno = errno;")
    lines.append("          if (conf_i == PROS_ERR) {")
    lines.append("            if (sample_errno == 0) sample_errno = conf_errno;")
    lines.append("            conf_i = 0;")
    lines.append("            conf_meaningful = false;")
    lines.append("          }")
    lines.append("        }")
    lines.append("        double conf_min = (cfg.min_confidence > 0.0) ? cfg.min_confidence : MCL_DIST_CONFIDENCE_MIN;")
    lines.append("        if (valid && conf_meaningful && conf_min > 0.0) {")
    lines.append("          double conf_thresh = conf_min;")
    lines.append("          if (conf_thresh <= 1.0) conf_thresh *= 63.0;")
    lines.append("          if (conf_thresh < 0.0) conf_thresh = 0.0;")
    lines.append("          if (conf_thresh > 63.0) conf_thresh = 63.0;")
    lines.append("          if (static_cast<double>(conf_i) < conf_thresh) valid = false;")
    lines.append("        }")
    lines.append("        int obj_i = PROS_ERR;")
    lines.append("        bool obj_valid = false;")
    lines.append("        if (valid && !no_object) {")
    lines.append("          errno = 0;")
    lines.append("          obj_i = d.get_object_size();")
    lines.append("          int obj_errno = errno;")
    lines.append("          obj_valid = (obj_i != PROS_ERR);")
    lines.append("          if (!obj_valid && sample_errno == 0) sample_errno = obj_errno;")
    lines.append("        }")
    lines.append("        double obj_min = (cfg.min_object_size > 0.0) ? cfg.min_object_size : MCL_DIST_OBJECT_SIZE_MIN;")
    lines.append("        double obj_max = (cfg.max_object_size > 0.0) ? cfg.max_object_size : MCL_DIST_OBJECT_SIZE_MAX;")
    lines.append("        if (valid && obj_valid && (obj_min > 0.0 || obj_max > 0.0)) {")
    lines.append("          if ((obj_min > 0.0 && obj_i < obj_min) || (obj_max > 0.0 && obj_i > obj_max)) valid = false;")
    lines.append("        }")
    lines.append("        conf_buf[mm_count] = static_cast<double>(conf_i);")
    lines.append("        conf_meaningful_buf[mm_count] = conf_meaningful ? 1 : 0;")
    lines.append("        obj_buf[mm_count] = static_cast<double>(obj_i);")
    lines.append("        errno_buf[mm_count] = sample_errno;")
    lines.append("        obj_valid_buf[mm_count] = obj_valid ? 1 : 0;")
    lines.append("        if (!valid) {")
    lines.append("          mm_count++;")
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
    lines.append("          meas = temp[cnt / 2];")
    lines.append("        }")
    lines.append("        mm_buf[mm_count] = meas;")
    lines.append("        mm_count++;")
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
    lines.append("    if (imu_ && imu_is_calibrating(*imu_)) {")
    lines.append("      imu_calibrating = true;")
    lines.append("      imu_unreliable = true;")
    lines.append("    }")
    lines.append("    if (do_sensor && imu_ && !sensor_batch_active_ && !imu_calibrating) {")
    lines.append("      double h = 0.0;")
    lines.append("      if (imu_read_heading_deg(*imu_, &h)) {")
    lines.append("        imu_heading = h;")
    lines.append("        have_imu_heading = true;")
    lines.append("      } else {")
    lines.append("        imu_unreliable = true;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    if (do_motion && imu_) {")
    lines.append("      double rot = 0.0;")
    lines.append("      if (!imu_calibrating && imu_read_rotation_deg(*imu_, &rot)) {")
    lines.append("        if (imu_last_rotation_valid_) {")
    lines.append("          imu_dtheta = rot - imu_last_rotation_;")
    lines.append("          have_imu_dtheta = true;")
    lines.append("        }")
    lines.append("        imu_last_rotation_ = rot;")
    lines.append("        imu_last_rotation_valid_ = true;")
    lines.append("      } else {")
    lines.append("        imu_last_rotation_valid_ = false;")
    lines.append("        imu_unreliable = true;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    {")
    lines.append("      ScopedMutex lk(mu_);")
    lines.append("      if (relocalize_requested_.load()) {")
    lines.append("        if (sensor_batch_active_) {")
    lines.append("          mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);")
    lines.append("          sensor_batch_active_ = false;")
    lines.append("          sensor_batch_count_ = 0;")
    lines.append("          sensor_batch_cursor_ = 0;")
    lines.append("          sensor_batch_have_imu_heading_ = false;")
    lines.append("          sensor_batch_imu_applied_ = false;")
    lines.append("        }")
    lines.append("        mcl_.initGlobal();")
    lines.append("        pose_ = mcl_.estimate();")
    lines.append("        ekf_.initialized = false;")
    lines.append("        fused_pose_ = pose_;")
    lines.append("        odom_dx_ = 0.0;")
    lines.append("        odom_dy_ = 0.0;")
    lines.append("        odom_dth_ = 0.0;")
    lines.append("        odom_fault_noise_cycles_ = 0;")
    lines.append("        last_speed_in_s_ = 0.0;")
    lines.append("        last_turn_deg_s_ = 0.0;")
    lines.append("        imu_last_rotation_valid_ = false;")
    lines.append("        provider_last_valid_ = false;")
    lines.append("        mcl_ambiguous_ = false;")
    lines.append("        mcl_recover_good_count_ = 0;")
    lines.append("        recovery_low_ess_streak_ = 0;")
    lines.append("        recovery_gate_reject_streak_ = 0;")
    lines.append("        recovery_cooldown_until_ms_ = 0;")
    lines.append("        recovery_exit_streak_ = 0;")
    lines.append("        if (recovery_lost_ && recovery_lost_since_ms_ == 0) recovery_lost_since_ms_ = now;")
    lines.append("        if (!recovery_lost_) {")
    lines.append("          recovery_lost_since_ms_ = 0;")
    lines.append("          mcl_.setForcedInjectionFraction(0.0);")
    lines.append("        }")
    lines.append("        sign_selftest_samples_ = 0;")
    lines.append("        sign_selftest_mismatch_ = 0;")
    lines.append("        sign_selftest_done_ = false;")
    lines.append("        ekf_intervene_cooldown_until_ms_ = 0;")
    lines.append("        relocalize_requested_.store(false);")
    lines.append("      }")
    lines.append("      if (localization_frozen_.load()) {")
    lines.append("        if (sensor_batch_active_) {")
    lines.append("          mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);")
    lines.append("          sensor_batch_active_ = false;")
    lines.append("          sensor_batch_count_ = 0;")
    lines.append("          sensor_batch_cursor_ = 0;")
    lines.append("          sensor_batch_have_imu_heading_ = false;")
    lines.append("          sensor_batch_imu_applied_ = false;")
    lines.append("        }")
    lines.append("        if (do_motion) {")
    lines.append("          if (provider_cb == pose_provider_ && provider_sample_ok) {")
    lines.append("            provider_last_pose_ = provider_sample;")
    lines.append("            provider_last_valid_ = true;")
    lines.append("          }")
    lines.append("          odom_dx_ = 0.0;")
    lines.append("          odom_dy_ = 0.0;")
    lines.append("          odom_dth_ = 0.0;")
    lines.append("          odom_fault_noise_cycles_ = 0;")
    lines.append("          last_speed_in_s_ = 0.0;")
    lines.append("          last_turn_deg_s_ = 0.0;")
    lines.append("          last_motion = now;")
    lines.append("        }")
    lines.append("        if (do_sensor) {")
    lines.append("          last_sensor = now;")
    lines.append("        }")
    lines.append("      } else if (do_motion) {")
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
    lines.append("        double dth_odom = odom_dth_;")
    lines.append("        double dth = dth_odom;")
    lines.append("        if (MCL_FRAME_SIGN_SELF_TEST_ENABLED && !sign_selftest_done_ && have_imu_dtheta) {")
    lines.append("          double min_delta = (MCL_FRAME_SIGN_SELF_TEST_MIN_DELTA_DEG > 0.0) ?")
    lines.append("            MCL_FRAME_SIGN_SELF_TEST_MIN_DELTA_DEG : 2.0;")
    lines.append("          if (std::fabs(imu_dtheta) >= min_delta && std::fabs(dth_odom) >= min_delta) {")
    lines.append("            sign_selftest_samples_++;")
    lines.append("            if ((imu_dtheta * dth_odom) < 0.0) sign_selftest_mismatch_++;")
    lines.append("            int target = std::max(1, MCL_FRAME_SIGN_SELF_TEST_SAMPLES);")
    lines.append("            int mismatch_th = std::max(1, MCL_FRAME_SIGN_SELF_TEST_MISMATCH);")
    lines.append("            if (sign_selftest_samples_ >= target) {")
    lines.append("              sign_selftest_done_ = true;")
    lines.append("              if (sign_selftest_mismatch_ >= mismatch_th) {")
    lines.append("                last_event_flags_ |= MCL_EVENT_FRAME_SIGN_MISMATCH;")
    lines.append("              }")
    lines.append("            }")
    lines.append("          }")
    lines.append("        }")
    lines.append("        if (have_imu_dtheta) dth = imu_dtheta;")
    lines.append("        odom_dx_ = 0.0;")
    lines.append("        odom_dy_ = 0.0;")
    lines.append("        odom_dth_ = 0.0;")
    lines.append("        double motion_noise_scale = 1.0;")
    lines.append("        if (MCL_ODOM_DELTA_GUARD_ENABLED) {")
    lines.append("          bool odom_fault = false;")
    lines.append("          if (MCL_ODOM_MAX_DX_IN_PER_TICK > 0.0 && std::fabs(dx) > MCL_ODOM_MAX_DX_IN_PER_TICK) odom_fault = true;")
    lines.append("          if (MCL_ODOM_MAX_DY_IN_PER_TICK > 0.0 && std::fabs(dy) > MCL_ODOM_MAX_DY_IN_PER_TICK) odom_fault = true;")
    lines.append("          if (MCL_ODOM_MAX_DTHETA_DEG_PER_TICK > 0.0 && std::fabs(dth) > MCL_ODOM_MAX_DTHETA_DEG_PER_TICK) odom_fault = true;")
    lines.append("          if (odom_fault) {")
    lines.append("            dx = 0.0;")
    lines.append("            dy = 0.0;")
    lines.append("            dth = 0.0;")
    lines.append("            if (MCL_ODOM_FAULT_INFLATE_CYCLES > 0) odom_fault_noise_cycles_ = std::max(odom_fault_noise_cycles_, MCL_ODOM_FAULT_INFLATE_CYCLES);")
    lines.append("          }")
    lines.append("        }")
    lines.append("        if (odom_fault_noise_cycles_ > 0) {")
    lines.append("          if (MCL_ODOM_FAULT_NOISE_SCALE > 1.0) motion_noise_scale = MCL_ODOM_FAULT_NOISE_SCALE;")
    lines.append("          odom_fault_noise_cycles_--;")
    lines.append("        }")
    lines.append("        if (imu_ && imu_unreliable && MCL_IMU_FALLBACK_NOISE_SCALE > 1.0) {")
    lines.append("          motion_noise_scale = std::max(motion_noise_scale, MCL_IMU_FALLBACK_NOISE_SCALE);")
    lines.append("        }")
    lines.append("        double dt_s = static_cast<double>((MCL_MOTION_UPDATE_MS > 0) ? MCL_MOTION_UPDATE_MS : MCL_LOOP_MS) / 1000.0;")
    lines.append("        if (dt_s < 1e-3) dt_s = 1e-3;")
    lines.append("        last_speed_in_s_ = std::sqrt(dx * dx + dy * dy) / dt_s;")
    lines.append("        last_turn_deg_s_ = std::fabs(dth) / dt_s;")
    lines.append("        mcl_.predict(dx, dy, dth, motion_noise_scale);")
    lines.append("        pose_ = mcl_.estimate();")
    lines.append("        if (MCL_EKF_ENABLED) {")
    lines.append("          if (ekf_.initialized) {")
    lines.append("            ekf_predict(ekf_, dx, dy, dth, motion_noise_scale);")
    lines.append("            fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("          } else {")
    lines.append("            fused_pose_ = pose_;")
    lines.append("          }")
    lines.append("        } else {")
    lines.append("          fused_pose_ = pose_;")
    lines.append("        }")
    lines.append("        last_motion = now;")
    lines.append("      }")
    lines.append("      if (!localization_frozen_.load()) {")
    lines.append("        if (do_sensor && !sensor_batch_active_) {")
    lines.append("          if (distance_updates_allowed && mm_count > 0) {")
    lines.append("            mcl_.copyWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);")
    lines.append("            sensor_batch_count_ = std::min(mm_count, MCL_DISTANCE_SENSOR_COUNT_SAFE);")
    lines.append("            for (int i = 0; i < sensor_batch_count_; ++i) {")
    lines.append("              sensor_batch_mm_[i] = mm_buf[i];")
    lines.append("              sensor_batch_conf_[i] = conf_buf[i];")
    lines.append("              sensor_batch_obj_[i] = obj_buf[i];")
    lines.append("              sensor_batch_errno_[i] = errno_buf[i];")
    lines.append("              sensor_batch_conf_meaningful_[i] = conf_meaningful_buf[i];")
    lines.append("              sensor_batch_obj_valid_[i] = obj_valid_buf[i];")
    lines.append("            }")
    lines.append("            for (int i = sensor_batch_count_; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("              sensor_batch_mm_[i] = -1.0;")
    lines.append("              sensor_batch_conf_[i] = 0.0;")
    lines.append("              sensor_batch_obj_[i] = 0.0;")
    lines.append("              sensor_batch_errno_[i] = 0;")
    lines.append("              sensor_batch_conf_meaningful_[i] = 0;")
    lines.append("              sensor_batch_obj_valid_[i] = 0;")
    lines.append("            }")
    lines.append("            sensor_batch_cursor_ = 0;")
    lines.append("            sensor_batch_have_imu_heading_ = have_imu_heading;")
    lines.append("            sensor_batch_imu_heading_ = imu_heading;")
    lines.append("            sensor_batch_imu_applied_ = false;")
    lines.append("            sensor_batch_active_ = true;")
    lines.append("          } else if (have_imu_heading) {")
    lines.append("            if (MCL_USE_IMU) mcl_.updateIMU(imu_heading);")
    lines.append("            if (MCL_EKF_ENABLED && MCL_EKF_USE_IMU) ekf_update_imu(ekf_, imu_heading);")
    lines.append("            mcl_.normalize();")
    lines.append("            double mcl_cov[3][3];")
    lines.append("            MCLPose posterior_pose = pose_;")
    lines.append("            mcl_.estimateCovariance(&posterior_pose, mcl_cov);")
    lines.append("            pose_ = posterior_pose;")
    lines.append("            bool gate_reject = false;")
    lines.append("            double conf_now = mcl_.confidence();")
    lines.append("            if (MCL_EKF_ENABLED) {")
    lines.append("              bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf_now, now, mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);")
    lines.append("              if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;")
    lines.append("              if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("              recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;")
    lines.append("            } else {")
    lines.append("              recovery_gate_reject_streak_ = 0;")
    lines.append("            }")
    lines.append("            double ess_ratio = mcl_.neff() / std::max(1.0, static_cast<double>(mcl_.particleCount()));")
    lines.append("            if (MCL_RECOVERY_ESS_RATIO_MIN > 0.0 && ess_ratio < MCL_RECOVERY_ESS_RATIO_MIN) recovery_low_ess_streak_++;")
    lines.append("            else recovery_low_ess_streak_ = 0;")
    lines.append("            bool recovery_trigger = false;")
    lines.append("            if (MCL_RECOVERY_ENABLED) {")
    lines.append("              if (MCL_RECOVERY_ESS_STREAK > 0 && recovery_low_ess_streak_ >= MCL_RECOVERY_ESS_STREAK) recovery_trigger = true;")
    lines.append("              if (MCL_EKF_ENABLED && MCL_RECOVERY_EKF_GATE_REJECT_STREAK > 0 &&")
    lines.append("                  recovery_gate_reject_streak_ >= MCL_RECOVERY_EKF_GATE_REJECT_STREAK) recovery_trigger = true;")
    lines.append("            }")
    lines.append("            if (recovery_trigger && now >= recovery_cooldown_until_ms_) {")
    lines.append("              recovery_lost_ = true;")
    lines.append("              recovery_exit_streak_ = 0;")
    lines.append("              recovery_lost_since_ms_ = now;")
    lines.append("              double lost_inj = MCL_RECOVERY_LOST_INJECTION_FRACTION;")
    lines.append("              if (lost_inj < 0.0) lost_inj = 0.0;")
    lines.append("              if (lost_inj > 1.0) lost_inj = 1.0;")
    lines.append("              mcl_.setForcedInjectionFraction(lost_inj);")
    lines.append("              relocalize_requested_.store(true);")
    lines.append("              recovery_low_ess_streak_ = 0;")
    lines.append("              recovery_gate_reject_streak_ = 0;")
    lines.append("              if (MCL_RECOVERY_COOLDOWN_MS > 0) recovery_cooldown_until_ms_ = now + static_cast<uint32_t>(MCL_RECOVERY_COOLDOWN_MS);")
    lines.append("              last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("            }")
    lines.append("            if (recovery_lost_) {")
    lines.append("              bool lost_good = (conf_now >= MCL_RECOVERY_LOST_EXIT_CONFIDENCE) && !gate_reject && !mcl_ambiguous_;")
    lines.append("              if (lost_good) recovery_exit_streak_++;")
    lines.append("              else recovery_exit_streak_ = 0;")
    lines.append("              if (MCL_RECOVERY_LOST_FORCE_REINIT_MS > 0 && now > recovery_lost_since_ms_ &&")
    lines.append("                  (now - recovery_lost_since_ms_) >= static_cast<uint32_t>(MCL_RECOVERY_LOST_FORCE_REINIT_MS)) {")
    lines.append("                relocalize_requested_.store(true);")
    lines.append("                recovery_lost_since_ms_ = now;")
    lines.append("              }")
    lines.append("              if (recovery_exit_streak_ >= std::max(1, MCL_RECOVERY_LOST_EXIT_STREAK)) {")
    lines.append("                recovery_lost_ = false;")
    lines.append("                recovery_exit_streak_ = 0;")
    lines.append("                recovery_lost_since_ms_ = 0;")
    lines.append("                mcl_.setForcedInjectionFraction(0.0);")
    lines.append("              } else {")
    lines.append("                last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("              }")
    lines.append("            }")
    lines.append("            mcl_.resample();")
    lines.append("            pose_ = posterior_pose;")
    lines.append("            if (MCL_EKF_ENABLED) {")
    lines.append("              if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("              else fused_pose_ = pose_;")
    lines.append("            } else {")
    lines.append("              fused_pose_ = pose_;")
    lines.append("            }")
    lines.append("            last_sensor = now;")
    lines.append("          } else {")
    lines.append("            last_sensor = now;")
    lines.append("          }")
    lines.append("        }")
    lines.append("        if (sensor_batch_active_) {")
    lines.append("          int n = sensor_batch_count_;")
    lines.append("          int step = MCL_DIST_BATCH_SIZE;")
    lines.append("          if (step <= 0) step = n;")
    lines.append("          if (step <= 0) step = MCL_DISTANCE_SENSOR_COUNT_SAFE;")
    lines.append("          int begin = sensor_batch_cursor_;")
    lines.append("          if (begin < 0) begin = 0;")
    lines.append("          if (begin > n) begin = n;")
    lines.append("          int end = begin + step;")
    lines.append("          if (end > n) end = n;")
    lines.append("          if (distance_updates_allowed && n > 0 && end > begin) {")
    lines.append("            double chunk_mm[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            double chunk_conf[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            double chunk_obj[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            int chunk_errno[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            int chunk_conf_meaningful[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            int chunk_obj_valid[MCL_DISTANCE_SENSOR_COUNT_SAFE];")
    lines.append("            for (int i = 0; i < n; ++i) {")
    lines.append("              chunk_mm[i] = -1.0;")
    lines.append("              chunk_conf[i] = 0.0;")
    lines.append("              chunk_obj[i] = 0.0;")
    lines.append("              chunk_errno[i] = 0;")
    lines.append("              chunk_conf_meaningful[i] = 0;")
    lines.append("              chunk_obj_valid[i] = 0;")
    lines.append("            }")
    lines.append("            for (int i = begin; i < end; ++i) {")
    lines.append("              chunk_mm[i] = sensor_batch_mm_[i];")
    lines.append("              chunk_conf[i] = sensor_batch_conf_[i];")
    lines.append("              chunk_obj[i] = sensor_batch_obj_[i];")
    lines.append("              chunk_errno[i] = sensor_batch_errno_[i];")
    lines.append("              chunk_conf_meaningful[i] = sensor_batch_conf_meaningful_[i];")
    lines.append("              chunk_obj_valid[i] = sensor_batch_obj_valid_[i];")
    lines.append("            }")
    lines.append("            mcl_.updateDistance(chunk_mm, chunk_conf, chunk_conf_meaningful, chunk_obj, chunk_obj_valid, chunk_errno, n);")
    lines.append("          }")
    lines.append("          sensor_batch_cursor_ = end;")
    lines.append("          bool batch_done = (sensor_batch_cursor_ >= n);")
    lines.append("          if (batch_done) {")
    lines.append("            if (sensor_batch_have_imu_heading_ && !sensor_batch_imu_applied_) {")
    lines.append("              if (MCL_USE_IMU) mcl_.updateIMU(sensor_batch_imu_heading_);")
    lines.append("              if (MCL_EKF_ENABLED && MCL_EKF_USE_IMU) ekf_update_imu(ekf_, sensor_batch_imu_heading_);")
    lines.append("              sensor_batch_imu_applied_ = true;")
    lines.append("            }")
    lines.append("            mcl_.normalize();")
    lines.append("            double mcl_cov[3][3];")
    lines.append("            MCLPose posterior_pose = pose_;")
    lines.append("            mcl_.estimateCovariance(&posterior_pose, mcl_cov);")
    lines.append("            pose_ = posterior_pose;")
    lines.append("            bool gate_reject = false;")
    lines.append("            double conf_now = mcl_.confidence();")
    lines.append("            if (MCL_EKF_ENABLED) {")
    lines.append("              bool applied = ekf_fusion_update(ekf_, pose_, mcl_cov, conf_now, now, mcl_ambiguous_, mcl_recover_good_count_, ekf_intervene_cooldown_until_ms_, &gate_reject);")
    lines.append("              if (applied) last_event_flags_ |= MCL_EVENT_MCL_EKF_APPLIED;")
    lines.append("              if (mcl_ambiguous_) last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("              recovery_gate_reject_streak_ = gate_reject ? (recovery_gate_reject_streak_ + 1) : 0;")
    lines.append("            } else {")
    lines.append("              recovery_gate_reject_streak_ = 0;")
    lines.append("            }")
    lines.append("            double ess_ratio = mcl_.neff() / std::max(1.0, static_cast<double>(mcl_.particleCount()));")
    lines.append("            if (MCL_RECOVERY_ESS_RATIO_MIN > 0.0 && ess_ratio < MCL_RECOVERY_ESS_RATIO_MIN) recovery_low_ess_streak_++;")
    lines.append("            else recovery_low_ess_streak_ = 0;")
    lines.append("            bool recovery_trigger = false;")
    lines.append("            if (MCL_RECOVERY_ENABLED) {")
    lines.append("              if (MCL_RECOVERY_ESS_STREAK > 0 && recovery_low_ess_streak_ >= MCL_RECOVERY_ESS_STREAK) recovery_trigger = true;")
    lines.append("              if (MCL_EKF_ENABLED && MCL_RECOVERY_EKF_GATE_REJECT_STREAK > 0 &&")
    lines.append("                  recovery_gate_reject_streak_ >= MCL_RECOVERY_EKF_GATE_REJECT_STREAK) recovery_trigger = true;")
    lines.append("            }")
    lines.append("            if (recovery_trigger && now >= recovery_cooldown_until_ms_) {")
    lines.append("              recovery_lost_ = true;")
    lines.append("              recovery_exit_streak_ = 0;")
    lines.append("              recovery_lost_since_ms_ = now;")
    lines.append("              double lost_inj = MCL_RECOVERY_LOST_INJECTION_FRACTION;")
    lines.append("              if (lost_inj < 0.0) lost_inj = 0.0;")
    lines.append("              if (lost_inj > 1.0) lost_inj = 1.0;")
    lines.append("              mcl_.setForcedInjectionFraction(lost_inj);")
    lines.append("              relocalize_requested_.store(true);")
    lines.append("              recovery_low_ess_streak_ = 0;")
    lines.append("              recovery_gate_reject_streak_ = 0;")
    lines.append("              if (MCL_RECOVERY_COOLDOWN_MS > 0) recovery_cooldown_until_ms_ = now + static_cast<uint32_t>(MCL_RECOVERY_COOLDOWN_MS);")
    lines.append("              last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("            }")
    lines.append("            if (recovery_lost_) {")
    lines.append("              bool lost_good = (conf_now >= MCL_RECOVERY_LOST_EXIT_CONFIDENCE) && !gate_reject && !mcl_ambiguous_;")
    lines.append("              if (lost_good) recovery_exit_streak_++;")
    lines.append("              else recovery_exit_streak_ = 0;")
    lines.append("              if (MCL_RECOVERY_LOST_FORCE_REINIT_MS > 0 && now > recovery_lost_since_ms_ &&")
    lines.append("                  (now - recovery_lost_since_ms_) >= static_cast<uint32_t>(MCL_RECOVERY_LOST_FORCE_REINIT_MS)) {")
    lines.append("                relocalize_requested_.store(true);")
    lines.append("                recovery_lost_since_ms_ = now;")
    lines.append("              }")
    lines.append("              if (recovery_exit_streak_ >= std::max(1, MCL_RECOVERY_LOST_EXIT_STREAK)) {")
    lines.append("                recovery_lost_ = false;")
    lines.append("                recovery_exit_streak_ = 0;")
    lines.append("                recovery_lost_since_ms_ = 0;")
    lines.append("                mcl_.setForcedInjectionFraction(0.0);")
    lines.append("              } else {")
    lines.append("                last_event_flags_ |= MCL_EVENT_RECOVERY_ACTIVE;")
    lines.append("              }")
    lines.append("            }")
    lines.append("            mcl_.resample();")
    lines.append("            pose_ = posterior_pose;")
    lines.append("            if (MCL_EKF_ENABLED) {")
    lines.append("              if (ekf_.initialized) fused_pose_ = {ekf_.x, ekf_.y, ekf_.theta};")
    lines.append("              else fused_pose_ = pose_;")
    lines.append("            } else {")
    lines.append("              fused_pose_ = pose_;")
    lines.append("            }")
    lines.append("            sensor_batch_active_ = false;")
    lines.append("            sensor_batch_count_ = 0;")
    lines.append("            sensor_batch_cursor_ = 0;")
    lines.append("            sensor_batch_have_imu_heading_ = false;")
    lines.append("            sensor_batch_imu_applied_ = false;")
    lines.append("            last_sensor = now;")
    lines.append("          }")
    lines.append("        }")
    lines.append("      }")
    lines.append("      int next_idx = 1 - pose_buf_idx_.load();")
    lines.append("      pose_buf_[next_idx] = pose_;")
    lines.append("      fused_pose_buf_[next_idx] = fused_pose_;")
    lines.append("      pose_buf_idx_.store(next_idx);")
    lines.append("    }")
    lines.append("    const uint32_t now_end = pros::millis();")
    lines.append("    const uint32_t late_ms = (now_end > wake_ms) ? (now_end - wake_ms) : 0u;")
    lines.append("    if (MCL_STALL_MS > 0 && late_ms > static_cast<uint32_t>(MCL_STALL_MS)) {")
    lines.append("      ScopedMutex lk(mu_);")
    lines.append("      if (sensor_batch_active_) {")
    lines.append("        mcl_.setWeights(sensor_batch_weights_backup_, MCL_PARTICLE_CAPACITY);")
    lines.append("        sensor_batch_active_ = false;")
    lines.append("        sensor_batch_count_ = 0;")
    lines.append("        sensor_batch_cursor_ = 0;")
    lines.append("        sensor_batch_have_imu_heading_ = false;")
    lines.append("        sensor_batch_imu_applied_ = false;")
    lines.append("      }")
    lines.append("      last_sensor = now_end;")
    lines.append("    }")
    lines.append("    pros::Task::delay_until(&wake_ms, loop_ms);")
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
        "  void predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale = 1.0);",
        "  void updateDistance(const double* dist_mm, const double* dist_conf, const int* conf_meaningful, const double* dist_obj_size, const int* obj_size_valid, const int* dist_errno, int count);",
        "  void updateIMU(double heading_deg);",
        "  void updateVision(double x_in, double y_in, double theta_deg, double confidence);",
        "  void estimateCovariance(MCLPose* pose_out, double cov_out[3][3]);",
        "  void copyWeights(double* out_weights, int capacity) const;",
        "  void setWeights(const double* in_weights, int capacity);",
        "  void setForcedInjectionFraction(double frac);",
        "  double forcedInjectionFraction() const { return forced_injection_fraction_; }",
        "  void setSegmentBand(const MCLPose* pts, int n, double radius_in);",
        "  void clearSegmentBand();",
        "  void normalize();",
        "  void resample();",
        "  MCLPose estimate();",
        "  // Localization peakedness: 0 = uniform weights, 1 = highly peaked weights.",
        "  double confidence() const { return confidence_; }",
        "  // ESS ratio: 1 = uniform weights, 0 = maximally concentrated.",
        "  double essRatio() const { return ess_ratio_; }",
        "  double neff() const { return effectiveN(); }",
        "  int particleCount() const { return count_; }",
        "  void getLastDistanceDebug(double* measured_mm, double* expected_mm, int* errno_codes, int capacity, uint32_t* used_mask) const;",
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
        "  double forced_injection_fraction_;",
        "  double confidence_;",
        "  double ess_ratio_;",
        "  MCLPose estimate_;",
        "  bool estimate_valid_;",
        "  bool have_estimate_ever_;",
        "  double last_dist_measured_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  double last_dist_expected_mm_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  int last_dist_errno_[MCL_DISTANCE_SENSOR_COUNT_SAFE];",
        "  uint32_t last_dist_used_mask_;",
        "  std::vector<MCLPose> segment_band_pts_;",
        "  double segment_band_radius_;",
        "  bool segment_band_active_;",
        "  double regionWeight(double x, double y, double heading_deg) const;",
        "  void cgrLiteRefineEstimate();",
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
            res_in = float(lf_cfg.get("resolution_in", 1.0))
        except Exception:
            res_in = 2.0
        try:
            lf_max_bytes = int(lf_cfg.get("max_bytes", 262144))
        except Exception:
            lf_max_bytes = 262144
        res_in = _choose_lf_resolution(field_w, field_h, res_in, lf_max_bytes)
        dist_field = _build_distance_field(segs, field_w, field_h, res_in)
        dist_field_perim = _build_distance_field(perim_segs, field_w, field_h, res_in)
        dist_field_obj = _build_distance_field(obj_segs, field_w, field_h, res_in)
    else:
        dist_field = None
        dist_field_perim = None
        dist_field_obj = None
    try:
        ray_bucket_in = float(dist_cfg.get("raycast_bucket_in", 12.0)) if isinstance(dist_cfg, dict) else 12.0
    except Exception:
        ray_bucket_in = 12.0
    bucket_map = _build_raycast_bucket_grid(segs, field_w, field_h, ray_bucket_in)
    bucket_perim = _build_raycast_bucket_grid(perim_segs, field_w, field_h, ray_bucket_in)
    bucket_obj = _build_raycast_bucket_grid(obj_segs, field_w, field_h, ray_bucket_in)
    lines: List[str] = []
    lines.append("#include \"api.h\"")
    lines.append("#include \"mcl_localizer.h\"")
    lines.append("#include \"mcl_config.hpp\"")
    lines.append("#include <algorithm>")
    lines.append("#include <cmath>")
    lines.append("#include <cstdint>")
    lines.append("#include <random>")
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
            lines.append(f"extern const uint16_t {name}[];")
        else:
            lines.append(f"static const int {name}_W = 1;")
            lines.append(f"static const int {name}_H = 1;")
            lines.append(f"static const double {name}_RES_IN = 1.0;")
            lines.append(f"static const double {name}_ORIGIN_X = 0.0;")
            lines.append(f"static const double {name}_ORIGIN_Y = 0.0;")
            lines.append(f"extern const uint16_t {name}[];")

    _emit_dist_field(dist_field, "MAP_DIST_FIELD")
    _emit_dist_field(dist_field_perim, "MAP_DIST_FIELD_PERIM")
    _emit_dist_field(dist_field_obj, "MAP_DIST_FIELD_OBJ")
    _emit_bucket_grid(lines, bucket_map, "MAP_RAY_BUCKET")
    _emit_bucket_grid(lines, bucket_perim, "PERIM_RAY_BUCKET")
    _emit_bucket_grid(lines, bucket_obj, "OBJECT_RAY_BUCKET")
    lines.append("")
    lines.append("static std::mt19937& rng_engine() {")
    lines.append("  static std::mt19937 eng(0u);")
    lines.append("  return eng;")
    lines.append("}")
    lines.append("")
    lines.append("static double rand_uniform() {")
    lines.append("  return std::generate_canonical<double, 53>(rng_engine());")
    lines.append("}")
    lines.append("")
    lines.append("static double rand_gaussian(double mean, double stddev) {")
    lines.append("  if (stddev <= 0.0) return mean;")
    lines.append("  std::normal_distribution<double> dist(mean, stddev);")
    lines.append("  return dist(rng_engine());")
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
    lines.append("static int fov_ray_count() {")
    lines.append("  int k = MCL_DIST_RAYS_PER_SENSOR;")
    lines.append("  if (k < 1) k = 1;")
    lines.append("  if (k > 9) k = 9;")
    lines.append("  return k;")
    lines.append("}")
    lines.append("")
    lines.append("static double fov_half_deg(double meas_mm) {")
    lines.append("  if (!MCL_DIST_FOV_MULTI_RAY) return 0.0;")
    lines.append("  return (meas_mm < MCL_DIST_FOV_SWITCH_MM) ? MCL_DIST_FOV_HALF_DEG_NEAR : MCL_DIST_FOV_HALF_DEG_FAR;")
    lines.append("}")
    lines.append("")
    lines.append("static double logmeanexp(const double* vals, int n) {")
    lines.append("  if (!vals || n <= 0) return -1e300;")
    lines.append("  double m = vals[0];")
    lines.append("  for (int i = 1; i < n; ++i) if (vals[i] > m) m = vals[i];")
    lines.append("  double s = 0.0;")
    lines.append("  for (int i = 0; i < n; ++i) s += std::exp(vals[i] - m);")
    lines.append("  if (s <= 1e-300) return -1e300;")
    lines.append("  return m + std::log(s) - std::log(static_cast<double>(n));")
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
    lines.append("static double distance_field_lookup(const uint16_t* field, int w, int h, double res,")
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
    lines.append("  double d00 = static_cast<double>(field[idx]) / MCL_MM_PER_IN;")
    lines.append("  double d10 = static_cast<double>(field[idx + 1]) / MCL_MM_PER_IN;")
    lines.append("  double d01 = static_cast<double>(field[idx + w]) / MCL_MM_PER_IN;")
    lines.append("  double d11 = static_cast<double>(field[idx + w + 1]) / MCL_MM_PER_IN;")
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
    lines.append("struct RayBucketView {")
    lines.append("  const uint32_t* offsets;")
    lines.append("  const uint16_t* indices;")
    lines.append("  int nx;")
    lines.append("  int ny;")
    lines.append("  double cell_in;")
    lines.append("  double origin_x;")
    lines.append("  double origin_y;")
    lines.append("};")
    lines.append("")
    lines.append("static double raycast_distance_segments_bucketed(const Segment* segs, int count, const RayBucketView& view,")
    lines.append("                                                 double ox, double oy, double dx, double dy, double max_dist_in) {")
    lines.append("  if (!segs || count <= 0) return -1.0;")
    lines.append("  if (!view.offsets || !view.indices || view.nx <= 0 || view.ny <= 0 || view.cell_in <= 1e-6) {")
    lines.append("    return raycast_distance_segments(segs, count, ox, oy, dx, dy, max_dist_in);")
    lines.append("  }")
    lines.append("  double best = -1.0;")
    lines.append("  double step = std::max(0.25, view.cell_in * 0.5);")
    lines.append("  int steps = static_cast<int>(std::ceil(max_dist_in / step));")
    lines.append("  if (steps < 1) steps = 1;")
    lines.append("  int last_cell = -1;")
    lines.append("  for (int st = 0; st <= steps; ++st) {")
    lines.append("    double d = std::min(max_dist_in, step * static_cast<double>(st));")
    lines.append("    if (best >= 0.0 && d > best + view.cell_in) break;")
    lines.append("    double x = ox + dx * d;")
    lines.append("    double y = oy + dy * d;")
    lines.append("    int ix = static_cast<int>(std::floor((x - view.origin_x) / view.cell_in));")
    lines.append("    int iy = static_cast<int>(std::floor((y - view.origin_y) / view.cell_in));")
    lines.append("    if (ix < 0 || iy < 0 || ix >= view.nx || iy >= view.ny) continue;")
    lines.append("    int cell = iy * view.nx + ix;")
    lines.append("    if (cell == last_cell) continue;")
    lines.append("    last_cell = cell;")
    lines.append("    uint32_t begin = view.offsets[cell];")
    lines.append("    uint32_t end = view.offsets[cell + 1];")
    lines.append("    for (uint32_t k = begin; k < end; ++k) {")
    lines.append("      int si = static_cast<int>(view.indices[k]);")
    lines.append("      if (si < 0 || si >= count) continue;")
    lines.append("      double t = 0.0;")
    lines.append("      if (!ray_segment_intersect(ox, oy, dx, dy, segs[si].x0, segs[si].y0, segs[si].x1, segs[si].y1, t)) continue;")
    lines.append("      if (t <= max_dist_in && (best < 0.0 || t < best)) best = t;")
    lines.append("    }")
    lines.append("  }")
    lines.append("  if (best >= 0.0) return best;")
    lines.append("  return raycast_distance_segments(segs, count, ox, oy, dx, dy, max_dist_in);")
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
    lines.append("  const RayBucketView perim_view = {PERIM_RAY_BUCKET_OFFSETS, PERIM_RAY_BUCKET_INDICES, PERIM_RAY_BUCKET_NX, PERIM_RAY_BUCKET_NY, PERIM_RAY_BUCKET_CELL_IN, PERIM_RAY_BUCKET_ORIGIN_X, PERIM_RAY_BUCKET_ORIGIN_Y};")
    lines.append("  const RayBucketView obj_view = {OBJECT_RAY_BUCKET_OFFSETS, OBJECT_RAY_BUCKET_INDICES, OBJECT_RAY_BUCKET_NX, OBJECT_RAY_BUCKET_NY, OBJECT_RAY_BUCKET_CELL_IN, OBJECT_RAY_BUCKET_ORIGIN_X, OBJECT_RAY_BUCKET_ORIGIN_Y};")
    lines.append("  double best = -1.0;")
    lines.append("  if (mode == MCL_MAP_MODE_PERIMETER) {")
    lines.append("    best = raycast_distance_segments_bucketed(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, perim_view, ox, oy, dx, dy, max_dist_in);")
    lines.append("  } else if (mode == MCL_MAP_MODE_OBJECTS) {")
    lines.append("    if (full_object_mask) {")
    lines.append("      best = raycast_distance_segments_bucketed(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, obj_view, ox, oy, dx, dy, max_dist_in);")
    lines.append("    } else {")
    lines.append("      best = raycast_distance_objects_mask(ox, oy, dx, dy, max_dist_in, sensor_idx);")
    lines.append("    }")
    lines.append("  } else {")
    lines.append("    best = raycast_distance_segments_bucketed(PERIM_SEGMENTS, PERIM_SEGMENTS_COUNT, perim_view, ox, oy, dx, dy, max_dist_in);")
    lines.append("    double o = full_object_mask ?")
    lines.append("      raycast_distance_segments_bucketed(OBJECT_SEGMENTS, OBJECT_SEGMENTS_COUNT, obj_view, ox, oy, dx, dy, max_dist_in) :")
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
    lines.append("  : count_(MCL_PARTICLE_COUNT), w_slow_(0.0), w_fast_(0.0), forced_injection_fraction_(0.0), confidence_(0.0), ess_ratio_(1.0),")
    lines.append("    estimate_valid_(false), have_estimate_ever_(false), last_dist_used_mask_(0), segment_band_radius_(0.0),")
    lines.append("    segment_band_active_(false) {")
    lines.append("  if (MCL_KLD_ENABLED) count_ = MCL_N_MIN;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    particles_[i] = {0.0, 0.0, 0.0, 1.0 / std::max(1, count_)};")
    lines.append("  }")
    lines.append("  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("    last_dist_measured_mm_[i] = -1.0;")
    lines.append("    last_dist_expected_mm_[i] = -1.0;")
    lines.append("    last_dist_errno_[i] = 0;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::seed(unsigned int seed) {")
    lines.append("  rng_engine().seed(seed);")
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
    lines.append("  confidence_ = 0.0;")
    lines.append("  ess_ratio_ = 1.0;")
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
    lines.append("  confidence_ = 0.0;")
    lines.append("  ess_ratio_ = 1.0;")
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
    lines.append("void MCLLocalizer::predict(double dx_in, double dy_in, double dtheta_deg, double noise_scale) {")
    lines.append("  if (!MCL_USE_MOTION) return;")
    lines.append("  if (std::fabs(dx_in) + std::fabs(dy_in) + std::fabs(dtheta_deg) <= 1e-9) return;")
    lines.append("  estimate_valid_ = false;")
    lines.append("  double ns = std::isfinite(noise_scale) ? noise_scale : 1.0;")
    lines.append("  if (ns < 1.0) ns = 1.0;")
    lines.append("  double dist = std::sqrt(dx_in * dx_in + dy_in * dy_in);")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double base_th = particles_[i].theta;")
    lines.append("    double ndx = dx_in;")
    lines.append("    double ndy = dy_in;")
    lines.append("    double nth = dtheta_deg;")
    lines.append("    if (MCL_USE_ALPHA_MODEL) {")
    lines.append("      double sigma_trans = (MCL_ALPHA1 * dist + MCL_ALPHA2 * std::fabs(dtheta_deg)) * ns;")
    lines.append("      double sigma_rot = (MCL_ALPHA3 * dist + MCL_ALPHA4 * std::fabs(dtheta_deg)) * ns;")
    lines.append("      ndx += rand_gaussian(0.0, sigma_trans);")
    lines.append("      ndy += rand_gaussian(0.0, sigma_trans);")
    lines.append("      nth += rand_gaussian(0.0, sigma_rot);")
    lines.append("    } else {")
    lines.append("      ndx += rand_gaussian(0.0, MCL_MOTION_SIGMA_X_IN * ns);")
    lines.append("      ndy += rand_gaussian(0.0, MCL_MOTION_SIGMA_Y_IN * ns);")
    lines.append("      nth += rand_gaussian(0.0, MCL_MOTION_SIGMA_THETA_DEG * ns);")
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
    lines.append("void MCLLocalizer::updateDistance(const double* dist_mm, const double* dist_conf, const int* conf_meaningful, const double* dist_obj_size, const int* obj_size_valid, const int* dist_errno, int count) {")
    lines.append("  if (!MCL_USE_DISTANCE || !dist_mm || count <= 0) return;")
    lines.append("  (void)dist_obj_size;")
    lines.append("  (void)obj_size_valid;")
    lines.append("  bool have_est = have_estimate_ever_;")
    lines.append("  MCLPose est = estimate_;")
    lines.append("  if (have_est) est = estimate();")
    lines.append("  estimate_valid_ = false;")
    lines.append("  int used = std::min(count, MCL_DISTANCE_SENSOR_COUNT);")
    lines.append("  last_dist_used_mask_ = 0;")
    lines.append("  for (int i = 0; i < MCL_DISTANCE_SENSOR_COUNT_SAFE; ++i) {")
    lines.append("    last_dist_measured_mm_[i] = -1.0;")
    lines.append("    last_dist_expected_mm_[i] = -1.0;")
    lines.append("    last_dist_errno_[i] = 0;")
    lines.append("  }")
    lines.append("  for (int i = 0; i < used; ++i) {")
    lines.append("    if (dist_mm[i] >= 0.0 && dist_mm[i] < 9000.0) last_dist_measured_mm_[i] = dist_mm[i];")
    lines.append("    if (dist_errno) last_dist_errno_[i] = dist_errno[i];")
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
    lines.append("      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min <= 0.0) s_min = 20.0;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (meas > s_max) {")
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
    lines.append("      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min <= 0.0) s_min = 20.0;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (meas > s_max) {")
    lines.append("        skip_sensor_[s] = 1;")
    lines.append("        continue;")
    lines.append("      }")
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
    lines.append("  double max_log_w = -1e300;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double w = particles_[i].w;")
    lines.append("    if (w < 1e-300) w = 1e-300;")
    lines.append("    double log_w = std::log(w);")
    lines.append("    for (int s = 0; s < used; ++s) {")
    lines.append("      if (skip_sensor_[s]) continue;")
    lines.append("      const MCLDistanceSensorConfig& cfg = MCL_DISTANCE_SENSORS[s];")
    lines.append("      double meas_raw = dist_mm[s];")
    lines.append("      if (!std::isfinite(meas_raw) || meas_raw < 0.0 || meas_raw >= 9000.0) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double meas = meas_raw - cfg.bias_mm;")
    lines.append("      double s_min = (cfg.min_range_mm > 0.0) ? cfg.min_range_mm : MCL_DIST_MIN_RANGE_MM;")
    lines.append("      double s_max = (cfg.max_range_mm > 0.0) ? cfg.max_range_mm : MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_max <= 0.0) s_max = MCL_DIST_MAX_RANGE_MM;")
    lines.append("      if (s_min <= 0.0) s_min = 20.0;")
    lines.append("      if (s_min > 0.0 && meas < s_min) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (MCL_LF_IGNORE_MAX && MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD &&")
    lines.append("          meas >= s_max) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      if (meas > s_max) {")
    lines.append("        continue;")
    lines.append("      }")
    lines.append("      double off_x = 0.0, off_y = 0.0;")
    lines.append("      rotate_local_to_world(cfg.x_in, cfg.y_in, particles_[i].theta, off_x, off_y);")
    lines.append("      double ox = particles_[i].x + off_x;")
    lines.append("      double oy = particles_[i].y + off_y;")
    lines.append("      double heading = wrap_deg(particles_[i].theta + cfg.angle_deg + cfg.angle_offset_deg);")
    lines.append("      double sigma_eff = (meas <= 200.0) ? MCL_DIST_SIGMA_HIT_MM : (MCL_DIST_SIGMA_FAR_SCALE * meas);")
    lines.append("      if (sigma_eff < MCL_DIST_SIGMA_MIN_MM) sigma_eff = MCL_DIST_SIGMA_MIN_MM;")
    lines.append("      if (sigma_eff > MCL_DIST_SIGMA_MAX_MM) sigma_eff = MCL_DIST_SIGMA_MAX_MM;")
    lines.append("      if (dist_conf && conf_meaningful && conf_meaningful[s] == 1) {")
    lines.append("        double conf_min_map = MCL_DIST_CONFIDENCE_MIN;")
    lines.append("        if (conf_min_map >= 63.0) conf_min_map = 62.0;")
    lines.append("        if (conf_min_map < 0.0) conf_min_map = 0.0;")
    lines.append("        double denom = 63.0 - conf_min_map;")
    lines.append("        if (denom > 1e-9) {")
    lines.append("          double q = (dist_conf[s] - conf_min_map) / denom;")
    lines.append("          if (q < 0.0) q = 0.0;")
    lines.append("          if (q > 1.0) q = 1.0;")
    lines.append("          sigma_eff *= (1.0 + MCL_DIST_CONF_SIGMA_SCALE * (1.0 - q));")
    lines.append("        }")
    lines.append("      }")
    lines.append("      int ray_count = (MCL_DIST_FOV_MULTI_RAY ? fov_ray_count() : 1);")
    lines.append("      double half_deg = fov_half_deg(meas);")
    lines.append("      double logp_rays[9];")
    lines.append("      int ray_used = 0;")
    lines.append("      for (int r = 0; r < ray_count; ++r) {")
    lines.append("        double delta = 0.0;")
    lines.append("        if (ray_count > 1) {")
    lines.append("          delta = -half_deg + (2.0 * half_deg * static_cast<double>(r) / static_cast<double>(ray_count - 1));")
    lines.append("        }")
    lines.append("        double ray_heading = wrap_deg(heading + delta);")
    lines.append("        double w_ray = 1.0;")
    lines.append("        if (MCL_DISTANCE_MODEL == MCL_DISTANCE_MODEL_LIKELIHOOD_FIELD) {")
    lines.append("          double meas_in = meas / MCL_MM_PER_IN;")
    lines.append("          double dir_x = 0.0, dir_y = 0.0;")
    lines.append("          heading_unit(ray_heading, dir_x, dir_y);")
    lines.append("          double end_x = ox + dir_x * meas_in;")
    lines.append("          double end_y = oy + dir_y * meas_in;")
    lines.append("          double dist_mm = distance_to_map_mode(end_x, end_y, cfg.map_mode, s) * MCL_MM_PER_IN;")
    lines.append("          double p_hit = gaussian(dist_mm, 0.0, sigma_eff);")
    lines.append("          double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;")
    lines.append("          double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;")
    lines.append("          w_ray = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_MAX * p_max;")
    lines.append("          if (MCL_DIST_GATE_MM > 0.0 && dist_mm > MCL_DIST_GATE_MM) {")
    lines.append("            if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {")
    lines.append("              w_ray *= MCL_DIST_GATE_PENALTY;")
    lines.append("            } else {")
    lines.append("              w_ray = 0.0;")
    lines.append("            }")
    lines.append("          }")
    lines.append("        } else {")
    lines.append("          double expected_in = raycast_distance_mode(ox, oy, ray_heading, s_max / MCL_MM_PER_IN, cfg.map_mode, s);")
    lines.append("          if (expected_in < 0.0) expected_in = s_max / MCL_MM_PER_IN;")
    lines.append("          double expected_mm = expected_in * MCL_MM_PER_IN;")
    lines.append("          bool gated = (MCL_DIST_GATE_MM > 0.0 && std::fabs(meas - expected_mm) > MCL_DIST_GATE_MM);")
    lines.append("          double error = meas - expected_mm;")
    lines.append("          double p_hit = gaussian(error, 0.0, sigma_eff);")
    lines.append("          double p_rand = (meas >= 0.0 && meas <= s_max) ? (1.0 / s_max) : 0.0;")
    lines.append("          double p_short = 0.0;")
    lines.append("          if (MCL_DIST_W_SHORT > 0.0 && meas >= 0.0 && meas <= expected_mm) {")
    lines.append("            p_short = MCL_DIST_LAMBDA_SHORT * std::exp(-MCL_DIST_LAMBDA_SHORT * meas);")
    lines.append("          }")
    lines.append("          double p_max = (MCL_DIST_W_MAX > 0.0 && meas >= s_max) ? 1.0 : 0.0;")
    lines.append("          w_ray = MCL_DIST_W_HIT * p_hit + MCL_DIST_W_RAND * p_rand + MCL_DIST_W_SHORT * p_short + MCL_DIST_W_MAX * p_max;")
    lines.append("          if (gated) {")
    lines.append("            if (MCL_DIST_GATE_MODE == MCL_GATE_MODE_SOFT) {")
    lines.append("              w_ray *= MCL_DIST_GATE_PENALTY;")
    lines.append("            } else {")
    lines.append("              w_ray = 0.0;")
    lines.append("            }")
    lines.append("          }")
    lines.append("        }")
    lines.append("        if (MCL_DIST_MIN_SENSOR_WEIGHT > 0.0 && w_ray < MCL_DIST_MIN_SENSOR_WEIGHT) w_ray = MCL_DIST_MIN_SENSOR_WEIGHT;")
    lines.append("        if (w_ray < 1e-300) w_ray = 1e-300;")
    lines.append("        if (ray_used < 9) logp_rays[ray_used++] = std::log(w_ray);")
    lines.append("      }")
    lines.append("      if (ray_used > 0) {")
    lines.append("        log_w += logmeanexp(logp_rays, ray_used);")
    lines.append("      }")
    lines.append("    }")
    lines.append("    tmp_weights_[i] = log_w;")
    lines.append("    if (log_w > max_log_w) max_log_w = log_w;")
    lines.append("  }")
    lines.append("  if (!std::isfinite(max_log_w)) return;")
    lines.append("  double total = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double w = std::exp(tmp_weights_[i] - max_log_w);")
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
    lines.append("    ess_ratio_ = 1.0;")
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
    lines.append("  ess_ratio_ = std::max(0.0, std::min(1.0, frac));")
    lines.append("  confidence_ = std::max(0.0, std::min(1.0, 1.0 - ess_ratio_));")
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
    lines.append("  if (MCL_MODE_SPLIT_ENABLED && confidence_ <= MCL_MODE_SPLIT_CONF_MAX && count_ >= 4) {")
    lines.append("    int seed_a = 0;")
    lines.append("    int seed_b = 0;")
    lines.append("    double best_w = particles_[0].w;")
    lines.append("    for (int i = 1; i < count_; ++i) {")
    lines.append("      if (particles_[i].w > best_w) {")
    lines.append("        best_w = particles_[i].w;")
    lines.append("        seed_a = i;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    double far_d2 = -1.0;")
    lines.append("    for (int i = 0; i < count_; ++i) {")
    lines.append("      double dx = particles_[i].x - particles_[seed_a].x;")
    lines.append("      double dy = particles_[i].y - particles_[seed_a].y;")
    lines.append("      double d2 = dx * dx + dy * dy;")
    lines.append("      if (d2 > far_d2) {")
    lines.append("        far_d2 = d2;")
    lines.append("        seed_b = i;")
    lines.append("      }")
    lines.append("    }")
    lines.append("    double cx[2] = {particles_[seed_a].x, particles_[seed_b].x};")
    lines.append("    double cy[2] = {particles_[seed_a].y, particles_[seed_b].y};")
    lines.append("    double mass[2] = {0.0, 0.0};")
    lines.append("    double sx[2] = {0.0, 0.0};")
    lines.append("    double sy[2] = {0.0, 0.0};")
    lines.append("    double ss[2] = {0.0, 0.0};")
    lines.append("    double cc[2] = {0.0, 0.0};")
    lines.append("    for (int iter = 0; iter < 2; ++iter) {")
    lines.append("      mass[0] = mass[1] = 0.0;")
    lines.append("      sx[0] = sx[1] = 0.0;")
    lines.append("      sy[0] = sy[1] = 0.0;")
    lines.append("      ss[0] = ss[1] = 0.0;")
    lines.append("      cc[0] = cc[1] = 0.0;")
    lines.append("      for (int i = 0; i < count_; ++i) {")
    lines.append("        double dx0 = particles_[i].x - cx[0];")
    lines.append("        double dy0 = particles_[i].y - cy[0];")
    lines.append("        double dx1 = particles_[i].x - cx[1];")
    lines.append("        double dy1 = particles_[i].y - cy[1];")
    lines.append("        int k = ((dx1 * dx1 + dy1 * dy1) < (dx0 * dx0 + dy0 * dy0)) ? 1 : 0;")
    lines.append("        double w = particles_[i].w;")
    lines.append("        mass[k] += w;")
    lines.append("        sx[k] += particles_[i].x * w;")
    lines.append("        sy[k] += particles_[i].y * w;")
    lines.append("        double th = particles_[i].theta * DEG_TO_RAD;")
    lines.append("        ss[k] += std::sin(th) * w;")
    lines.append("        cc[k] += std::cos(th) * w;")
    lines.append("      }")
    lines.append("      for (int k = 0; k < 2; ++k) {")
    lines.append("        if (mass[k] > 1e-12) {")
    lines.append("          cx[k] = sx[k] / mass[k];")
    lines.append("          cy[k] = sy[k] / mass[k];")
    lines.append("        }")
    lines.append("      }")
    lines.append("    }")
    lines.append("    double min_mass = MCL_MODE_SPLIT_MIN_MASS;")
    lines.append("    if (min_mass < 0.0) min_mass = 0.0;")
    lines.append("    if (min_mass > 0.49) min_mass = 0.49;")
    lines.append("    double sep_min2 = MCL_MODE_SPLIT_MIN_SEPARATION_IN * MCL_MODE_SPLIT_MIN_SEPARATION_IN;")
    lines.append("    double cdx = cx[0] - cx[1];")
    lines.append("    double cdy = cy[0] - cy[1];")
    lines.append("    double sep2 = cdx * cdx + cdy * cdy;")
    lines.append("    if (mass[0] >= min_mass && mass[1] >= min_mass && sep2 >= sep_min2) {")
    lines.append("      int keep = (mass[1] > mass[0]) ? 1 : 0;")
    lines.append("      if (mass[keep] > 1e-12) {")
    lines.append("        estimate_.x = sx[keep] / mass[keep];")
    lines.append("        estimate_.y = sy[keep] / mass[keep];")
    lines.append("        estimate_.theta = wrap_deg(std::atan2(ss[keep], cc[keep]) * RAD_TO_DEG);")
    lines.append("      }")
    lines.append("    }")
    lines.append("  }")
    lines.append("  if (MCL_CGR_LITE_ENABLED) cgrLiteRefineEstimate();")
    lines.append("  estimate_valid_ = true;")
    lines.append("  have_estimate_ever_ = true;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::cgrLiteRefineEstimate() {")
    lines.append("  if (!MCL_CGR_LITE_ENABLED || count_ <= 0) return;")
    lines.append("  int top_k = std::max(1, std::min(MCL_CGR_LITE_TOP_K, count_));")
    lines.append("  int max_iters = std::max(1, MCL_CGR_LITE_MAX_ITERS);")
    lines.append("  double budget_ms = MCL_CGR_LITE_BUDGET_MS;")
    lines.append("  if (!std::isfinite(budget_ms) || budget_ms < 0.0) budget_ms = 0.0;")
    lines.append("  std::uint32_t t0 = pros::millis();")
    lines.append("  double cx = estimate_.x;")
    lines.append("  double cy = estimate_.y;")
    lines.append("  double ct = estimate_.theta;")
    lines.append("  const double sigma_xy = std::max(2.0, MCL_MODE_SPLIT_MIN_SEPARATION_IN * 0.5);")
    lines.append("  const double sigma_th = 20.0;")
    lines.append("  for (int iter = 0; iter < max_iters; ++iter) {")
    lines.append("    if (budget_ms > 0.0) {")
    lines.append("      std::uint32_t elapsed = pros::millis() - t0;")
    lines.append("      if (static_cast<double>(elapsed) >= budget_ms) break;")
    lines.append("    }")
    lines.append("    for (int j = 0; j < top_k; ++j) {")
    lines.append("      tmp_weights_[j] = -1.0;")
    lines.append("      cdf_buf_[j] = 0.0;")
    lines.append("    }")
    lines.append("    for (int i = 0; i < count_; ++i) {")
    lines.append("      double dx = particles_[i].x - cx;")
    lines.append("      double dy = particles_[i].y - cy;")
    lines.append("      double dth = angle_diff_deg(particles_[i].theta, ct);")
    lines.append("      double score = particles_[i].w * std::exp(-0.5 * ((dx * dx + dy * dy) / (sigma_xy * sigma_xy) + (dth * dth) / (sigma_th * sigma_th)));")
    lines.append("      int worst = 0;")
    lines.append("      for (int j = 1; j < top_k; ++j) {")
    lines.append("        if (tmp_weights_[j] < tmp_weights_[worst]) worst = j;")
    lines.append("      }")
    lines.append("      if (score > tmp_weights_[worst]) {")
    lines.append("        tmp_weights_[worst] = score;")
    lines.append("        cdf_buf_[worst] = static_cast<double>(i);")
    lines.append("      }")
    lines.append("    }")
    lines.append("    double sw = 0.0, sx = 0.0, sy = 0.0, ss = 0.0, cc = 0.0;")
    lines.append("    for (int j = 0; j < top_k; ++j) {")
    lines.append("      double w = tmp_weights_[j];")
    lines.append("      if (!(w > 0.0)) continue;")
    lines.append("      int idx = static_cast<int>(cdf_buf_[j]);")
    lines.append("      if (idx < 0 || idx >= count_) continue;")
    lines.append("      sw += w;")
    lines.append("      sx += particles_[idx].x * w;")
    lines.append("      sy += particles_[idx].y * w;")
    lines.append("      double th = particles_[idx].theta * DEG_TO_RAD;")
    lines.append("      ss += std::sin(th) * w;")
    lines.append("      cc += std::cos(th) * w;")
    lines.append("    }")
    lines.append("    if (sw <= 1e-12) break;")
    lines.append("    cx = sx / sw;")
    lines.append("    cy = sy / sw;")
    lines.append("    ct = wrap_deg(std::atan2(ss, cc) * RAD_TO_DEG);")
    lines.append("  }")
    lines.append("  estimate_.x = cx;")
    lines.append("  estimate_.y = cy;")
    lines.append("  estimate_.theta = ct;")
    lines.append("}")
    lines.append("")
    lines.append("double MCLLocalizer::effectiveN() const {")
    lines.append("  double denom = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) denom += particles_[i].w * particles_[i].w;")
    lines.append("  return (denom <= 1e-12) ? 0.0 : (1.0 / denom);")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::getLastDistanceDebug(double* measured_mm, double* expected_mm, int* errno_codes, int capacity, uint32_t* used_mask) const {")
    lines.append("  int n = std::min(capacity, MCL_DISTANCE_SENSOR_COUNT_SAFE);")
    lines.append("  for (int i = 0; i < n; ++i) {")
    lines.append("    if (measured_mm) measured_mm[i] = last_dist_measured_mm_[i];")
    lines.append("    if (expected_mm) expected_mm[i] = last_dist_expected_mm_[i];")
    lines.append("    if (errno_codes) errno_codes[i] = last_dist_errno_[i];")
    lines.append("  }")
    lines.append("  if (used_mask) *used_mask = last_dist_used_mask_;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::copyWeights(double* out_weights, int capacity) const {")
    lines.append("  if (!out_weights || capacity <= 0) return;")
    lines.append("  int n = std::min(capacity, count_);")
    lines.append("  for (int i = 0; i < n; ++i) out_weights[i] = particles_[i].w;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::setWeights(const double* in_weights, int capacity) {")
    lines.append("  if (!in_weights || capacity <= 0) return;")
    lines.append("  int n = std::min(capacity, count_);")
    lines.append("  double total = 0.0;")
    lines.append("  for (int i = 0; i < n; ++i) {")
    lines.append("    double w = in_weights[i];")
    lines.append("    if (!std::isfinite(w) || w < 0.0) w = 0.0;")
    lines.append("    particles_[i].w = w;")
    lines.append("    total += w;")
    lines.append("  }")
    lines.append("  for (int i = n; i < count_; ++i) particles_[i].w = 0.0;")
    lines.append("  if (total <= 1e-12) {")
    lines.append("    for (int i = 0; i < count_; ++i) particles_[i].w = 1.0 / std::max(1, count_);")
    lines.append("  } else {")
    lines.append("    for (int i = 0; i < count_; ++i) particles_[i].w /= total;")
    lines.append("  }")
    lines.append("  estimate_valid_ = false;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::setForcedInjectionFraction(double frac) {")
    lines.append("  if (!std::isfinite(frac)) frac = 0.0;")
    lines.append("  if (frac < 0.0) frac = 0.0;")
    lines.append("  if (frac > 1.0) frac = 1.0;")
    lines.append("  forced_injection_fraction_ = frac;")
    lines.append("}")
    lines.append("")
    lines.append("void MCLLocalizer::estimateCovariance(MCLPose* pose_out, double cov_out[3][3]) {")
    lines.append("  MCLPose est = estimate();")
    lines.append("  if (pose_out) *pose_out = est;")
    lines.append("  if (!cov_out) return;")
    lines.append("  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) cov_out[r][c] = 0.0;")
    lines.append("  for (int i = 0; i < count_; ++i) {")
    lines.append("    double w = particles_[i].w;")
    lines.append("    double dx = particles_[i].x - est.x;")
    lines.append("    double dy = particles_[i].y - est.y;")
    lines.append("    double dth = angle_diff_deg(particles_[i].theta, est.theta);")
    lines.append("    cov_out[0][0] += w * dx * dx;")
    lines.append("    cov_out[0][1] += w * dx * dy;")
    lines.append("    cov_out[0][2] += w * dx * dth;")
    lines.append("    cov_out[1][0] += w * dy * dx;")
    lines.append("    cov_out[1][1] += w * dy * dy;")
    lines.append("    cov_out[1][2] += w * dy * dth;")
    lines.append("    cov_out[2][0] += w * dth * dx;")
    lines.append("    cov_out[2][1] += w * dth * dy;")
    lines.append("    cov_out[2][2] += w * dth * dth;")
    lines.append("  }")
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
    lines.append("  double inj = 0.0;")
    lines.append("  if (MCL_AUGMENTED_ENABLED) {")
    lines.append("    if (w_slow_ > 1e-9 && w_fast_ > 1e-9) {")
    lines.append("      inj = std::max(0.0, 1.0 - w_fast_ / w_slow_);")
    lines.append("    }")
    lines.append("  } else {")
    lines.append("    inj = MCL_RANDOM_INJECTION;")
    lines.append("  }")
    lines.append("  if (forced_injection_fraction_ > inj) inj = forced_injection_fraction_;")
    lines.append("  if (inj > 1.0) inj = 1.0;")
    lines.append("  for (int i = 0; i < target; ++i) {")
    lines.append("    if (inj > 0.0 && rand_uniform() < inj) {")
    lines.append("      PoseSample p = sample_random_pose();")
    lines.append("      resample_buf_[i].x = p.x;")
    lines.append("      resample_buf_[i].y = p.y;")
    lines.append("      resample_buf_[i].theta = p.theta;")
    lines.append("      resample_buf_[i].w = 1.0 / std::max(1, target);")
    lines.append("    }")
    lines.append("  }")
    lines.append("  if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0 || MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {")
    lines.append("    for (int i = 0; i < target; ++i) {")
    lines.append("      if (MCL_RESAMPLE_ROUGHEN_XY_IN > 0.0) {")
    lines.append("        resample_buf_[i].x += rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_XY_IN);")
    lines.append("        resample_buf_[i].y += rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_XY_IN);")
    lines.append("      }")
    lines.append("      if (MCL_RESAMPLE_ROUGHEN_THETA_DEG > 0.0) {")
    lines.append("        resample_buf_[i].theta = wrap_deg(resample_buf_[i].theta + rand_gaussian(0.0, MCL_RESAMPLE_ROUGHEN_THETA_DEG));")
    lines.append("      }")
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
'''

    tune_log_h = r'''#pragma once

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
'''

    tune_log_cpp = r'''#include "mcl_tune_log.h"
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
'''

    tune_wizard_h = r'''#pragma once

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
    dist_model = str(dist_cfg.get("model", "likelihood_field")).strip().lower()
    if dist_model not in ("likelihood_field", "beam"):
        dist_model = "likelihood_field"
    dist_sigma_hit = float(dist_cfg.get("sigma_hit_mm", 15.0)) if isinstance(dist_cfg, dict) else 15.0
    dist_sigma_far = float(dist_cfg.get("sigma_far_scale", 0.05)) if isinstance(dist_cfg, dict) else 0.05
    dist_gate_mm = float(dist_cfg.get("gate_mm", 150.0)) if isinstance(dist_cfg, dict) else 150.0
    dist_innov_gate_mm = float(dist_cfg.get("innovation_gate_mm", 0.0)) if isinstance(dist_cfg, dict) else 0.0
    ekf_cfg = mcl.get("ekf", {}) if isinstance(mcl, dict) else {}
    ekf_on = int(ekf_cfg.get("enabled", 1)) == 1 if isinstance(ekf_cfg, dict) else True
    corr_cfg = mcl.get("correction", {}) if isinstance(mcl, dict) else {}
    corr_alpha_min = float(corr_cfg.get("alpha_min", 0.05)) if isinstance(corr_cfg, dict) else 0.05
    corr_alpha_max = float(corr_cfg.get("alpha_max", 0.25)) if isinstance(corr_cfg, dict) else 0.25
    corr_min_conf = float(corr_cfg.get("min_confidence", 0.6)) if isinstance(corr_cfg, dict) else 0.6
    conf_cfg = mcl.get("confidence", {}) if isinstance(mcl, dict) else {}
    conf_metric = str(conf_cfg.get("metric", "peakedness")).strip().lower() if isinstance(conf_cfg, dict) else "peakedness"
    parts = mcl.get("particles", {}) if isinstance(mcl, dict) else {}
    dist_sensors = get_distance_sensors(cfg)

    particle_n = int(parts.get("n", parts.get("n_min", 200)))
    particle_n_min = int(parts.get("n_min", particle_n))
    particle_n_max = int(parts.get("n_max", max(particle_n, particle_n_min)))
    rates_cfg = mcl.get("rates", {}) if isinstance(mcl.get("rates", {}), dict) else {}
    loop_ms = int(mcl.get("loop_ms", 10))
    stall_ms = int(mcl.get("stall_ms", max(1, 2 * loop_ms)))
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
        "This runtime uses an internal Atticus field frame, plus explicit external pose adapters.\n\n"
        "## Axes and heading\n"
        "- Internal MCL frame: +x is forward, +y is left, heading is clockwise-positive.\n"
        "- Internal heading uses 0 deg = +y (left), 90 deg = +x (forward).\n"
        "- External adapters are configured by `mcl.interop`:\n"
        "  - `pose_convention`: `atticus` / `cw_zero_forward` / `ccw_zero_forward`\n"
        "  - `swap_xy`, `invert_x`, `invert_y` for library axis mapping.\n\n"
        "## Frame expectations by API\n"
        "- `setPose(x, y, theta)`: field frame pose in inches + degrees.\n"
        "- `setOdomDelta(dx, dy, dtheta)`: robot-frame delta since last update (`dx` forward, `dy` left, `dtheta` CW deg).\n"
        "- Never pass field-frame deltas into `setOdomDelta`.\n"
        "- `getPose()` / `getFusedPose()`: field frame pose.\n\n"
        "## Practical sanity checks\n"
        "1. Drive straight forward: `dx` should be positive and `dy` near zero.\n"
        "2. Strafe/slide left (if possible): `dy` should be positive.\n"
        "3. Turn right in place: `dtheta` should be positive.\n"
        "4. Run `ProsMCL::runFrameSanityCheck(...)` using start/forward/turn poses to verify adapter settings.\n"
        "5. If signs are wrong, fix adapter/odom signs first (do not tune noise to hide sign errors).\n"
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
        "Map/raycast blobs include deterministic hash + dimension metadata and are checked at runtime.\n"
        "On mismatch, map-dependent distance updates are disabled fail-safe while odom/IMU fusion continues.\n\n"
        "## Current export profile\n"
        f"- Particle budget: `{particle_n}` (`n_min={particle_n_min}`, `n_max={particle_n_max}`)\n"
        f"- Runtime cadence targets: loop `{loop_ms} ms`, stall `{stall_ms} ms`, motion `{motion_ms} ms`, sensor `{sensor_ms} ms`\n"
        f"- Distance sensors: {dist_status} (`{len(dist_sensors)}` configured), model `{dist_model}`\n"
        f"- EKF: {ekf_status}\n\n"
        "## Fast-start integration checklist\n"
        "1. Create one global `ProsMCL` instance with your IMU + distance sensor ports.\n"
        "2. In `initialize()`, call `startEasyExternal(seed, initial_heading_deg, x, y, theta)` so IMU calibration does not consume autonomous time.\n"
        "3. Choose one odometry feed mode:\n"
        "   - manual: call `setOdomDelta(...)` in your 10 ms control loop\n"
        "   - automatic: set `setFieldPoseProvider(...)` once (best for LemLib)\n"
        "   Runtime treats these modes as exclusive; `setOdomDelta` is ignored while provider mode is active.\n"
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
        "4. External frame adapter: run `runFrameSanityCheck(...)` and set `mcl.interop` (`pose_convention`, `swap_xy`, `invert_x`, `invert_y`) from the recommended result.\n"
        "5. Track width: run 360 deg in-place turn, compare measured vs expected heading.\n"
        "   - If under-rotating, decrease track width by 2-5%.\n"
        "   - If over-rotating, increase track width by 2-5%.\n"
        "   - Repeat until 360 deg test error <= 3 deg.\n\n"
        "## 3) Runtime integration baseline\n"
        "1. Create one global `ProsMCL` instance.\n"
        "2. Call `startEasyExternal(...)` once with correct start tile pose.\n"
        "3. Feed odom one way only:\n"
        "   - Manual `setOdomDelta(...)` every 10 ms, or\n"
        "   - LemLib provider via `setFieldPoseProvider(...)`.\n"
        "   - Manual deltas are ignored while provider mode is active.\n"
        "   - If provider heading already comes from IMU, set `mcl.ekf.use_imu_update=0` to avoid IMU double-counting.\n"
        "4. Use `getFusedPose()` for autonomous decisions.\n\n"
        "## 4) Baseline values (good first pass)\n"
        f"- Particles: `n={particle_n}`, `n_min={particle_n_min}`, `n_max={particle_n_max}`.\n"
        "- Motion: `sigma_x_in=0.12`, `sigma_y_in=0.12`, `sigma_theta_deg=1.0`.\n"
        f"- Distance: `sigma_hit_mm={dist_sigma_hit:g}`, `sigma_far_scale={dist_sigma_far:g}`, `gate_mm={dist_gate_mm:g}`, `innovation_gate_mm={dist_innov_gate_mm:g}`.\n"
        f"- Correction: `alpha_min={corr_alpha_min:g}`, `alpha_max={corr_alpha_max:g}`, `min_confidence={corr_min_conf:g}`.\n\n"
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
        f"   - Start at `{corr_alpha_min:g}/{corr_alpha_max:g}`.\n"
        "   - If recovery after drift is too slow, raise both by +0.01.\n"
        "   - If oscillation appears, lower `alpha_max` by -0.02 first.\n"
        f"   - Keep `alpha_max <= {max(0.18, corr_alpha_max):g}` for most robots.\n"
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
        "### `startEasyExternal(unsigned seed, double initial_heading_deg, double start_x_in, double start_y_in, double start_theta_deg)`\n"
        "- Same as `startEasy`, but inputs are interpreted in external convention (`mcl.interop`).\n"
        "- Recommended for conventional VEX `0 deg = forward` integrations.\n\n"
        "### `startExternal(unsigned seed, double initial_heading_deg)`\n"
        "- Starts runtime where the initial heading is interpreted in external convention.\n"
        "- Useful when your odom stack already works in external frame and you want consistent startup semantics.\n\n"
        "### `stop()`\n"
        "- Stops runtime task cleanly (usually optional for normal PROS lifecycle).\n\n"
        "## State input\n"
        "### `setPose(double x_in, double y_in, double theta_deg)`\n"
        "- Re-initializes the particle cloud around the pose in field frame (not an exact hard lock unless set-pose sigmas are 0).\n"
        "- Use at init, explicit re-localization, or known reset points.\n\n"
        "### `setPoseExternal(double x_in, double y_in, double theta_deg)`\n"
        "- Same as `setPose`, but input pose is external-frame and converted by `mcl.interop`.\n\n"
        "### `setOdomDelta(double dx_in, double dy_in, double dtheta_deg)`\n"
        "- Robot-frame motion delta since the previous call.\n"
        "- `dx_in`: forward inches; `dy_in`: left inches; `dtheta_deg`: clockwise degrees.\n"
        "- Feed every control cycle (~10 ms) in manual mode.\n"
        "- Ignored while `setFieldPoseProvider(...)` mode is active.\n"
        "- Prefer encoder-derived `dtheta`; runtime already incorporates IMU heading updates.\n\n"
        "### `setFieldPoseProvider(FieldPoseProvider provider, void* user)` / `clearFieldPoseProvider()`\n"
        "- Auto-feed mode: provide field pose samples and runtime computes deltas internally.\n"
        "- Best low-code path for most VEX teams using existing odometry frameworks.\n"
        "- Callback output must be internal MCL frame; for external library poses call `ProsMCL::externalPoseToMCL(...)` in your provider.\n"
        "- `mcl.interop` controls those external conversion helpers (`pose_convention`, `swap_xy`, `invert_x`, `invert_y`).\n"
        "- If provider heading already uses IMU, set `mcl.ekf.use_imu_update=0` to avoid double-counting heading.\n\n"
        "### `runFrameSanityCheck(start, after_forward, after_turn, forward_target_in, turn_target_deg)`\n"
        "- Static helper that scores current adapter settings and suggests better ones.\n"
        "- Use after a quick on-robot test: set known start pose with heading near 0 external, drive forward ~12 in, then turn CW ~90 deg.\n"
        "- Inputs are external-frame poses (same convention as your provider/library).\n"
        "- Returns pass/fail for current mapping plus recommended `pose_convention`/axis flags.\n\n"
        "### `updateVision(double x_in, double y_in, double theta_deg, double confidence)`\n"
        "- Optional external absolute pose measurement (vision/camera/landmark).\n"
        "- `confidence` is [0..1] and should reflect measurement trust.\n\n"
        "### `setSegmentBand(const MCLPose* pts, int n, double radius_in)` / `clearSegmentBand()`\n"
        "- Optional temporary corridor constraint (for known route segments).\n"
        "- Use sparingly; overly tight bands can block recovery from collisions.\n\n"
        "### `freezeLocalization(bool freeze)` / `isLocalizationFrozen()`\n"
        "- Debug control: when frozen, runtime still updates baselines/timestamps but does not update particles/EKF.\n"
        "- Intended for deterministic troubleshooting without injecting stale odom deltas on unfreeze.\n\n"
        "## State output\n"
        "- Runtime updates happen automatically in the internal task.\n"
        "### `getPose()` vs `getFusedPose()`\n"
        "- `getPose()`: raw MCL estimate.\n"
        "- `getFusedPose()`: output after EKF smoothing/blending (usually best for control).\n\n"
        "### `getPoseExternal()` / `getFusedPoseExternal()`\n"
        "- Same pose outputs mapped into external convention (`mcl.interop`).\n"
        "- Recommended for direct drop-in with LemLib/JAR-style external field frames.\n\n"
        "### `applyOdomCorrection(MCLPose& odom_pose, double* out_alpha)`\n"
        "- Blends odom pose toward fused pose when EKF is enabled; otherwise toward raw MCL pose.\n"
        "- Returns `true` if correction is applied.\n"
        "- Safe-window guard can block correction during aggressive motion (`safe_max_speed_in_s`, `safe_max_turn_deg_s`).\n"
        "- Use in LemLib bridging loops to gently remove drift instead of hard snaps.\n\n"
        "### `getDebugSnapshot()`\n"
        "- Returns a lock-safe telemetry snapshot for tuning/logging.\n"
        "- Includes odom/MCL/fused poses, EKF covariance terms, peakedness confidence, ESS ratio, Neff, and distance expected/measured arrays.\n\n"
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
        "- `enabled`: allows turning motion model on/off (normally on).\n"
        "- `motion_model` / `motion_source` are normalized to `drive` / `encoders` for deterministic runtime behavior.\n\n"
        "### Distance sensor model (`mcl.sensors.distance`)\n"
        "- `enabled`: distance update active.\n"
        "- `model`: `likelihood_field` (fast, robust default) or beam model.\n"
        "- Multi-ray FOV is LF-only; beam mode is constrained to single-ray for bounded runtime.\n"
        "- Core weighting knobs: `sigma_hit_mm`, `sigma_far_scale`, `sigma_min_mm`, `sigma_max_mm`, `conf_sigma_scale`, `min_sensor_weight`.\n"
        "- Validity/gating knobs: `max_range_mm`, `min_range_mm`, `confidence_min`, `object_size_min/max`, `gate_mm`, `innovation_gate_mm`.\n"
        "- Throughput knobs: `batch_size`, `model`, LF resolution, and `raycast_bucket_in` for beam expected-distance acceleration.\n"
        "- `gate_mode`, `innovation_gate_mm`: reject outliers hard/soft.\n"
        "- Per-sensor fields: `x_in`, `y_in`, `angle_deg`, `bias_mm`, range limits, object-size filters, map mode.\n\n"
        "### IMU / Vision (`mcl.sensors.imu`, `mcl.sensors.vision`)\n"
        "- IMU: heading noise and gating behavior.\n"
        "- Vision: confidence threshold and influence of absolute fixes.\n\n"
        "### Resampling (`mcl.resample`, `mcl.kld`, `mcl.augmented`)\n"
        "- `method`: systematic/stratified/multinomial.\n"
        "- `threshold`: effective sample size trigger.\n"
        "- KLD (`epsilon`, `delta`, bin sizes): adaptive particle sizing.\n"
        "- Injection policy is deterministic: augmented injection when `mcl.augmented.enabled=1`, otherwise fixed `random_injection`.\n"
        "- Recovery LOST mode can force temporary injection (`lost_injection_fraction`) and exit after sustained confidence (`lost_exit_confidence`, `lost_exit_streak`).\n\n"
        "### CGR-lite (`mcl.cgr_lite`)\n"
        "- Optional bounded top-K local refinement on finalized posterior estimate.\n"
        "- Budget controls: `top_k`, `max_iters`, `budget_ms`.\n"
        "- Designed to be watchdog-safe: bounded iterations with per-call time cap.\n\n"
        "### Region + confidence + correction + EKF\n"
        "- `mcl.region`: hard/soft geographic constraints and penalties.\n"
        f"- `mcl.confidence`: thresholding and auto-reinit policy (metric is locked to `{conf_metric}`).\n"
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
        "- **Confidence (peakedness):** `1 - ESS/N`; higher means particles are more concentrated.\n\n"
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

    ex1 = r'''// 01_minimal_motor_encoders_imu.cpp

#include "api.h"
#include "mcl_runtime.h"
#include <cmath>
#include <vector>

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
static constexpr double kPi = 3.14159265358979323846;
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
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / kPi;

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
  // External convention here is 0 deg = forward (CW+), converted via mcl.interop.
  localizer.startEasyExternal((int)pros::millis(), 0.0, 0.0, 0.0, 0.0);

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

#include "api.h"
#include "mcl_runtime.h"
#include <cmath>
#include <vector>

// Vision-assisted autonomous example.
// This is the practical pattern when you have:
// - wheel odometry + IMU
// - distance sensors for semi-absolute anchoring
// - optional vision pose fixes for long-run drift cleanup

// -------------------------- REQUIRED USER INPUTS --------------------------
static constexpr int IMU_PORT = 10;            // Change to your IMU port.
static constexpr double TRACK_WIDTH_IN = 12.0; // Change to measured track width in inches.
static constexpr double kPi = 3.14159265358979323846;
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
  const double dtheta_deg_cw = ((dL - dR) / TRACK_WIDTH_IN) * 180.0 / kPi;
  localizer.setOdomDelta(dx_in, dy_in, dtheta_deg_cw);
}

void initialize() {
  pros::lcd::initialize();

  // Start runtime once and set known start pose.
  // Edit these values to your actual start tile.
  localizer.startEasyExternal((int)pros::millis(), 0.0, 0.0, 0.0, 0.0);

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

#include "api.h"
#include "mcl_runtime.h"
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "lemlib/api.hpp"

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
// By default they use generator-selected conversion from mcl.interop
// (pose_convention + swap_xy/invert_x/invert_y).
static MCLPose lemlib_to_mcl(const lemlib::Pose& p) {
  return ProsMCL::externalPoseToMCL(MCLPose{p.x, p.y, p.theta});
}

static void mcl_to_lemlib(const MCLPose& p, double& x, double& y, double& theta) {
  MCLPose out = ProsMCL::mclPoseToExternal(p);
  x = out.x;
  y = out.y;
  theta = out.theta;
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
  localizer.startExternal((int)pros::millis(), 0.0);
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

#include "api.h"
#include "mcl_runtime.h"
#include "mcl_tuning.h"
#include <cmath>

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
  // This provider emits internal MCL-frame pose directly, so this example
  // keeps internal initialization (`startEasy`, 90 deg forward in internal frame).
  // If your provider emits external library pose, convert with externalPoseToMCL
  // and use startEasyExternal instead.
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
