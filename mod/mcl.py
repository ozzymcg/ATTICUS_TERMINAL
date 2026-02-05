from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT
    from .draw import get_field_object_entries
    from .geom import point_in_poly, oriented_rect_corners_px, rect_oob
except Exception:
    from config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT  # type: ignore
    from draw import get_field_object_entries  # type: ignore
    from geom import point_in_poly, oriented_rect_corners_px, rect_oob  # type: ignore

MM_PER_IN = 25.4



def _wrap_deg(deg: float) -> float:
    """Handle wrap deg."""
    return float(deg) % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    """Handle angle diff deg."""
    return ((float(a) - float(b) + 180.0) % 360.0) - 180.0


def _clamp(val: float, lo: float, hi: float) -> float:
    """Handle clamp."""
    return max(lo, min(hi, val))


def in_to_px(inches: float) -> float:
    """Handle in to px."""
    return float(inches) * PPI


def px_to_in(px: float) -> float:
    """Handle px to in."""
    return float(px) / PPI


def _heading_unit(heading_deg: float) -> Tuple[float, float]:
    """Return unit vector in world pixels for heading (0=left, 90=up, clockwise+)."""
    th = math.radians(float(heading_deg))
    return (-math.cos(th), -math.sin(th))


def _heading_from_unit(dx: float, dy: float) -> float:
    """Convert world unit vector to heading (0=left, 90=up, clockwise+)."""
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return _wrap_deg(math.degrees(math.atan2(-dy, -dx)))


def _ekf_cfg(cfg: dict) -> dict:
    """Handle ekf cfg."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    ekf = mcl.get("ekf", {}) if isinstance(mcl, dict) else {}
    motion = mcl.get("motion", {}) if isinstance(mcl, dict) else {}
    sensors = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    imu_cfg = sensors.get("imu", {}) if isinstance(sensors, dict) else {}
    corr = mcl.get("correction", {}) if isinstance(mcl, dict) else {}
    set_xy = float(mcl.get("set_pose_sigma_xy_in", 0.2))
    set_th = float(mcl.get("set_pose_sigma_theta_deg", 2.0))
    return {
        "enabled": int(ekf.get("enabled", 1)) == 1,
        "mcl_min_conf": float(ekf.get("mcl_min_conf", corr.get("min_confidence", 0.6))),
        "sigma_dx_in": float(ekf.get("sigma_dx_in", motion.get("sigma_x_in", 0.1275))),
        "sigma_dy_in": float(ekf.get("sigma_dy_in", motion.get("sigma_y_in", 0.1275))),
        "sigma_dtheta_deg": float(ekf.get("sigma_dtheta_deg", motion.get("sigma_theta_deg", 1.0))),
        "imu_sigma_deg": float(ekf.get("imu_sigma_deg", imu_cfg.get("sigma_deg", 1.0))),
        "mcl_sigma_x_min": float(ekf.get("mcl_sigma_x_min", set_xy)),
        "mcl_sigma_x_max": float(ekf.get("mcl_sigma_x_max", 6.0)),
        "mcl_sigma_y_min": float(ekf.get("mcl_sigma_y_min", set_xy)),
        "mcl_sigma_y_max": float(ekf.get("mcl_sigma_y_max", 6.0)),
        "mcl_sigma_theta_min": float(ekf.get("mcl_sigma_theta_min", set_th)),
        "mcl_sigma_theta_max": float(ekf.get("mcl_sigma_theta_max", 15.0)),
        "init_sigma_xy_in": float(ekf.get("init_sigma_xy_in", set_xy)),
        "init_sigma_theta_deg": float(ekf.get("init_sigma_theta_deg", set_th)),
    }


def rotate_robot_to_world(dx_px: float, dy_px: float, heading_deg: float) -> Tuple[float, float]:
    """Rotate robot-frame offset (+x forward, +y left) into world pixel delta."""
    fx, fy = _heading_unit(heading_deg)
    lx, ly = _heading_unit(heading_deg - 90.0)
    return (dx_px * fx + dy_px * lx, dx_px * fy + dy_px * ly)


def rotate_world_to_robot(dx_px: float, dy_px: float, heading_deg: float) -> Tuple[float, float]:
    """Rotate world pixel delta into robot frame (+x forward, +y left)."""
    fx, fy = _heading_unit(heading_deg)
    lx, ly = _heading_unit(heading_deg - 90.0)
    rx = dx_px * fx + dy_px * fy
    ry = dx_px * lx + dy_px * ly
    return rx, ry


def _robot_corners_px(cfg: dict, x_px: float, y_px: float, heading_deg: float) -> List[Tuple[float, float]]:
    """Handle robot corners px."""
    bd = cfg.get("bot_dimensions", {}) if isinstance(cfg, dict) else {}
    width = float(bd.get("width", 14.0) or 14.0)
    length = float(bd.get("length", 14.0) or 14.0)
    off_x = float(bd.get("full_offset_x_in", 0.0) or 0.0)
    off_y = float(bd.get("full_offset_y_in", 0.0) or 0.0)
    return oriented_rect_corners_px((x_px, y_px), heading_deg, width, length, off_x, off_y)


def _pose_outside_perimeter(cfg: dict, x_px: float, y_px: float, heading_deg: float) -> bool:
    """Handle pose outside perimeter."""
    try:
        corners = _robot_corners_px(cfg, x_px, y_px, heading_deg)
        return rect_oob(corners, 0.0, WINDOW_WIDTH, WINDOW_HEIGHT)
    except Exception:
        return x_px < 0.0 or y_px < 0.0 or x_px > WINDOW_WIDTH or y_px > WINDOW_HEIGHT


def _dist_point_to_seg_sq(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    """Handle dist point to seg sq."""
    dx = x1 - x0
    dy = y1 - y0
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return (px - x0) ** 2 + (py - y0) ** 2
    t = ((px - x0) * dx + (py - y0) * dy) / max(1e-9, dx * dx + dy * dy)
    t = _clamp(t, 0.0, 1.0)
    cx = x0 + t * dx
    cy = y0 + t * dy
    return (px - cx) ** 2 + (py - cy) ** 2


def _polyline_dist_sq(px: float, py: float, pts: List[Tuple[float, float]]) -> float:
    """Handle polyline dist sq."""
    if len(pts) < 2:
        if not pts:
            return 1e18
        return (px - pts[0][0]) ** 2 + (py - pts[0][1]) ** 2
    best = 1e18
    for i in range(len(pts) - 1):
        d2 = _dist_point_to_seg_sq(px, py, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
        if d2 < best:
            best = d2
    return best


def _ray_segment_intersection(ox: float, oy: float, dx: float, dy: float,
                              x0: float, y0: float, x1: float, y1: float) -> Optional[float]:
    """Return t for ray intersection if it hits the segment, else None."""
    rx, ry = dx, dy
    sx, sy = x1 - x0, y1 - y0
    rxs = rx * sy - ry * sx
    if abs(rxs) < 1e-9:
        return None
    qpx, qpy = x0 - ox, y0 - oy
    t = (qpx * sy - qpy * sx) / rxs
    u = (qpx * ry - qpy * rx) / rxs
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def raycast_distance(segments: List[Tuple[float, float, float, float]],
                     origin: Tuple[float, float], heading_deg: float,
                     max_dist_px: float) -> Optional[float]:
    """Raycast from origin with heading (deg). Returns distance in px or None."""
    dx, dy = _heading_unit(heading_deg)
    best = None
    for x0, y0, x1, y1 in segments:
        t = _ray_segment_intersection(origin[0], origin[1], dx, dy, x0, y0, x1, y1)
        if t is None:
            continue
        if t <= max_dist_px and (best is None or t < best):
            best = t
    return best



def _entry_default_enabled(cfg: dict, entry: dict) -> bool:
    """Handle entry default enabled."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    map_defaults = mcl.get("map_objects", {}) if isinstance(mcl, dict) else {}
    kind = entry.get("kind", "")
    kind_key = {
        "long_goal": "long_goals",
        "long_goal_brace": "long_goal_braces",
        "center_goal": "center_goals",
        "matchloader": "matchloaders",
        "park_zone": "park_zones",
        "perimeter": "perimeter",
    }.get(kind, kind)
    return bool(int(map_defaults.get(kind_key, 1)))


def _entry_enabled(cfg: dict, entry: dict) -> bool:
    """Handle entry enabled."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    selection = mcl.get("object_selection", {}) if isinstance(mcl, dict) else {}
    obj_id = entry.get("id")
    if isinstance(selection, dict) and obj_id in selection:
        return bool(int(selection.get(obj_id, 0)))
    return _entry_default_enabled(cfg, entry)


def _perimeter_segments() -> List[Tuple[float, float, float, float]]:
    """Handle perimeter segments."""
    w, h = float(WINDOW_WIDTH), float(WINDOW_HEIGHT)
    return [
        (0.0, 0.0, w, 0.0),
        (w, 0.0, w, h),
        (w, h, 0.0, h),
        (0.0, h, 0.0, 0.0),
    ]


def _poly_to_segments(poly: List[Tuple[float, float]]) -> List[Tuple[float, float, float, float]]:
    """Handle poly to segments."""
    if not poly or len(poly) < 2:
        return []
    segs = []
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        segs.append((float(x0), float(y0), float(x1), float(y1)))
    return segs


def build_perimeter_segments(cfg: dict) -> List[Tuple[float, float, float, float]]:
    """Build perimeter segments."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    map_defaults = mcl.get("map_objects", {}) if isinstance(mcl, dict) else {}
    if int(map_defaults.get("perimeter", 1)) != 1:
        return []
    return _perimeter_segments()


def build_object_segments_by_id(cfg: dict) -> Dict[str, List[Tuple[float, float, float, float]]]:
    """Build object segments by id."""
    segs_by_id: Dict[str, List[Tuple[float, float, float, float]]] = {}
    for entry in get_field_object_entries(cfg):
        if not _entry_enabled(cfg, entry):
            continue
        obj_id = str(entry.get("id", "") or "").strip()
        if not obj_id:
            continue
        poly = entry.get("poly", [])
        if not poly:
            continue
        segs = _poly_to_segments(poly)
        if not segs:
            continue
        segs_by_id[obj_id] = segs
    return segs_by_id


def build_object_segments(cfg: dict) -> List[Tuple[float, float, float, float]]:
    """Build object segments."""
    segs: List[Tuple[float, float, float, float]] = []
    for obj_segs in build_object_segments_by_id(cfg).values():
        segs.extend(obj_segs)
    return segs


def build_map_segments(cfg: dict) -> List[Tuple[float, float, float, float]]:
    """Build map segments."""
    segs: List[Tuple[float, float, float, float]] = []
    segs.extend(build_perimeter_segments(cfg))
    segs.extend(build_object_segments(cfg))
    return segs


def _default_distance_sensors(cfg: dict) -> List[dict]:
    """Handle default distance sensors."""
    bd = cfg.get("bot_dimensions", {}) if isinstance(cfg, dict) else {}
    length = float(bd.get("length", 14.0) or 14.0)
    width = float(bd.get("width", 14.0) or 14.0)
    return [
        {"name": "front", "x_in": length * 0.5, "y_in": 0.0, "angle_deg": 0.0, "enabled": 1,
         "bias_mm": 0.0, "angle_offset_deg": 0.0, "map_mode": "both"},
        {"name": "left", "x_in": 0.0, "y_in": width * 0.5, "angle_deg": 90.0, "enabled": 1,
         "bias_mm": 0.0, "angle_offset_deg": 0.0, "map_mode": "both"},
        {"name": "back", "x_in": -length * 0.5, "y_in": 0.0, "angle_deg": 180.0, "enabled": 0,
         "bias_mm": 0.0, "angle_offset_deg": 0.0, "map_mode": "both"},
        {"name": "right", "x_in": 0.0, "y_in": -width * 0.5, "angle_deg": 270.0, "enabled": 0,
         "bias_mm": 0.0, "angle_offset_deg": 0.0, "map_mode": "both"},
    ]


def get_distance_sensors(cfg: dict, include_disabled: bool = False) -> List[dict]:
    """Return distance sensors."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    sensor_obj_vis_cfg = mcl.get("sensor_object_visibility", {}) if isinstance(mcl, dict) else {}
    if not isinstance(sensor_obj_vis_cfg, dict):
        sensor_obj_vis_cfg = {}
    def _sensor_vis(idx: int, name: str):
        """Handle sensor vis."""
        vis = sensor_obj_vis_cfg.get(str(idx), {})
        if not isinstance(vis, dict):
            vis = sensor_obj_vis_cfg.get(name, {})
        if not isinstance(vis, dict):
            vis = sensor_obj_vis_cfg.get(f"sensor_{idx + 1}", {})
        if isinstance(vis, dict):
            return vis
        return {}
    def _num(v, d=0.0):
        """Handle num."""
        try:
            return float(v)
        except Exception:
            return float(d)
    def _s_default(sensor: dict) -> dict:
        """Handle s default."""
        return {
            "bias_mm": _num(sensor.get("bias_mm", 0.0)),
            "angle_offset_deg": _num(sensor.get("angle_offset_deg", 0.0)),
            "min_range_mm": _num(sensor.get("min_range_mm", dist_cfg.get("min_range_mm", 0.0))),
            "max_range_mm": _num(sensor.get("max_range_mm", dist_cfg.get("max_range_mm", 0.0))),
            "min_confidence": _num(sensor.get("min_confidence", dist_cfg.get("confidence_min", 0.0))),
            "min_object_size": _num(sensor.get("min_object_size", dist_cfg.get("object_size_min", 0.0))),
            "max_object_size": _num(sensor.get("max_object_size", dist_cfg.get("object_size_max", 0.0))),
            "innovation_gate_mm": _num(sensor.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 0.0))),
            "map_mode": str(sensor.get("map_mode", "both")).strip().lower() or "both",
        }
    geom = mcl.get("sensor_geometry", {}) if isinstance(mcl, dict) else {}
    sensors = geom.get("distance_sensors", []) if isinstance(geom, dict) else []
    if isinstance(sensors, list) and sensors:
        out = []
        had_entry = False
        for idx, sensor in enumerate(sensors):
            if not isinstance(sensor, dict):
                continue
            had_entry = True
            merged = dict(sensor)
            merged.update(_s_default(sensor))
            name = str(merged.get("name", "")).strip()
            vis = _sensor_vis(idx, name)
            if isinstance(vis, dict):
                merged["object_visibility"] = dict(vis)
            merged["enabled"] = int(sensor.get("enabled", 1))
            if not include_disabled and merged["enabled"] != 1:
                continue
            out.append(merged)
        if had_entry:
            if include_disabled:
                defaults = _default_distance_sensors(cfg)
                for idx in range(len(out), len(defaults)):
                    entry = dict(defaults[idx])
                    entry.update(_s_default(entry))
                    name = str(entry.get("name", "")).strip()
                    vis = _sensor_vis(idx, name)
                    if isinstance(vis, dict):
                        entry["object_visibility"] = dict(vis)
                    entry["enabled"] = int(entry.get("enabled", 1))
                    out.append(entry)
            return out
    defaults = _default_distance_sensors(cfg)
    merged_defaults = []
    for idx, sensor in enumerate(defaults):
        entry = dict(sensor)
        entry.update(_s_default(sensor))
        name = str(entry.get("name", "")).strip()
        vis = _sensor_vis(idx, name)
        if isinstance(vis, dict):
            entry["object_visibility"] = dict(vis)
        entry["enabled"] = int(entry.get("enabled", 1))
        merged_defaults.append(entry)
    if include_disabled:
        return merged_defaults
    return [s for s in merged_defaults if int(s.get("enabled", 1)) == 1]



def _build_likelihood_field(cfg: dict, segments: List[Tuple[float, float, float, float]]) -> dict:
    """Build likelihood field."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    lf_cfg = dist_cfg.get("likelihood_field", {}) if isinstance(dist_cfg, dict) else {}
    res_in = float(lf_cfg.get("resolution_in", 2.0))
    res_px = max(1.0, res_in * PPI)
    w, h = float(WINDOW_WIDTH), float(WINDOW_HEIGHT)
    nx = int(math.ceil(w / res_px)) + 1
    ny = int(math.ceil(h / res_px)) + 1
    grid = [[0.0 for _ in range(nx)] for _ in range(ny)]
    for j in range(ny):
        y = j * res_px
        for i in range(nx):
            x = i * res_px
            best = 1e18
            for x0, y0, x1, y1 in segments:
                d2 = _dist_point_to_seg_sq(x, y, x0, y0, x1, y1)
                if d2 < best:
                    best = d2
            grid[j][i] = math.sqrt(best)
    return {
        "grid": grid,
        "res_px": res_px,
        "nx": nx,
        "ny": ny,
        "width": w,
        "height": h,
    }


def _lf_distance_px(lf: dict, x_px: float, y_px: float) -> float:
    """Handle lf distance px."""
    res = float(lf.get("res_px", 1.0))
    nx = int(lf.get("nx", 1))
    ny = int(lf.get("ny", 1))
    if res <= 1e-6:
        return 0.0
    fx = _clamp(x_px / res, 0.0, nx - 1.001)
    fy = _clamp(y_px / res, 0.0, ny - 1.001)
    x0 = int(math.floor(fx))
    y0 = int(math.floor(fy))
    x1 = min(nx - 1, x0 + 1)
    y1 = min(ny - 1, y0 + 1)
    tx = fx - x0
    ty = fy - y0
    g = lf["grid"]
    v00 = g[y0][x0]
    v10 = g[y0][x1]
    v01 = g[y1][x0]
    v11 = g[y1][x1]
    v0 = v00 + (v10 - v00) * tx
    v1 = v01 + (v11 - v01) * tx
    return v0 + (v1 - v0) * ty


def _sensor_segments(state: MCLState, sensor: dict) -> List[Tuple[float, float, float, float]]:
    """Handle sensor segments."""
    vis = sensor.get("object_visibility")
    has_obj_filter = isinstance(vis, dict) and len(vis) > 0
    def _filtered_objects() -> List[Tuple[float, float, float, float]]:
        """Handle filtered objects."""
        segs: List[Tuple[float, float, float, float]] = []
        if not has_obj_filter:
            return state.map_segments_obj
        for obj_id, obj_segs in state.map_segments_obj_by_id.items():
            try:
                on = int(vis.get(obj_id, 1))
            except Exception:
                on = 1
            if on != 1:
                continue
            segs.extend(obj_segs)
        return segs
    mode = str(sensor.get("map_mode", "both")).strip().lower()
    if mode == "perimeter":
        return state.map_segments_perim or state.map_segments
    if mode == "objects":
        obj = _filtered_objects()
        return obj or state.map_segments
    if has_obj_filter:
        return (state.map_segments_perim or []) + _filtered_objects()
    return state.map_segments


def _sensor_lf(state: MCLState, sensor: dict) -> Optional[dict]:
    """Handle sensor lf."""
    vis = sensor.get("object_visibility")
    has_obj_filter = isinstance(vis, dict) and len(vis) > 0
    mode = str(sensor.get("map_mode", "both")).strip().lower()
    if mode == "perimeter":
        return state.lf_perim or state.lf
    if mode == "objects":
        if has_obj_filter:
            return None
        return state.lf_obj or state.lf
    if has_obj_filter:
        return None
    return state.lf



@dataclass
class MCLState:
    particles: List[Dict[str, float]] = field(default_factory=list)
    estimate: Optional[Tuple[float, float, float]] = None
    cov_xy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    n_eff: float = 0.0
    stats: Dict[str, float] = field(default_factory=dict)
    rng: random.Random = field(default_factory=random.Random)
    last_true_pose: Optional[Tuple[float, float, float]] = None
    motion_accum: float = 0.0
    sensor_accum: float = 0.0
    map_segments: List[Tuple[float, float, float, float]] = field(default_factory=list)
    map_segments_perim: List[Tuple[float, float, float, float]] = field(default_factory=list)
    map_segments_obj: List[Tuple[float, float, float, float]] = field(default_factory=list)
    map_segments_obj_by_id: Dict[str, List[Tuple[float, float, float, float]]] = field(default_factory=dict)
    lf: Optional[dict] = None
    lf_perim: Optional[dict] = None
    lf_obj: Optional[dict] = None
    last_rays: List[dict] = field(default_factory=list)
    last_measurements: Dict[str, float] = field(default_factory=dict)
    dist_hist: Dict[str, List[float]] = field(default_factory=dict)
    region_id: Optional[object] = None
    region_gate: Optional[dict] = None
    region_changed: bool = False
    w_slow: Optional[float] = None
    w_fast: Optional[float] = None
    confidence: float = 0.0
    ekf_pose: Optional[Tuple[float, float, float]] = None
    ekf_P: Optional[List[List[float]]] = None



def _mat3_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    """Handle mat3 mul."""
    out = [[0.0, 0.0, 0.0] for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
    return out


def _mat3_transpose(a: List[List[float]]) -> List[List[float]]:
    """Handle mat3 transpose."""
    return [
        [a[0][0], a[1][0], a[2][0]],
        [a[0][1], a[1][1], a[2][1]],
        [a[0][2], a[1][2], a[2][2]],
    ]


def _mat3_inv(a: List[List[float]]) -> Optional[List[List[float]]]:
    """Handle mat3 inv."""
    det = (
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    )
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv_det,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv_det,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv_det,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv_det,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv_det,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv_det,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv_det,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv_det,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv_det,
        ],
    ]


def ekf_reset_state(state: MCLState, cfg: dict,
                    pose: Optional[Tuple[float, float, float]] = None) -> None:
    """Handle ekf reset state."""
    ekf_cfg = _ekf_cfg(cfg)
    if not ekf_cfg["enabled"]:
        state.ekf_pose = None
        state.ekf_P = None
        return
    if pose is None:
        pose = state.estimate or state.last_true_pose
    if pose is None:
        return
    sigma_xy_px = ekf_cfg["init_sigma_xy_in"] * PPI
    sigma_th = ekf_cfg["init_sigma_theta_deg"]
    state.ekf_pose = (float(pose[0]), float(pose[1]), _wrap_deg(float(pose[2])))
    state.ekf_P = [
        [sigma_xy_px * sigma_xy_px, 0.0, 0.0],
        [0.0, sigma_xy_px * sigma_xy_px, 0.0],
        [0.0, 0.0, sigma_th * sigma_th],
    ]


def ekf_predict(state: MCLState, cfg: dict, motion_input: Optional[dict]) -> None:
    """Handle ekf predict."""
    if not motion_input:
        return
    ekf_cfg = _ekf_cfg(cfg)
    if not ekf_cfg["enabled"]:
        return
    if state.ekf_pose is None or state.ekf_P is None:
        ekf_reset_state(state, cfg)
        if state.ekf_pose is None or state.ekf_P is None:
            return
    dx_px = float(motion_input.get("dx_in", 0.0)) * PPI
    dy_px = float(motion_input.get("dy_in", 0.0)) * PPI
    dtheta = float(motion_input.get("dtheta_deg", 0.0))
    x, y, theta = state.ekf_pose
    th_rad = math.radians(theta)
    s = math.sin(th_rad)
    c = math.cos(th_rad)
    x += dx_px * s - dy_px * c
    y += dx_px * c + dy_px * s
    theta = _wrap_deg(theta + dtheta)
    rad_per_deg = math.pi / 180.0
    F = [
        [1.0, 0.0, (dx_px * c + dy_px * s) * rad_per_deg],
        [0.0, 1.0, (-dx_px * s + dy_px * c) * rad_per_deg],
        [0.0, 0.0, 1.0],
    ]
    L = [
        [s, -c, 0.0],
        [c, s, 0.0],
        [0.0, 0.0, 1.0],
    ]
    sigma_dx_px = ekf_cfg["sigma_dx_in"] * PPI
    sigma_dy_px = ekf_cfg["sigma_dy_in"] * PPI
    sigma_dth = ekf_cfg["sigma_dtheta_deg"]
    Q = [
        [sigma_dx_px * sigma_dx_px, 0.0, 0.0],
        [0.0, sigma_dy_px * sigma_dy_px, 0.0],
        [0.0, 0.0, sigma_dth * sigma_dth],
    ]
    P = state.ekf_P
    FP = _mat3_mul(F, P)
    Ft = _mat3_transpose(F)
    temp = _mat3_mul(FP, Ft)
    LQ = _mat3_mul(L, Q)
    Lt = _mat3_transpose(L)
    LQLt = _mat3_mul(LQ, Lt)
    Pnew = [[temp[i][j] + LQLt[i][j] for j in range(3)] for i in range(3)]
    state.ekf_pose = (x, y, theta)
    state.ekf_P = Pnew


def ekf_update_imu(state: MCLState, cfg: dict, heading_deg: float) -> None:
    """Handle ekf update imu."""
    ekf_cfg = _ekf_cfg(cfg)
    if not ekf_cfg["enabled"]:
        return
    if state.ekf_pose is None or state.ekf_P is None:
        return
    x, y, theta = state.ekf_pose
    P = state.ekf_P
    nu = _angle_diff_deg(float(heading_deg), theta)
    R = ekf_cfg["imu_sigma_deg"] * ekf_cfg["imu_sigma_deg"]
    S = P[2][2] + R
    if S < 1e-12:
        return
    K = [P[0][2] / S, P[1][2] / S, P[2][2] / S]
    x += K[0] * nu
    y += K[1] * nu
    theta = _wrap_deg(theta + K[2] * nu)
    I_KH = [
        [1.0, 0.0, -K[0]],
        [0.0, 1.0, -K[1]],
        [0.0, 0.0, 1.0 - K[2]],
    ]
    temp = _mat3_mul(I_KH, P)
    IKHt = _mat3_transpose(I_KH)
    Pnew = _mat3_mul(temp, IKHt)
    KRKt = [
        [K[0] * K[0] * R, K[0] * K[1] * R, K[0] * K[2] * R],
        [K[1] * K[0] * R, K[1] * K[1] * R, K[1] * K[2] * R],
        [K[2] * K[0] * R, K[2] * K[1] * R, K[2] * K[2] * R],
    ]
    Pnew = [[Pnew[i][j] + KRKt[i][j] for j in range(3)] for i in range(3)]
    state.ekf_pose = (x, y, theta)
    state.ekf_P = Pnew


def ekf_update_mcl(state: MCLState, cfg: dict,
                   mcl_pose: Tuple[float, float, float], confidence: float) -> None:
    """Handle ekf update mcl."""
    ekf_cfg = _ekf_cfg(cfg)
    if not ekf_cfg["enabled"]:
        return
    if confidence < ekf_cfg["mcl_min_conf"]:
        return
    if state.ekf_pose is None or state.ekf_P is None:
        ekf_reset_state(state, cfg, mcl_pose)
        return
    x, y, theta = state.ekf_pose
    P = state.ekf_P
    sx = ekf_cfg["mcl_sigma_x_max"] - confidence * (ekf_cfg["mcl_sigma_x_max"] - ekf_cfg["mcl_sigma_x_min"])
    sy = ekf_cfg["mcl_sigma_y_max"] - confidence * (ekf_cfg["mcl_sigma_y_max"] - ekf_cfg["mcl_sigma_y_min"])
    sth = ekf_cfg["mcl_sigma_theta_max"] - confidence * (ekf_cfg["mcl_sigma_theta_max"] - ekf_cfg["mcl_sigma_theta_min"])
    sx_px = sx * PPI
    sy_px = sy * PPI
    R = [
        [sx_px * sx_px, 0.0, 0.0],
        [0.0, sy_px * sy_px, 0.0],
        [0.0, 0.0, sth * sth],
    ]
    S = [[P[i][j] + R[i][j] for j in range(3)] for i in range(3)]
    S_inv = _mat3_inv(S)
    if S_inv is None:
        return
    K = _mat3_mul(P, S_inv)
    nu = [
        float(mcl_pose[0]) - x,
        float(mcl_pose[1]) - y,
        _angle_diff_deg(float(mcl_pose[2]), theta),
    ]
    dx = K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2]
    dy = K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2]
    dth = K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2]
    x += dx
    y += dy
    theta = _wrap_deg(theta + dth)
    I_K = [[(1.0 if i == j else 0.0) - K[i][j] for j in range(3)] for i in range(3)]
    temp = _mat3_mul(I_K, P)
    IKt = _mat3_transpose(I_K)
    Pnew = _mat3_mul(temp, IKt)
    KR = _mat3_mul(K, R)
    Kt = _mat3_transpose(K)
    KRKt = _mat3_mul(KR, Kt)
    Pnew = [[Pnew[i][j] + KRKt[i][j] for j in range(3)] for i in range(3)]
    state.ekf_pose = (x, y, theta)
    state.ekf_P = Pnew

def init_particles(cfg: dict, n: int, rng: Optional[random.Random] = None) -> List[Dict[str, float]]:
    """Handle init particles."""
    rng = rng or random.Random()
    particles = []
    for _ in range(max(1, int(n))):
        particles.append(_sample_random_pose(cfg, rng))
    _normalize_weights(particles)
    return particles


def _reset_state_to_pose(state: MCLState, cfg: dict, pose: Tuple[float, float, float]) -> None:
    """Handle reset state to pose."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    parts = mcl.get("particles", {}) if isinstance(mcl, dict) else {}
    try:
        n_cfg = int(parts.get("n", parts.get("n_min", 200)))
    except Exception:
        try:
            n_cfg = int(parts.get("n_min", 200))
        except Exception:
            n_cfg = 200
    n = len(state.particles) if state.particles else n_cfg
    sigma_xy_in = float(mcl.get("set_pose_sigma_xy_in", 0.2))
    sigma_theta = float(mcl.get("set_pose_sigma_theta_deg", 2.0))
    sigma_xy_px = sigma_xy_in * PPI
    rng = state.rng
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    attempts = max(1, int(region.get("sample_attempts", 50)))
    perim_on = int(region.get("perimeter_gate", 1)) == 1
    base_x = float(pose[0])
    base_y = float(pose[1])
    base_th = float(pose[2])
    out = []
    for _ in range(max(1, int(n))):
        candidate = None
        for _ in range(attempts):
            x = base_x + rng.gauss(0.0, sigma_xy_px)
            y = base_y + rng.gauss(0.0, sigma_xy_px)
            th = _wrap_deg(base_th + rng.gauss(0.0, sigma_theta))
            if perim_on and _pose_outside_perimeter(cfg, x, y, th):
                continue
            candidate = (x, y, th)
            break
        if candidate is None:
            x = base_x
            y = base_y
            th = _wrap_deg(base_th)
            if perim_on and _pose_outside_perimeter(cfg, x, y, th):
                out.append(_sample_random_pose(cfg, rng))
            else:
                out.append({"x": x, "y": y, "theta": th, "w": 1.0})
        else:
            out.append({"x": candidate[0], "y": candidate[1], "theta": candidate[2], "w": 1.0})
    state.particles = out
    _normalize_weights(state.particles)
    ekf_reset_state(state, cfg, pose)


def reset_state_to_pose(state: MCLState, cfg: dict, pose: Tuple[float, float, float]) -> None:
    """Handle reset state to pose."""
    _reset_state_to_pose(state, cfg, pose)
    state.estimate = None
    state.cov_xy = (0.0, 0.0, 0.0)
    state.n_eff = 0.0
    state.last_true_pose = pose
    state.w_slow = None
    state.w_fast = None
    state.confidence = 0.0


def update_map_segments(state: MCLState, cfg: dict) -> None:
    """Update map segments."""
    perim = build_perimeter_segments(cfg)
    obj_by_id = build_object_segments_by_id(cfg)
    obj = []
    for segs in obj_by_id.values():
        obj.extend(segs)
    segments = perim + obj
    state.map_segments = segments
    state.map_segments_perim = perim
    state.map_segments_obj = obj
    state.map_segments_obj_by_id = obj_by_id
    state.lf = _build_likelihood_field(cfg, segments)
    state.lf_perim = _build_likelihood_field(cfg, perim) if perim else None
    state.lf_obj = _build_likelihood_field(cfg, obj) if obj else None



def region_id_for_pose(cfg: dict, x_px: float, y_px: float) -> Optional[int]:
    """Handle region id for pose."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    grid_type = str(region.get("grid_type", "quadrant")).strip().lower()
    if grid_type == "quadrant":
        gx, gy = 2, 2
    else:
        gx = max(1, int(region.get("grid_x", 2)))
        gy = max(1, int(region.get("grid_y", 2)))
    try:
        x_min = float(region.get("x_min_in", 0.0)) * PPI
        y_min = float(region.get("y_min_in", 0.0)) * PPI
        x_max = float(region.get("x_max_in", WINDOW_WIDTH / PPI)) * PPI
        y_max = float(region.get("y_max_in", WINDOW_HEIGHT / PPI)) * PPI
    except Exception:
        x_min, y_min = 0.0, 0.0
        x_max, y_max = float(WINDOW_WIDTH), float(WINDOW_HEIGHT)
    if x_max <= x_min:
        x_max = x_min + float(WINDOW_WIDTH)
    if y_max <= y_min:
        y_max = y_min + float(WINDOW_HEIGHT)
    x_px = _clamp(x_px, x_min, x_max - 1e-6)
    y_px = _clamp(y_px, y_min, y_max - 1e-6)
    gx = max(1, gx)
    gy = max(1, gy)
    cell_w = (x_max - x_min) / gx
    cell_h = (y_max - y_min) / gy
    ix = int((x_px - x_min) / max(1e-6, cell_w))
    iy = int((y_px - y_min) / max(1e-6, cell_h))
    ix = max(0, min(gx - 1, ix))
    iy = max(0, min(gy - 1, iy))
    return iy * gx + ix


def region_ids_for_pose(cfg: dict, x_px: float, y_px: float, heading_deg: float) -> List[int]:
    """Handle region ids for pose."""
    rid = region_id_for_pose(cfg, x_px, y_px)
    return [] if rid is None else [rid]


def _poly_edge_dist_px(poly: List[Tuple[float, float]], x_px: float, y_px: float) -> float:
    """Handle poly edge dist px."""
    if not poly or len(poly) < 2:
        return 0.0
    best = 1e18
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        d2 = _dist_point_to_seg_sq(x_px, y_px, x0, y0, x1, y1)
        if d2 < best:
            best = d2
    return math.sqrt(best) if best > 0.0 else 0.0


def _object_clip_overlap_px(cfg: dict, x_px: float, y_px: float, heading_deg: float) -> float:
    """Handle object clip overlap px."""
    corners = _robot_corners_px(cfg, x_px, y_px, heading_deg)
    max_pen = 0.0
    for cx, cy in corners:
        corner_pen = 0.0
        for entry in get_field_object_entries(cfg):
            if not _entry_enabled(cfg, entry):
                continue
            poly = entry.get("poly", [])
            if poly and point_in_poly((cx, cy), poly):
                dist = _poly_edge_dist_px(poly, cx, cy)
                if dist > corner_pen:
                    corner_pen = dist
        if corner_pen > max_pen:
            max_pen = corner_pen
    return max_pen


def _region_object_mode(region: dict) -> int:
    """Handle region object mode."""
    raw = region.get("object_mode", None) if isinstance(region, dict) else None
    if isinstance(raw, dict) and "value" in raw:
        raw = raw.get("value")
    if raw is None:
        try:
            return 2 if int(region.get("object_gate", 0)) == 1 else 0
        except Exception:
            return 0
    if isinstance(raw, str):
        low = raw.strip().lower()
        if low in ("off", "0", "false", "none"):
            return 0
        if low in ("soft", "1", "penalty"):
            return 1
        if low in ("hard", "2", "gate", "reject"):
            return 2
        try:
            return int(raw)
        except Exception:
            return 0
    try:
        return int(raw)
    except Exception:
        return 0


def _object_clip_weight(cfg: dict, region: dict, x_px: float, y_px: float, heading_deg: float, mode: int) -> float:
    """Handle object clip weight."""
    if mode <= 0:
        return 1.0
    overlap = _object_clip_overlap_px(cfg, x_px, y_px, heading_deg)
    if overlap <= 0.0:
        return 1.0
    if mode >= 2:
        return 0.0
    def _rnum(key: str, default: float) -> float:
        """Handle rnum."""
        val = region.get(key, default)
        if isinstance(val, dict) and "value" in val:
            val = val.get("value", default)
        try:
            return float(val)
        except Exception:
            return float(default)
    free_px = _rnum("object_clip_free_in", 0.5) * PPI
    max_px = _rnum("object_clip_max_in", 2.0) * PPI
    sigma_px = _rnum("object_clip_sigma_in", 0.75) * PPI
    if overlap <= free_px:
        return 1.0
    if max_px <= free_px or sigma_px <= 1e-6:
        return 0.0
    if overlap >= max_px:
        return 0.0
    z = (overlap - free_px) / max(1e-6, sigma_px)
    return math.exp(-0.5 * z * z)


def _apply_region_gate(cfg: dict, state: MCLState, particle: Dict[str, float]) -> float:
    """Handle apply region gate."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    mode = str(region.get("mode", "soft")).strip().lower()
    penalty = float(region.get("penalty", 0.2))
    region_type = str(region.get("type", "segment_band")).strip().lower()
    x = float(particle.get("x", 0.0))
    y = float(particle.get("y", 0.0))
    heading = float(particle.get("theta", 0.0))

    weight = 1.0
    if int(region.get("perimeter_gate", 1)) == 1:
        if _pose_outside_perimeter(cfg, x, y, heading):
            return 0.0
    obj_mode = _region_object_mode(region)
    if obj_mode:
        obj_w = _object_clip_weight(cfg, region, x, y, heading, obj_mode)
        if obj_w <= 0.0:
            return 0.0
        weight *= obj_w

    if int(region.get("enabled", 0)) != 1:
        return weight

    if region_type in ("segment_band", "segment", "band", "segment_path", "segmentpath", "path"):
        gate = state.region_gate
        if isinstance(gate, dict):
            pts = gate.get("points", [])
            radius_px = float(gate.get("radius_px", 0.0))
            if pts and radius_px > 0.0:
                d2 = _polyline_dist_sq(x, y, pts)
                if d2 > radius_px * radius_px:
                    if mode == "hard":
                        return 0.0
                    return weight * penalty
    elif region_type == "grid":
        current = region.get("current_region", None)
        if current is not None:
            rid = region_id_for_pose(cfg, x, y)
            allowed = current
            if isinstance(allowed, (list, tuple, set)):
                if rid not in allowed:
                    if mode == "hard":
                        return 0.0
                    return weight * penalty
            else:
                if rid != allowed:
                    if mode == "hard":
                        return 0.0
                    return weight * penalty
    elif region_type in ("manual", "bounds", "rect", "rectangle"):
        try:
            x_min = float(region.get("x_min_in", 0.0)) * PPI
            y_min = float(region.get("y_min_in", 0.0)) * PPI
            x_max = float(region.get("x_max_in", WINDOW_WIDTH / PPI)) * PPI
            y_max = float(region.get("y_max_in", WINDOW_HEIGHT / PPI)) * PPI
        except Exception:
            x_min, y_min = 0.0, 0.0
            x_max, y_max = float(WINDOW_WIDTH), float(WINDOW_HEIGHT)
        if x_max <= x_min:
            x_max = x_min + float(WINDOW_WIDTH)
        if y_max <= y_min:
            y_max = y_min + float(WINDOW_HEIGHT)
        if x < x_min or x > x_max or y < y_min or y > y_max:
            if mode == "hard":
                return 0.0
            return weight * penalty
    return weight



def _normalize_weights(particles: List[Dict[str, float]]) -> None:
    """Handle normalize weights."""
    total = 0.0
    for p in particles:
        total += max(0.0, float(p.get("w", 0.0)))
    if total <= 1e-12:
        for p in particles:
            p["w"] = 1.0 / max(1, len(particles))
        return
    for p in particles:
        p["w"] = max(0.0, float(p.get("w", 0.0))) / total


def _effective_n(particles: List[Dict[str, float]]) -> float:
    """Handle effective n."""
    denom = 0.0
    for p in particles:
        w = float(p.get("w", 0.0))
        denom += w * w
    return 0.0 if denom <= 1e-12 else 1.0 / denom


def _estimate_pose(particles: List[Dict[str, float]]) -> Optional[Tuple[float, float, float]]:
    """Handle estimate pose."""
    if not particles:
        return None
    x = 0.0
    y = 0.0
    sum_x = 0.0
    sum_y = 0.0
    for p in particles:
        w = float(p.get("w", 0.0))
        x += float(p.get("x", 0.0)) * w
        y += float(p.get("y", 0.0)) * w
        ux, uy = _heading_unit(float(p.get("theta", 0.0)))
        sum_x += ux * w
        sum_y += uy * w
    theta = _heading_from_unit(sum_x, sum_y)
    return (x, y, theta)


def _cov_xy(particles: List[Dict[str, float]], mean: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Handle cov xy."""
    if not particles:
        return (0.0, 0.0, 0.0)
    mx, my = mean[0], mean[1]
    var_x = 0.0
    var_y = 0.0
    cov_xy = 0.0
    for p in particles:
        w = float(p.get("w", 0.0))
        dx = float(p.get("x", 0.0)) - mx
        dy = float(p.get("y", 0.0)) - my
        var_x += w * dx * dx
        var_y += w * dy * dy
        cov_xy += w * dx * dy
    return (var_x, var_y, cov_xy)



def _resample_systematic(particles: List[Dict[str, float]], n: int, rng: random.Random) -> List[Dict[str, float]]:
    """Handle resample systematic."""
    n = max(1, int(n))
    step = 1.0 / n
    r = rng.random() * step
    c = particles[0]["w"]
    i = 0
    out = []
    for m in range(n):
        u = r + m * step
        while u > c and i < len(particles) - 1:
            i += 1
            c += particles[i]["w"]
        p = dict(particles[i])
        p["w"] = 1.0 / n
        out.append(p)
    return out


def _resample_stratified(particles: List[Dict[str, float]], n: int, rng: random.Random) -> List[Dict[str, float]]:
    """Handle resample stratified."""
    n = max(1, int(n))
    out = []
    c = particles[0]["w"]
    i = 0
    for m in range(n):
        u = (m + rng.random()) / n
        while u > c and i < len(particles) - 1:
            i += 1
            c += particles[i]["w"]
        p = dict(particles[i])
        p["w"] = 1.0 / n
        out.append(p)
    return out


def _resample_multinomial(particles: List[Dict[str, float]], n: int, rng: random.Random) -> List[Dict[str, float]]:
    """Handle resample multinomial."""
    n = max(1, int(n))
    cdf = []
    total = 0.0
    for p in particles:
        total += float(p.get("w", 0.0))
        cdf.append(total)
    if total <= 1e-12:
        return _resample_systematic(particles, n, rng)
    out = []
    for _ in range(n):
        r = rng.random() * total
        idx = 0
        while idx < len(cdf) and r > cdf[idx]:
            idx += 1
        idx = min(idx, len(particles) - 1)
        p = dict(particles[idx])
        p["w"] = 1.0 / n
        out.append(p)
    return out


def _kld_required_particles(k: int, epsilon: float, delta: float) -> int:
    """Handle kld required particles."""
    if k <= 1:
        return 1
    p = delta if delta >= 0.5 else 1.0 - delta
    z = _inv_norm_cdf(p)
    k_minus = k - 1.0
    frac = 1.0 - 2.0 / (9.0 * k_minus) + z * math.sqrt(2.0 / (9.0 * k_minus))
    n = (k_minus / (2.0 * epsilon)) * (frac ** 3)
    return int(math.ceil(n))


def _inv_norm_cdf(p: float) -> float:
    """Handle inv norm cdf."""
    if p <= 0.0:
        return -1e9
    if p >= 1.0:
        return 1e9
    a = [
        -3.969683028665376e+01,
        2.209460984245205e+02,
        -2.759285104469687e+02,
        1.383577518672690e+02,
        -3.066479806614716e+01,
        2.506628277459239e+00,
    ]
    b = [
        -5.447609879822406e+01,
        1.615858368580409e+02,
        -1.556989798598866e+02,
        6.680131188771972e+01,
        -1.328068155288572e+01,
    ]
    c = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e+00,
        -2.549732539343734e+00,
        4.374664141464968e+00,
        2.938163982698783e+00,
    ]
    d = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e+00,
        3.754408661907416e+00,
    ]
    plow = 0.02425
    phigh = 1.0 - plow
    if p < plow:
        q = math.sqrt(-2.0 * math.log(p))
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
            ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    if p > phigh:
        q = math.sqrt(-2.0 * math.log(1.0 - p))
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / \
            ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    q = p - 0.5
    r = q * q
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / \
        (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)


def _bin_index(p: Dict[str, float], bin_xy_in: float, bin_theta_deg: float) -> Tuple[int, int, int]:
    """Handle bin index."""
    bx = max(1e-6, bin_xy_in) * PPI
    by = bx
    bth = max(1e-6, bin_theta_deg)
    ix = int(float(p.get("x", 0.0)) / bx)
    iy = int(float(p.get("y", 0.0)) / by)
    it = int(_wrap_deg(float(p.get("theta", 0.0))) / bth)
    return (ix, iy, it)


def _kld_target(cfg: dict, particles: List[Dict[str, float]]) -> int:
    """Handle kld target."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    kld = mcl.get("kld", {}) if isinstance(mcl, dict) else {}
    if int(kld.get("enabled", 0)) != 1:
        return len(particles)
    epsilon = float(kld.get("epsilon", 0.05))
    delta = float(kld.get("delta", 0.99))
    bin_xy_in = float(kld.get("bin_xy_in", 2.0))
    bin_theta_deg = float(kld.get("bin_theta_deg", 10.0))
    bins = set(_bin_index(p, bin_xy_in, bin_theta_deg) for p in particles)
    k = max(1, len(bins))
    n_req = _kld_required_particles(k, epsilon, delta)
    n_min = int(mcl.get("particles", {}).get("n_min", len(particles)))
    n_max = int(mcl.get("particles", {}).get("n_max", len(particles)))
    return max(n_min, min(n_max, n_req))



def _gaussian(x: float, mu: float, sigma: float, normalize: bool = False) -> float:
    """Handle gaussian."""
    if sigma <= 1e-9:
        return 1.0 if abs(x - mu) <= 1e-9 else 0.0
    z = (x - mu) / sigma
    val = math.exp(-0.5 * z * z)
    if not normalize:
        return val
    return val / (sigma * math.sqrt(2.0 * math.pi))


def _distance_model_weight(dist_cfg: dict, expected_mm: float, measured_mm: float) -> Tuple[float, bool]:
    """Handle distance model weight."""
    sigma = float(dist_cfg.get("sigma_hit_mm", 8.5))
    w_hit = float(dist_cfg.get("w_hit", 0.9))
    w_rand = float(dist_cfg.get("w_rand", 0.1))
    w_short = float(dist_cfg.get("w_short", 0.0))
    w_max = float(dist_cfg.get("w_max", 0.0))
    lam_short = float(dist_cfg.get("lambda_short", 0.1))
    max_range = float(dist_cfg.get("max_range_mm", 2000.0))
    gate_mm = float(dist_cfg.get("gate_mm", 0.0))
    gate_mode = str(dist_cfg.get("gate_mode", "hard")).strip().lower()
    gate_penalty = float(dist_cfg.get("gate_penalty", 0.05))
    if gate_mm > 0.0 and abs(measured_mm - expected_mm) > gate_mm:
        if gate_mode == "soft":
            return max(0.0, gate_penalty), True
        return 0.0, True
    if max_range <= 0.0:
        max_range = max(measured_mm, expected_mm, 1.0)
    z = measured_mm
    z_hat = expected_mm
    p_hit = _gaussian(z, z_hat, sigma, normalize=False)
    p_rand = 1.0 / max_range if 0.0 <= z <= max_range else 0.0
    p_short = 0.0
    if w_short > 0.0 and 0.0 <= z <= z_hat and lam_short > 0.0:
        p_short = lam_short * math.exp(-lam_short * z)
    p_max = 1.0 if w_max > 0.0 and z >= max_range - 1e-6 else 0.0
    w = w_hit * p_hit + w_rand * p_rand + w_short * p_short + w_max * p_max
    return max(w, 0.0), False


def _imu_weight(imu_cfg: dict, particle_theta: float, imu_deg: float) -> float:
    """Handle imu weight."""
    sigma = float(imu_cfg.get("sigma_deg", 1.0))
    diff = _angle_diff_deg(particle_theta, imu_deg)
    return _gaussian(diff, 0.0, sigma, normalize=False)


def _vision_weight(vision_cfg: dict, particle_xy_in: Tuple[float, float], particle_theta: float,
                   vision_xy_in: Tuple[float, float], vision_theta: Optional[float]) -> float:
    """Handle vision weight."""
    sigma_xy = float(vision_cfg.get("sigma_xy_in", 2.0))
    sigma_th = float(vision_cfg.get("sigma_theta_deg", 5.0))
    dx = particle_xy_in[0] - vision_xy_in[0]
    dy = particle_xy_in[1] - vision_xy_in[1]
    w = _gaussian(math.hypot(dx, dy), 0.0, sigma_xy, normalize=False)
    if vision_theta is not None:
        diff = _angle_diff_deg(particle_theta, vision_theta)
        w *= _gaussian(diff, 0.0, sigma_th, normalize=False)
    return w



def simulate_motion_input(state: MCLState, new_pose: Tuple[float, float, float],
                          prev_pose: Optional[Tuple[float, float, float]] = None) -> Optional[dict]:
    """Handle simulate motion input."""
    if prev_pose is None:
        prev_pose = state.last_true_pose
    state.last_true_pose = new_pose
    if prev_pose is None:
        return None
    dx_px = float(new_pose[0]) - float(prev_pose[0])
    dy_px = float(new_pose[1]) - float(prev_pose[1])
    dtheta_deg = _angle_diff_deg(float(new_pose[2]), float(prev_pose[2]))
    heading_mid = _wrap_deg(float(prev_pose[2]) + 0.5 * dtheta_deg)
    dx_r_px, dy_r_px = rotate_world_to_robot(dx_px, dy_px, heading_mid)
    return {
        "dx_in": px_to_in(dx_r_px),
        "dy_in": px_to_in(dy_r_px),
        "dtheta_deg": dtheta_deg,
        "heading_deg": float(new_pose[2]),
    }


def motion_update(state: MCLState, cfg: dict, motion_input: Optional[dict]) -> None:
    """Handle motion update."""
    if not motion_input:
        return
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    motion = mcl.get("motion", {}) if isinstance(mcl, dict) else {}
    if int(motion.get("enabled", 1)) != 1:
        return
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    perim_on = int(region.get("perimeter_gate", 1)) == 1
    dx_in = float(motion_input.get("dx_in", 0.0))
    dy_in = float(motion_input.get("dy_in", 0.0))
    dtheta = float(motion_input.get("dtheta_deg", 0.0))
    if abs(dx_in) + abs(dy_in) + abs(dtheta) <= 1e-9:
        return
    use_alpha = int(motion.get("use_alpha_model", 0)) == 1
    sigma_x = float(motion.get("sigma_x_in", 0.1275))
    sigma_y = float(motion.get("sigma_y_in", 0.1275))
    sigma_theta = float(motion.get("sigma_theta_deg", 1.0))
    alpha1 = float(motion.get("alpha1", 0.05))
    alpha2 = float(motion.get("alpha2", 0.05))
    alpha3 = float(motion.get("alpha3", 0.05))
    alpha4 = float(motion.get("alpha4", 0.05))
    dist = math.hypot(dx_in, dy_in)
    rng = state.rng
    for p in state.particles:
        ndx = dx_in
        ndy = dy_in
        nth = dtheta
        if use_alpha:
            sigma_trans = alpha1 * dist + alpha2 * abs(dtheta)
            sigma_rot = alpha3 * dist + alpha4 * abs(dtheta)
            ndx += rng.gauss(0.0, sigma_trans)
            ndy += rng.gauss(0.0, sigma_trans)
            nth += rng.gauss(0.0, sigma_rot)
        else:
            ndx += rng.gauss(0.0, sigma_x)
            ndy += rng.gauss(0.0, sigma_y)
            nth += rng.gauss(0.0, sigma_theta)
        dx_px = in_to_px(ndx)
        dy_px = in_to_px(ndy)
        base_th = float(p.get("theta", 0.0))
        wx, wy = rotate_robot_to_world(dx_px, dy_px, base_th)
        x = float(p.get("x", 0.0)) + wx
        y = float(p.get("y", 0.0)) + wy
        th = _wrap_deg(base_th + nth)
        if perim_on and _pose_outside_perimeter(cfg, x, y, th):
            continue
        p["x"] = x
        p["y"] = y
        p["theta"] = th



def _sensor_world_pose(x_px: float, y_px: float, heading_deg: float, sensor: dict) -> Tuple[Tuple[float, float], float]:
    """Handle sensor world pose."""
    off_x_in = float(sensor.get("x_in", 0.0))
    off_y_in = float(sensor.get("y_in", 0.0))
    ang = float(sensor.get("angle_deg", 0.0)) + float(sensor.get("angle_offset_deg", 0.0))
    dx_px = in_to_px(off_x_in)
    dy_px = in_to_px(off_y_in)
    wx, wy = rotate_robot_to_world(dx_px, dy_px, heading_deg)
    origin = (float(x_px) + wx, float(y_px) + wy)
    return origin, _wrap_deg(heading_deg + ang)


def _sample_random_pose(cfg: dict, rng: random.Random) -> Dict[str, float]:
    """Handle sample random pose."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    try:
        x_min = float(region.get("x_min_in", 0.0)) * PPI
        y_min = float(region.get("y_min_in", 0.0)) * PPI
        x_max = float(region.get("x_max_in", WINDOW_WIDTH / PPI)) * PPI
        y_max = float(region.get("y_max_in", WINDOW_HEIGHT / PPI)) * PPI
    except Exception:
        x_min, y_min = 0.0, 0.0
        x_max, y_max = float(WINDOW_WIDTH), float(WINDOW_HEIGHT)
    if x_max <= x_min:
        x_max = x_min + float(WINDOW_WIDTH)
    if y_max <= y_min:
        y_max = y_min + float(WINDOW_HEIGHT)
    attempts = int(region.get("sample_attempts", 50))
    perim_on = int(region.get("perimeter_gate", 1)) == 1
    for _ in range(max(1, attempts)):
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        th = rng.uniform(0.0, 360.0)
        if perim_on and _pose_outside_perimeter(cfg, x, y, th):
            continue
        return {"x": x, "y": y, "theta": th, "w": 1.0}
    for _ in range(max(1, attempts)):
        x = rng.uniform(0.0, float(WINDOW_WIDTH))
        y = rng.uniform(0.0, float(WINDOW_HEIGHT))
        th = rng.uniform(0.0, 360.0)
        if perim_on and _pose_outside_perimeter(cfg, x, y, th):
            continue
        return {"x": x, "y": y, "theta": th, "w": 1.0}
    x = float(WINDOW_WIDTH) * 0.5
    y = float(WINDOW_HEIGHT) * 0.5
    th = rng.uniform(0.0, 360.0)
    if perim_on and _pose_outside_perimeter(cfg, x, y, th):
        th = 0.0
    return {"x": x, "y": y, "theta": th, "w": 1.0}


def _nearest_polyline_heading(points: List[Tuple[float, float]],
                              x_px: float, y_px: float) -> Tuple[Optional[float], float]:
    """Handle nearest polyline heading."""
    if not points or len(points) < 2:
        return None, 1e18
    best_d2 = 1e18
    best_heading = None
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        d2 = _dist_point_to_seg_sq(x_px, y_px, x0, y0, x1, y1)
        if d2 < best_d2:
            best_d2 = d2
            best_heading = _heading_from_unit(x1 - x0, y1 - y0)
    return best_heading, best_d2


def _apply_slope_field(cfg: dict, state: MCLState, particle: Dict[str, float]) -> float:
    """Handle apply slope field."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    region = mcl.get("region", {}) if isinstance(mcl, dict) else {}
    if int(region.get("slope_enabled", 0)) != 1:
        return 1.0
    gate = state.region_gate
    if not isinstance(gate, dict):
        return 1.0
    slope_field = gate.get("slope_field")
    if slope_field is None:
        return 1.0
    sigma = float(region.get("slope_sigma_deg", 20.0))
    best_heading = None
    best_d2 = 1e18
    items = []
    if isinstance(slope_field, dict):
        items = [slope_field]
    elif isinstance(slope_field, list):
        items = slope_field
    else:
        return 1.0
    for item in items:
        pts = None
        if isinstance(item, dict):
            pts = item.get("points")
        elif isinstance(item, list):
            pts = item
        if not isinstance(pts, list) or len(pts) < 2:
            continue
        heading, d2 = _nearest_polyline_heading(pts, float(particle.get("x", 0.0)), float(particle.get("y", 0.0)))
        if heading is None:
            continue
        if d2 < best_d2:
            best_d2 = d2
            best_heading = heading
    if best_heading is None:
        return 1.0
    diff = _angle_diff_deg(float(particle.get("theta", 0.0)), best_heading)
    return _gaussian(diff, 0.0, sigma, normalize=False)


def _update_ray_overlays(state: MCLState, cfg: dict, measurements: dict,
                         estimate: Optional[Tuple[float, float, float]]) -> None:
    """Update ray overlays."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    if int(dist_cfg.get("enabled", 1)) != 1:
        return
    dist_meas = measurements.get("distance", {})
    if not isinstance(dist_meas, dict) or not dist_meas:
        return
    sensors = get_distance_sensors(cfg)
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    gate_mm = float(dist_cfg.get("gate_mm", 0.0))
    rays_by_name = {r.get("name"): r for r in state.last_rays if isinstance(r, dict)}
    new_rays = []
    for sensor in sensors:
        name = sensor.get("name", "")
        if name not in dist_meas:
            continue
        try:
            meas_mm = float(dist_meas.get(name))
        except Exception:
            continue
        bias_mm = float(sensor.get("bias_mm", 0.0))
        s_max = float(sensor.get("max_range_mm", max_range_mm))
        if s_max <= 0.0:
            s_max = max_range_mm
        max_range_px_s = s_max / MM_PER_IN * PPI
        ray = rays_by_name.get(name, {"name": name})
        if estimate is not None:
            est_origin, est_heading = _sensor_world_pose(estimate[0], estimate[1], estimate[2], sensor)
            pred_px = raycast_distance(_sensor_segments(state, sensor), est_origin, est_heading, max_range_px_s)
            pred_mm = s_max if pred_px is None else px_to_in(pred_px) * MM_PER_IN
            pred_px = max_range_px_s if pred_px is None else pred_px
            ux, uy = _heading_unit(est_heading)
            pred_end = (
                est_origin[0] + ux * pred_px,
                est_origin[1] + uy * pred_px,
            )
            ray["pred_origin"] = est_origin
            ray["pred_end"] = pred_end
            ray["pred_mm"] = pred_mm
            ray["gated"] = gate_mm > 0.0 and abs((meas_mm - bias_mm) - pred_mm) > gate_mm
            if "origin" not in ray:
                ray["origin"] = est_origin
            if "angle_deg" not in ray:
                ray["angle_deg"] = est_heading
        if "origin" in ray and "meas_end" not in ray:
            origin = ray.get("origin")
            heading = ray.get("angle_deg", 0.0)
            if origin is not None:
                meas_px = max(0.0, min(max_range_px_s, meas_mm / MM_PER_IN * PPI))
                ux, uy = _heading_unit(heading)
                ray["meas_end"] = (
                    origin[0] + ux * meas_px,
                    origin[1] + uy * meas_px,
                )
        ray["meas_mm"] = meas_mm
        new_rays.append(ray)
    if new_rays:
        state.last_rays = new_rays


def sensor_update(state: MCLState, cfg: dict, measurements: Optional[dict]) -> None:
    """Handle sensor update."""
    if measurements is None:
        return
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    imu_cfg = sensors_cfg.get("imu", {}) if isinstance(sensors_cfg, dict) else {}
    vision_cfg = sensors_cfg.get("vision", {}) if isinstance(sensors_cfg, dict) else {}
    use_distance = int(dist_cfg.get("enabled", 1)) == 1
    use_imu = int(imu_cfg.get("enabled", 1)) == 1
    use_vision = int(vision_cfg.get("enabled", 0)) == 1
    dist_meas = measurements.get("distance", {})
    imu_meas = measurements.get("imu", None)
    vision_meas = measurements.get("vision", None)
    sensors = get_distance_sensors(cfg) if use_distance else []
    model = str(dist_cfg.get("model", "likelihood_field")).strip().lower()
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    max_range_px = max_range_mm / MM_PER_IN * PPI
    min_range_mm = float(dist_cfg.get("min_range_mm", 0.0))
    lf_ignore_max = int(dist_cfg.get("lf_ignore_max", 0)) == 1
    gate_mm = float(dist_cfg.get("gate_mm", 0.0))
    gate_mode = str(dist_cfg.get("gate_mode", "hard")).strip().lower()
    gate_penalty = float(dist_cfg.get("gate_penalty", 0.05))
    gate_reject_ratio = _clamp(float(dist_cfg.get("gate_reject_ratio", 0.9)), 0.0, 1.0)
    innov_gate_mm = float(dist_cfg.get("innovation_gate_mm", 0.0))
    median_window = int(dist_cfg.get("median_window", 1))
    sigma_hit = float(dist_cfg.get("sigma_hit_mm", 8.5))
    w_hit = float(dist_cfg.get("w_hit", 0.9))
    w_rand = float(dist_cfg.get("w_rand", 0.1))
    w_max = float(dist_cfg.get("w_max", 0.0))
    vision_conf_min = float(vision_cfg.get("confidence_min", 0.0))
    dist_weights = None
    skip_sensors = set()
    if use_distance and isinstance(dist_meas, dict) and median_window > 1:
        for name, raw in list(dist_meas.items()):
            try:
                meas_mm = float(raw)
            except Exception:
                continue
            hist = state.dist_hist.setdefault(name, [])
            hist.append(meas_mm)
            if len(hist) > median_window:
                hist.pop(0)
            try:
                dist_meas[name] = sorted(hist)[len(hist) // 2]
            except Exception:
                dist_meas[name] = meas_mm

    if use_distance and isinstance(dist_meas, dict):
        if state.estimate is not None:
            ex, ey, eth = state.estimate
            for sensor in sensors:
                name = sensor.get("name", "")
                if name not in dist_meas:
                    continue
                s_innov = float(sensor.get("innovation_gate_mm", innov_gate_mm))
                if s_innov <= 0.0:
                    continue
                try:
                    meas_mm = float(dist_meas.get(name))
                except Exception:
                    continue
                bias_mm = float(sensor.get("bias_mm", 0.0))
                meas_mm -= bias_mm
                s_min = float(sensor.get("min_range_mm", min_range_mm))
                s_max = float(sensor.get("max_range_mm", max_range_mm))
                if s_max <= 0.0:
                    s_max = max_range_mm
                if s_min > 0.0 and meas_mm < s_min:
                    continue
                if lf_ignore_max and model == "likelihood_field" and meas_mm >= s_max:
                    continue
                origin, heading = _sensor_world_pose(ex, ey, eth, sensor)
                segs = _sensor_segments(state, sensor)
                pred_px = raycast_distance(segs, origin, heading, s_max / MM_PER_IN * PPI)
                pred_mm = s_max if pred_px is None else px_to_in(pred_px) * MM_PER_IN
                if abs(meas_mm - pred_mm) > s_innov:
                    skip_sensors.add(name)
        if gate_mm > 0.0 and gate_reject_ratio > 0.0 and state.particles:
            n_particles = len(state.particles)
            for sensor in sensors:
                name = sensor.get("name", "")
                if name not in dist_meas:
                    continue
                try:
                    meas_mm = float(dist_meas.get(name))
                except Exception:
                    continue
                bias_mm = float(sensor.get("bias_mm", 0.0))
                meas_mm -= bias_mm
                s_min = float(sensor.get("min_range_mm", min_range_mm))
                s_max = float(sensor.get("max_range_mm", max_range_mm))
                if s_max <= 0.0:
                    s_max = max_range_mm
                if s_min > 0.0 and meas_mm < s_min:
                    skip_sensors.add(name)
                    continue
                if lf_ignore_max and model == "likelihood_field" and meas_mm >= s_max:
                    skip_sensors.add(name)
                    continue
                gated_count = 0
                for p in state.particles:
                    origin, heading = _sensor_world_pose(p.get("x", 0.0), p.get("y", 0.0),
                                                         p.get("theta", 0.0), sensor)
                    segs = _sensor_segments(state, sensor)
                    pred_px = raycast_distance(segs, origin, heading, s_max / MM_PER_IN * PPI)
                    pred_mm = s_max if pred_px is None else px_to_in(pred_px) * MM_PER_IN
                    if abs(meas_mm - pred_mm) > gate_mm:
                        gated_count += 1
                if gated_count >= gate_reject_ratio * n_particles:
                    skip_sensors.add(name)

        def _distance_weight(particle: Dict[str, float]) -> float:
            """Handle distance weight."""
            w_dist = 1.0
            for sensor in sensors:
                name = sensor.get("name", "")
                if name in skip_sensors:
                    continue
                if name not in dist_meas:
                    continue
                try:
                    meas_mm = float(dist_meas.get(name))
                except Exception:
                    continue
                bias_mm = float(sensor.get("bias_mm", 0.0))
                meas_mm -= bias_mm
                s_min = float(sensor.get("min_range_mm", min_range_mm))
                s_max = float(sensor.get("max_range_mm", max_range_mm))
                if s_max <= 0.0:
                    s_max = max_range_mm
                if s_min > 0.0 and meas_mm < s_min:
                    continue
                if lf_ignore_max and model == "likelihood_field" and meas_mm >= s_max:
                    continue
                origin, heading = _sensor_world_pose(particle.get("x", 0.0), particle.get("y", 0.0),
                                                     particle.get("theta", 0.0), sensor)
                segs = _sensor_segments(state, sensor)
                pred_px = raycast_distance(segs, origin, heading, s_max / MM_PER_IN * PPI)
                pred_mm = s_max if pred_px is None else px_to_in(pred_px) * MM_PER_IN
                gated = gate_mm > 0.0 and abs(meas_mm - pred_mm) > gate_mm
                lf = _sensor_lf(state, sensor)
                if model == "likelihood_field" and lf is not None:
                    max_range_px_s = s_max / MM_PER_IN * PPI
                    meas_px = max(0.0, min(max_range_px_s, meas_mm / MM_PER_IN * PPI))
                    ux, uy = _heading_unit(heading)
                    end_x = origin[0] + ux * meas_px
                    end_y = origin[1] + uy * meas_px
                    dist_px = _lf_distance_px(lf, end_x, end_y)
                    dist_mm = px_to_in(dist_px) * MM_PER_IN
                    p_hit = _gaussian(dist_mm, 0.0, sigma_hit, normalize=False)
                    p_rand = 1.0 / s_max if 0.0 <= meas_mm <= s_max else 0.0
                    p_max = 1.0 if w_max > 0.0 and meas_mm >= s_max - 1e-6 else 0.0
                    w_meas = w_hit * p_hit + w_rand * p_rand + w_max * p_max
                    if gated:
                        if gate_mode == "soft":
                            w_meas *= max(0.0, gate_penalty)
                        else:
                            w_meas = 0.0
                    w_dist *= w_meas
                else:
                    w_meas, _g = _distance_model_weight(dist_cfg, pred_mm, meas_mm)
                    w_dist *= w_meas
            return w_dist

        dist_total = 0.0
        dist_weights = []
        for p in state.particles:
            w_dist = _distance_weight(p)
            dist_weights.append(w_dist)
            dist_total += float(p.get("w", 1.0)) * w_dist
        if dist_total <= 1e-12:
            dist_weights = None
    total_w = 0.0
    for idx, p in enumerate(state.particles):
        w = float(p.get("w", 1.0))
        if dist_weights is not None:
            w *= dist_weights[idx]
        if use_imu and imu_meas is not None:
            try:
                w *= _imu_weight(imu_cfg, float(p.get("theta", 0.0)), float(imu_meas))
            except Exception:
                pass
        if use_vision and isinstance(vision_meas, dict):
            conf = vision_meas.get("confidence", 1.0)
            try:
                conf_val = float(conf)
            except Exception:
                conf_val = 1.0
            if conf_val >= vision_conf_min:
                try:
                    vis_x = float(vision_meas.get("x_in", 0.0))
                    vis_y = float(vision_meas.get("y_in", 0.0))
                    vis_th = vision_meas.get("theta_deg", None)
                    vis_th_val = float(vis_th) if vis_th is not None else None
                    w *= _vision_weight(vision_cfg,
                                        (px_to_in(p.get("x", 0.0)), px_to_in(p.get("y", 0.0))),
                                        float(p.get("theta", 0.0)),
                                        (vis_x, vis_y),
                                        vis_th_val)
                except Exception:
                    pass
        w *= _apply_region_gate(cfg, state, p)
        w *= _apply_slope_field(cfg, state, p)
        p["w"] = w
        total_w += w
    n = len(state.particles)
    w_avg = total_w / max(1, n)
    aug = mcl.get("augmented", {}) if isinstance(mcl, dict) else {}
    augmented_enabled = int(aug.get("enabled", 0)) == 1
    if augmented_enabled:
        alpha_slow = float(aug.get("alpha_slow", 0.001))
        alpha_fast = float(aug.get("alpha_fast", 0.1))
        if state.w_slow is None:
            state.w_slow = w_avg
        else:
            state.w_slow += alpha_slow * (w_avg - state.w_slow)
        if state.w_fast is None:
            state.w_fast = w_avg
        else:
            state.w_fast += alpha_fast * (w_avg - state.w_fast)
    _normalize_weights(state.particles)
    state.n_eff = _effective_n(state.particles)
    state.estimate = _estimate_pose(state.particles)
    if state.estimate is not None:
        state.cov_xy = _cov_xy(state.particles, state.estimate)
    state.confidence = _clamp(1.0 - (state.n_eff / max(1.0, float(n))), 0.0, 1.0)
    state.stats = {
        "n": float(n),
        "n_eff": state.n_eff,
        "w_avg": w_avg,
        "w_slow": 0.0 if state.w_slow is None else state.w_slow,
        "w_fast": 0.0 if state.w_fast is None else state.w_fast,
        "confidence": state.confidence,
    }
    conf_cfg = mcl.get("confidence", {}) if isinstance(mcl, dict) else {}
    conf_thresh = float(conf_cfg.get("threshold", 0.0))
    if int(conf_cfg.get("auto_reinit", 0)) == 1 and conf_thresh > 0.0 and state.confidence < conf_thresh:
        mode = str(conf_cfg.get("reinit_mode", "global")).strip().lower()
        if mode == "estimate" and state.estimate is not None:
            _reset_state_to_pose(state, cfg, state.estimate)
        else:
            state.particles = init_particles(cfg, n, state.rng)
        _normalize_weights(state.particles)
        state.n_eff = _effective_n(state.particles)
        state.estimate = _estimate_pose(state.particles)
        if state.estimate is not None:
            state.cov_xy = _cov_xy(state.particles, state.estimate)
        state.confidence = _clamp(1.0 - (state.n_eff / max(1.0, float(n))), 0.0, 1.0)
        state.stats["reinit"] = 1.0
        return
    res_cfg = mcl.get("resample", {}) if isinstance(mcl, dict) else {}
    method = str(res_cfg.get("method", "systematic")).strip().lower()
    threshold = float(res_cfg.get("threshold", 0.5))
    do_resample = int(res_cfg.get("always", 0)) == 1 or state.n_eff < threshold * max(1, n)
    if do_resample:
        n_target = _kld_target(cfg, state.particles)
        if n_target <= 0:
            n_target = n
        if method == "stratified":
            new_particles = _resample_stratified(state.particles, n_target, state.rng)
        elif method == "multinomial":
            new_particles = _resample_multinomial(state.particles, n_target, state.rng)
        else:
            new_particles = _resample_systematic(state.particles, n_target, state.rng)
        inj = float(mcl.get("random_injection", 0.0))
        if augmented_enabled and state.w_slow is not None and state.w_fast is not None and state.w_slow > 1e-9:
            inj = max(inj, max(0.0, 1.0 - state.w_fast / state.w_slow))
        if inj > 0.0:
            for i in range(len(new_particles)):
                if state.rng.random() < inj:
                    new_particles[i] = _sample_random_pose(cfg, state.rng)
        _normalize_weights(new_particles)
        state.particles = new_particles
        state.stats["resampled"] = 1.0
        state.stats["n_target"] = float(n_target)
        state.stats["n_eff_resampled"] = _effective_n(state.particles)
    state.last_measurements = dict(measurements)
    _update_ray_overlays(state, cfg, measurements, state.estimate)


def simulate_measurements(state: MCLState, cfg: dict, true_pose: Tuple[float, float, float],
                          add_noise: bool = True) -> dict:
    """Handle simulate measurements."""
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    imu_cfg = sensors_cfg.get("imu", {}) if isinstance(sensors_cfg, dict) else {}
    vision_cfg = sensors_cfg.get("vision", {}) if isinstance(sensors_cfg, dict) else {}
    use_distance = int(dist_cfg.get("enabled", 1)) == 1
    use_imu = int(imu_cfg.get("enabled", 1)) == 1
    use_vision = int(vision_cfg.get("enabled", 0)) == 1
    sensors = get_distance_sensors(cfg) if use_distance else []
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    max_range_px = max_range_mm / MM_PER_IN * PPI
    sigma_hit = float(dist_cfg.get("sigma_hit_mm", 8.5))
    imu_sigma = float(imu_cfg.get("sigma_deg", 1.0))
    vision_sigma = float(vision_cfg.get("sigma_xy_in", 2.0))
    vision_theta_sigma = float(vision_cfg.get("sigma_theta_deg", 5.0))
    meas = {}
    rays = []
    rng = state.rng
    if use_distance:
        dist_meas = {}
        for sensor in sensors:
            name = sensor.get("name", "")
            origin, heading = _sensor_world_pose(true_pose[0], true_pose[1], true_pose[2], sensor)
            segs = _sensor_segments(state, sensor)
            s_max = float(sensor.get("max_range_mm", max_range_mm))
            if s_max <= 0.0:
                s_max = max_range_mm
            max_range_px = s_max / MM_PER_IN * PPI
            pred_px = raycast_distance(segs, origin, heading, max_range_px)
            pred_px = max_range_px if pred_px is None else pred_px
            pred_mm = px_to_in(pred_px) * MM_PER_IN
            meas_mm = pred_mm
            if add_noise and sigma_hit > 0.0:
                meas_mm = max(0.0, pred_mm + rng.gauss(0.0, sigma_hit))
            meas_mm = max(0.0, min(s_max, meas_mm))
            dist_meas[name] = meas_mm
            meas_px = meas_mm / MM_PER_IN * PPI
            meas_px = max(0.0, min(max_range_px, meas_px))
            ux, uy = _heading_unit(heading)
            meas_end = (
                origin[0] + ux * meas_px,
                origin[1] + uy * meas_px,
            )
            rays.append({
                "name": name,
                "origin": origin,
                "angle_deg": heading,
                "meas_end": meas_end,
                "meas_mm": meas_mm,
                "pred_mm": pred_mm,
            })
        meas["distance"] = dist_meas
    if use_imu:
        imu_deg = float(true_pose[2])
        if add_noise and imu_sigma > 0.0:
            imu_deg = _wrap_deg(imu_deg + rng.gauss(0.0, imu_sigma))
        meas["imu"] = _wrap_deg(imu_deg)
    if use_vision:
        vis_x = px_to_in(true_pose[0])
        vis_y = px_to_in(true_pose[1])
        if add_noise and vision_sigma > 0.0:
            vis_x += rng.gauss(0.0, vision_sigma)
            vis_y += rng.gauss(0.0, vision_sigma)
        vis_th = float(true_pose[2])
        if add_noise and vision_theta_sigma > 0.0:
            vis_th = _wrap_deg(vis_th + rng.gauss(0.0, vision_theta_sigma))
        meas["vision"] = {
            "x_in": vis_x,
            "y_in": vis_y,
            "theta_deg": _wrap_deg(vis_th),
            "confidence": float(vision_cfg.get("sim_confidence", 1.0)),
        }
    state.last_rays = rays
    state.last_measurements = dict(meas)
    return meas
