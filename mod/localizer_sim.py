from __future__ import annotations

import math
import random
import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

try:
    from .mcl import (
        MM_PER_IN, PPI, _angle_diff_deg, _clamp, _ekf_cfg, _entry_default_enabled, _heading_from_unit, _heading_unit, _mat3_inv,
        _mat3_mul, _mat3_transpose, _pose_outside_perimeter, _sensor_segments,
        _sensor_world_pose, _wrap_deg, _ray_segment_intersection,
        build_object_segments_by_id,
        build_perimeter_segments, get_distance_sensors as _legacy_get_distance_sensors,
        in_to_px, px_to_in, raycast_distance, rotate_robot_to_world,
        simulate_motion_input as _legacy_simulate_motion_input,
    )
    from .draw import get_field_object_entries
except Exception:
    from mcl import (  # type: ignore
        MM_PER_IN, PPI, _angle_diff_deg, _clamp, _ekf_cfg, _entry_default_enabled, _heading_from_unit, _heading_unit, _mat3_inv,
        _mat3_mul, _mat3_transpose, _pose_outside_perimeter, _sensor_segments,
        _sensor_world_pose, _wrap_deg, _ray_segment_intersection,
        build_object_segments_by_id,
        build_perimeter_segments, get_distance_sensors as _legacy_get_distance_sensors,
        in_to_px, px_to_in, raycast_distance, rotate_robot_to_world,
        simulate_motion_input as _legacy_simulate_motion_input,
    )
    from draw import get_field_object_entries  # type: ignore


def _pose_is_finite(pose: Tuple[float, float, float]) -> bool:
    return all(math.isfinite(float(v)) for v in pose)


def _pose_from_any(state: "MCLState", pose: Optional[Tuple[float, float, float]] = None) -> Optional[Tuple[float, float, float]]:
    if pose is None:
        pose = state.estimate or state.ekf_pose or state.last_true_pose
    if pose is None:
        return None
    try:
        out = (float(pose[0]), float(pose[1]), _wrap_deg(float(pose[2])))
    except Exception:
        return None
    return out if _pose_is_finite(out) else None


_RELIABLE_SURFACE_MARGIN_PX = 1.5 * PPI
_EPS_DERIV_PX = 0.25 * PPI
_EPS_DERIV_THETA_DEG = 1.0
_THETA_ALIGN_MIN_DT_S = 0.020
_THETA_ALIGN_MIN_FORWARD_SPEED_IN_S = 2.0
_THETA_ALIGN_RATE_ALPHA = 0.35
_THETA_ALIGN_GAIN = 0.6
_THETA_ALIGN_MAX_STEP_DEG = 1.25
_THETA_ALIGN_MAX_START_ERROR_DEG = 20.0
_THETA_ALIGN_MAX_ERROR_DEG = 30.0
_THETA_ALIGN_RATE_ERROR_TOLERANCE_DEG = 12.0
_THETA_ALIGN_MAX_BEAM_TO_NORMAL_DEG = 25.0
_THETA_ALIGN_TANGENT_TOLERANCE_DEG = 10.0
_ROUGH_WALL_SIGMA_X_SCALE = 1.8
_ROUGH_WALL_SIGMA_Y_SCALE = 3.5
_ROUGH_WALL_SIGMA_THETA_SCALE = 1.25


def _clamp_magnitude(value: float, limit: float) -> float:
    """Clamp a signed value to +/- limit."""
    lim = max(0.0, float(limit))
    return max(-lim, min(lim, float(value)))


def _body_forward_from_world_delta(dx_px: float, dy_px: float, heading_deg: float) -> float:
    """Project a field-frame delta onto the robot forward axis."""
    fx, fy = _heading_unit(heading_deg)
    return dx_px * fx + dy_px * fy


def _atticus_cfg(cfg: dict) -> dict:
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    return mcl if isinstance(mcl, dict) else {}


def _distance_cfg(cfg: dict) -> dict:
    sensors_cfg = _atticus_cfg(cfg).get("sensors", {})
    if not isinstance(sensors_cfg, dict):
        return {}
    dist_cfg = sensors_cfg.get("distance", {})
    return dist_cfg if isinstance(dist_cfg, dict) else {}


def _motion_cfg(cfg: dict) -> dict:
    """Return normalized Atticus motion settings."""
    motion = _atticus_cfg(cfg).get("motion", {})
    return motion if isinstance(motion, dict) else {}


def _horizontal_odom_enabled(cfg: dict) -> bool:
    tracking_cfg = _atticus_cfg(cfg).get("tracking", {})
    if isinstance(tracking_cfg, dict) and "horizontal_enabled" in tracking_cfg:
        try:
            return int(tracking_cfg.get("horizontal_enabled", 0)) == 1
        except Exception:
            return bool(tracking_cfg.get("horizontal_enabled", False))
    try:
        return int(cfg.get("robot_physics", {}).get("tracking_wheels", 0)) >= 2
    except Exception:
        return False


def _default_distance_sensors_native() -> List[dict]:
    return [
        {
            "name": "front",
            "x_in": 0.0,
            "y_in": 0.0,
            "angle_deg": 0.0,
            "enabled": 0,
            "bias_mm": 0.0,
            "angle_offset_deg": 0.0,
            "min_range_mm": 20.0,
            "max_range_mm": 2000.0,
            "min_confidence": 0.0,
            "min_object_size": 0.0,
            "max_object_size": 0.0,
            "innovation_gate_mm": 180.0,
            "map_mode": "perimeter",
        },
        {
            "name": "right",
            "x_in": 5.75,
            "y_in": 0.0,
            "angle_deg": 90.0,
            "enabled": 1,
            "bias_mm": 0.0,
            "angle_offset_deg": 0.0,
            "min_range_mm": 20.0,
            "max_range_mm": 2000.0,
            "min_confidence": 0.0,
            "min_object_size": 0.0,
            "max_object_size": 0.0,
            "innovation_gate_mm": 180.0,
            "map_mode": "perimeter",
        },
        {
            "name": "back",
            "x_in": 0.0,
            "y_in": 0.0,
            "angle_deg": 180.0,
            "enabled": 0,
            "bias_mm": 0.0,
            "angle_offset_deg": 0.0,
            "min_range_mm": 20.0,
            "max_range_mm": 2000.0,
            "min_confidence": 0.0,
            "min_object_size": 0.0,
            "max_object_size": 0.0,
            "innovation_gate_mm": 180.0,
            "map_mode": "perimeter",
        },
        {
            "name": "left",
            "x_in": -5.75,
            "y_in": 0.0,
            "angle_deg": -90.0,
            "enabled": 1,
            "bias_mm": 0.0,
            "angle_offset_deg": 0.0,
            "min_range_mm": 20.0,
            "max_range_mm": 2000.0,
            "min_confidence": 0.0,
            "min_object_size": 0.0,
            "max_object_size": 0.0,
            "innovation_gate_mm": 180.0,
            "map_mode": "perimeter",
        },
    ]


def _normalize_distance_sensor_token(sensor_token: object) -> str:
    """Normalize user-facing sensor tokens like RIGHT or DistanceSensorId::RIGHT."""
    token = str(sensor_token or "").strip()
    if not token:
        return ""
    token = re.split(r"[\s,;/|+]+", token, maxsplit=1)[0]
    token = token.split("::")[-1]
    token = token.strip().upper()
    alias_map = {
        "F": "FRONT",
        "FR": "FRONT",
        "FRONT": "FRONT",
        "R": "RIGHT",
        "RT": "RIGHT",
        "RIGHT": "RIGHT",
        "B": "BACK",
        "BK": "BACK",
        "BACK": "BACK",
        "L": "LEFT",
        "LT": "LEFT",
        "LEFT": "LEFT",
        "AUTO": "",
        "ALL": "",
        "ANY": "",
    }
    return alias_map.get(token, token)


def _sensor_visibility_cfg(cfg: dict) -> dict:
    sensor_obj_vis_cfg = _atticus_cfg(cfg).get("sensor_object_visibility", {})
    return sensor_obj_vis_cfg if isinstance(sensor_obj_vis_cfg, dict) else {}


def get_distance_sensors(cfg: dict, include_disabled: bool = False) -> List[dict]:
    """Return Atticus distance sensors in native robot coordinates."""
    dist_cfg = _distance_cfg(cfg)
    geom = _atticus_cfg(cfg).get("sensor_geometry", {})
    sensors = geom.get("distance_sensors", []) if isinstance(geom, dict) else []
    defaults = _default_distance_sensors_native()
    vis_cfg = _sensor_visibility_cfg(cfg)

    def _num(v: object, default: float = 0.0) -> float:
        try:
            return float(v)
        except Exception:
            return float(default)

    def _sensor_vis(idx: int, name: str) -> dict:
        vis = vis_cfg.get(str(idx), {})
        if not isinstance(vis, dict):
            vis = vis_cfg.get(name, {})
        if not isinstance(vis, dict):
            vis = vis_cfg.get(f"sensor_{idx + 1}", {})
        return dict(vis) if isinstance(vis, dict) else {}

    out: List[dict] = []
    if not isinstance(sensors, list) or not sensors:
        sensors = defaults
    for idx in range(max(len(defaults), len(sensors))):
        base = dict(defaults[idx]) if idx < len(defaults) else {}
        raw = sensors[idx] if idx < len(sensors) and isinstance(sensors[idx], dict) else {}
        entry = dict(base)
        entry.update(raw)
        entry["name"] = str(entry.get("name", f"sensor_{idx + 1}") or f"sensor_{idx + 1}")
        entry["x_in"] = _num(entry.get("x_in", 0.0), 0.0)
        entry["y_in"] = _num(entry.get("y_in", 0.0), 0.0)
        entry["angle_deg"] = _num(entry.get("angle_deg", 0.0), 0.0)
        entry["bias_mm"] = _num(entry.get("bias_mm", 0.0), 0.0)
        entry["angle_offset_deg"] = _num(entry.get("angle_offset_deg", 0.0), 0.0)
        entry["min_range_mm"] = _num(entry.get("min_range_mm", dist_cfg.get("min_range_mm", 20.0)), 20.0)
        entry["max_range_mm"] = _num(entry.get("max_range_mm", dist_cfg.get("max_range_mm", 2000.0)), 2000.0)
        entry["min_confidence"] = _num(entry.get("min_confidence", dist_cfg.get("confidence_min", 0.0)), 0.0)
        entry["min_object_size"] = _num(entry.get("min_object_size", dist_cfg.get("object_size_min", 0.0)), 0.0)
        entry["max_object_size"] = _num(entry.get("max_object_size", dist_cfg.get("object_size_max", 0.0)), 0.0)
        entry["innovation_gate_mm"] = _num(entry.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)), 180.0)
        mode = str(entry.get("map_mode", "perimeter") or "perimeter").strip().lower()
        entry["map_mode"] = mode if mode in ("both", "perimeter", "objects") else "perimeter"
        try:
            entry["enabled"] = 1 if int(entry.get("enabled", 0)) == 1 else 0
        except Exception:
            entry["enabled"] = 1 if entry.get("enabled") else 0
        entry["object_visibility"] = _sensor_vis(idx, entry["name"])
        if include_disabled or entry["enabled"] == 1:
            out.append(entry)
    return out


def _runtime_sensor(sensor: dict) -> dict:
    """Convert native Atticus sensor geometry into the legacy sim robot frame."""
    runtime = dict(sensor)
    right_in = float(sensor.get("x_in", 0.0))
    forward_in = float(sensor.get("y_in", 0.0))
    runtime["x_in"] = forward_in
    runtime["y_in"] = -right_in
    runtime["angle_deg"] = float(sensor.get("angle_deg", 0.0))
    return runtime


def simulate_motion_input(state: "MCLState",
                          new_pose: Tuple[float, float, float],
                          prev_pose: Optional[Tuple[float, float, float]] = None) -> Optional[dict]:
    """Attach the adapter pose so the predictor can mirror Atticus teleport resets."""
    motion_input = _legacy_simulate_motion_input(state, new_pose, prev_pose)
    if isinstance(motion_input, dict):
        motion_input["pose"] = (
            float(new_pose[0]),
            float(new_pose[1]),
            float(new_pose[2]),
        )
    return motion_input


def _feature_meta_for_kind(kind: str) -> dict:
    kind = str(kind or "").strip().lower()
    if kind == "perimeter":
        return {"sigma_scale": 0.8, "definite": True, "priority": 3, "feature": "perimeter"}
    if kind == "long_goal_brace":
        return {"sigma_scale": 0.85, "definite": True, "priority": 3, "feature": "long_goal_support"}
    if kind == "long_goal":
        return {"sigma_scale": 1.35, "definite": False, "priority": 2, "feature": "long_goal_body"}
    if kind == "center_goal":
        return {"sigma_scale": 1.5, "definite": False, "priority": 1, "feature": "center_goal"}
    if kind == "matchloader":
        return {"sigma_scale": 1.65, "definite": False, "priority": 0, "feature": "matchloader"}
    return {"sigma_scale": 1.0, "definite": False, "priority": -1, "feature": "unknown"}


def _object_kinds_by_id(cfg: dict) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for entry in get_field_object_entries(cfg):
        obj_id = str(entry.get("id", "") or "").strip()
        if not obj_id:
            continue
        out[obj_id] = str(entry.get("kind", "") or "")
    return out


def _sensor_allows_object(sensor: dict, obj_id: str) -> bool:
    vis = sensor.get("object_visibility", {})
    if not isinstance(vis, dict) or obj_id not in vis:
        return True
    try:
        return int(vis.get(obj_id, 1)) == 1
    except Exception:
        return bool(vis.get(obj_id, True))


def _update_best_hit(best: Optional[dict],
                     distance_px: float,
                     segment: Tuple[float, float, float, float],
                     meta: dict,
                     object_id: Optional[str] = None) -> dict:
    nearer = best is None or distance_px < float(best.get("distance_px", 0.0))
    preferred_same_band = (
        best is not None
        and int(meta.get("priority", -1)) > int(best.get("priority", -1))
        and distance_px <= float(best.get("distance_px", 0.0)) + _RELIABLE_SURFACE_MARGIN_PX
    )
    if nearer or preferred_same_band:
        return {
            "distance_px": float(distance_px),
            "segment": segment,
            "sigma_scale": float(meta.get("sigma_scale", 1.0)),
            "definite": bool(meta.get("definite", False)),
            "priority": int(meta.get("priority", -1)),
            "feature": str(meta.get("feature", "unknown")),
            "object_id": object_id,
        }
    return best if best is not None else {
        "distance_px": float(distance_px),
        "segment": segment,
        "sigma_scale": float(meta.get("sigma_scale", 1.0)),
        "definite": bool(meta.get("definite", False)),
        "priority": int(meta.get("priority", -1)),
        "feature": str(meta.get("feature", "unknown")),
        "object_id": object_id,
    }


def _raycast_expected_measurement(state: "MCLState",
                                  cfg: dict,
                                  pose: Tuple[float, float, float],
                                  sensor: dict) -> Optional[dict]:
    runtime_sensor = _runtime_sensor(sensor)
    origin, heading = _sensor_world_pose(pose[0], pose[1], pose[2], runtime_sensor)
    max_range_mm = float(sensor.get("max_range_mm", _distance_cfg(cfg).get("max_range_mm", 2000.0)))
    max_range_px = max_range_mm / MM_PER_IN * PPI
    mode = str(sensor.get("map_mode", "perimeter") or "perimeter").strip().lower()
    object_kinds = _object_kinds_by_id(cfg)
    best: Optional[dict] = None

    if mode in ("both", "perimeter"):
        for seg in state.map_segments_perim:
            t = _ray_segment_intersection(origin[0], origin[1], *_heading_unit(heading), *seg)
            if t is None or t < 0.0 or t > max_range_px:
                continue
            best = _update_best_hit(best, t, seg, _feature_meta_for_kind("perimeter"))

    if mode in ("both", "objects"):
        for obj_id, segs in state.map_segments_obj_by_id.items():
            if not _sensor_allows_object(sensor, obj_id):
                continue
            meta = _feature_meta_for_kind(object_kinds.get(obj_id, ""))
            for seg in segs:
                t = _ray_segment_intersection(origin[0], origin[1], *_heading_unit(heading), *seg)
                if t is None or t < 0.0 or t > max_range_px:
                    continue
                best = _update_best_hit(best, t, seg, meta, object_id=obj_id)

    if best is None:
        return None
    distance_mm = px_to_in(float(best["distance_px"])) * MM_PER_IN
    return {
        "origin": origin,
        "heading_deg": heading,
        "distance_px": float(best["distance_px"]),
        "distance_mm": distance_mm,
        "segment": best["segment"],
        "sigma_scale": float(best.get("sigma_scale", 1.0)),
        "definite": bool(best.get("definite", False)),
        "feature": best.get("feature", "unknown"),
        "object_id": best.get("object_id"),
    }


def _distance_jacobian(state: "MCLState", cfg: dict, sensor: dict, expected_mm: float) -> Optional[Tuple[float, float, float]]:
    if state.ekf_pose is None:
        return None

    def _expected_or_fallback(pose: Tuple[float, float, float]) -> float:
        hit = _raycast_expected_measurement(state, cfg, pose, sensor)
        if hit is None:
            return expected_mm
        return float(hit["distance_mm"])

    plus = (state.ekf_pose[0] + _EPS_DERIV_PX, state.ekf_pose[1], state.ekf_pose[2])
    minus = (state.ekf_pose[0] - _EPS_DERIV_PX, state.ekf_pose[1], state.ekf_pose[2])
    hx = (_expected_or_fallback(plus) - _expected_or_fallback(minus)) / (2.0 * _EPS_DERIV_PX)

    plus = (state.ekf_pose[0], state.ekf_pose[1] + _EPS_DERIV_PX, state.ekf_pose[2])
    minus = (state.ekf_pose[0], state.ekf_pose[1] - _EPS_DERIV_PX, state.ekf_pose[2])
    hy = (_expected_or_fallback(plus) - _expected_or_fallback(minus)) / (2.0 * _EPS_DERIV_PX)

    plus = (state.ekf_pose[0], state.ekf_pose[1], _wrap_deg(state.ekf_pose[2] + _EPS_DERIV_THETA_DEG))
    minus = (state.ekf_pose[0], state.ekf_pose[1], _wrap_deg(state.ekf_pose[2] - _EPS_DERIV_THETA_DEG))
    ht = (_expected_or_fallback(plus) - _expected_or_fallback(minus)) / (2.0 * _EPS_DERIV_THETA_DEG)

    if abs(hx) < 1e-6 and abs(hy) < 1e-6 and abs(ht) < 1e-6:
        return None
    return (hx, hy, ht)


def _cov_bounds(cfg: dict) -> Tuple[float, float, float, float, float, float]:
    ekf = _ekf_cfg(cfg)
    min_xy = max(1e-9, float(ekf["init_sigma_xy_in"]) * PPI) ** 2
    max_xy = max(1e-9, float(ekf["pose_sigma_x_max"]) * PPI, float(ekf["pose_sigma_y_max"]) * PPI) ** 2
    min_th = max(1e-9, float(ekf["init_sigma_theta_deg"])) ** 2
    max_th = max(1e-9, float(ekf["pose_sigma_theta_max"])) ** 2
    return min_xy, max_xy, min_xy, max_xy, min_th, max_th


def _stabilize_covariance(P: List[List[float]], cfg: dict) -> List[List[float]]:
    min_x2, max_x2, min_y2, max_y2, min_t2, max_t2 = _cov_bounds(cfg)
    out = [[float(P[i][j]) if math.isfinite(float(P[i][j])) else 0.0 for j in range(3)] for i in range(3)]
    for i in range(3):
        for j in range(i + 1, 3):
            v = 0.5 * (out[i][j] + out[j][i])
            out[i][j] = v
            out[j][i] = v
    out[0][0] = _clamp(out[0][0], min_x2, max_x2)
    out[1][1] = _clamp(out[1][1], min_y2, max_y2)
    out[2][2] = _clamp(out[2][2], min_t2, max_t2)
    for i, j, lim in ((0, 1, math.sqrt(max(0.0, out[0][0] * out[1][1]))),
                      (0, 2, math.sqrt(max(0.0, out[0][0] * out[2][2]))),
                      (1, 2, math.sqrt(max(0.0, out[1][1] * out[2][2])))):
        lim = max(0.0, lim)
        out[i][j] = _clamp(out[i][j], -lim, lim)
        out[j][i] = out[i][j]
    return out


def _sync_pose_cache(state: "MCLState", cfg: dict) -> None:
    pose = _pose_from_any(state)
    if pose is None:
        state.cov_xy = (0.0, 0.0, 0.0)
        return
    state.estimate = pose
    state.ekf_pose = pose
    if state.ekf_P is not None:
        state.ekf_P = _stabilize_covariance(state.ekf_P, cfg)
        state.cov_xy = (
            float(state.ekf_P[0][0]),
            float(state.ekf_P[1][1]),
            float(state.ekf_P[0][1]),
        )
    else:
        state.cov_xy = (0.0, 0.0, 0.0)


def _sensor_measurement_mm(raw: object) -> Optional[float]:
    if isinstance(raw, dict):
        for key in ("mm", "distance_mm", "value", "measured_mm"):
            try:
                val = raw.get(key)
            except Exception:
                val = None
            if val is None:
                continue
            try:
                return float(val)
            except Exception:
                continue
        return None
    try:
        return float(raw)
    except Exception:
        return None


def _distance_sigma_mm(dist_cfg: dict, meas_mm: float, sigma_scale: float = 1.0) -> float:
    sigma_hit = float(dist_cfg.get("sigma_hit_mm", 15.0))
    sigma_far_scale = float(dist_cfg.get("sigma_far_scale", 0.05))
    sigma_min = float(dist_cfg.get("sigma_min_mm", 15.0))
    sigma_max = float(dist_cfg.get("sigma_max_mm", 100.0))
    sigma = max(sigma_hit, sigma_far_scale * max(0.0, float(meas_mm))) * _clamp(float(sigma_scale), 0.5, 3.0)
    return _clamp(sigma, sigma_min, sigma_max)


def _scalar_kf_update(state: "MCLState", cfg: dict, H: Tuple[float, float, float], innovation: float, R: float) -> bool:
    if state.ekf_pose is None or state.ekf_P is None:
        return False
    P = state.ekf_P
    h0, h1, h2 = float(H[0]), float(H[1]), float(H[2])
    ph = [
        P[0][0] * h0 + P[0][1] * h1 + P[0][2] * h2,
        P[1][0] * h0 + P[1][1] * h1 + P[1][2] * h2,
        P[2][0] * h0 + P[2][1] * h1 + P[2][2] * h2,
    ]
    S = h0 * ph[0] + h1 * ph[1] + h2 * ph[2] + R
    if S <= 1e-12 or not math.isfinite(S):
        return False
    K = [ph[i] / S for i in range(3)]
    x, y, th = state.ekf_pose
    x += K[0] * innovation
    y += K[1] * innovation
    th = _wrap_deg(th + K[2] * innovation)
    I_KH = [
        [1.0 - K[0] * h0, -K[0] * h1, -K[0] * h2],
        [-K[1] * h0, 1.0 - K[1] * h1, -K[1] * h2],
        [-K[2] * h0, -K[2] * h1, 1.0 - K[2] * h2],
    ]
    temp = _mat3_mul(I_KH, P)
    IKHt = _mat3_transpose(I_KH)
    Pnew = _mat3_mul(temp, IKHt)
    KRKt = [
        [K[0] * K[0] * R, K[0] * K[1] * R, K[0] * K[2] * R],
        [K[1] * K[0] * R, K[1] * K[1] * R, K[1] * K[2] * R],
        [K[2] * K[0] * R, K[2] * K[1] * R, K[2] * K[2] * R],
    ]
    state.ekf_pose = (x, y, th)
    state.estimate = state.ekf_pose
    state.ekf_P = [[Pnew[i][j] + KRKt[i][j] for j in range(3)] for i in range(3)]
    _sync_pose_cache(state, cfg)
    return True


def _refresh_confidence(state: "MCLState", cfg: dict, accepted: int = 0, total: int = 0,
                        innovation_rms_mm: Optional[float] = None) -> None:
    if state.ekf_P is None:
        state.confidence = 0.0
        state.ess_ratio = 1.0
        return
    ekf = _ekf_cfg(cfg)
    sx = math.sqrt(max(0.0, float(state.ekf_P[0][0]))) / PPI
    sy = math.sqrt(max(0.0, float(state.ekf_P[1][1]))) / PPI
    sth = math.sqrt(max(0.0, float(state.ekf_P[2][2])))
    max_xy = max(1e-6, float(ekf["pose_sigma_x_max"]), float(ekf["pose_sigma_y_max"]))
    max_th = max(1e-6, float(ekf["pose_sigma_theta_max"]))
    cov_score = 1.0 / (1.0 + max(sx / max_xy, sy / max_xy, sth / max_th))
    meas_score = float(accepted) / float(total) if total > 0 else max(0.0, state.confidence * 0.98)
    qual_score = 1.0
    if innovation_rms_mm is not None:
        scale = max(1.0, float(cfg.get("mcl", {}).get("sensors", {}).get("distance", {}).get("max_range_mm", 2000.0)) * 0.25)
        qual_score = math.exp(-min(8.0, innovation_rms_mm / scale))
    base = 0.15 + 0.55 * meas_score + 0.30 * qual_score
    state.confidence = _clamp(base * cov_score, 0.0, 1.0)
    state.ess_ratio = _clamp(1.0 - state.confidence, 0.0, 1.0)


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
    # Measurements may contain nested payloads such as {"distance": {...}}.
    last_measurements: Dict[str, object] = field(default_factory=dict)
    dist_hist: Dict[str, List[float]] = field(default_factory=dict)
    region_id: Optional[object] = None
    region_gate: Optional[dict] = None
    region_changed: bool = False
    w_slow: Optional[float] = None
    w_fast: Optional[float] = None
    confidence: float = 0.0
    ess_ratio: float = 1.0
    ekf_pose: Optional[Tuple[float, float, float]] = None
    ekf_P: Optional[List[List[float]]] = None
    rough_wall_ctx: Optional[dict] = None


def ekf_reset_state(state: MCLState, cfg: dict, pose: Optional[Tuple[float, float, float]] = None) -> None:
    pose = _pose_from_any(state, pose)
    if pose is None:
        return
    ekf = _ekf_cfg(cfg)
    sigma_xy_px = max(1e-6, float(ekf["init_sigma_xy_in"]) * PPI)
    sigma_th = max(1e-6, float(ekf["init_sigma_theta_deg"]))
    state.estimate = pose
    state.ekf_pose = pose
    state.ekf_P = [
        [sigma_xy_px * sigma_xy_px, 0.0, 0.0],
        [0.0, sigma_xy_px * sigma_xy_px, 0.0],
        [0.0, 0.0, sigma_th * sigma_th],
    ]
    state.cov_xy = (state.ekf_P[0][0], state.ekf_P[1][1], 0.0)
    _refresh_confidence(state, cfg)


def ekf_predict(state: MCLState, cfg: dict, motion_input: Optional[dict]) -> None:
    if not motion_input:
        return
    motion_cfg = _motion_cfg(cfg)
    if isinstance(motion_cfg, dict) and int(motion_cfg.get("enabled", 1)) != 1:
        return
    try:
        dx_in = float(motion_input.get("dx_in", 0.0))
        dy_in = float(motion_input.get("dy_in", 0.0))
        dtheta = float(motion_input.get("dtheta_deg", 0.0))
    except Exception:
        return
    if abs(dx_in) + abs(dy_in) + abs(dtheta) <= 1e-9:
        return
    pose_raw = motion_input.get("pose")
    pose = None
    if isinstance(pose_raw, (list, tuple)) and len(pose_raw) >= 3:
        try:
            pose = (float(pose_raw[0]), float(pose_raw[1]), float(pose_raw[2]))
        except Exception:
            pose = None
    corr_cfg = _atticus_cfg(cfg).get("correction", {})
    if isinstance(corr_cfg, dict):
        teleport_trans_in = float(corr_cfg.get("teleport_reset_trans_in", 18.0))
        teleport_theta_deg = float(corr_cfg.get("teleport_reset_theta_deg", 45.0))
    else:
        teleport_trans_in = 18.0
        teleport_theta_deg = 45.0
    if math.hypot(dx_in, dy_in) > teleport_trans_in or abs(dtheta) > teleport_theta_deg:
        if pose is not None:
            ekf_reset_state(state, cfg, pose)
        state.rough_wall_ctx = None
        return
    if state.ekf_pose is None or state.ekf_P is None:
        ekf_reset_state(state, cfg, pose)
        if state.ekf_pose is None or state.ekf_P is None:
            return
    noise_scale = float(motion_input.get("noise_scale", 1.0))
    if not math.isfinite(noise_scale) or noise_scale < 1.0:
        noise_scale = 1.0
    dx_px = dx_in * PPI
    dy_px = dy_in * PPI
    x, y, th = state.ekf_pose
    th_mid = _wrap_deg(th + 0.5 * dtheta)
    wx, wy = rotate_robot_to_world(dx_px, dy_px, th_mid)
    x += wx
    y += wy
    th = _wrap_deg(th + dtheta)
    rad_per_deg = math.pi / 180.0
    fx, fy = _heading_unit(th_mid)
    lx, ly = _heading_unit(th_mid - 90.0)
    F = [
        [1.0, 0.0, (dx_px * fx + dy_px * lx) * rad_per_deg],
        [0.0, 1.0, (dx_px * fy + dy_px * ly) * rad_per_deg],
        [0.0, 0.0, 1.0],
    ]
    L = [
        [fx, lx, 0.0],
        [fy, ly, 0.0],
        [0.0, 0.0, 1.0],
    ]
    right_delta_in = -dy_in
    forward_delta_in = dx_in
    q_right_in = float(motion_cfg.get("sigma_x_in", 0.08)) + abs(right_delta_in) * float(motion_cfg.get("sigma_x_per_in", 0.02))
    q_forward_in = float(motion_cfg.get("sigma_y_in", 0.08)) + abs(forward_delta_in) * float(motion_cfg.get("sigma_y_per_in", 0.03))
    qtheta_deg = float(motion_cfg.get("sigma_theta_deg", 0.7)) + abs(dtheta) * float(motion_cfg.get("sigma_theta_per_deg", 0.05))
    if not _horizontal_odom_enabled(cfg):
        q_right_in *= float(motion_cfg.get("no_horizontal_lateral_scale", 2.5))
        if abs(dtheta) > 5.0 or abs(right_delta_in) > 0.5:
            q_right_in *= float(motion_cfg.get("turn_lateral_scale", 2.0))
    if state.rough_wall_ctx:
        q_right_in *= float(motion_cfg.get("rough_wall_sigma_x_scale", _ROUGH_WALL_SIGMA_X_SCALE))
        q_forward_in *= float(motion_cfg.get("rough_wall_sigma_y_scale", _ROUGH_WALL_SIGMA_Y_SCALE))
        qtheta_deg *= float(motion_cfg.get("rough_wall_sigma_theta_scale", _ROUGH_WALL_SIGMA_THETA_SCALE))
    sigma_dx_px = q_forward_in * PPI * noise_scale
    sigma_dy_px = q_right_in * PPI * noise_scale
    sigma_dth = qtheta_deg * noise_scale
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
    state.ekf_pose = (x, y, th)
    state.estimate = state.ekf_pose
    state.ekf_P = _stabilize_covariance(Pnew, cfg)
    state.cov_xy = (state.ekf_P[0][0], state.ekf_P[1][1], state.ekf_P[0][1])
    _refresh_confidence(state, cfg)


def ekf_update_imu(state: MCLState, cfg: dict, heading_deg: float) -> None:
    if state.ekf_pose is None or state.ekf_P is None:
        ekf_reset_state(state, cfg)
        if state.ekf_pose is None or state.ekf_P is None:
            return
    ekf = _ekf_cfg(cfg)
    R = float(ekf["imu_sigma_deg"]) ** 2
    if _scalar_kf_update(state, cfg, (0.0, 0.0, 1.0), _angle_diff_deg(float(heading_deg), state.ekf_pose[2]), R):
        state.estimate = state.ekf_pose
        _refresh_confidence(state, cfg)


def ekf_update_mcl(state: MCLState, cfg: dict, mcl_pose: Tuple[float, float, float], confidence: float) -> None:
    """Compatibility shim: this backend already uses a single EKF estimate."""
    pose = _pose_from_any(state, mcl_pose)
    if pose is not None and (state.ekf_pose is None or state.ekf_P is None):
        ekf_reset_state(state, cfg, pose)
    _sync_pose_cache(state, cfg)
    _refresh_confidence(state, cfg)


def reset_state_to_pose(state: MCLState, cfg: dict, pose: Tuple[float, float, float]) -> None:
    pose = _pose_from_any(state, pose)
    if pose is None:
        return
    state.particles = []
    state.estimate = pose
    state.last_true_pose = pose
    state.motion_accum = 0.0
    state.sensor_accum = 0.0
    state.last_rays = []
    state.last_measurements = {}
    state.dist_hist = {}
    state.stats = {}
    state.w_slow = None
    state.w_fast = None
    state.confidence = 0.0
    state.ess_ratio = 1.0
    state.rough_wall_ctx = None
    ekf_reset_state(state, cfg, pose)


def update_map_segments(state: MCLState, cfg: dict) -> None:
    perim = build_perimeter_segments(cfg)
    obj_by_id = build_object_segments_by_id(cfg)
    obj: List[Tuple[float, float, float, float]] = []
    for segs in obj_by_id.values():
        obj.extend(segs)
    state.map_segments_perim = perim
    state.map_segments_obj_by_id = obj_by_id
    state.map_segments_obj = obj
    state.map_segments = perim + obj
    state.lf = {"segments": len(state.map_segments), "mode": "sim"}
    state.lf_perim = {"segments": len(perim), "mode": "perimeter"} if perim else None
    state.lf_obj = {"segments": len(obj), "mode": "objects"} if obj else None
    state.stats["map_segments"] = float(len(state.map_segments))


def motion_update(state: MCLState, cfg: dict, motion_input: Optional[dict]) -> None:
    if not motion_input:
        return
    state.stats["motion_seen"] = state.stats.get("motion_seen", 0.0) + 1.0


def simulate_measurements(state: MCLState, cfg: dict, true_pose: Tuple[float, float, float],
                          add_noise: bool = True) -> dict:
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    imu_cfg = sensors_cfg.get("imu", {}) if isinstance(sensors_cfg, dict) else {}
    use_distance = int(dist_cfg.get("enabled", 1)) == 1
    use_imu = int(imu_cfg.get("enabled", 1)) == 1
    sensors = get_distance_sensors(cfg) if use_distance else []
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    imu_sigma = float(imu_cfg.get("sigma_deg", 1.0))
    meas: Dict[str, object] = {}
    rays: List[dict] = []
    rng = state.rng
    if use_distance:
        dist_meas: Dict[str, float] = {}
        for sensor in sensors:
            name = str(sensor.get("name", "") or "")
            s_max = float(sensor.get("max_range_mm", max_range_mm))
            if s_max <= 0.0:
                s_max = max_range_mm
            hit = _raycast_expected_measurement(state, cfg, true_pose, sensor)
            if hit is None:
                continue
            origin = hit["origin"]
            heading = float(hit["heading_deg"])
            pred_px = float(hit["distance_px"])
            pred_mm = float(hit["distance_mm"])
            meas_mm = pred_mm
            if add_noise:
                sigma_mm = _distance_sigma_mm(dist_cfg, pred_mm, sigma_scale=float(hit.get("sigma_scale", 1.0)))
                meas_mm = _clamp(meas_mm + rng.gauss(0.0, sigma_mm), 0.0, s_max)
            dist_meas[name] = meas_mm
            meas_px = meas_mm / MM_PER_IN * PPI
            ux, uy = _heading_unit(heading)
            meas_end = (origin[0] + ux * meas_px, origin[1] + uy * meas_px)
            rays.append({
                "name": name,
                "origin": origin,
                "angle_deg": heading,
                "meas_end": meas_end,
                "meas_mm": meas_mm,
                "pred_mm": pred_mm,
                "pred_end": (origin[0] + ux * pred_px, origin[1] + uy * pred_px),
                "definite": bool(hit.get("definite", False)),
                "feature": str(hit.get("feature", "unknown")),
            })
        meas["distance"] = dist_meas
    if use_imu:
        imu_deg = float(true_pose[2])
        if add_noise and imu_sigma > 0.0:
            imu_deg = _wrap_deg(imu_deg + rng.gauss(0.0, imu_sigma))
        meas["imu"] = _wrap_deg(imu_deg)
    state.last_rays = rays
    state.last_measurements = dict(meas)
    return meas


def _update_ray_overlays(state: MCLState,
                         cfg: dict,
                         measurements: Optional[dict],
                         estimate: Optional[Tuple[float, float, float]]) -> None:
    """Rebuild displayed distance rays using native Atticus geometry and gating."""
    if measurements is None:
        return
    dist_meas = measurements.get("distance", {})
    if not isinstance(dist_meas, dict) or not dist_meas:
        return
    dist_cfg = _distance_cfg(cfg)
    sensors = get_distance_sensors(cfg)
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    new_rays: List[dict] = []
    for sensor in sensors:
        name = str(sensor.get("name", "") or "")
        if name not in dist_meas:
            continue
        meas_mm = _sensor_measurement_mm(dist_meas.get(name))
        if meas_mm is None or not math.isfinite(meas_mm):
            continue
        s_max = float(sensor.get("max_range_mm", max_range_mm))
        if s_max <= 0.0:
            s_max = max_range_mm
        display_hit = None
        if estimate is not None:
            display_hit = _raycast_expected_measurement(state, cfg, estimate, sensor)
        if display_hit is None and state.last_true_pose is not None:
            display_hit = _raycast_expected_measurement(state, cfg, state.last_true_pose, sensor)
        if display_hit is None:
            continue
        origin = display_hit["origin"]
        heading = float(display_hit["heading_deg"])
        pred_px = float(display_hit["distance_px"])
        pred_mm = float(display_hit["distance_mm"])
        bias_mm = float(sensor.get("bias_mm", 0.0))
        corrected_meas_mm = float(meas_mm) - bias_mm
        meas_px = _clamp(corrected_meas_mm / MM_PER_IN * PPI, 0.0, s_max / MM_PER_IN * PPI)
        ux, uy = _heading_unit(heading)
        gate_mm = float(sensor.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)))
        new_rays.append({
            "name": name,
            "origin": origin,
            "angle_deg": heading,
            "pred_origin": origin,
            "pred_end": (origin[0] + ux * pred_px, origin[1] + uy * pred_px),
            "pred_mm": pred_mm,
            "meas_end": (origin[0] + ux * meas_px, origin[1] + uy * meas_px),
            "meas_mm": corrected_meas_mm,
            "gated": gate_mm > 0.0 and abs(corrected_meas_mm - pred_mm) > gate_mm,
            "definite": bool(display_hit.get("definite", False)),
            "feature": str(display_hit.get("feature", "unknown")),
        })
    if new_rays:
        state.last_rays = new_rays


def sensor_update(state: MCLState, cfg: dict, measurements: Optional[dict]) -> None:
    if measurements is None:
        return
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    imu_cfg = sensors_cfg.get("imu", {}) if isinstance(sensors_cfg, dict) else {}
    use_distance = int(dist_cfg.get("enabled", 1)) == 1
    use_imu = int(imu_cfg.get("enabled", 1)) == 1
    if state.ekf_pose is None or state.ekf_P is None:
        if state.last_true_pose is not None:
            ekf_reset_state(state, cfg, state.last_true_pose)
        elif state.estimate is not None:
            ekf_reset_state(state, cfg, state.estimate)
        else:
            return
    dist_meas = measurements.get("distance", {})
    imu_meas = measurements.get("imu", None)
    sensors = get_distance_sensors(cfg) if use_distance else []
    min_range_mm = float(dist_cfg.get("min_range_mm", 20.0))
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    gate_mm = float(dist_cfg.get("innovation_gate_mm", 180.0))
    gate_mode = str(dist_cfg.get("gate_mode", "hard")).strip().lower()
    gate_penalty = float(dist_cfg.get("gate_penalty", 0.05))
    median_window = max(1, int(dist_cfg.get("median_window", 1)))
    rough_selected = None
    if isinstance(state.rough_wall_ctx, dict):
        selected_names = state.rough_wall_ctx.get("selected_sensor_names", [])
        if isinstance(selected_names, (list, tuple, set)):
            rough_selected = {str(name).strip() for name in selected_names if str(name).strip()}
    accepted = 0
    total = 0
    innov_sq = 0.0
    if use_distance and isinstance(dist_meas, dict):
        for sensor in sensors:
            name = str(sensor.get("name", "") or "")
            if rough_selected is not None and name not in rough_selected:
                continue
            if name not in dist_meas:
                continue
            meas_mm = _sensor_measurement_mm(dist_meas.get(name))
            if meas_mm is None or not math.isfinite(meas_mm):
                continue
            total += 1
            hist = state.dist_hist.setdefault(name, [])
            if median_window > 1:
                hist.append(meas_mm)
                if len(hist) > median_window:
                    hist.pop(0)
                meas_mm = sorted(hist)[len(hist) // 2]
            meas_mm -= float(sensor.get("bias_mm", 0.0))
            s_min = float(sensor.get("min_range_mm", min_range_mm))
            s_max = float(sensor.get("max_range_mm", max_range_mm))
            if s_max <= 0.0:
                s_max = max_range_mm
            if s_min <= 0.0:
                s_min = min_range_mm
            if meas_mm < s_min or meas_mm > s_max:
                continue
            expected = _raycast_expected_measurement(state, cfg, state.estimate, sensor)
            if expected is None:
                continue
            pred_mm = float(expected["distance_mm"])
            innov = meas_mm - pred_mm
            sensor_gate_mm = float(sensor.get("innovation_gate_mm", gate_mm))
            if sensor_gate_mm > 0.0 and abs(innov) > sensor_gate_mm:
                if gate_mode == "hard":
                    continue
                innov *= gate_penalty
            H = _distance_jacobian(state, cfg, sensor, pred_mm)
            if H is None:
                continue
            sigma_mm = _distance_sigma_mm(dist_cfg, meas_mm, sigma_scale=float(expected.get("sigma_scale", 1.0)))
            if _scalar_kf_update(state, cfg, H, innov, sigma_mm * sigma_mm):
                accepted += 1
                innov_sq += innov * innov
    if use_imu and imu_meas is not None:
        try:
            imu_deg = float(imu_meas)
        except Exception:
            imu_deg = None
        if imu_deg is not None and math.isfinite(imu_deg):
            ekf_update_imu(state, cfg, imu_deg)
    state.last_measurements = dict(measurements)
    if accepted > 0:
        rms = math.sqrt(innov_sq / float(accepted))
        _refresh_confidence(state, cfg, accepted=accepted, total=max(total, 1), innovation_rms_mm=rms)
    else:
        _refresh_confidence(state, cfg, accepted=0, total=max(total, 1))
    _update_ray_overlays(state, cfg, measurements, state.estimate)


def _raycast_segment_hit(segments: List[Tuple[float, float, float, float]],
                         origin: Tuple[float, float],
                         heading_deg: float,
                         max_range_px: float) -> Optional[dict]:
    """Return the nearest hit segment for a raycast, including hit point and distance."""
    dx, dy = _heading_unit(heading_deg)
    best_t = None
    best_seg = None
    for x0, y0, x1, y1 in segments:
        t = _ray_segment_intersection(origin[0], origin[1], dx, dy, x0, y0, x1, y1)
        if t is None or t < 0.0 or t > max_range_px:
            continue
        if best_t is None or t < best_t:
            best_t = t
            best_seg = (x0, y0, x1, y1)
    if best_t is None or best_seg is None:
        return None
    return {
        "distance_px": float(best_t),
        "segment": best_seg,
        "hit_point": (origin[0] + dx * best_t, origin[1] + dy * best_t),
    }


def _distance_update_from_measurements(state: MCLState,
                                       cfg: dict,
                                       measurements: Optional[dict],
                                       sensor_filter: Optional[set] = None) -> List[str]:
    """Apply one-shot translation updates from distance measurements and return accepted sensors."""
    if measurements is None:
        return []
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    if int(dist_cfg.get("enabled", 1)) != 1:
        return []
    if state.ekf_pose is None or state.ekf_P is None:
        if state.last_true_pose is not None:
            ekf_reset_state(state, cfg, state.last_true_pose)
        elif state.estimate is not None:
            ekf_reset_state(state, cfg, state.estimate)
        else:
            return []
    dist_meas = measurements.get("distance", {})
    if not isinstance(dist_meas, dict) or not dist_meas:
        return []
    sensors = get_distance_sensors(cfg)
    min_range_mm = float(dist_cfg.get("min_range_mm", 20.0))
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    median_window = max(1, int(dist_cfg.get("median_window", 1)))
    accepted = 0
    total = 0
    innov_sq = 0.0
    accepted_names: List[str] = []
    for sensor in sensors:
        name = str(sensor.get("name", "") or "")
        if sensor_filter is not None and name not in sensor_filter:
            continue
        if name not in dist_meas:
            continue
        meas_mm = _sensor_measurement_mm(dist_meas.get(name))
        if meas_mm is None or not math.isfinite(meas_mm):
            continue
        total += 1
        hist = state.dist_hist.setdefault(name, [])
        if median_window > 1:
            hist.append(meas_mm)
            if len(hist) > median_window:
                hist.pop(0)
            meas_mm = sorted(hist)[len(hist) // 2]
        meas_mm -= float(sensor.get("bias_mm", 0.0))
        s_min = float(sensor.get("min_range_mm", min_range_mm))
        s_max = float(sensor.get("max_range_mm", max_range_mm))
        if s_max <= 0.0:
            s_max = max_range_mm
        if s_min <= 0.0:
            s_min = min_range_mm
        if meas_mm < s_min or meas_mm > s_max:
            continue
        expected = _raycast_expected_measurement(state, cfg, state.estimate, sensor)
        if expected is None:
            continue
        pred_mm = float(expected["distance_mm"])
        innov = meas_mm - pred_mm
        gate_mm = float(sensor.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)))
        if gate_mm > 0.0 and abs(innov) > gate_mm:
            continue
        H = _distance_jacobian(state, cfg, sensor, pred_mm)
        if H is None:
            continue
        sigma_mm = _distance_sigma_mm(dist_cfg, meas_mm, sigma_scale=float(expected.get("sigma_scale", 1.0)))
        if _scalar_kf_update(state, cfg, H, innov, sigma_mm * sigma_mm):
            accepted += 1
            innov_sq += innov * innov
            accepted_names.append(name)
    state.last_measurements = dict(measurements)
    if accepted > 0:
        rms = math.sqrt(innov_sq / float(accepted))
        _refresh_confidence(state, cfg, accepted=accepted, total=max(total, 1), innovation_rms_mm=rms)
    elif total > 0:
        _refresh_confidence(state, cfg, accepted=0, total=max(total, 1))
    _update_ray_overlays(state, cfg, measurements, state.estimate)
    return accepted_names


def _pack_theta_wall_alignment_ctx(state: MCLState,
                                   accepted: List[str],
                                   candidate: dict) -> dict:
    """Build the mutable theta-wall-alignment context from a chosen candidate."""
    ctx = dict(candidate)
    ctx["accepted_sensors"] = list(accepted)
    ctx["last_pose"] = tuple(state.ekf_pose or state.estimate or (0.0, 0.0, 0.0))
    ctx["last_distance_mm"] = float(candidate.get("measured_mm", 0.0))
    ctx["filtered_rate_in_s"] = 0.0
    ctx["last_valid"] = False
    return ctx


def apply_immediate_correction_auto(state: MCLState, cfg: dict, measurements: Optional[dict]) -> List[str]:
    """Run a one-shot translation cleanup using all currently enabled distance sensors."""
    return _distance_update_from_measurements(state, cfg, measurements, sensor_filter=None)


def _resolve_distance_sensor_names(cfg: dict, sensor_tokens: object, max_sensors: int = 2) -> List[str]:
    """Resolve up to `max_sensors` requested slots/names to configured distance-sensor names."""
    tokens: List[str] = []
    if isinstance(sensor_tokens, (list, tuple, set)):
        raw_parts = list(sensor_tokens)
    else:
        raw_parts = [sensor_tokens]
    for part in raw_parts:
        for raw_token in re.split(r"[\s,;/|+]+", str(part or "").strip()):
            token = _normalize_distance_sensor_token(raw_token)
            if token:
                tokens.append(token)
    if not tokens:
        return []
    sensors = get_distance_sensors(cfg, include_disabled=True)
    slot_lookup = {"FRONT": 0, "RIGHT": 1, "BACK": 2, "LEFT": 3}
    names: List[str] = []
    seen = set()
    for token in tokens:
        resolved = None
        slot_idx = slot_lookup.get(token)
        if slot_idx is not None and 0 <= slot_idx < len(sensors):
            candidate = str(sensors[slot_idx].get("name", "") or "").strip()
            if candidate:
                resolved = candidate
        if resolved is None:
            for sensor in sensors:
                candidate = str(sensor.get("name", "") or "").strip()
                if candidate and candidate.upper() == token:
                    resolved = candidate
                    break
        if resolved and resolved not in seen:
            names.append(resolved)
            seen.add(resolved)
            if len(names) >= max(1, int(max_sensors)):
                break
    return names


def resolve_distance_sensor_name(cfg: dict, sensor_token: object) -> Optional[str]:
    """Resolve a FRONT/RIGHT/BACK/LEFT token to the current configured sensor name."""
    names = _resolve_distance_sensor_names(cfg, sensor_token, max_sensors=1)
    return names[0] if names else None


def resolve_distance_sensor_names(cfg: dict, sensor_tokens: object, max_sensors: int = 2) -> List[str]:
    """Resolve up to two requested slots/names to configured distance-sensor names."""
    return _resolve_distance_sensor_names(cfg, sensor_tokens, max_sensors=max_sensors)


def apply_immediate_correction_sensor(state: MCLState,
                                      cfg: dict,
                                      measurements: Optional[dict],
                                      sensor_token: object) -> List[str]:
    """Run a one-shot translation cleanup using one named distance sensor."""
    sensor_name = resolve_distance_sensor_name(cfg, sensor_token)
    if not sensor_name:
        return []
    return _distance_update_from_measurements(state, cfg, measurements, sensor_filter={sensor_name})


def apply_immediate_correction_sensors(state: MCLState,
                                       cfg: dict,
                                       measurements: Optional[dict],
                                       sensor_tokens: object) -> List[str]:
    """Run a one-shot translation cleanup using up to two named distance sensors."""
    sensor_names = resolve_distance_sensor_names(cfg, sensor_tokens, max_sensors=2)
    if not sensor_names:
        return []
    return _distance_update_from_measurements(state, cfg, measurements, sensor_filter=set(sensor_names))


def _choose_wall_trim_candidate(state: MCLState,
                                cfg: dict,
                                measurements: Optional[dict],
                                preferred_sensor: Optional[str] = None,
                                reference_heading: Optional[float] = None) -> Optional[dict]:
    """Pick the best currently valid sensor/wall pair for theta wall alignment start."""
    if measurements is None or state.estimate is None:
        return None
    mcl = cfg.get("mcl", {}) if isinstance(cfg, dict) else {}
    sensors_cfg = mcl.get("sensors", {}) if isinstance(mcl, dict) else {}
    dist_cfg = sensors_cfg.get("distance", {}) if isinstance(sensors_cfg, dict) else {}
    dist_meas = measurements.get("distance", {})
    if not isinstance(dist_meas, dict) or not dist_meas:
        return None
    sensors = get_distance_sensors(cfg)
    max_range_mm = float(dist_cfg.get("max_range_mm", 2000.0))
    min_range_mm = float(dist_cfg.get("min_range_mm", 20.0))
    min_beam_alignment = math.cos(math.radians(_THETA_ALIGN_MAX_BEAM_TO_NORMAL_DEG))
    current_heading = float(reference_heading if reference_heading is not None else state.estimate[2])
    best = None
    for sensor in sensors:
        name = str(sensor.get("name", "") or "")
        if preferred_sensor and name != preferred_sensor:
            continue
        meas_mm = _sensor_measurement_mm(dist_meas.get(name))
        if meas_mm is None or not math.isfinite(meas_mm):
            continue
        meas_mm -= float(sensor.get("bias_mm", 0.0))
        s_min = float(sensor.get("min_range_mm", min_range_mm))
        s_max = float(sensor.get("max_range_mm", max_range_mm))
        if s_max <= 0.0:
            s_max = max_range_mm
        if s_min <= 0.0:
            s_min = min_range_mm
        if meas_mm < s_min or meas_mm > s_max:
            continue
        hit = _raycast_expected_measurement(state, cfg, state.estimate, sensor)
        if hit is None or not bool(hit.get("definite", False)):
            continue
        origin = hit["origin"]
        heading = float(hit["heading_deg"])
        pred_mm = float(hit["distance_mm"])
        innovation_mm = meas_mm - pred_mm
        gate_mm = float(sensor.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)))
        if gate_mm > 0.0 and abs(innovation_mm) > gate_mm:
            continue
        x0, y0, x1, y1 = hit["segment"]
        tangent_x = x1 - x0
        tangent_y = y1 - y0
        tangent_norm = math.hypot(tangent_x, tangent_y)
        if tangent_norm <= 1e-6:
            continue
        tangent_x /= tangent_norm
        tangent_y /= tangent_norm
        normal_x = tangent_y
        normal_y = -tangent_x
        hit_x = origin[0] + _heading_unit(heading)[0] * hit["distance_px"]
        hit_y = origin[1] + _heading_unit(heading)[1] * hit["distance_px"]
        to_sensor_x = origin[0] - hit_x
        to_sensor_y = origin[1] - hit_y
        if to_sensor_x * normal_x + to_sensor_y * normal_y < 0.0:
            normal_x = -normal_x
            normal_y = -normal_y
        beam_x, beam_y = _heading_unit(heading)
        beam_alignment = abs(beam_x * normal_x + beam_y * normal_y)
        if beam_alignment < min_beam_alignment:
            continue
        wall_heading = _heading_from_unit(tangent_x, tangent_y)
        cand_a = wall_heading
        cand_b = (wall_heading + 180.0) % 360.0
        target_heading = cand_a if abs(_angle_diff_deg(cand_a, current_heading)) <= abs(_angle_diff_deg(cand_b, current_heading)) else cand_b
        start_error_deg = _angle_diff_deg(current_heading, target_heading)
        if abs(start_error_deg) > _THETA_ALIGN_MAX_START_ERROR_DEG:
            continue
        positive_error_right_x, positive_error_right_y = _heading_unit((target_heading + 90.0) % 360.0)
        theta_error_sign = positive_error_right_x * normal_x + positive_error_right_y * normal_y
        if abs(theta_error_sign) < 0.5:
            continue
        score = abs(innovation_mm) + 0.5 * abs(start_error_deg)
        cand = {
            "sensor_name": name,
            "target_heading_deg": target_heading,
            "wall_heading_deg": wall_heading,
            "sensor_heading_deg": heading,
            "wall_tangent_x": tangent_x,
            "wall_tangent_y": tangent_y,
            "wall_normal_x": normal_x,
            "wall_normal_y": normal_y,
            "theta_error_sign": 1.0 if theta_error_sign > 0.0 else -1.0,
            "feature": str(hit.get("feature", "unknown")),
            "object_id": hit.get("object_id"),
            "measured_mm": meas_mm,
            "innovation_mm": innovation_mm,
            "score": score,
        }
        if best is None or score < best["score"]:
            best = cand
    return best


def start_theta_wall_alignment_auto(state: MCLState, cfg: dict, measurements: Optional[dict]) -> Optional[dict]:
    """Start a wall-trim segment using the best currently valid enabled sensor."""
    accepted = apply_immediate_correction_auto(state, cfg, measurements)
    candidate = _choose_wall_trim_candidate(state, cfg, measurements)
    if candidate is None:
        return None
    return _pack_theta_wall_alignment_ctx(state, accepted, candidate)


def start_rough_wall_traverse_auto(state: MCLState, cfg: dict, measurements: Optional[dict]) -> Optional[dict]:
    """Start the rough wall-traverse mode using enabled sensors plus a locked wall target."""
    selected_sensor_names = [str(sensor.get("name", "") or "").strip() for sensor in get_distance_sensors(cfg)]
    selected_sensor_names = [name for name in selected_sensor_names if name]
    if not selected_sensor_names:
        return None
    accepted = apply_immediate_correction_auto(state, cfg, measurements)
    candidate = _choose_wall_trim_candidate(state, cfg, measurements)
    if candidate is None:
        return None
    ctx = _pack_theta_wall_alignment_ctx(state, accepted, candidate)
    ctx["selected_sensor_names"] = list(selected_sensor_names)
    return ctx


def update_theta_wall_alignment(state: MCLState,
                                cfg: dict,
                                measurements: Optional[dict],
                                alignment_ctx: Optional[dict],
                                motion_input: Optional[dict] = None) -> bool:
    """Apply a bounded rate-based heading trim toward the active wall-parallel heading."""
    if not alignment_ctx or state.ekf_pose is None or state.ekf_P is None:
        return False
    if measurements is None:
        return False
    preferred = str(alignment_ctx.get("sensor_name", "") or "").strip()
    if not preferred:
        return False
    dist_meas = measurements.get("distance", {})
    if not isinstance(dist_meas, dict):
        return False
    sensor = None
    for entry in get_distance_sensors(cfg):
        if str(entry.get("name", "") or "") == preferred:
            sensor = entry
            break
    if sensor is None:
        alignment_ctx["last_valid"] = False
        return False
    meas_mm = _sensor_measurement_mm(dist_meas.get(preferred))
    if meas_mm is None or not math.isfinite(meas_mm):
        alignment_ctx["last_valid"] = False
        return False
    try:
        meas_mm -= float(sensor.get("bias_mm", 0.0))
    except Exception:
        pass
    dist_cfg = _distance_cfg(cfg)
    min_range_mm = float(sensor.get("min_range_mm", dist_cfg.get("min_range_mm", 20.0)))
    max_range_mm = float(sensor.get("max_range_mm", dist_cfg.get("max_range_mm", 2000.0)))
    if meas_mm < min_range_mm or meas_mm > max_range_mm:
        alignment_ctx["last_valid"] = False
        return False
    pose = state.ekf_pose
    hit = _raycast_expected_measurement(state, cfg, pose, sensor)
    if hit is None or not bool(hit.get("definite", False)):
        alignment_ctx["last_valid"] = False
        return False
    if str(hit.get("feature", "unknown")) != str(alignment_ctx.get("feature", "unknown")):
        alignment_ctx["last_valid"] = False
        return False
    if alignment_ctx.get("object_id") is not None and hit.get("object_id") != alignment_ctx.get("object_id"):
        alignment_ctx["last_valid"] = False
        return False
    x0, y0, x1, y1 = hit["segment"]
    tangent_x = x1 - x0
    tangent_y = y1 - y0
    tangent_norm = math.hypot(tangent_x, tangent_y)
    if tangent_norm <= 1e-6:
        alignment_ctx["last_valid"] = False
        return False
    tangent_x /= tangent_norm
    tangent_y /= tangent_norm
    tangent_alignment = abs(
        tangent_x * float(alignment_ctx.get("wall_tangent_x", 0.0)) +
        tangent_y * float(alignment_ctx.get("wall_tangent_y", 0.0))
    )
    if tangent_alignment < math.cos(math.radians(_THETA_ALIGN_TANGENT_TOLERANCE_DEG)):
        alignment_ctx["last_valid"] = False
        return False
    gate_mm = float(sensor.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)))
    expected_mm = float(hit["distance_mm"])
    residual_mm = meas_mm - expected_mm
    if gate_mm > 0.0 and abs(residual_mm) > gate_mm:
        alignment_ctx["last_valid"] = False
        return False
    last_pose = alignment_ctx.get("last_pose")
    if not isinstance(last_pose, (list, tuple)) or len(last_pose) < 3:
        last_pose = pose
    try:
        last_pose_t = (float(last_pose[0]), float(last_pose[1]), float(last_pose[2]))
    except Exception:
        last_pose_t = pose
    try:
        dt_s = float(_atticus_cfg(cfg).get("sensor_ms", _THETA_ALIGN_MIN_DT_S * 1000.0)) / 1000.0
    except Exception:
        dt_s = _THETA_ALIGN_MIN_DT_S
    dt_s = max(_THETA_ALIGN_MIN_DT_S, dt_s)
    if not bool(alignment_ctx.get("last_valid", False)):
        alignment_ctx["last_pose"] = pose
        alignment_ctx["last_distance_mm"] = float(meas_mm)
        alignment_ctx["filtered_rate_in_s"] = 0.0
        alignment_ctx["last_valid"] = True
        return False
    dx_px = pose[0] - last_pose_t[0]
    dy_px = pose[1] - last_pose_t[1]
    avg_heading = _wrap_deg(last_pose_t[2] + 0.5 * _angle_diff_deg(pose[2], last_pose_t[2]))
    forward_speed_in_s = (_body_forward_from_world_delta(dx_px, dy_px, avg_heading) / PPI) / dt_s
    current_error_deg = _angle_diff_deg(pose[2], float(alignment_ctx.get("target_heading_deg", pose[2])))
    if (abs(forward_speed_in_s) < _THETA_ALIGN_MIN_FORWARD_SPEED_IN_S or
            abs(current_error_deg) > _THETA_ALIGN_MAX_ERROR_DEG):
        alignment_ctx["last_pose"] = pose
        alignment_ctx["last_distance_mm"] = float(meas_mm)
        alignment_ctx["last_valid"] = True
        return False
    try:
        last_distance_mm = float(alignment_ctx.get("last_distance_mm", meas_mm))
    except Exception:
        last_distance_mm = float(meas_mm)
    distance_rate_in_s = ((float(meas_mm) - last_distance_mm) / MM_PER_IN) / dt_s
    filtered_prev = float(alignment_ctx.get("filtered_rate_in_s", 0.0) or 0.0)
    filtered_rate_in_s = filtered_prev * (1.0 - _THETA_ALIGN_RATE_ALPHA) + distance_rate_in_s * _THETA_ALIGN_RATE_ALPHA
    ratio = filtered_rate_in_s / max(1e-6, forward_speed_in_s) if forward_speed_in_s >= 0.0 else filtered_rate_in_s / min(-1e-6, forward_speed_in_s)
    signed_ratio = _clamp(ratio * float(alignment_ctx.get("theta_error_sign", 1.0)), -1.0, 1.0)
    inferred_error_deg = math.degrees(math.asin(signed_ratio))
    if abs(inferred_error_deg - current_error_deg) > _THETA_ALIGN_RATE_ERROR_TOLERANCE_DEG:
        alignment_ctx["last_pose"] = pose
        alignment_ctx["last_distance_mm"] = float(meas_mm)
        alignment_ctx["filtered_rate_in_s"] = filtered_rate_in_s
        alignment_ctx["last_valid"] = True
        return False
    correction_deg = _clamp_magnitude(-inferred_error_deg * _THETA_ALIGN_GAIN, _THETA_ALIGN_MAX_STEP_DEG)
    correction_deg = _clamp_magnitude(correction_deg, abs(current_error_deg))
    if abs(correction_deg) < 1e-4 or correction_deg * current_error_deg > 0.0:
        alignment_ctx["last_pose"] = pose
        alignment_ctx["last_distance_mm"] = float(meas_mm)
        alignment_ctx["filtered_rate_in_s"] = filtered_rate_in_s
        alignment_ctx["last_valid"] = True
        return False
    corrected_pose = (pose[0], pose[1], _wrap_deg(pose[2] + correction_deg))
    state.ekf_pose = corrected_pose
    state.estimate = corrected_pose
    state.ekf_P[0][2] = 0.0
    state.ekf_P[1][2] = 0.0
    state.ekf_P[2][0] = 0.0
    state.ekf_P[2][1] = 0.0
    state.ekf_P[2][2] = 1.0
    alignment_ctx["last_pose"] = corrected_pose
    alignment_ctx["last_distance_mm"] = float(meas_mm)
    alignment_ctx["filtered_rate_in_s"] = filtered_rate_in_s
    alignment_ctx["last_valid"] = True
    applied = True
    if applied:
        state.cov_xy = (state.ekf_P[0][0], state.ekf_P[1][1], state.ekf_P[0][1])
        _refresh_confidence(state, cfg)
    return applied


def update_rough_wall_traverse(state: MCLState,
                               cfg: dict,
                               measurements: Optional[dict],
                               rough_ctx: Optional[dict],
                               motion_input: Optional[dict] = None) -> bool:
    """Update the active rough wall-traverse heading trim from the locked wall sensor."""
    if not rough_ctx:
        return False
    return update_theta_wall_alignment(state, cfg, measurements, rough_ctx, motion_input=motion_input)
