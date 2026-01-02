# mod/sim.py
import math
import bisect

# Support running as script (no package) by falling back to absolute imports
if __package__ is None or __package__ == "":
    import os, sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from config import PPI  # type: ignore
    from geom import heading_from_points, approach_unit, convert_heading_input  # type: ignore
    from util import get_node_offset_in  # type: ignore
    from pathing import generate_bezier_path, calculate_path_heading, resample_path_uniform  # type: ignore
else:
    from .config import PPI
    from .geom import heading_from_points, approach_unit, convert_heading_input
    from .util import get_node_offset_in
    from .pathing import generate_bezier_path, calculate_path_heading, resample_path_uniform

MIN_TURN_TIME_S = 0.14
MIN_MOVE_TIME_S = 1.0 / 60.0
TURN_ACCEL_MIN_DPS2 = 60.0
TURN_ACCEL_MAX_DPS2 = 3000.0
PROFILE_SPEED_SCALE = {
    "precise": 0.75,
    "normal": 1.0,
    "fast": 1.15,
    "slam": 1.3
}
_PHYS_CACHE_KEY = "_phys_cache"

_PHYS_CONST_DEFAULTS = {
    "load_factor": 0.9,
    "accel_mu_scale": 0.95,
    "accel_min": 60.0,
    "accel_max": 450.0,
    "t_to_v_base": 0.2,
    "t_to_v_min": 0.12,
    "t_to_v_max": 0.60,
    "vmax_min": 10.0,
    "vmax_max": 220.0,
    "turn_rate_scale": 0.8,
    "turn_rate_min": 120.0,
    "turn_rate_max": 900.0,
    "turn_accel_min": 0.12,
    "turn_accel_max": 0.30,
    "omni_scale": 0.9
}

def _phys_const(cfg, key: str, default: float) -> float:
    consts = cfg.get("physics_constants", {})
    val = consts.get(key, default)
    if isinstance(val, dict):
        val = val.get("value", default)
    try:
        return float(val)
    except Exception:
        return float(default)


def _physics_cache(cfg: dict) -> dict:
    """Compute and cache derived physics values on the cfg dict."""
    cache = cfg.get(_PHYS_CACHE_KEY)
    if cache:
        return cache

    rp = cfg.get("robot_physics", {})
    bd = cfg.get("bot_dimensions", {})

    rpm = float(rp.get("rpm", 200.0))
    d_in = float(rp.get("diameter", 4.0))
    v_straight = float(rp.get("volts_straight", 12.0))
    v_turn = float(rp.get("volts_turn", 12.0))
    wlb = float(rp.get("weight", 20.0))
    gear_ratio = float(cfg.get("gear_ratio", 1.0) or 1.0)
    mu = float(rp.get("mu", 0.9))
    omni = float(rp.get("all_omni", 0))
    max_cmd = float(rp.get("max_cmd", 127.0))

    load_factor = _phys_const(cfg, "load_factor", _PHYS_CONST_DEFAULTS["load_factor"])
    accel_mu_scale = _phys_const(cfg, "accel_mu_scale", _PHYS_CONST_DEFAULTS["accel_mu_scale"])
    accel_min = _phys_const(cfg, "accel_min", _PHYS_CONST_DEFAULTS["accel_min"])
    accel_max = _phys_const(cfg, "accel_max", _PHYS_CONST_DEFAULTS["accel_max"])
    t_to_v_base = _phys_const(cfg, "t_to_v_base", _PHYS_CONST_DEFAULTS["t_to_v_base"])
    t_to_v_min = _phys_const(cfg, "t_to_v_min", _PHYS_CONST_DEFAULTS["t_to_v_min"])
    t_to_v_max = _phys_const(cfg, "t_to_v_max", _PHYS_CONST_DEFAULTS["t_to_v_max"])
    vmax_min = _phys_const(cfg, "vmax_min", _PHYS_CONST_DEFAULTS["vmax_min"])
    vmax_max = _phys_const(cfg, "vmax_max", _PHYS_CONST_DEFAULTS["vmax_max"])
    turn_rate_scale = _phys_const(cfg, "turn_rate_scale", _PHYS_CONST_DEFAULTS["turn_rate_scale"])
    turn_rate_min = _phys_const(cfg, "turn_rate_min", _PHYS_CONST_DEFAULTS["turn_rate_min"])
    turn_rate_max = _phys_const(cfg, "turn_rate_max", _PHYS_CONST_DEFAULTS["turn_rate_max"])
    omni_scale = _phys_const(cfg, "omni_scale", _PHYS_CONST_DEFAULTS["omni_scale"])

    cal = cfg.get("codegen", {}).get("calibration", {})
    enabled = cal.get("enabled", 0) if isinstance(cal, dict) else 0
    if isinstance(enabled, dict):
        enabled = enabled.get("value", 0)
    use_cal_consts = False
    cal_consts = cal.get("constants", {}) if isinstance(cal, dict) else {}
    if enabled and isinstance(cal_consts, dict) and cal_consts:
        use_cal_consts = True
        try:
            if "load_factor" in cal_consts:
                load_factor = float(cal_consts["load_factor"])
            if "accel_mu_scale" in cal_consts:
                accel_mu_scale = float(cal_consts["accel_mu_scale"])
            if "t_to_v_base" in cal_consts:
                t_to_v_base = float(cal_consts["t_to_v_base"])
            if "turn_rate_scale" in cal_consts:
                turn_rate_scale = float(cal_consts["turn_rate_scale"])
            if "omni_scale" in cal_consts:
                omni_scale = float(cal_consts["omni_scale"])
        except Exception:
            use_cal_consts = False

    free = rpm * math.pi * max(0.01, d_in) / 60.0
    load = load_factor * (v_straight / 12.0)
    mc = max(0.0, min(127.0, max_cmd))
    scale = mc / 127.0
    vmax = max(vmax_min, min(vmax_max, free * load * scale))

    a_t = max(accel_min, min(accel_mu_scale * mu * 386.09, accel_max))
    mass_scale = max(0.7, min(1.6, math.sqrt(max(1e-6, wlb) / 15.0)))
    t_to_v = t_to_v_base * mass_scale
    a_m = vmax / max(t_to_v_min, min(t_to_v_max, t_to_v))
    accel_raw = max(accel_min, min(a_t, a_m))
    torque_scale = max(0.5, min(2.0, gear_ratio))
    if a_m <= a_t:
        accel_raw *= torque_scale
    accel = accel_raw
    if omni:
        accel *= omni_scale

    track = max(6.0, float(bd.get("dt_width", bd.get("width", 12.0))))
    base_omega_rad = (2.0 * vmax) / max(1e-6, track)
    base_deg = base_omega_rad * (180.0 / math.pi)
    rate = base_deg * (v_turn / 12.0) * turn_rate_scale
    turn_rate_val = max(turn_rate_min, min(turn_rate_max, rate))

    alpha_rad = (2.0 * accel) / max(1e-6, track)
    alpha_deg = alpha_rad * (180.0 / math.pi)
    turn_accel = alpha_deg * (v_turn / 12.0) * turn_rate_scale
    turn_accel = max(TURN_ACCEL_MIN_DPS2, min(TURN_ACCEL_MAX_DPS2, turn_accel))

    cache = {
        "vmax": vmax,
        "accel": accel,
        "turn_rate": turn_rate_val,
        "turn_accel": turn_accel,
        "track_width_in": track,
        "half_track_px": 0.5 * track * PPI,
        "max_cmd": mc,
    }
    if enabled and not use_cal_consts:
        dyn = cal.get("dynamics", {})
        if isinstance(dyn, dict):
            for key, cache_key in (
                ("vmax_ips", "vmax"),
                ("accel_ips2", "accel"),
                ("turn_rate_dps", "turn_rate"),
                ("turn_accel_dps2", "turn_accel"),
            ):
                if key in dyn:
                    try:
                        cache[cache_key] = float(dyn[key])
                    except Exception:
                        pass
    cfg[_PHYS_CACHE_KEY] = cache
    return cache

def snap_to_grid(pos, grid_size_px, snap_enabled=True):
    """Snap position to grid."""
    if not snap_enabled:
        return pos
    step = grid_size_px / 4.0
    return (int(round(pos[0] / step) * step), int(round(pos[1] / step) * step))

def _heading_vec_px(heading_deg: float):
    """Unit vector in screen space for a given internal heading."""
    th = math.radians(heading_deg)
    return (math.cos(th), -math.sin(th))

def _sample_quad_bezier(p0, p1, p2, num=40):
    """Sample a quadratic Bezier curve for path construction."""
    if num <= 1:
        return [p0, p2]
    pts = []
    for i in range(num + 1):
        t = i / num
        it = 1.0 - t
        a = it * it
        b = 2.0 * it * t
        c = t * t
        x = a * p0[0] + b * p1[0] + c * p2[0]
        y = a * p0[1] + b * p1[1] + c * p2[1]
        pts.append((x, y))
    return pts

def _clamp_cmd(cmd_val):
    """Clamp a command-style speed to [0, 127]."""
    try:
        return max(0.0, min(127.0, float(cmd_val)))
    except Exception:
        return 0.0

def _cmd_to_ips(cmd_val, cfg):
    """Convert a command speed (0-127) into ips based on current drivetrain limits."""
    vmax_cmd = _physics_cache(cfg)["vmax"]
    return vmax_cmd * (_clamp_cmd(cmd_val) / 127.0)

def _profile_speed_scale(name: str) -> float:
    """Map profile name to a speed multiplier."""
    return PROFILE_SPEED_SCALE.get(str(name).lower(), 1.0) if name else 1.0

def _has_custom_waits_at_node(node: dict) -> bool:
    """Check if node has custom wait actions."""
    acts = node.get("actions_out", node.get("actions", []))
    return any(a.get("type") == "wait" and float(a.get("s", 0) or 0) > 0 for a in acts)

def vmax_straight(cfg):
    """Calculate maximum straight-line velocity."""
    return _physics_cache(cfg)["vmax"]

def accel_straight(cfg):
    """Calculate straight-line acceleration."""
    return _physics_cache(cfg)["accel"]

def _path_length_px(path_points):
    """Calculate total path length in pixels."""
    if not path_points or len(path_points) < 2:
        return 0.0
    total_px = 0.0
    for i in range(len(path_points) - 1):
        dx = path_points[i+1][0] - path_points[i][0]
        dy = path_points[i+1][1] - path_points[i][1]
        total_px += math.hypot(dx, dy)
    return total_px

def _curvature_of_triplet(p_prev, p, p_next):
    """Compute curvature (approx) from three points."""
    ax, ay = p_prev
    bx, by = p
    cx, cy = p_next
    abx, aby = bx - ax, by - ay
    bcx, bcy = cx - bx, cy - by
    denom = (abx * abx + aby * aby) ** 0.5 * (bcx * bcx + bcy * bcy) ** 0.5
    if denom <= 1e-9:
        return 0.0
    # Signed area via cross product
    cross = abx * bcy - aby * bcx
    # Use simple curvature magnitude
    return cross / max(1e-9, denom * max(1e-3, math.hypot(cx - ax, cy - ay)))


def path_time_with_curvature(path_points, cfg, speed_mult=1.0, min_speed_override=None, max_speed_override=None, v0=None, v1=None):
    """Compute length (in) and time considering curvature speed limits."""
    if not path_points or len(path_points) < 2:
        return 0.0, 0.0, path_points, [], []
    pcfg = cfg.get("path_config", {})
    pts_per_in = float(cfg["robot_physics"].get("point_density_per_in", 4.0))
    spacing_px = PPI / max(1e-6, pts_per_in)
    resampled = resample_path_uniform(path_points, spacing_px)
    total_px = _path_length_px(resampled)
    if total_px <= 1e-9:
        return 0.0, 0.0, resampled, [], []

    # Interpret min/max as command units (0-127) and convert to ips for physics
    max_cfg_cmd = pcfg.get("max_speed_cmd", pcfg.get("max_speed_ips", 127.0))
    max_base_cmd = _clamp_cmd(max_speed_override if max_speed_override is not None else max_cfg_cmd)
    max_base = _cmd_to_ips(max_base_cmd, cfg) * speed_mult
    if max_base <= 1e-6:
        max_base = _cmd_to_ips(127.0, cfg) * speed_mult
    min_cfg_cmd = pcfg.get("min_speed_cmd", pcfg.get("min_speed_ips", 0.0))
    min_cmd = _clamp_cmd(min_speed_override if min_speed_override is not None else min_cfg_cmd)
    min_speed = min(_cmd_to_ips(min_cmd, cfg), max_base) * speed_mult
    curv_gain = max(0.0, float(cfg["robot_physics"].get("curvature_gain", pcfg.get("curvature_gain", 0.05))))

    curvatures_mid = []
    for i in range(1, len(resampled) - 1):
        p0, p1, p2 = resampled[i - 1], resampled[i], resampled[i + 1]
        curvature = abs(_curvature_of_triplet(p0, p1, p2))
        curvatures_mid.append(curvature)

    time_s = 0.0
    point_curvatures = [0.0] * len(resampled)
    point_speeds = [max_base] * len(resampled)
    for seg_idx in range(len(resampled) - 1):
        pA, pB = resampled[seg_idx], resampled[seg_idx + 1]
        ds_px = math.hypot(pB[0] - pA[0], pB[1] - pA[1])
        if ds_px <= 1e-9:
            continue
        if curvatures_mid:
            if seg_idx == 0:
                curv_seg = curvatures_mid[0]
            elif seg_idx == len(resampled) - 2:
                curv_seg = curvatures_mid[-1]
            else:
                curv_seg = 0.5 * (curvatures_mid[seg_idx - 1] + curvatures_mid[seg_idx])
        else:
            curv_seg = 0.0
        point_curvatures[seg_idx] = curv_seg
        point_curvatures[seg_idx + 1] = curv_seg
        # Curvature impact; always enforce turn-rate limit
        if curv_gain <= 0.0:
            v_cap = max_base
        else:
            curv_in = abs(curv_seg) * PPI  # convert to curvature in 1/in
            curv_scale = curv_gain * curv_in * 120.0
            v_cap = max(min_speed, max_base / (1.0 + curv_scale))
        if curv_seg > 1e-6:
            omega_max = turn_rate(cfg) * (math.pi / 180.0)  # rad/s
            v_cap = min(v_cap, omega_max / curv_seg)
        point_speeds[seg_idx] = v_cap
        point_speeds[seg_idx + 1] = v_cap
        time_s += (ds_px / PPI) / max(1e-6, v_cap)

    base_time = time_s
    profile_time = _move_total_time(total_px / PPI, cfg, v_override=max_base, v0=v0 or 0.0, v1=v1 or 0.0)
    final_time = max(base_time, profile_time)

    # soften endpoints to reduce discontinuities
    if len(point_speeds) >= 2:
        point_speeds[0] = min(point_speeds[0], point_speeds[1])
        point_speeds[-1] = min(point_speeds[-1], point_speeds[-2])

    # Preserve a copy of the unscaled profile for export purposes
    point_speeds_raw = list(point_speeds)

    # If we padded time for acceleration, scale speeds so animation/export match final_time
    if final_time > 1e-9 and base_time > 1e-9 and final_time > base_time:
        scale = base_time / final_time
        point_speeds = [v * scale for v in point_speeds]

    return total_px / PPI, final_time, resampled, point_curvatures, point_speeds, point_speeds_raw
def _move_total_time(L_in, cfg, v_override=None, v0=0.0, v1=0.0):
    """Calculate total time for move distance, honoring optional speed override and end speeds."""
    if L_in <= 0.0: 
        return 0.0
    v_nom = vmax_straight(cfg)
    v_cap = min(v_nom, float(v_override)) if v_override is not None else v_nom
    a_nom = accel_straight(cfg)
    a = a_nom * (v_cap / max(1e-6, v_nom))
    try:
        v0 = float(v0)
    except Exception:
        v0 = 0.0
    try:
        v1 = float(v1)
    except Exception:
        v1 = 0.0
    v0 = max(0.0, min(v_cap, v0))
    v1 = max(0.0, min(v_cap, v1))

    d_acc = max(0.0, (v_cap * v_cap - v0 * v0) / max(1e-6, 2.0 * a))
    d_dec = max(0.0, (v_cap * v_cap - v1 * v1) / max(1e-6, 2.0 * a))
    if d_acc + d_dec <= L_in:
        t_acc = (v_cap - v0) / max(1e-6, a)
        t_dec = (v_cap - v1) / max(1e-6, a)
        t_flat = (L_in - d_acc - d_dec) / max(1e-6, v_cap)
        return max(MIN_MOVE_TIME_S, t_acc + t_flat + t_dec)

    v_peak_sq = max(0.0, (2.0 * a * L_in + v0 * v0 + v1 * v1) / 2.0)
    v_peak = math.sqrt(v_peak_sq)
    v_peak = max(v_peak, max(v0, v1))
    v_peak = min(v_peak, v_cap)
    t_acc = max(0.0, (v_peak - v0) / max(1e-6, a))
    t_dec = max(0.0, (v_peak - v1) / max(1e-6, a))
    return max(MIN_MOVE_TIME_S, t_acc + t_dec)


def _build_path_meta(path_points, path_speeds=None):
    """Precompute path segment lengths and cumulative distances."""
    if not path_points or len(path_points) < 2:
        return None
    seg_lengths_px = []
    cum_lengths_px = [0.0]
    total_px = 0.0
    for i in range(len(path_points) - 1):
        dx = path_points[i + 1][0] - path_points[i][0]
        dy = path_points[i + 1][1] - path_points[i][1]
        seg_len = math.hypot(dx, dy)
        seg_lengths_px.append(seg_len)
        total_px += seg_len
        cum_lengths_px.append(total_px)

    meta = {
        "seg_lengths_px": seg_lengths_px,
        "cum_lengths_px": cum_lengths_px,
        "total_px": total_px,
    }

    if path_speeds and len(path_speeds) == len(path_points):
        seg_times = []
        total_T = 0.0
        for i, ds_px in enumerate(seg_lengths_px):
            v0 = max(1e-3, path_speeds[i])
            v1 = max(1e-3, path_speeds[i + 1])
            v_avg = 0.5 * (v0 + v1)
            dt = (ds_px / PPI) / v_avg
            seg_times.append(dt)
            total_T += dt
        meta["seg_times"] = seg_times
        meta["seg_time_total"] = total_T

    return meta

def sample_move_profile(t, L_in, cfg, v_override=None, v0=0.0, v1=0.0):
    """Sample distance at time t along move profile with optional speed override and end speeds."""
    if L_in <= 1e-9 or t <= 0.0: 
        return 0.0
    v_nom = vmax_straight(cfg)
    v_cap = min(v_nom, float(v_override)) if v_override is not None else v_nom
    a_nom = accel_straight(cfg)
    a = a_nom * (v_cap / max(1e-6, v_nom))
    try:
        v0 = float(v0)
    except Exception:
        v0 = 0.0
    try:
        v1 = float(v1)
    except Exception:
        v1 = 0.0
    v0 = max(0.0, min(v_cap, v0))
    v1 = max(0.0, min(v_cap, v1))

    d_acc = max(0.0, (v_cap * v_cap - v0 * v0) / max(1e-6, 2.0 * a))
    d_dec = max(0.0, (v_cap * v_cap - v1 * v1) / max(1e-6, 2.0 * a))
    if d_acc + d_dec <= L_in:
        t_acc = max(0.0, (v_cap - v0) / max(1e-6, a))
        t_dec = max(0.0, (v_cap - v1) / max(1e-6, a))
        t_flat = max(0.0, (L_in - d_acc - d_dec) / max(1e-6, v_cap))
        if t <= t_acc:
            s = v0 * t + 0.5 * a * t * t
        elif t <= t_acc + t_flat:
            s = d_acc + v_cap * (t - t_acc)
        else:
            t2 = t - (t_acc + t_flat)
            s = d_acc + v_cap * t_flat + (v_cap * t2 - 0.5 * a * t2 * t2)
        return max(0.0, min(L_in, s))

    v_peak_sq = max(0.0, (2.0 * a * L_in + v0 * v0 + v1 * v1) / 2.0)
    v_peak = math.sqrt(v_peak_sq)
    v_peak = max(v_peak, max(v0, v1))
    v_peak = min(v_peak, v_cap)
    t_acc = max(0.0, (v_peak - v0) / max(1e-6, a))
    t_dec = max(0.0, (v_peak - v1) / max(1e-6, a))
    if t <= t_acc:
        s = v0 * t + 0.5 * a * t * t
    else:
        t2 = t - t_acc
        # decel phase only (no cruise)
        s = (v0 + v_peak) * 0.5 * t_acc + (v_peak * t2 - 0.5 * a * t2 * t2)
    return max(0.0, min(L_in, s))

def _track_width_in(cfg):
    """Get robot track width."""
    return _physics_cache(cfg)["track_width_in"]

def _bot_half_track_px(cfg):
    """Half the track width, in pixels (used for swing arcs)."""
    return _physics_cache(cfg)["half_track_px"]

def turn_rate(cfg):
    """Calculate turn rate in degrees per second."""
    return _physics_cache(cfg)["turn_rate"]

def _turn_accel_deg_s2(cfg):
    """Calculate turn acceleration."""
    return _physics_cache(cfg)["turn_accel"]

def _angle_diff_deg(h0, h1):
    """Calculate shortest angle difference."""
    d = ((h1 - h0) % 360.0 + 360.0) % 360.0
    return d - 360.0 if d > 180.0 else d

def _turn_dynamics(cfg, rate_override=None):
    """Return (rate_deg_s, accel_deg_s2) honoring optional override speed."""
    base_rate = turn_rate(cfg)
    rate = float(rate_override) if rate_override is not None else base_rate
    rate = max(30.0, rate)
    alpha_base = _turn_accel_deg_s2(cfg)
    scale = rate / max(1e-6, base_rate)
    alpha = alpha_base * scale
    return rate, alpha

def turn_time(diff_deg, cfg, rate_override=None):
    """Calculate time required for turn."""
    rate, alpha = _turn_dynamics(cfg, rate_override)
    ang = abs(float(diff_deg)) % 360.0
    if ang > 180.0: 
        ang = 360.0 - ang
    if ang < 0.5: 
        return 0.0
    
    t_acc = rate / max(1e-6, alpha)
    theta_acc = 0.5 * alpha * t_acc * t_acc
    if 2.0 * theta_acc >= ang:
        return max(MIN_TURN_TIME_S, 2.0 * math.sqrt(ang / max(1e-6, alpha)))
    
    t_flat = (ang - 2.0 * theta_acc) / max(1e-6, rate)
    return max(MIN_TURN_TIME_S, 2.0 * t_acc + t_flat)

def sample_turn_heading_trap(t, h0, h1, cfg, rate_override=None):
    """Sample heading at time t during trapezoidal turn."""
    d = _angle_diff_deg(h0, h1)
    ang = abs(d)
    if ang < 1e-9 or t <= 0.0:
        return float(h0 if t <= 0.0 else h1) % 360.0
    
    rate, alpha = _turn_dynamics(cfg, rate_override)
    t_acc = rate / max(1e-6, alpha)
    theta_acc = 0.5 * alpha * t_acc * t_acc
    sign = 1.0 if d >= 0.0 else -1.0
    
    if 2.0 * theta_acc >= ang:
        t_half = math.sqrt(ang / max(1e-6, alpha))
        if t <= t_half:
            theta = 0.5 * alpha * t * t
        else:
            t2 = t - t_half
            theta = 0.5 * ang + (alpha * t_half) * t2 - 0.5 * alpha * t2 * t2
    else:
        t_flat = (ang - 2.0 * theta_acc) / max(1e-6, rate)
        if t <= t_acc:
            theta = 0.5 * alpha * t * t
        elif t <= t_acc + t_flat:
            theta = theta_acc + rate * (t - t_acc)
        else:
            t2 = t - (t_acc + t_flat)
            theta = theta_acc + rate * t_flat + (rate * t2 - 0.5 * alpha * t2 * t2)
        theta = min(theta, ang)
    
    return (h0 + sign * theta) % 360.0

def _effective_centers(nodes, cfg, initial_heading):
    """Compute effective node centers accounting for offsets."""
    eff = []
    for i, n in enumerate(nodes):
        p = n["pos"]
        if i == 0:
            eff.append(p)
            continue
        prev = nodes[i-1]["pos"]
        prev_node = nodes[i - 1]
        prev_pd = nodes[i - 1].get("path_to_next", {}) if i - 1 >= 0 else {}
        prev_swing = prev_pd.get("swing_vis")
        start_override = prev_pd.get("start_override")
        if prev_swing and prev_swing.get("end_pos") is not None:
            start_override = prev_swing.get("end_pos")
        prev_anchor = start_override if start_override is not None else prev

        off_in = get_node_offset_in(n, cfg, i)
        ghost_ang = n.get("offset_ghost_angle")
        if off_in != 0.0 and ghost_ang is None and prev_node.get("move_to_pose"):
            pose_h = prev_node.get("pose_heading_deg")
            if pose_h is None:
                pose_h = heading_from_points(prev_anchor, p)
            if prev_node.get("reverse", False):
                pose_h = (pose_h + 180.0) % 360.0
            ghost_ang = (pose_h + 180.0) % 360.0
        if off_in != 0.0 and ghost_ang is None:
            arrival_heading = None
            if prev_swing and prev_swing.get("target_heading") is not None:
                arrival_heading = prev_swing.get("target_heading")
            else:
                has_curve = prev_pd.get("use_path", False) or bool(prev_pd.get("pose_preview_points"))
                if has_curve:
                    pts = list(prev_pd.get("path_points") or prev_pd.get("pose_preview_points") or [])
                    if not pts:
                        cps = list(prev_pd.get("control_points", []))
                        if len(cps) >= 2:
                            cps[0] = prev_anchor
                            cps[-1] = p
                            pts = generate_bezier_path(cps, num_samples=20)
                    if pts:
                        arrival_heading = calculate_path_heading(pts, len(pts) - 1)
            if arrival_heading is None and has_curve:
                try:
                    arrival_heading = calculate_path_heading([prev_anchor, p], 1)
                except Exception:
                    arrival_heading = None
            if arrival_heading is not None:
                ghost_ang = arrival_heading

        if off_in != 0.0 and ghost_ang is not None:
            rad = math.radians(ghost_ang)
            eff.append((p[0] + math.cos(rad) * off_in * PPI,
                        p[1] - math.sin(rad) * off_in * PPI))
        elif off_in == 0.0:
            eff.append(p)
        else:
            ux, uy = approach_unit(prev_anchor, p)
            eff.append((p[0] - ux * off_in * PPI, p[1] - uy * off_in * PPI))
    return eff


def _tangent_into_node(nodes, eff, idx):
    """Return vector of tangent coming into node idx from previous segment."""
    if idx <= 0 or idx >= len(nodes):
        return None
    prev_seg = nodes[idx - 1].get("path_to_next", {})
    if prev_seg.get("use_path") and prev_seg.get("control_points"):
        cps = prev_seg["control_points"]
        if len(cps) >= 2:
            return (cps[-1][0] - cps[-2][0], cps[-1][1] - cps[-2][1])
    p_prev = eff[idx - 1] if eff and idx - 1 < len(eff) else nodes[idx - 1]["pos"]
    p_here = eff[idx] if eff and idx < len(eff) else nodes[idx]["pos"]
    return (p_here[0] - p_prev[0], p_here[1] - p_prev[1])


def _tangent_out_of_node(nodes, eff, idx):
    """Return vector of tangent leaving node idx toward the next segment."""
    if idx < 0 or idx >= len(nodes) - 1:
        return None
    next_seg = nodes[idx].get("path_to_next", {})
    if next_seg.get("use_path") and next_seg.get("control_points"):
        cps = next_seg["control_points"]
        if len(cps) >= 2:
            return (cps[1][0] - cps[0][0], cps[1][1] - cps[0][1])
    p_here = eff[idx] if eff and idx < len(eff) else nodes[idx]["pos"]
    p_next = eff[idx + 1] if eff and idx + 1 < len(eff) else nodes[idx + 1]["pos"]
    return (p_next[0] - p_here[0], p_next[1] - p_here[1])

def _swing_segment(p0, p1, start_heading, swing_dir, cfg, reverse=False, drive_override=None, pose_heading=None, pose_lead=None):
    """
    Build a swing arc segment.
    
    The arc uses a fixed radius (half the track width). The arc ends once the
    robot's heading matches the desired facing toward p1 (respecting reverse).
    """
    def _swing_geom(delta_deg: float, dir_norm_local: str):
        """Compute swing end position and geometry for a given delta."""
        R = _bot_half_track_px(cfg)
        th0 = math.radians(start_heading)
        off_angle = th0 - math.pi * 0.5 if dir_norm_local == "cw" else th0 + math.pi * 0.5
        cx = p0[0] + math.cos(off_angle) * R
        cy = p0[1] - math.sin(off_angle) * R
        r0 = (p0[0] - cx, p0[1] - cy)
        geom_delta_deg = -delta_deg  # screen space rotation
        d_rad = math.radians(geom_delta_deg)
        cosd, sind = math.cos(d_rad), math.sin(d_rad)
        rx = r0[0] * cosd - r0[1] * sind
        ry = r0[0] * sind + r0[1] * cosd
        end_pos = (cx + rx, cy + ry)
        return end_pos, (cx, cy), r0, geom_delta_deg, R, d_rad

    dir_norm = (swing_dir or "auto").lower()
    if dir_norm not in ("cw", "ccw", "auto"):
        dir_norm = "auto"

    # Initial guess based on facing toward the next node (respect reverse travel)
    facing0 = heading_from_points(p0, p1)
    if reverse:
        facing0 = (facing0 + 180.0) % 360.0
    base_diff = ((facing0 - start_heading + 180.0) % 360.0) - 180.0
    delta_deg = base_diff

    # Choose initial direction if auto
    dir_effective = dir_norm
    if dir_effective == "auto":
        dir_effective = "ccw" if delta_deg >= 0 else "cw"

    # Reverse travel flips the geometric rotation (pivot side effect)
    dir_geom = dir_effective
    if reverse and dir_geom in ("cw", "ccw"):
        dir_geom = "ccw" if dir_geom == "cw" else "cw"

    # Enforce direction on initial delta (geometry direction)
    if dir_geom == "cw":
        delta_deg = -abs(delta_deg)
    elif dir_geom == "ccw":
        delta_deg = abs(delta_deg)

    def _desired_heading(end_pos):
        desired = heading_from_points(end_pos, p1)
        lead_in = 0.0
        try:
            if pose_lead is not None:
                lead_in = max(0.0, float(pose_lead))
        except Exception:
            lead_in = 0.0
        if lead_in > 0.0:
            ph = pose_heading
            if ph is None:
                ph = heading_from_points(end_pos, p1)
            end_heading = ph
            if reverse:
                end_heading = (end_heading + 180.0) % 360.0
            disp_heading = convert_heading_input(end_heading, None)
            th = math.radians(disp_heading)
            dist_px = math.hypot(p1[0] - end_pos[0], p1[1] - end_pos[1])
            if dist_px > 1e-6:
                carrot = (
                    p1[0] + math.cos(th) * dist_px * lead_in,
                    p1[1] + math.sin(th) * dist_px * lead_in
                )
                desired = heading_from_points(end_pos, carrot)
        if reverse:
            desired = (desired + 180.0) % 360.0
        return desired

    # Iteratively refine delta so final heading matches desired heading from arc end to target
    for _ in range(8):
        end_pos, center, r0, geom_delta_deg, R, d_rad = _swing_geom(delta_deg, dir_geom)
        desired = _desired_heading(end_pos)
        current_target = (start_heading + delta_deg) % 360.0
        diff = ((desired - current_target + 180.0) % 360.0) - 180.0
        delta_new = delta_deg + diff
        if dir_effective == "cw":
            delta_new = -abs(delta_new)
        elif dir_effective == "ccw":
            delta_new = abs(delta_new)
        # If change is tiny, stop
        if abs(diff) < 1e-3:
            break
        delta_deg = delta_new

    end_pos, center, r0, geom_delta_deg, R, d_rad = _swing_geom(delta_deg, dir_geom)

    if abs(delta_deg) < 1e-4:
        return None

    arc_in = (abs(d_rad) * R) / PPI
    Tmove = _move_total_time(arc_in, cfg, v_override=drive_override)
    target_heading = (start_heading + delta_deg) % 360.0

    swing_vis = {
        "start_pos": p0,
        "end_pos": end_pos,
        "pivot": center,
        "start_heading": start_heading,
        "target_heading": target_heading,
        "diff_geom": {"center": center, "r0": r0, "delta_deg": geom_delta_deg, "cw": dir_geom == "cw"},
        "reverse": bool(reverse),
        "swing_dir": dir_norm
    }

    seg = {
        "type": "swing",
        "T": Tmove,
        "start_pos": p0,
        "end_pos": end_pos,
        "start_heading": start_heading,
        "target_heading": target_heading,
        "diff_geom": swing_vis["diff_geom"],
        "reverse": bool(reverse),
        "drive_speed_ips": drive_override,
        "delta_heading": delta_deg,
        "swing_dir": dir_norm,
        "length_in": arc_in
    }
    return seg, end_pos, target_heading, swing_vis

def compile_timeline(display_nodes, cfg, initial_heading, fps=60):
    """
    Compile node path into motion timeline.
    
    For curved segments (with path_to_next data), creates a path segment
    with pre-generated curve points. For straight segments, uses original logic.
    """
    n = len(display_nodes)
    if n <= 1: 
        return []
    
    segs = []
    reverse = reshape = False
    curr_heading = float(initial_heading)
    eff = _effective_centers(display_nodes, cfg, initial_heading)
    
    def _has_reverse_action(acts):
        return any(a.get("type") == "reverse" for a in acts)

    def _apply_reverse_action(reverse_state, act):
        state = act.get("state", None)
        if state is None:
            return not reverse_state
        return bool(state)

    def _edge_events_for(node):
        events = node.get("edge_events", [])
        if not isinstance(events, list):
            return []
        out = []
        for ev in events:
            if not isinstance(ev, dict):
                continue
            try:
                t_val = float(ev.get("t", 0.0))
            except Exception:
                t_val = 0.0
            t_val = max(0.0, min(1.0, t_val))
            ev_copy = dict(ev)
            ev_copy["t"] = t_val
            out.append(ev_copy)
        out.sort(key=lambda e: e.get("t", 0.0))
        return out

    def _is_chain_node(node):
        return bool(node.get("chain_through", False) or node.get("chain", False))

    def _chain_looseness(node):
        val = node.get("chain_looseness")
        try:
            val = 0.5 if val is None else float(val)
        except Exception:
            val = 0.5
        return max(0.0, min(1.0, val))

    def _node_has_pre_actions(node, acts=None):
        acts = acts if acts is not None else node.get("actions_out", node.get("actions", []))
        if acts:
            return True
        if node.get("reshape_toggle", False):
            return True
        if node.get("reverse", False) and not _has_reverse_action(acts):
            return True
        return False

    def _dir_between(p0, p1):
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        if dx * dx + dy * dy <= 1e-9:
            return None
        return math.atan2(dy, dx)

    def _dir_from_points(pts, at_end=False):
        if not pts or len(pts) < 2:
            return None
        if at_end:
            p0, p1 = pts[-2], pts[-1]
        else:
            p0, p1 = pts[0], pts[1]
        return _dir_between(p0, p1)

    def _out_dir_for_node(idx):
        if idx >= n - 1:
            return None
        start = eff[idx]
        pd = display_nodes[idx].get("path_to_next", {}) or {}
        pts = pd.get("pose_preview_points") or pd.get("path_points")
        if pts and len(pts) >= 2:
            return _dir_from_points(pts, at_end=False)
        cps = pd.get("control_points") or []
        if len(cps) >= 2:
            return _dir_between(start, cps[1])
        return _dir_between(start, eff[idx + 1])

    def _chain_params(in_dir, out_dir, looseness, max_cmd, max_speed, length_in):
        if in_dir is None or out_dir is None:
            ang_deg = 0.0
        else:
            d = (out_dir - in_dir + math.pi) % (2.0 * math.pi) - math.pi
            ang_deg = abs(math.degrees(d))
        ang_deg = max(0.0, min(180.0, ang_deg))
        ang_factor = 1.0 - (ang_deg / 180.0)
        try:
            looseness = max(0.0, min(1.0, float(looseness)))
        except Exception:
            looseness = 0.5
        looseness_curve = 1.0 - (1.0 - looseness) ** 2
        base_frac = 0.2 + 0.75 * looseness_curve
        dir_scale = 0.55 + 0.45 * ang_factor
        frac = max(0.12, min(0.95, base_frac * dir_scale))
        max_speed = max(0.0, float(max_speed))
        v_end = max_speed * frac
        max_cmd = max(0.0, float(max_cmd))
        min_cmd = int(round(max_cmd * frac * 0.75))
        base_exit = 0.8 + 6.0 * looseness_curve
        exit_scale = 0.6 + 0.4 * ang_factor
        early_exit = base_exit * exit_scale
        if length_in > 1e-6:
            early_exit = min(early_exit, length_in * (0.3 + 0.4 * looseness_curve))
        early_exit = max(0.5, early_exit)
        return v_end, frac, min_cmd, early_exit, ang_deg

    carry_speed = 0.0

    def _swing_action(p0, target_heading, start_heading, swing_dir, reverse_state, drive_override):
        th = math.radians(target_heading)
        p1 = (p0[0] + math.cos(th) * PPI, p0[1] - math.sin(th) * PPI)
        return _swing_segment(p0, p1, start_heading, swing_dir, cfg, reverse_state, drive_override)
    
    # Process edges
    path_cfg = cfg.get("path_config", {})
    base_la_in_global = path_cfg.get("lookahead_in")
    if base_la_in_global is None:
        bd = cfg.get("bot_dimensions", {})
        base_la_in_global = max(8.0, min(24.0, float(bd.get("dt_width", bd.get("width", 12.0))) * 0.9))
    base_lookahead_px = float(base_la_in_global) * PPI

    for i in range(n - 1):
        node_i = display_nodes[i]
        p0, p1 = eff[i], eff[i+1]
        # Anchor start at the node (do not carry overrides so arcs begin at the segment start)
        start_anchor = p0
        p0 = start_anchor
        profile_override = node_i.get("profile_override")
        prof_scale = _profile_speed_scale(profile_override)
        drive_override_cmd = node_i.get("custom_lateral_cmd")
        if drive_override_cmd is None:
            drive_override_cmd = node_i.get("custom_lateral_ips")
        drive_override_cmd = float(drive_override_cmd) if drive_override_cmd is not None else None
        if drive_override_cmd is not None:
            drive_override_cmd = _clamp_cmd(drive_override_cmd)
        if drive_override_cmd is not None:
            drive_override_cmd = _clamp_cmd(drive_override_cmd * prof_scale)
            drive_override = _cmd_to_ips(drive_override_cmd, cfg)
        else:
            drive_override = vmax_straight(cfg) * prof_scale
        turn_override = float(node_i.get("custom_turn_dps")) if node_i.get("custom_turn_dps") else None
        turn_speed_override = (turn_override * prof_scale) if turn_override is not None else (turn_rate(cfg) * prof_scale if prof_scale != 1.0 else None)
        vmax_cfg = vmax_straight(cfg)
        drive_speed_frac = 1.0
        try:
            if vmax_cfg > 1e-6:
                drive_speed_frac = float(drive_override) / vmax_cfg
        except Exception:
            drive_speed_frac = 1.0
        drive_speed_frac = max(0.2, min(1.0, drive_speed_frac))
        base_turn_rate = turn_rate(cfg)
        turn_speed_frac = 1.0
        try:
            if turn_speed_override is not None and base_turn_rate > 1e-6:
                turn_speed_frac = float(turn_speed_override) / base_turn_rate
        except Exception:
            turn_speed_frac = 1.0
        turn_speed_frac = max(0.2, min(1.0, min(1.0, turn_speed_frac)))
        acts = node_i.get("actions_out", node_i.get("actions", []))
        has_reverse_action = _has_reverse_action(acts)
        if node_i.get("reverse", False) and not has_reverse_action:
            reverse = not reverse
            segs.append({"type": "reverse", "T": 0.0, "pos": p0, "state": 2 if reverse else 1})
        reverse_after_swing = bool(node_i.get("reverse_after_swing"))
        next_is_swing = display_nodes[i + 1].get("turn_mode") == "swing" if i + 1 < n else False
        edge_events = _edge_events_for(node_i)
        if carry_speed > 1e-6 and _node_has_pre_actions(node_i, acts=acts):
            carry_speed = 0.0
        
        if node_i.get("reshape_toggle", False):
            reshape = not reshape
            segs.append({"type": "reshape", "T": 0.0, "pos": p0, "state": 2 if reshape else 1})
        
        # Pre-edge actions
        for act in acts:
            t = act.get("type")
            if t == "wait":
                segs.append({"type": "wait", "T": float(act.get("s", 0.0)),
                            "pos": start_anchor, "heading": curr_heading, "role": "custom", "node_i": i})
            elif t in ("reshape", "geom"):
                reshape = not reshape
                segs.append({"type": "reshape", "T": 0.0, "pos": start_anchor, "state": 2 if reshape else 1})
            elif t == "reverse":
                reverse = _apply_reverse_action(reverse, act)
                segs.append({"type": "reverse", "T": 0.0, "pos": start_anchor, "state": 2 if reverse else 1})
            elif t == "turn":
                tgt = float(act.get("deg", curr_heading)) % 360.0
                diff = _angle_diff_deg(curr_heading, tgt)
                Tturn = turn_time(diff, cfg, rate_override=turn_speed_override)
                segs.append({"type": "turn", "T": Tturn, "pos": start_anchor,
                            "start_heading": curr_heading, "target_heading": tgt,
                            "role": "explicit", "node_i": i, "turn_speed_dps": turn_speed_override,
                            "T_speed_frac": turn_speed_frac})
                curr_heading = tgt
                carry_speed = 0.0
            elif t == "swing":
                tgt = float(act.get("deg", curr_heading)) % 360.0
                sdir = act.get("dir", "auto")
                swing_res = _swing_action(start_anchor, tgt, curr_heading, sdir, reverse, drive_override)
                if swing_res is not None:
                    swing_seg, swing_end, swing_heading, _swing_vis = swing_res
                    swing_seg["role"] = "explicit"
                    swing_seg["node_i"] = i
                    swing_seg["T_speed_frac"] = drive_speed_frac
                    swing_seg["swing_settle"] = bool(act.get("settle", False))
                    segs.append(swing_seg)
                    start_anchor = swing_end
                    curr_heading = swing_heading
                    carry_speed = 0.0
        
        # Check if this segment uses a curved path
        path_data = node_i.setdefault("path_to_next", {})
        # clear transient visualization helpers
        for k in ("swing_vis", "start_override", "pose_preview_points"):
            if k in path_data:
                path_data.pop(k, None)
        use_path = path_data.get("use_path", False)
        profile_override = node_i.get("profile_override")
        prof_scale = _profile_speed_scale(profile_override)
        
        if use_path and path_data.get("control_points"):
            # CURVED SEGMENT - generate path points
            control_points = list(path_data["control_points"])
            if len(control_points) >= 2:
                # Anchor endpoints to effective centers (respect offsets)
                control_points[0] = start_anchor
                control_points[-1] = p1
            if len(control_points) < 3:
                use_path = False
                path_data["use_path"] = False
                path_data["control_points"] = []
        if use_path and path_data.get("control_points"):
            # CURVED SEGMENT - generate path points
            control_points = list(path_data["control_points"])
            if len(control_points) >= 2:
                # Anchor endpoints to effective centers (respect offsets)
                control_points[0] = start_anchor
                control_points[-1] = p1
            path_points = generate_bezier_path(control_points, num_samples=50)
            
            if len(path_points) >= 2:
                # Per-path lookahead override (base value before scaling)
                la_override_in = path_data.get("lookahead_in_override")
                base_la_in = la_override_in if la_override_in is not None else base_la_in_global
                if base_la_in is None:
                    bd = cfg.get("bot_dimensions", {})
                    base_la_in = max(8.0, min(24.0, float(bd.get("dt_width", bd.get("width", 12.0))) * 0.9))
                base_lookahead_px = float(base_la_in) * PPI

                # Turn to face the start of the path only if heading is far off (>15°)
                start_heading = calculate_path_heading(path_points, 0)
                end_heading = calculate_path_heading(path_points, len(path_points) - 1)
                if reverse:
                    start_heading = (start_heading + 180.0) % 360.0
                    end_heading = (end_heading + 180.0) % 360.0
                
                diff_to_start = _angle_diff_deg(curr_heading, start_heading)
                if carry_speed > 1e-6:
                    diff_to_start = 0.0
                if abs(diff_to_start) > 15.0:
                    Tface = turn_time(diff_to_start, cfg, rate_override=turn_speed_override)
                    if Tface > 0.0:
                        segs.append({
                            "type": "turn",
                            "T": Tface,
                            "pos": start_anchor,
                            "start_heading": curr_heading,
                            "target_heading": start_heading,
                            "role": "face_path",
                            "edge_i": i,
                            "turn_speed_dps": turn_speed_override,
                            "T_speed_frac": turn_speed_frac
                        })
                    curr_heading = start_heading
                else:
                    # Assume we simply roll into the path with existing heading
                    curr_heading = start_heading
                
                # Calculate path length/time with curvature + resampling
                speed_mult = float(path_data.get("speed_mult", 1.0)) * prof_scale
                min_override = path_data.get("min_speed_cmd")
                if min_override is None:
                    min_override = path_data.get("min_speed_ips")
                min_override = _clamp_cmd(min_override) if min_override is not None else None
                max_override = path_data.get("max_speed_cmd")
                if max_override is None:
                    max_override = path_data.get("max_speed_ips")
                max_override = _clamp_cmd(max_override) if max_override is not None else None
                max_cfg_cmd = path_cfg.get("max_speed_cmd", path_cfg.get("max_speed_ips", 127.0))
                base_max_cmd = max_override if max_override is not None else (drive_override_cmd if drive_override_cmd is not None else max_cfg_cmd)
                base_max_cmd = _clamp_cmd(base_max_cmd)
                path_speed_frac = (base_max_cmd / 127.0) * speed_mult
                path_speed_frac = max(0.2, min(1.0, path_speed_frac))
                max_speed_ips = _cmd_to_ips(base_max_cmd, cfg) * speed_mult
                v_start = min(carry_speed, max_speed_ips)
                v_end = 0.0
                chain_next = False
                chain_frac = None
                chain_min_cmd = 0
                chain_early_exit = 0.0
                chain_angle = None
                in_dir = _dir_between(start_anchor, p1)
                if reverse and in_dir is not None:
                    in_dir = (in_dir + math.pi) % (2.0 * math.pi)
                out_dir = None
                looseness = None
                if i + 1 < n - 1:
                    next_node = display_nodes[i + 1]
                    if _is_chain_node(next_node) and not _node_has_pre_actions(next_node):
                        out_dir = _out_dir_for_node(i + 1)
                        looseness = _chain_looseness(next_node)
                        v_end, chain_frac, chain_min_cmd, chain_early_exit, chain_angle = _chain_params(
                            in_dir, out_dir, looseness, base_max_cmd, max_speed_ips, 0.0
                        )
                        chain_next = True
                L_in, Tmove, resampled, curvatures, speeds, speeds_raw = path_time_with_curvature(  # type: ignore[misc]
                    path_points, cfg, speed_mult=speed_mult,
                    min_speed_override=min_override,
                    max_speed_override=max_override if drive_override_cmd is None else drive_override_cmd,
                    v0=v_start,
                    v1=v_end
                )
                if chain_next and looseness is not None:
                    v_end, chain_frac, chain_min_cmd, chain_early_exit, chain_angle = _chain_params(
                        in_dir, out_dir, looseness, base_max_cmd, max_speed_ips, L_in
                    )
                # Adjust lookahead based on curvature and expected velocity.
                # Curvature dominates (tighter = shorter), speed nudges longer/shorter.
                avg_curv = 0.0
                max_curv = 0.0
                if curvatures:
                    vals = [abs(c) for c in curvatures]
                    avg_curv = sum(vals) / max(1, len(vals))
                    max_curv = max(vals)
                curv_metric = max(avg_curv * 1.5, max_curv * 0.75)
                la_scale_curv = 1.0 / (1.0 + 8.0 * curv_metric)
                la_scale_curv = max(0.2, min(1.0, la_scale_curv))

                # Speed influence: map avg speed to a modest scale (<=60 => shrink, >=110 => grow)
                vmax_cfg = vmax_straight(cfg)
                avg_speed = vmax_cfg
                try:
                    if speeds:
                        avg_speed = sum(speeds) / max(1, len(speeds))
                except Exception:
                    avg_speed = vmax_cfg
                # linear map: 60 -> 0.75x, 110 -> 1.2x
                lo_s, hi_s = 60.0, 110.0
                lo_f, hi_f = 0.75, 1.2
                t = (avg_speed - lo_s) / max(1e-6, hi_s - lo_s)
                speed_factor = lo_f + (hi_f - lo_f) * max(0.0, min(1.0, t))
                speed_factor = max(0.7, min(1.25, speed_factor))

                la_scale = la_scale_curv * speed_factor
                la_scale = max(0.2, min(1.1, la_scale))
                seg_lookahead_px = base_lookahead_px * la_scale
                seg_lookahead_px = max(6.0 * PPI, min(60.0 * PPI, seg_lookahead_px))

                path_meta = _build_path_meta(resampled, speeds)
                
                segs.append({
                    "type": "path",  # New segment type for curved paths
                    "T": Tmove,
                    "path_length_in": L_in,
                    "p0": start_anchor,
                    "p1": p1,
                    "path_points": resampled,
                    "path_curvatures": curvatures,
                    "path_speeds": speeds,
                    "path_speeds_export": speeds_raw,
                    "path_meta": path_meta,
                    "min_speed_cmd": min_override if min_override is not None else cfg.get("path_config", {}).get("min_speed_cmd", cfg.get("path_config", {}).get("min_speed_ips", 0.0)),
                    "max_speed_cmd": max_override if max_override is not None else (drive_override_cmd if drive_override_cmd is not None else cfg.get("path_config", {}).get("max_speed_cmd", cfg.get("path_config", {}).get("max_speed_ips", 127.0))),
                    "facing": end_heading,
                    "i0": i,
                    "i1": i + 1,
                    "reverse": reverse,
                    "skip_buffer": _has_custom_waits_at_node(display_nodes[i+1]) or next_is_swing,
                    "segment_idx": i,  # For path file export
                    "drive_speed_cmd": drive_override_cmd,
                    "drive_speed_ips": drive_override,
                    "T_speed_frac": path_speed_frac,
                    "lookahead_px": seg_lookahead_px,
                    "lookahead_in_override": la_override_in,
                    "profile_override": profile_override,
                    "v_start": v_start,
                    "v_end": v_end,
                    "chain_to_next": chain_next,
                    "chain_min_speed": chain_min_cmd,
                    "chain_early_exit": chain_early_exit,
                    "chain_speed_frac": chain_frac,
                    "chain_angle_deg": chain_angle
                })
                if edge_events:
                    segs[-1]["edge_events"] = edge_events
                carry_speed = v_end if chain_next else 0.0
                
            curr_heading = end_heading
        else:
            # Fallback to straight if path generation failed
            use_path = False
        
        if not use_path:
            start_pt = start_anchor
            turn_mode = node_i.get("turn_mode", "turn")
            swing_dir = node_i.get("swing_dir", "auto")
            # Optional swing arc before the straight/pose motion
            if turn_mode == "swing":
                swing_start_heading = curr_heading
                pose_heading = node_i.get("pose_heading_deg") if node_i.get("move_to_pose") else None
                pose_lead = node_i.get("pose_lead_in") if node_i.get("move_to_pose") else None
                swing_res = _swing_segment(
                    start_pt,
                    p1,
                    swing_start_heading,
                    swing_dir,
                    cfg,
                    reverse,
                    drive_override,
                    pose_heading=pose_heading,
                    pose_lead=pose_lead
                )
                if swing_res is not None:
                    swing_seg, swing_end, swing_heading, swing_vis = swing_res
                    swing_seg["i0"] = i
                    swing_seg["i1"] = i + 1
                    swing_seg["profile_override"] = profile_override
                    swing_seg["drive_speed_cmd"] = drive_override_cmd
                    swing_seg["T_speed_frac"] = drive_speed_frac
                    swing_seg["swing_settle"] = bool(node_i.get("swing_settle", False))
                    segs.append(swing_seg)
                    swing_heading_disp = swing_heading
                    swing_vis["start_heading"] = swing_start_heading
                    swing_vis["target_heading"] = swing_heading_disp
                    path_data["swing_vis"] = swing_vis
                    path_data["start_override"] = swing_end
                    start_pt = swing_end
                    curr_heading = swing_heading_disp
                    if reverse_after_swing:
                        reverse = not reverse

            facing_override = node_i.get("swing_target_heading_deg")
            facing = heading_from_points(start_pt, p1)
            if facing_override is not None:
                facing = float(facing_override) % 360.0
                if reverse:
                    facing = (facing + 180.0) % 360.0
            elif reverse:
                facing = (facing + 180.0) % 360.0

            # movetopose curve (only on straight segments)
            use_pose = bool(node_i.get("move_to_pose", False))
            pose_heading = node_i.get("pose_heading_deg")
            pose_lead = node_i.get("pose_lead_in")
            if use_pose:
                if pose_heading is None:
                    pose_heading = facing
                lead_in = 0.0
                try:
                    if pose_lead is not None:
                        lead_in = max(0.0, float(pose_lead))
                except Exception:
                    lead_in = 0.0
                dist_px = math.hypot(p1[0] - start_pt[0], p1[1] - start_pt[1])
                if dist_px <= 1e-6:
                    dist_px = 1.0
                dist_in = dist_px / PPI

                end_heading = pose_heading
                if reverse:
                    end_heading = (end_heading + 180.0) % 360.0
                disp_heading = convert_heading_input(end_heading, None)
                th = math.radians(disp_heading)
                carrot = (
                    p1[0] + math.cos(th) * dist_px * lead_in,
                    p1[1] + math.sin(th) * dist_px * lead_in
                )
                samples = max(20, int(dist_in * 4.0))
                path_points = _sample_quad_bezier(start_pt, carrot, p1, num=samples)
                if len(path_points) >= 2:
                    start_heading = calculate_path_heading(path_points, 0)
                else:
                    start_heading = heading_from_points(start_pt, p1)
                if reverse:
                    start_heading = (start_heading + 180.0) % 360.0
                diff_to_start = _angle_diff_deg(curr_heading, start_heading)
                if carry_speed > 1e-6:
                    diff_to_start = 0.0
                if abs(diff_to_start) > 0.5:
                    Tface = turn_time(diff_to_start, cfg, rate_override=turn_speed_override)
                    if Tface > 0.0:
                        segs.append({
                            "type": "turn",
                            "T": Tface,
                            "pos": start_pt,
                            "start_heading": curr_heading,
                            "target_heading": start_heading,
                            "role": "pose_entry",
                            "edge_i": i,
                            "turn_speed_dps": turn_speed_override,
                            "T_speed_frac": turn_speed_frac
                        })
                    curr_heading = start_heading
                    carry_speed = 0.0
                else:
                    curr_heading = start_heading
                speed_mult = 1.0 * prof_scale
                max_cfg_cmd = path_cfg.get("max_speed_cmd", path_cfg.get("max_speed_ips", 127.0))
                base_max_cmd = drive_override_cmd if drive_override_cmd is not None else max_cfg_cmd
                base_max_cmd = _clamp_cmd(base_max_cmd)
                path_speed_frac = (base_max_cmd / 127.0) * speed_mult
                path_speed_frac = max(0.2, min(1.0, path_speed_frac))
                min_override = None
                max_override = drive_override_cmd
                max_speed_ips = _cmd_to_ips(base_max_cmd, cfg) * speed_mult
                v_start = min(carry_speed, max_speed_ips)
                v_end = 0.0
                chain_next = False
                chain_frac = None
                chain_min_cmd = 0
                chain_early_exit = 0.0
                chain_angle = None
                in_dir = _dir_between(start_pt, p1)
                if reverse and in_dir is not None:
                    in_dir = (in_dir + math.pi) % (2.0 * math.pi)
                out_dir = None
                looseness = None
                if i + 1 < n - 1:
                    next_node = display_nodes[i + 1]
                    if _is_chain_node(next_node) and not _node_has_pre_actions(next_node):
                        out_dir = _out_dir_for_node(i + 1)
                        looseness = _chain_looseness(next_node)
                        v_end, chain_frac, chain_min_cmd, chain_early_exit, chain_angle = _chain_params(
                            in_dir, out_dir, looseness, base_max_cmd, max_speed_ips, 0.0
                        )
                        chain_next = True
                L_in, Tmove, resampled, curvatures, speeds, speeds_raw = path_time_with_curvature(  # type: ignore[misc]
                    path_points, cfg, speed_mult=speed_mult,
                    min_speed_override=min_override,
                    max_speed_override=max_override,
                    v0=v_start,
                    v1=v_end
                )
                if chain_next and looseness is not None:
                    v_end, chain_frac, chain_min_cmd, chain_early_exit, chain_angle = _chain_params(
                        in_dir, out_dir, looseness, base_max_cmd, max_speed_ips, L_in
                    )
                avg_curv = 0.0
                max_curv = 0.0
                if curvatures:
                    vals = [abs(c) for c in curvatures]
                    avg_curv = sum(vals) / max(1, len(vals))
                    max_curv = max(vals)
                curv_metric = max(avg_curv * 1.5, max_curv * 0.75)
                la_scale_curv = 1.0 / (1.0 + 8.0 * curv_metric)
                la_scale_curv = max(0.2, min(1.0, la_scale_curv))
                vmax_cfg = vmax_straight(cfg)
                avg_speed = vmax_cfg
                try:
                    if speeds:
                        avg_speed = sum(speeds) / max(1, len(speeds))
                except Exception:
                    avg_speed = vmax_cfg
                lo_s, hi_s = 60.0, 110.0
                lo_f, hi_f = 0.75, 1.2
                t = (avg_speed - lo_s) / max(1e-6, hi_s - lo_s)
                speed_factor = lo_f + (hi_f - lo_f) * max(0.0, min(1.0, t))
                speed_factor = max(0.7, min(1.25, speed_factor))
                la_scale = la_scale_curv * speed_factor
                la_scale = max(0.2, min(1.1, la_scale))
                seg_lookahead_px = base_lookahead_px * la_scale
                seg_lookahead_px = max(6.0 * PPI, min(60.0 * PPI, seg_lookahead_px))

                segs.append({
                    "type": "path",
                    "T": Tmove,
                    "path_length_in": L_in,
                    "p0": start_pt,
                    "p1": p1,
                    "path_points": resampled,
                    "path_curvatures": curvatures,
                    "path_speeds": speeds,
                    "path_speeds_export": speeds_raw,
                    "min_speed_cmd": cfg.get("path_config", {}).get("min_speed_cmd", cfg.get("path_config", {}).get("min_speed_ips", 0.0)),
                    "max_speed_cmd": drive_override_cmd if drive_override_cmd is not None else cfg.get("path_config", {}).get("max_speed_cmd", cfg.get("path_config", {}).get("max_speed_ips", 127.0)),
                    "facing": pose_heading,
                    "i0": i,
                    "i1": i + 1,
                    "reverse": reverse,
                    "skip_buffer": _has_custom_waits_at_node(display_nodes[i+1]) or next_is_swing,
                    "segment_idx": i,
                    "drive_speed_cmd": drive_override_cmd,
                    "drive_speed_ips": drive_override,
                    "T_speed_frac": path_speed_frac,
                    "lookahead_px": seg_lookahead_px,
                    "move_to_pose": True,
                    "pose_heading": pose_heading,
                    "pose_lead_in": pose_lead,
                    "v_start": v_start,
                    "v_end": v_end,
                    "chain_to_next": chain_next,
                    "chain_min_speed": chain_min_cmd,
                    "chain_early_exit": chain_early_exit,
                    "chain_speed_frac": chain_frac,
                    "chain_angle_deg": chain_angle
                })
                if edge_events:
                    segs[-1]["edge_events"] = edge_events
                carry_speed = v_end if chain_next else 0.0
                path_data["pose_preview_points"] = resampled
                path_data["start_override"] = start_pt
                curr_heading = pose_heading
            else:
                diff_to_facing = _angle_diff_deg(curr_heading, facing)
                if carry_speed > 1e-6:
                    diff_to_facing = 0.0
                L_in = math.hypot(p1[0] - start_pt[0], p1[1] - start_pt[1]) / PPI

                Tturn = turn_time(diff_to_facing, cfg, rate_override=turn_speed_override)
                if Tturn > 0.0:
                    segs.append({
                        "type": "turn",
                        "T": Tturn,
                        "pos": start_pt,
                        "start_heading": curr_heading,
                        "target_heading": facing,
                        "role": "face_line",
                        "edge_i": i,
                        "turn_speed_dps": turn_speed_override,
                        "T_speed_frac": turn_speed_frac
                    })
                    carry_speed = 0.0
                curr_heading = facing

                # Move along straight segment
                v_start = min(carry_speed, drive_override)
                v_end = 0.0
                chain_next = False
                chain_frac = None
                chain_min_cmd = 0
                chain_early_exit = 0.0
                chain_angle = None
                in_dir = _dir_between(start_pt, p1)
                if reverse and in_dir is not None:
                    in_dir = (in_dir + math.pi) % (2.0 * math.pi)
                if i + 1 < n - 1:
                    next_node = display_nodes[i + 1]
                    if _is_chain_node(next_node) and not _node_has_pre_actions(next_node):
                        out_dir = _out_dir_for_node(i + 1)
                        looseness = _chain_looseness(next_node)
                        max_cmd = drive_override_cmd if drive_override_cmd is not None else 127.0
                        v_end, chain_frac, chain_min_cmd, chain_early_exit, chain_angle = _chain_params(
                            in_dir, out_dir, looseness, max_cmd, drive_override, L_in
                        )
                        chain_next = True
                Tmove = _move_total_time(L_in, cfg, v_override=drive_override, v0=v_start, v1=v_end)
                segs.append({
                    "type": "move",
                    "T": Tmove,
                    "length_in": L_in,
                    "p0": start_pt,
                    "p1": p1,
                    "facing": facing,
                    "i0": i,
                    "i1": i + 1,
                    "reverse": reverse,
                    "skip_buffer": _has_custom_waits_at_node(display_nodes[i+1]) or next_is_swing,
                    "profile_override": profile_override,
                    "drive_speed_cmd": drive_override_cmd,
                    "drive_speed_ips": drive_override,
                    "T_speed_frac": drive_speed_frac,
                    "v_start": v_start,
                    "v_end": v_end,
                    "chain_to_next": chain_next,
                    "chain_min_speed": chain_min_cmd,
                    "chain_early_exit": chain_early_exit,
                    "chain_speed_frac": chain_frac,
                    "chain_angle_deg": chain_angle
                })
                if edge_events:
                    segs[-1]["edge_events"] = edge_events
                carry_speed = v_end if chain_next else 0.0
                curr_heading = facing

        
        # Arrival reshape
        if display_nodes[i+1].get("reshape_toggle_arrival", False):
            reshape = not reshape
            segs.append({"type": "reshape", "T": 0.0, "pos": p1, "state": 2 if reshape else 1})
    
    return segs

def estimate_time(display_nodes, cfg, initial_heading, fps=60):
    """Estimate total routine time."""
    tl = compile_timeline(display_nodes, cfg, initial_heading, fps)
    return sum(seg.get("T", 0.0) for seg in tl)

def _line_circle_intersections(p1, p2, center, radius_px):
    """Return intersection points (0,1) of segment p1-p2 with circle center/radius."""
    (x1, y1), (x2, y2) = p1, p2
    cx, cy = center
    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - cx, y1 - cy
    a = dx*dx + dy*dy
    b = 2 * (fx*dx + fy*dy)
    c = (fx*fx + fy*fy) - radius_px*radius_px
    disc = b*b - 4*a*c
    if disc < 0 or abs(a) <= 1e-12:
        return []
    disc = disc**0.5
    t1 = (-b - disc) / (2*a)
    t2 = (-b + disc) / (2*a)
    pts = []
    for t in (t1, t2):
        if 0.0 <= t <= 1.0:
            pts.append((x1 + t*dx, y1 + t*dy, t))
    return pts


def sample_path_position(t, path_points, total_time, cfg, lookahead_px=None, use_pursuit=False, path_speeds=None, current_pos=None, path_meta=None):
    """
    Sample position along a curved path at time t.
    Uses the same trapezoidal motion profile as straight moves.
    
    Returns (x, y) position, heading, and optional lookahead point.
    """
    if not path_points or len(path_points) < 2:
        return (path_points[0] if path_points else (0, 0)), 0.0, None
    # helper: closest point on path to current_pos
    def _closest_path_progress(pts, pos):
        best_d2 = float("inf")
        best_s = 0.0
        best_seg = 0
        best_pt = pts[0]
        accum = 0.0
        for i in range(len(pts) - 1):
            a = pts[i]; b = pts[i+1]
            abx = b[0] - a[0]; aby = b[1] - a[1]
            seg_len2 = abx*abx + aby*aby
            if seg_len2 <= 1e-9:
                continue
            apx = pos[0] - a[0]; apy = pos[1] - a[1]
            t_proj = max(0.0, min(1.0, (apx*abx + apy*aby) / seg_len2))
            proj = (a[0] + abx*t_proj, a[1] + aby*t_proj)
            d2 = (proj[0]-pos[0])**2 + (proj[1]-pos[1])**2
            if d2 < best_d2:
                best_d2 = d2
                best_seg = i
                best_pt = proj
                best_s = accum + math.sqrt(seg_len2) * t_proj
            accum += math.sqrt(seg_len2)
        return best_s, best_seg, best_pt
    
    # Calculate segment lengths
    meta = path_meta or {}
    total_px = meta.get("total_px")
    if total_px is None:
        total_px = _path_length_px(path_points)
    if total_px <= 1e-9:
        return path_points[-1], calculate_path_heading(path_points, len(path_points) - 1), None

    # If we have per-point speeds, use them to map time -> distance
    speeds = path_speeds if path_speeds and len(path_speeds) == len(path_points) else None
    seg_lengths_px = meta.get("seg_lengths_px")
    if seg_lengths_px is None:
        seg_lengths_px = []
        for i in range(len(path_points) - 1):
            seg_lengths_px.append(math.hypot(path_points[i+1][0] - path_points[i][0],
                                             path_points[i+1][1] - path_points[i][1]))
    if speeds and total_time > 1e-9:
        seg_times = meta.get("seg_times")
        total_T = meta.get("seg_time_total", 0.0)
        if not seg_times or len(seg_times) != len(seg_lengths_px):
            seg_times = []
            total_T = 0.0
            for i, ds_px in enumerate(seg_lengths_px):
                v0 = max(1e-3, speeds[i])
                v1 = max(1e-3, speeds[i+1])
                v = 0.5 * (v0 + v1)
                dt = (ds_px / PPI) / v
                seg_times.append(dt)
                total_T += dt
            if path_meta is not None:
                meta["seg_times"] = seg_times
                meta["seg_time_total"] = total_T
        # Clamp t to total_T (fallback to provided total_time if mismatch)
        budget = total_T if total_T > 1e-9 else total_time
        t_clamped = max(0.0, min(budget, t))
        # Find segment by accumulated time
        acc_t = 0.0
        seg_idx = 0
        while seg_idx < len(seg_times) and acc_t + seg_times[seg_idx] < t_clamped - 1e-9:
            acc_t += seg_times[seg_idx]
            seg_idx += 1
        if seg_idx >= len(seg_times):
            seg_idx = len(seg_times) - 1
            acc_t = max(0.0, budget - seg_times[seg_idx])
        seg_dt = seg_times[seg_idx] if seg_idx < len(seg_times) else 1e-9
        frac = 0.0 if seg_dt <= 1e-9 else (t_clamped - acc_t) / seg_dt
        frac = max(0.0, min(1.0, frac))
        # Distance along path in px
        s_px = sum(seg_lengths_px[:seg_idx]) + frac * seg_lengths_px[seg_idx]
    else:
        # Fallback: proportional distance
        if total_time <= 0.0:
            return path_points[-1], calculate_path_heading(path_points, len(path_points) - 1), None
        progress = max(0.0, min(1.0, t / total_time))
        s_px = total_px * progress
        seg_idx = 0
    
    # If pursuing, anchor progress to closest point to the current position to avoid fallback
    if use_pursuit and current_pos is not None:
        s_px, seg_idx, pos = _closest_path_progress(path_points, current_pos)
    else:
        pos, seg_idx = path_point_at_distance(path_points, s_px, path_meta=meta)
    heading = calculate_path_heading(path_points, seg_idx)
    look_pt = None
    
    if use_pursuit and lookahead_px and lookahead_px > 0.0:
        center = current_pos if current_pos is not None else pos
        # search for first intersection ahead along path
        remaining = s_px
        found = None
        for i in range(seg_idx, len(path_points) - 1):
            pA = path_points[i]
            pB = path_points[i+1]
            seg_len = math.hypot(pB[0]-pA[0], pB[1]-pA[1])
            if remaining > 0 and seg_len > 0:
                t_skip = min(1.0, remaining / seg_len)
            else:
                t_skip = 0.0
            # Move start along segment by remaining distance
            start_pt = (pA[0] + (pB[0]-pA[0])*t_skip, pA[1] + (pB[1]-pA[1])*t_skip)
            inters = _line_circle_intersections(start_pt, pB, center, lookahead_px)
            if inters:
                # pick the first along this segment
                found = inters[0][:2]
                break
            remaining = max(0.0, remaining - seg_len)
        if found is None:
            la = min(lookahead_px, max(0.0, total_px - s_px))
            found, _ = path_point_at_distance(path_points, s_px + la, path_meta=meta)
        look_pt = found
        if look_pt:
            dx = look_pt[0] - center[0]
            dy = look_pt[1] - center[1]
            heading = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
    
    return pos, heading, look_pt


def path_point_at_distance(path_points, dist_px, path_meta=None):
    """Return point and segment index along path at given distance (px)."""
    if not path_points:
        return (0.0, 0.0), 0
    if dist_px <= 0.0:
        return path_points[0], 0

    meta = path_meta or {}
    cum_lengths = meta.get("cum_lengths_px")
    seg_lengths = meta.get("seg_lengths_px")
    if cum_lengths and seg_lengths and len(cum_lengths) == len(path_points):
        idx = bisect.bisect_left(cum_lengths, dist_px)
        if idx <= 0:
            return path_points[0], 0
        if idx >= len(cum_lengths):
            return path_points[-1], len(path_points) - 1
        seg_idx = idx - 1
        segment_len = seg_lengths[seg_idx]
        start_dist = cum_lengths[seg_idx]
        frac = 0.0 if segment_len <= 1e-9 else (dist_px - start_dist) / segment_len
        x = path_points[seg_idx][0] + frac * (path_points[seg_idx + 1][0] - path_points[seg_idx][0])
        y = path_points[seg_idx][1] + frac * (path_points[seg_idx + 1][1] - path_points[seg_idx][1])
        return (x, y), seg_idx

    cumulative_dist = 0.0
    for i in range(len(path_points) - 1):
        segment_len = math.hypot(
            path_points[i+1][0] - path_points[i][0],
            path_points[i+1][1] - path_points[i][1]
        )
        if cumulative_dist + segment_len >= dist_px:
            frac = 0.0 if segment_len <= 1e-9 else (dist_px - cumulative_dist) / segment_len
            x = path_points[i][0] + frac * (path_points[i+1][0] - path_points[i][0])
            y = path_points[i][1] + frac * (path_points[i+1][1] - path_points[i][1])
            return (x, y), i
        cumulative_dist += segment_len

    return path_points[-1], len(path_points) - 1

def correct_nodes_inbounds(nodes, cfg, init_heading, window_w, window_h):
    """Previous OOB clamping removed to avoid teleporting nodes; no-op now."""
    return
