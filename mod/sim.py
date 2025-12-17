# mod/sim.py
import math

# Support running as script (no package) by falling back to absolute imports
if __package__ is None or __package__ == "":
    import os, sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from config import PPI  # type: ignore
    from util import heading_from_points, get_node_offset_in, approach_unit  # type: ignore
    from path_utils import generate_bezier_path, calculate_path_heading, resample_path_uniform  # type: ignore
else:
    from .config import PPI
    from .util import heading_from_points, get_node_offset_in, approach_unit
    from .path_utils import generate_bezier_path, calculate_path_heading, resample_path_uniform

MIN_TURN_TIME_S = 0.14
MIN_MOVE_TIME_S = 1.0 / 60.0
PROFILE_SPEED_SCALE = {
    "precise": 0.75,
    "normal": 1.0,
    "fast": 1.15,
    "slam": 1.3
}
DRIFT_MAX = 15.0

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

def _free_speed_ips(cfg):
    """Calculate free wheel speed in inches per second."""
    rpm = float(cfg["robot_physics"].get("rpm", 200.0))
    d_in = float(cfg["robot_physics"].get("diameter", 4.0))
    return rpm * math.pi * max(0.01, d_in) / 60.0

def _clamp_cmd(cmd_val):
    """Clamp a command-style speed to [0, 127]."""
    try:
        return max(0.0, min(127.0, float(cmd_val)))
    except Exception:
        return 0.0

def _cmd_to_ips(cmd_val, cfg):
    """Convert a command speed (0-127) into ips based on current drivetrain limits."""
    vmax_cmd = vmax_straight(cfg)
    return vmax_cmd * (_clamp_cmd(cmd_val) / 127.0)

def _drift_norm(cfg):
    """Clamp horizontal drift to [0, DRIFT_MAX] and return normalized 0..1."""
    try:
        val = float(cfg.get("robot_physics", {}).get("horizontal_drift", 0.0))
    except Exception:
        val = 0.0
    val = max(0.0, min(DRIFT_MAX, val))
    return val / DRIFT_MAX

def _profile_speed_scale(name: str) -> float:
    """Map profile name to a speed multiplier."""
    return PROFILE_SPEED_SCALE.get(str(name).lower(), 1.0) if name else 1.0

def _has_custom_waits_at_node(node: dict) -> bool:
    """Check if node has custom wait actions."""
    acts = node.get("actions_out", node.get("actions", []))
    return any(a.get("type") == "wait" and float(a.get("s", 0) or 0) > 0 for a in acts)

def vmax_straight(cfg):
    """Calculate maximum straight-line velocity."""
    v = float(cfg["robot_physics"].get("volts_straight", 12.0))
    wlb = float(cfg["robot_physics"].get("weight", 20.0))
    drift_norm = _drift_norm(cfg)
    omni = float(cfg["robot_physics"].get("all_omni", 0))
    free = _free_speed_ips(cfg)
    load = 0.88 * (v / 12.0) * (20.0 / max(10.0, wlb)) ** 0.06
    mc = max(0.0, min(127.0, float(cfg["robot_physics"].get("max_cmd", 127.0))))
    scale = mc / 127.0
    dyn_scale = 1.0
    if omni:
        dyn_scale *= 0.9
    if drift_norm > 0.0:
        dyn_scale *= max(0.5, 1.0 - 0.3 * drift_norm)
    return max(10.0, min(220.0, free * load * scale * dyn_scale))

def accel_straight(cfg):
    """Calculate straight-line acceleration."""
    mu = float(cfg["robot_physics"].get("mu", 0.9))
    v = float(cfg["robot_physics"].get("volts_straight", 12.0))
    drift_norm = _drift_norm(cfg)
    omni = float(cfg["robot_physics"].get("all_omni", 0))
    a_t = max(60.0, min(0.95 * mu * 386.09, 450.0))
    vmax = vmax_straight(cfg)
    t_to_v = 0.28 * (12.0 / max(1e-3, v)) ** 0.5
    a_m = vmax / max(0.12, min(0.60, t_to_v))
    dyn_scale = 1.0
    if omni:
        dyn_scale *= 0.9
    if drift_norm > 0.0:
        dyn_scale *= max(0.5, 1.0 - 0.3 * drift_norm)
    return max(60.0, min(a_t, a_m)) * dyn_scale

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


def path_time_with_curvature(path_points, cfg, speed_mult=1.0, min_speed_override=None, max_speed_override=None):
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
    max_base_cmd = _clamp_cmd(max_speed_override if max_speed_override is not None else pcfg.get("max_speed_ips", 127.0))
    max_base = _cmd_to_ips(max_base_cmd, cfg) * speed_mult
    if max_base <= 1e-6:
        max_base = _cmd_to_ips(127.0, cfg) * speed_mult
    min_cmd = _clamp_cmd(min_speed_override if min_speed_override is not None else pcfg.get("min_speed_ips", 0.0))
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
    profile_time = _move_total_time(total_px / PPI, cfg, v_override=max_base)
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

    time_s = final_time

    profile_time = _move_total_time(total_px / PPI, cfg, v_override=max_base)
    time_s = max(time_s, profile_time)

    return total_px / PPI, time_s, resampled, point_curvatures, point_speeds, point_speeds_raw
def _move_total_time(L_in, cfg, v_override=None):
    """Calculate total time for move distance, honoring optional speed override."""
    if L_in <= 0.0: 
        return 0.0
    v_nom = vmax_straight(cfg)
    v = min(v_nom, float(v_override)) if v_override is not None else v_nom
    a_nom = accel_straight(cfg)
    a = a_nom * (v / max(1e-6, v_nom))
    t_acc = v / max(1e-6, a)
    d_acc = 0.5 * a * t_acc * t_acc
    if 2.0 * d_acc >= L_in:
        return max(MIN_MOVE_TIME_S, 2.0 * math.sqrt(L_in / max(1e-6, a)))
    return max(MIN_MOVE_TIME_S, 2.0 * t_acc + (L_in - 2.0 * d_acc) / max(1e-6, v))

def sample_move_profile(t, L_in, cfg, v_override=None):
    """Sample distance at time t along move profile with optional speed override."""
    if L_in <= 1e-9 or t <= 0.0: 
        return 0.0
    v_nom = vmax_straight(cfg)
    v = min(v_nom, float(v_override)) if v_override is not None else v_nom
    a_nom = accel_straight(cfg)
    a = a_nom * (v / max(1e-6, v_nom))
    t_acc = v / max(1e-6, a)
    d_acc = 0.5 * a * t_acc * t_acc
    
    if 2.0 * d_acc >= L_in:
        t_half = math.sqrt(L_in / max(1e-6, a))
        if t <= t_half:
            s = 0.5 * a * t * t
        else:
            t2 = t - t_half
            s = 0.5 * L_in + a * t_half * t2 - 0.5 * a * t2 * t2
    else:
        t_flat = (L_in - 2.0 * d_acc) / max(1e-6, v)
        if t <= t_acc:
            s = 0.5 * a * t * t
        elif t <= t_acc + t_flat:
            s = d_acc + v * (t - t_acc)
        else:
            t2 = t - (t_acc + t_flat)
            s = d_acc + v * t_flat + (v * t2 - 0.5 * a * t2 * t2)
    
    return max(0.0, min(L_in, s))

def _track_width_in(cfg):
    """Get robot track width."""
    bd = cfg.get("bot_dimensions", {})
    return max(6.0, float(bd.get("dt_width", bd.get("width", 12.0))))

def _bot_half_track_px(cfg):
    """Half the track width, in pixels (used for swing arcs)."""
    return 0.5 * _track_width_in(cfg) * PPI

def turn_rate(cfg):
    """Calculate turn rate in degrees per second."""
    v_lin = vmax_straight(cfg)
    track = _track_width_in(cfg)
    drift_norm = _drift_norm(cfg)
    omni = float(cfg["robot_physics"].get("all_omni", 0))
    base_omega_rad = (2.0 * v_lin) / max(1e-6, track)
    base_deg = base_omega_rad * (180.0 / math.pi)
    vt = float(cfg["robot_physics"].get("volts_turn", 12.0))
    rate = base_deg * (vt / 12.0) * 0.95
    if omni:
        rate *= 0.9
    if drift_norm > 0.0:
        rate *= max(0.6, 1.0 - 0.25 * drift_norm)
    return max(120.0, min(900.0, rate))

def _turn_accel_deg_s2(cfg):
    """Calculate turn acceleration."""
    vt = float(cfg["robot_physics"].get("volts_turn", 12.0))
    rate = turn_rate(cfg)
    t_acc = max(0.12, min(0.30, 0.20 * (12.0 / max(1e-6, vt)) ** 0.5))
    return rate / max(1e-6, t_acc)

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
        prev_pd = nodes[i - 1].get("path_to_next", {}) if i - 1 >= 0 else {}
        prev_swing = prev_pd.get("swing_vis")
        start_override = prev_pd.get("start_override")
        if prev_swing and prev_swing.get("end_pos") is not None:
            start_override = prev_swing.get("end_pos")
        prev_anchor = start_override if start_override is not None else prev

        off_in = get_node_offset_in(n, cfg, i)
        ghost_ang = n.get("offset_ghost_angle")
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

def _swing_segment(p0, p1, start_heading, swing_dir, cfg, reverse=False, drive_override=None):
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

    # Iteratively refine delta so final heading matches desired heading from arc end to target
    for _ in range(8):
        end_pos, center, r0, geom_delta_deg, R, d_rad = _swing_geom(delta_deg, dir_geom)
        desired = heading_from_points(end_pos, p1)
        if reverse:
            desired = (desired + 180.0) % 360.0
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
    
    # Node 0 pre-edge actions
    if display_nodes[0].get("reshape_toggle", False):
        reshape = not reshape
        segs.append({"type": "reshape", "T": 0.0, "pos": eff[0], "state": 2 if reshape else 1})
    
    for act in display_nodes[0].get("actions", []):
        t = act.get("type")
        if t in ("reshape", "geom"):
            reshape = not reshape
            segs.append({"type": "reshape", "T": 0.0, "pos": eff[0], "state": 2 if reshape else 1})
        elif t == "wait":
            segs.append({"type": "wait", "T": float(act.get("s", 0.0)),
                        "pos": eff[0], "heading": curr_heading, "role": "custom", "node_i": 0})
        elif t == "turn":
            tgt = float(act.get("deg", curr_heading)) % 360.0
            diff = _angle_diff_deg(curr_heading, tgt)
            Tturn = turn_time(diff, cfg)
            if Tturn > 0.0:
                segs.append({"type": "turn", "T": Tturn, "pos": eff[0],
                            "start_heading": curr_heading, "target_heading": tgt,
                            "role": "explicit", "node_i": 0})
            curr_heading = tgt
    
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
        drive_override_cmd = float(node_i.get("custom_lateral_ips")) if node_i.get("custom_lateral_ips") else None
        if drive_override_cmd is not None:
            drive_override_cmd = _clamp_cmd(drive_override_cmd)
        if drive_override_cmd is not None:
            drive_override_cmd = _clamp_cmd(drive_override_cmd * prof_scale)
            drive_override = _cmd_to_ips(drive_override_cmd, cfg)
        else:
            drive_override = vmax_straight(cfg) * prof_scale
        turn_override = float(node_i.get("custom_turn_dps")) if node_i.get("custom_turn_dps") else None
        turn_speed_override = (turn_override * prof_scale) if turn_override is not None else (turn_rate(cfg) * prof_scale if prof_scale != 1.0 else None)
        if node_i.get("reverse", False):
            reverse = not reverse
        reverse_after_swing = bool(node_i.get("reverse_after_swing"))
        next_is_swing = display_nodes[i + 1].get("turn_mode") == "swing" if i + 1 < n else False
        
        if node_i.get("reshape_toggle", False) and i != 0:
            reshape = not reshape
            segs.append({"type": "reshape", "T": 0.0, "pos": p0, "state": 2 if reshape else 1})
        
        # Pre-edge actions
        for act in node_i.get("actions_out", node_i.get("actions", [])):
            t = act.get("type")
            if t == "wait":
                segs.append({"type": "wait", "T": float(act.get("s", 0.0)),
                            "pos": start_anchor, "heading": curr_heading, "role": "custom", "node_i": i})
            elif t in ("reshape", "geom"):
                reshape = not reshape
                segs.append({"type": "reshape", "T": 0.0, "pos": start_anchor, "state": 2 if reshape else 1})
            elif t == "turn":
                tgt = float(act.get("deg", curr_heading)) % 360.0
                diff = _angle_diff_deg(curr_heading, tgt)
                Tturn = turn_time(diff, cfg, rate_override=turn_speed_override)
                segs.append({"type": "turn", "T": Tturn, "pos": start_anchor,
                            "start_heading": curr_heading, "target_heading": tgt,
                            "role": "explicit", "node_i": i, "turn_speed_dps": turn_speed_override})
                curr_heading = tgt
        
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
                base_la_in = path_data.get("lookahead_in_override", base_la_in_global)
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
                if abs(diff_to_start) > 15.0:
                    Tface = turn_time(diff_to_start, cfg, rate_override=turn_speed_override)
                    if Tface > 0.0:
                        segs.append({
                            "type": "turn",
                            "T": Tface,
                            "pos": p0,
                            "start_heading": curr_heading,
                            "target_heading": start_heading,
                            "role": "face_path",
                            "edge_i": i,
                            "turn_speed_dps": turn_speed_override
                        })
                    curr_heading = start_heading
                else:
                    # Assume we simply roll into the path with existing heading
                    curr_heading = start_heading
                
                # Calculate path length/time with curvature + resampling
                speed_mult = float(path_data.get("speed_mult", 1.0)) * prof_scale
                min_override = _clamp_cmd(path_data.get("min_speed_ips")) if path_data.get("min_speed_ips") is not None else None
                max_override = _clamp_cmd(path_data.get("max_speed_ips")) if path_data.get("max_speed_ips") is not None else None
                L_in, Tmove, resampled, curvatures, speeds, speeds_raw = path_time_with_curvature(  # type: ignore[misc]
                    path_points, cfg, speed_mult=speed_mult,
                    min_speed_override=min_override,
                    max_speed_override=max_override if drive_override_cmd is None else drive_override_cmd
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
                    "min_speed_cmd": min_override if min_override is not None else cfg.get("path_config", {}).get("min_speed_ips", 0.0),
                    "max_speed_cmd": max_override if max_override is not None else (drive_override_cmd if drive_override_cmd is not None else cfg.get("path_config", {}).get("max_speed_ips", 127.0)),
                    "facing": end_heading,
                    "i0": i,
                    "i1": i + 1,
                    "reverse": reverse,
                    "skip_buffer": _has_custom_waits_at_node(display_nodes[i+1]) or next_is_swing,
                    "segment_idx": i,  # For path file export
                    "drive_speed_cmd": drive_override_cmd,
                    "drive_speed_ips": drive_override,
                    "lookahead_px": seg_lookahead_px
                })
                
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
                swing_res = _swing_segment(start_pt, p1, swing_start_heading, swing_dir, cfg, reverse, drive_override)
                if swing_res is not None:
                    swing_seg, swing_end, swing_heading, swing_vis = swing_res
                    swing_seg["i0"] = i
                    swing_seg["i1"] = i + 1
                    swing_seg["profile_override"] = profile_override
                    swing_seg["drive_speed_cmd"] = drive_override_cmd
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
                    lead_in = max(0.0, float(pose_lead))
                except Exception:
                    lead_in = 0.0
                dist_px = math.hypot(p1[0] - start_pt[0], p1[1] - start_pt[1])
                if dist_px <= 1e-6:
                    dist_px = 1.0
                # Clamp lead to [0.1, 1.0] as a fraction of segment length; higher = mellower curve
                try:
                    lead_clamped = max(0.1, min(1.0, lead_in))
                except Exception:
                    lead_clamped = 0.1
                lead_px = dist_px * lead_clamped
                # Boomerang-style control points: keep a straight run before the turn, then bend over the lead distance
                straight_run = max(0.0, dist_px - lead_px)
                curve_run = max(0.0, lead_px)
                # Force the straight leg to stay on the line toward the target (LemLib boomerang style)
                dir_to_target = (p1[0] - start_pt[0], p1[1] - start_pt[1])
                mag_dir = math.hypot(dir_to_target[0], dir_to_target[1])
                if mag_dir > 1e-9:
                    vx0, vy0 = dir_to_target[0] / mag_dir, dir_to_target[1] / mag_dir
                else:
                    vx0, vy0 = _heading_vec_px(curr_heading)
                # End direction toward desired heading with gentle easing
                vx_end_h, vy_end_h = _heading_vec_px(pose_heading)
                blend = min(1.0, max(0.0, 0.35 + 0.65 * lead_clamped))
                vx_end = (1.0 - blend) * vx0 + blend * vx_end_h
                vy_end = (1.0 - blend) * vy0 + blend * vy_end_h
                mag_end = math.hypot(vx_end, vy_end)
                if mag_end > 1e-9:
                    vx_end /= mag_end
                    vy_end /= mag_end
                else:
                    vx_end, vy_end = vx_end_h, vy_end_h

                cp1 = (start_pt[0] + vx0 * straight_run, start_pt[1] + vy0 * straight_run)
                # Single mid control to soften curvature without bending back
                mid_blend = min(1.0, 0.25 + 0.5 * lead_clamped)
                vx_mid = (1.0 - mid_blend) * vx0 + mid_blend * vx_end_h
                vy_mid = (1.0 - mid_blend) * vy0 + mid_blend * vy_end_h
                mag_mid = math.hypot(vx_mid, vy_mid)
                if mag_mid > 1e-9:
                    vx_mid /= mag_mid
                    vy_mid /= mag_mid
                else:
                    vx_mid, vy_mid = vx_end_h, vy_end_h
                cp2 = (start_pt[0] + vx_mid * (straight_run + 0.5 * curve_run),
                       start_pt[1] + vy_mid * (straight_run + 0.5 * curve_run))
                cp3 = (p1[0] - vx_end * (0.6 * curve_run), p1[1] - vy_end * (0.6 * curve_run))
                cps = [start_pt, cp1, cp2, cp3, p1]
                path_points = generate_bezier_path(cps, num_samples=50)
                speed_mult = 1.0 * prof_scale
                min_override = None
                max_override = drive_override_cmd
                L_in, Tmove, resampled, curvatures, speeds, speeds_raw = path_time_with_curvature(  # type: ignore[misc]
                    path_points, cfg, speed_mult=speed_mult,
                    min_speed_override=min_override,
                    max_speed_override=max_override
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
                    "min_speed_cmd": cfg.get("path_config", {}).get("min_speed_ips", 0.0),
                    "max_speed_cmd": drive_override_cmd if drive_override_cmd is not None else cfg.get("path_config", {}).get("max_speed_ips", 127.0),
                    "facing": pose_heading,
                    "i0": i,
                    "i1": i + 1,
                    "reverse": reverse,
                    "skip_buffer": _has_custom_waits_at_node(display_nodes[i+1]) or next_is_swing,
                    "segment_idx": i,
                    "drive_speed_cmd": drive_override_cmd,
                    "drive_speed_ips": drive_override,
                    "lookahead_px": seg_lookahead_px,
                    "move_to_pose": True,
                    "pose_heading": pose_heading,
                    "pose_lead_in": pose_lead
                })
                path_data["pose_preview_points"] = resampled
                path_data["start_override"] = start_pt
                curr_heading = pose_heading
            else:
                diff_to_facing = _angle_diff_deg(curr_heading, facing)
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
                        "turn_speed_dps": turn_speed_override
                    })
                curr_heading = facing

                # Move along straight segment
                Tmove = _move_total_time(L_in, cfg, v_override=drive_override)
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
                    "drive_speed_ips": drive_override
                })
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


def sample_path_position(t, path_points, total_time, cfg, lookahead_px=None, use_pursuit=False, path_speeds=None, current_pos=None):
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
    total_px = _path_length_px(path_points)
    if total_px <= 1e-9:
        return path_points[-1], calculate_path_heading(path_points, len(path_points) - 1), None

    # If we have per-point speeds, use them to map time -> distance
    speeds = list(path_speeds) if path_speeds and len(path_speeds) == len(path_points) else None
    seg_lengths_px = []
    for i in range(len(path_points) - 1):
        seg_lengths_px.append(math.hypot(path_points[i+1][0] - path_points[i][0],
                                         path_points[i+1][1] - path_points[i][1]))
    if speeds and total_time > 1e-9:
        seg_times = []
        total_T = 0.0
        for i, ds_px in enumerate(seg_lengths_px):
            v0 = max(1e-3, speeds[i])
            v1 = max(1e-3, speeds[i+1])
            v = 0.5 * (v0 + v1)
            dt = (ds_px / PPI) / v
            seg_times.append(dt)
            total_T += dt
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
        pos, seg_idx = path_point_at_distance(path_points, s_px)
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
            found, _ = path_point_at_distance(path_points, s_px + la)
        look_pt = found
        if look_pt:
            dx = look_pt[0] - center[0]
            dy = look_pt[1] - center[1]
            heading = (math.degrees(math.atan2(-dy, dx)) + 360.0) % 360.0
    
    return pos, heading, look_pt


def path_point_at_distance(path_points, dist_px):
    """Return point and segment index along path at given distance (px)."""
    if not path_points:
        return (0.0, 0.0), 0
    if dist_px <= 0.0:
        return path_points[0], 0
    
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
