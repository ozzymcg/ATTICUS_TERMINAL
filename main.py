# terminal/main.py
import os, math, json, copy, re
from datetime import datetime
from typing import Any, Callable

# Ensure SDL picks a usable video driver (helps when run from terminals that default to headless)
if os.name == "nt" and not os.environ.get("SDL_VIDEODRIVER"):
    os.environ["SDL_VIDEODRIVER"] = "windows"

import pygame
import tkinter as tk
from tkinter import simpledialog, ttk, messagebox, filedialog

from mod.config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, GRID_SIZE_PX, BG_COLOR, NODE_COLOR, TEXT_COLOR, PPI,
    save_config, reload_cfg, auto_lookahead_in, DEFAULT_CONFIG
)
from mod.geom import (
    convert_heading_input, interpret_input_angle, heading_from_points,
    coords_str, constrain_to_8dirs, approach_unit
)
from mod.sim import (
    snap_to_grid, compile_timeline,
    sample_turn_heading_trap, sample_move_profile, vmax_straight, accel_straight, _turn_accel_deg_s2,
    correct_nodes_inbounds, sample_path_position, path_time_with_curvature, turn_rate
)
from mod.draw import (
    draw_grid, draw_robot, draw_chevron, draw_nodes,
    draw_hover_box, draw_time_label, draw_constraint_visual,
    draw_field_objects, draw_geometry_borders, draw_follow_geometry,
    draw_curved_path, swing_arc_points, draw_path_control_points, draw_path_edit_overlay,
    draw_label
)
from mod.storage import save_nodes, load_nodes, compile_log
from mod.util import (
    pros_convert_inches, build_compile_header, apply_tbuffer,
    snapshot as util_snapshot, push_undo_prev as util_push_undo_prev,
    get_node_offset_in
)
from mod.codegen import build_export_lines, pick_profile, SETTLE_BASE, VOLTAGE_SHAPES
from mod import ui


# Try to import path utilities - these are NEW modules
try:
    from mod.pathing import generate_bezier_path, calculate_path_heading, resample_path_uniform
    from mod.pathing import export_lemlib_path, generate_path_asset_name
    PATH_FEATURES_AVAILABLE = True
except ImportError:
    PATH_FEATURES_AVAILABLE = False
    print("Warning: Path utilities not available. Path editing disabled.")
    # Define dummy functions
    def generate_bezier_path(points, num_samples=50):
        return points
    def calculate_path_heading(points, index):
        return 0.0
    def export_lemlib_path(points, name, cfg):
        pass
    def generate_path_asset_name(routine, idx):
        return f"path_{idx}"

# Type helpers for linting
generate_bezier_path: Callable[[Any, int], Any]
calculate_path_heading: Callable[[Any, Any], float]
export_lemlib_path: Callable[..., Any]
generate_path_asset_name: Callable[..., str]

APP_TITLE = "THE ATTICUS TERMINAL"

CONTROLS = [
    ("LeftClick", "Select nearest node or create new node\n(snaps to grid if enabled)."),
    ("LeftClick + Drag", "Drag selected node to new position."),
    ("Ctrl+Drag Node", "Constrain drag to 8 global\ndirections from press point"),
    ("Shift+Click Line", "Insert a node along a straight segment."),
    ("RightClick", "Node commands: turn X, wait Y,\n offset Z, reshape"),
    ("M", "Add/Edit a mechanism marker on the hovered segment."),
    ("P", "Toggle path edit mode for \n hovered/selected node"),
    ("T", "Toggle path look-ahead simulation \n(Pure Pursuit)"),
    ("SPACE / CTRL+SPACE", "Start-Pause / Reset"),
    ("Q", "Toggle Snap-to-Grid"),
    ("F", "Cycle offset: none → 1 → 2"),
    ("R / G", "Toggle Reverse / Reshape Geometry at node"),
    ("Hold Backspace/Delete + Click", "Delete clicked node(s)"),
    ("Delete / Backspace", "Delete currently selected node\n(node 0 is protected)."),
    ("CTRL+SHIFT+Backspace/Delete", "Reset field to node 0"),
    ("CTRL+Z / CTRL+Y", "Undo / Redo"),
    ("S / L", "Save / Load routine"),
    ("C", "Export generated program"),
    ("O", "Toggle Output window"),
]

# ---------------- pygame init ----------------
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption(APP_TITLE)
try:
    icon_path = os.path.join(os.path.dirname(__file__), 'ATTICUS.png')
    icon = pygame.image.load(icon_path)
    pygame.display.set_icon(icon)
except Exception: 
    pass
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)
font_small = pygame.font.SysFont(None, 16)

# Drawing colors
TEMPLATE_BG = "#f2f4f7"

CFG = reload_cfg()

# ---------------- app state ----------------
initial_state = {"position": (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2), "heading": 0.0}
try:
    saved_deg = float(CFG.get("initial_heading_deg", 0.0))
    initial_state["heading"] = interpret_input_angle(saved_deg)
except Exception:
    initial_state["heading"] = 0.0

display_nodes = [{"pos": initial_state["position"]}]
robot_pos = initial_state["position"]
robot_heading = initial_state["heading"]
reshape_live = False

moving = False
paused = False
timeline = []
seg_i = 0
fps = 60
t_local = 0.0
last_logged_seg = -1
show_chevron = False

snap_enabled = True
selected_idx = None
SELECTION_RADIUS_PX = int(GRID_SIZE_PX // 4)
dragging = False
constrain_active = False
constrain_origin = None
offset_dragging_idx = None
offset_drag_prev = None

undo_stack, redo_stack = [], []
last_snapshot = None
last_path_sig = None
history_freeze = False
total_estimate_s = 0.0
log_lines = []

# Path sim parameters
path_lookahead_enabled = bool(CFG.get("path_config", {}).get("simulate_pursuit", 1))
path_lookahead_px = float(CFG.get("path_config", {}).get("lookahead_in", auto_lookahead_in(CFG))) * PPI
last_lookahead_point = None
last_lookahead_radius = path_lookahead_px
last_heading_target = None
path_draw_cache = {}  # cache for gradient-drawn path segments
insert_preview = None  # (segment_idx, (x,y), t) while shift-inserting

# Path editing state
path_edit_mode = False
path_edit_segment_idx = None  # Which segment (node index) is being edited
path_control_points = []  # Current control points being edited
selected_control_point = None  # Index of selected control point
dragging_control_point = False
PATH_CONTROL_RADIUS = 8  # Click detection radius for control points
EDGE_MARKER_RADIUS = 7
EDGE_MARKER_HIT_RADIUS = 10
edge_marker_drag = None  # {"seg_idx": int, "event_idx": int, "prev": snapshot}

def _edge_events_sig(node):
    events = node.get("edge_events", [])
    if not isinstance(events, list):
        return ()
    sig = []
    for ev in events:
        if not isinstance(ev, dict):
            continue
        try:
            t_val = round(float(ev.get("t", 0.0)), 4)
        except Exception:
            t_val = 0.0
        enabled = bool(ev.get("enabled", True))
        acts = ev.get("actions", [])
        act_sig = []
        if isinstance(acts, list):
            for act in acts:
                if not isinstance(act, dict):
                    continue
                kind = str(act.get("kind", "")).lower()
                if kind == "preset":
                    act_sig.append((
                        "preset",
                        str(act.get("name", "")).strip().lower(),
                        str(act.get("state", "")).strip().lower(),
                        str(act.get("value", "")).strip()
                    ))
                else:
                    act_sig.append((
                        "code",
                        str(act.get("code", "")).strip()
                    ))
        sig.append((t_val, enabled, tuple(act_sig)))
    return tuple(sig)

def path_sig_for_node(node):
    """Small signature tuple capturing path state for change detection."""
    pd = node.get("path_to_next")
    if not pd:
        chain_flag = bool(node.get("chain_through", False) or node.get("chain", False))
        chain_val = node.get("chain_looseness")
        try:
            chain_val = None if chain_val is None else round(float(chain_val), 4)
        except Exception:
            chain_val = None
        return (False, (), _edge_events_sig(node), chain_flag, chain_val)
    cps = pd.get("control_points") or []
    cps_sig = tuple((int(round(cp[0])), int(round(cp[1]))) for cp in cps)
    chain_flag = bool(node.get("chain_through", False) or node.get("chain", False))
    chain_val = node.get("chain_looseness")
    try:
        chain_val = None if chain_val is None else round(float(chain_val), 4)
    except Exception:
        chain_val = None
    return (
        bool(pd.get("use_path", False)),
        cps_sig,
        _edge_events_sig(node),
        chain_flag,
        chain_val
    )

def _path_speed_override_cmd(path_data, key):
    if not path_data:
        return None
    val = path_data.get(key)
    if val is None:
        val = path_data.get(key.replace("_cmd", "_ips"))
    return val

def _path_speed_cmd(path_data, cfg, key, default):
    val = _path_speed_override_cmd(path_data, key)
    if val is None:
        pcfg = cfg.get("path_config", {})
        val = pcfg.get(key)
        if val is None:
            val = pcfg.get(key.replace("_cmd", "_ips"))
    return default if val is None else val

def _node_lateral_cmd(node):
    val = node.get("custom_lateral_cmd")
    if val is None:
        val = node.get("custom_lateral_ips")
    return val

def _clamp_cmd(val, default=0.0):
    try:
        return max(0.0, min(127.0, float(val)))
    except Exception:
        return float(default)

def _normalize_cmd_range(min_val, max_val):
    min_cmd = _clamp_cmd(min_val)
    max_cmd = _clamp_cmd(max_val, default=127.0)
    if min_cmd > max_cmd:
        min_cmd, max_cmd = max_cmd, min_cmd
    return min_cmd, max_cmd

def _apply_profile_override_for_speed_overrides(node):
    lat_cmd = _node_lateral_cmd(node)
    turn_dps = node.get("custom_turn_dps")
    has_lat = False
    has_turn = False
    try:
        has_lat = lat_cmd is not None and float(lat_cmd) > 0.0
    except Exception:
        has_lat = False
    try:
        has_turn = turn_dps is not None and float(turn_dps) > 0.0
    except Exception:
        has_turn = False
    if has_lat or has_turn:
        node["profile_override"] = "custom"
    elif node.get("profile_override") == "custom":
        node.pop("profile_override", None)

def normalize_speed_units(nodes):
    for node in nodes:
        if node.get("custom_lateral_cmd") is None and node.get("custom_lateral_ips") is not None:
            node["custom_lateral_cmd"] = node.pop("custom_lateral_ips")
        else:
            node.pop("custom_lateral_ips", None)
        pd = node.get("path_to_next")
        if isinstance(pd, dict):
            if "min_speed_cmd" not in pd and pd.get("min_speed_ips") is not None:
                pd["min_speed_cmd"] = pd.pop("min_speed_ips")
            else:
                pd.pop("min_speed_ips", None)
            if "max_speed_cmd" not in pd and pd.get("max_speed_ips") is not None:
                pd["max_speed_cmd"] = pd.pop("max_speed_ips")
            else:
                pd.pop("max_speed_ips", None)
            min_present = "min_speed_cmd" in pd
            max_present = "max_speed_cmd" in pd
            if min_present:
                pd["min_speed_cmd"] = _clamp_cmd(pd.get("min_speed_cmd", 0.0))
            if max_present:
                pd["max_speed_cmd"] = _clamp_cmd(pd.get("max_speed_cmd", 127.0), default=127.0)
            if min_present and max_present and pd["min_speed_cmd"] > pd["max_speed_cmd"]:
                pd["min_speed_cmd"], pd["max_speed_cmd"] = pd["max_speed_cmd"], pd["min_speed_cmd"]
        _apply_profile_override_for_speed_overrides(node)

def path_shade_key(idx, node):
    """Build cache key for shaded path rendering."""
    pd = node.get("path_to_next", {})
    curv_gain = float(CFG.get("robot_physics", {}).get("curvature_gain", CFG.get("path_config", {}).get("curvature_gain", 0.05)))
    vmin = _path_speed_cmd(pd, CFG, "min_speed_cmd", 0.0)
    vmax = _path_speed_cmd(pd, CFG, "max_speed_cmd", 127.0)
    la_override = pd.get("lookahead_in_override")
    return (idx, path_sig_for_node(node), round(curv_gain, 6), round(float(vmin), 3), round(float(vmax), 3), None if la_override is None else round(float(la_override), 3))

def build_shaded_segments(node_idx):
    """Compute gradient-colored path segments for rendering."""
    node = display_nodes[node_idx]
    pd = node.get("path_to_next", {})
    cps = list(pd.get("control_points", []))
    if len(cps) < 2:
        return None
    # Anchor endpoints
    cps[0] = effective_node_pos(node_idx)
    cps[-1] = effective_node_pos(node_idx + 1)
    if not PATH_FEATURES_AVAILABLE:
        return [("plain", cps)]
    try:
        smooth_path = generate_bezier_path(cps, num_samples=50)
        if len(smooth_path) < 2:
            return None
        min_override = _path_speed_override_cmd(pd, "min_speed_cmd")
        max_override = _path_speed_override_cmd(pd, "max_speed_cmd")
        # Shading driven by speeds/curvature
        _, _, sampled, curvs, speeds, _ = path_time_with_curvature(
            smooth_path, CFG,
            min_speed_override=min_override,
            max_speed_override=max_override
        )
        if not speeds or len(sampled) != len(speeds):
            return [("plain", sampled)]
        v_min = min(speeds); v_max = max(speeds)
        curv_gain = float(CFG.get("robot_physics", {}).get("curvature_gain", CFG.get("path_config", {}).get("curvature_gain", 0.05)))
        segs = []
        for j in range(len(sampled) - 1):
            v = speeds[j]
            cv = curvs[j] if curvs else 0.0
            t = 0.0 if v_max <= v_min else (v - v_min) / (v_max - v_min)
            curv_in = abs(cv) * PPI
            c = min(1.0, curv_in * curv_gain * 25.0)
            base_r, base_g, base_b = 180, 220, 255
            dark_r, dark_g, dark_b = 10, 60, 140
            mix = min(1.0, max(0.0, t * 0.8 + c * 0.2))
            r = int(base_r + (dark_r - base_r) * mix)
            g = int(base_g + (dark_g - base_g) * mix)
            b = int(base_b + (dark_b - base_b) * mix)
            segs.append((sampled[j], sampled[j+1], (r, g, b)))
        return segs
    except Exception:
        return [("plain", cps)]

def _tangent_into_node(idx):
    """Return vector of tangent coming into node idx from previous segment."""
    if idx <= 0 or idx >= len(display_nodes):
        return None
    prev_seg = display_nodes[idx - 1].get("path_to_next", {})
    if prev_seg.get("use_path") and prev_seg.get("control_points"):
        cps = prev_seg["control_points"]
        if len(cps) >= 2:
            return (cps[-1][0] - cps[-2][0], cps[-1][1] - cps[-2][1])
    # fallback straight line
    p_prev = effective_node_pos(idx - 1)
    p_here = effective_node_pos(idx)
    return (p_here[0] - p_prev[0], p_here[1] - p_prev[1])

def _tangent_out_of_node(idx):
    """Return vector of tangent leaving node idx to next segment."""
    if idx < 0 or idx >= len(display_nodes) - 1:
        return None
    next_seg = display_nodes[idx].get("path_to_next", {})
    if next_seg.get("use_path") and next_seg.get("control_points"):
        cps = next_seg["control_points"]
        if len(cps) >= 2:
            return (cps[1][0] - cps[0][0], cps[1][1] - cps[0][1])
    # fallback straight line
    p_here = effective_node_pos(idx)
    p_next = effective_node_pos(idx + 1)
    return (p_next[0] - p_here[0], p_next[1] - p_here[1])

def sync_path_endpoints_for_node(idx):
    """Ensure any paths touching idx stay anchored to their nodes."""
    for seg_idx in (idx - 1, idx):
        if seg_idx < 0 or seg_idx >= len(display_nodes) - 1:
            continue
        node = display_nodes[seg_idx]
        path_data = node.get("path_to_next")
        if not path_data or not path_data.get("use_path") or not path_data.get("control_points"):
            continue
        cps = list(path_data["control_points"])
        if len(cps) < 2:
            continue
        start = effective_node_pos(seg_idx)
        end = effective_node_pos(seg_idx + 1)
        changed = False
        if cps[0] != start:
            cps[0] = start
            changed = True
        if cps[-1] != end:
            cps[-1] = end
            changed = True
        if changed:
            node.setdefault("path_to_next", {})["control_points"] = cps

def sync_all_path_endpoints():
    """Anchor all path endpoints to their owning nodes."""
    for i in range(max(0, len(display_nodes) - 1)):
        sync_path_endpoints_for_node(i)
        # If the segment is no longer a path, clear ghost angle on the endpoint so offset snaps inline
        pd = display_nodes[i].get("path_to_next", {})
        if not pd.get("use_path", False) and (i + 1) < len(display_nodes):
            display_nodes[i + 1].pop("offset_ghost_angle", None)
    # Path geometry changed; invalidate cached gradients
    path_draw_cache.clear()

def annotate_motion_profiles(timeline_list):
    """Attach motion profile tags to nodes for hover when advanced motion is enabled."""
    for node in display_nodes:
        node.pop("profile_info", None)
    adv_enabled = bool(CFG.get("robot_physics", {}).get("advanced_motion", 0))
    if not adv_enabled:
        return
    try:
        default_profile = str(CFG.get("codegen", {}).get("opts", {}).get("jar_profile_default", "normal"))
    except Exception:
        default_profile = "normal"
    for seg in timeline_list:
        st = seg.get("type")
        node_idx = seg.get("i0")
        if node_idx is None:
            node_idx = seg.get("edge_i", seg.get("node_i"))
        if node_idx is None or node_idx < 0 or node_idx >= len(display_nodes):
            continue
        info = display_nodes[node_idx].setdefault("profile_info", {})
        if st in ("move", "pose"):
            L_in = seg.get("length_in")
            if L_in is None and "p0" in seg and "p1" in seg:
                L_in = math.hypot(seg["p1"][0]-seg["p0"][0], seg["p1"][1]-seg["p0"][1]) / PPI
            chosen_prof = seg.get("profile_override") or pick_profile("drive", abs(L_in or 0.0), default_profile, cfg=CFG)
            info["drive_profile"] = chosen_prof
        elif st == "path":
            L_in = seg.get("path_length_in")
            if L_in is None:
                pts = seg.get("path_points") or []
                if pts and len(pts) > 1:
                    L_in = 0.0
                    for j in range(len(pts) - 1):
                        L_in += math.hypot(pts[j+1][0] - pts[j][0], pts[j+1][1] - pts[j][1]) / PPI
                elif "p0" in seg and "p1" in seg:
                    L_in = math.hypot(seg["p1"][0]-seg["p0"][0], seg["p1"][1]-seg["p0"][1]) / PPI
            chosen_prof = seg.get("profile_override") or pick_profile("drive", abs(L_in or 0.0), default_profile, cfg=CFG)
            info["drive_profile"] = chosen_prof
        elif st == "turn":
            delta = ((seg.get("target_heading", 0.0) - seg.get("start_heading", 0.0) + 180.0) % 360.0) - 180.0
            chosen_prof = seg.get("profile_override") or pick_profile("turn", abs(delta), default_profile, cfg=CFG)
            info["turn_profile"] = chosen_prof
        elif st == "swing":
            delta = seg.get("delta_heading")
            if delta is None:
                delta = ((seg.get("target_heading", 0.0) - seg.get("start_heading", 0.0) + 180.0) % 360.0) - 180.0
            chosen_prof = seg.get("profile_override") or pick_profile("swing", abs(delta), default_profile, cfg=CFG)
            info["swing_profile"] = chosen_prof

def build_timeline_with_buffers():
    """Generate timeline with buffers, ensuring paths are synced first."""
    sync_all_path_endpoints()
    tl = apply_tbuffer(CFG, compile_timeline(display_nodes, CFG, initial_state["heading"], fps))
    annotate_motion_profiles(tl)
    return tl

def compute_total_from_timeline(tl):
    """Sum timeline duration, excluding a trailing buffer wait."""
    total = sum(seg.get("T", 0.0) for seg in tl)
    if tl and tl[-1].get("type") == "wait" and tl[-1].get("role") == "buffer":
        total -= tl[-1].get("T", 0.0)
        if total < 0.0:
            total = 0.0
    return total

def remove_path_leading_to(idx):
    """Remove any stored path that ends at idx (path_to_next on idx-1)."""
    if idx <= 0:
        return
    prev_idx = idx - 1
    if 0 <= prev_idx < len(display_nodes):
        display_nodes[prev_idx].pop("path_to_next", None)
        if path_edit_mode and path_edit_segment_idx == prev_idx:
            exit_path_edit_mode()

def delete_node_at(idx):
    """Delete node at index and clean up adjacent paths."""
    global selected_idx, dragging, last_snapshot, last_path_sig, total_estimate_s
    global moving, paused, show_chevron, timeline, seg_i, t_local, last_logged_seg
    if idx <= 0 or idx >= len(display_nodes):
        return
    prev_state = util_snapshot(display_nodes, robot_pos, robot_heading)
    remove_path_leading_to(idx)
    if path_edit_mode and path_edit_segment_idx == idx:
        exit_path_edit_mode()
    try:
        del display_nodes[idx]
    except Exception:
        return
    util_push_undo_prev(undo_stack, prev_state)
    moving = False; paused = False; show_chevron = False
    timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
    selected_idx = None; dragging = False
    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
    last_path_sig = None
    total_estimate_s = compute_total_estimate_s()

def _arrival_heading_for_offset(prev_pd, prev_anchor, target_pos):
    prev_swing = prev_pd.get("swing_vis") if isinstance(prev_pd, dict) else None
    if prev_swing and prev_swing.get("end_pos") is not None:
        return heading_from_points(prev_swing.get("end_pos"), target_pos)
    if prev_swing and prev_swing.get("target_heading") is not None:
        return prev_swing.get("target_heading")
    has_curve = False
    if isinstance(prev_pd, dict):
        has_curve = bool(prev_pd.get("use_path", False) or prev_pd.get("pose_preview_points"))
    if has_curve:
        pts = list(prev_pd.get("path_points") or prev_pd.get("pose_preview_points") or [])
        if not pts:
            cps = list(prev_pd.get("control_points", []))
            if len(cps) >= 2:
                cps[0] = prev_anchor
                cps[-1] = target_pos
                pts = generate_bezier_path(cps, num_samples=20)
        if pts:
            try:
                return calculate_path_heading(pts, len(pts) - 1)
            except Exception:
                return None
        try:
            return calculate_path_heading([prev_anchor, target_pos], 1)
        except Exception:
            return None
    return None


def effective_node_pos(idx):
    """Return node position adjusted for offsets (same logic as motion)."""
    if idx <= 0:
        return display_nodes[0]["pos"]
    node = display_nodes[idx]
    prev = display_nodes[idx - 1]["pos"]
    prev_node = display_nodes[idx - 1]
    off_in = get_node_offset_in(node, CFG, idx)
    ghost_ang = node.get("offset_ghost_angle")
    prev_pd = display_nodes[idx - 1].get("path_to_next", {}) if idx - 1 >= 0 else {}
    prev_swing = prev_pd.get("swing_vis")
    use_radial = prev_pd.get("use_path", False) or bool(prev_swing) or bool(prev_pd.get("pose_preview_points"))
    if off_in != 0.0 and ghost_ang is None and prev_node.get("move_to_pose"):
        pose_h = prev_node.get("pose_heading_deg")
        if pose_h is None:
            pose_h = heading_from_points(prev, node["pos"])
        if prev_node.get("reverse", False):
            pose_h = (pose_h + 180.0) % 360.0
        ghost_ang = (pose_h + 180.0) % 360.0
        node["offset_ghost_angle"] = ghost_ang
    if use_radial and off_in != 0.0:
        try:
            prev_eff = display_nodes[idx - 1]["pos"] if idx - 1 >= 0 else prev
        except Exception:
            prev_eff = prev
        arrival_heading = _arrival_heading_for_offset(prev_pd, prev_eff, node["pos"])
        if arrival_heading is not None:
            ghost_ang = arrival_heading
        elif ghost_ang is None:
            ghost_ang = heading_from_points(prev_eff, node["pos"])
        node["offset_ghost_angle"] = ghost_ang
        if ghost_ang is not None:
            rad = math.radians(ghost_ang)
            return (
                node["pos"][0] - math.cos(rad) * off_in * PPI,
                node["pos"][1] + math.sin(rad) * off_in * PPI
            )
    else:
        # Straight segments ignore radial; clear ghost if present
        node.pop("offset_ghost_angle", None)
    if off_in == 0.0:
        return node["pos"]
    ux, uy = approach_unit(prev, node["pos"])
    return (node["pos"][0] - ux * off_in * PPI, node["pos"][1] - uy * off_in * PPI)

# ---------------- compile helpers ----------------
def compute_total_estimate_s():
    """Compute total routine time estimate."""
    tl = build_timeline_with_buffers()
    return compute_total_from_timeline(tl)


def clamp_heading_rate(current_h, target_h, cfg, dt):
    """Limit heading change per frame to respect physical turn constraints."""
    max_rate = turn_rate(cfg)  # deg/s from physics
    max_step = max_rate * dt
    d = ((target_h - current_h + 180.0) % 360.0) - 180.0
    # Snap tiny errors to eliminate jitter on very small turns (animation only)
    if abs(d) < max(0.5, max_step):
        return (target_h + 360.0) % 360.0
    if abs(d) <= max_step:
        return (target_h + 360.0) % 360.0
    step = max(-max_step, min(max_step, d))
    return (current_h + step + 360.0) % 360.0


def smooth_angle(prev_h, target_h, alpha=0.35):
    """Blend angles on shortest arc."""
    if prev_h is None:
        return target_h % 360.0
    d = ((target_h - prev_h + 180.0) % 360.0) - 180.0
    return (prev_h + d * alpha + 360.0) % 360.0

def _segment_insert_preview(pos):
    """Return (seg_idx, projected_pos, t) for shift-insert, or None."""
    if len(display_nodes) < 2:
        return None
    thresh = SELECTION_RADIUS_PX * 1.6
    best_d2 = thresh * thresh
    best = None
    for i in range(len(display_nodes) - 1):
        pd = display_nodes[i].get("path_to_next", {}) or {}
        # Skip movetopose segments (pose preview points)
        if pd.get("pose_preview_points") and not pd.get("use_path", False):
            continue
        pts = _edge_polyline(i)
        if len(pts) < 2:
            continue
        t, proj, d2 = _polyline_nearest(pos, pts)
        if d2 < best_d2:
            best_d2 = d2
            best = (i, proj, t)
    return best

def _split_edge_events(events, t_split):
    if not isinstance(events, list):
        return None, None
    t_split = max(0.0, min(1.0, float(t_split)))
    if t_split <= 1e-6 or t_split >= 1.0 - 1e-6:
        return list(events), []
    left = []
    right = []
    for ev in events:
        if not isinstance(ev, dict):
            continue
        t = ev.get("t", 0.0)
        try:
            t = float(t)
        except Exception:
            t = 0.0
        if t <= t_split:
            new_ev = dict(ev)
            new_ev["t"] = 0.0 if t_split <= 1e-6 else max(0.0, min(1.0, t / t_split))
            left.append(new_ev)
        else:
            new_ev = dict(ev)
            denom = max(1e-6, 1.0 - t_split)
            new_ev["t"] = max(0.0, min(1.0, (t - t_split) / denom))
            right.append(new_ev)
    return left, right

def _split_path_control_points(cps, insert_pt):
    if not cps or len(cps) < 2:
        return [insert_pt], [insert_pt]
    best_i = 0
    best_d2 = float("inf")
    for i in range(len(cps) - 1):
        p0 = cps[i]
        p1 = cps[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= 1e-9:
            continue
        t_seg = ((insert_pt[0] - p0[0]) * dx + (insert_pt[1] - p0[1]) * dy) / seg_len2
        t_seg = max(0.0, min(1.0, t_seg))
        proj = (p0[0] + t_seg * dx, p0[1] + t_seg * dy)
        d2 = (insert_pt[0] - proj[0]) ** 2 + (insert_pt[1] - proj[1]) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    left = list(cps[:best_i + 1]) + [insert_pt]
    right = [insert_pt] + list(cps[best_i + 1:])
    return left, right

def _edge_polyline(seg_idx):
    if seg_idx < 0 or seg_idx >= len(display_nodes) - 1:
        return []
    node = display_nodes[seg_idx]
    pd = node.get("path_to_next", {}) or {}
    start = effective_node_pos(seg_idx)
    end = effective_node_pos(seg_idx + 1)
    pts = []
    if pd.get("use_path", False) and pd.get("control_points"):
        cps = list(pd.get("control_points", []))
        if len(cps) >= 2:
            cps[0] = start
            cps[-1] = end
        pts = generate_bezier_path(cps, num_samples=60)
    elif pd.get("pose_preview_points"):
        pts = list(pd.get("pose_preview_points") or [])
        if pts:
            pts[0] = start
            pts[-1] = end
    elif pd.get("swing_vis"):
        arc_pts = swing_arc_points(pd.get("swing_vis")) or []
        pts = list(arc_pts) if arc_pts else [start]
        end_pos = pd.get("swing_vis", {}).get("end_pos")
        if end_pos is not None and (not pts or pts[-1] != end_pos):
            pts.append(end_pos)
        if pts and pts[-1] != end:
            pts.append(end)
    else:
        pts = [start, end]
    if len(pts) < 2:
        pts = [start, end]
    return pts

def _polyline_lengths(pts):
    seg_lens = []
    total = 0.0
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        seg = math.hypot(dx, dy)
        seg_lens.append(seg)
        total += seg
    return seg_lens, total

def _polyline_point_at(pts, t):
    if not pts:
        return (0.0, 0.0)
    t = max(0.0, min(1.0, float(t)))
    seg_lens, total = _polyline_lengths(pts)
    if total <= 1e-6:
        return pts[0]
    target = t * total
    accum = 0.0
    for i, seg_len in enumerate(seg_lens):
        if seg_len <= 1e-6:
            continue
        if target <= accum + seg_len:
            local = (target - accum) / seg_len
            p0 = pts[i]
            p1 = pts[i + 1]
            return (p0[0] + (p1[0] - p0[0]) * local, p0[1] + (p1[1] - p0[1]) * local)
        accum += seg_len
    return pts[-1]

def _polyline_nearest(pos, pts):
    if len(pts) < 2:
        return 0.0, (pts[0] if pts else (0.0, 0.0)), float("inf")
    best_d2 = float("inf")
    best_s = 0.0
    best_pt = pts[0]
    accum = 0.0
    for i in range(len(pts) - 1):
        p0 = pts[i]
        p1 = pts[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= 1e-9:
            continue
        t_seg = ((pos[0] - p0[0]) * dx + (pos[1] - p0[1]) * dy) / seg_len2
        t_seg = max(0.0, min(1.0, t_seg))
        proj = (p0[0] + t_seg * dx, p0[1] + t_seg * dy)
        d2 = (pos[0] - proj[0]) ** 2 + (pos[1] - proj[1]) ** 2
        seg_len = math.sqrt(seg_len2)
        if d2 < best_d2:
            best_d2 = d2
            best_pt = proj
            best_s = accum + t_seg * seg_len
        accum += seg_len
    total = accum
    t = 0.0 if total <= 1e-6 else best_s / total
    return t, best_pt, best_d2

def _polyline_tangent(pts, t):
    if len(pts) < 2:
        return (1.0, 0.0)
    t = max(0.0, min(1.0, float(t)))
    t0 = max(0.0, t - 0.002)
    t1 = min(1.0, t + 0.002)
    p0 = _polyline_point_at(pts, t0)
    p1 = _polyline_point_at(pts, t1)
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    mag = math.hypot(dx, dy)
    if mag > 1e-6:
        return (dx / mag, dy / mag)
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        mag = math.hypot(dx, dy)
        if mag > 1e-6:
            return (dx / mag, dy / mag)
    return (1.0, 0.0)

def _edge_marker_hit(pos):
    best = None
    hit_r2 = EDGE_MARKER_HIT_RADIUS * EDGE_MARKER_HIT_RADIUS
    for i in range(len(display_nodes) - 1):
        node = display_nodes[i]
        events = node.get("edge_events", [])
        if not isinstance(events, list) or not events:
            continue
        pts = _edge_polyline(i)
        for j, ev in enumerate(events):
            try:
                t = float(ev.get("t", 0.0))
            except Exception:
                t = 0.0
            p = _polyline_point_at(pts, t)
            d2 = (pos[0] - p[0]) ** 2 + (pos[1] - p[1]) ** 2
            if d2 <= hit_r2 and (best is None or d2 < best[3]):
                best = (i, j, p, d2)
    return best

def _edge_marker_preview(pos):
    best = None
    thresh = EDGE_MARKER_HIT_RADIUS * 1.8
    best_d2 = thresh * thresh
    for i in range(len(display_nodes) - 1):
        pts = _edge_polyline(i)
        if len(pts) < 2:
            continue
        t, proj, d2 = _polyline_nearest(pos, pts)
        if d2 < best_d2:
            best_d2 = d2
            best = (i, t, proj)
    return best

def log_action(kind, **kw):
    """Log action to output."""
    _refresh_output_header()
    if kind == "move":
        i0, i1 = kw["i0"], kw["i1"]
        p0, p1 = kw["p0"], kw["p1"]
        log_lines.append(f"\n --- Node {i0} {coords_str(p0, CFG, display_nodes[0]['pos'])} to Node {i1} {coords_str(p1, CFG, display_nodes[0]['pos'])} ---")
        inches = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
        if kw.get("reverse", False): 
            inches = -inches
        conv, unit = pros_convert_inches(CFG, inches)
        log_lines.append(f"  Distance: {conv:.3f} {unit}")
    elif kind == "path":
        i0, i1 = kw["i0"], kw["i1"]
        p0, p1 = kw["p0"], kw["p1"]
        log_lines.append(f"\n --- Path Node {i0} {coords_str(p0, CFG, display_nodes[0]['pos'])} to Node {i1} {coords_str(p1, CFG, display_nodes[0]['pos'])} ---")
        path_points = kw.get("path_points") or []
        if path_points and len(path_points) > 1:
            total_px = 0.0
            for j in range(len(path_points) - 1):
                total_px += math.hypot(
                    path_points[j+1][0] - path_points[j][0],
                    path_points[j+1][1] - path_points[j][1]
                )
            inches = total_px / PPI
        else:
            inches = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
        conv, unit = pros_convert_inches(CFG, inches)
        log_lines.append(f"  Path Length: {conv:.3f} {unit}")
    elif kind == "turn":
        h0, h1 = kw["h0"], kw["h1"]
        delta_ccw = (h1 - h0 + 360.0) % 360.0
        if delta_ccw > 180.0:
            delta_ccw -= 360.0
        chosen = float(-delta_ccw)
        dir_tag = "CCW" if chosen >= 0 else "CW"
        to_face_deg = convert_heading_input(h1, None)
        if int(CFG.get("angle_units", 0)) == 1:
            chosen_val = chosen * (math.pi/180.0)
            to_face_val = to_face_deg * (math.pi/180.0)
            log_lines.append(f"  Turn: {chosen_val:.6f} rad ({dir_tag} +) | To face: {to_face_val:.6f} rad")
        else:
            log_lines.append(f"  Turn: {chosen:.3f}° ({dir_tag} +) | To face: {to_face_deg:.3f}°")
    elif kind == "swing":
        h0, h1 = kw["h0"], kw["h1"]
        delta_ccw = (h1 - h0 + 360.0) % 360.0
        if delta_ccw > 180.0:
            delta_ccw -= 360.0
        chosen = float(-delta_ccw)
        dir_tag = str(kw.get("dir", "AUTO")).upper()
        to_face_deg = convert_heading_input(h1, None)
        log_lines.append(f"  Swing: {chosen:.3f}° dir={dir_tag} to {to_face_deg:.3f}°")
    elif kind == "wait":
        log_lines.append(f"  Wait: {kw['s']:.3f} s")
    elif kind == "reshape":
        log_lines.append(f"  Reshape: state {kw['state']}")
    elif kind == "reverse":
        log_lines.append(f"  Reverse: {'ON' if kw['state'] else 'OFF'}")
    elif kind == "marker":
        label = str(kw.get("label", "")).strip()
        if label:
            log_lines.append(f"  Marker: {label}")
        else:
            log_lines.append("  Marker")
    ui.output_refresh(log_lines)

def _refresh_output_header():
    """Keep output header in sync with current settings when present."""
    if not log_lines:
        return
    if not str(log_lines[0]).startswith("=== ATTICUS TERMINAL COMPILE"):
        return
    end = None
    for i, line in enumerate(log_lines[:40]):
        if line == "":
            end = i + 1
            break
    if end is None:
        return
    header = build_compile_header(CFG, initial_state["heading"])
    log_lines[:end] = header

def enter_path_edit_mode(segment_idx):
    """Enter path editing mode for a segment."""
    global path_edit_mode, path_edit_segment_idx, path_control_points, selected_control_point
    global dragging, constrain_active, constrain_origin

    if segment_idx is None or segment_idx >= len(display_nodes) - 1:
        return
    
    sync_path_endpoints_for_node(segment_idx)
    path_edit_mode = True
    path_edit_segment_idx = segment_idx
    dragging = False
    constrain_active = False
    constrain_origin = None

    node = display_nodes[segment_idx]

    # Initialize or load path data. If this segment is currently straight
    # (no usable control points), create a default three-point path.
    path_data = node.get("path_to_next") or {}
    cps = None
    if (
        path_data is None
        or not path_data.get("use_path", False)
        or not path_data.get("control_points")
        or len(path_data.get("control_points", ())) < 2
    ):
        # Create default path with one mid control point
        p0 = effective_node_pos(segment_idx)
        p1 = effective_node_pos(segment_idx + 1)
        mid_x = (p0[0] + p1[0]) // 2
        mid_y = (p0[1] + p1[1]) // 2

        cps = [p0, (mid_x, mid_y), p1]
        node["path_to_next"] = {
            "use_path": True,
            "control_points": cps
        }
        # Seed offset ghost for endpoint if offset is active
        end_node = display_nodes[segment_idx + 1]
        if end_node.get("offset", 0) != 0 or end_node.get("offset_custom_in") is not None:
            if end_node.get("offset_ghost_angle") is None:
                arrival_heading = _arrival_heading_for_offset(path_data, p0, p1)
                if arrival_heading is None:
                    arrival_heading = heading_from_points(p0, p1)
                end_node["offset_ghost_angle"] = arrival_heading
    else:
        cps = list(path_data["control_points"])
    
    path_control_points = list(cps)
    selected_control_point = None

def exit_path_edit_mode():
    """Exit path editing mode and save changes."""
    global path_edit_mode, path_edit_segment_idx, path_control_points, selected_control_point
    global last_path_sig, last_snapshot, total_estimate_s, selected_idx
    global dragging, constrain_active, constrain_origin, dragging_control_point

    if path_edit_mode and path_edit_segment_idx is not None:
        node = display_nodes[path_edit_segment_idx]
        # If there are at least one internal control point (endpoints + handles),
        # keep this segment curved. Otherwise, treat it as straight.
        if path_control_points and len(path_control_points) > 2:
            cp = list(path_control_points)
            # Ensure endpoints follow the current node positions
            cp[0] = effective_node_pos(path_edit_segment_idx)
            cp[-1] = effective_node_pos(path_edit_segment_idx + 1)
            node.setdefault("path_to_next", {})
            node["path_to_next"]["control_points"] = cp
            node["path_to_next"]["use_path"] = True
        else:
            path_to_next = node.get("path_to_next")
            if path_to_next is not None:
                # No internal handles: fall back to a straight segment
                path_to_next["control_points"] = []
                path_to_next["use_path"] = False
                path_to_next.pop("min_speed_cmd", None)
                path_to_next.pop("max_speed_cmd", None)
                path_to_next.pop("min_speed_ips", None)
                path_to_next.pop("max_speed_ips", None)
                path_to_next.pop("mirror_start", None)
                path_to_next.pop("mirror_end", None)
        sync_path_endpoints_for_node(path_edit_segment_idx)

    path_edit_mode = False
    path_edit_segment_idx = None
    path_control_points = []
    selected_control_point = None
    selected_idx = None
    dragging = False
    dragging_control_point = False
    constrain_active = False
    constrain_origin = None

    # Trigger recompilation
    last_path_sig = None
    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
    total_estimate_s = compute_total_estimate_s()

def toggle_segment_curved(segment_idx):
    """Toggle whether a segment uses a curved path."""
    if segment_idx is None or segment_idx >= len(display_nodes) - 1:
        return
    
    node = display_nodes[segment_idx]
    
    if "path_to_next" in node and node["path_to_next"].get("use_path"):
        # Disable curved path
        node["path_to_next"]["use_path"] = False
    else:
        # Enable curved path - enter edit mode to set it up
        enter_path_edit_mode(segment_idx)

def add_control_point_at_mouse(mouse_pos):
    """Add a control point at mouse position."""
    global path_control_points
    
    if not path_edit_mode or not path_control_points:
        return
    
    # Find where to insert (closest segment)
    min_dist = float('inf')
    insert_idx = 1
    
    for i in range(len(path_control_points) - 1):
        p1 = path_control_points[i]
        p2 = path_control_points[i + 1]
        
        # Distance from mouse to line segment
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length_sq = dx*dx + dy*dy
        
        if length_sq == 0:
            dist = math.hypot(mouse_pos[0] - p1[0], mouse_pos[1] - p1[1])
        else:
            t = max(0, min(1, ((mouse_pos[0] - p1[0]) * dx + (mouse_pos[1] - p1[1]) * dy) / length_sq))
            proj_x = p1[0] + t * dx
            proj_y = p1[1] + t * dy
            dist = math.hypot(mouse_pos[0] - proj_x, mouse_pos[1] - proj_y)
        
        if dist < min_dist:
            min_dist = dist
            insert_idx = i + 1
    
    path_control_points.insert(insert_idx, tuple(mouse_pos))

def remove_control_point(point_idx):
    """Remove a control point (cannot remove endpoints)."""
    global path_control_points, selected_control_point

    if not path_edit_mode or not path_control_points:
        return

    # Cannot remove first or last point (segment endpoints)
    if point_idx <= 0 or point_idx >= len(path_control_points) - 1:
        return

    path_control_points.pop(point_idx)
    selected_control_point = None

# ---------------- node command parsing ----------------
def compile_cmd_string(node, idx):
    """Generate command string from node actions."""
    parts = []
    acts = node.get("actions", [])
    has_rev_act = any(a.get("type") == "reverse" for a in acts)
    if node.get("reverse") and not has_rev_act:
        parts.append("reverse")
    for act in acts:
        if act.get("type") == "turn":
            disp = convert_heading_input(act.get("deg", 0.0), None)
            parts.append(f"turn {disp:g}")
        elif act.get("type") == "swing":
            disp = convert_heading_input(act.get("deg", 0.0), None)
            sd = str(act.get("dir", "auto")).lower()
            settle = bool(act.get("settle", False))
            if sd in ("cw", "ccw"):
                prefix = "settleswing" if settle else "swing"
                parts.append(f"{prefix} {disp:g} {sd}")
            else:
                prefix = "settleswing" if settle else "swing"
                parts.append(f"{prefix} {disp:g}")
        elif act.get("type") == "wait":
            parts.append(f"wait {act.get('s', 0):g}")
        elif act.get("type") in ("reshape", "geom"):
            parts.append("reshape")
        elif act.get("type") == "reverse":
            state = act.get("state", None)
            if state is True:
                parts.append("reverse on")
            elif state is False:
                parts.append("reverse off")
            else:
                parts.append("reverse")
    if node.get("swing_target_heading_deg") is not None:
        disp = convert_heading_input(node["swing_target_heading_deg"], None)
        parts.append(f"swingto {disp:g}")
    if idx != 0 and int(node.get("offset", 0)) == 0 and node.get("offset_custom_in") is not None:
        parts.append(f"offset {node['offset_custom_in']:g}")
    if node.get("chain_through", False) or node.get("chain", False):
        looseness = node.get("chain_looseness")
        if looseness is None:
            parts.append("chain")
        else:
            try:
                parts.append(f"chain {float(looseness):g}")
            except Exception:
                parts.append("chain")
    lat_cmd = _node_lateral_cmd(node)
    if lat_cmd:
        parts.append(f"latspeed {float(lat_cmd):g}")
    if node.get("custom_turn_dps"):
        parts.append(f"turnspeed {node['custom_turn_dps']:g}")
    return ", ".join(parts)

def parse_and_apply_cmds(node, cmd_str, idx):
    """Parse and apply commands to node."""
    acts = []
    node.pop("swing_target_heading_deg", None)
    node.pop("reverse_after_swing", None)
    orig_reverse = bool(node.get("reverse", False))
    node["reverse"] = orig_reverse
    seen_swingto = False
    has_reverse_action = False
    if cmd_str:
        for part in [p.strip() for p in cmd_str.replace(";", ",").split(",") if p.strip()]:
            low = part.lower()
            try:
                if low.startswith(("wait", "w", "sleep", "pause")):
                    x = float(low.split()[1].replace("sec", "").replace("s", ""))
                    if x > 0:
                        acts.append({"type": "wait", "s": x})
                elif low.startswith(("turn", "t", "rotate")):
                    raw = float(low.split()[1].replace("deg", "").replace("Aų", ""))
                    x_internal = interpret_input_angle(raw)
                    acts.append({"type": "turn", "deg": x_internal})
                elif low.startswith("swingto"):
                    raw = float(low.split()[1].replace("deg", "").replace("Aų", ""))
                    node["turn_mode"] = "swing"
                    node["swing_dir"] = "auto"
                    node["swing_target_heading_deg"] = interpret_input_angle(raw)
                    seen_swingto = True
                elif low.startswith("settleswing") or low.startswith("swing"):
                    settle_flag = low.startswith("settleswing")
                    tokens = low.split()
                    dir_tok = None
                    val_tok = None
                    if len(tokens) >= 3 and tokens[1] in ("cw", "ccw", "auto"):
                        dir_tok = tokens[1]
                        val_tok = tokens[2]
                    else:
                        if len(tokens) >= 2:
                            val_tok = tokens[1]
                        if len(tokens) >= 3 and tokens[2] in ("cw", "ccw", "auto"):
                            dir_tok = tokens[2]
                    if val_tok is not None:
                        raw = float(val_tok.replace("deg", "").replace("AŹ3", ""))
                        x_internal = interpret_input_angle(raw)
                        sd = dir_tok if dir_tok in ("cw", "ccw", "auto") else "auto"
                        acts.append({"type": "swing", "deg": x_internal, "dir": sd, "settle": settle_flag})
                elif low.startswith("offset"):
                    if idx != 0 and int(node.get("offset", 0)) == 0:
                        x = float(low.split()[1].replace("in", ""))
                        node["offset_custom_in"] = x
                elif low in ("reshape", "rs", "geom"):
                    acts.append({"type": "reshape"})
                elif low.startswith("reverse"):
                    tokens = low.split()
                    if seen_swingto:
                        node["reverse_after_swing"] = True
                    else:
                        state = None
                        if len(tokens) >= 2 and tokens[1] in ("on", "1", "true", "yes"):
                            state = True
                        elif len(tokens) >= 2 and tokens[1] in ("off", "0", "false", "no"):
                            state = False
                        acts.append({"type": "reverse", "state": state})
                        has_reverse_action = True
                elif low.startswith(("latspeed", "lat", "drive_speed")):
                    parts_split = low.split()
                    if len(parts_split) >= 2:
                        token = parts_split[1]
                        if token in ("off", "none", "clear"):
                            node.pop("custom_lateral_cmd", None)
                            node.pop("custom_lateral_ips", None)
                        else:
                            try:
                                if token.endswith("ips"):
                                    continue
                                if token.endswith("cmd"):
                                    token = token[:-3]
                                val = float(token)
                                val = max(0.0, min(127.0, val))
                                node["custom_lateral_cmd"] = val if val > 0 else None
                                node.pop("custom_lateral_ips", None)
                            except Exception:
                                pass
                elif low.startswith("chain"):
                    tokens = low.split()
                    if len(tokens) >= 2 and tokens[1] in ("off", "0", "false", "no", "disable"):
                        node["chain_through"] = False
                        node.pop("chain", None)
                        node.pop("chain_looseness", None)
                    else:
                        node["chain_through"] = True
                        node.pop("chain", None)
                        if len(tokens) >= 2 and tokens[1] not in ("on", "true", "1", "yes"):
                            try:
                                val = float(tokens[1])
                                val = max(0.0, min(1.0, val))
                                node["chain_looseness"] = val
                            except Exception:
                                pass
                elif low.startswith(("turnspeed", "turnrate", "omega")):
                    parts_split = low.split()
                    if len(parts_split) >= 2:
                        token = parts_split[1]
                        if token in ("off", "none", "clear"):
                            node.pop("custom_turn_dps", None)
                        else:
                            try:
                                val = float(token.replace("dps", "").replace("deg/s", ""))
                                node["custom_turn_dps"] = val if val > 0 else None
                            except Exception:
                                pass
            except Exception:
                pass
    node["actions"] = acts

def _mech_presets():
    raw = CFG.get("codegen", {}).get("mech_presets", [])
    if not isinstance(raw, list):
        return []
    out = []
    for p in raw:
        if not isinstance(p, dict):
            continue
        name = str(p.get("name", "")).strip()
        if not name:
            continue
        mode = str(p.get("mode", "action")).strip().lower()
        if mode not in ("action", "toggle"):
            mode = "action"
        out.append({
            "name": name,
            "mode": mode,
            "template": str(p.get("template", "") or p.get("action", "") or ""),
            "on": str(p.get("on", "")),
            "off": str(p.get("off", "")),
            "default": bool(p.get("default", False)),
        })
    return out

def _marker_actions_to_text(actions):
    parts = []
    if not isinstance(actions, list):
        return ""
    for act in actions:
        if not isinstance(act, dict):
            continue
        kind = str(act.get("kind", "code")).lower()
        if kind == "preset":
            name = str(act.get("name", "")).strip()
            state = str(act.get("state", "")).strip().lower()
            value = str(act.get("value", "")).strip()
            text = name
            if state:
                text = f"{text} {state}"
            if value:
                text = f"{text} {value}"
            if text.strip():
                parts.append(text.strip())
        else:
            code = str(act.get("code", "")).strip()
            if code:
                parts.append(f"code {code}")
    return ", ".join(parts)

def _marker_apply_reshape(actions, current_state):
    state = bool(current_state)
    if not isinstance(actions, list):
        return state
    for act in actions:
        if not isinstance(act, dict):
            continue
        kind = str(act.get("kind", "")).lower()
        if kind == "preset":
            name = str(act.get("name", "")).strip().lower()
            if name != "reshape":
                continue
            st = str(act.get("state", "")).strip().lower()
            if st in ("on", "true", "1", "enable", "enabled"):
                state = True
            elif st in ("off", "false", "0", "disable", "disabled"):
                state = False
            else:
                state = not state
    return state

def _marker_hover_text(actions):
    if not isinstance(actions, list):
        return ""
    items = []
    reshape_label = str(CFG.get("reshape_label", "Reshape")) if isinstance(CFG, dict) else "Reshape"
    for act in actions:
        if not isinstance(act, dict):
            continue
        kind = str(act.get("kind", "")).lower()
        if kind == "preset":
            name = str(act.get("name", "")).strip()
            if name:
                if name.strip().lower() == "reshape":
                    items.append(reshape_label)
                else:
                    items.append(name)
        else:
            items.append("Custom")
    if not items:
        return ""
    out = []
    seen = set()
    for item in items:
        key = item.strip().lower()
        if not key or key in seen:
            continue
        out.append(item)
        seen.add(key)
    return ", ".join(out)

def _draw_marker_hover(surface, text, mouse_pos, font_small):
    if not text:
        return
    draw_label(surface, mouse_pos, [text], font_small)

def _parse_marker_actions(text):
    actions = []
    if not text:
        return actions
    presets = _mech_presets()
    preset_names = [p["name"] for p in presets]
    preset_names.sort(key=lambda s: len(s), reverse=True)
    for part in [p.strip() for p in text.replace(";", ",").split(",") if p.strip()]:
        low = part.lower()
        if low.startswith("code "):
            actions.append({"kind": "code", "code": part[5:].strip()})
            continue
        matched = None
        matched_name = None
        for name in preset_names:
            if low == name.lower() or low.startswith(name.lower() + " "):
                matched = name
                matched_name = name
                break
        if matched:
            rem = part[len(matched_name):].strip()
            tokens = rem.split() if rem else []
            state = None
            value = None
            if tokens and tokens[0].lower() in ("on", "off", "toggle"):
                state = tokens[0].lower()
                tokens = tokens[1:]
            if tokens:
                value = " ".join(tokens)
            actions.append({"kind": "preset", "name": matched_name, "state": state, "value": value})
        else:
            actions.append({"kind": "code", "code": part})
    return actions

def _marker_prompt_text():
    presets = _mech_presets()
    lines = [
        "Commands (comma/semicolon separated). Examples:",
        "  intake on, clamp toggle",
        "  lift 300",
        "  code digitalWrite(PNEU, 1);",
        ""
    ]
    if presets:
        lines.append("Presets:")
        for p in presets:
            mode = p.get("mode", "action")
            lines.append(f"  {p['name']} ({mode})")
    else:
        lines.append("Presets: none (set in Export > Mechanism presets)")
    return "\n".join(lines)

def _edit_edge_marker(seg_idx, event_idx=None, t=None):
    if seg_idx < 0 or seg_idx >= len(display_nodes) - 1:
        return False
    node = display_nodes[seg_idx]
    events = node.setdefault("edge_events", [])
    if not isinstance(events, list):
        node["edge_events"] = []
        events = node["edge_events"]
    creating = event_idx is None
    if creating:
        if t is None:
            t = 0.5
        try:
            t = float(t)
        except Exception:
            t = 0.5
        t = max(0.0, min(1.0, t))
        events.append({"t": t, "enabled": True, "actions": []})
        event_idx = len(events) - 1
    if event_idx is None or event_idx < 0 or event_idx >= len(events):
        return False
    ev = events[event_idx]
    init = _marker_actions_to_text(ev.get("actions", []))
    resp = _askstring_centered("Mechanism marker", _marker_prompt_text(), initialvalue=init)
    if resp is None:
        if creating and event_idx < len(events):
            events.pop(event_idx)
        return False
    if not resp.strip():
        if event_idx < len(events):
            events.pop(event_idx)
        return True
    ev["actions"] = _parse_marker_actions(resp)
    ev["enabled"] = True
    try:
        events.sort(key=lambda e: float(e.get("t", 0.0)))
    except Exception:
        pass
    return True
    if has_reverse_action:
        node["reverse"] = False
    _apply_profile_override_for_speed_overrides(node)

# ---------------- heading realization prompt ----------------
def _heading_realization_string(node: dict) -> str:
    """Build a user-facing heading realization string from node settings."""
    tm = node.get("turn_mode", "turn")
    sd = node.get("swing_dir", "auto")
    parts = []
    if tm == "swing":
        if node.get("swing_settle"):
            parts.append(f"settleswing {sd}")
        else:
            parts.append(f"swing {sd}")
    else:
        parts.append(f"turn {sd}")
    if node.get("move_to_pose"):
        h = node.get("pose_heading_deg")
        lead = node.get("pose_lead_in")
        if h is not None:
            disp = convert_heading_input(h, None)
            if lead not in (None, "", 0, 0.0):
                parts.append(f"movetopose {disp:.3f} {float(lead):g}")
            else:
                parts.append(f"movetopose {disp:.3f}")
    # Motion profile override (skip if segment is a path)
    pd = node.get("path_to_next", {})
    if not pd.get("use_path", False):
        prof = node.get("profile_override")
        if prof:
            parts.append(f"profile {prof}")
    return "; ".join(parts)

def _apply_heading_realization(node: dict, text: str):
    """Parse heading realization text and apply to node."""
    tm = node.get("turn_mode", "turn")
    sd = node.get("swing_dir", "auto")
    swing_settle = bool(node.get("swing_settle", False))
    move_pose = bool(node.get("move_to_pose", False))
    pose_h = node.get("pose_heading_deg")
    pose_lead = node.get("pose_lead_in")
    prof_override = node.get("profile_override")

    if text:
        for part in [p.strip() for p in text.replace(";", ",").split(",") if p.strip()]:
            tokens = part.split()
            if not tokens:
                continue
            key = tokens[0].lower()
            if key in ("turn", "t"):
                tm = "turn"
                swing_settle = False
                if len(tokens) >= 2 and tokens[1].lower() in ("cw", "ccw", "auto"):
                    sd = tokens[1].lower()
            elif key in ("swing", "settleswing"):
                tm = "swing"
                swing_settle = (key == "settleswing")
                if len(tokens) >= 2 and tokens[1].lower() in ("cw", "ccw", "auto"):
                    sd = tokens[1].lower()
            elif key in ("movetopose", "pose", "poseheading"):
                if len(tokens) >= 2 and tokens[1].lower() not in ("off", "none", "clear"):
                    try:
                        disp = float(tokens[1])
                        pose_h = interpret_input_angle(disp)
                        move_pose = True
                    except Exception:
                        pass
                else:
                    move_pose = False
                if len(tokens) >= 3:
                    try:
                        pose_lead = float(tokens[2])
                    except Exception:
                        pass
            elif key in ("profile", "mode", "speedprofile"):
                if len(tokens) >= 2:
                    cand = tokens[1].lower()
                    if cand in ("precise", "normal", "fast", "slam"):
                        prof_override = cand
                    elif cand in ("off", "clear", "none"):
                        prof_override = None
    node["turn_mode"] = tm
    node["swing_dir"] = sd
    if tm == "swing":
        node["swing_settle"] = bool(swing_settle)
    else:
        node.pop("swing_settle", None)
    node["move_to_pose"] = bool(move_pose)
    if prof_override:
        node["profile_override"] = prof_override
    else:
        node.pop("profile_override", None)
    if move_pose:
        if pose_h is not None:
            node["pose_heading_deg"] = pose_h % 360.0
        if pose_lead not in (None, ""):
            node["pose_lead_in"] = float(pose_lead)
        else:
            node.pop("pose_lead_in", None)
    else:
        node.pop("pose_heading_deg", None)
        node.pop("pose_lead_in", None)
    _apply_profile_override_for_speed_overrides(node)

def _terminal_window_rect():
    """Return terminal window (x, y, w, h) or screen-centered fallback."""
    try:
        pos_fn = getattr(pygame.display, "get_window_position", None)
        size_fn = getattr(pygame.display, "get_window_size", None)
        x, y = (None, None)
        if callable(pos_fn):
            x, y = pos_fn()
        w, h = WINDOW_WIDTH, WINDOW_HEIGHT
        if callable(size_fn):
            w, h = size_fn()
        return x, y, w, h
    except Exception:
        return None, None, WINDOW_WIDTH, WINDOW_HEIGHT

def _center_tk_root_on_terminal():
    """Place the hidden Tk root at the terminal window center for dialogs."""
    if not ui.tk_root:
        return
    try:
        x, y, w, h = _terminal_window_rect()
        if x is None or y is None:
            sw = ui.tk_root.winfo_screenwidth()
            sh = ui.tk_root.winfo_screenheight()
            cx = sw // 2
            cy = sh // 2
        else:
            cx = int(x + w / 2)
            cy = int(y + h / 2)
        ui.tk_root.geometry(f"1x1+{cx}+{cy}")
        ui.tk_root.update_idletasks()
    except Exception:
        pass

def _center_tk_window_on_terminal(win):
    """Center a Tk window on the terminal window."""
    try:
        win.update_idletasks()
        w = win.winfo_reqwidth()
        h = win.winfo_reqheight()
        x, y, tw, th = _terminal_window_rect()
        if x is None or y is None:
            sw = win.winfo_screenwidth()
            sh = win.winfo_screenheight()
            cx = sw // 2
            cy = sh // 2
        else:
            cx = int(x + tw / 2)
            cy = int(y + th / 2)
        win.geometry(f"{w}x{h}+{int(cx - w / 2)}+{int(cy - h / 2)}")
    except Exception:
        pass

def _askstring_centered(title, prompt, initialvalue=None):
    """Ask for a string with the dialog centered and selected for editing."""
    ui.ensure_tk_root()
    resp = {"value": None}
    try:
        if ui.tk_root:
            try:
                ui.tk_root.attributes("-alpha", 0.0)
            except Exception:
                pass
            ui.tk_root.deiconify()
            ui.tk_root.lift()
            _center_tk_root_on_terminal()
        top = tk.Toplevel(ui.tk_root)
        top.title(title)
        top.resizable(False, False)
        top.transient(ui.tk_root)
        frame = ttk.Frame(top, padding=10)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text=prompt, justify="left", wraplength=520).pack(anchor="w")
        entry_var = tk.StringVar(value="" if initialvalue is None else str(initialvalue))
        entry = ttk.Entry(frame, textvariable=entry_var, width=70)
        entry.pack(fill="x", pady=(8, 8))
        entry.selection_range(0, "end")
        entry.icursor("end")
        entry.focus_set()
        btns = ttk.Frame(frame)
        btns.pack(fill="x")

        def _ok():
            resp["value"] = entry_var.get()
            try:
                top.destroy()
            except Exception:
                pass

        def _cancel():
            resp["value"] = None
            try:
                top.destroy()
            except Exception:
                pass

        ttk.Button(btns, text="OK", command=_ok).pack(side="right")
        ttk.Button(btns, text="Cancel", command=_cancel).pack(side="right", padx=(0, 6))
        top.bind("<Return>", lambda _e: _ok())
        top.bind("<Escape>", lambda _e: _cancel())
        top.protocol("WM_DELETE_WINDOW", _cancel)
        top.grab_set()
        _center_tk_window_on_terminal(top)
        top.wait_window()
    finally:
        if ui.tk_root:
            try:
                ui.tk_root.withdraw()
                ui.tk_root.attributes("-alpha", 1.0)
            except Exception:
                pass
    return resp["value"]

def prompt_heading_realization(node: dict, idx: int) -> bool:
    """Show heading realization prompt; return True if node mutated."""
    init = _heading_realization_string(node)
    resp = _askstring_centered(
        "Heading realization",
        "turn/swing/settleswing + optional movetopose heading [+lead] (comma/semicolon separated).\n"
        "Optional: profile precise|normal|fast|slam (non-path segments only).\n"
        "Examples: 'turn auto;'; 'swing cw; movetopose 180 6'; 'settleswing cw'; 'profile precise'.\n"
        "Omit movetopose to disable enforcing final heading.\n"
        "Angles use 0=left, 90=up.",
        initialvalue=init
    )
    if resp is None:
        return False
    before = {k: node.get(k) for k in ("turn_mode", "swing_dir", "move_to_pose", "pose_heading_deg", "pose_lead_in")}
    _apply_heading_realization(node, resp)
    after = {k: node.get(k) for k in ("turn_mode", "swing_dir", "move_to_pose", "pose_heading_deg", "pose_lead_in")}
    return before != after

def open_settings_window():
    """Open settings configuration window."""
    global CFG, initial_state, robot_heading
    ui.ensure_tk_root()
    if ui.tk_settings_win is not None and ui.tk_settings_win.winfo_exists():
        return
    
    # Reload config from disk
    try:
        CFG = reload_cfg()
        try:
            saved_deg = float(CFG.get("initial_heading_deg", 0.0))
            initial_state["heading"] = interpret_input_angle(saved_deg)
            robot_heading = initial_state["heading"]
        except Exception:
            pass
        global path_lookahead_enabled, path_lookahead_px
        path_lookahead_enabled = bool(CFG.get("path_config", {}).get("simulate_pursuit", 1))
        auto_la = auto_lookahead_in(CFG)
        CFG.setdefault("path_config", {})["lookahead_in"] = auto_la
        path_lookahead_px = max(0.0, float(auto_la) * PPI)
    except Exception:
        pass
    
    top = tk.Toplevel(ui.tk_root)
    ui.tk_settings_win = top
    top.title("Settings")
    top.geometry("500x600")
    top.resizable(False, False)
    
    outer = ttk.Frame(top, padding=12)
    outer.pack(fill="both", expand=True)
    ttk.Label(outer, text="Terminal Menu", style="Header.TLabel").pack(anchor="w", pady=(0, 8))
    ttk.Label(outer, text="Hover any label for help.", style="Help.TLabel").pack(anchor="w", pady=(0, 8))
    notebook = ttk.Notebook(outer)
    notebook.pack(fill="both", expand=True)
    
    # Create tabs
    tabs = {
        "controls": ttk.Frame(notebook),
        "general": ttk.Frame(notebook),
        "physics": ttk.Frame(notebook),
        "geometry": ttk.Frame(notebook),
        "codegen": ttk.Frame(notebook)
    }
    
    notebook.add(tabs["controls"], text="Controls")
    notebook.add(tabs["general"], text="General")
    notebook.add(tabs["physics"], text="Physics")
    notebook.add(tabs["geometry"], text="Geometry")
    notebook.add(tabs["codegen"], text="Export")
    
    for name in ("general", "geometry", "codegen"):
        for c in range(2):
            tabs[name].columnconfigure(c, weight=1)
    
    # Controls tab (scrollable)
    tabs["controls"].rowconfigure(0, weight=1)
    tabs["controls"].columnconfigure(0, weight=1)
    
    controls_canvas = tk.Canvas(tabs["controls"], borderwidth=0, highlightthickness=0)
    controls_frame = ttk.Frame(controls_canvas)
    controls_scrollbar = ttk.Scrollbar(tabs["controls"], orient="vertical", command=controls_canvas.yview)
    controls_canvas.configure(yscrollcommand=controls_scrollbar.set)
    
    controls_canvas.grid(row=0, column=0, sticky="nsew")
    controls_scrollbar.grid(row=0, column=1, sticky="ns")
    controls_canvas.create_window((0, 0), window=controls_frame, anchor="nw")
    
    def _on_controls_configure(event):
        controls_canvas.configure(scrollregion=controls_canvas.bbox("all"))
    
    controls_frame.bind("<Configure>", _on_controls_configure)
    
    def _on_mousewheel(event):
        delta = 0
        if hasattr(event, "delta") and event.delta:
            delta = int(-event.delta / 40) if abs(event.delta) >= 40 else (-2 if event.delta > 0 else 2)
        else:
            num = getattr(event, "num", None)
            if num == 4: delta = -1
            elif num == 5: delta = 1
        if delta: controls_canvas.yview_scroll(delta, "units")
    
    for w in (tabs["controls"], controls_canvas, controls_frame):
        w.bind("<MouseWheel>", _on_mousewheel)
        w.bind("<Button-4>", _on_mousewheel)
        w.bind("<Button-5>", _on_mousewheel)
    
    for i, (key, desc) in enumerate(CONTROLS):
        ttk.Label(controls_frame, text=key, font=("Segoe UI", 10, "bold")).grid(
            row=i, column=0, sticky="w", padx=(8, 6), pady=3
        )
        ttk.Label(controls_frame, text=desc, anchor="w").grid(
            row=i, column=1, sticky="w", padx=(0, 8), pady=3
        )
    
    # Dropdown mappings
    dist_labels = ["Inches", "Encoder degrees", "Encoder rotations", "Ticks"]
    dist_map = {dist_labels[0]: 0, dist_labels[1]: 1, dist_labels[2]: 2, dist_labels[3]: 3}
    dist_inv = {0: dist_labels[0], 1: dist_labels[1], 2: dist_labels[2], 3: dist_labels[3]}
    
    ang_labels = ["Degrees", "Radians"]
    ang_map = {ang_labels[0]: 0, ang_labels[1]: 1}
    ang_inv = {0: ang_labels[0], 1: ang_labels[1]}
    
    # Initialize UI variables from CFG
    dist_value = int(CFG.get("distance_units", 0))
    dist_value = 0 if dist_value not in (0, 1, 2, 3) else dist_value
    ang_value = int(CFG.get("angle_units", 0))
    ang_value = 0 if ang_value not in (0, 1) else ang_value
    
    dist_var = tk.StringVar(value=dist_inv[dist_value])
    ang_var = tk.StringVar(value=ang_inv[ang_value])
    
    init_heading_disp = convert_heading_input(initial_state["heading"], None)
    init_head = tk.StringVar(value=f"{init_heading_disp:.3f}")
    
    show_hitboxes_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_hitboxes", 1)))
    show_field_objects_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_field_objects", 1)))
    show_node_numbers_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_node_numbers", 1)))
    reshape_label_var = tk.StringVar(value=str(CFG.get("reshape_label", "Reshape")))
    auto_heading_node1_var = tk.IntVar(value=0)
    
    # Physics variables
    rp = CFG["robot_physics"]
    rpm_var = tk.StringVar(value=str(rp.get("rpm", 0)))
    diam_var = tk.StringVar(value=str(rp.get("diameter", 4)))
    vs_var = tk.StringVar(value="12")
    vt_var = tk.StringVar(value="12")
    omni_var = tk.IntVar(value=int(rp.get("all_omni", 0)))
    tracking_wheels_var = tk.StringVar(value=str(int(rp.get("tracking_wheels", 0))))
    w_var = tk.StringVar(value=str(rp.get("weight", 20)))
    tb_var = tk.StringVar(value=str(rp.get("t_buffer", 0)))
    adv_motion_var = tk.IntVar(value=int(rp.get("advanced_motion", 0)))
    # maxcmd removed from UI (fixed in config)
    gr_var = tk.StringVar(value=str(CFG.get("gear_ratio", 1.0)))
    dens_var = tk.StringVar(value=str(rp.get("point_density_per_in", 4.0)))
    curv_var = tk.DoubleVar(value=float(rp.get("curvature_gain", 0.05)))
    
    # Dimensions variables
    bd = CFG["bot_dimensions"]
    dt_w = tk.StringVar(value=str(bd.get("dt_width", bd.get("width", 0))))
    dt_l = tk.StringVar(value=str(bd.get("dt_length", bd.get("length", 0))))
    w_full = tk.StringVar(value=str(bd.get("width", 0)))
    l_full = tk.StringVar(value=str(bd.get("length", 0)))
    full_off_x = tk.StringVar(value=str(bd.get("full_offset_x_in", 0)))
    full_off_y = tk.StringVar(value=str(bd.get("full_offset_y_in", 0)))
    rs_w = tk.StringVar(value=str(bd.get("reshape_width", bd.get("width", 0))))
    rs_l = tk.StringVar(value=str(bd.get("reshape_length", bd.get("length", 0))))
    rs_off_x = tk.StringVar(value=str(bd.get("reshape_offset_x_in", 0)))
    rs_off_y = tk.StringVar(value=str(bd.get("reshape_offset_y_in", 0)))
    
    # Offsets variables
    off = CFG["offsets"]
    off1 = tk.StringVar(value=str(off.get("offset_1_in", 0)))
    off2 = tk.StringVar(value=str(off.get("offset_2_in", 0)))
    pad = tk.StringVar(value=str(off.get("padding_in", 0)))
    # Margin helpers (side margin = edge to track, back margin = rear edge to axle)
    def _init_margins():
        try:
            _dw = float(dt_w.get()); _dl = float(dt_l.get())
            _wf = float(w_full.get()); _lf = float(l_full.get())
            _wr = float(rs_w.get()); _lr = float(rs_l.get())
            _oy = float(full_off_y.get()); _ox = float(full_off_x.get())
            _oy_r = float(rs_off_y.get()); _ox_r = float(rs_off_x.get())
        except Exception:
            _dw = _dl = _wf = _lf = _wr = _lr = _oy = _ox = _oy_r = _ox_r = 0.0
        ml = max(0.0, _wf / 2.0 + _oy - _dw / 2.0)
        mr = max(0.0, _wf / 2.0 - _oy - _dw / 2.0)
        mb = max(0.0, _lf / 2.0 - _ox - _dl / 2.0)
        mf = max(0.0, _lf / 2.0 + _ox - _dl / 2.0)
        ml_r = max(0.0, _wr / 2.0 + _oy_r - _dw / 2.0)
        mr_r = max(0.0, _wr / 2.0 - _oy_r - _dw / 2.0)
        mb_r = max(0.0, _lr / 2.0 - _ox_r - _dl / 2.0)
        mf_r = max(0.0, _lr / 2.0 + _ox_r - _dl / 2.0)
        return (ml, mr, mb, mf, ml_r, mr_r, mb_r, mf_r)
    (ml0, mr0, mb0, mf0, mlr0, mrr0, mbr0, mfr0) = _init_margins()
    margin_left_var = tk.StringVar(value=f"{ml0:.3f}")
    margin_right_var = tk.StringVar(value=f"{mr0:.3f}")
    margin_back_var = tk.StringVar(value=f"{mb0:.3f}")
    margin_front_var = tk.StringVar(value=f"{mf0:.3f}")
    margin_left_r_var = tk.StringVar(value=f"{mlr0:.3f}")
    margin_right_r_var = tk.StringVar(value=f"{mrr0:.3f}")
    margin_back_r_var = tk.StringVar(value=f"{mbr0:.3f}")
    margin_front_r_var = tk.StringVar(value=f"{mfr0:.3f}")
    
    def _row(parent, r, label, widget, tip):
        """Helper to create labeled row with tooltip."""
        lbl = ttk.Label(parent, text=label)
        lbl.grid(row=r, column=0, sticky="w", padx=6, pady=4)
        widget.grid(row=r, column=1, sticky="ew", padx=6, pady=4)
        ui.add_tooltip(lbl, tip)
        ui.add_tooltip(widget, tip)
        ui.track_live_widget(widget)
    
    geometry_win = {"win": None}
    def open_geometry_visualizer():
        """Open a drag-to-edit geometry visualizer for drivetrain/full/reshape boxes."""
        if geometry_win["win"] is not None and geometry_win["win"].winfo_exists():
            geometry_win["win"].lift()
            geometry_win["win"].focus_force()
            return
        
        def _num(var, default=0.0):
            try:
                return float(var.get())
            except Exception:
                return float(default)
        
        win = tk.Toplevel(top)
        geometry_win["win"] = win
        win.title("Geometry Visualizer")
        win.resizable(False, False)
        
        canvas_size = 320
        frame = ttk.Frame(win, padding=4)
        frame.pack(fill="both", expand=True)
        canvas = tk.Canvas(frame, width=canvas_size, height=canvas_size, background="#f8f8f8",
                           highlightthickness=1, highlightbackground="#cccccc")
        canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        sidebar = ttk.Frame(frame)
        sidebar.grid(row=0, column=1, sticky="nw")
        bottom = ttk.Frame(frame)
        bottom.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8,0))
        for c in range(3):
            bottom.columnconfigure(c, weight=0)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        
        mode_var = tk.StringVar(value="normal")
        ttk.Radiobutton(sidebar, text="Normal geometry", variable=mode_var, value="normal",
                        command=lambda: (drag_state.update({"target": None}), _set_margin_mode(), draw())).grid(row=0, column=0, sticky="w", pady=(0, 4))
        ttk.Radiobutton(sidebar, text="Reshape geometry", variable=mode_var, value="reshape",
                        command=lambda: (drag_state.update({"target": None}), _set_margin_mode(), draw())).grid(row=1, column=0, sticky="w", pady=(0, 8))
        
        info_var = tk.StringVar(value="")
        ttk.Label(sidebar, textvariable=info_var, foreground="#555555", wraplength=180, justify="left").grid(row=3, column=0, sticky="w", pady=(0, 6))
        # Margin entry rows (asymmetric)
        lbl_left = ttk.Label(sidebar, text="Left margin (edge→track):"); lbl_left.grid(row=4, column=0, sticky="w")
        m_left_entry = ttk.Entry(sidebar, textvariable=margin_left_var, width=10); m_left_entry.grid(row=5, column=0, sticky="w", pady=(0, 4))
        lbl_right = ttk.Label(sidebar, text="Right margin (edge→track):"); lbl_right.grid(row=6, column=0, sticky="w")
        m_right_entry = ttk.Entry(sidebar, textvariable=margin_right_var, width=10); m_right_entry.grid(row=7, column=0, sticky="w", pady=(0, 6))
        lbl_back = ttk.Label(sidebar, text="Back margin (rear→drive edge):"); lbl_back.grid(row=8, column=0, sticky="w")
        m_back_entry = ttk.Entry(sidebar, textvariable=margin_back_var, width=10); m_back_entry.grid(row=9, column=0, sticky="w", pady=(0, 4))
        lbl_front = ttk.Label(sidebar, text="Front margin (front→drive edge):"); lbl_front.grid(row=10, column=0, sticky="w")
        m_front_entry = ttk.Entry(sidebar, textvariable=margin_front_var, width=10); m_front_entry.grid(row=11, column=0, sticky="w", pady=(0, 8))
        lbl_lr_r = ttk.Label(sidebar, text="Reshape left/right margins:"); lbl_lr_r.grid(row=12, column=0, sticky="w")
        m_left_r_entry = ttk.Entry(sidebar, textvariable=margin_left_r_var, width=10); m_left_r_entry.grid(row=13, column=0, sticky="w", pady=(0, 2))
        m_right_r_entry = ttk.Entry(sidebar, textvariable=margin_right_r_var, width=10); m_right_r_entry.grid(row=14, column=0, sticky="w", pady=(0, 6))
        lbl_fb_r = ttk.Label(sidebar, text="Reshape back/front margins:"); lbl_fb_r.grid(row=15, column=0, sticky="w")
        m_back_r_entry = ttk.Entry(sidebar, textvariable=margin_back_r_var, width=10); m_back_r_entry.grid(row=16, column=0, sticky="w", pady=(0, 2))
        m_front_r_entry = ttk.Entry(sidebar, textvariable=margin_front_r_var, width=10); m_front_r_entry.grid(row=17, column=0, sticky="w", pady=(0, 8))
        def _apply_margins():
            """Apply margin inputs to dimensions (asymmetric, center derived)."""
            def _parse(var, default=0.0):
                try:
                    return float(var.get())
                except Exception:
                    return float(default)
            ml = _parse(margin_left_var); mr = _parse(margin_right_var)
            mb = _parse(margin_back_var); mf = _parse(margin_front_var)
            mlr = _parse(margin_left_r_var); mrr = _parse(margin_right_r_var)
            mbr = _parse(margin_back_r_var); mfr = _parse(margin_front_r_var)
            last_margin = getattr(_apply_margins, "_last", {"mode": mode_var.get(), "name": None})
            def adjust_pair(avail, first, second, name_first, name_second, mode_tag):
                """Ensure pair sums to avail; preserve the last-changed margin if in this pair."""
                changed = None
                if last_margin.get("mode") == mode_tag and last_margin.get("name") in (name_first, name_second):
                    changed = last_margin["name"]
                # Default to first if unknown
                if changed == name_second:
                    new_second = min(second, avail)
                    new_first = max(0.0, avail - new_second)
                else:
                    new_first = min(first, avail)
                    new_second = max(0.0, avail - new_first)
                return new_first, new_second
            try:
                dw = float(dt_w.get()); dl = float(dt_l.get())
                ow = float(w_full.get()); ol = float(l_full.get())
                rw = float(rs_w.get()); rl = float(rs_l.get())
            except Exception:
                dw = dl = ow = ol = rw = rl = 0.0
            # Adjust normal margins to occupy available space; favor last changed counterpart
            avail_w = max(0.0, ow - dw)
            ml, mr = adjust_pair(avail_w, ml, mr, "left", "right", "normal")
            avail_l = max(0.0, ol - dl)
            mb, mf = adjust_pair(avail_l, mb, mf, "back", "front", "normal")
            # Adjust reshape margins
            avail_w_r = max(0.0, rw - dw)
            mlr, mrr = adjust_pair(avail_w_r, mlr, mrr, "left_r", "right_r", "reshape")
            avail_l_r = max(0.0, rl - dl)
            mbr, mfr = adjust_pair(avail_l_r, mbr, mfr, "back_r", "front_r", "reshape")
            # Normal geometry from asymmetric margins
            w_full.set(f"{max(MIN_SIZE, dw + ml + mr):.3f}")
            l_full.set(f"{max(MIN_SIZE, dl + mb + mf):.3f}")
            full_off_y.set(f"{((ml - mr) / 2.0):.3f}")
            full_off_x.set(f"{((mf - mb) / 2.0):.3f}")
            # Reshape geometry from asymmetric margins
            rs_w.set(f"{max(MIN_SIZE, dw + mlr + mrr):.3f}")
            rs_l.set(f"{max(MIN_SIZE, dl + mbr + mfr):.3f}")
            rs_off_y.set(f"{((mlr - mrr) / 2.0):.3f}")
            rs_off_x.set(f"{((mfr - mbr) / 2.0):.3f}")
            margin_left_var.set(f"{ml:.3f}"); margin_right_var.set(f"{mr:.3f}")
            margin_back_var.set(f"{mb:.3f}"); margin_front_var.set(f"{mf:.3f}")
            margin_left_r_var.set(f"{mlr:.3f}"); margin_right_r_var.set(f"{mrr:.3f}")
            margin_back_r_var.set(f"{mbr:.3f}"); margin_front_r_var.set(f"{mfr:.3f}")
            draw()
        # Track last-edited margin to decide counterpart compensation
        def _mark_margin(name, mode_tag):
            _apply_margins._last = {"mode": mode_tag, "name": name}
        _apply_margins._last = {"mode": None, "name": None}
        ttk.Button(sidebar, text="Apply margins", command=_apply_margins).grid(row=18, column=0, sticky="w", pady=(0, 10))
        
        normal_margin_widgets = [lbl_left, m_left_entry, lbl_right, m_right_entry, lbl_back, m_back_entry, lbl_front, m_front_entry]
        reshape_margin_widgets = [lbl_lr_r, m_left_r_entry, m_right_r_entry, lbl_fb_r, m_back_r_entry, m_front_r_entry]
        normal_grid = {w: w.grid_info() for w in normal_margin_widgets}
        reshape_grid = {w: w.grid_info() for w in reshape_margin_widgets}
        
        def _current_margins(mode):
            ow, ol, ox, oy = _outer_vals(mode)
            dw, dl = _dt_vals()
            ml = max(0.0, ow / 2.0 + oy - dw / 2.0)
            mr = max(0.0, ow / 2.0 - oy - dw / 2.0)
            mb = max(0.0, ol / 2.0 - ox - dl / 2.0)
            mf = max(0.0, ol / 2.0 + ox - dl / 2.0)
            return ml, mr, mb, mf
        
        def _sync_margin_vars(mode):
            ml, mr, mb, mf = _current_margins(mode)
            if mode == "reshape":
                margin_left_r_var.set(f"{ml:.3f}")
                margin_right_r_var.set(f"{mr:.3f}")
                margin_back_r_var.set(f"{mb:.3f}")
                margin_front_r_var.set(f"{mf:.3f}")
            else:
                margin_left_var.set(f"{ml:.3f}")
                margin_right_var.set(f"{mr:.3f}")
                margin_back_var.set(f"{mb:.3f}")
                margin_front_var.set(f"{mf:.3f}")
        
        def _set_margin_mode():
            mode = mode_var.get()
            for w in normal_margin_widgets:
                try:
                    if mode == "normal":
                        w.grid(**normal_grid[w])
                    else:
                        w.grid_remove()
                except Exception:
                    pass
            for w in reshape_margin_widgets:
                try:
                    if mode == "reshape":
                        w.grid(**reshape_grid[w])
                    else:
                        w.grid_remove()
                except Exception:
                    pass
            _sync_margin_vars(mode)
        
        # Place dimension entries below editor to shorten sidebar
        dim_sections = [
            ("Drivetrain", [(dt_w, "Width (in)"), (dt_l, "Length (in)")]),
            ("Full", [(w_full, "Width (in)"), (l_full, "Length (in)")]),
            ("Reshape", [(rs_w, "Width (in)"), (rs_l, "Length (in)")]),
        ]
        row_b = 0
        for col, (title, fields) in enumerate(dim_sections):
            section = ttk.LabelFrame(bottom, text=title)
            section.grid(row=0, column=col, padx=4, pady=4, sticky="nsew")
            for r, (var, label_txt) in enumerate(fields):
                ttk.Label(section, text=label_txt).grid(row=r*2, column=0, sticky="w", padx=4, pady=(2,0))
                ttk.Entry(section, textvariable=var, width=10).grid(row=r*2+1, column=0, sticky="w", padx=4, pady=(0,4))
            section.columnconfigure(0, weight=1)
        
        def _bind_redraw(widget):
            widget.bind("<FocusOut>", lambda _e: draw())
            widget.bind("<Return>", lambda _e: draw())
        
        redraw_targets = [
            m_left_entry, m_right_entry, m_back_entry, m_front_entry,
            m_left_r_entry, m_right_r_entry, m_back_r_entry, m_front_r_entry,
        ]
        margin_marks = {
            m_left_entry: ("left", "normal"), m_right_entry: ("right", "normal"),
            m_back_entry: ("back", "normal"), m_front_entry: ("front", "normal"),
            m_left_r_entry: ("left_r", "reshape"), m_right_r_entry: ("right_r", "reshape"),
            m_back_r_entry: ("back_r", "reshape"), m_front_r_entry: ("front_r", "reshape"),
        }
        for section in bottom.winfo_children():
            for child in section.winfo_children():
                try:
                    if isinstance(child, ttk.Entry):
                        redraw_targets.append(child)
                except Exception:
                    pass
        
        for w in redraw_targets:
            try:
                _bind_redraw(w)
                if w in margin_marks:
                    name, mode_tag = margin_marks[w]
                    w.bind("<FocusIn>", lambda _e, n=name, m=mode_tag: _mark_margin(n, m))
                    w.bind("<KeyRelease>", lambda _e, n=name, m=mode_tag: (_mark_margin(n, m), _apply_margins()))
            except Exception:
                pass
        
        handles = []
        vis_state = {"scale": 1.0, "outer_c": (canvas_size/2, canvas_size/2), "base_c": (canvas_size/2, canvas_size/2)}
        drag_state = {"target": None}
        MIN_SIZE = 1.0
        STEP_IN = 1.0  # inch grid step
        
        def _outer_vals(active_mode=None):
            m = active_mode or mode_var.get()
            if m == "reshape":
                return (_num(rs_w, 0), _num(rs_l, 0), _num(rs_off_x, 0), _num(rs_off_y, 0))
            return (_num(w_full, 0), _num(l_full, 0), _num(full_off_x, 0), _num(full_off_y, 0))
        
        def _set_outer(w=None, l=None, offx=None, offy=None, active_mode=None):
            m = active_mode or mode_var.get()
            tgt_w, tgt_l, tgt_ox, tgt_oy = _outer_vals(m)
            tgt_w = w if w is not None else tgt_w
            tgt_l = l if l is not None else tgt_l
            tgt_ox = 0.0 if offx is None else offx
            tgt_oy = 0.0 if offy is None else offy
            tgt_w = max(MIN_SIZE, tgt_w)
            tgt_l = max(MIN_SIZE, tgt_l)
            if m == "reshape":
                rs_w.set(f"{tgt_w:.3f}")
                rs_l.set(f"{tgt_l:.3f}")
                rs_off_x.set(f"{tgt_ox:.3f}")
                rs_off_y.set(f"{tgt_oy:.3f}")
                try:
                    ms = max(0.0, (tgt_w - _num(dt_w, tgt_w)) / 2.0)
                    mb = max(0.0, tgt_l / 2.0)
                    margin_side_r_var.set(f"{ms:.3f}")
                    margin_back_r_var.set(f"{mb:.3f}")
                except Exception:
                    pass
            else:
                w_full.set(f"{tgt_w:.3f}")
                l_full.set(f"{tgt_l:.3f}")
                full_off_x.set(f"{tgt_ox:.3f}")
                full_off_y.set(f"{tgt_oy:.3f}")
                try:
                    ms = max(0.0, (tgt_w - _num(dt_w, tgt_w)) / 2.0)
                    mb = max(0.0, tgt_l / 2.0)
                    margin_side_var.set(f"{ms:.3f}")
                    margin_back_var.set(f"{mb:.3f}")
                except Exception:
                    pass
        
        def _dt_vals():
            return (_num(dt_w, 0), _num(dt_l, 0))
        
        def _set_dt(w=None, l=None):
            cur_w, cur_l = _dt_vals()
            cur_w = w if w is not None else cur_w
            cur_l = l if l is not None else cur_l
            dt_w.set(f"{max(MIN_SIZE, cur_w):.3f}")
            dt_l.set(f"{max(MIN_SIZE, cur_l):.3f}")
            try:
                ms = max(0.0, (float(w_full.get()) - float(dt_w.get())) / 2.0)
                margin_side_var.set(f"{ms:.3f}")
                ms_r = max(0.0, (float(rs_w.get()) - float(dt_w.get())) / 2.0)
                margin_side_r_var.set(f"{ms_r:.3f}")
            except Exception:
                pass
        
        def _extents():
            ow, ol, ox, oy = _outer_vals("normal")
            rw, rl, r_ox, r_oy = _outer_vals("reshape")
            dw, dl = _dt_vals()
            def span(w, l, offx, offy):
                return (abs(offy) + w * 0.5, abs(offx) + l * 0.5)
            sx = max(span(ow, ol, ox, oy)[0], span(rw, rl, r_ox, r_oy)[0], dw * 0.5)
            sy = max(span(ow, ol, ox, oy)[1], span(rw, rl, r_ox, r_oy)[1], dl * 0.5)
            return sx, sy
        
        def draw():
            canvas.delete("all")
            handles.clear()
            base_cx = base_cy = canvas_size / 2
            vis_state["base_c"] = (base_cx, base_cy)
            sx, sy = _extents()
            # Constrain view to a fixed 36x36 in window (matches field scale)
            view_in = 36.0
            scale = (canvas_size * 0.90) / view_in if view_in > 0 else 10.0
            vis_state["scale"] = scale
            
            mode = mode_var.get()
            ow, ol, ox, oy = _outer_vals(mode)
            dw, dl = _dt_vals()
            
            outer_cx = base_cx - oy * scale
            outer_cy = base_cy - ox * scale
            vis_state["outer_c"] = (outer_cx, outer_cy)
            
            # Grid background (1 in spacing), 36x36 in region centered on canvas
            spacing = max(2, int(round(scale)))
            start = int(base_cx - (view_in/2.0)*scale)
            end = int(base_cx + (view_in/2.0)*scale)
            for gx in range(start, end + spacing, spacing):
                fill = "#e8e8e8" if ((gx-start) // spacing) % 5 else "#d0d0d0"
                canvas.create_line(gx, start, gx, end, fill=fill)
            for gy in range(start, end + spacing, spacing):
                fill = "#e8e8e8" if ((gy-start) // spacing) % 5 else "#d0d0d0"
                canvas.create_line(start, gy, end, gy, fill=fill)
            canvas.create_rectangle(start, start, end, end, outline="#cccccc")
            canvas.create_oval(base_cx-4, base_cy-4, base_cx+4, base_cy+4, fill="#555555", outline="#222222")
            
            # Outer box
            x0 = outer_cx - (ow * 0.5 * scale)
            x1 = outer_cx + (ow * 0.5 * scale)
            y0 = outer_cy - (ol * 0.5 * scale)
            y1 = outer_cy + (ol * 0.5 * scale)
            canvas.create_rectangle(x0, y0, x1, y1, outline="#3c6cc9", width=3, fill="#dfe9f7")
            canvas.create_text((x0 + x1) / 2, y0 - 12, text=f"W: {ow:.2f} in", fill="#1f4b8f")
            canvas.create_text(x1 + 50, (y0 + y1) / 2, text=f"L: {ol:.2f} in", angle=90, fill="#1f4b8f")
            
            # Drivetrain box (always at base center)
            dx0 = base_cx - (dw * 0.5 * scale)
            dx1 = base_cx + (dw * 0.5 * scale)
            dy0 = base_cy - (dl * 0.5 * scale)
            dy1 = base_cy + (dl * 0.5 * scale)
            canvas.create_rectangle(dx0, dy0, dx1, dy1, outline="#666666", width=2, dash=(4,2), fill="")
            canvas.create_text(dx0 - 40, (dy0 + dy1) / 2, text=f"Track {dw:.2f}", angle=90, fill="#444444")
            
            # Handles for outer
            handle_color = "#2b5fad"
            for (hx, hy, kind) in [
                (x0, outer_cy, "outer_left"),
                (x1, outer_cy, "outer_right"),
                (outer_cx, y0, "outer_top"),
                (outer_cx, y1, "outer_bottom"),
            ]:
                handles.append((hx, hy, kind))
                canvas.create_oval(hx-6, hy-6, hx+6, hy+6, fill=handle_color, outline="#0f305f")
            
            # Handles for drivetrain
            d_color = "#444444"
            for (hx, hy, kind) in [
                (dx0, base_cy, "drive_left"),
                (dx1, base_cy, "drive_right"),
                (base_cx, dy0, "drive_top"),
                (base_cx, dy1, "drive_bottom"),
            ]:
                handles.append((hx, hy, kind))
                canvas.create_rectangle(hx-5, hy-5, hx+5, hy+5, outline=d_color, fill="#e6e6e6")
            
            # Info label
            margin_left = max(0.0, ow / 2.0 + oy - dw / 2.0)
            margin_right = max(0.0, ow / 2.0 - oy - dw / 2.0)
            margin_back = max(0.0, ol / 2.0 - ox - dl / 2.0)
            margin_front = max(0.0, ol / 2.0 + ox - dl / 2.0)
            _sync_margin_vars(mode)
            info_var.set(f"Margins L/R: {margin_left:.3f} / {margin_right:.3f} in | Front/Back: {margin_front:.3f} / {margin_back:.3f} in")
        
        def _nearest_handle(x, y):
            if not handles:
                return None
            best = None
            best_d2 = 1e9
            for hx, hy, kind in handles:
                d2 = (hx - x) ** 2 + (hy - y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = (hx, hy, kind)
            if best_d2 <= 18**2:
                return best
            return None
        
        def _apply_drag(kind, event):
            scale = vis_state["scale"]
            base_cx, base_cy = vis_state["base_c"]
            ocx, ocy = vis_state["outer_c"]
            if scale <= 1e-6:
                return
            mode = mode_var.get()
            ow, ol, ox, oy = _outer_vals(mode)
            dw, dl = _dt_vals()
            x, y = event.x, event.y
            
            if kind == "outer_left":
                new_w = max(MIN_SIZE, 2.0 * max(0.1, (ocx - x) / scale))
                new_w = round(new_w / STEP_IN) * STEP_IN
                _set_outer(w=new_w, active_mode=mode)
            elif kind == "outer_right":
                new_w = max(MIN_SIZE, 2.0 * max(0.1, (x - ocx) / scale))
                new_w = round(new_w / STEP_IN) * STEP_IN
                _set_outer(w=new_w, active_mode=mode)
            elif kind == "outer_top":
                new_l = max(MIN_SIZE, 2.0 * max(0.1, (ocy - y) / scale))
                new_l = round(new_l / STEP_IN) * STEP_IN
                _set_outer(l=new_l, active_mode=mode)
            elif kind == "outer_bottom":
                new_l = max(MIN_SIZE, 2.0 * max(0.1, (y - ocy) / scale))
                new_l = round(new_l / STEP_IN) * STEP_IN
                _set_outer(l=new_l, active_mode=mode)
            elif kind == "drive_left":
                new_w = max(MIN_SIZE, 2.0 * max(0.1, (base_cx - x) / scale))
                new_w = round(new_w / STEP_IN) * STEP_IN
                _set_dt(w=new_w)
            elif kind == "drive_right":
                new_w = max(MIN_SIZE, 2.0 * max(0.1, (x - base_cx) / scale))
                new_w = round(new_w / STEP_IN) * STEP_IN
                _set_dt(w=new_w)
            elif kind == "drive_top":
                new_l = max(MIN_SIZE, 2.0 * max(0.1, (base_cy - y) / scale))
                new_l = round(new_l / STEP_IN) * STEP_IN
                _set_dt(l=new_l)
            elif kind == "drive_bottom":
                new_l = max(MIN_SIZE, 2.0 * max(0.1, (y - base_cy) / scale))
                new_l = round(new_l / STEP_IN) * STEP_IN
                _set_dt(l=new_l)
            draw()
        
        def on_press(event):
            hit = _nearest_handle(event.x, event.y)
            drag_state["target"] = hit[2] if hit else None
        
        def on_motion(event):
            if drag_state.get("target"):
                _apply_drag(drag_state["target"], event)
        
        def on_release(_event):
            drag_state["target"] = None
        
        canvas.bind("<Button-1>", on_press)
        canvas.bind("<B1-Motion>", on_motion)
        canvas.bind("<ButtonRelease-1>", on_release)
        
        win.protocol("WM_DELETE_WINDOW", lambda: win.destroy())
        _set_margin_mode()
        draw()
        # Ensure latest values persisted when closing
        def _close_and_apply():
            try:
                _apply_margins()
            except Exception:
                pass
            try:
                on_update()
            except Exception:
                pass
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", _close_and_apply)
    
    offset_win = {"win": None}
    def open_offset_visualizer():
        """Visual editor for offsets along drivetrain centerline."""
        if offset_win["win"] is not None and offset_win["win"].winfo_exists():
            offset_win["win"].lift()
            offset_win["win"].focus_force()
            return
        
        win = tk.Toplevel(top)
        offset_win["win"] = win
        win.title("Offset Visualizer")
        win.resizable(False, False)
        
        canvas_size = 320
        frame = ttk.Frame(win, padding=6)
        frame.pack(fill="both", expand=True)
        canvas = tk.Canvas(frame, width=canvas_size, height=canvas_size, background="#f8f8f8",
                           highlightthickness=1, highlightbackground="#cccccc")
        canvas.grid(row=0, column=0, rowspan=8, sticky="nsew", padx=(0, 10))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        
        mode_var = tk.StringVar(value="normal")
        normal_btn = ttk.Radiobutton(frame, text="Normal view", variable=mode_var, value="normal")
        normal_btn.grid(row=0, column=1, sticky="w")
        reshape_btn = ttk.Radiobutton(frame, text="Reshape view", variable=mode_var, value="reshape")
        reshape_btn.grid(row=1, column=1, sticky="w", pady=(0,6))
        
        offset_sel = tk.StringVar(value="1")
        ttk.Label(frame, text="Target offset:").grid(row=2, column=1, sticky="w")
        sel_frame = ttk.Frame(frame)
        sel_frame.grid(row=3, column=1, sticky="w", pady=(0,6))
        off1_btn = ttk.Radiobutton(sel_frame, text="Offset 1", variable=offset_sel, value="1")
        off1_btn.pack(side="left")
        off2_btn = ttk.Radiobutton(sel_frame, text="Offset 2", variable=offset_sel, value="2")
        off2_btn.pack(side="left", padx=(6,0))
        custom_offset_var = tk.StringVar(value="0.0")
        custom_btn = ttk.Radiobutton(sel_frame, text="Custom", variable=offset_sel, value="custom")
        custom_btn.pack(side="left", padx=(6,0))
        ttk.Entry(frame, textvariable=custom_offset_var, width=10).grid(row=4, column=1, sticky="w")
        
        val_label = ttk.Label(frame, text="Offset: 0.000 in")
        val_label.grid(row=5, column=1, sticky="w", pady=(4,0))
        ttk.Label(frame, text="Drag the node along the centerline.").grid(row=6, column=1, sticky="w")
        reverse_var = tk.IntVar(value=0)
        ttk.Checkbutton(frame, text="Reverse (mirror over centerline)", variable=reverse_var, command=lambda: draw()).grid(row=7, column=1, sticky="w", pady=(4,0))
        
        view_in = 36.0
        scale = (canvas_size * 0.90) / view_in if view_in > 0 else 8.0
        node_pos_px = [canvas_size/2, canvas_size/2]
        
        def _dims(mode):
            try:
                dw = float(dt_w.get()); dl = float(dt_l.get())
            except Exception:
                dw = dl = 0.0
            if mode == "reshape":
                try:
                    ow = float(rs_w.get()); ol = float(rs_l.get())
                    ox = float(rs_off_x.get()); oy = float(rs_off_y.get())
                except Exception:
                    ow = ol = ox = oy = 0.0
            else:
                try:
                    ow = float(w_full.get()); ol = float(l_full.get())
                    ox = float(full_off_x.get()); oy = float(full_off_y.get())
                except Exception:
                    ow = ol = ox = oy = 0.0
            return dw, dl, ow, ol, ox, oy
        
        def _current_offset():
            sel = offset_sel.get()
            try:
                if sel == "1":
                    return float(off1.get())
                if sel == "2":
                    return float(off2.get())
                return float(custom_offset_var.get())
            except Exception:
                return 0.0
        
        def _set_offset(val):
            sel = offset_sel.get()
            if sel == "1":
                off1.set(f"{val:.3f}")
            elif sel == "2":
                off2.set(f"{val:.3f}")
            else:
                custom_offset_var.set(f"{val:.3f}")
            val_label.config(text=f"Offset: {val:.3f} in")
        
        def draw():
            canvas.delete("all")
            mode = mode_var.get()
            dw, dl, ow, ol, ox, oy = _dims(mode)
            rev = bool(reverse_var.get())
            # ensure visible defaults if inputs are empty/zero (match geometry editor)
            try:
                bd = CFG.get("bot_dimensions", {})
            except Exception:
                bd = {}
            if dw <= 0.0:
                dw = float(bd.get("dt_width", bd.get("width", 12.0)) or 12.0)
            if dl <= 0.0:
                dl = float(bd.get("dt_length", bd.get("length", 12.0)) or 12.0)
            if ow <= 0.0:
                ow = max(dw + 1.0, float(bd.get("width", dw + 2.0)) or (dw + 2.0))
            if ol <= 0.0:
                ol = max(dl + 1.0, float(bd.get("length", dl + 2.0)) or (dl + 2.0))
            # Clamp offsets to keep robot visible in 36x36 view
            max_off_y = max(0.0, (view_in / 2.0) - (ow / 2.0))
            max_off_x = max(0.0, (view_in / 2.0) - (ol / 2.0))
            oy = max(-max_off_y, min(max_off_y, oy))
            ox = max(-max_off_x, min(max_off_x, ox))
            base_cx = base_cy = canvas_size / 2
            # grid
            spacing = max(2, int(round(scale)))
            start = max(0, int(base_cx - (view_in/2.0)*scale))
            end = min(canvas_size, int(base_cx + (view_in/2.0)*scale))
            for gx in range(start, end + spacing, spacing):
                fill = "#e8e8e8" if ((gx-start) // spacing) % 5 else "#d0d0d0"
                canvas.create_line(gx, start, gx, end, fill=fill)
            for gy in range(start, end + spacing, spacing):
                fill = "#e8e8e8" if ((gy-start) // spacing) % 5 else "#d0d0d0"
                canvas.create_line(start, gy, end, gy, fill=fill)
            canvas.create_rectangle(start, start, end, end, outline="#cccccc")
            
            outer_cx = base_cx - oy * scale
            outer_cy = base_cy - ox * scale
            if rev:
                outer_cy = base_cy + ox * scale
            # outer
            x0 = outer_cx - (ow * 0.5 * scale)
            x1 = outer_cx + (ow * 0.5 * scale)
            y0 = outer_cy - (ol * 0.5 * scale)
            y1 = outer_cy + (ol * 0.5 * scale)
            canvas.create_rectangle(x0, y0, x1, y1, outline="#3c6cc9", width=2, fill="#e6eefb")
            # drivetrain
            dx0 = base_cx - (dw * 0.5 * scale)
            dx1 = base_cx + (dw * 0.5 * scale)
            dy0 = base_cy - (dl * 0.5 * scale)
            dy1 = base_cy + (dl * 0.5 * scale)
            if rev:
                dy0, dy1 = base_cy - (dl * 0.5 * scale), base_cy + (dl * 0.5 * scale)
            canvas.create_rectangle(dx0, dy0, dx1, dy1, outline="#444444", dash=(4,2), fill="#f7f7f7")
            # node pos (offset always along drivetrain centerline; node not mirrored)
            val = _current_offset()
            node_pos_px[0] = base_cx
            node_pos_px[1] = base_cy - val * scale
            # clamp to view bounds
            node_pos_px[1] = min(end, max(start, node_pos_px[1]))
            canvas.create_line(base_cx, start, base_cx, end, fill="#bbbbbb", dash=(3,3))
            canvas.create_oval(node_pos_px[0]-7, node_pos_px[1]-7, node_pos_px[0]+7, node_pos_px[1]+7, fill="#2b5fad", outline="#0f305f")
            val_label.config(text=f"Offset: {val:.3f} in")
        
        normal_btn.config(command=draw)
        reshape_btn.config(command=draw)
        off1_btn.config(command=draw)
        off2_btn.config(command=draw)
        custom_btn.config(command=draw)
        
        def _on_drag(event):
            base_cy = canvas_size / 2
            val = (base_cy - event.y) / scale
            # clamp to visible window
            max_abs = view_in / 2.0
            val = max(-max_abs, min(max_abs, val))
            _set_offset(val)
            draw()
        
        canvas.bind("<B1-Motion>", _on_drag)
        canvas.bind("<Button-1>", _on_drag)
        custom_offset_var.trace_add("write", lambda *_: draw())
        
        def _close_offset():
            try:
                on_update()
            except Exception:
                pass
            win.destroy()
        win.protocol("WM_DELETE_WINDOW", _close_offset)
        draw()
    
    def _heading_for_node0_to1():
        """Return heading from node 0 toward node 1, following the path tangent if present."""
        if len(display_nodes) < 2:
            return None
        p0 = effective_node_pos(0)
        p1 = effective_node_pos(1)
        reverse = bool(display_nodes[0].get("reverse", False))

        path_pts = None
        pose_hdg = None

        # Prefer live-edited control points when editing the 0->1 segment
        if path_edit_mode and path_edit_segment_idx == 0 and path_control_points:
            cps = list(path_control_points)
            if len(cps) >= 2:
                cps[0] = p0
                cps[-1] = p1
                try:
                    path_pts = generate_bezier_path(cps, num_samples=50)
                except Exception:
                    path_pts = cps
        else:
            pd = display_nodes[0].get("path_to_next", {})
            pose_pts = pd.get("pose_preview_points") or []
            if pose_pts and len(pose_pts) > 1:
                path_pts = list(pose_pts)
                path_pts[0] = p0
                path_pts[-1] = p1
            elif pd.get("use_path", False):
                pts = pd.get("path_points") or []
                if pts and len(pts) > 1:
                    path_pts = list(pts)
                    path_pts[0] = p0
                    path_pts[-1] = p1
                else:
                    cps = list(pd.get("control_points") or [])
                    if len(cps) >= 2:
                        cps[0] = p0
                        cps[-1] = p1
                        try:
                            path_pts = generate_bezier_path(cps, num_samples=50)
                        except Exception:
                            path_pts = cps
            elif display_nodes[0].get("move_to_pose"):
                pose_heading = display_nodes[0].get("pose_heading_deg")
                pose_lead = display_nodes[0].get("pose_lead_in")
                if pose_heading is None:
                    pose_heading = heading_from_points(p0, p1)
                lead_in = 0.0
                try:
                    if pose_lead is not None:
                        lead_in = max(0.0, float(pose_lead))
                except Exception:
                    lead_in = 0.0
                if lead_in > 0.0:
                    dist_px = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                    if dist_px > 1e-6:
                        end_heading = pose_heading
                        if reverse:
                            end_heading = (end_heading + 180.0) % 360.0
                        disp_heading = convert_heading_input(end_heading, None)
                        th = math.radians(disp_heading)
                        carrot = (
                            p1[0] + math.cos(th) * dist_px * lead_in,
                            p1[1] + math.sin(th) * dist_px * lead_in
                        )
                        pose_hdg = heading_from_points(p0, carrot)

        if path_pts and len(path_pts) > 1:
            try:
                hdg = calculate_path_heading(path_pts, 0)
            except Exception:
                hdg = heading_from_points(path_pts[0], path_pts[1])
        elif pose_hdg is not None:
            hdg = pose_hdg
        else:
            hdg = heading_from_points(p0, p1)
    
        if reverse:
            hdg = (hdg + 180.0) % 360.0
        return hdg
    
    def _flip_routine_horizontal():
        """Mirror routine horizontally."""
        global display_nodes, robot_pos, robot_heading, initial_state
        global undo_stack, last_snapshot, last_path_sig, total_estimate_s
        global moving, paused, show_chevron, timeline, seg_i, t_local, last_logged_seg, reshape_live
        
        if not display_nodes:
            return
        
        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
        cx = WINDOW_WIDTH / 2.0
        
        for node in display_nodes:
            x, y = node["pos"]
            node["pos"] = (2 * cx - x, y)
            # Mirror any path control points originating from this node
            pd = node.get("path_to_next", {})
            cps = pd.get("control_points")
            if cps:
                pd["control_points"] = [(2 * cx - cp[0], cp[1]) for cp in cps]
        
        initial_state["position"] = display_nodes[0]["pos"]
        initial_state["heading"] = (180.0 - initial_state["heading"]) % 360.0
        robot_pos = initial_state["position"]
        robot_heading = initial_state["heading"]
        
        for node in display_nodes:
            if node.get("pose_heading_deg") is not None:
                try:
                    node["pose_heading_deg"] = (180.0 - float(node["pose_heading_deg"])) % 360.0
                except Exception:
                    pass
            if node.get("swing_target_heading_deg") is not None:
                try:
                    node["swing_target_heading_deg"] = (180.0 - float(node["swing_target_heading_deg"])) % 360.0
                except Exception:
                    pass
            sd = str(node.get("swing_dir", "auto")).lower()
            if sd in ("cw", "ccw"):
                node["swing_dir"] = "ccw" if sd == "cw" else "cw"
            for act in node.get("actions", []):
                if act.get("type") in ("turn", "swing"):
                    try:
                        ang = float(act.get("deg", 0.0))
                    except Exception:
                        continue
                    act["deg"] = (180.0 - ang) % 360.0
                if act.get("type") == "swing":
                    ad = str(act.get("dir", "auto")).lower()
                    if ad in ("cw", "ccw"):
                        act["dir"] = "ccw" if ad == "cw" else "cw"
        
        try:
            correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
            sync_all_path_endpoints()
        except Exception:
            pass
        
        util_push_undo_prev(undo_stack, prev)
        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
        last_path_sig = None
        moving = False
        paused = False
        show_chevron = False
        timeline.clear()
        seg_i = 0
        t_local = 0.0
        last_logged_seg = -1
        reshape_live = False
        total_estimate_s = compute_total_estimate_s()
        
    # Heading entry with Node 1 helper
    heading_frame = ttk.Frame(tabs["general"])
    heading_entry = ttk.Entry(heading_frame, textvariable=init_head, width=10)
    heading_entry.pack(side="left", fill="x", expand=True)
    
    def _heading_to_node1():
        auto_heading_node1_var.set(1)
        try:
            hdg = _heading_for_node0_to1()
            if hdg is not None:
                disp = convert_heading_input(hdg, None)
                init_head.set(f"{disp:.3f}")
            on_update()
        finally: 
            auto_heading_node1_var.set(0)
    
    heading_node1_btn = ttk.Button(heading_frame, text="→ Node 1", command=_heading_to_node1)
    heading_node1_btn.pack(side="left", padx=(6, 0))
    
    # General tab rows
    _row(tabs["general"], 0, "Distance Units:", ttk.Combobox(tabs["general"], textvariable=dist_var, values=dist_labels, state="readonly"), "Output units")
    _row(tabs["general"], 1, "Angle Units:", ttk.Combobox(tabs["general"], textvariable=ang_var, values=ang_labels, state="readonly"), "Angle units")
    _row(tabs["general"], 2, "Initial heading (deg):", heading_frame, "Robot start heading")
    _row(tabs["general"], 3, "Show node hitboxes:", ttk.Checkbutton(tabs["general"], variable=show_hitboxes_var), "Toggle geometry boxes")
    field_obj_row = ttk.Frame(tabs["general"])
    field_obj_chk = ttk.Checkbutton(field_obj_row, variable=show_field_objects_var)
    field_obj_chk.pack(side="left")
    node_nums_lbl = ttk.Label(field_obj_row, text="Node numbers")
    node_nums_lbl.pack(side="left", padx=(10, 4))
    node_nums_chk = ttk.Checkbutton(field_obj_row, variable=show_node_numbers_var)
    node_nums_chk.pack(side="left")
    ui.track_live_widget(field_obj_chk)
    ui.track_live_widget(node_nums_chk)
    _row(tabs["general"], 4, "Show field objects:", field_obj_row, "Draw field objects / toggle node labels")
    _row(tabs["general"], 5, "Reshape label:", ttk.Entry(tabs["general"], textvariable=reshape_label_var), "Custom label used anywhere reshape appears")
    _row(tabs["general"], 6, "Flip routine:", ttk.Button(tabs["general"], text="Flip", command=_flip_routine_horizontal), "Mirror horizontally")
    
    # Physics tab scrollable container
    physics_tab = tabs["physics"]
    physics_canvas = tk.Canvas(physics_tab, borderwidth=0, highlightthickness=0)
    physics_scroll = ttk.Scrollbar(physics_tab, orient="vertical", command=physics_canvas.yview)
    physics_canvas.configure(yscrollcommand=physics_scroll.set)
    physics_body = ttk.Frame(physics_canvas)
    physics_win = physics_canvas.create_window((0, 0), window=physics_body, anchor="nw")
    physics_tab.rowconfigure(0, weight=1)
    physics_tab.columnconfigure(0, weight=1)
    physics_tab.columnconfigure(1, weight=0)
    physics_body.columnconfigure(0, weight=1)
    physics_body.columnconfigure(1, weight=1)
    physics_canvas.grid(row=0, column=0, sticky="nsew")
    physics_scroll.grid(row=0, column=1, sticky="ns")

    def _physics_scroll_active():
        return bool(adv_motion_var.get())

    def _refresh_physics_scrollbar():
        try:
            needs = _physics_scroll_active()
            bbox = physics_canvas.bbox("all")
            content_h = (bbox[3] - bbox[1]) if bbox else 0
            canvas_h = physics_canvas.winfo_height()
            show = needs
            if show:
                physics_scroll.grid()
            else:
                physics_scroll.grid_remove()
        except Exception:
            pass

    def _physics_on_configure(_evt=None):
        try:
            physics_canvas.configure(scrollregion=physics_canvas.bbox("all"))
            physics_canvas.itemconfigure(physics_win, width=physics_canvas.winfo_width())
        except Exception:
            pass
        _refresh_physics_scrollbar()

    def _physics_mousewheel(event):
        if not _physics_scroll_active():
            return
        delta = 0
        if hasattr(event, "delta") and event.delta:
            delta = int(-event.delta / 40) if abs(event.delta) >= 40 else (-2 if event.delta > 0 else 2)
        else:
            num = getattr(event, "num", None)
            if num == 4:
                delta = -1
            elif num == 5:
                delta = 1
        if delta:
            physics_canvas.yview_scroll(delta, "units")
            return "break"

    physics_body.bind("<Configure>", _physics_on_configure)
    physics_canvas.bind("<Configure>", _physics_on_configure)
    for w in (physics_canvas, physics_body):
        w.bind("<MouseWheel>", _physics_mousewheel)
        w.bind("<Button-4>", _physics_mousewheel)
        w.bind("<Button-5>", _physics_mousewheel)

    # Physics tab rows
    _row(physics_body, 0, "Drive RPM:", ttk.Entry(physics_body, textvariable=rpm_var), "Affects vmax and accel")
    _row(physics_body, 1, "Wheel diameter (in):", ttk.Entry(physics_body, textvariable=diam_var), "Wheel size")
    _row(physics_body, 2, "Robot weight (lb):", ttk.Entry(physics_body, textvariable=w_var), "Robot mass")
    _row(physics_body, 3, "Buffer Time (s):", ttk.Entry(physics_body, textvariable=tb_var), "Pause per node")
    _row(physics_body, 4, "Gear ratio:", ttk.Entry(physics_body, textvariable=gr_var), "Motor:wheel ratio")
    _row(physics_body, 5, "Point density (/in):", ttk.Entry(physics_body, textvariable=dens_var), "Uniform resample density for curved paths")
    curv_disp = tk.StringVar(value=f"{curv_var.get():.3f}")
    def _curv_live(_=None):
        try:
            val = float(curv_var.get())
            CFG["robot_physics"]["curvature_gain"] = val
            curv_disp.set(f"{val:.3f}")
        except Exception:
            pass
    curv_frame = ttk.Frame(physics_body)
    curv_scale = ttk.Scale(curv_frame, orient="horizontal", from_=0.0, to=1.0, variable=curv_var, command=lambda _evt=None: _curv_live(), length=180)
    curv_scale.pack(side="left", padx=(0,6))
    curv_label = ttk.Label(curv_frame, textvariable=curv_disp, width=7)
    curv_label.pack(side="left")
    _row(physics_body, 6, "Curvature gain:", curv_frame, "Higher = stronger slow-down in tight turns (live)")

    constants_win = {"win": None}

    def _phys_const_defaults():
        defaults = DEFAULT_CONFIG.get("physics_constants", {})
        out = {}
        for key, val in defaults.items():
            if isinstance(val, dict):
                val = val.get("value", val)
            out[key] = val
        return out

    PHYS_CONST_FIELDS = [
        ("load_factor", "Load factor", "Scales free-speed (rpm*diam) to a realistic loaded top speed."),
        ("accel_mu_scale", "Accel mu scale", "Scales the traction-limited accel (mu*g); higher = more accel."),
        ("t_to_v_base", "t->v base (s)", "Baseline seconds to reach vmax at 12V; actual weight scales it."),
        ("turn_rate_scale", "Turn rate scale", "Scales turn rate derived from vmax/track width."),
        ("omni_scale", "Omni scale", "Multiplier on accel for all-omni drivetrains (lower = less accel).")
    ]

    def _derive_cal_constants_from_dynamics(dyn: dict) -> dict:
        if not isinstance(dyn, dict) or not dyn:
            return {}
        rp = CFG.get("robot_physics", {})
        bd = CFG.get("bot_dimensions", {})
        rpm = float(rp.get("rpm", 200.0))
        d_in = float(rp.get("diameter", 4.0))
        v_straight = float(rp.get("volts_straight", 12.0))
        v_turn = float(rp.get("volts_turn", 12.0))
        wlb = float(rp.get("weight", 20.0))
        mu = float(rp.get("mu", 0.9))
        max_cmd = float(rp.get("max_cmd", 127.0))
        omni = float(rp.get("all_omni", 0))
        gear_ratio = float(CFG.get("gear_ratio", 1.0) or 1.0)
        track = max(6.0, float(bd.get("dt_width", bd.get("width", 12.0))))
        phys_defaults = DEFAULT_CONFIG.get("physics_constants", {})
        accel_mu_scale = CFG.get("physics_constants", {}).get("accel_mu_scale", phys_defaults.get("accel_mu_scale", {"value": 0.95}))
        if isinstance(accel_mu_scale, dict):
            accel_mu_scale = accel_mu_scale.get("value", 0.95)
        try:
            accel_mu_scale = float(accel_mu_scale)
        except Exception:
            accel_mu_scale = 0.95
        vmax = dyn.get("vmax_ips")
        accel = dyn.get("accel_ips2")
        turn_rate = dyn.get("turn_rate_dps")
        constants = {}

        try:
            free = rpm * math.pi * max(0.01, d_in) / 60.0
            scale = max(0.0, min(127.0, max_cmd)) / 127.0
            denom = free * (v_straight / 12.0) * scale
            if vmax is not None and denom > 1e-6:
                load_factor = float(vmax) / denom
                constants["load_factor"] = max(0.3, min(1.2, load_factor))
        except Exception:
            pass

        try:
            if vmax is not None and accel is not None:
                a_t = accel_mu_scale * mu * 386.09
                omni_scale = CFG.get("physics_constants", {}).get("omni_scale", phys_defaults.get("omni_scale", {"value": 0.9}))
                if isinstance(omni_scale, dict):
                    omni_scale = omni_scale.get("value", 0.9)
                try:
                    omni_scale = float(omni_scale)
                except Exception:
                    omni_scale = 0.9
                a_t_eff = a_t * (omni_scale if omni else 1.0)
                if float(accel) < 0.95 * a_t_eff:
                    mass_scale = max(0.7, min(1.6, math.sqrt(max(1e-6, wlb) / 15.0)))
                    torque_scale = max(0.5, min(2.0, gear_ratio))
                    denom = max(1e-6, torque_scale * (omni_scale if omni else 1.0))
                    a_m = float(accel) / denom
                    t_base = (float(vmax) / max(1e-6, a_m)) / max(1e-6, mass_scale)
                    constants["t_to_v_base"] = max(0.08, min(1.2, t_base))
        except Exception:
            pass

        try:
            if turn_rate is not None and vmax is not None:
                base_deg = (2.0 * float(vmax)) / max(1e-6, track) * (180.0 / math.pi)
                denom = base_deg * (v_turn / 12.0)
                if denom > 1e-6:
                    tr_scale = float(turn_rate) / denom
                    constants["turn_rate_scale"] = max(0.4, min(1.3, tr_scale))
        except Exception:
            pass

        return constants

    def _open_physics_constants_editor():
        if constants_win["win"] is not None and constants_win["win"].winfo_exists():
            constants_win["win"].lift()
            constants_win["win"].focus_force()
            return

        win = tk.Toplevel(top)
        constants_win["win"] = win
        win.title("Physics Constants")
        win.geometry("300x450")
        win.resizable(True, True)

        body = ttk.Frame(win, padding=10)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)

        ttk.Label(
            body,
            text="These constants tune the physics time estimator used for timeline and timeouts.",
            foreground="#555555",
            wraplength=260,
            justify="left"
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        lock_var = tk.StringVar(value="")
        lock_lbl = ttk.Label(body, textvariable=lock_var, foreground="#b00020", wraplength=260, justify="left")
        lock_lbl.grid(row=1, column=0, columnspan=2, sticky="w", pady=(0, 8))

        dyn_frame = ttk.LabelFrame(body, text="Calibration dynamics (read-only)")
        dyn_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 10))
        dyn_frame.columnconfigure(1, weight=1)
        dyn_vars = {
            "vmax_ips": tk.StringVar(value="n/a"),
            "accel_ips2": tk.StringVar(value="n/a"),
            "turn_rate_dps": tk.StringVar(value="n/a"),
            "turn_accel_dps2": tk.StringVar(value="n/a")
        }
        dyn_rows = [
            ("vmax_ips", "Drive vmax (ips):"),
            ("accel_ips2", "Drive accel (ips^2):"),
            ("turn_rate_dps", "Turn rate (deg/s):"),
            ("turn_accel_dps2", "Turn accel (deg/s^2):")
        ]
        for i, (key, label) in enumerate(dyn_rows):
            ttk.Label(dyn_frame, text=label).grid(row=i, column=0, sticky="w", padx=6, pady=2)
            ttk.Label(dyn_frame, textvariable=dyn_vars[key]).grid(row=i, column=1, sticky="w", padx=6, pady=2)

        const_frame = ttk.LabelFrame(body, text="Physics constants (heuristic model)")
        const_frame.grid(row=3, column=0, columnspan=2, sticky="nsew")
        const_frame.columnconfigure(1, weight=1)

        const_vars = {}
        const_entries = {}
        defaults = _phys_const_defaults()
        current = CFG.get("physics_constants", {})
        for idx, (key, label, desc) in enumerate(PHYS_CONST_FIELDS):
            ttk.Label(const_frame, text=label).grid(row=idx, column=0, sticky="w", padx=6, pady=2)
            val = current.get(key, defaults.get(key))
            try:
                val = float(val)
            except Exception:
                val = defaults.get(key, 0.0)
            var = tk.StringVar(value=str(val))
            entry = ttk.Entry(const_frame, textvariable=var, width=12)
            entry.grid(row=idx, column=1, sticky="w", padx=6, pady=2)
            const_vars[key] = var
            const_entries[key] = entry
            ui.add_tooltip(entry, desc)

        def _load_const_values(source, fallback=None):
            src = source if isinstance(source, dict) else {}
            fb = fallback if isinstance(fallback, dict) else {}
            for key, var in const_vars.items():
                val = src.get(key, fb.get(key, defaults.get(key, 0.0)))
                if isinstance(val, dict):
                    val = val.get("value", defaults.get(key, 0.0))
                try:
                    var.set(str(float(val)))
                except Exception:
                    var.set(str(defaults.get(key, 0.0)))

        _load_const_values(current)

        btns = ttk.Frame(body)
        btns.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        btns.columnconfigure(0, weight=1)
        reset_btn = ttk.Button(btns, text="Reset to defaults")
        reset_btn.pack(side="left")
        apply_btn = ttk.Button(btns, text="Apply")
        apply_btn.pack(side="right")
        ttk.Button(btns, text="Close", command=win.destroy).pack(side="right", padx=(0, 6))

        def _refresh_dyn_labels():
            cal = CFG.get("codegen", {}).get("calibration", {})
            dyn = cal.get("dynamics", {}) if isinstance(cal, dict) else {}
            use_estimate = False
            cal_enabled = bool(cal_enabled_var.get())
            if not cal_enabled:
                dyn = {}
            if not isinstance(dyn, dict) or not dyn:
                try:
                    dyn = {
                        "vmax_ips": float(vmax_straight(CFG)),
                        "accel_ips2": float(accel_straight(CFG)),
                        "turn_rate_dps": float(turn_rate(CFG)),
                        "turn_accel_dps2": float(_turn_accel_deg_s2(CFG))
                    }
                    use_estimate = cal_enabled
                except Exception:
                    dyn = {}
            alias_map = {
                "vmax_ips": ["vmax_ips", "vmax", "drive_vmax", "vmax_in_s", "vmax_inps"],
                "accel_ips2": ["accel_ips2", "accel", "drive_accel", "accel_in_s2", "accel_inps2"],
                "turn_rate_dps": ["turn_rate_dps", "turn_rate", "turn_dps", "turn_rate_deg_s"],
                "turn_accel_dps2": ["turn_accel_dps2", "turn_accel", "turn_accel_deg_s2", "turn_accel_deg_s"]
            }

            def _to_float(raw):
                if isinstance(raw, dict):
                    raw = raw.get("value", None)
                try:
                    return float(raw)
                except Exception:
                    return None

            def _get_dyn_val(key):
                for alias in alias_map.get(key, [key]):
                    if alias in dyn:
                        val = _to_float(dyn.get(alias))
                        if val is not None:
                            return val
                return None

            for key in dyn_vars:
                val = _get_dyn_val(key)
                if val is None:
                    dyn_vars[key].set("n/a")
                elif use_estimate:
                    dyn_vars[key].set(f"~{val:.3f}")
                else:
                    dyn_vars[key].set(f"{val:.3f}")

        def _refresh_constants_lock(*_):
            locked = bool(cal_enabled_var.get())
            for entry in const_entries.values():
                entry.configure(state="disabled" if locked else "normal")
            reset_btn.configure(state="disabled" if locked else "normal")
            apply_btn.configure(state="disabled" if locked else "normal")
            cal = CFG.get("codegen", {}).get("calibration", {})
            dyn = cal.get("dynamics", {}) if isinstance(cal, dict) else {}
            has_dyn = isinstance(dyn, dict) and bool(dyn)
            cal_consts = cal.get("constants", {}) if isinstance(cal, dict) else {}
            cfg_consts = CFG.get("physics_constants", {})
            if locked:
                if has_dyn:
                    lock_var.set("Calibration is enabled. Physics constants are locked to calibration dynamics.")
                else:
                    lock_var.set("Calibration is enabled, but no dynamics were found. Run wizard to populate.")
                if isinstance(cal_consts, dict) and cal_consts:
                    _load_const_values(cal_consts, cfg_consts)
            else:
                lock_var.set("")
                _load_const_values(cfg_consts)
            _refresh_dyn_labels()

        constants_win["refresh"] = _refresh_constants_lock

        def _reset_defaults():
            defaults = _phys_const_defaults()
            for key, var in const_vars.items():
                var.set(str(defaults.get(key, 0.0)))

        def _apply_constants():
            if cal_enabled_var.get():
                return
            updated = {}
            for key, var in const_vars.items():
                try:
                    updated[key] = float(var.get())
                except Exception:
                    messagebox.showerror("Constants", f"Invalid value for {key}.")
                    return
            CFG["physics_constants"] = updated
            CFG.pop("_phys_cache", None)
            on_update()

        reset_btn.configure(command=_reset_defaults)
        apply_btn.configure(command=_apply_constants)
        _refresh_constants_lock()
        try:
            cal_enabled_var.trace_add("write", _refresh_constants_lock)
        except Exception:
            pass

    omni_frame = ttk.Frame(physics_body)
    omni_chk = ttk.Checkbutton(omni_frame, variable=omni_var)
    omni_chk.pack(side="left")
    constants_btn = ttk.Button(omni_frame, text="Constants", command=_open_physics_constants_editor)
    constants_btn.pack(side="left", padx=(10, 0))
    ui.add_tooltip(constants_btn, "Edit physics constants used for time estimates.")
    ui.track_live_widget(omni_chk)
    ui.track_live_widget(constants_btn)
    _row(physics_body, 7, "All omni wheels:", omni_frame, "Adjust physics limits and settle heuristics for all-omni behavior")

    motion_profile_win = {"win": None}
    def _motion_profile_defaults():
        return {
            "voltage_shapes": copy.deepcopy(VOLTAGE_SHAPES),
            "settle_base": copy.deepcopy(SETTLE_BASE),
            "profile_rules": {
                "drive": {"precise_max": 12.0, "fast_min": 36.0},
                "turn": {"precise_max": 25.0, "fast_min": 120.0},
                "swing": {"precise_max": 25.0, "fast_min": 120.0}
            }
        }

    def _open_motion_profile_editor():
        if motion_profile_win["win"] is not None and motion_profile_win["win"].winfo_exists():
            motion_profile_win["win"].lift()
            motion_profile_win["win"].focus_force()
            return

        win = tk.Toplevel(top)
        motion_profile_win["win"] = win
        win.title("Motion Profile Requirements")
        win.geometry("760x520")

        wrapper = ttk.Frame(win)
        wrapper.pack(fill="both", expand=True)
        profile_canvas = tk.Canvas(wrapper, borderwidth=0, highlightthickness=0)
        profile_scroll = ttk.Scrollbar(wrapper, orient="vertical", command=profile_canvas.yview)
        profile_canvas.configure(yscrollcommand=profile_scroll.set)
        profile_scroll.pack(side="right", fill="y")
        profile_canvas.pack(side="left", fill="both", expand=True)

        body = ttk.Frame(profile_canvas, padding=8)
        body_id = profile_canvas.create_window((0, 0), window=body, anchor="nw")
        body.columnconfigure(0, weight=1)

        def _profile_on_configure(_evt=None):
            try:
                profile_canvas.configure(scrollregion=profile_canvas.bbox("all"))
                profile_canvas.itemconfigure(body_id, width=profile_canvas.winfo_width())
            except Exception:
                pass

        def _profile_mousewheel(event):
            delta = 0
            if hasattr(event, "delta") and event.delta:
                delta = int(-event.delta / 40) if abs(event.delta) >= 40 else (-2 if event.delta > 0 else 2)
            else:
                num = getattr(event, "num", None)
                if num == 4:
                    delta = -1
                elif num == 5:
                    delta = 1
            if delta:
                profile_canvas.yview_scroll(delta, "units")
                return "break"

        body.bind("<Configure>", _profile_on_configure)
        profile_canvas.bind("<Configure>", _profile_on_configure)
        for w in (profile_canvas, body):
            w.bind("<MouseWheel>", _profile_mousewheel)
            w.bind("<Button-4>", _profile_mousewheel)
            w.bind("<Button-5>", _profile_mousewheel)

        frame = body

        help_lbl = ttk.Label(
            frame,
            text="Edit voltage caps and settle defaults used when calibration is off. "
                 "Format is JSON; recommended defaults are provided.",
            foreground="#555555",
            wraplength=720,
            justify="left"
        )
        help_lbl.pack(anchor="w", pady=(0, 8))

        guide_visible = tk.BooleanVar(value=False)
        guide_toggle_row = ttk.Frame(frame)
        guide_toggle_row.pack(fill="x", pady=(0, 8))
        guide_toggle_btn = ttk.Button(guide_toggle_row, text="?", width=3)
        guide_toggle_btn.pack(side="left")
        ui.add_tooltip(guide_toggle_btn, "Show/hide guide")

        guide_box = ttk.LabelFrame(frame, text="Guide")
        guide_body = ttk.Frame(guide_box)
        guide_body.pack(fill="both", expand=True, padx=6, pady=6)
        guide_text = tk.Text(guide_body, height=12, wrap="word", font=("Consolas", 9))
        guide_scroll = ttk.Scrollbar(guide_body, orient="vertical", command=guide_text.yview)
        guide_text.configure(yscrollcommand=guide_scroll.set)
        guide_text.grid(row=0, column=0, sticky="nsew")
        guide_scroll.grid(row=0, column=1, sticky="ns")
        guide_body.columnconfigure(0, weight=1)
        guide_body.rowconfigure(0, weight=1)

        def _toggle_guide():
            if guide_visible.get():
                guide_box.pack_forget()
                guide_visible.set(False)
                guide_toggle_btn.configure(text="?")
            else:
                guide_box.pack(fill="x", pady=(0, 8), before=cap_lock_frame)
                guide_visible.set(True)
                guide_toggle_btn.configure(text="×")

        def _guide_mousewheel(event):
            if not guide_visible.get():
                return
            delta = 0
            if hasattr(event, "delta") and event.delta:
                delta = int(-event.delta / 40) if abs(event.delta) >= 40 else (-2 if event.delta > 0 else 2)
            else:
                num = getattr(event, "num", None)
                if num == 4:
                    delta = -1
                elif num == 5:
                    delta = 1
            if delta:
                guide_text.yview_scroll(delta, "units")
                return "break"

        for w in (guide_text, guide_body):
            w.bind("<MouseWheel>", _guide_mousewheel)
            w.bind("<Button-4>", _guide_mousewheel)
            w.bind("<Button-5>", _guide_mousewheel)
        guide_toggle_btn.configure(command=_toggle_guide)

        guide_text.insert(
            "1.0",
            "How to use:\n"
            "1) Edit the JSON below. Use drive/turn/swing and profiles precise/normal/fast/slam.\n"
            "2) voltage_shapes can be [small, mid, large] volts or {v_small, v_mid, v_large}.\n"
            "3) settle_base uses err_min/err_max (in or deg) and t_min/t_max (ms).\n"
            "4) profile_rules uses inches for drive and degrees for turn/swing.\n"
            "   Keys: precise_max, fast_min, optional slam_min.\n"
            "5) Remove a key to fall back to recommended defaults.\n"
            "\n"
            "Example voltage_shapes:\n"
            "{\n"
            "  \"drive\": {\"precise\": [4.0, 6.0, 8.0]}\n"
            "}\n"
            "Meaning: for precise drive moves, small/medium/large distances cap at 4/6/8 volts.\n"
            "Example settle_base:\n"
            "{\n"
            "  \"turn\": {\"fast\": {\"err_min\": 1.0, \"err_max\": 2.0, \"t_min\": 150, \"t_max\": 280}}\n"
            "}\n"
            "Meaning: fast turns settle between 1-2 deg and 150-280 ms, based on turn size.\n"
            "Example profile_rules:\n"
            "{\n"
            "  \"drive\": {\"precise_max\": 10.0, \"fast_min\": 40.0, \"slam_min\": 80.0}\n"
            "}\n"
            "Meaning: drive distances <10 in use precise, >40 in use fast, >=80 in use slam.\n"
        )
        guide_text.configure(state="disabled")

        single_cap_var = tk.BooleanVar(value=False)
        cap_lock_frame = ttk.Frame(frame)
        cap_lock_frame.pack(fill="x", pady=(0, 8))
        cap_lock_chk = ttk.Checkbutton(cap_lock_frame, text="Single cap per profile", variable=single_cap_var)
        cap_lock_chk.pack(side="left")
        cap_lock_note = ttk.Label(cap_lock_frame, text="Forces small/medium/large caps to match.", foreground="#555555")
        cap_lock_note.pack(side="left", padx=(8, 0))
        ui.add_tooltip(cap_lock_chk, "Lock each profile to one voltage cap for all sizes.")

        def _make_json_box(parent, title):
            box = ttk.LabelFrame(parent, text=title)
            box.pack(fill="both", expand=True, pady=(0, 8))
            inner = ttk.Frame(box)
            inner.pack(fill="both", expand=True, padx=6, pady=6)
            text = tk.Text(inner, height=10, wrap="none", font=("Consolas", 9))
            scroll = ttk.Scrollbar(inner, orient="vertical", command=text.yview)
            text.configure(yscrollcommand=scroll.set)
            text.grid(row=0, column=0, sticky="nsew")
            scroll.grid(row=0, column=1, sticky="ns")
            inner.columnconfigure(0, weight=1)
            inner.rowconfigure(0, weight=1)
            return box, text

        voltage_box, voltage_text = _make_json_box(frame, "Voltage caps (V)")
        settle_box, settle_text = _make_json_box(frame, "Settle base (err/time)")
        rules_box, rules_text = _make_json_box(frame, "Profile rules (when each profile applies)")

        def _pick_single_cap(value):
            if isinstance(value, dict):
                for key in ("v_mid", "mid", "v_small", "small", "v_large", "large"):
                    if key in value:
                        try:
                            return float(value[key])
                        except Exception:
                            break
                for val in value.values():
                    try:
                        return float(val)
                    except Exception:
                        continue
                return None
            if isinstance(value, (list, tuple)):
                if not value:
                    return None
                idx = 1 if len(value) >= 3 else 0
                try:
                    return float(value[idx])
                except Exception:
                    return None
            try:
                return float(value)
            except Exception:
                return None

        def _coerce_single_caps(voltage_data):
            if not isinstance(voltage_data, dict):
                raise ValueError("Voltage caps must be a JSON object.")
            for move_type, profiles in voltage_data.items():
                if not isinstance(profiles, dict):
                    continue
                for profile, cap_val in list(profiles.items()):
                    cap = _pick_single_cap(cap_val)
                    if cap is None:
                        raise ValueError(f"{move_type}.{profile} cap must be numeric.")
                    if isinstance(cap_val, dict):
                        profiles[profile] = {"v_small": cap, "v_mid": cap, "v_large": cap}
                    else:
                        profiles[profile] = [cap, cap, cap]
            return voltage_data

        def _caps_are_single(voltage_data):
            if not isinstance(voltage_data, dict):
                return False
            for profiles in voltage_data.values():
                if not isinstance(profiles, dict):
                    continue
                for cap_val in profiles.values():
                    try:
                        if isinstance(cap_val, dict):
                            a = float(cap_val.get("v_small", cap_val.get("small")))
                            b = float(cap_val.get("v_mid", cap_val.get("mid")))
                            c = float(cap_val.get("v_large", cap_val.get("large")))
                        else:
                            if not isinstance(cap_val, (list, tuple)) or len(cap_val) < 3:
                                return False
                            a, b, c = float(cap_val[0]), float(cap_val[1]), float(cap_val[2])
                    except Exception:
                        return False
                    if abs(a - b) > 1e-6 or abs(b - c) > 1e-6:
                        return False
            return True

        def _on_single_cap_toggle():
            if not single_cap_var.get():
                return
            try:
                data = json.loads(voltage_text.get("1.0", "end").strip() or "{}")
            except Exception as exc:
                messagebox.showerror("Invalid voltage caps", f"Voltage caps JSON error: {exc}")
                single_cap_var.set(False)
                return
            if not isinstance(data, dict):
                messagebox.showerror("Invalid voltage caps", "Voltage caps must be a JSON object.")
                single_cap_var.set(False)
                return
            try:
                _coerce_single_caps(data)
            except Exception as exc:
                messagebox.showerror("Single cap lock", f"Single cap lock failed: {exc}")
                single_cap_var.set(False)
                return
            voltage_text.delete("1.0", "end")
            voltage_text.insert("1.0", json.dumps(data, indent=2))

        cap_lock_chk.configure(command=_on_single_cap_toggle)

        def _load_values(data):
            voltage_text.delete("1.0", "end")
            settle_text.delete("1.0", "end")
            rules_text.delete("1.0", "end")
            voltage_text.insert("1.0", json.dumps(data["voltage_shapes"], indent=2))
            settle_text.insert("1.0", json.dumps(data["settle_base"], indent=2))
            rules_text.insert("1.0", json.dumps(data["profile_rules"], indent=2))

        defaults = _motion_profile_defaults()
        current = CFG.get("codegen", {}).get("motion_profiles", {})
        if not isinstance(current, dict):
            current = {}
        merged = {
            "voltage_shapes": current.get("voltage_shapes", defaults["voltage_shapes"]),
            "settle_base": current.get("settle_base", defaults["settle_base"]),
            "profile_rules": current.get("profile_rules", defaults["profile_rules"])
        }
        _load_values(merged)
        single_cap_var.set(_caps_are_single(merged["voltage_shapes"]))

        btns = ttk.Frame(frame)
        btns.pack(fill="x", pady=(4, 0))

        def _reset_defaults():
            _load_values(_motion_profile_defaults())

        def _apply_profiles():
            try:
                voltage_data = json.loads(voltage_text.get("1.0", "end").strip() or "{}")
            except Exception as exc:
                messagebox.showerror("Invalid voltage caps", f"Voltage caps JSON error: {exc}")
                return
            if not isinstance(voltage_data, dict):
                messagebox.showerror("Invalid voltage caps", "Voltage caps must be a JSON object.")
                return
            if single_cap_var.get():
                try:
                    _coerce_single_caps(voltage_data)
                except Exception as exc:
                    messagebox.showerror("Single cap lock", f"Single cap lock failed: {exc}")
                    return
            try:
                settle_data = json.loads(settle_text.get("1.0", "end").strip() or "{}")
            except Exception as exc:
                messagebox.showerror("Invalid settle base", f"Settle base JSON error: {exc}")
                return
            if not isinstance(settle_data, dict):
                messagebox.showerror("Invalid settle base", "Settle base must be a JSON object.")
                return
            try:
                rules_data = json.loads(rules_text.get("1.0", "end").strip() or "{}")
            except Exception as exc:
                messagebox.showerror("Invalid profile rules", f"Profile rules JSON error: {exc}")
                return
            if not isinstance(rules_data, dict):
                messagebox.showerror("Invalid profile rules", "Profile rules must be a JSON object.")
                return
            CFG.setdefault("codegen", {})["motion_profiles"] = {
                "voltage_shapes": voltage_data,
                "settle_base": settle_data,
                "profile_rules": rules_data
            }
            save_config(CFG)
            win.destroy()

        ttk.Button(btns, text="Reset to recommended defaults", command=_reset_defaults).pack(side="left")
        ttk.Button(btns, text="Apply", command=_apply_profiles).pack(side="right")
        ttk.Button(btns, text="Close", command=win.destroy).pack(side="right", padx=(0, 6))

    adv_frame = ttk.Frame(physics_body)
    adv_chk = ttk.Checkbutton(adv_frame, variable=adv_motion_var)
    adv_chk.pack(side="left")
    adv_profiles_btn = ttk.Button(adv_frame, text="Profiles...", command=_open_motion_profile_editor)
    adv_profiles_btn.pack(side="left", padx=(8, 0))
    ui.add_tooltip(adv_chk, "Enable JAR-style voltage/settle placeholders and profile hover info.")
    ui.add_tooltip(adv_profiles_btn, "Edit advanced motion profile rules, caps, and settle defaults.")
    ui.track_live_widget(adv_chk)
    ui.track_live_widget(adv_profiles_btn)
    _row(physics_body, 8, "Advanced motion vars:", adv_frame, "Enable JAR-style voltage/settle placeholders and profile hover info")

    cal_frame = None
    cal_export_btn = None

    def _refresh_adv_visibility():
        show_advanced = bool(adv_motion_var.get())
        try:
            if cal_frame is not None:
                if show_advanced:
                    cal_frame.grid()
                else:
                    cal_frame.grid_remove()
        except Exception:
            pass
        _refresh_physics_scrollbar()

    adv_motion_var.trace_add("write", lambda *_: _refresh_adv_visibility())
    _refresh_adv_visibility()

    # Dimensions tab rows
    _row(tabs["geometry"], 0, "Bot Geometry Visualizer:", ttk.Button(tabs["geometry"], text="Open visual editor", command=open_geometry_visualizer), "Drag-to-edit drivetrain and robot geometry, including reshape.")
    _row(tabs["geometry"], 1, "Offset 1 (in):", ttk.Entry(tabs["geometry"], textvariable=off1), "First preset")
    _row(tabs["geometry"], 2, "Offset 2 (in):", ttk.Entry(tabs["geometry"], textvariable=off2), "Second preset")
    _row(tabs["geometry"], 3, "Wall padding (in):", ttk.Entry(tabs["geometry"], textvariable=pad), "Boundary margin")
    _row(tabs["geometry"], 4, "Offset Visualizer:", ttk.Button(tabs["geometry"], text="Open offset editor", command=open_offset_visualizer), "Drag node along centerline to set offsets.")
    
    # Codegen tab (simplified - full implementation available if needed)
    # Replace the codegen tab section (around line 1100-1150) with this:

    # --- Export tab with template customization ---
    codegen_defaults = {
        "LemLib": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}});",
            "turn_global": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {{.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}});",
            "turn_local": "chassis.turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS}, {{.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}});",
            "swing": "chassis.swingToHeading({HEADING_DEG}, lemlib::DriveSide::{SIDE}, {TIMEOUT_MS}, {{.minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled per-command",
            "reverse_off": "// reverse handled per-command",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "chassis.waitUntilDone();",
            "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {{.forwards = {FORWARDS}}});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "JAR": {
            "wait": "pros::delay({MS});",
            "move": "driveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {HEADING_DEG});",
            "turn_global": "turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
            "turn_local": "turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS});",
            "pose": "driveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
            "swing": "swingToHeading({HEADING_DEG}, {DIR}, {TIMEOUT_MS});",
            "path_follow": 'followPath("{PATH_FILE}", {TIMEOUT_MS});',
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled inline",
            "reverse_off": "// reverse handled inline",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "",
            "marker_wait_done": "",
            "setpose": "setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "PROS": {
            "wait": "pros::delay({MS});",
            "move": "drive_distance({DIST_IN});",
            "turn_global": "turn_to({HEADING_DEG});",
            "turn_local": "turn_angle({TURN_DELTA_DEG});",
            "pose": "// move_to_pose x={X_IN}, y={Y_IN}, h={HEADING_DEG}",
            "swing": "swing_to({HEADING_DEG}, {DIR});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "",
            "marker_wait_done": "",
            "path_follow": 'follow_path("{PATH_FILE}", {LOOKAHEAD});',
            "setpose": "// set pose {X_IN},{Y_IN},{HEADING_DEG}"
        },
        "Custom": {
            "wait": "pros::delay({MS});",
            "move": "move({X_IN}, {Y_IN}, {HEADING_DEG});",
            "turn_global": "face({HEADING_DEG});",
            "turn_local": "turn_relative({TURN_DELTA_DEG});",
            "pose": "pose({X_IN}, {Y_IN}, {HEADING_DEG});",
            "swing": "swing_to_heading({HEADING_DEG}, {DIR});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "waitUntilDone();",
            "path_follow": 'follow_path("{PATH_NAME}", {TIMEOUT_MS}, {LOOKAHEAD});',
            "setpose": "setpose({X_IN},{Y_IN},{HEADING_DEG});"
        }
    }
    # Ensure codegen structure exists in CFG
    CFG.setdefault("codegen", {
        "style": "Action List",
        "templates": {},
        "opts": {
            "ticks_per_rotation": 360,
            "pad_factor": 1.0,
            "min_timeout_s": 0.0,
            "reshape_output": "1/2"
        },
        "path_dir": "export/paths",
        "path_columns": "{X}, {Y}, {COMMAND}",
        "mech_presets": [
            {"name": "reshape", "mode": "toggle", "template": "", "on": "", "off": "", "default": False}
        ],
        "calibration": {
            "enabled": 0,
            "last_run": "",
            "trial_count": 0,
            "hold_ms": 120,
            "k_noise": 4.0,
            "err_scale": 1.1,
            "time_scale": 1.1,
            "noise": {
                "drive_in": 0.0,
                "turn_deg": 0.0
            },
            "profiles": {}
        }
    })

    if not any(str(p.get("name", "")).strip().lower() == "reshape" for p in CFG["codegen"].get("mech_presets", [])):
        CFG["codegen"].setdefault("mech_presets", []).append(
            {"name": "reshape", "mode": "toggle", "template": "", "on": "", "off": "", "default": False}
        )

    # Initialize defaults for each style if not present
    for _style, _tpl in codegen_defaults.items():
        CFG["codegen"].setdefault("templates", {}).setdefault(_style, dict(_tpl))
        CFG["codegen"]["templates"].setdefault(_style, {}).setdefault("__optional__", ["setpose"])

    style_labels = ["Action List", "LemLib", "JAR", "PROS", "Custom"]
    codegen_style_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("style", "Action List")))

    # Template variables for customizable tokens
    base_tpl_keys = ["wait", "move", "turn_global", "turn_local", "pose", "path_follow", "tbuffer", "setpose", "swing", "marker_wait", "marker_wait_done"]
    optional_pool = ["reverse_on", "reverse_off", "reshape", "setpose", "swing"]
    tpl_keys = base_tpl_keys + optional_pool
    tpl_vars = {k: tk.StringVar() for k in tpl_keys}
    motion_mode_var = tk.StringVar(value="move")
    turn_mode_var = tk.StringVar(value="turn_global")
    template_tokens = [
        "MS", "S", "TIMEOUT_MS", "TIMEOUT_S",
        "X_IN", "Y_IN", "DIST_IN", "HEADING_DEG", "TURN_DELTA_DEG",
        "FORWARDS", "DIR", "SIDE", "LOOKAHEAD",
        "PATH_NAME", "PATH_FILE", "PATH_ASSET",
        "MOVE_SPEED", "TURN_SPEED", "PATH_MIN_SPEED", "PATH_MAX_SPEED",
        "DRIVE_MAX_V", "HEADING_MAX_V", "TURN_MAX_V", "SWING_MAX_V",
        "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
        "TURN_SETTLE_ERR", "TURN_SETTLE_TIME",
        "SWING_SETTLE_ERR", "SWING_SETTLE_TIME",
        "DRIVE_MIN_SPEED", "TURN_MIN_SPEED", "SWING_MIN_SPEED",
        "DRIVE_EARLY_EXIT", "TURN_EARLY_EXIT", "SWING_EARLY_EXIT",
        "LEAD_IN", "STATE", "NAME",
        "MARKER_DIST_IN", "MARKER_FRAC", "MARKER_INDEX"
    ]

    # Options
    ticks_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("ticks_per_rotation", 360)))
    pad_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("pad_factor", 1.0)))
    min_s_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("min_timeout_s", 0.0)))
    reshape_output_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("reshape_output", "1/2")))
    path_dir_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("path_dir", "export/paths")))
    path_columns_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("path_columns", "{X}, {Y}, {COMMAND}")))
    cal_raw = CFG.get("codegen", {}).get("calibration", {})
    cal_raw = dict(cal_raw) if isinstance(cal_raw, dict) else {}
    cal_enabled_val = cal_raw.get("enabled", 0)
    if isinstance(cal_enabled_val, dict):
        cal_enabled_val = cal_enabled_val.get("value", 0)
    cal_noise = cal_raw.get("noise", {})
    if not isinstance(cal_noise, dict):
        cal_noise = {}
    cal_enabled_var = tk.IntVar(value=int(cal_enabled_val))
    cal_err_scale_var = tk.StringVar(value=str(cal_raw.get("err_scale", 1.1)))
    cal_time_scale_var = tk.StringVar(value=str(cal_raw.get("time_scale", 1.1)))
    cal_noise_mult_var = tk.StringVar(value=str(cal_raw.get("k_noise", 4.0)))
    cal_noise_drive_var = tk.StringVar(value=str(cal_noise.get("drive_in", cal_raw.get("noise_drive_in", 0.0))))
    cal_noise_turn_var = tk.StringVar(value=str(cal_noise.get("turn_deg", cal_raw.get("noise_turn_deg", 0.0))))
    cal_status_var = tk.StringVar(value="")
    cal_detail_var = tk.StringVar(value="")
    cal_warn_var = tk.StringVar(value="")

    def _browse_path_dir():
        """Let user choose output directory for path files."""
        start_dir = path_dir_var.get() or os.getcwd()
        chosen = filedialog.askdirectory(
            parent=top,
            initialdir=start_dir if os.path.isdir(start_dir) else os.getcwd(),
            title="Select path export directory"
        )
        if chosen:
            path_dir_var.set(chosen)
            on_update()

    # Layout: Basic options first
    style_widget = ttk.Combobox(tabs["codegen"], textvariable=codegen_style_var, values=style_labels, state="readonly")
    _row(tabs["codegen"], 0, "Output style:", style_widget, "Select export style. Action List keeps current log.")
    _row(tabs["codegen"], 1, "Timeout pad ×:", 
        ttk.Entry(tabs["codegen"], textvariable=pad_var), 
        "Multiply projected time by constant for timeout margin.")
    _row(tabs["codegen"], 2, "Min timeout (s):", 
        ttk.Entry(tabs["codegen"], textvariable=min_s_var), 
        "Minimum timeout in seconds.")
    
    ticks_label = ttk.Label(tabs["codegen"], text="Ticks per rotation:")
    ticks_entry = ttk.Entry(tabs["codegen"], textvariable=ticks_var)
    ticks_label.grid(row=3, column=0, sticky="w", padx=6, pady=4)
    ticks_entry.grid(row=3, column=1, sticky="ew", padx=6, pady=4)
    ui.add_tooltip(ticks_label, "Encoder ticks per wheel rotation for distance conversion.")
    ui.add_tooltip(ticks_entry, "Encoder ticks per wheel rotation for distance conversion.")
    ui.track_live_widget(ticks_entry)

    path_dir_frame = ttk.Frame(tabs["codegen"])
    path_dir_entry = ttk.Entry(path_dir_frame, textvariable=path_dir_var)
    path_dir_entry.pack(side="left", fill="x", expand=True)
    path_dir_btn = ttk.Button(path_dir_frame, text="Browse...", command=_browse_path_dir)
    path_dir_btn.pack(side="left", padx=4)
    def _edit_path_columns():
        """Prompt for custom path file column format."""
        cur = path_columns_var.get().strip() or "{X}, {Y}, {COMMAND}"
        msg = "Enter path file columns using tokens:\n{X}, {Y}, {COMMAND}, {HEADING}"
        val = simpledialog.askstring("Path file columns", msg, initialvalue=cur)
        if val is not None:
            path_columns_var.set(val.strip() or "{X}, {Y}, {COMMAND}")
            on_update()
    path_cols_btn = ttk.Button(path_dir_frame, text="Columns...", command=_edit_path_columns)
    path_cols_btn.pack(side="left", padx=4)
    _row(tabs["codegen"], 4, "Path export dir:", path_dir_frame, "Directory to save generated path files.")
    ui.add_tooltip(path_dir_btn, "Directory to save generated path files.")
    ui.add_tooltip(path_cols_btn, "Set output columns for path files.")
    ui.track_live_widget(path_dir_entry)

    def _cal_is_float(val):
        try:
            float(val)
            return True
        except Exception:
            return False

    def _cal_has_data(cal: dict) -> bool:
        profiles = cal.get("profiles", {}) if isinstance(cal, dict) else {}
        if not isinstance(profiles, dict):
            return False
        return any(bool(profiles.get(mv)) for mv in ("drive", "turn", "swing"))

    def _cal_bucket_count(cal: dict) -> int:
        profiles = cal.get("profiles", {}) if isinstance(cal, dict) else {}
        if not isinstance(profiles, dict):
            return 0
        def _walk(node):
            if not isinstance(node, dict):
                return 0
            if any(k in node for k in ("err_p90", "settle_ms_p90", "err", "settle_ms")):
                return 1
            total = 0
            for v in node.values():
                total += _walk(v)
            return total
        return _walk(profiles)

    def _cal_status_text(cal: dict) -> str:
        if not isinstance(cal, dict) or not _cal_has_data(cal):
            return "Calibration: Not configured (Run Wizard)"
        enabled = cal.get("enabled", 0)
        if isinstance(enabled, dict):
            enabled = enabled.get("value", 0)
        status = "Calibration: Active" if enabled else "Calibration: Loaded"
        last_run = cal.get("last_run")
        parts = []
        if last_run:
            try:
                if isinstance(last_run, (int, float)):
                    last_str = datetime.fromtimestamp(float(last_run)).strftime("%Y-%m-%d %H:%M")
                else:
                    last_str = str(last_run).replace("T", " ").replace("Z", "").strip()
                parts.append(f"last run {last_str}")
            except Exception:
                pass
        trial_count = cal.get("trial_count")
        if isinstance(trial_count, (int, float)):
            parts.append(f"{int(trial_count)} trials")
        else:
            bucket_count = _cal_bucket_count(cal)
            if bucket_count:
                parts.append(f"{bucket_count} buckets")
        if parts:
            status += " (" + ", ".join(parts) + ")"
        return status

    def _cal_detail_text(cal: dict) -> str:
        profiles = cal.get("profiles", {}) if isinstance(cal, dict) else {}
        moves_present = []
        if isinstance(profiles, dict):
            for mv in ("drive", "turn", "swing"):
                if profiles.get(mv):
                    moves_present.append(mv)
        if not moves_present:
            return ""
        profile_keys = set()
        cap_keys = set()
        skip_keys = {"caps", "default", "small", "medium", "large", "err_p90", "settle_ms_p90", "err", "settle_ms"}

        def _walk(node):
            if not isinstance(node, dict):
                return
            for k, v in node.items():
                if isinstance(v, dict):
                    if _cal_is_float(k):
                        cap_keys.add(f"{float(k):g}")
                    elif str(k).lower() not in skip_keys:
                        profile_keys.add(str(k))
                    _walk(v)

        for mv in moves_present:
            _walk(profiles.get(mv, {}))
        moves_str = "/".join([m.capitalize() for m in moves_present])
        prof_str = ", ".join(sorted(profile_keys)) if profile_keys else "default"
        cap_str = ", ".join(sorted(cap_keys, key=lambda x: float(x))) if cap_keys else "default"
        return f"Moves: {moves_str} | Profiles: {prof_str} | Caps: {cap_str}"

    def _refresh_cal_summary():
        cal = CFG.get("codegen", {}).get("calibration", {})
        cal_status_var.set(_cal_status_text(cal))
        cal_detail_var.set(_cal_detail_text(cal))
        enabled = cal.get("enabled", 0) if isinstance(cal, dict) else 0
        if isinstance(enabled, dict):
            enabled = enabled.get("value", 0)
        if enabled and not _cal_has_data(cal):
            cal_warn_var.set("No calibration loaded. Run Wizard or Import JSON.")
        else:
            cal_warn_var.set("")
        try:
            if cal_export_btn is not None:
                cal_export_btn.configure(state=("normal" if _cal_has_data(cal) else "disabled"))
        except Exception:
            pass

    def _sync_cal_vars_from_cfg():
        cal = CFG.get("codegen", {}).get("calibration", {})
        cal = dict(cal) if isinstance(cal, dict) else {}
        enabled = cal.get("enabled", 0)
        if isinstance(enabled, dict):
            enabled = enabled.get("value", 0)
        cal_enabled_var.set(int(enabled))
        cal_err_scale_var.set(str(cal.get("err_scale", 1.1)))
        cal_time_scale_var.set(str(cal.get("time_scale", 1.1)))
        cal_noise_mult_var.set(str(cal.get("k_noise", 4.0)))
        noise = cal.get("noise", {})
        if not isinstance(noise, dict):
            noise = {}
        cal_noise_drive_var.set(str(noise.get("drive_in", cal.get("noise_drive_in", 0.0))))
        cal_noise_turn_var.set(str(noise.get("turn_deg", cal.get("noise_turn_deg", 0.0))))
        _refresh_cal_summary()

    def _import_calibration():
        path = filedialog.askopenfilename(
            parent=top,
            title="Import calibration JSON",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            messagebox.showerror("Import failed", f"Could not read JSON:\n{e}")
            return
        cal = None
        if isinstance(data, dict):
            if "codegen" in data and isinstance(data["codegen"], dict):
                cal = data["codegen"].get("calibration")
            elif "calibration" in data:
                cal = data.get("calibration")
            else:
                cal = data
        if not isinstance(cal, dict):
            messagebox.showerror("Import failed", "No calibration data found in JSON.")
            return
        CFG.setdefault("codegen", {})["calibration"] = cal
        _sync_cal_vars_from_cfg()
        on_update()

    def _export_calibration():
        cal = CFG.get("codegen", {}).get("calibration", {})
        if not isinstance(cal, dict) or not _cal_has_data(cal):
            messagebox.showinfo("Export calibration", "No calibration data to export.")
            return
        path = filedialog.asksaveasfilename(
            parent=top,
            title="Export calibration JSON",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"calibration": cal}, f, indent=2)
        except Exception as e:
            messagebox.showerror("Export failed", f"Could not write JSON:\n{e}")

    def _clear_calibration():
        CFG.setdefault("codegen", {})["calibration"] = {
            "enabled": 0,
            "last_run": "",
            "trial_count": 0,
            "hold_ms": 120,
            "k_noise": 4.0,
            "err_scale": 1.1,
            "time_scale": 1.1,
            "noise": {"drive_in": 0.0, "turn_deg": 0.0},
            "profiles": {}
        }
        _sync_cal_vars_from_cfg()
        on_update()

    def _reset_calibration_recommended():
        cal_err_scale_var.set("1.2")
        cal_time_scale_var.set("1.2")
        cal_noise_mult_var.set("4.0")
        on_update()

    def _cal_bucket(move_type: str, magnitude: float):
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

    def _cal_p90(values):
        vals = sorted(values)
        if not vals:
            return None
        idx = int(math.ceil(0.9 * len(vals))) - 1
        idx = max(0, min(len(vals) - 1, idx))
        return vals[idx]

    def _analyze_calibration_logs(log_text: str):
        buckets = {}
        noise_drive = None
        noise_turn = None
        errors = []
        keys_seen = set()
        trial_count = 0
        counts = {"drive": 0, "turn": 0, "swing": 0}
        profile_counts = {}
        settle_variance = {"drive": [], "turn": [], "swing": []}
        dyn_samples = []
        for line_no, raw in enumerate(log_text.splitlines(), start=1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                row = json.loads(raw)
            except Exception:
                errors.append(f"Line {line_no}: invalid JSON")
                continue
            if not isinstance(row, dict):
                errors.append(f"Line {line_no}: not a JSON object")
                continue
            keys_seen.update(row.keys())
            mv = row.get("move_type", row.get("move", row.get("type")))
            if not mv:
                errors.append(f"Line {line_no}: missing move_type")
                continue
            mv = str(mv).lower()
            if mv in ("move", "pose", "path"):
                mv = "drive"
            elif mv in ("turn", "face"):
                mv = "turn"
            elif mv not in ("drive", "turn", "swing"):
                errors.append(f"Line {line_no}: unknown move_type '{mv}'")
                continue
            profile = str(row.get("profile", "default")).lower()
            cap = row.get("cap_frac", row.get("cap", row.get("cap_v")))
            if cap is None:
                cap_frac = 1.0
            else:
                try:
                    cap_val = float(cap)
                    cap_frac = cap_val / 12.0 if cap_val > 1.5 else cap_val
                except Exception:
                    cap_frac = 1.0
            cap_frac = max(0.1, min(1.2, cap_frac))
            cap_key = f"{cap_frac:.2f}".rstrip("0").rstrip(".")
            mag_bucket = row.get("mag_bucket")
            if not mag_bucket:
                mag_val = row.get("magnitude", row.get("dist_in", row.get("distance_in", row.get("angle_deg"))))
                if mag_val is not None:
                    try:
                        mag_bucket = _cal_bucket(mv, float(mag_val))
                    except Exception:
                        mag_bucket = "default"
                else:
                    mag_bucket = "default"
            err_val = row.get("final_err", row.get("error"))
            settle_val = row.get("settle_ms", row.get("settle_time_ms"))
            missing = []
            if err_val is None:
                missing.append("final_err")
            if settle_val is None:
                missing.append("settle_ms")
            if missing:
                errors.append(f"Line {line_no}: missing {', '.join(missing)}")
                continue
            key = (mv, profile, cap_key, str(mag_bucket))
            if key not in buckets:
                buckets[key] = {"err": [], "settle": []}
            try:
                buckets[key]["err"].append(abs(float(err_val)))
            except Exception:
                errors.append(f"Line {line_no}: invalid final_err")
                continue
            try:
                settle_ms_val = max(0.0, float(settle_val))
            except Exception:
                errors.append(f"Line {line_no}: invalid settle_ms")
                continue
            buckets[key]["settle"].append(settle_ms_val)
            trial_count += 1
            counts[mv] = counts.get(mv, 0) + 1
            profile_counts[(mv, profile)] = profile_counts.get((mv, profile), 0) + 1
            if mv == "drive":
                mag_raw = row.get("distance_in", row.get("dist_in", row.get("magnitude")))
            elif mv == "turn":
                mag_raw = row.get("angle_deg", row.get("turn_deg", row.get("magnitude")))
            else:
                mag_raw = None
            if mag_raw is not None:
                try:
                    mag_val = float(mag_raw)
                    dyn_samples.append((mv, cap_frac, mag_val, settle_ms_val))
                except Exception:
                    pass
            if noise_drive is None:
                nd = row.get("noise_drive_in")
                if nd is not None:
                    try:
                        noise_drive = float(nd)
                    except Exception:
                        pass
            if noise_turn is None:
                nt = row.get("noise_turn_deg")
                if nt is not None:
                    try:
                        noise_turn = float(nt)
                    except Exception:
                        pass
        if not buckets:
            return {
                "ok": False,
                "errors": errors or ["No valid calibration lines found."],
                "keys_seen": keys_seen,
                "counts": counts,
                "trial_count": 0
            }
        cal_profiles = {}
        for (mv, profile, cap_key, mag_bucket), vals in buckets.items():
            mv_dict = cal_profiles.setdefault(mv, {})
            prof_dict = mv_dict.setdefault(profile, {})
            caps_dict = prof_dict.setdefault("caps", {})
            cap_dict = caps_dict.setdefault(cap_key, {})
            bucket_dict = cap_dict.setdefault(mag_bucket, {})
            err_p90 = _cal_p90(vals["err"])
            settle_p90 = _cal_p90(vals["settle"])
            if err_p90 is not None:
                bucket_dict["err_p90"] = float(err_p90)
            if settle_p90 is not None:
                bucket_dict["settle_ms_p90"] = int(round(settle_p90))
            if vals["settle"]:
                settle_variance[mv].append(list(vals["settle"]))
        noise = {}
        if noise_drive is not None:
            noise["drive_in"] = float(noise_drive)
        if noise_turn is not None:
            noise["turn_deg"] = float(noise_turn)
        warnings = []
        for mv, buckets_list in settle_variance.items():
            high_var = False
            for vals in buckets_list:
                if len(vals) < 2:
                    continue
                mean = sum(vals) / len(vals)
                if mean <= 1e-6:
                    continue
                var = sum((v - mean) ** 2 for v in vals) / len(vals)
                ratio = math.sqrt(var) / mean
                if ratio > 0.25:
                    high_var = True
                    break
            if high_var:
                warnings.append(f"High variance in {mv} settle time (>25%)")

        dynamics = {}
        if dyn_samples:
            hold_ms = CFG.get("codegen", {}).get("calibration", {}).get("hold_ms", 120)
            try:
                hold_ms = float(hold_ms)
            except Exception:
                hold_ms = 120.0
            drive_vmax = []
            drive_vmax_any = []
            drive_accel = []
            drive_accel_any = []
            turn_rate = []
            turn_rate_any = []
            turn_accel = []
            turn_accel_any = []
            for mv, cap_frac, mag_val, settle_ms_val in dyn_samples:
                try:
                    settle_ms_val = float(settle_ms_val)
                except Exception:
                    continue
                travel_ms = settle_ms_val - hold_ms
                if travel_ms < 80.0:
                    travel_ms = settle_ms_val
                t_s = max(0.05, travel_ms / 1000.0)
                cap_for_scale = max(0.35, min(1.2, float(cap_frac)))
                cap_scale = 1.0 / cap_for_scale
                if mv == "drive":
                    dist = abs(float(mag_val))
                    v_est = (dist / t_s) * cap_scale
                    a_est = (4.0 * dist / (t_s * t_s)) * cap_scale
                    drive_vmax_any.append(v_est)
                    drive_accel_any.append(a_est)
                    if dist >= 24.0:
                        drive_vmax.append(v_est)
                    if dist <= 12.0:
                        drive_accel.append(a_est)
                elif mv == "turn":
                    ang = abs(float(mag_val))
                    r_est = (ang / t_s) * cap_scale
                    a_est = (4.0 * ang / (t_s * t_s)) * cap_scale
                    turn_rate_any.append(r_est)
                    turn_accel_any.append(a_est)
                    if ang >= 60.0:
                        turn_rate.append(r_est)
                    if ang <= 30.0:
                        turn_accel.append(a_est)

            def _median(vals):
                vals = sorted(v for v in vals if v is not None)
                if not vals:
                    return None
                mid = len(vals) // 2
                if len(vals) % 2:
                    return vals[mid]
                return 0.5 * (vals[mid - 1] + vals[mid])

            vmax_val = _median(drive_vmax) if drive_vmax else _median(drive_vmax_any)
            accel_val = _median(drive_accel) if drive_accel else _median(drive_accel_any)
            turn_rate_val = _median(turn_rate) if turn_rate else _median(turn_rate_any)
            turn_accel_val = _median(turn_accel) if turn_accel else _median(turn_accel_any)
            if vmax_val is not None:
                dynamics["vmax_ips"] = float(vmax_val)
            if accel_val is not None:
                dynamics["accel_ips2"] = float(accel_val)
            if turn_rate_val is not None:
                dynamics["turn_rate_dps"] = float(turn_rate_val)
            if turn_accel_val is not None:
                dynamics["turn_accel_dps2"] = float(turn_accel_val)
        return {
            "ok": True,
            "errors": errors,
            "keys_seen": keys_seen,
            "counts": counts,
            "profile_counts": profile_counts,
            "trial_count": trial_count,
            "noise": noise,
            "profiles": cal_profiles,
            "warnings": warnings,
            "dynamics": dynamics
        }

    def _run_calibration_wizard():
        win = tk.Toplevel(top)
        win.title("Calibration Wizard")
        win.geometry("760x640")
        win.resizable(True, True)

        wrapper = ttk.Frame(win)
        wrapper.pack(fill="both", expand=True)
        wizard_canvas = tk.Canvas(wrapper, borderwidth=0, highlightthickness=0)
        wizard_scroll = ttk.Scrollbar(wrapper, orient="vertical", command=wizard_canvas.yview)
        wizard_canvas.configure(yscrollcommand=wizard_scroll.set)
        wizard_scroll.pack(side="right", fill="y")
        wizard_canvas.pack(side="left", fill="both", expand=True)

        body = ttk.Frame(wizard_canvas, padding=10)
        body_id = wizard_canvas.create_window((0, 0), window=body, anchor="nw")
        body.columnconfigure(0, weight=1)

        def _wizard_on_configure(_evt=None):
            try:
                wizard_canvas.configure(scrollregion=wizard_canvas.bbox("all"))
                wizard_canvas.itemconfigure(body_id, width=wizard_canvas.winfo_width())
            except Exception:
                pass

        def _wizard_mousewheel(event):
            delta = 0
            if hasattr(event, "delta") and event.delta:
                delta = int(-event.delta / 40) if abs(event.delta) >= 40 else (-2 if event.delta > 0 else 2)
            else:
                num = getattr(event, "num", None)
                if num == 4:
                    delta = -1
                elif num == 5:
                    delta = 1
            if delta:
                wizard_canvas.yview_scroll(delta, "units")
                return "break"

        body.bind("<Configure>", _wizard_on_configure)
        wizard_canvas.bind("<Configure>", _wizard_on_configure)
        for w in (wizard_canvas, body):
            w.bind("<MouseWheel>", _wizard_mousewheel)
            w.bind("<Button-4>", _wizard_mousewheel)
            w.bind("<Button-5>", _wizard_mousewheel)

        step_lbl = ttk.Label(
            body,
            text="Step 1: Generate plan  \u2192  Step 2: Run on robot  \u2192  Step 3: Paste logs & apply",
            font=("Segoe UI", 10, "bold")
        )
        step_lbl.grid(row=0, column=0, sticky="w", pady=(0, 6))

        plan_frame = ttk.LabelFrame(body, text="Step 1: Generate plan")
        plan_frame.grid(row=1, column=0, sticky="ew")
        plan_frame.columnconfigure(1, weight=1)
        plan_frame.columnconfigure(3, weight=1)

        def _safe_auton_ident(raw_name: str) -> str:
            raw = str(raw_name or "").strip()
            cleaned = []
            last_us = False
            for ch in raw:
                if ch.isascii() and ch.isalnum():
                    cleaned.append(ch.lower())
                    last_us = False
                else:
                    if not last_us:
                        cleaned.append("_")
                        last_us = True
            ident = "".join(cleaned).strip("_")
            if not ident:
                ident = "calibration_auton"
            if ident[0].isdigit():
                ident = "cal_" + ident
            if not ident.endswith("_auton"):
                ident += "_auton"
            return ident

        style_options = ["JAR", "LemLib", "Custom"]
        style_default = codegen_style_var.get()
        if style_default not in style_options:
            style_default = "JAR"
        plan_style_var = tk.StringVar(value=style_default)
        preset_var = tk.StringVar(value="Standard (10 min)")
        drive_type_var = tk.StringVar(value="traction")
        imu_var = tk.IntVar(value=1)
        track_default = str(int(CFG.get("robot_physics", {}).get("tracking_wheels", 0)))
        track_var = tk.StringVar(value=track_default)
        drive_err_source_var = tk.StringVar(value="Tape measure")
        profile_order = ["precise", "normal", "fast", "slam"]
        profile_vars = {p: tk.IntVar(value=1 if p in ("precise", "normal", "fast") else 0) for p in profile_order}
        caps_var = tk.StringVar(value="0.5, 0.75, 1.0")
        caps_mode_var = tk.StringVar(value="Fractions (0-1)")
        caps_interp_var = tk.StringVar(value="")
        drive_var = tk.StringVar(value="6, 24, 48")
        turn_var = tk.StringVar(value="15, 45, 90")
        repeat_var = tk.StringVar(value="6")
        auton_name_var = tk.StringVar(value="Calibration Tests")
        auton_fn_var = tk.StringVar(value=_safe_auton_ident(auton_name_var.get()))

        def _refresh_auton_fn(*_):
            auton_fn_var.set(_safe_auton_ident(auton_name_var.get()))

        auton_name_var.trace_add("write", _refresh_auton_fn)

        ttk.Label(plan_frame, text="Preset:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=preset_var, values=["Quick (5 min)", "Standard (10 min)", "Deep (20 min)"], state="readonly", width=16).grid(
            row=0, column=1, sticky="w", padx=4, pady=2
        )
        ttk.Label(plan_frame, text="Export style:").grid(row=0, column=2, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=plan_style_var, values=style_options, state="readonly", width=10).grid(
            row=0, column=3, sticky="w", padx=4, pady=2
        )

        ttk.Label(plan_frame, text="Drivetrain:").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=drive_type_var, values=["traction", "all-omni", "mixed"], state="readonly", width=10).grid(
            row=1, column=1, sticky="w", padx=4, pady=2
        )
        ttk.Label(plan_frame, text="IMU:").grid(row=1, column=2, sticky="w", padx=4, pady=2)
        ttk.Checkbutton(plan_frame, variable=imu_var).grid(row=1, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(plan_frame, text="Tracking wheels:").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=track_var, values=["0", "1", "2"], state="readonly", width=5).grid(
            row=2, column=1, sticky="w", padx=4, pady=2
        )
        ttk.Label(plan_frame, text="Drive error source:").grid(row=2, column=2, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=drive_err_source_var, values=["Tracking odom", "Tape measure", "Drive encoders (rough)"], state="readonly", width=18).grid(
            row=2, column=3, sticky="w", padx=4, pady=2
        )

        ttk.Label(plan_frame, text="Profiles:").grid(row=3, column=0, sticky="w", padx=4, pady=2)
        prof_frame = ttk.Frame(plan_frame)
        prof_frame.grid(row=3, column=1, columnspan=3, sticky="w", padx=4, pady=2)
        for p in profile_order:
            ttk.Checkbutton(prof_frame, text=p, variable=profile_vars[p]).pack(side="left", padx=(0, 6))

        ttk.Label(plan_frame, text="Caps input:").grid(row=4, column=0, sticky="w", padx=4, pady=2)
        ttk.Combobox(plan_frame, textvariable=caps_mode_var, values=["Fractions (0-1)", "Volts"], state="readonly", width=16).grid(
            row=4, column=1, sticky="w", padx=4, pady=2
        )
        ttk.Label(plan_frame, text="Caps:").grid(row=4, column=2, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=caps_var, width=18).grid(row=4, column=3, sticky="w", padx=4, pady=2)

        caps_interp_lbl = ttk.Label(plan_frame, textvariable=caps_interp_var, foreground="#555555")
        caps_interp_lbl.grid(row=5, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 2))

        ttk.Label(plan_frame, text="Drive (in):").grid(row=6, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=drive_var, width=18).grid(row=6, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(plan_frame, text="Turn (deg):").grid(row=6, column=2, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=turn_var, width=16).grid(row=6, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(plan_frame, text="Repeat:").grid(row=7, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=repeat_var, width=6).grid(row=7, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(plan_frame, text="Auton name:").grid(row=8, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=auton_name_var, width=18).grid(row=8, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(plan_frame, text="Function:").grid(row=8, column=2, sticky="w", padx=4, pady=2)
        ttk.Entry(plan_frame, textvariable=auton_fn_var, width=18, state="readonly").grid(row=8, column=3, sticky="w", padx=4, pady=2)

        drive_err_note = ttk.Label(plan_frame, text="", foreground="#b00020")
        drive_err_note.grid(row=9, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 2))
        plan_note_lbl = ttk.Label(
            plan_frame,
            text="Export Robot Test Code outputs a calibration auton routine and embeds the plan for reference.",
            foreground="#555555"
        )
        plan_note_lbl.grid(row=10, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 4))

        run_frame = ttk.LabelFrame(body, text="Step 2: Run on robot")
        run_frame.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        run_text = (
            "Click Export Robot Test Code... and add it to your auton project.\n"
            "Select the calibration auton in your selector and run on field tiles.\n"
            "Copy the printed JSON lines from the brain/serial log.\n"
            "Paste them below and click Analyze Logs.\n"
            "Click Apply Calibration to save into settings."
        )
        ttk.Label(run_frame, text=run_text, justify="left").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(run_frame, text="Minimum recommended repeats: 5-6", foreground="#555555").grid(row=1, column=0, sticky="w", padx=6)
        ttk.Label(run_frame, text="Calibrate on the same tiles/surface you compete on.", foreground="#555555").grid(row=2, column=0, sticky="w", padx=6, pady=(0, 4))

        paste_frame = ttk.LabelFrame(body, text="Step 3: Paste logs & analyze")
        paste_frame.grid(row=3, column=0, sticky="nsew", pady=(6, 0))
        paste_frame.columnconfigure(0, weight=1)
        paste_frame.rowconfigure(1, weight=1)

        ttk.Label(
            paste_frame,
            text="Log schema: move_type, profile, cap_frac, mag_bucket (or magnitude), final_err, settle_ms.",
            justify="left",
            foreground="#555555"
        ).grid(row=0, column=0, sticky="w", padx=6, pady=(4, 0))

        text = tk.Text(paste_frame, height=10, wrap="none")
        text.grid(row=1, column=0, sticky="nsew", padx=6, pady=(4, 4))

        example_line = (
            "{\"move_type\":\"turn\",\"profile\":\"normal\",\"cap_frac\":0.75,"
            "\"angle_deg\":90,\"final_err\":0.8,\"settle_ms\":220,\"noise_turn_deg\":0.15}"
        )
        example_lbl = ttk.Label(paste_frame, text="Example:", foreground="#555555")
        example_lbl.grid(row=2, column=0, sticky="w", padx=6)
        example_val = ttk.Label(paste_frame, text=example_line, font=("Consolas", 9))
        example_val.grid(row=3, column=0, sticky="w", padx=6, pady=(0, 4))
        ttk.Label(
            paste_frame,
            text="cap_frac is 0-1 (0.5=6V, 0.75=9V). final_err units: inches for drive, degrees for turn.",
            foreground="#555555"
        ).grid(row=4, column=0, sticky="w", padx=6, pady=(0, 4))

        keys_frame = ttk.Frame(paste_frame)
        keys_frame.grid(row=5, column=0, sticky="w", padx=6, pady=(0, 4))
        ttk.Label(keys_frame, text="Required keys:", foreground="#555555").pack(side="left")
        req_keys = ["move_type", "profile", "cap_frac", "final_err", "settle_ms"]
        key_labels = {}
        for key in req_keys:
            lbl = ttk.Label(keys_frame, text=key, foreground="#999999")
            lbl.pack(side="left", padx=(6, 0))
            key_labels[key] = lbl

        btns = ttk.Frame(paste_frame)
        btns.grid(row=6, column=0, sticky="ew", padx=6, pady=(4, 6))
        btns.columnconfigure(4, weight=1)

        summary_frame = ttk.LabelFrame(body, text="Results")
        summary_frame.grid(row=4, column=0, sticky="ew", pady=(6, 0))
        summary_frame.columnconfigure(0, weight=1)
        analysis_summary_var = tk.StringVar(value="Analyze logs to see summary and warnings.")
        ttk.Label(summary_frame, textvariable=analysis_summary_var, justify="left", wraplength=700).grid(row=0, column=0, sticky="w", padx=6, pady=4)

        body.rowconfigure(3, weight=1)

        def _parse_list(raw, cast=float):
            items = []
            for part in str(raw).split(","):
                part = part.strip()
                if not part:
                    continue
                try:
                    items.append(cast(part))
                except Exception:
                    pass
            return items

        def _parse_caps(raw, mode):
            caps = _parse_list(raw, float)
            if not caps:
                return []
            is_volts = (mode == "Volts")
            if is_volts or any(v > 1.5 for v in caps):
                return [max(0.1, min(1.2, v / 12.0)) for v in caps]
            return [max(0.1, min(1.2, v)) for v in caps]

        def _update_caps_interp(*_):
            caps = _parse_caps(caps_var.get(), caps_mode_var.get())
            if not caps:
                caps_interp_var.set("")
                return
            volts = [v * 12.0 for v in caps]
            caps_interp_var.set(
                "Caps: " + ", ".join(f"{v:.0f}V" for v in volts) +
                " (" + ", ".join(f"{v:.2f}" for v in caps) + ")"
            )

        def _apply_preset(*_):
            preset = preset_var.get()
            if "Quick" in preset:
                caps_var.set("0.75, 1.0")
                drive_var.set("24, 48")
                turn_var.set("45, 90")
                repeat_var.set("4")
                for p in profile_order:
                    profile_vars[p].set(1 if p in ("normal", "fast") else 0)
            elif "Deep" in preset:
                caps_var.set("0.5, 0.75, 1.0")
                drive_var.set("6, 24, 48, 72")
                turn_var.set("15, 45, 90, 180")
                repeat_var.set("8")
                for p in profile_order:
                    profile_vars[p].set(1)
            else:
                caps_var.set("0.5, 0.75, 1.0")
                drive_var.set("6, 24, 48")
                turn_var.set("15, 45, 90")
                repeat_var.set("6")
                for p in profile_order:
                    profile_vars[p].set(1 if p in ("precise", "normal", "fast") else 0)
            _update_caps_interp()

        def _refresh_drive_err_note(*_):
            if str(track_var.get()) == "0":
                drive_err_source_var.set("Tape measure")
                drive_err_note.configure(text="Drive settle error will be approximate unless you enter measured error.")
            else:
                if drive_err_source_var.get() == "Tape measure":
                    drive_err_source_var.set("Tracking odom")
                drive_err_note.configure(text="")

        def _export_plan():
            profiles = [p for p in profile_order if profile_vars[p].get()]
            if not profiles:
                profiles = ["precise", "normal", "fast"]
            caps = _parse_caps(caps_var.get(), caps_mode_var.get())
            drives = _parse_list(drive_var.get(), float)
            turns = _parse_list(turn_var.get(), float)
            try:
                repeat = max(1, int(repeat_var.get()))
            except Exception:
                repeat = 6
            auton_name = auton_name_var.get().strip() or "Calibration Tests"
            auton_fn = auton_fn_var.get().strip() or _safe_auton_ident(auton_name)
            plan = {
                "export_style": plan_style_var.get(),
                "drivetrain": drive_type_var.get(),
                "imu": int(imu_var.get()),
                "tracking_wheels": int(track_var.get() or 0),
                "drive_error_source": drive_err_source_var.get(),
                "profiles": profiles,
                "caps": caps or [0.33, 0.66, 1.0],
                "drive_in": drives or [6, 24, 48],
                "turn_deg": turns or [15, 45, 90],
                "repeat": repeat,
                "auton_name": auton_name,
                "auton_fn": auton_fn
            }
            path = filedialog.asksaveasfilename(
                parent=win,
                title="Export robot test code",
                defaultextension=".cpp",
                filetypes=[("C++ files", "*.cpp"), ("Text files", "*.txt"), ("JSON files", "*.json"), ("All files", "*.*")]
            )
            if not path:
                return
            ext = os.path.splitext(path)[1].lower()
            try:
                if ext == ".json":
                    with open(path, "w", encoding="utf-8") as f:
                        json.dump({"calibration_plan": plan}, f, indent=2)
                else:
                    def _plan_comment(pl):
                        raw = json.dumps(pl, indent=2)
                        return "\n".join(["// " + line for line in raw.splitlines()])

                    def _emit_code(pl):
                        def _cpp_ident(raw_name: str) -> str:
                            raw = str(raw_name or "").strip()
                            cleaned = []
                            last_us = False
                            for ch in raw:
                                if ch.isascii() and ch.isalnum():
                                    cleaned.append(ch.lower())
                                    last_us = False
                                else:
                                    if not last_us:
                                        cleaned.append("_")
                                        last_us = True
                            ident = "".join(cleaned).strip("_")
                            if not ident:
                                ident = "calibration_auton"
                            if ident[0].isdigit():
                                ident = "cal_" + ident
                            if not ident.endswith("_auton"):
                                ident += "_auton"
                            return ident

                        def _cpp_string(raw_name: str) -> str:
                            out = []
                            for ch in str(raw_name or ""):
                                if ch == "\\":
                                    out.append("\\\\")
                                elif ch == "\"":
                                    out.append("\\\"")
                                elif ch.isascii():
                                    out.append(ch)
                                else:
                                    out.append("?")
                            return "".join(out)

                        auton_name = pl.get("auton_name") or "Calibration Tests"
                        auton_fn = pl.get("auton_fn") or _cpp_ident(auton_name)
                        auton_fn = _cpp_ident(auton_fn)
                        auton_name_cpp = _cpp_string(auton_name)
                        style = pl.get("export_style", "Custom")
                        lines = []
                        lines.append("// Calibration test code generated by Atticus Terminal")
                        lines.append("// Plan:")
                        lines.append(_plan_comment(pl))
                        lines.append("")
                        lines.append("#include \"main.h\"")
                        lines.append("#include <cmath>")
                        lines.append("#include <cstdio>")
                        lines.append("#include <cstring>")
                        lines.append("")
                        lines.append("// Move/turn commands are generated from your current Export templates for this style.")
                        lines.append("// Define these macros to match your project (optional):")
                        lines.append("//   CAL_SET_PROFILE(name)         -> set drive profile")
                        lines.append("//   CAL_SET_DRIVE_CAP(volts)       -> set max drive voltage")
                        lines.append("//   CAL_SET_TURN_CAP(volts)        -> set max turn voltage")
                        lines.append("//   CAL_GET_POSE_X(), _Y(), _H()   -> return odom pose (in, in, deg)")
                        lines.append("//   CAL_SET_POSE(x, y, heading)    -> reset odom pose (if available)")
                        lines.append("#ifndef CAL_SET_PROFILE")
                        lines.append("#define CAL_SET_PROFILE(name) ((void)(name))")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_SET_DRIVE_CAP")
                        lines.append("#define CAL_SET_DRIVE_CAP(volts) ((void)(volts))")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_SET_TURN_CAP")
                        lines.append("#define CAL_SET_TURN_CAP(volts) ((void)(volts))")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_GET_POSE_X")
                        lines.append("#define CAL_GET_POSE_X() 0.0")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_GET_POSE_Y")
                        lines.append("#define CAL_GET_POSE_Y() 0.0")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_GET_POSE_H")
                        lines.append("#define CAL_GET_POSE_H() 0.0")
                        lines.append("#endif")
                        lines.append("#ifndef CAL_SET_POSE")
                        lines.append("#define CAL_SET_POSE(x, y, heading) ((void)(x), (void)(y), (void)(heading))")
                        lines.append("#endif")
                        lines.append("")
                        lines.append("// Auton selector hint:")
                        lines.append(f"//   Add {{CAL_AUTON_NAME, {auton_fn}}} to your auton list, or call {auton_fn}() in autonomous().")
                        lines.append(f"static const char* CAL_AUTON_NAME = \"{auton_name_cpp}\";")

                        tpl_source = CFG.get("codegen", {}).get("templates", {}).get(style, {})
                        tpl_defaults = codegen_defaults.get(style, codegen_defaults.get("Custom", {}))
                        tpls = dict(tpl_defaults)
                        if isinstance(tpl_source, dict):
                            for k, v in tpl_source.items():
                                if isinstance(v, str):
                                    tpls[k] = v
                        modes = tpl_source.get("__modes__", {}) if isinstance(tpl_source, dict) else {}
                        move_key = modes.get("motion", "move")
                        turn_key = modes.get("turn", "turn_global" if style == "LemLib" else "turn_local")
                        if move_key not in tpls:
                            move_key = "move" if "move" in tpls else next(iter(tpls.keys()))
                        if turn_key not in tpls:
                            turn_key = "turn_global" if "turn_global" in tpls else ("turn_local" if "turn_local" in tpls else move_key)

                        def _normalize_tpl(val):
                            if isinstance(val, (list, tuple)):
                                return [str(v) for v in val if str(v).strip()]
                            if isinstance(val, str):
                                parts = [p.strip() for p in val.split("||")]
                                return [p for p in parts if p]
                            return []

                        def _escape_struct_braces(part: str) -> str:
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

                        def _format_tpl_lines(tpl_val, tokens: dict):
                            out_lines = []
                            for part in _normalize_tpl(tpl_val):
                                try:
                                    line = part.format(**tokens)
                                except Exception:
                                    part_fixed = _escape_struct_braces(part)
                                    try:
                                        line = part_fixed.format(**tokens)
                                    except Exception:
                                        line = "// template error: update output"
                                if line.strip():
                                    out_lines.append(line)
                            if not out_lines:
                                out_lines.append("// template missing: update output")
                            return out_lines

                        move_tokens = {
                            "X_IN": "x_in",
                            "Y_IN": "y_in",
                            "DIST_IN": "dist_in",
                            "TIMEOUT_MS": "timeout_ms",
                            "TIMEOUT_S": "timeout_ms / 1000.0",
                            "MS": "timeout_ms",
                            "S": "timeout_ms / 1000.0",
                            "FORWARDS": "forwards",
                            "HEADING_DEG": "heading_deg",
                            "TURN_DELTA_DEG": "delta_deg",
                            "LOOKAHEAD": "0",
                            "LEAD_IN": "0.0",
                            "DRIVE_MAX_V": "drive_max_v",
                            "HEADING_MAX_V": "heading_max_v",
                            "DRIVE_SETTLE_ERR": "drive_settle_err",
                            "DRIVE_SETTLE_TIME": "drive_settle_time",
                            "DRIVE_EARLY_EXIT": "0",
                            "DRIVE_MIN_SPEED": "0"
                        }
                        turn_tokens = {
                            "HEADING_DEG": "heading_deg",
                            "TURN_DELTA_DEG": "delta_deg",
                            "TIMEOUT_MS": "timeout_ms",
                            "TIMEOUT_S": "timeout_ms / 1000.0",
                            "MS": "timeout_ms",
                            "S": "timeout_ms / 1000.0",
                            "DIR": "CW_CLOCKWISE",
                            "SIDE": "LEFT",
                            "TURN_MAX_V": "turn_max_v",
                            "TURN_SETTLE_ERR": "turn_settle_err",
                            "TURN_SETTLE_TIME": "turn_settle_time",
                            "TURN_EARLY_EXIT": "0",
                            "TURN_MIN_SPEED": "0",
                            "SWING_MAX_V": "turn_max_v",
                            "SWING_SETTLE_ERR": "turn_settle_err",
                            "SWING_SETTLE_TIME": "turn_settle_time",
                            "SWING_EARLY_EXIT": "0",
                            "SWING_MIN_SPEED": "0"
                        }
                        move_lines = _format_tpl_lines(tpls.get(move_key, ""), move_tokens)
                        turn_lines = _format_tpl_lines(tpls.get(turn_key, ""), turn_tokens)
                        setpose_tpl = tpls.get("setpose", "")
                        setpose_lines = _format_tpl_lines(setpose_tpl, {
                            "X_IN": "x_in",
                            "Y_IN": "y_in",
                            "HEADING_DEG": "heading_deg"
                        }) if setpose_tpl else []
                        has_setpose = bool(setpose_tpl)

                        lines.append(f"static const bool CAL_HAS_SETPOS = {'true' if has_setpose else 'false'};")
                        lines.append("static const double CAL_PI = 3.141592653589793;")
                        lines.append("static double cal_last_target_x = 0.0;")
                        lines.append("static double cal_last_target_y = 0.0;")
                        lines.append("static double cal_last_target_heading = 0.0;")
                        lines.append("static int cal_last_timeout_ms = 0;")
                        lines.append("static double cal_drive_settle_err = 1.0;")
                        lines.append("static double cal_turn_settle_err = 1.0;")
                        lines.append("static int cal_drive_settle_time_ms = 120;")
                        lines.append("static int cal_turn_settle_time_ms = 120;")
                        lines.append("void cal_set_profile(const char* profile) { CAL_SET_PROFILE(profile); }")
                        lines.append("void cal_set_drive_cap(double volts) { CAL_SET_DRIVE_CAP(volts); }")
                        lines.append("void cal_set_turn_cap(double volts) { CAL_SET_TURN_CAP(volts); }")
                        lines.append("double cal_get_pose_x_in() { return CAL_GET_POSE_X(); }")
                        lines.append("double cal_get_pose_y_in() { return CAL_GET_POSE_Y(); }")
                        lines.append("double cal_get_heading_deg() { return CAL_GET_POSE_H(); }")
                        lines.append("void cal_set_pose(double x_in, double y_in, double heading_deg) {")
                        if setpose_lines:
                            for line in setpose_lines:
                                lines.append(f"  {line}")
                        else:
                            lines.append("  CAL_SET_POSE(x_in, y_in, heading_deg);")
                        lines.append("}")
                        lines.append("static double cal_wrap_deg(double deg) {")
                        lines.append("  double out = std::fmod(deg, 360.0);")
                        lines.append("  if (out < 0) out += 360.0;")
                        lines.append("  return out;")
                        lines.append("}")
                        lines.append("static double cal_angle_error_deg(double target_deg, double current_deg) {")
                        lines.append("  double diff = std::fmod(target_deg - current_deg + 540.0, 360.0) - 180.0;")
                        lines.append("  return std::fabs(diff);")
                        lines.append("}")
                        lines.append("static int cal_drive_timeout_ms(double dist_in, double cap_frac) {")
                        lines.append("  double cap = cap_frac < 0.2 ? 0.2 : cap_frac;")
                        lines.append("  double ms = 500.0 + (std::fabs(dist_in) * 75.0) / cap;")
                        lines.append("  return (int)std::ceil(ms);")
                        lines.append("}")
                        lines.append("static int cal_turn_timeout_ms(double angle_deg, double cap_frac) {")
                        lines.append("  double cap = cap_frac < 0.2 ? 0.2 : cap_frac;")
                        lines.append("  double ms = 500.0 + (std::fabs(angle_deg) * 12.0) / cap;")
                        lines.append("  return (int)std::ceil(ms);")
                        lines.append("}")
                        lines.append("static double cal_drive_error_in(double target_x, double target_y) {")
                        lines.append("  double dx = cal_get_pose_x_in() - target_x;")
                        lines.append("  double dy = cal_get_pose_y_in() - target_y;")
                        lines.append("  return std::sqrt(dx * dx + dy * dy);")
                        lines.append("}")
                        lines.append("static double cal_turn_error_deg(double target_deg) {")
                        lines.append("  return cal_angle_error_deg(target_deg, cal_get_heading_deg());")
                        lines.append("}")
                        lines.append("static int cal_measure_settle_ms_drive(double target_x, double target_y, int start_ms, double err_in, int hold_ms, int max_ms) {")
                        lines.append("  int inside_ms = 0;")
                        lines.append("  while (inside_ms < hold_ms && (pros::millis() - start_ms) < max_ms) {")
                        lines.append("    double err = cal_drive_error_in(target_x, target_y);")
                        lines.append("    if (err <= err_in) inside_ms += 10; else inside_ms = 0;")
                        lines.append("    pros::delay(10);")
                        lines.append("  }")
                        lines.append("  return pros::millis() - start_ms;")
                        lines.append("}")
                        lines.append("static int cal_measure_settle_ms_turn(double target_deg, int start_ms, double err_deg, int hold_ms, int max_ms) {")
                        lines.append("  int inside_ms = 0;")
                        lines.append("  while (inside_ms < hold_ms && (pros::millis() - start_ms) < max_ms) {")
                        lines.append("    double err = cal_angle_error_deg(target_deg, cal_get_heading_deg());")
                        lines.append("    if (err <= err_deg) inside_ms += 10; else inside_ms = 0;")
                        lines.append("    pros::delay(10);")
                        lines.append("  }")
                        lines.append("  return pros::millis() - start_ms;")
                        lines.append("}")
                        lines.append("void cal_drive_to(double dist_in, double cap_frac) {")
                        lines.append("  double heading_deg = cal_get_heading_deg();")
                        lines.append("  double x_in = cal_get_pose_x_in();")
                        lines.append("  double y_in = cal_get_pose_y_in();")
                        lines.append("  double delta_deg = 0.0;")
                        lines.append("  if (CAL_HAS_SETPOS) {")
                        lines.append("    cal_set_pose(0.0, 0.0, 0.0);")
                        lines.append("    heading_deg = 0.0;")
                        lines.append("    x_in = dist_in;")
                        lines.append("    y_in = 0.0;")
                        lines.append("  } else {")
                        lines.append("    double rad = heading_deg * (CAL_PI / 180.0);")
                        lines.append("    x_in = x_in + dist_in * std::cos(rad);")
                        lines.append("    y_in = y_in + dist_in * std::sin(rad);")
                        lines.append("  }")
                        lines.append("  cal_last_target_x = x_in;")
                        lines.append("  cal_last_target_y = y_in;")
                        lines.append("  cal_last_target_heading = heading_deg;")
                        lines.append("  const bool forwards = dist_in >= 0.0;")
                        lines.append("  const double drive_max_v = cap_frac * 12.0;")
                        lines.append("  const double heading_max_v = cap_frac * 12.0;")
                        lines.append("  const double drive_settle_err = cal_drive_settle_err;")
                        lines.append("  const int drive_settle_time = cal_drive_settle_time_ms;")
                        lines.append("  int timeout_ms = cal_drive_timeout_ms(dist_in, cap_frac);")
                        lines.append("  cal_last_timeout_ms = timeout_ms;")
                        lines.append(f"  // Template: {move_key}")
                        for line in move_lines:
                            lines.append(f"  {line}")
                        lines.append("}")
                        lines.append("void cal_turn_to(double heading_deg, double cap_frac) {")
                        lines.append("  double delta_deg = heading_deg;")
                        lines.append("  double turn_mag = std::fabs(delta_deg);")
                        if turn_key == "turn_local":
                            lines.append("  heading_deg = cal_wrap_deg(cal_get_heading_deg() + delta_deg);")
                            lines.append("  turn_mag = std::fabs(delta_deg);")
                        else:
                            lines.append("  turn_mag = cal_angle_error_deg(heading_deg, cal_get_heading_deg());")
                        lines.append("  cal_last_target_heading = heading_deg;")
                        lines.append("  const double turn_max_v = cap_frac * 12.0;")
                        lines.append("  const double turn_settle_err = cal_turn_settle_err;")
                        lines.append("  const int turn_settle_time = cal_turn_settle_time_ms;")
                        lines.append("  int timeout_ms = cal_turn_timeout_ms(turn_mag, cap_frac);")
                        lines.append("  cal_last_timeout_ms = timeout_ms;")
                        lines.append(f"  // Template: {turn_key}")
                        for line in turn_lines:
                            lines.append(f"  {line}")
                        lines.append("}")
                        lines.append("static void cal_log(const char* move_type, const char* profile, double cap_frac,")
                        lines.append("                   double magnitude, double final_err, int settle_ms) {")
                        lines.append("  printf(\"{\\\"move_type\\\":\\\"%s\\\",\\\"profile\\\":\\\"%s\\\",\\\"cap_frac\\\":%.2f,\",")
                        lines.append("         move_type, profile, cap_frac);")
                        lines.append("  if (std::strcmp(move_type, \"drive\") == 0) {")
                        lines.append("    printf(\"\\\"distance_in\\\":%.2f,\", magnitude);")
                        lines.append("  } else {")
                        lines.append("    printf(\"\\\"angle_deg\\\":%.2f,\", magnitude);")
                        lines.append("  }")
                        lines.append("  printf(\"\\\"final_err\\\":%.3f,\\\"settle_ms\\\":%d}\\n\", final_err, settle_ms);")
                        lines.append("}")
                        lines.append("")
                        profiles_list = ", ".join([f"\"{p}\"" for p in pl.get("profiles", [])])
                        caps_list = ", ".join([f"{c:.2f}" for c in pl.get("caps", [])])
                        drive_list = ", ".join([f"{d:.2f}" for d in pl.get("drive_in", [])])
                        turn_list = ", ".join([f"{t:.2f}" for t in pl.get("turn_deg", [])])
                        repeats_val = int(pl.get("repeat", 1))
                        lines.append("void run_calibration_tests() {")
                        lines.append(f"  const char* profiles[] = {{{profiles_list}}};")
                        lines.append(f"  const double caps[] = {{{caps_list}}};")
                        lines.append(f"  const double drive_in[] = {{{drive_list}}};")
                        lines.append(f"  const double turn_deg[] = {{{turn_list}}};")
                        lines.append(f"  const int repeats = {repeats_val};")
                        lines.append("  const double drive_settle_err = 1.0;")
                        lines.append("  const double turn_settle_err = 1.0;")
                        lines.append("  const int settle_hold_ms = 120;")
                        lines.append("  cal_drive_settle_err = drive_settle_err;")
                        lines.append("  cal_turn_settle_err = turn_settle_err;")
                        lines.append("  cal_drive_settle_time_ms = settle_hold_ms;")
                        lines.append("  cal_turn_settle_time_ms = settle_hold_ms;")
                        lines.append("  const int profile_count = sizeof(profiles) / sizeof(profiles[0]);")
                        lines.append("  const int cap_count = sizeof(caps) / sizeof(caps[0]);")
                        lines.append("  const int drive_count = sizeof(drive_in) / sizeof(drive_in[0]);")
                        lines.append("  const int turn_count = sizeof(turn_deg) / sizeof(turn_deg[0]);")
                        lines.append("")
                        lines.append("  for (int p = 0; p < profile_count; ++p) {")
                        lines.append("    cal_set_profile(profiles[p]);")
                        lines.append("    for (int c = 0; c < cap_count; ++c) {")
                        lines.append("      const double cap_frac = caps[c];")
                        lines.append("      const double cap_v = cap_frac * 12.0;")
                        lines.append("      for (int r = 0; r < repeats; ++r) {")
                        lines.append("        for (int i = 0; i < drive_count; ++i) {")
                        lines.append("          const double dist = drive_in[i];")
                        lines.append("          cal_set_drive_cap(cap_v);")
                        lines.append("          const int t0 = pros::millis();")
                        lines.append("          cal_drive_to(dist, cap_frac);")
                        lines.append("          const double tx = cal_last_target_x;")
                        lines.append("          const double ty = cal_last_target_y;")
                        lines.append("          const int settle_ms = cal_measure_settle_ms_drive(tx, ty, t0, drive_settle_err, settle_hold_ms, cal_last_timeout_ms + 1000);")
                        lines.append("          const double err = cal_drive_error_in(tx, ty);")
                        lines.append("          cal_log(\"drive\", profiles[p], cap_frac, dist, err, settle_ms);")
                        lines.append("          pros::delay(120);")
                        lines.append("        }")
                        lines.append("        for (int i = 0; i < turn_count; ++i) {")
                        lines.append("          const double ang = turn_deg[i];")
                        lines.append("          if (CAL_HAS_SETPOS) {")
                        lines.append("            cal_set_pose(0.0, 0.0, 0.0);")
                        lines.append("          }")
                        lines.append("          cal_set_turn_cap(cap_v);")
                        lines.append("          const int t0 = pros::millis();")
                        lines.append("          cal_turn_to(ang, cap_frac);")
                        lines.append("          const double th = cal_last_target_heading;")
                        lines.append("          const int settle_ms = cal_measure_settle_ms_turn(th, t0, turn_settle_err, settle_hold_ms, cal_last_timeout_ms + 1000);")
                        lines.append("          const double err = cal_turn_error_deg(th);")
                        lines.append("          cal_log(\"turn\", profiles[p], cap_frac, ang, err, settle_ms);")
                        lines.append("          pros::delay(120);")
                        lines.append("        }")
                        lines.append("      }")
                        lines.append("    }")
                        lines.append("  }")
                        lines.append("}")
                        lines.append("")
                        lines.append(f"void {auton_fn}() {{")
                        lines.append("  run_calibration_tests();")
                        lines.append("}")
                        return "\n".join(lines) + "\n"

                    code = _emit_code(plan)
                    with open(path, "w", encoding="utf-8") as f:
                        f.write(code)
            except Exception as e:
                messagebox.showerror("Calibration", f"Failed to export code:\n{e}")

        def _get_log_text():
            raw = text.get("1.0", "end-1c")
            if raw.strip() == placeholder:
                return ""
            return raw

        def _update_key_status(raw_text: str):
            aliases = {
                "cap_frac": ["cap_frac", "cap", "cap_v"],
                "final_err": ["final_err", "error"],
                "settle_ms": ["settle_ms", "settle_time_ms"],
            }
            for key, lbl in key_labels.items():
                keys = aliases.get(key, [key])
                present = any(f"\"{k}\"" in raw_text for k in keys)
                lbl.configure(foreground=("#1b7f3a" if present else "#999999"))

        def _on_text_modified(_evt=None):
            text.edit_modified(False)
            raw_text = _get_log_text()
            _update_key_status(raw_text)
            if analysis_state.get("data") is not None:
                analysis_state["data"] = None
                try:
                    apply_btn.configure(state="disabled")
                except Exception:
                    pass
                analysis_summary_var.set("Logs changed. Analyze again to refresh results.")

        def _load_from_file():
            path = filedialog.askopenfilename(
                parent=win,
                title="Import log file",
                filetypes=[("Text files", "*.txt"), ("JSON files", "*.json"), ("All files", "*.*")]
            )
            if not path:
                return
            try:
                with open(path, "r", encoding="utf-8") as f:
                    data = f.read()
            except Exception as e:
                messagebox.showerror("Calibration", f"Failed to load file:\n{e}")
                return
            _clear_placeholder()
            text.delete("1.0", "end")
            text.insert("1.0", data.strip())
            _update_key_status(data)

        analysis_state = {"data": None}

        def _analyze():
            raw = _get_log_text().strip()
            if not raw:
                messagebox.showerror("Calibration", "No log data provided.")
                return
            result = _analyze_calibration_logs(raw)
            _update_key_status(raw)
            if not result.get("ok"):
                msg = "\n".join(result.get("errors", [])[:8])
                messagebox.showerror("Calibration", msg or "No valid log lines found.")
                analysis_summary_var.set("No valid logs found. Fix errors and analyze again.")
                analysis_state["data"] = None
                apply_btn.configure(state="disabled")
                return
            warnings = list(result.get("warnings", []))
            profiles_sel = [p for p in profile_order if profile_vars[p].get()]
            if not profiles_sel:
                profiles_sel = ["precise", "normal", "fast"]
            for p in profiles_sel:
                total = 0
                for mv in ("drive", "turn", "swing"):
                    total += result.get("profile_counts", {}).get((mv, p), 0)
                if total == 0:
                    warnings.append(f"Not enough repeats for {p} profile")
                elif total < 3:
                    warnings.append(f"Low repeats for {p} profile ({total})")
            expected_caps = _parse_caps(caps_var.get(), caps_mode_var.get())
            found_caps = set()
            profs = result.get("profiles", {})
            if isinstance(profs, dict):
                for mv in profs.values():
                    if not isinstance(mv, dict):
                        continue
                    for prof in mv.values():
                        if not isinstance(prof, dict):
                            continue
                        caps = prof.get("caps", {})
                        if isinstance(caps, dict):
                            for cap_key in caps.keys():
                                try:
                                    found_caps.add(float(cap_key))
                                except Exception:
                                    pass
            missing_caps = []
            for cap in expected_caps:
                if not any(abs(cap - fc) < 0.05 for fc in found_caps):
                    missing_caps.append(f"{cap:.2f}")
            if missing_caps:
                warnings.append("Caps missing: " + ", ".join(missing_caps))
            if result.get("errors"):
                msg = "\n".join(result["errors"][:8])
                messagebox.showwarning("Calibration", "Some lines were skipped:\n" + msg)
            noise = result.get("noise", {})
            def _fmt_noise(val):
                try:
                    return f"{float(val):.3f}"
                except Exception:
                    return "n/a"
            drive_noise = _fmt_noise(noise.get("drive_in", None))
            turn_noise = _fmt_noise(noise.get("turn_deg", None))
            def _median(vals):
                vals = sorted([v for v in vals if v is not None])
                if not vals:
                    return None
                mid = len(vals) // 2
                if len(vals) % 2:
                    return vals[mid]
                return 0.5 * (vals[mid - 1] + vals[mid])

            def _bucket_summary(move, prof):
                profs = result.get("profiles", {}).get(move, {})
                if not isinstance(profs, dict):
                    return {}
                prof_dict = profs.get(prof)
                if not isinstance(prof_dict, dict):
                    return {}
                caps = prof_dict.get("caps", {})
                if not isinstance(caps, dict):
                    return {}
                collected = {"small": [], "medium": [], "large": []}
                for cap_dict in caps.values():
                    if not isinstance(cap_dict, dict):
                        continue
                    for mag_key, bucket in cap_dict.items():
                        if not isinstance(bucket, dict):
                            continue
                        if mag_key not in collected:
                            continue
                        err = bucket.get("err_p90", bucket.get("err"))
                        settle = bucket.get("settle_ms_p90", bucket.get("settle_ms"))
                        collected[mag_key].append((err, settle))
                out = {}
                for mag_key, pairs in collected.items():
                    if not pairs:
                        continue
                    errs = [_median([p[0] for p in pairs if p[0] is not None])]
                    times = [_median([p[1] for p in pairs if p[1] is not None])]
                    err_val = errs[0]
                    time_val = times[0]
                    if err_val is None and time_val is None:
                        continue
                    out[mag_key] = (err_val, time_val)
                return out

            summary_lines = [
                "Collected",
                f"Drive trials: {result['counts'].get('drive', 0)} | Turn trials: {result['counts'].get('turn', 0)} | Swing trials: {result['counts'].get('swing', 0)}",
                f"Noise: drive {drive_noise} in, turn {turn_noise} deg",
            ]
            dyn = result.get("dynamics", {})
            if isinstance(dyn, dict) and dyn:
                dyn_parts = []
                if "vmax_ips" in dyn:
                    dyn_parts.append(f"vmax {dyn['vmax_ips']:.1f} ips")
                if "accel_ips2" in dyn:
                    dyn_parts.append(f"accel {dyn['accel_ips2']:.1f} ips^2")
                if "turn_rate_dps" in dyn:
                    dyn_parts.append(f"turn {dyn['turn_rate_dps']:.1f} dps")
                if "turn_accel_dps2" in dyn:
                    dyn_parts.append(f"turn accel {dyn['turn_accel_dps2']:.1f} dps^2")
                if dyn_parts:
                    summary_lines.append("Dynamics: " + ", ".join(dyn_parts))
            for move_label, move_key in (("Drive", "drive"), ("Turn", "turn")):
                for prof in profiles_sel:
                    bucket_vals = _bucket_summary(move_key, prof)
                    if not bucket_vals:
                        continue
                    parts = []
                    for mag in ("small", "medium", "large"):
                        if mag not in bucket_vals:
                            continue
                        err_val, time_val = bucket_vals[mag]
                        if err_val is None and time_val is None:
                            continue
                        err_txt = f"{err_val:.2f}" if err_val is not None else "-"
                        time_txt = f"{int(round(time_val))}ms" if time_val is not None else "-"
                        parts.append(f"{mag[0].upper()} {err_txt}/{time_txt}")
                    if parts:
                        summary_lines.append(f"{move_label} {prof}: " + ", ".join(parts))
            if warnings:
                summary_lines.append("Warnings: " + "; ".join(warnings))
            if result.get("errors"):
                summary_lines.append("Errors: " + "; ".join(result["errors"][:3]))
            analysis_summary_var.set("\n".join(summary_lines))
            analysis_state["data"] = {
                "profiles": result["profiles"],
                "noise": result.get("noise", {}),
                "trial_count": result.get("trial_count", 0),
                "dynamics": result.get("dynamics", {})
            }
            apply_btn.configure(state="normal")

        def _apply():
            data = analysis_state.get("data")
            if not data:
                messagebox.showerror("Calibration", "Analyze logs before applying.")
                return
            cal = CFG.setdefault("codegen", {}).setdefault("calibration", {})
            cal["profiles"] = data.get("profiles", {})
            if data.get("noise"):
                cal.setdefault("noise", {}).update(data["noise"])
            if data.get("dynamics"):
                cal["dynamics"] = data["dynamics"]
            else:
                cal.pop("dynamics", None)
            constants = _derive_cal_constants_from_dynamics(cal.get("dynamics", {}))
            if constants:
                cal["constants"] = constants
            else:
                cal.pop("constants", None)
            cal["enabled"] = 1
            cal["last_run"] = datetime.now().isoformat(timespec="seconds")
            cal["trial_count"] = int(data.get("trial_count", 0))
            try:
                tw_val = int(track_var.get() or 0)
            except Exception:
                tw_val = 0
            tracking_wheels_var.set(str(max(0, min(2, tw_val))))
            _sync_cal_vars_from_cfg()
            on_update()
            refresh_cb = constants_win.get("refresh")
            if callable(refresh_cb):
                try:
                    refresh_cb()
                except Exception:
                    pass
            messagebox.showinfo("Calibration", "Calibration applied.")

        def _copy_example():
            try:
                win.clipboard_clear()
                win.clipboard_append(example_line)
            except Exception:
                pass

        placeholder = "Paste robot JSON log lines here..."
        text.insert("1.0", placeholder)
        text.configure(foreground="#888888")

        def _clear_placeholder(_evt=None):
            if text.get("1.0", "end-1c") == placeholder:
                text.delete("1.0", "end")
                text.configure(foreground="#000000")

        def _restore_placeholder(_evt=None):
            if not text.get("1.0", "end-1c").strip():
                text.insert("1.0", placeholder)
                text.configure(foreground="#888888")

        text.bind("<FocusIn>", _clear_placeholder)
        text.bind("<FocusOut>", _restore_placeholder)
        text.bind("<<Modified>>", _on_text_modified)

        ttk.Button(btns, text="Load from file...", command=_load_from_file).grid(row=0, column=0, sticky="w")
        ttk.Button(btns, text="Copy example log line", command=_copy_example).grid(row=0, column=1, padx=(6, 0))
        ttk.Button(btns, text="Analyze Logs", command=_analyze).grid(row=0, column=2, padx=(6, 0))
        apply_btn = ttk.Button(btns, text="Apply Calibration", command=_apply, state="disabled")
        apply_btn.grid(row=0, column=3, padx=(6, 0))
        ttk.Button(btns, text="Export Robot Test Code...", command=_export_plan).grid(row=0, column=4, padx=(6, 0))
        ttk.Button(btns, text="Close", command=win.destroy).grid(row=0, column=5, sticky="e", padx=(6, 0))

        preset_var.trace_add("write", _apply_preset)
        track_var.trace_add("write", _refresh_drive_err_note)
        caps_var.trace_add("write", _update_caps_interp)
        caps_mode_var.trace_add("write", _update_caps_interp)
        _apply_preset()
        _refresh_drive_err_note()
        _update_caps_interp()

    cal_frame = ttk.LabelFrame(physics_body, text="Calibration")
    cal_frame.grid(row=11, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 2))
    cal_frame.columnconfigure(1, weight=1)
    cal_frame.columnconfigure(2, weight=1)
    cal_frame.columnconfigure(3, weight=0)

    cal_use_chk = ttk.Checkbutton(cal_frame, text="Use calibration", variable=cal_enabled_var)
    cal_use_chk.grid(row=0, column=0, sticky="w", padx=4, pady=2)
    cal_status_lbl = ttk.Label(cal_frame, textvariable=cal_status_var)
    cal_status_lbl.grid(row=0, column=1, columnspan=2, sticky="w", padx=6)
    cal_run_inline_btn = ttk.Button(cal_frame, text="Run wizard...", command=_run_calibration_wizard)
    cal_run_inline_btn.grid(row=0, column=3, sticky="e", padx=4, pady=2)

    cal_detail_lbl = ttk.Label(cal_frame, textvariable=cal_detail_var, foreground="#555555", wraplength=360, justify="left")
    cal_detail_lbl.grid(row=1, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 4))

    ttk.Label(cal_frame, text="Error scale (x):").grid(row=2, column=0, sticky="w", padx=4, pady=2)
    cal_err_entry = ttk.Entry(cal_frame, textvariable=cal_err_scale_var, width=8)
    cal_err_entry.grid(row=2, column=1, sticky="w", padx=(0, 12))
    ttk.Label(cal_frame, text="Time scale (x):").grid(row=2, column=2, sticky="w", padx=4, pady=2)
    cal_time_entry = ttk.Entry(cal_frame, textvariable=cal_time_scale_var, width=8)
    cal_time_entry.grid(row=2, column=3, sticky="w")

    ttk.Label(cal_frame, text="Noise mult (x):").grid(row=3, column=0, sticky="w", padx=4, pady=2)
    cal_noise_entry = ttk.Entry(cal_frame, textvariable=cal_noise_mult_var, width=8)
    cal_noise_entry.grid(row=3, column=1, sticky="w", padx=(0, 12))
    ttk.Label(cal_frame, text="Noise drive (in):").grid(row=3, column=2, sticky="w", padx=4, pady=2)
    cal_noise_drive_entry = ttk.Entry(cal_frame, textvariable=cal_noise_drive_var, width=8)
    cal_noise_drive_entry.grid(row=3, column=3, sticky="w")

    ttk.Label(cal_frame, text="Noise turn (deg):").grid(row=4, column=0, sticky="w", padx=4, pady=2)
    cal_noise_turn_entry = ttk.Entry(cal_frame, textvariable=cal_noise_turn_var, width=8)
    cal_noise_turn_entry.grid(row=4, column=1, sticky="w", padx=(0, 12))

    cal_help_lbl = ttk.Label(
        cal_frame,
        text="Error/Time scale multiply calibrated values after calibration is applied (safety margin). "
             "Noise mult sets minimum error = noise x mult. Noise drive/turn override jitter floor when logs omit it.",
        foreground="#666666",
        wraplength=360,
        justify="left"
    )
    cal_help_lbl.grid(row=5, column=0, columnspan=4, sticky="w", padx=4, pady=(2, 2))

    cal_warn_lbl = ttk.Label(cal_frame, textvariable=cal_warn_var, foreground="#b00020")
    cal_warn_lbl.grid(row=6, column=0, columnspan=4, sticky="w", padx=4, pady=(0, 2))

    cal_btns = ttk.Frame(cal_frame)
    cal_btns.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(4, 2))
    ttk.Button(cal_btns, text="Import JSON...", command=_import_calibration).pack(side="left")
    cal_export_btn = ttk.Button(cal_btns, text="Export JSON...", command=_export_calibration)
    cal_export_btn.pack(side="left", padx=(6, 0))
    ttk.Button(cal_btns, text="Reset recommended", command=_reset_calibration_recommended).pack(side="left", padx=(6, 0))
    ttk.Button(cal_btns, text="Clear calibration", command=_clear_calibration).pack(side="left", padx=(6, 0))

    for w in (cal_use_chk, cal_err_entry, cal_time_entry, cal_noise_entry, cal_noise_drive_entry, cal_noise_turn_entry):
        ui.track_live_widget(w)
    ui.add_tooltip(cal_use_chk, "Enable calibrated settle windows and times when available.")
    ui.add_tooltip(cal_err_entry, "Safety multiplier for calibrated settle error.")
    ui.add_tooltip(cal_time_entry, "Safety multiplier for calibrated settle time.")
    ui.add_tooltip(cal_noise_entry, "Multiplier applied to sensor noise floor.")
    ui.add_tooltip(cal_noise_drive_entry, "Measured drive/pose noise floor in inches.")
    ui.add_tooltip(cal_noise_turn_entry, "Measured turn/heading noise floor in degrees.")
    ui.add_tooltip(cal_status_lbl, "Calibration status and last run information.")
    ui.add_tooltip(cal_detail_lbl, "Summary of calibration coverage.")
    _refresh_cal_summary()
    _refresh_adv_visibility()
    try:
        cal_enabled_var.trace_add("write", lambda *_: _refresh_cal_summary())
    except Exception:
        pass

    def _refresh_ticks_visibility(*_):
        show = dist_var.get() == dist_labels[3]
        for w in (ticks_label, ticks_entry):
            try:
                if show:
                    w.grid()
                else:
                    w.grid_remove()
            except Exception:
                pass
    _refresh_ticks_visibility()
    try:
        dist_var.trace_add("write", lambda *_: _refresh_ticks_visibility())
    except Exception:
        pass
    
    # Scrollable template panel
    tabs["codegen"].rowconfigure(5, weight=1)
    tabs["codegen"].columnconfigure(1, weight=1)
    tpl_container = ttk.Frame(tabs["codegen"])
    tpl_container.grid(row=5, column=0, columnspan=2, sticky="nsew", padx=6, pady=6)
    tpl_container.rowconfigure(0, weight=1)
    tpl_container.columnconfigure(0, weight=1)
    tabs["codegen"].rowconfigure(5, weight=1)
    tabs["codegen"].columnconfigure(0, weight=1)

    tpl_panel = ttk.Frame(tpl_container, style="TFrame")
    tpl_panel.grid(row=0, column=0, sticky="nsew")
    tpl_canvas = tpl_panel  # placeholder to satisfy bindings
    tpl_win = None

    def _tpl_on_configure(event=None):
        pass

    def _tpl_mousewheel(event):
        pass

    def _bind_tpl_live_handlers():
        """Attach live update events to tracked widgets in template panel."""
        for w in list(ui.live_update_widgets):
            try:
                if not getattr(w, "winfo_exists", lambda: False)():
                    continue
                if isinstance(w, ttk.Entry):
                    for ev in ("<FocusOut>", "<Return>"):
                        w.bind(ev, lambda e: on_update())
                elif isinstance(w, ttk.Combobox):
                    for ev in ("<FocusOut>", "<<ComboboxSelected>>", "<Return>"):
                        w.bind(ev, lambda e: on_update())
                elif isinstance(w, ttk.Checkbutton):
                    w.configure(command=on_update)
            except Exception:
                continue

    def _active_optional_for(style_name: str):
        stored = CFG.get("codegen", {}).get("templates", {}).get(style_name, {})
        return list(stored.get("__optional__", []))

    def _set_active_optional(style_name: str, lst):
        CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})["__optional__"] = list(lst)
        return list(lst)

    def _add_optional(key):
        style = codegen_style_var.get()
        active = _active_optional_for(style)
        if key not in active:
            active.append(key)
            _set_active_optional(style, active)
            _rebuild_tpl_panel()

    def _remove_optional(key):
        style = codegen_style_var.get()
        active = _active_optional_for(style)
        if key in active:
            active.remove(key)
            _set_active_optional(style, active)
            _rebuild_tpl_panel()

    def _default_modes(style_name: str):
        return {
            "motion": "move",
            "turn": "turn_global" if style_name == "LemLib" else "turn_local"
        }

    def _current_modes(style_name: str):
        stored = CFG.get("codegen", {}).get("templates", {}).get(style_name, {})
        modes = _default_modes(style_name)
        m = stored.get("__modes__", {}) if isinstance(stored, dict) else {}
        if m.get("motion") in ("move", "pose"):
            modes["motion"] = m["motion"]
        if m.get("turn") in ("turn_local", "turn_global"):
            modes["turn"] = m["turn"]
        motion_mode_var.set(modes["motion"])
        turn_mode_var.set(modes["turn"])
        return modes

    def _set_modes(style_name: str, modes: dict):
        CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})["__modes__"] = dict(modes)
        motion_mode_var.set(modes.get("motion", "move"))
        turn_mode_var.set(modes.get("turn", "turn_global" if style_name == "LemLib" else "turn_local"))
        return modes

    def _fill_tpl_vars_for(style_name: str):
        """Load template values into UI variables."""
        if style_name == "Action List":
            return
        stored = CFG.get("codegen", {}).get("templates", {}).get(style_name, {})
        defaults = codegen_defaults.get(style_name, codegen_defaults["Custom"]).copy()
        for k in tpl_keys:
            val = (stored or {}).get(k)
            if val is None or val == "":
                tpl_vars[k].set(defaults.get(k, ""))
            else:
                tpl_vars[k].set(val)
        _current_modes(style_name)

    def _save_tpl_vars_for(style_name: str):
        """Save UI variables to config."""
        if style_name == "Action List":
            return
        tdict = CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})
        active_opt = _active_optional_for(style_name)
        for k in base_tpl_keys + optional_pool:
            if k in optional_pool and k not in active_opt:
                tdict.pop(k, None)
                continue
            tdict[k] = tpl_vars[k].get()
        tdict["__optional__"] = active_opt
        _set_modes(style_name, _current_modes(style_name))

    def _apply_mode_selection(style_name: str, motion_val=None, turn_val=None):
        modes = _current_modes(style_name)
        if motion_val in ("move", "pose"):
            modes["motion"] = motion_val
        if turn_val in ("turn_local", "turn_global"):
            modes["turn"] = turn_val
        _set_modes(style_name, modes)
        return modes

    def _on_mode_change(_=None):
        style = codegen_style_var.get()
        modes = _apply_mode_selection(style, motion_mode_var.get(), turn_mode_var.get())
        _set_modes(style, modes)
        _rebuild_tpl_panel()

    template_builder_win = {"win": None}
    mech_preset_win = {"win": None}

    def _open_template_builder():
        style = codegen_style_var.get()
        if style == "Action List":
            messagebox.showinfo("Edit templates", "Action List mode has no templates to edit.")
            return
        _fill_tpl_vars_for(style)
        if template_builder_win["win"] is not None and template_builder_win["win"].winfo_exists():
            template_builder_win["win"].lift()
            template_builder_win["win"].focus_force()
            return

        win = tk.Toplevel(top)
        template_builder_win["win"] = win
        win.title(f"Edit Command Templates ({style})")
        win.geometry("760x520")

        container = ttk.Frame(win, padding=8)
        container.pack(fill="both", expand=True)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)

        left = ttk.Frame(container)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)
        left.rowconfigure(2, weight=0)

        options_frame = ttk.LabelFrame(left, text="Command options")
        options_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        options_frame.columnconfigure(0, weight=1)

        modes = _current_modes(style)
        turn_choice_var = tk.StringVar(value=modes.get("turn", "turn_global"))
        turn_row = ttk.Frame(options_frame)
        turn_row.pack(fill="x", padx=6, pady=(6, 2))
        ttk.Label(turn_row, text="Turn command:").pack(side="left")
        turn_global_rb = ttk.Radiobutton(turn_row, text="Turn global", variable=turn_choice_var, value="turn_global")
        turn_global_rb.pack(side="left", padx=(10, 4))
        turn_local_rb = ttk.Radiobutton(turn_row, text="Turn local", variable=turn_choice_var, value="turn_local")
        turn_local_rb.pack(side="left")
        warn_lbl = ttk.Label(options_frame, text="Turn global and turn local are mutually exclusive.", foreground="#b00020")
        warn_lbl.pack(fill="x", padx=6, pady=(0, 6))

        opt_row = ttk.Frame(options_frame)
        opt_row.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(opt_row, text="Optional commands:").pack(side="left")
        opt_map = {
            "reverse_on": "Reverse on",
            "reverse_off": "Reverse off",
            "reshape": "Reshape",
            "setpose": "Set pose",
            "swing": "Swing"
        }
        active_optional = set(_active_optional_for(style))
        opt_vars = {}
        opt_widgets = []
        for key, label in opt_map.items():
            var = tk.BooleanVar(value=key in active_optional)
            opt_vars[key] = var
            chk = ttk.Checkbutton(opt_row, text=label, variable=var)
            chk.pack(side="left", padx=(8, 0))
            opt_widgets.append(chk)

        cmd_frame = ttk.LabelFrame(left, text="Active commands")
        cmd_frame.grid(row=1, column=0, sticky="nsew")
        cmd_frame.rowconfigure(0, weight=1)
        cmd_frame.columnconfigure(0, weight=1)
        cmd_list = tk.Listbox(cmd_frame, height=12, exportselection=False)
        cmd_scroll = ttk.Scrollbar(cmd_frame, orient="vertical", command=cmd_list.yview)
        cmd_list.configure(yscrollcommand=cmd_scroll.set)
        cmd_list.grid(row=0, column=0, sticky="nsew")
        cmd_scroll.grid(row=0, column=1, sticky="ns")

        palette_frame = ttk.LabelFrame(left, text="Placeholders (drag into template)")
        palette_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        palette_frame.columnconfigure(0, weight=1)
        palette_list = tk.Listbox(palette_frame, height=6, exportselection=False)
        palette_scroll = ttk.Scrollbar(palette_frame, orient="vertical", command=palette_list.yview)
        palette_list.configure(yscrollcommand=palette_scroll.set)
        palette_list.grid(row=0, column=0, sticky="nsew", padx=(6, 0), pady=6)
        palette_scroll.grid(row=0, column=1, sticky="ns", pady=6, padx=(0, 6))

        right = ttk.Frame(container)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=1)
        right.rowconfigure(2, weight=0)

        builder_frame = ttk.LabelFrame(right, text="Template parts (drag to reorder)")
        builder_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        builder_frame.columnconfigure(0, weight=1)
        builder_frame.rowconfigure(0, weight=1)
        parts_list = tk.Listbox(builder_frame, height=10, exportselection=False)
        parts_scroll = ttk.Scrollbar(builder_frame, orient="vertical", command=parts_list.yview)
        parts_list.configure(yscrollcommand=parts_scroll.set)
        parts_list.grid(row=0, column=0, sticky="nsew", padx=(6, 0), pady=6)
        parts_scroll.grid(row=0, column=1, sticky="ns", pady=6, padx=(0, 6))
        selected_part_idx = {"idx": None}

        preview_frame = ttk.LabelFrame(right, text="Preview")
        preview_frame.grid(row=1, column=0, sticky="nsew")
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        preview_text = tk.Text(preview_frame, height=6, wrap="word", font=("Consolas", 9))
        preview_scroll = ttk.Scrollbar(preview_frame, orient="vertical", command=preview_text.yview)
        preview_text.configure(yscrollcommand=preview_scroll.set)
        preview_text.grid(row=0, column=0, sticky="nsew", padx=(6, 0), pady=6)
        preview_scroll.grid(row=0, column=1, sticky="ns", pady=6, padx=(0, 6))
        preview_text.configure(state="disabled")

        controls = ttk.Frame(container)
        controls.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)
        left_btns = ttk.Frame(controls)
        left_btns.grid(row=0, column=0, sticky="w")
        right_btns = ttk.Frame(controls)
        right_btns.grid(row=0, column=1, sticky="e")
        ttk.Button(left_btns, text="Add Text", command=lambda: _add_text_part()).pack(side="left")
        ttk.Button(left_btns, text="Edit Text", command=lambda: _edit_text_part()).pack(side="left", padx=(6, 0))
        ttk.Button(left_btns, text="Remove", command=lambda: _remove_part()).pack(side="left", padx=(6, 0))
        ttk.Button(left_btns, text="Clear", command=lambda: _clear_parts()).pack(side="left", padx=(6, 0))
        ttk.Button(left_btns, text="Reset to Default", command=lambda: _reset_command()).pack(side="left", padx=(6, 0))
        ttk.Button(right_btns, text="Apply", command=lambda: _apply_current_command()).pack(side="left")
        ttk.Button(right_btns, text="Apply & Close", command=lambda: _apply_and_close()).pack(side="left", padx=(6, 0))

        token_re = re.compile(r"\{([A-Z_]+)\}")
        parts = []
        current_cmd = {"name": None}

        def _render_parts():
            parts_list.delete(0, "end")
            for part in parts:
                if part["kind"] == "token":
                    label = f"{{{part['value']}}}"
                else:
                    text_val = part["value"].replace("\n", "\\n")
                    label = f"\"{text_val}\""
                parts_list.insert("end", label)
            idx = selected_part_idx.get("idx")
            if idx is not None and 0 <= idx < len(parts):
                parts_list.selection_set(idx)
            _refresh_preview()

        def _refresh_preview():
            preview = "".join(
                f"{{{p['value']}}}" if p["kind"] == "token" else p["value"]
                for p in parts
            )
            preview_text.configure(state="normal")
            preview_text.delete("1.0", "end")
            preview_text.insert("1.0", preview)
            preview_text.configure(state="disabled")

        def _split_template(text):
            out = []
            idx = 0
            for match in token_re.finditer(text):
                if match.start() > idx:
                    chunk = text[idx:match.start()]
                    if chunk:
                        out.append({"kind": "text", "value": chunk})
                out.append({"kind": "token", "value": match.group(1)})
                idx = match.end()
            if idx < len(text):
                tail = text[idx:]
                if tail:
                    out.append({"kind": "text", "value": tail})
            return out

        def _allowed_tokens_for_cmd(cmd_name: str):
            base = {
                "wait": ["MS", "S"],
                "tbuffer": ["MS", "S"],
                "move": ["X_IN", "Y_IN", "DIST_IN", "TIMEOUT_MS", "TIMEOUT_S", "FORWARDS", "HEADING_DEG", "MOVE_SPEED"],
                "pose": ["X_IN", "Y_IN", "HEADING_DEG", "TIMEOUT_MS", "TIMEOUT_S", "FORWARDS", "LEAD_IN"],
                "turn_global": ["HEADING_DEG", "TIMEOUT_MS", "TIMEOUT_S", "TURN_SPEED"],
                "turn_local": ["TURN_DELTA_DEG", "TIMEOUT_MS", "TIMEOUT_S", "TURN_SPEED"],
                "swing": ["HEADING_DEG", "DIR", "SIDE", "TIMEOUT_MS", "TIMEOUT_S"],
                "path_follow": ["PATH_NAME", "PATH_FILE", "PATH_ASSET", "LOOKAHEAD", "TIMEOUT_MS", "TIMEOUT_S", "FORWARDS", "PATH_MIN_SPEED", "PATH_MAX_SPEED"],
                "setpose": ["X_IN", "Y_IN", "HEADING_DEG"],
                "reshape": ["STATE"],
                "reverse_on": [],
                "reverse_off": [],
                "marker_wait": ["MARKER_DIST_IN", "MARKER_FRAC", "MARKER_INDEX"],
                "marker_wait_done": []
            }
            tokens = list(base.get(cmd_name, []))
            if (style == "JAR" and adv_motion_var.get()) or style == "Custom":
                jar_tokens = {
                    "move": ["DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME"],
                    "pose": ["DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME"],
                    "path_follow": ["DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME"],
                    "turn_global": ["TURN_MAX_V", "TURN_SETTLE_ERR", "TURN_SETTLE_TIME"],
                    "turn_local": ["TURN_MAX_V", "TURN_SETTLE_ERR", "TURN_SETTLE_TIME"],
                    "swing": ["SWING_MAX_V", "SWING_SETTLE_ERR", "SWING_SETTLE_TIME"]
                }
                tokens.extend(jar_tokens.get(cmd_name, []))
            if style in ("LemLib", "Custom"):
                lemlib_tokens = {
                    "move": ["DRIVE_MIN_SPEED", "DRIVE_EARLY_EXIT"],
                    "pose": ["DRIVE_MIN_SPEED", "DRIVE_EARLY_EXIT"],
                    "turn_global": ["TURN_MIN_SPEED", "TURN_EARLY_EXIT"],
                    "turn_local": ["TURN_MIN_SPEED", "TURN_EARLY_EXIT"],
                    "swing": ["SWING_MIN_SPEED", "SWING_EARLY_EXIT"]
                }
                tokens.extend(lemlib_tokens.get(cmd_name, []))
            # Remove duplicates while preserving order
            seen = set()
            out = []
            for tok in tokens:
                if tok not in seen:
                    seen.add(tok)
                    out.append(tok)
            return out

        def _refresh_palette(cmd_name):
            palette_list.delete(0, "end")
            tokens = _allowed_tokens_for_cmd(cmd_name)
            if not tokens:
                tokens = list(template_tokens)
            for token in sorted(tokens):
                palette_list.insert("end", f"{{{token}}}")

        def _load_command(name):
            current_cmd["name"] = name
            tpl = tpl_vars.get(name).get()
            if not tpl:
                tpl = codegen_defaults.get(style, codegen_defaults["Custom"]).get(name, "")
            parts.clear()
            parts.extend(_split_template(tpl))
            _render_parts()
            _refresh_palette(name)

        def _active_cmds_for_builder():
            turn_key = turn_choice_var.get() or ("turn_global" if style == "LemLib" else "turn_local")
            base_cmds = ["wait", "move", "pose", turn_key, "path_follow", "tbuffer"]
            if style in ("LemLib", "Custom"):
                base_cmds += ["marker_wait", "marker_wait_done"]
            optional_cmds = [k for k in optional_pool if opt_vars.get(k) and opt_vars[k].get()]
            return base_cmds + optional_cmds

        def _refresh_cmd_list(select_name=None):
            cmd_list.delete(0, "end")
            active_cmds = _active_cmds_for_builder()
            for key in active_cmds:
                cmd_list.insert("end", key)
            if not active_cmds:
                return
            if select_name in active_cmds:
                idx = active_cmds.index(select_name)
            else:
                idx = 0
            cmd_list.selection_set(idx)
            cmd_list.see(idx)
            _load_command(active_cmds[idx])

        def _update_optional():
            active = [k for k, var in opt_vars.items() if var.get()]
            _set_active_optional(style, active)
            _refresh_cmd_list(current_cmd["name"])
            on_update()

        def _update_turn_choice():
            modes = {"motion": motion_mode_var.get(), "turn": turn_choice_var.get()}
            _set_modes(style, modes)
            _refresh_cmd_list(current_cmd["name"])
            on_update()

        def _apply_current_command():
            name = current_cmd["name"]
            if not name:
                return
            rendered = "".join(
                f"{{{p['value']}}}" if p["kind"] == "token" else p["value"]
                for p in parts
            )
            tpl_vars[name].set(rendered)
            _save_tpl_vars_for(style)
            _rebuild_tpl_panel()
            on_update()

        def _apply_and_close():
            _apply_current_command()
            win.destroy()

        def _add_text_part():
            val = simpledialog.askstring("Add text", "Enter literal text to insert:", parent=win)
            if val is None:
                return
            parts.append({"kind": "text", "value": val})
            _render_parts()

        def _edit_text_part():
            idx = parts_list.curselection()
            if not idx and selected_part_idx.get("idx") is not None:
                idx = (selected_part_idx["idx"],)
            if not idx:
                for i, part in enumerate(parts):
                    if part["kind"] == "text":
                        parts_list.selection_set(i)
                        selected_part_idx["idx"] = i
                        idx = (i,)
                        break
            if not idx:
                messagebox.showinfo("Edit text", "No text parts yet. Use Add Text first.")
                return
            i = idx[0]
            if parts[i]["kind"] != "text":
                messagebox.showinfo("Edit text", "Select a text part to edit.")
                return
            val = simpledialog.askstring("Edit text", "Update literal text:", initialvalue=parts[i]["value"], parent=win)
            if val is None:
                return
            parts[i]["value"] = val
            _render_parts()

        def _remove_part():
            idx = parts_list.curselection()
            if not idx and selected_part_idx.get("idx") is not None:
                idx = (selected_part_idx["idx"],)
            if not idx:
                if not parts:
                    return
                idx = (len(parts) - 1,)
            rem_idx = idx[0]
            if rem_idx < 0 or rem_idx >= len(parts):
                return
            parts.pop(rem_idx)
            if parts:
                selected_part_idx["idx"] = min(rem_idx, len(parts) - 1)
            else:
                selected_part_idx["idx"] = None
            _render_parts()

        def _clear_parts():
            parts.clear()
            _render_parts()

        def _reset_command():
            name = current_cmd["name"]
            if not name:
                return
            tpl = codegen_defaults.get(style, codegen_defaults["Custom"]).get(name, "")
            parts.clear()
            parts.extend(_split_template(tpl))
            _render_parts()

        for w in opt_widgets:
            w.configure(command=_update_optional)
        turn_global_rb.configure(command=_update_turn_choice)
        turn_local_rb.configure(command=_update_turn_choice)

        drag_state = {"source": None, "index": None, "token": None}

        def _start_drag_palette(event):
            idx = palette_list.nearest(event.y)
            if idx < 0:
                return
            val = palette_list.get(idx)
            drag_state.update({"source": "palette", "index": None, "token": val.strip("{}")})

        def _start_drag_parts(event):
            idx = parts_list.nearest(event.y)
            if idx < 0:
                return
            parts_list.selection_clear(0, "end")
            parts_list.selection_set(idx)
            selected_part_idx["idx"] = idx
            drag_state.update({"source": "parts", "index": idx, "token": None})

        def _drop_on_parts_at(y_root=None, y_local=None):
            if not drag_state["source"]:
                return
            if y_local is None:
                if y_root is None:
                    return
                y_local = y_root - parts_list.winfo_rooty()
            insert_idx = parts_list.nearest(y_local)
            if y_local >= parts_list.winfo_height():
                insert_idx = len(parts)
            if drag_state["source"] == "palette":
                token = drag_state["token"]
                if token:
                    parts.insert(insert_idx, {"kind": "token", "value": token})
                    selected_part_idx["idx"] = insert_idx
                    _render_parts()
            elif drag_state["source"] == "parts":
                from_idx = drag_state["index"]
                if from_idx is not None and 0 <= from_idx < len(parts):
                    part = parts.pop(from_idx)
                    if insert_idx > from_idx:
                        insert_idx -= 1
                    insert_idx = max(0, min(insert_idx, len(parts)))
                    parts.insert(insert_idx, part)
                    selected_part_idx["idx"] = insert_idx
                    _render_parts()
            drag_state.update({"source": None, "index": None, "token": None})

        def _cancel_drag(_event=None):
            drag_state.update({"source": None, "index": None, "token": None})

        def _insert_token_at_end(event):
            idx = palette_list.nearest(event.y)
            if idx < 0:
                return
            token = palette_list.get(idx).strip("{}")
            parts.append({"kind": "token", "value": token})
            selected_part_idx["idx"] = len(parts) - 1
            _render_parts()

        def _drop_anywhere(event):
            if not drag_state["source"]:
                return
            widget = win.winfo_containing(event.x_root, event.y_root)
            if widget == parts_list:
                _drop_on_parts_at(y_root=event.y_root)
            else:
                _cancel_drag()

        palette_list.bind("<ButtonPress-1>", _start_drag_palette)
        palette_list.bind("<Double-Button-1>", _insert_token_at_end)
        parts_list.bind("<ButtonPress-1>", _start_drag_parts)
        parts_list.bind("<ButtonRelease-1>", lambda e: _drop_on_parts_at(y_local=e.y, y_root=e.y_root))
        parts_list.bind("<Double-Button-1>", lambda _e: _edit_text_part())
        parts_list.bind("<<ListboxSelect>>", lambda _e: selected_part_idx.__setitem__("idx", (parts_list.curselection() or [None])[0]))
        parts_list.bind("<Escape>", _cancel_drag)
        win.bind("<ButtonRelease-1>", _drop_anywhere, add="+")

        def _on_select_cmd(_evt=None):
            sel = cmd_list.curselection()
            if not sel:
                return
            name = cmd_list.get(sel[0])
            _load_command(name)

        cmd_list.bind("<<ListboxSelect>>", _on_select_cmd)
        _refresh_cmd_list()

    def _open_mech_preset_editor():
        if mech_preset_win["win"] is not None and mech_preset_win["win"].winfo_exists():
            mech_preset_win["win"].lift()
            mech_preset_win["win"].focus_force()
            return
        presets = CFG.setdefault("codegen", {}).setdefault("mech_presets", [])
        if not isinstance(presets, list):
            presets = []
            CFG["codegen"]["mech_presets"] = presets

        win = tk.Toplevel(top)
        mech_preset_win["win"] = win
        win.title("Mechanism presets")
        win.geometry("720x420")
        win.minsize(620, 360)

        container = ttk.Frame(win, padding=8)
        container.pack(fill="both", expand=True)
        container.columnconfigure(1, weight=1)
        container.rowconfigure(0, weight=1)

        left = ttk.Frame(container)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        right = ttk.Frame(container)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(1, weight=1)

        preset_list = tk.Listbox(left, height=12, exportselection=False)
        preset_scroll = ttk.Scrollbar(left, orient="vertical", command=preset_list.yview)
        preset_list.configure(yscrollcommand=preset_scroll.set)
        preset_list.grid(row=0, column=0, sticky="nsew")
        preset_scroll.grid(row=0, column=1, sticky="ns")

        btn_row = ttk.Frame(left)
        btn_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(btn_row, text="Add", command=lambda: _add_preset()).pack(side="left")
        ttk.Button(btn_row, text="Remove", command=lambda: _remove_preset()).pack(side="left", padx=(6, 0))

        name_var = tk.StringVar()
        mode_var = tk.StringVar(value="Action")
        default_var = tk.IntVar(value=0)
        current_idx = {"idx": None}

        ttk.Label(right, text="Name:").grid(row=0, column=0, sticky="w", pady=(0, 4))
        name_entry = ttk.Entry(right, textvariable=name_var)
        name_entry.grid(row=0, column=1, sticky="ew", pady=(0, 4))

        ttk.Label(right, text="Type:").grid(row=1, column=0, sticky="w", pady=(0, 4))
        mode_combo = ttk.Combobox(right, values=["Action", "Toggle"], state="readonly", textvariable=mode_var, width=12)
        mode_combo.grid(row=1, column=1, sticky="w", pady=(0, 4))

        default_chk = ttk.Checkbutton(right, text="Default ON", variable=default_var)
        default_chk.grid(row=2, column=1, sticky="w", pady=(0, 6))

        ttk.Label(right, text="Action template:").grid(row=3, column=0, sticky="nw")
        action_text = tk.Text(right, height=4, wrap="word")
        action_text.grid(row=3, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(right, text="Toggle ON template:").grid(row=4, column=0, sticky="nw")
        on_text = tk.Text(right, height=3, wrap="word")
        on_text.grid(row=4, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(right, text="Toggle OFF template:").grid(row=5, column=0, sticky="nw")
        off_text = tk.Text(right, height=3, wrap="word")
        off_text.grid(row=5, column=1, sticky="ew", pady=(0, 6))

        ttk.Label(right, text="Hint: Use {VALUE} in templates to insert a number from the marker action.").grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(4, 0)
        )

        def _preset_label(preset):
            name = str(preset.get("name", "")).strip() or "(unnamed)"
            mode = str(preset.get("mode", "action")).strip().lower()
            return f"{name} ({mode})"

        def _refresh_list(select_idx=None):
            preset_list.delete(0, "end")
            for p in presets:
                preset_list.insert("end", _preset_label(p))
            if presets:
                idx = 0 if select_idx is None else max(0, min(select_idx, len(presets) - 1))
                preset_list.selection_set(idx)
                preset_list.see(idx)
                _load_selected(idx)

        def _set_text(widget, text_val):
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.insert("1.0", text_val)

        def _get_text(widget):
            return widget.get("1.0", "end-1c").strip()

        def _refresh_mode_visibility():
            mode = mode_var.get().strip().lower()
            is_toggle = mode == "toggle"
            default_chk.configure(state="normal" if is_toggle else "disabled")
            on_text.configure(state="normal" if is_toggle else "disabled")
            off_text.configure(state="normal" if is_toggle else "disabled")
            action_text.configure(state="normal" if not is_toggle else "disabled")

        def _save_current():
            idx = current_idx.get("idx")
            if idx is None or idx >= len(presets):
                return
            preset = presets[idx]
            name = name_var.get().strip() or f"Preset {idx + 1}"
            mode = mode_var.get().strip().lower()
            if mode not in ("action", "toggle"):
                mode = "action"
            preset["name"] = name
            preset["mode"] = mode
            preset["default"] = bool(default_var.get())
            preset["template"] = _get_text(action_text)
            preset["on"] = _get_text(on_text)
            preset["off"] = _get_text(off_text)
            _refresh_list(idx)

        def _load_selected(idx):
            if idx is None or idx >= len(presets):
                return
            current_idx["idx"] = idx
            preset = presets[idx]
            name_var.set(str(preset.get("name", "")))
            mode_val = str(preset.get("mode", "action")).strip().lower()
            mode_var.set("Toggle" if mode_val == "toggle" else "Action")
            default_var.set(1 if preset.get("default") else 0)
            _set_text(action_text, str(preset.get("template", "")))
            _set_text(on_text, str(preset.get("on", "")))
            _set_text(off_text, str(preset.get("off", "")))
            _refresh_mode_visibility()

        def _add_preset():
            presets.append({
                "name": f"Preset {len(presets) + 1}",
                "mode": "action",
                "template": "",
                "on": "",
                "off": "",
                "default": False
            })
            _refresh_list(len(presets) - 1)

        def _remove_preset():
            sel = preset_list.curselection()
            if not sel:
                return
            idx = sel[0]
            presets.pop(idx)
            current_idx["idx"] = None
            _refresh_list(idx if idx < len(presets) else len(presets) - 1)

        def _on_select(_evt=None):
            sel = preset_list.curselection()
            if not sel:
                return
            _save_current()
            _load_selected(sel[0])

        def _apply_and_close():
            _save_current()
            win.destroy()

        mode_combo.bind("<<ComboboxSelected>>", lambda _e: _refresh_mode_visibility())
        preset_list.bind("<<ListboxSelect>>", _on_select)

        action_text.bind("<FocusOut>", lambda _e: _save_current())
        on_text.bind("<FocusOut>", lambda _e: _save_current())
        off_text.bind("<FocusOut>", lambda _e: _save_current())
        name_entry.bind("<FocusOut>", lambda _e: _save_current())

        _refresh_list(0 if presets else None)

        btns = ttk.Frame(container)
        btns.grid(row=1, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(btns, text="Close", command=_apply_and_close).pack(side="right")

    # Expose builder in UI
    def _rebuild_tpl_panel(*_):
        """Rebuild the template editor panel based on selected style."""
        # Clear existing widgets
        for w in list(tpl_panel.children.values()):
            try: w.destroy()
            except Exception: pass
        
        style = codegen_style_var.get()
        if style == "Action List":
            ttk.Label(tpl_panel, text="Action List mode uses default log output.\nNo templates to configure.", 
                    justify="center").grid(row=0, column=0, columnspan=2, pady=20)
            return

        # Load current templates and active optionals
        _fill_tpl_vars_for(style)
        modes = _current_modes(style)
        active_optional = _active_optional_for(style)

        # Output options
        mode_row = ttk.Frame(tpl_panel)
        mode_row.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        ttk.Label(mode_row, text="Reshape output:").pack(side="left", padx=(10, 3))
        reshape_combo = ttk.Combobox(mode_row, width=10, values=["1/2", "true/false"], state="readonly", textvariable=reshape_output_var)
        reshape_combo.pack(side="left")
        reshape_combo.bind("<<ComboboxSelected>>", lambda _e: on_update())
        ui.add_tooltip(reshape_combo, "Choose reshape token format (1/2 or true/false).")
        ui.track_live_widget(reshape_combo)

        # Header
        hdr1 = ttk.Label(tpl_panel, text="CMD", font=("Segoe UI", 10, "bold"))
        hdr2 = ttk.Label(tpl_panel, text="Template", font=("Segoe UI", 10, "bold"))
        hdr1.grid(row=1, column=0, sticky="w", padx=(0, 6))
        hdr2.grid(row=1, column=1, sticky="ew")
        tpl_panel.columnconfigure(1, weight=1)
        
        # Help text for each token
        help_map = {
            "wait": "{MS} or {S} delay tokens. TIMEOUT_MS/S include pad- and min floor.",
            "move": "{DIST_IN}/{X_IN},{Y_IN} {FORWARDS}; {HEADING_DEG} (global) optional; {MOVE_SPEED} optional cmd (0-127) override.",
            "turn_global": "Field heading {HEADING_DEG}; {TURN_SPEED} optional dps.",
            "turn_local": "Relative turn {TURN_DELTA_DEG}; {TURN_SPEED} optional dps.",
            "pose": "{X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}; {FORWARDS}.",
            "setpose": "Set initial pose {X_IN},{Y_IN},{HEADING_DEG}",
            "reshape": "{STATE} 1=normal, 2=reshaped (or true/false depending on reshape output)",
            "reverse_on": "Toggle reverse drive ON",
            "reverse_off": "Toggle reverse drive OFF",
            "tbuffer": "{MS} or {S} for buffer wait",
            "path_follow": "Tokens: {PATH_NAME}, {PATH_FILE}, {PATH_ASSET}, {LOOKAHEAD}, {TIMEOUT_MS}, {FORWARDS}, {PATH_MIN_SPEED}, {PATH_MAX_SPEED} (cmd 0-127).",
            "swing": "{HEADING_DEG} target, {DIR}=AUTO/CW_CLOCKWISE/CCW_COUNTERCLOCKWISE, {SIDE}=LEFT/RIGHT/AUTO, {TIMEOUT_MS}.",
            "marker_wait": "Edge marker wait: {MARKER_DIST_IN} inches (or use {MARKER_FRAC} 0-1).",
            "marker_wait_done": "Finish motion after markers (e.g. waitUntilDone)."
        }
        if adv_motion_var.get() and style == "JAR":
            # Add JAR advanced placeholders to help text
            adv_drive_note = " Auto caps/settle: short=precise, long=fast; voltage caps scale with move size; settle err/time scale with move size and cap."
            adv_turn_note = " Auto caps/settle: small angles=precise, large=fast; voltage/settle scale with angle and cap."
            adv_swing_note = " Auto caps/settle: small swings=precise, large=fast; voltage/settle scale with swing angle and cap."
            help_map["move"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["pose"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["path_follow"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["turn_global"] += " JAR: {TURN_MAX_V}, {TURN_SETTLE_ERR}, {TURN_SETTLE_TIME}." + adv_turn_note
            help_map["turn_local"] += " JAR: {TURN_MAX_V}, {TURN_SETTLE_ERR}, {TURN_SETTLE_TIME}." + adv_turn_note
            help_map["swing"] += " JAR: {SWING_MAX_V}, {SWING_SETTLE_ERR}, {SWING_SETTLE_TIME}." + adv_swing_note
        
        # Active command list honoring selected modes (pose and turn can coexist)
        turn_key = modes.get("turn") or ("turn_global" if style == "LemLib" else "turn_local")
        base_cmds = ["wait", "move", "pose", turn_key, "path_follow", "tbuffer"]
        if style in ("LemLib", "Custom"):
            base_cmds += ["marker_wait", "marker_wait_done"]
        active_cmds = base_cmds + active_optional
        
        # Scrollable command list only
        list_container = ttk.Frame(tpl_panel)
        list_container.grid(row=2, column=0, columnspan=3, sticky="nsew", pady=(4, 4))
        list_container.rowconfigure(0, weight=1)
        list_container.columnconfigure(0, weight=1)
        list_canvas = tk.Canvas(list_container, highlightthickness=0, background=TEMPLATE_BG)
        list_vbar = ttk.Scrollbar(list_container, orient="vertical", command=list_canvas.yview)
        list_canvas.configure(yscrollcommand=list_vbar.set)
        list_canvas.grid(row=0, column=0, sticky="nsew")
        list_vbar.grid(row=0, column=1, sticky="ns")
        list_panel = ttk.Frame(list_canvas, style="TFrame")
        list_win = list_canvas.create_window((0, 0), window=list_panel, anchor="nw")

        def _on_list_config(_evt=None):
            try:
                list_canvas.configure(scrollregion=list_canvas.bbox("all"))
                list_canvas.itemconfigure(list_win, width=list_canvas.winfo_width())
            except Exception:
                pass
        list_panel.bind("<Configure>", _on_list_config)
        list_canvas.bind("<Configure>", _on_list_config)

        def _auto_size_text(txt: tk.Text):
            """Resize a Text widget height to fit content up to a small cap."""
            try:
                content = txt.get("1.0", "end-1c")
            except Exception:
                return
            lines = content.splitlines() or [""]
            max_len = max((len(line) for line in lines), default=0)
            est_lines = max(len(lines), max(1, max_len // 50 + 1))
            est_lines = max(2, min(6, est_lines))
            try:
                txt.configure(height=est_lines)
            except Exception:
                pass

        row_idx = 0
        for key in active_cmds:
            if key not in tpl_vars:
                continue
            ttk.Label(list_panel, text=key).grid(row=row_idx, column=0, sticky="w", padx=(0, 6), pady=2)
            txt = tk.Text(list_panel, wrap="word", height=2)
            txt.insert("1.0", tpl_vars[key].get())
            _auto_size_text(txt)
            txt.configure(state="disabled")
            txt.grid(row=row_idx, column=1, sticky="ew", pady=2)
            row_idx += 1
        list_panel.columnconfigure(1, weight=1)
        tpl_panel.rowconfigure(2, weight=1)
        
        # Action buttons (static)
        btns = ttk.Frame(tpl_panel)
        btns.grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(btns, text="Edit command templates...", command=_open_template_builder).pack(side="left", padx=(0, 6))
        ttk.Button(btns, text="Mechanism presets...", command=_open_mech_preset_editor).pack(side="left", padx=(0, 6))
        ttk.Button(btns, text="Reset to Defaults", command=_reset_defaults).pack(side="left")
        tpl_panel.after(0, _bind_tpl_live_handlers)

    try:
        adv_motion_var.trace_add("write", lambda *_: _rebuild_tpl_panel())
    except Exception:
        pass
        
    def _reset_defaults():
        style = codegen_style_var.get()
        # restore default optional commands (setpose)
        _set_active_optional(style, ["setpose"])
        _set_modes(style, _default_modes(style))
        for k in base_tpl_keys + optional_pool:
            if k in codegen_defaults.get(style, codegen_defaults["Custom"]):
                tpl_vars[k].set(codegen_defaults.get(style, codegen_defaults["Custom"]).get(k, ""))
        _rebuild_tpl_panel()

    def _persist_now():
        style = codegen_style_var.get()
        _save_tpl_vars_for(style)
        CFG.setdefault("codegen", {})["style"] = style
        CFG["codegen"].setdefault("opts", {}).update({
            "ticks_per_rotation": float(ticks_var.get() or 360),
            "pad_factor": float(pad_var.get() or 1.0),
            "min_timeout_s": float(min_s_var.get() or 0.0),
            "reshape_output": reshape_output_var.get() or "1/2"
        })
        tpl_panel.after(0, _bind_tpl_live_handlers)

    def _on_style_change(_=None):
        """Handle style dropdown change."""
        _rebuild_tpl_panel()
    style_widget.bind("<<ComboboxSelected>>", _on_style_change)

    def _bind_live_handlers():
        """Attach live update events to tracked widgets."""
        for w in list(ui.live_update_widgets):
            try:
                if not getattr(w, "winfo_exists", lambda: False)():
                    continue
                if isinstance(w, ttk.Entry):
                    for ev in ("<FocusOut>", "<Return>"):
                        w.bind(ev, lambda e: on_update())
                elif isinstance(w, ttk.Combobox):
                    for ev in ("<FocusOut>", "<<ComboboxSelected>>", "<Return>"):
                        w.bind(ev, lambda e: on_update())
                elif isinstance(w, ttk.Checkbutton):
                    w.configure(command=on_update)
            except Exception:
                continue

    def on_update():
        """Apply settings and save config."""
        try:
            global CFG, moving, paused, timeline, seg_i, t_local, last_logged_seg
            global robot_pos, robot_heading, last_path_sig, last_snapshot, total_estimate_s
            global path_lookahead_enabled, path_lookahead_px, last_lookahead_radius
            
            # Update CFG dictionary
            CFG["field_centric"] = 1
            CFG["distance_units"] = dist_map[dist_var.get()]
            CFG["angle_units"] = ang_map[ang_var.get()]
            
            # Handle auto heading to node 1
            use_auto = bool(auto_heading_node1_var.get())
            if use_auto and len(display_nodes) >= 2:
                target_internal = _heading_for_node0_to1()
                if target_internal is None:
                    target_internal = heading_from_points(effective_node_pos(0), effective_node_pos(1))
                disp_deg = convert_heading_input(target_internal, None)
                init_head.set(f"{disp_deg:.3f}")
                entered = disp_deg
            else:
                entered = float(init_head.get())
            
            initial_state["heading"] = interpret_input_angle(entered)
            robot_heading = initial_state["heading"]
            CFG["initial_heading_deg"] = float(init_head.get())
            
            # Update physics
            CFG["robot_physics"]["rpm"] = float(rpm_var.get())
            CFG["robot_physics"]["diameter"] = float(diam_var.get())
            CFG["robot_physics"]["volts_straight"] = 12.0
            CFG["robot_physics"]["volts_turn"] = 12.0
            CFG["robot_physics"]["weight"] = float(w_var.get())
            CFG["robot_physics"]["t_buffer"] = float(tb_var.get())
            CFG["robot_physics"]["advanced_motion"] = int(adv_motion_var.get())
            CFG["robot_physics"]["all_omni"] = int(omni_var.get())
            try:
                tw_val = int(tracking_wheels_var.get())
            except Exception:
                tw_val = 0
            CFG["robot_physics"]["tracking_wheels"] = max(0, min(2, tw_val))
            CFG["robot_physics"]["point_density_per_in"] = float(dens_var.get() or rp.get("point_density_per_in", 4.0))
            CFG["robot_physics"]["curvature_gain"] = float(curv_var.get() or rp.get("curvature_gain", 0.05))
            CFG["gear_ratio"] = float(gr_var.get())
            
            # Update UI
            CFG.setdefault("ui", {})["show_hitboxes"] = int(show_hitboxes_var.get())
            CFG.setdefault("ui", {})["show_field_objects"] = int(show_field_objects_var.get())
            CFG.setdefault("ui", {})["show_node_numbers"] = int(show_node_numbers_var.get())
            CFG["reshape_label"] = reshape_label_var.get().strip() or "Reshape"
            
            # Update dimensions
            bd = CFG["bot_dimensions"]
            bd["dt_width"] = float(dt_w.get())
            bd["dt_length"] = float(dt_l.get())
            bd["width"] = float(w_full.get())
            bd["length"] = float(l_full.get())
            bd["full_offset_x_in"] = float(full_off_x.get())
            bd["full_offset_y_in"] = float(full_off_y.get())
            bd["reshape_width"] = float(rs_w.get())
            bd["reshape_length"] = float(rs_l.get())
            bd["reshape_offset_x_in"] = float(rs_off_x.get())
            bd["reshape_offset_y_in"] = float(rs_off_y.get())
            
            # Update offsets
            CFG["offsets"]["offset_1_in"] = float(off1.get())
            CFG["offsets"]["offset_2_in"] = float(off2.get())
            CFG["offsets"]["padding_in"] = float(pad.get())
            
            # Path config (preserve defaults if absent)
            pc = CFG.setdefault("path_config", {})
            pc["lookahead_in"] = auto_lookahead_in(CFG)
            vmin_cfg, vmax_cfg = _normalize_cmd_range(
                pc.get("min_speed_cmd", pc.get("min_speed_ips", 0.0)),
                pc.get("max_speed_cmd", pc.get("max_speed_ips", 127.0))
            )
            pc["min_speed_cmd"] = vmin_cfg
            pc["max_speed_cmd"] = vmax_cfg
            pc.pop("min_speed_ips", None)
            pc.pop("max_speed_ips", None)
            pc["simulate_pursuit"] = int(path_lookahead_enabled)
            
            # Refresh lookahead globals
            path_lookahead_enabled = bool(pc.get("simulate_pursuit", 1))
            path_lookahead_px = max(0.0, float(pc.get("lookahead_in", auto_lookahead_in(CFG))) * PPI)
            last_lookahead_radius = path_lookahead_px
            
            # Update codegen
            CFG.setdefault("codegen", {})["style"] = codegen_style_var.get()
            opts = CFG["codegen"].setdefault("opts", {})
            opts.update({
                "ticks_per_rotation": float(ticks_var.get() or 360),
                "pad_factor": float(pad_var.get() or 1.0),
                "min_timeout_s": float(min_s_var.get() or 0.0),
                "reshape_output": reshape_output_var.get() or "1/2"
            })
            cal = CFG["codegen"].setdefault("calibration", {})
            cal["enabled"] = int(cal_enabled_var.get())
            try:
                cal["err_scale"] = float(cal_err_scale_var.get())
            except Exception:
                cal["err_scale"] = 1.1
            try:
                cal["time_scale"] = float(cal_time_scale_var.get())
            except Exception:
                cal["time_scale"] = 1.1
            try:
                cal["k_noise"] = float(cal_noise_mult_var.get())
            except Exception:
                cal["k_noise"] = 4.0
            noise = cal.setdefault("noise", {})
            try:
                noise["drive_in"] = float(cal_noise_drive_var.get())
            except Exception:
                noise["drive_in"] = 0.0
            try:
                noise["turn_deg"] = float(cal_noise_turn_var.get())
            except Exception:
                noise["turn_deg"] = 0.0
            CFG["codegen"]["path_dir"] = path_dir_var.get().strip() or "export/paths"
            CFG["codegen"]["path_columns"] = path_columns_var.get().strip() or "{X}, {Y}, {COMMAND}"
            _save_tpl_vars_for(codegen_style_var.get())  # Save current templates
            _refresh_cal_summary()
            
            # Save to disk
            save_config(CFG)
            CFG.pop("_phys_cache", None)
            
            # Reset motion state
            moving = False; paused = False; show_chevron = False
            timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
            robot_pos = display_nodes[0]["pos"]; robot_heading = initial_state["heading"]
            last_path_sig = None
            last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
            total_estimate_s = compute_total_estimate_s()
        
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply settings:\n{e}")
    
    _bind_live_handlers()
    _rebuild_tpl_panel()
    
    top.protocol("WM_DELETE_WINDOW", lambda: None)
    top.lift()
    top.focus_force()

def main():
    """Main application loop."""
    global constrain_active, constrain_origin, selected_control_point, path_control_points, path_edit_mode, dragging_control_point
    global moving, paused, robot_pos, robot_heading, reshape_live, show_chevron
    global selected_idx, dragging, last_snapshot, last_path_sig, history_freeze
    global timeline, seg_i, t_local, last_logged_seg, CFG, total_estimate_s
    global path_lookahead_enabled, path_lookahead_px, last_lookahead_point, last_lookahead_radius, last_heading_target
    global offset_dragging_idx, offset_drag_prev, edge_marker_drag
    
    open_settings_window()
    
    running = True
    while running:
        clock.tick(60)
        ui.pump_tk()
        
        # Detect path changes
        sig = tuple(
            (int(n["pos"][0]), int(n["pos"][1]),
             int(n.get("offset", 0)),
             (None if n.get("offset_custom_in") is None else round(float(n["offset_custom_in"]), 3)),
             bool(n.get("reverse", False)),
             bool(n.get("reshape_toggle", False)),
             tuple(
                 (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),
                  round(float(a.get("s", 0.0)), 3)) if a.get("type") == "wait"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),
                       round(float(a.get("deg", 0.0)), 3), str(a.get("dir", "auto")).lower()) if a.get("type") == "swing"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),
                       round(float(a.get("deg", 0.0)), 3)) if a.get("type") == "turn"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),
                       ("toggle" if a.get("state", None) is None else bool(a.get("state")))) if a.get("type") == "reverse"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),)
                 for a in n.get("actions", [])
             ),
             (None if _node_lateral_cmd(n) is None else round(float(_node_lateral_cmd(n)), 3)),
             (None if n.get("custom_turn_dps") is None else round(float(n.get("custom_turn_dps", 0.0)), 3)),
             path_sig_for_node(n))
            for n in display_nodes
        )
        
        if last_snapshot is None:
            last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
            last_path_sig = sig
            total_estimate_s = compute_total_estimate_s()
        
        mouse_pos = pygame.mouse.get_pos()
        hover_idx, best = None, float("inf")
        for i, n in enumerate(display_nodes):
            dx, dy = n["pos"][0] - mouse_pos[0], n["pos"][1] - mouse_pos[1]
            d2 = dx*dx + dy*dy
            if d2 < best and d2 <= SELECTION_RADIUS_PX**2:
                best, hover_idx = d2, i
        last_lookahead_point = None
        
        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False
            
            if event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()
                
                if event.key == pygame.K_o:
                    _refresh_output_header()
                    ui.toggle_output_window(log_lines)
                
                # === PATH EDIT MODE KEYBINDS ===
                if event.key == pygame.K_p and not (mods & pygame.KMOD_CTRL):
                    # P: Toggle path edit mode for the hovered/selected node
                    if path_edit_mode:
                        exit_path_edit_mode()
                    else:
                        target_idx = selected_idx if selected_idx is not None else hover_idx
                        if target_idx is not None and target_idx < len(display_nodes) - 1:
                            enter_path_edit_mode(target_idx)
                            selected_idx = target_idx

                elif event.key == pygame.K_ESCAPE and path_edit_mode:
                    # ESC: Exit path edit mode
                    exit_path_edit_mode()
                
                elif event.key == pygame.K_a and path_edit_mode and not (mods & pygame.KMOD_CTRL):
                    # A: Add control point at mouse
                    add_control_point_at_mouse(mouse_pos)
                
                elif event.key == pygame.K_x and path_edit_mode and selected_control_point is not None:
                    # X: Delete selected control point
                    remove_control_point(selected_control_point)
                
                elif path_edit_mode and event.key == pygame.K_m:
                    # Mirroring disabled
                    pass
                
                elif event.key == pygame.K_m and not path_edit_mode:
                    if not moving:
                        hit = _edge_marker_hit(mouse_pos)
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        changed = False
                        if hit is not None:
                            seg_idx, ev_idx, _pos, _d2 = hit
                            changed = _edit_edge_marker(seg_idx, event_idx=ev_idx)
                        else:
                            preview = _edge_marker_preview(mouse_pos)
                            if preview is not None:
                                seg_idx, t_val, _pos = preview
                                changed = _edit_edge_marker(seg_idx, t=t_val)
                        if changed:
                            util_push_undo_prev(undo_stack, prev)
                            last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                            last_path_sig = None
                            total_estimate_s = compute_total_estimate_s()
                
                elif event.key == pygame.K_z and (mods & pygame.KMOD_CTRL):
                    if undo_stack:
                        history_freeze = True
                        redo_stack.append(util_snapshot(display_nodes, robot_pos, robot_heading))
                        prev_nodes, prev_pos, prev_head = undo_stack.pop()
                        display_nodes[:] = prev_nodes
                        robot_pos, robot_heading = prev_pos, prev_head
                        moving = False; paused = False; show_chevron = False
                        timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                        last_path_sig = None
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        total_estimate_s = compute_total_estimate_s()
                        history_freeze = False
                
                elif event.key == pygame.K_y and (mods & pygame.KMOD_CTRL):
                    if redo_stack:
                        history_freeze = True
                        util_push_undo_prev(undo_stack, util_snapshot(display_nodes, robot_pos, robot_heading))
                        next_nodes, next_pos, next_head = redo_stack.pop()
                        display_nodes[:] = next_nodes
                        robot_pos, robot_heading = next_pos, next_head
                        moving = False; paused = False; show_chevron = False
                        timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                        last_path_sig = None
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        total_estimate_s = compute_total_estimate_s()
                        history_freeze = False
                
                elif event.key == pygame.K_q:
                    global snap_enabled
                    snap_enabled = not snap_enabled
                
                elif event.key == pygame.K_t and not (mods & pygame.KMOD_CTRL):
                    path_lookahead_enabled = not path_lookahead_enabled
                    CFG.setdefault("path_config", {})["simulate_pursuit"] = int(path_lookahead_enabled)
                    if not path_lookahead_enabled:
                        last_lookahead_point = None
                        last_lookahead_radius = 0.0
                        last_heading_target = None
                    else:
                        last_lookahead_radius = path_lookahead_px
            
                elif event.key == pygame.K_c:
                    correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                    sync_all_path_endpoints()
                    log_lines.clear()
                    log_lines.extend(build_compile_header(CFG, initial_state["heading"]))
                    tl = build_timeline_with_buffers()
                    total_estimate_s = compute_total_from_timeline(tl)
                    style_norm = str(CFG.get("codegen", {}).get("style", "Action List")).strip().lower()
                    
                    if style_norm not in ("action list", "actionlist", "list"):
                        code_lines = build_export_lines(CFG, tl, routine_name="autonomous", initial_heading=initial_state["heading"]) or []
                        compile_log(code_lines)
                    else:
                        last_turn = None
                        tbuf = float(CFG.get("robot_physics", {}).get("t_buffer", 0.0) or 0.0)
                        for idx, seg in enumerate(tl):
                            T = float(seg.get("T", 0.0))
                            if T <= 0.0: 
                                continue
                            st = seg.get("type")
                            if st == "move":
                                log_action("move", i0=seg.get("i0", ...), i1=seg.get("i1", ...), 
                                          p0=seg["p0"], p1=seg["p1"], reverse=seg.get("reverse", False))
                            elif st == "path":
                                pts = seg.get("path_points", []) or []
                                p0 = seg.get("p0", pts[0] if pts else (0, 0))
                                p1 = seg.get("p1", pts[-1] if pts else p0)
                                log_action("path", i0=seg.get("i0", ...), i1=seg.get("i1", ...),
                                           p0=p0, p1=p1, path_points=pts)
                            elif st == "turn":
                                _key = (round(seg["start_heading"], 6), round(seg["target_heading"], 6), seg.get("role"))
                                if _key != last_turn:
                                    log_action("turn", h0=seg["start_heading"], h1=seg["target_heading"])
                                    last_turn = _key
                            elif st == "swing":
                                log_action("swing", h0=seg.get("start_heading", 0.0), h1=seg.get("target_heading", 0.0), dir=seg.get("swing_dir", "AUTO"))
                            elif st == "wait":
                                if not (idx == len(tl)-1 and abs(T - tbuf) <= 1e-6 and seg.get("role") == "buffer"):
                                    log_action("wait", s=T)
                            elif st == "reverse":
                                log_action("reverse", state=seg.get("state", 0))
                            elif st == "reshape":
                                log_action("reshape", state=seg["state"])
                        total_t = sum(float(sg.get("T", 0.0)) for sg in tl)
                        if tl and tl[-1].get("type") == "wait" and tl[-1].get("role") == "buffer":
                            total_t = max(0.0, total_t - float(tbuf))
                        log_lines.append(f"\nEstimated total time: {total_t:.2f} s")
                        compile_log(log_lines)
                
                elif event.key == pygame.K_SPACE and not (mods & pygame.KMOD_CTRL):
                    if moving:
                        paused = not paused
                    else:
                        if len(display_nodes) >= 2:
                            correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                            sync_all_path_endpoints()
                            log_lines.clear()
                            log_lines.extend(build_compile_header(CFG, initial_state["heading"]))
                            moving = True; paused = False; show_chevron = True
                            robot_pos = display_nodes[0]["pos"]
                            robot_heading = initial_state["heading"]
                            reshape_live = False
                            timeline = build_timeline_with_buffers()
                            total_estimate_s = compute_total_from_timeline(timeline)
                            seg_i = 0; t_local = 0.0; last_logged_seg = -1
                
                elif event.key == pygame.K_SPACE and (mods & pygame.KMOD_CTRL):
                    moving = False; paused = False; show_chevron = False
                    robot_pos = display_nodes[0]["pos"]
                    robot_heading = initial_state["heading"]
                    reshape_live = False
                    timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                
                elif event.key == pygame.K_f:
                    tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                    if tgt is not None and tgt >= 0 and tgt != 0:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        off = display_nodes[tgt].get("offset", 0)
                        new_off = {0: 1, 1: 2, 2: 0}[off]
                        display_nodes[tgt]["offset"] = new_off
                        if new_off != 0:
                            display_nodes[tgt].pop("offset_custom_in", None)
                            if display_nodes[tgt].get("offset_ghost_angle") is None and tgt > 0:
                                pd_prev = display_nodes[tgt - 1].get("path_to_next", {})
                                if pd_prev.get("use_path", False):
                                    display_nodes[tgt]["offset_ghost_angle"] = heading_from_points(display_nodes[tgt - 1]["pos"], display_nodes[tgt]["pos"])
                                else:
                                    display_nodes[tgt].pop("offset_ghost_angle", None)
                        else:
                            display_nodes[tgt].pop("offset_ghost_angle", None)
                        util_push_undo_prev(undo_stack, prev)
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        last_path_sig = None
                        total_estimate_s = compute_total_estimate_s()
                
                elif event.key == pygame.K_r:
                    tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                    if tgt is not None and tgt >= 0:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        display_nodes[tgt]["reverse"] = not display_nodes[tgt].get("reverse", False)
                        util_push_undo_prev(undo_stack, prev)
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        last_path_sig = None
                        total_estimate_s = compute_total_estimate_s()
                
                elif event.key == pygame.K_g:
                    tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                    if tgt is not None and tgt >= 0:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        display_nodes[tgt]["reshape_toggle"] = not display_nodes[tgt].get("reshape_toggle", False)
                        util_push_undo_prev(undo_stack, prev)
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        last_path_sig = None
                        total_estimate_s = compute_total_estimate_s()
                
                elif event.key == pygame.K_s:
                    save_nodes(initial_state, display_nodes)
                
                elif event.key == pygame.K_l:
                    ini, nds = load_nodes()
                    if ini and nds:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        initial_state.update(ini)
                        normalize_speed_units(nds)
                        for n in nds:
                            if "geom_toggle" in n and "reshape_toggle" not in n:
                                n["reshape_toggle"] = n.pop("geom_toggle")
                            for a in n.get("actions", []):
                                if a.get("type") == "geom": 
                                    a["type"] = "reshape"
                        display_nodes[:] = nds
                        robot_pos = display_nodes[0]["pos"]
                        robot_heading = initial_state["heading"]
                        sync_all_path_endpoints()
                        undo_stack.clear(); redo_stack.clear()
                        timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                        show_chevron = False
                        last_path_sig = None
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        total_estimate_s = compute_total_estimate_s()
                
                elif (event.key in (pygame.K_DELETE, pygame.K_BACKSPACE)) and (mods & pygame.KMOD_CTRL) and (mods & pygame.KMOD_SHIFT):
                    if path_edit_mode:
                        exit_path_edit_mode()
                    prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                    display_nodes[:] = [{"pos": initial_state["position"]}]
                    util_push_undo_prev(undo_stack, prev)
                    moving = False; paused = False; show_chevron = False
                    timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                    robot_pos = display_nodes[0]["pos"]
                    robot_heading = initial_state["heading"]
                    last_path_sig = None
                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                    total_estimate_s = compute_total_estimate_s()
                    selected_idx = None
                
                elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE):
                    if not path_edit_mode:
                        marker_target = None
                        if edge_marker_drag is not None:
                            marker_target = (edge_marker_drag.get("seg_idx"), edge_marker_drag.get("event_idx"))
                        else:
                            hit = _edge_marker_hit(mouse_pos)
                            if hit is not None:
                                marker_target = (hit[0], hit[1])
                        if marker_target is not None:
                            seg_idx, ev_idx = marker_target
                            if seg_idx is not None and ev_idx is not None:
                                events = display_nodes[seg_idx].get("edge_events", [])
                                if isinstance(events, list) and 0 <= ev_idx < len(events):
                                    prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                                    events.pop(ev_idx)
                                    if not events:
                                        display_nodes[seg_idx].pop("edge_events", None)
                                    util_push_undo_prev(undo_stack, prev)
                                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                    last_path_sig = None
                                    total_estimate_s = compute_total_estimate_s()
                                    edge_marker_drag = None
                                    continue
                    if selected_idx is not None:
                        delete_node_at(selected_idx)
                        selected_idx = None
            
            # Mouse events

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    if path_edit_mode:
                        # Path edit mode: select/drag control points
                        hover_cp = None
                        for i, cp in enumerate(path_control_points):
                            if math.hypot(cp[0] - mouse_pos[0], cp[1] - mouse_pos[1]) <= PATH_CONTROL_RADIUS:
                                hover_cp = i
                                break
                        
                        if hover_cp is not None:
                            selected_control_point = hover_cp
                            dragging_control_point = True
                            continue
                        # Check offset ghost in path mode
                        ghost_hit = None
                        for idx, n in enumerate(display_nodes):
                            if idx == 0: 
                                continue
                            if n.get("offset_ghost_angle") is None:
                                continue
                            prev_pd = display_nodes[idx - 1].get("path_to_next", {}) if idx - 1 >= 0 else {}
                            if not prev_pd.get("use_path", False):
                                continue
                            off_in = get_node_offset_in(n, CFG, idx)
                            if off_in <= 0.0:
                                continue
                            gpos = effective_node_pos(idx)
                            dx, dy = gpos[0] - mouse_pos[0], gpos[1] - mouse_pos[1]
                            hit_r2 = (SELECTION_RADIUS_PX * 3.0) ** 2
                            if dx*dx + dy*dy <= hit_r2:
                                ghost_hit = idx
                                break
                        if ghost_hit is not None:
                            offset_dragging_idx = ghost_hit
                            offset_drag_prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                            last_snapshot = offset_drag_prev
                            continue
                        selected_control_point = None
                    else:
                        # Normal mode
                        pos = pygame.mouse.get_pos()
                        marker_hit = _edge_marker_hit(pos)
                        if marker_hit is not None:
                            seg_idx, ev_idx, _p, _d2 = marker_hit
                            edge_marker_drag = {
                                "seg_idx": seg_idx,
                                "event_idx": ev_idx,
                                "prev": util_snapshot(display_nodes, robot_pos, robot_heading)
                            }
                            last_snapshot = edge_marker_drag["prev"]
                            dragging = False
                            selected_idx = None
                            continue
                        keys = pygame.key.get_pressed()
                        mods = pygame.key.get_mods()
                        if mods & pygame.KMOD_SHIFT:
                            cand = _segment_insert_preview(pos)
                            if cand is not None:
                                seg_idx, proj, t_split = cand
                                prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                                display_nodes.insert(seg_idx + 1, {"pos": (proj[0], proj[1])})
                                # Split edge markers between the two new segments
                                events = display_nodes[seg_idx].get("edge_events", [])
                                ev_left, ev_right = _split_edge_events(events, t_split)
                                if ev_left is not None:
                                    if ev_left:
                                        display_nodes[seg_idx]["edge_events"] = ev_left
                                    else:
                                        display_nodes[seg_idx].pop("edge_events", None)
                                    if ev_right:
                                        display_nodes[seg_idx + 1]["edge_events"] = ev_right
                                # Split curved paths into two segments
                                pd = display_nodes[seg_idx].get("path_to_next", {}) or {}
                                if pd.get("use_path", False) and not pd.get("pose_preview_points"):
                                    cps = list(pd.get("control_points") or [])
                                    if len(cps) >= 2:
                                        left_cps, right_cps = _split_path_control_points(cps, proj)
                                        pd_left = copy.deepcopy(pd)
                                        pd_right = copy.deepcopy(pd)
                                        pd_left["control_points"] = left_cps
                                        pd_right["control_points"] = right_cps
                                        for k in ("pose_preview_points", "swing_vis", "start_override", "path_points"):
                                            pd_left.pop(k, None)
                                            pd_right.pop(k, None)
                                        display_nodes[seg_idx]["path_to_next"] = pd_left
                                        display_nodes[seg_idx + 1]["path_to_next"] = pd_right
                                correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                                sync_all_path_endpoints()
                                util_push_undo_prev(undo_stack, prev)
                                last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                last_path_sig = None
                                total_estimate_s = compute_total_estimate_s()
                                selected_idx = seg_idx + 1
                                dragging = False
                                continue
                        if keys[pygame.K_DELETE] or keys[pygame.K_BACKSPACE]:
                            pos = pygame.mouse.get_pos()
                            pick, bestp = None, float("inf")
                            for i, n in enumerate(display_nodes):
                                dx, dy = n["pos"][0] - pos[0], n["pos"][1] - pos[1]
                                d2 = dx*dx + dy*dy
                                if d2 < bestp and d2 <= SELECTION_RADIUS_PX**2:
                                    bestp, pick = d2, i
                            if pick is not None and pick != 0:
                                delete_node_at(pick)
                            selected_idx = None
                            dragging = False
                        else:
                            pos = pygame.mouse.get_pos()
                            pick, bestp = None, float("inf")
                            for i, n in enumerate(display_nodes):
                                dx, dy = n["pos"][0] - pos[0], n["pos"][1] - pos[1]
                                d2 = dx*dx + dy*dy
                                if d2 < bestp and d2 <= SELECTION_RADIUS_PX**2:
                                    bestp, pick = d2, i
                            if pick is not None:
                                selected_idx = pick
                                dragging = True
                            else:
                                prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                                p_new = snap_to_grid(pos, GRID_SIZE_PX, snap_enabled)
                                display_nodes.append({"pos": p_new})
                                correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                                sync_all_path_endpoints()
                                util_push_undo_prev(undo_stack, prev)
                                last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                last_path_sig = None
                                total_estimate_s = compute_total_estimate_s()
                
                elif event.button == 3:  # Right click
                    if path_edit_mode:
                        # Add control point at mouse
                        add_control_point_at_mouse(mouse_pos)
                    else:
                        marker_hit = _edge_marker_hit(mouse_pos)
                        if marker_hit is not None:
                            seg_idx, ev_idx, _p, _d2 = marker_hit
                            prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                            changed = _edit_edge_marker(seg_idx, event_idx=ev_idx)
                            if changed:
                                util_push_undo_prev(undo_stack, prev)
                                last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                last_path_sig = None
                                total_estimate_s = compute_total_estimate_s()
                            continue
                        # First, see if right-clicked near the START of a curved path to set custom speeds
                        hit_seg = None
                        hit_dist = float("inf")
                        for i, node in enumerate(display_nodes[:-1]):
                            path_data = node.get("path_to_next", {})
                            if not (path_data.get("use_path", False) and path_data.get("control_points")):
                                continue
                            cps = list(path_data["control_points"])
                            if len(cps) < 2:
                                continue
                            start_pt = effective_node_pos(i)
                            cps[0] = start_pt
                            cps[-1] = effective_node_pos(i + 1)
                            if math.hypot(mouse_pos[0] - start_pt[0], mouse_pos[1] - start_pt[1]) <= SELECTION_RADIUS_PX:
                                hit_seg = i
                                break
                        if hit_seg is not None:
                            prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                            try:
                                pd_cur = display_nodes[hit_seg].get("path_to_next", {})
                                init_min = _path_speed_cmd(pd_cur, CFG, "min_speed_cmd", 0.0)
                                init_max = _path_speed_cmd(pd_cur, CFG, "max_speed_cmd", 127.0)
                                init_la = pd_cur.get("lookahead_in_override", CFG.get("path_config", {}).get("lookahead_in", auto_lookahead_in(CFG)))
                                inp = _askstring_centered(
                                    "Path speeds / lookahead",
                                    "Enter min_cmd,max_cmd[,lookahead in]:",
                                    initialvalue=f"{init_min},{init_max},{init_la}"
                                )
                                if inp:
                                    parts = [p.strip() for p in inp.replace(";", ",").split(",")]
                                    if len(parts) >= 2:
                                        vmin, vmax = _normalize_cmd_range(parts[0], parts[1])
                                        pd = display_nodes[hit_seg].setdefault("path_to_next", {})
                                        pd["min_speed_cmd"] = vmin
                                        pd["max_speed_cmd"] = vmax
                                        pd.pop("min_speed_ips", None)
                                        pd.pop("max_speed_ips", None)
                                        if len(parts) >= 3 and parts[2] != "":
                                            try:
                                                la_override = float(parts[2])
                                                pd["lookahead_in_override"] = la_override
                                            except Exception:
                                                pass
                                        elif pd.get("lookahead_in_override") is not None:
                                            pd.pop("lookahead_in_override", None)
                                        total_estimate_s = compute_total_estimate_s()
                                        util_push_undo_prev(undo_stack, prev)
                                # After speed prompt, also show standard node command prompt for the start node
                                start_node = display_nodes[hit_seg]
                                node_pos_locked = tuple(start_node.get("pos", (0, 0)))
                                init_cmd = compile_cmd_string(start_node, hit_seg)
                                cmd = _askstring_centered(
                                    "Node actions",
                                    "Commands (comma/semicolon separated). Examples:\n"
                                    "  turn 45, wait 2, turn 90\n"
                                    "  wait 1.5; turn -30\n"
                                    "  swing 90 cw  (swing to heading; optional cw/ccw/auto)\n"
                                    "  settleswing 90 cw  (swing and allow settling)\n"
                                    "  offset 7   (custom offset)\n"
                                    "  reshape    (toggle geometry)\n"
                                    "  reverse    (toggle / reverse on/off)\n"
                                    "  swingto 180   (force swing heading)\n"
                                    "  latspeed 50   (drive cmd override 0-127)\n"
                                    "  turnspeed 180 (deg/s override)\n"
                                    "  chain [0-1/off] (chain through this node; optional looseness)\n",
                                    initialvalue=init_cmd
                                )
                                changed = False
                                if cmd is not None:
                                    parse_and_apply_cmds(start_node, cmd, hit_seg)
                                    changed = True
                                if prompt_heading_realization(start_node, hit_seg):
                                    changed = True
                                if changed:
                                    start_node["pos"] = node_pos_locked
                                    display_nodes[hit_seg]["pos"] = node_pos_locked
                                    correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                                    sync_all_path_endpoints()
                                    util_push_undo_prev(undo_stack, prev)
                                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                    last_path_sig = None
                                    total_estimate_s = compute_total_estimate_s()
                            except Exception:
                                pass
                            continue
                        dragging = False
                        pos = pygame.mouse.get_pos()
                        pick, bestp = None, float("inf")
                        for i, n in enumerate(display_nodes):
                            dx, dy = n["pos"][0] - pos[0], n["pos"][1] - pos[1]
                            d2 = dx*dx + dy*dy
                            if d2 < bestp and d2 <= SELECTION_RADIUS_PX**2:
                                bestp, pick = d2, i
                        if pick is not None:
                            node = display_nodes[pick]
                            node_pos_locked = tuple(node.get("pos", (0, 0)))
                            init = compile_cmd_string(node, pick)
                            prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                            cmd = _askstring_centered(
                                "Node actions",
                                "Commands (comma/semicolon separated). Examples:\n"
                                "  turn 45, wait 2, turn 90\n"
                                "  wait 1.5; turn -30\n"
                                "  swing 90 cw  (swing to heading; optional cw/ccw/auto)\n"
                                "  settleswing 90 cw  (swing and allow settling)\n"
                                "  offset 7   (custom offset)\n"
                                "  reshape    (toggle geometry)\n"
                                "  reverse    (toggle / reverse on/off)\n"
                                "  swingto 180   (force swing heading)\n"
                                "  latspeed 50   (drive cmd override 0-127)\n"
                                "  turnspeed 180 (deg/s override)\n"
                                "  chain [0-1/off] (chain through this node; optional looseness)\n"
                                "Angles use 0=left, 90=up, 180=right.",
                                initialvalue=init
                            )
                            changed = False
                            if cmd is not None:
                                parse_and_apply_cmds(node, cmd, pick)
                                changed = True
                            if prompt_heading_realization(node, pick):
                                changed = True
                            if changed:
                                # Keep node anchored while editing commands
                                node["pos"] = node_pos_locked
                                display_nodes[pick]["pos"] = node_pos_locked
                                correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                                sync_all_path_endpoints()
                                util_push_undo_prev(undo_stack, prev)
                                last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                last_path_sig = None
                                total_estimate_s = compute_total_estimate_s()
                        else:
                            preview = _edge_marker_preview(mouse_pos)
                            if preview is not None:
                                seg_idx, t_val, _proj = preview
                                prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                                changed = _edit_edge_marker(seg_idx, t=t_val)
                                if changed:
                                    util_push_undo_prev(undo_stack, prev)
                                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                    last_path_sig = None
                                    total_estimate_s = compute_total_estimate_s()
                                continue

            if event.type == pygame.MOUSEMOTION:
                if edge_marker_drag is not None and not moving and not path_edit_mode:
                    seg_idx = edge_marker_drag.get("seg_idx", -1)
                    ev_idx = edge_marker_drag.get("event_idx", -1)
                    if 0 <= seg_idx < len(display_nodes) - 1:
                        pts = _edge_polyline(seg_idx)
                        t_val, _proj, _d2 = _polyline_nearest(event.pos, pts)
                        events = display_nodes[seg_idx].setdefault("edge_events", [])
                        if 0 <= ev_idx < len(events):
                            events[ev_idx]["t"] = max(0.0, min(1.0, float(t_val)))
                            last_path_sig = None
                    continue
                if path_edit_mode and dragging_control_point and selected_control_point is not None:
                    # Drag control point (cannot drag endpoints)
                    if 0 < selected_control_point < len(path_control_points) - 1:
                        path_control_points[selected_control_point] = tuple(event.pos)
                
                elif path_edit_mode and offset_dragging_idx is not None:
                    idx = offset_dragging_idx
                    node = display_nodes[idx]
                    snapped = snap_to_grid(event.pos, GRID_SIZE_PX, snap_enabled)
                    ang = heading_from_points(node["pos"], snapped)
                    node["offset_ghost_angle"] = ang
                    sync_all_path_endpoints()
                
                elif dragging and selected_idx is not None and not moving and not path_edit_mode:
                    mods_now = pygame.key.get_mods()
                    ctrl_now = bool(mods_now & pygame.KMOD_CTRL)
                    if ctrl_now and not constrain_active:
                        constrain_active = True
                        constrain_origin = tuple(display_nodes[selected_idx]["pos"])
                    elif not ctrl_now and constrain_active:
                        constrain_active = False
                        constrain_origin = None
                    
                    raw_p = snap_to_grid(event.pos, GRID_SIZE_PX, snap_enabled)
                    if constrain_active and constrain_origin is not None:
                        constrained = constrain_to_8dirs(constrain_origin, raw_p)
                        p = snap_to_grid(constrained, GRID_SIZE_PX, snap_enabled)
                    else:
                        p = raw_p
                    display_nodes[selected_idx]["pos"] = p
                    sync_path_endpoints_for_node(selected_idx)
            
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if edge_marker_drag is not None:
                    seg_idx = edge_marker_drag.get("seg_idx", -1)
                    if 0 <= seg_idx < len(display_nodes) - 1:
                        events = display_nodes[seg_idx].get("edge_events", [])
                        if isinstance(events, list):
                            try:
                                events.sort(key=lambda e: float(e.get("t", 0.0)))
                            except Exception:
                                pass
                    prev = edge_marker_drag.get("prev")
                    edge_marker_drag = None
                    if prev is not None:
                        util_push_undo_prev(undo_stack, prev)
                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                    last_path_sig = None
                    total_estimate_s = compute_total_estimate_s()
                    continue
                if path_edit_mode:
                    dragging_control_point = False
                elif offset_dragging_idx is not None:
                    offset_dragging_idx = None
                    if offset_drag_prev is not None:
                        util_push_undo_prev(undo_stack, offset_drag_prev)
                    offset_drag_prev = None
                    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                    last_path_sig = None
                    total_estimate_s = compute_total_estimate_s()
                else:
                    constrain_active = False
                    constrain_origin = None
                    if dragging:
                        dragging = False
                        selected_idx = None
                        correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                        sync_all_path_endpoints()
                        util_push_undo_prev(undo_stack, last_snapshot)
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        last_path_sig = None
                        total_estimate_s = compute_total_estimate_s()
                    else:
                        selected_idx = None
            

        # Path change detection
        if not history_freeze and sig != last_path_sig:
            last_path_sig = sig
            moving = False; paused = False; show_chevron = False
            timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
            robot_pos = display_nodes[0]["pos"]
            robot_heading = initial_state["heading"]
            reshape_live = False
            offset_dragging_idx = None
            offset_drag_prev = None
            edge_marker_drag = None
            path_draw_cache.clear()
            if not moving:
                total_estimate_s = compute_total_estimate_s()
        
        # Animation
        if moving and not paused:
            if seg_i >= len(timeline):
                moving = False
            else:
                seg = timeline[seg_i]
                T = seg["T"]
                if seg_i != last_logged_seg and t_local <= 1e-9 and T >= 0:
                    if seg["type"] == "move":
                        log_action("move", i0=seg.get("i0", seg_i), i1=seg.get("i1", seg_i+1), 
                                  p0=seg["p0"], p1=seg["p1"], reverse=seg.get("reverse", False))
                    elif seg["type"] == "path":
                        log_action("path", i0=seg.get("i0", seg_i), i1=seg.get("i1", seg_i+1),
                                   p0=seg.get("p0", seg.get("path_points", [robot_pos])[0]),
                                   p1=seg.get("p1", seg.get("path_points", [robot_pos])[-1]),
                                   path_points=seg.get("path_points", []))
                    elif seg["type"] == "turn":
                        log_action("turn", h0=seg["start_heading"], h1=seg["target_heading"])
                    elif seg["type"] == "wait":
                        try:
                            _tbuf = float(CFG.get("robot_physics", {}).get("t_buffer", 0.0))
                        except Exception:
                            _tbuf = 0.0
                        if not (seg_i == len(timeline)-1 and abs(seg.get("T", 0.0) - _tbuf) <= 1e-6):
                            log_action("wait", s=seg["T"])
                    elif seg["type"] == "reshape":
                        log_action("reshape", state=seg["state"])
                    elif seg["type"] == "swing":
                        log_action("turn", h0=seg.get("start_heading"), h1=seg.get("target_heading"))
                    last_logged_seg = seg_i
                    if seg.get("edge_events"):
                        seg["_edge_event_idx"] = 0
                
                seg_type = seg.get("type")
                if T <= 0:
                    if seg_type in ("path", "path_follow"):
                        path_points = seg.get("path_points", [])
                        if path_points:
                            robot_pos = path_points[-1]
                            if seg.get("reverse"):
                                robot_heading = (robot_heading + 180.0) % 360.0
                        seg_i += 1
                        t_local = 0.0
                    elif seg_type == "swing":
                        robot_pos = seg.get("end_pos", robot_pos)
                        robot_heading = seg.get("target_heading", robot_heading)
                        seg_i += 1
                        t_local = 0.0
                    elif seg_type == "turn":
                        robot_pos, robot_heading = seg["pos"], seg["target_heading"]
                    elif seg_type == "wait":
                        robot_pos = seg.get("pos", robot_pos)
                        robot_heading = seg.get("heading", robot_heading)
                    elif seg_type == "move":
                        robot_pos, robot_heading = seg["p1"], seg.get("facing", robot_heading)
                    elif seg_type == "reshape":
                        reshape_live = not reshape_live
                    seg_i += 1
                    t_local = 0.0
                else:
                    t_local = min(T, t_local + (1.0/60.0))
                    if seg_type == "move":
                        L_in = seg.get("length_in")
                        if L_in is None:
                            L_in = math.hypot(seg["p1"][0]-seg["p0"][0], seg["p1"][1]-seg["p0"][1]) / PPI
                        drive_override = seg.get("drive_speed_ips")
                        s_in = sample_move_profile(
                            t_local,
                            L_in,
                            CFG,
                            v_override=drive_override,
                            v0=seg.get("v_start", 0.0),
                            v1=seg.get("v_end", 0.0)
                        )
                        frac = 0.0 if L_in <= 0 else max(0.0, min(1.0, s_in / max(1e-6, L_in)))
                        robot_pos = (seg["p0"][0] + frac*(seg["p1"][0]-seg["p0"][0]),
                                    seg["p0"][1] + frac*(seg["p1"][1]-seg["p0"][1]))
                        robot_heading = seg.get("facing", robot_heading)
                    
                    elif seg_type in ("path", "path_follow"):
                        path_points = seg.get("path_points", [])
                        use_pursuit = path_lookahead_enabled and not seg.get("move_to_pose")
                        la_px = seg.get("lookahead_px", path_lookahead_px)
                        path_speeds = seg.get("path_speeds")
                        path_meta = seg.get("path_meta")
                        if use_pursuit:
                            # Pure pursuit-style step toward lookahead
                            _, heading_to_look, look_pt = sample_path_position(
                                t_local, path_points, T, CFG,
                                lookahead_px=la_px,
                                use_pursuit=True,
                                path_speeds=path_speeds,
                                current_pos=robot_pos,
                                path_meta=path_meta
                            )
                            if look_pt:
                                # If we're close to the end of the path, bias heading to the final tangent
                                dist_end = math.hypot(path_points[-1][0] - robot_pos[0], path_points[-1][1] - robot_pos[1])
                                if dist_end <= max(la_px * 0.8, PPI * 1.5):
                                    heading_to_look = calculate_path_heading(path_points, len(path_points) - 1)
                                if seg.get("reverse"):
                                    heading_to_look = (heading_to_look + 180.0) % 360.0
                                heading_to_look = smooth_angle(last_heading_target if last_heading_target is not None else robot_heading, heading_to_look, 0.25)
                                dx = look_pt[0] - robot_pos[0]
                                dy = look_pt[1] - robot_pos[1]
                                dist = math.hypot(dx, dy)
                                speed_ips = seg.get("path_speeds", [None])[0] if path_speeds else None
                                try:
                                    idx_progress = int(max(0, min(len(path_points)-1, (t_local / max(1e-6, T)) * (len(path_points)-1))))
                                    if path_speeds:
                                        speed_ips = path_speeds[idx_progress]
                                except Exception:
                                    pass
                                if speed_ips is None:
                                    cfg_cmd = float(CFG.get("path_config", {}).get("max_speed_cmd", CFG.get("path_config", {}).get("max_speed_ips", 127.0)))
                                    cfg_cmd = max(0.0, min(127.0, cfg_cmd))
                                    speed_ips = vmax_straight(CFG) * (cfg_cmd / 127.0)
                                v_px = float(speed_ips) * PPI
                                step = min(dist, v_px * (1.0/60.0))
                                if dist > 1e-6:
                                    robot_pos = (robot_pos[0] + dx/dist * step, robot_pos[1] + dy/dist * step)
                                robot_heading = clamp_heading_rate(robot_heading, heading_to_look, CFG, dt=1.0/60.0)
                                last_heading_target = heading_to_look
                                # Smooth lookahead point for stability (heavier weight on previous)
                                if last_lookahead_point:
                                    look_pt = (
                                        0.8 * last_lookahead_point[0] + 0.2 * look_pt[0],
                                        0.8 * last_lookahead_point[1] + 0.2 * look_pt[1],
                                    )
                                last_lookahead_point = look_pt
                            else:
                                robot_pos, robot_heading, _ = sample_path_position(
                                    t_local, path_points, T, CFG,
                                    lookahead_px=la_px,
                                    use_pursuit=False,
                                    path_speeds=path_speeds,
                                    path_meta=path_meta
                                )
                                last_heading_target = None
                            last_lookahead_radius = la_px
                        else:
                            robot_pos, heading_sample, look_pt = sample_path_position(
                                t_local, path_points, T, CFG,
                                lookahead_px=la_px,
                                use_pursuit=False,
                                path_speeds=path_speeds,
                                path_meta=path_meta
                            )
                            if seg.get("reverse"):
                                heading_sample = (heading_sample + 180.0) % 360.0
                            robot_heading = clamp_heading_rate(robot_heading, heading_sample, CFG, dt=1.0/60.0)
                            last_lookahead_point = None
                            last_heading_target = None
                    
                    elif seg_type == "swing":
                        dg = seg.get("diff_geom", {})
                        center = dg.get("center")
                        r0 = dg.get("r0")
                        delta_deg = dg.get("delta_deg", 0.0)  # geometry delta (screen-space)
                        delta_heading = seg.get("delta_heading", seg.get("target_heading", robot_heading) - seg.get("start_heading", robot_heading))
                        frac = 0.0 if T <= 0 else max(0.0, min(1.0, t_local / max(1e-6, T)))
                        if center is not None and r0 is not None:
                            ang = math.radians(delta_deg) * frac
                            cosd, sind = math.cos(ang), math.sin(ang)
                            rx = r0[0] * cosd - r0[1] * sind
                            ry = r0[0] * sind + r0[1] * cosd
                            robot_pos = (center[0] + rx, center[1] + ry)
                        robot_heading = (seg.get("start_heading", robot_heading) + delta_heading * frac) % 360.0
                    
                    elif seg_type == "wait":
                        robot_pos = seg.get("pos", robot_pos)
                        robot_heading = seg.get("heading", robot_heading)
                    
                    elif seg_type == "turn":
                        robot_pos = seg["pos"]
                        robot_heading = sample_turn_heading_trap(t_local, seg["start_heading"], seg["target_heading"], CFG, rate_override=seg.get("turn_speed_dps"))
                    
                    # Fire edge markers during motion segments
                    if seg_type in ("move", "path", "path_follow") and seg.get("edge_events"):
                        if "_edge_event_idx" not in seg:
                            seg["_edge_event_idx"] = 0
                        try:
                            progress = 0.0 if T <= 0 else max(0.0, min(1.0, t_local / max(1e-6, T)))
                            events = seg.get("edge_events", [])
                            idx = int(seg.get("_edge_event_idx", 0))
                            while idx < len(events):
                                try:
                                    t_evt = float(events[idx].get("t", 0.0))
                                except Exception:
                                    t_evt = 0.0
                                if progress + 1e-6 < t_evt:
                                    break
                                actions = events[idx].get("actions", [])
                                label = _marker_actions_to_text(actions)
                                reshape_live = _marker_apply_reshape(actions, reshape_live)
                                log_action("marker", label=label)
                                idx += 1
                            seg["_edge_event_idx"] = idx
                        except Exception:
                            pass

                    if t_local >= T - 1e-9:
                        if seg_type == "move":
                            robot_pos, robot_heading = seg["p1"], seg.get("facing", robot_heading)
                        elif seg_type == "path" or seg_type == "path_follow":
                            path_points = seg.get("path_points", [])
                            if path_points:
                                robot_pos = path_points[-1]
                                end_h = seg.get("facing")
                                if end_h is None and PATH_FEATURES_AVAILABLE:
                                    end_h = calculate_path_heading(path_points, len(path_points) - 1)
                                if end_h is not None:
                                    robot_heading = end_h
                                if seg.get("reverse"):
                                    robot_heading = (robot_heading + 180.0) % 360.0
                                last_heading_target = robot_heading
                        elif seg_type == "wait":
                            robot_pos = seg.get("pos", robot_pos)
                            robot_heading = seg.get("heading", robot_heading)
                        elif seg_type == "turn":
                            robot_pos, robot_heading = seg["pos"], seg["target_heading"]
                        elif seg_type == "swing":
                            robot_pos = seg.get("end_pos", robot_pos)
                            robot_heading = seg.get("target_heading", robot_heading)
                        elif seg_type == "reshape":
                            reshape_live = not reshape_live
                        seg_i += 1
                        t_local = 0.0
        
        # Update shift insert preview
        insert_preview = None
        if not path_edit_mode and not moving:
            mods_now = pygame.key.get_mods()
            if mods_now & pygame.KMOD_SHIFT:
                insert_preview = _segment_insert_preview(mouse_pos)

        #   Drawing
        screen.fill(BG_COLOR)
        draw_grid(screen, GRID_SIZE_PX)
        draw_field_objects(screen, CFG)
        draw_geometry_borders(screen, display_nodes, CFG, initial_state["heading"])
        draw_follow_geometry(screen, CFG, robot_pos, robot_heading, reshape_live)

        # Draw swing arcs and movetopose previews (straight segments only)
        for i in range(len(display_nodes) - 1):
            node = display_nodes[i]
            pd = node.get("path_to_next", {})
            swing_vis = pd.get("swing_vis")
            pose_preview = pd.get("pose_preview_points")
            next_eff = effective_node_pos(i + 1)
            start_override = pd.get("start_override", node.get("pos"))
            if swing_vis:
                arc_pts = swing_arc_points(swing_vis)
                start_pos = swing_vis.get("start_pos", start_override)
                end_pos = swing_vis.get("end_pos", start_pos)
                if arc_pts:
                    pygame.draw.lines(screen, NODE_COLOR, False, arc_pts, 3)
                pygame.draw.line(screen, NODE_COLOR, end_pos, next_eff, 2)
            if pose_preview and not pd.get("use_path", False):
                draw_curved_path(screen, pose_preview, color=(80, 220, 200), width=3)
        
        # Draw paths for curved segments
        for i, node in enumerate(display_nodes[:-1]):
            path_data = node.get("path_to_next", {})
            if not (path_data.get("use_path") and path_data.get("control_points")):
                continue
            if path_edit_mode and i == path_edit_segment_idx:
                # The actively edited path is drawn in the overlay below
                continue
            color = (100, 200, 255)
            width = 3
            if path_edit_mode:
                color = (120, 120, 120)
                width = 2
            # Cached shaded segments
            key = path_shade_key(i, node)
            segs = path_draw_cache.get(key)
            if segs is None:
                segs = build_shaded_segments(i)
                path_draw_cache[key] = segs
            if segs:
                for seg_entry in segs:
                    if not seg_entry:
                        continue
                    if isinstance(seg_entry, tuple) and seg_entry[0] == "plain":
                        pts = seg_entry[1]
                        draw_curved_path(screen, pts, color=color, width=width)
                    else:
                        p0, p1, col = seg_entry
                        pygame.draw.line(screen, col, p0, p1, width)
        
        # Draw normal node connections (straight segments only)
        for i in range(len(display_nodes) - 1):
            node = display_nodes[i]
            path_data = node.get("path_to_next", {})
            if path_data.get("pose_preview_points"):
                continue
            if path_data.get("swing_vis"):
                continue
            if not (path_data.get("use_path") and path_data.get("control_points")):
                # Draw straight line
                p0, p1 = effective_node_pos(i), effective_node_pos(i + 1)
                pygame.draw.line(screen, NODE_COLOR, p0, p1, 2)

        # Draw edge mechanism markers
        marker_hover = _edge_marker_hit(mouse_pos)
        marker_hover_idx = None
        if marker_hover is not None:
            marker_hover_idx = (marker_hover[0], marker_hover[1])
        for i in range(len(display_nodes) - 1):
            node = display_nodes[i]
            events = node.get("edge_events", [])
            if not isinstance(events, list) or not events:
                continue
            pts = _edge_polyline(i)
            for j, ev in enumerate(events):
                try:
                    t_val = float(ev.get("t", 0.0))
                except Exception:
                    t_val = 0.0
                p = _polyline_point_at(pts, t_val)
                enabled = bool(ev.get("enabled", True))
                color = NODE_COLOR if enabled else GREY
                # Cut a small hole in the segment line under the marker
                tan = _polyline_tangent(pts, t_val)
                hole_r = EDGE_MARKER_RADIUS + 2
                hx0 = (p[0] - tan[0] * hole_r, p[1] - tan[1] * hole_r)
                hx1 = (p[0] + tan[0] * hole_r, p[1] + tan[1] * hole_r)
                pygame.draw.line(screen, BG_COLOR, hx0, hx1, 4)
                radius = EDGE_MARKER_RADIUS + (2 if marker_hover_idx == (i, j) else 0)
                pygame.draw.circle(screen, color, (int(p[0]), int(p[1])), radius, 2)

        # Draw nodes with proper arguments
        draw_nodes(screen, display_nodes, selected_idx, font, CFG, path_edit_mode, draw_links=False)

        # Draw grey offset endpoint for any segment type
        for i in range(len(display_nodes) - 1):
            next_node = display_nodes[i + 1]
            off_in = get_node_offset_in(next_node, CFG, i + 1)
            if off_in == 0.0:
                continue
            end_pt = effective_node_pos(i + 1)
            if end_pt is None:
                continue
            pygame.draw.circle(screen, (140, 140, 140), (int(end_pt[0]), int(end_pt[1])), 6)

        # Shift insert preview (draw hole after lines so it stays visible)
        if insert_preview is not None:
            if len(insert_preview) >= 3:
                seg_idx, pos, t_ins = insert_preview
            else:
                seg_idx, pos = insert_preview
                t_ins = None
            if 0 <= seg_idx < len(display_nodes) - 1:
                pts = _edge_polyline(seg_idx)
                if t_ins is None:
                    try:
                        t_ins, _proj, _d2 = _polyline_nearest(pos, pts)
                    except Exception:
                        t_ins = 0.0
                tan = _polyline_tangent(pts, t_ins) if pts else (1.0, 0.0)
                hole_r = 9.0
                hx0 = (pos[0] - tan[0] * hole_r, pos[1] - tan[1] * hole_r)
                hx1 = (pos[0] + tan[0] * hole_r, pos[1] + tan[1] * hole_r)
                pygame.draw.line(screen, BG_COLOR, hx0, hx1, 4)
            pygame.draw.circle(screen, NODE_COLOR, (int(pos[0]), int(pos[1])), 7, 2)
        
        # PATH EDIT MODE OVERLAY
        if path_edit_mode and path_control_points:
            cp_preview = list(path_control_points)
            # Draw smooth path preview
            if PATH_FEATURES_AVAILABLE:
                smooth_path = generate_bezier_path(cp_preview, num_samples=50)
                draw_curved_path(screen, smooth_path, color=(255, 200, 100), width=4)
            
            # Draw control point handles
            draw_path_control_points(screen, path_control_points, selected_control_point, PATH_CONTROL_RADIUS)
            
            # Draw lines connecting control points
            if len(cp_preview) >= 2:
                pygame.draw.lines(screen, (150, 150, 150), False, cp_preview, 1)
            
            # Draw edit mode label
            draw_path_edit_overlay(screen, path_edit_segment_idx, display_nodes, font_small)
        
        if show_chevron:
            draw_chevron(screen, robot_pos, robot_heading)
        draw_robot(screen, robot_pos)
        
        # Lookahead visualization
        if path_lookahead_enabled and last_lookahead_point:
            try:
                rad = max(4, int(last_lookahead_radius))
                pygame.draw.circle(screen, (80, 160, 255), (int(robot_pos[0]), int(robot_pos[1])), rad, 1)
                pygame.draw.circle(screen, (255, 200, 120), (int(last_lookahead_point[0]), int(last_lookahead_point[1])), 5)
            except Exception:
                pass
        draw_constraint_visual(screen, dragging, selected_idx, display_nodes, constrain_origin)

        if marker_hover is not None:
            seg_idx, ev_idx, _, _ = marker_hover
            if 0 <= seg_idx < len(display_nodes):
                events = display_nodes[seg_idx].get("edge_events", [])
                if isinstance(events, list) and 0 <= ev_idx < len(events):
                    hover_text = _marker_hover_text(events[ev_idx].get("actions", []))
                    _draw_marker_hover(screen, hover_text, mouse_pos, font_small)
        
        # Draw hover box or help text
        if path_edit_mode:
            # Help text already drawn in path_edit_overlay
            pass
        elif hover_idx is not None and 0 <= hover_idx < len(display_nodes):
            draw_hover_box(screen, display_nodes[hover_idx], hover_idx, mouse_pos, CFG, initial_state["heading"], font_small)
        
        draw_time_label(screen, display_nodes, total_estimate_s, font_small)
        
        pygame.display.flip()
    
    pygame.quit()

if __name__ == "__main__":
    main()
