# terminal/main.py
import os, math, json
from typing import Any, Callable

# Ensure SDL picks a usable video driver (helps when run from terminals that default to headless)
if os.name == "nt" and not os.environ.get("SDL_VIDEODRIVER"):
    os.environ["SDL_VIDEODRIVER"] = "windows"

import pygame
import tkinter as tk
from tkinter import simpledialog, ttk, messagebox, filedialog

from mod.config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, GRID_SIZE_PX, BG_COLOR, PPI,
    load_config, save_config, physics_flat, dims_flat, offsets_flat
)
from mod.geom import convert_heading_input
from mod.sim import (
    snap_to_grid, compile_timeline, estimate_time,
    sample_turn_heading_trap, sample_move_profile, vmax_straight, accel_straight,
    correct_nodes_inbounds, sample_path_position, path_time_with_curvature, turn_rate
)
from mod.draw import (
    draw_grid, draw_robot, draw_chevron, draw_nodes,
    draw_hover_box, draw_time_label, draw_constraint_visual,
    draw_field_objects, draw_geometry_borders, draw_follow_geometry
)
from mod.storage import save_nodes, load_nodes, compile_log
from mod.util import (
    interpret_input_angle, heading_from_points, pros_convert_inches,
    build_compile_header, apply_tbuffer, coords_str,
    snapshot as util_snapshot, push_undo_prev as util_push_undo_prev,
    constrain_to_8dirs, get_node_offset_in, approach_unit
)
from mod.codegen import build_export_lines


# Try to import path utilities - these are NEW modules
try:
    from mod.path_utils import generate_bezier_path, calculate_path_heading, resample_path_uniform
    from mod.path_export import export_lemlib_path, generate_path_asset_name
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
    ("Shift+Drag Node", "Constrain drag to 8 global\ndirections from press point"),
    ("RightClick", "Node commands: turn X, wait Y,\n offset Z, reshape"),
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
NODE_COLOR = (100, 100, 255)
SELECTED_COLOR = (255, 100, 100)
PATH_COLOR = (100, 200, 255)
CONTROL_POINT_COLOR = (255, 150, 0)
SELECTED_CONTROL_COLOR = (255, 100, 100)
TEMPLATE_BG = "#f2f4f7"

# ---------------- Tk plumbing ----------------
tk_root = None
tk_settings_win = None
tk_output_win = None
tk_output_text = None
live_update_widgets = []

def ensure_tk_root():
    """Initialize Tkinter root window with styling."""
    global tk_root
    if tk_root is None or not (hasattr(tk_root, "winfo_exists") and tk_root.winfo_exists()):
        tk_root = tk.Tk()
        try:
            style = ttk.Style(tk_root)
            if "clam" in style.theme_names(): 
                style.theme_use("clam")
            style.configure("TFrame", padding=6)
            style.configure("TLabel", padding=2)
            style.configure("TEntry", padding=2)
            style.configure("TButton", padding=4)
            style.configure("TNotebook.Tab", padding=(12, 6))
            style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"))
            style.configure("Help.TLabel", foreground="#777777")
        except Exception: 
            pass
        tk_root.withdraw()

def pump_tk():
    """Update Tkinter event loop."""
    global tk_root, tk_settings_win, tk_output_win
    if tk_root is None: 
        return
    try:
        tk_root.update()
        if tk_settings_win is not None and not tk_settings_win.winfo_exists(): 
            tk_settings_win = None
        if tk_output_win is not None and not tk_output_win.winfo_exists(): 
            tk_output_win = None
    except tk.TclError:
        tk_root = None
        tk_settings_win = None
        tk_output_win = None

class _Tooltip:
    """Hover tooltip for widgets."""
    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)
    
    def _show(self, _=None):
        if self.tip or not self.text: 
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 8
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.geometry(f"+{x}+{y}")
        f = ttk.Frame(self.tip, padding=6, style="TFrame")
        f.pack()
        lbl = ttk.Label(f, text=self.text, justify="left", wraplength=360)
        lbl.pack()
    
    def _hide(self, _=None):
        if self.tip:
            try: 
                self.tip.destroy()
            except Exception: 
                pass
            self.tip = None

def add_tooltip(widget, text):
    """Add tooltip to widget."""
    if text:
        _Tooltip(widget, text)

def track_live_widget(widget):
    """Collect widgets that should trigger live updates."""
    if widget not in live_update_widgets:
        live_update_widgets.append(widget)

# ---------------- Helper drawing functions ----------------
def draw_curved_path(screen, points, color=(100, 200, 255), width=3):
    """Draw a smooth curved path through points."""
    if len(points) >= 2:
        pygame.draw.lines(screen, color, False, points, width)

def draw_path_control_points(screen, control_points, selected_idx):
    """Draw control point handles."""
    for i, cp in enumerate(control_points):
        # Skip endpoints (first and last)
        if i == 0 or i == len(control_points) - 1:
            continue
        
        color = SELECTED_CONTROL_COLOR if i == selected_idx else CONTROL_POINT_COLOR
        pygame.draw.circle(screen, color, (int(cp[0]), int(cp[1])), PATH_CONTROL_RADIUS)
        pygame.draw.circle(screen, (255, 255, 255), (int(cp[0]), int(cp[1])), PATH_CONTROL_RADIUS, 2)

def draw_path_edit_overlay(screen, segment_idx, nodes, font_obj):
    """Draw path edit mode overlay."""
    if segment_idx is not None and segment_idx < len(nodes) - 1:
        help_text = [
            "PATH EDIT MODE",
            f"Editing segment: Node {segment_idx} -> Node {segment_idx + 1}",
            "Left-click: Select control point",
            "Right-click or A: Add/insert control point",
            "X: Delete selected point",
            "P or ESC: Exit"
        ]
        # Compute placement and shift if mouse overlaps the area
        mouse = pygame.mouse.get_pos()
        pad = 10
        line_height = 20
        w = max(font_obj.size(line)[0] for line in help_text) + 10
        h = len(help_text) * line_height + 8
        x, y = pad, pad
        if x <= mouse[0] <= x + w and y <= mouse[1] <= y + h:
            x = max(pad, screen.get_width() - w - pad)
        y_offset = y
        for line in help_text:
            text_surf = font_obj.render(line, True, (255, 255, 255))
            bg_rect = text_surf.get_rect()
            bg_rect.topleft = (x, y_offset)
            bg_rect.inflate_ip(10, 4)
            pygame.draw.rect(screen, (50, 50, 50), bg_rect)
            pygame.draw.rect(screen, (200, 200, 200), bg_rect, 1)
            screen.blit(text_surf, (x, y_offset))
            y_offset += line_height

def _draw_label(screen, pos, lines):
    """Draw multi-line text label."""
    y_offset = pos[1]
    for line in lines:
        text_surf = font_small.render(line, True, (255, 255, 255))
        bg_rect = text_surf.get_rect()
        bg_rect.topleft = (pos[0], y_offset)
        bg_rect.inflate_ip(10, 4)
        pygame.draw.rect(screen, (50, 50, 50), bg_rect)
        pygame.draw.rect(screen, (200, 200, 200), bg_rect, 1)
        screen.blit(text_surf, (pos[0], y_offset))
        y_offset += 20

# ---------------- config helpers ----------------
def _coerce_int_range(v, default, valid_range):
    """Coerce value to integer within range."""
    if isinstance(v, dict): 
        v = v.get("value", default)
    try:
        iv = int(v)
        return iv if iv in valid_range else default
    except Exception:
        return default

def _num(x, d=0.0):
    """Extract numeric value from dict or return default."""
    if isinstance(x, dict): 
        x = x.get("value", d)
    try: 
        return float(x)
    except Exception: 
        return d

def reload_cfg():
    """Reload configuration from disk."""
    raw = load_config()
    dims = dims_flat(raw)
    bd_raw = raw.get("bot_dimensions", {})
    reshape_raw = bd_raw.get("reshape", {})
    dims["full_offset_x_in"] = _num(bd_raw.get("full_offset_x_in", {"value": 0.0}), 0.0)
    dims["full_offset_y_in"] = _num(bd_raw.get("full_offset_y_in", {"value": 0.0}), 0.0)
    dims["reshape_width"] = _num(reshape_raw.get("width", {"value": dims.get("width", 0.0)}), dims.get("width", 0.0))
    dims["reshape_length"] = _num(reshape_raw.get("length", {"value": dims.get("length", 0.0)}), dims.get("length", 0.0))
    dims["reshape_offset_x_in"] = _num(reshape_raw.get("offset_x_in", {"value": 0.0}), 0.0)
    dims["reshape_offset_y_in"] = _num(reshape_raw.get("offset_y_in", {"value": 0.0}), 0.0)
    
    ui_raw = raw.get("ui", {})
    ui = {k: v.get("value", v) if isinstance(v, dict) else v for k, v in ui_raw.items()}
    
    path_raw = raw.get("path_config", {})
    path_cfg = {
        "lookahead_in": _num(path_raw.get("lookahead_in", {"value": 15.0}), 15.0),
        "min_speed_ips": _num(path_raw.get("min_speed_ips", {"value": 30.0}), 30.0),
        "max_speed_ips": _num(path_raw.get("max_speed_ips", {"value": 127.0}), 127.0),
        "simulate_pursuit": _coerce_int_range(path_raw.get("simulate_pursuit", {"value": 1}), 1, (0, 1)),
    }
    
    codegen_raw = raw.get("codegen", {})
    style_raw = codegen_raw.get("style", {"value": "Action List"})
    style_val = style_raw.get("value", "Action List") if isinstance(style_raw, dict) else (style_raw or "Action List")
    path_dir_raw = codegen_raw.get("path_dir", {"value": "export/paths"})
    if isinstance(path_dir_raw, dict):
        path_dir_val = path_dir_raw.get("value", "export/paths")
    else:
        path_dir_val = path_dir_raw or "export/paths"
    codegen = {
        "style": style_val,
        "templates": codegen_raw.get("templates", {}),
        "opts": codegen_raw.get("opts", {}),
        "path_dir": path_dir_val,
    }
    
    return {
        "robot_physics": physics_flat(raw),
        "bot_dimensions": dims,
        "offsets": offsets_flat(raw),
        "ui": ui,
        "field_centric": 1,
        "distance_units": _coerce_int_range(raw.get("distance_units", 0), 0, (0, 1, 2, 3)),
        "gear_ratio": _num(raw.get("gear_ratio", {"value": 1.0}), 1.0),
        "angle_units": _coerce_int_range(raw.get("angle_units", 0), 0, (0, 1)),
        "initial_heading_deg": _num(raw.get("initial_heading_deg", {"value": 0.0}), 0.0),
        "codegen": codegen,
        "path_config": path_cfg,
    }

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
def auto_lookahead_in(cfg):
    """Auto-compute base lookahead (inches) from robot size."""
    bd = cfg.get("bot_dimensions", {})
    base = float(bd.get("dt_width", bd.get("width", 12.0)))
    # Slightly tighter, more pragmatic lookahead for pure pursuit/path following
    base = max(8.0, min(24.0, base * 0.9))
    return base

path_lookahead_enabled = bool(CFG.get("path_config", {}).get("simulate_pursuit", 1))
path_lookahead_px = float(CFG.get("path_config", {}).get("lookahead_in", auto_lookahead_in(CFG))) * PPI
last_lookahead_point = None
last_lookahead_radius = path_lookahead_px
last_heading_target = None
path_draw_cache = {}  # cache for gradient-drawn path segments

# Path editing state
path_edit_mode = False
path_edit_segment_idx = None  # Which segment (node index) is being edited
path_control_points = []  # Current control points being edited
selected_control_point = None  # Index of selected control point
dragging_control_point = False
path_edit_speed_range = [0.0, 0.0]  # kept for backward compat; not user-edited
tangent_cache = {}
path_mirror_start = "free"  # deprecated
path_mirror_end = "free"    # deprecated
PATH_CONTROL_RADIUS = 8  # Click detection radius for control points

def path_sig_for_node(node):
    """Small signature tuple capturing path state for change detection."""
    pd = node.get("path_to_next")
    if not pd:
        return (False, ())
    cps = pd.get("control_points") or []
    cps_sig = tuple((int(round(cp[0])), int(round(cp[1]))) for cp in cps)
    return (
        bool(pd.get("use_path", False)),
        cps_sig
    )

def path_shade_key(idx, node):
    """Build cache key for shaded path rendering."""
    pd = node.get("path_to_next", {})
    curv_gain = float(CFG.get("robot_physics", {}).get("curvature_gain", CFG.get("path_config", {}).get("curvature_gain", 0.05)))
    vmin = pd.get("min_speed_ips", CFG.get("path_config", {}).get("min_speed_ips", 30.0))
    vmax = pd.get("max_speed_ips", CFG.get("path_config", {}).get("max_speed_ips", 127.0))
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
        min_override = pd.get("min_speed_ips")
        max_override = pd.get("max_speed_ips")
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

def effective_node_pos(idx):
    """Return node position adjusted for offsets (same logic as motion)."""
    if idx <= 0:
        return display_nodes[0]["pos"]
    node = display_nodes[idx]
    prev = display_nodes[idx - 1]["pos"]
    off_in = get_node_offset_in(node, CFG, idx)
    ghost_ang = node.get("offset_ghost_angle")
    prev_pd = display_nodes[idx - 1].get("path_to_next", {}) if idx - 1 >= 0 else {}
    use_radial = prev_pd.get("use_path", False)
    if use_radial and off_in != 0.0:
        if ghost_ang is None:
            try:
                prev_eff = display_nodes[idx - 1]["pos"] if idx - 1 >= 0 else prev
            except Exception:
                prev_eff = prev
            # Default along path tangent (previous to this node)
            ghost_ang = heading_from_points(prev_eff, node["pos"])
            node["offset_ghost_angle"] = ghost_ang
        if ghost_ang is not None:
            rad = math.radians(ghost_ang)
            return (
                node["pos"][0] + math.cos(rad) * off_in * PPI,
                node["pos"][1] - math.sin(rad) * off_in * PPI
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
    tl = apply_tbuffer(CFG, compile_timeline(display_nodes, CFG, initial_state["heading"], fps))
    total = sum(seg.get("T", 0.0) for seg in tl)
    try:
        tbuf = float(CFG.get("robot_physics", {}).get("t_buffer", 0.0))
    except Exception:
        tbuf = 0.0
    if tl and tbuf > 0.0:
        lastT = float(tl[-1].get("T", 0.0))
        total -= min(tbuf, lastT)
        if total < 0.0: 
            total = 0.0
    return total


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

def log_action(kind, **kw):
    """Log action to output."""
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
    elif kind == "wait":
        log_lines.append(f"  Wait: {kw['s']:.3f} s")
    elif kind == "reshape":
        log_lines.append(f"  Reshape: state {kw['state']}")
    elif kind == "reverse":
        log_lines.append(f"  Reverse: {'ON' if kw['state'] else 'OFF'}")
    output_refresh()

def enter_path_edit_mode(segment_idx):
    """Enter path editing mode for a segment."""
    global path_edit_mode, path_edit_segment_idx, path_control_points, selected_control_point, path_edit_speed_range, path_mirror_start, path_mirror_end
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
                end_node["offset_ghost_angle"] = heading_from_points(p0, p1)
    else:
        cps = list(path_data["control_points"])
    
    ptn = node.get("path_to_next", {})
    path_edit_speed_range = [0.0, 0.0]

    path_control_points = list(cps)
    selected_control_point = None

def exit_path_edit_mode():
    """Exit path editing mode and save changes."""
    global path_edit_mode, path_edit_segment_idx, path_control_points, selected_control_point, path_edit_speed_range, path_mirror_start, path_mirror_end
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
    path_edit_speed_range = [0.0, 0.0]
    path_mirror_start = "free"
    path_mirror_end = "free"

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

def apply_mirror_mode(control_points, mode_start: str, mode_end: str, tang_in=None, tang_out=None):
    """Return a mirrored copy of control_points (does not mutate the original)."""
    if not control_points or len(control_points) < 3:
        return list(control_points)
    cp = list(control_points)
    p0, p1 = cp[0], cp[-1]
    h0 = list(cp[1])
    h1 = list(cp[-2])
    adhere_px = 2.0 * PPI  # force ~2in of tangent adherence
    forward_vec = (p1[0] - p0[0], p1[1] - p0[1])

    if mode_start == "mirror":
        v = tang_in
        if v is None:
            v = forward_vec
        mag = math.hypot(h0[0] - p0[0], h0[1] - p0[1])
        v_mag = math.hypot(v[0], v[1])
        target_mag = max(adhere_px, mag)
        if v_mag > 1e-6:
            ux, uy = v[0] / v_mag, v[1] / v_mag
            v = (ux * target_mag, uy * target_mag)
            h0 = [p0[0] + v[0], p0[1] + v[1]]
    if mode_end == "mirror":
        v = tang_out if tang_out is not None else forward_vec
        mag = math.hypot(h1[0] - p1[0], h1[1] - p1[1])
        v_mag = math.hypot(v[0], v[1])
        target_mag = max(adhere_px, mag)
        if v_mag > 1e-6:
            ux, uy = v[0] / v_mag, v[1] / v_mag
            v = (ux * target_mag, uy * target_mag)
            h1 = [p1[0] + v[0], p1[1] + v[1]]

    cp[1] = (float(h0[0]), float(h0[1]))
    cp[-2] = (float(h1[0]), float(h1[1]))
    return cp

# ---------------- node command parsing ----------------
def compile_cmd_string(node, idx):
    """Generate command string from node actions."""
    parts = []
    for act in node.get("actions", []):
        if act.get("type") == "turn":
            disp = convert_heading_input(act.get("deg", 0.0), None)
            parts.append(f"turn {disp:g}")
        elif act.get("type") == "wait":
            parts.append(f"wait {act.get('s', 0):g}")
        elif act.get("type") in ("reshape", "geom"):
            parts.append("reshape")
    if idx != 0 and int(node.get("offset", 0)) == 0 and node.get("offset_custom_in") is not None:
        parts.append(f"offset {node['offset_custom_in']:g}")
    if node.get("custom_lateral_ips"):
        parts.append(f"latspeed {node['custom_lateral_ips']:g}")
    if node.get("custom_turn_dps"):
        parts.append(f"turnspeed {node['custom_turn_dps']:g}")
    return ", ".join(parts)

def parse_and_apply_cmds(node, cmd_str, idx):
    """Parse and apply commands to node."""
    acts = []
    if cmd_str:
        for part in [p.strip() for p in cmd_str.replace(";", ",").split(",") if p.strip()]:
            low = part.lower()
            try:
                if low.startswith(("wait", "w", "sleep", "pause")):
                    x = float(low.split()[1].replace("sec", "").replace("s", ""))
                    if x > 0: 
                        acts.append({"type": "wait", "s": x})
                elif low.startswith(("turn", "t", "rotate")):
                    raw = float(low.split()[1].replace("deg", "").replace("°", ""))
                    x_internal = interpret_input_angle(raw)
                    acts.append({"type": "turn", "deg": x_internal})
                elif low.startswith("offset"):
                    if idx != 0 and int(node.get("offset", 0)) == 0:
                        x = float(low.split()[1].replace("in", ""))
                        node["offset_custom_in"] = x
                elif low in ("reshape", "rs", "geom"):
                    acts.append({"type": "reshape"})
                elif low.startswith(("latspeed", "lat", "drive_speed")):
                    parts_split = low.split()
                    if len(parts_split) >= 2:
                        token = parts_split[1]
                        if token in ("off", "none", "clear"):
                            node.pop("custom_lateral_ips", None)
                        else:
                            try:
                                val = float(token.replace("ips", ""))
                                node["custom_lateral_ips"] = val if val > 0 else None
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

# ---------------- output window ----------------
def output_refresh():
    """Refresh output window content."""
    global tk_output_text
    if tk_output_win and tk_output_text:
        try:
            tk_output_text.config(state="normal")
            tk_output_text.delete("1.0", "end")
            tk_output_text.insert("end", "\n".join(log_lines))
            tk_output_text.see("end")
            tk_output_text.config(state="disabled")
        except Exception: 
            pass

def toggle_output_window():
    """Toggle output window visibility."""
    global tk_output_win, tk_output_text
    ensure_tk_root()
    if tk_output_win and tk_output_win.winfo_exists():
        try: 
            tk_output_win.destroy()
        except Exception: 
            pass
        tk_output_win = None
        tk_output_text = None
        return
    top = tk.Toplevel(tk_root)
    tk_output_win = top
    top.title("Output")
    top.geometry("700x500")
    top.resizable(True, True)
    txt = tk.Text(top, wrap="word")
    txt.pack(fill="both", expand=True)
    txt.insert("end", "\n".join(log_lines))
    txt.config(state="disabled")
    tk_output_text = txt
    top.protocol("WM_DELETE_WINDOW", lambda: top.destroy())


def open_settings_window():
    """Open settings configuration window."""
    global tk_settings_win, CFG, initial_state, robot_heading
    ensure_tk_root()
    if tk_settings_win is not None and tk_settings_win.winfo_exists(): 
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
    
    top = tk.Toplevel(tk_root)
    tk_settings_win = top
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
    
    for name in ("general", "physics", "geometry", "codegen"):
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
    auto_heading_node1_var = tk.IntVar(value=0)
    
    # Physics variables
    rp = CFG["robot_physics"]
    rpm_var = tk.StringVar(value=str(rp.get("rpm", 0)))
    diam_var = tk.StringVar(value=str(rp.get("diameter", 4)))
    vs_var = tk.StringVar(value=str(rp.get("volts_straight", 12)))
    vt_var = tk.StringVar(value=str(rp.get("volts_turn", 12)))
    w_var = tk.StringVar(value=str(rp.get("weight", 20)))
    tb_var = tk.StringVar(value=str(rp.get("t_buffer", 0)))
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
        add_tooltip(lbl, tip)
        add_tooltip(widget, tip)
        track_live_widget(widget)
    
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
            if pd.get("use_path", False):
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
    
        if path_pts and len(path_pts) > 1:
            try:
                hdg = calculate_path_heading(path_pts, 0)
            except Exception:
                hdg = heading_from_points(path_pts[0], path_pts[1])
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
            for act in node.get("actions", []):
                if act.get("type") == "turn":
                    try:
                        ang = float(act.get("deg", 0.0))
                    except Exception:
                        continue
                    act["deg"] = (180.0 - ang) % 360.0
        
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
    _row(tabs["general"], 4, "Show field objects:", ttk.Checkbutton(tabs["general"], variable=show_field_objects_var), "Draw field objects")
    _row(tabs["general"], 5, "Flip routine:", ttk.Button(tabs["general"], text="Flip", command=_flip_routine_horizontal), "Mirror horizontally")
    
    # Physics tab rows
    _row(tabs["physics"], 0, "Drive RPM:", ttk.Entry(tabs["physics"], textvariable=rpm_var), "Affects vmax and accel")
    _row(tabs["physics"], 1, "Wheel diameter (in):", ttk.Entry(tabs["physics"], textvariable=diam_var), "Wheel size")
    _row(tabs["physics"], 2, "Straight volts (0–12):", ttk.Entry(tabs["physics"], textvariable=vs_var), "Forward voltage")
    _row(tabs["physics"], 3, "Turn volts (0–12):", ttk.Entry(tabs["physics"], textvariable=vt_var), "Turn voltage")
    _row(tabs["physics"], 4, "Robot weight (lb):", ttk.Entry(tabs["physics"], textvariable=w_var), "Robot mass")
    _row(tabs["physics"], 5, "Buffer Time (s):", ttk.Entry(tabs["physics"], textvariable=tb_var), "Pause per node")
    _row(tabs["physics"], 6, "Gear ratio:", ttk.Entry(tabs["physics"], textvariable=gr_var), "Motor:wheel ratio")
    _row(tabs["physics"], 7, "Point density (/in):", ttk.Entry(tabs["physics"], textvariable=dens_var), "Uniform resample density for curved paths")
    curv_disp = tk.StringVar(value=f"{curv_var.get():.3f}")
    def _curv_live(_=None):
        try:
            val = float(curv_var.get())
            CFG["robot_physics"]["curvature_gain"] = val
            curv_disp.set(f"{val:.3f}")
        except Exception:
            pass
    curv_frame = ttk.Frame(tabs["physics"])
    curv_scale = ttk.Scale(curv_frame, orient="horizontal", from_=0.0, to=1.0, variable=curv_var, command=lambda _evt=None: _curv_live(), length=180)
    curv_scale.pack(side="left", padx=(0,6))
    curv_label = ttk.Label(curv_frame, textvariable=curv_disp, width=7)
    curv_label.pack(side="left")
    _row(tabs["physics"], 9, "Curvature gain:", curv_frame, "Higher = stronger slow-down in tight turns (live)")
    
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
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}}});",
            "turn_global": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
            "turn_local": "chassis.turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}}});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled per-command",
            "reverse_off": "// reverse handled per-command",
            "tbuffer": "pros::delay({MS});",
            "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {{.forwards = {FORWARDS}}});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "PROS": {
            "wait": "pros::delay({MS});",
            "move": "drive_distance({DIST_IN});",
            "turn_global": "turn_to({HEADING_DEG});",
            "turn_local": "turn_angle({TURN_DELTA_DEG});",
            "pose": "// move_to_pose x={X_IN}, y={Y_IN}, h={HEADING_DEG}",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "path_follow": 'follow_path("{PATH_FILE}", {LOOKAHEAD});',
            "setpose": "// set pose {X_IN},{Y_IN},{HEADING_DEG}"
        },
        "Custom": {
            "wait": "pros::delay({MS});",
            "move": "move({DIST_IN});",
            "turn_global": "face({HEADING_DEG});",
            "turn_local": "turn_relative({TURN_DELTA_DEG});",
            "pose": "pose({X_IN}, {Y_IN}, {HEADING_DEG});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
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
            "min_timeout_s": 0.0
        },
        "path_dir": "export/paths"
    })

    # Initialize defaults for each style if not present
    for _style, _tpl in codegen_defaults.items():
        CFG["codegen"].setdefault("templates", {}).setdefault(_style, dict(_tpl))
        CFG["codegen"]["templates"].setdefault(_style, {}).setdefault("__optional__", ["setpose"])

    style_labels = ["Action List", "LemLib", "PROS", "Custom"]
    codegen_style_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("style", "Action List")))

    # Template variables for customizable tokens
    base_tpl_keys = ["wait", "move", "turn_global", "turn_local", "pose", "path_follow", "tbuffer", "setpose"]
    optional_pool = ["reverse_on", "reverse_off", "reshape", "setpose"]
    tpl_keys = base_tpl_keys + optional_pool
    tpl_vars = {k: tk.StringVar() for k in tpl_keys}
    motion_mode_var = tk.StringVar(value="move")
    turn_mode_var = tk.StringVar(value="turn_global")

    # Options
    ticks_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("ticks_per_rotation", 360)))
    pad_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("pad_factor", 1.0)))
    min_s_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("min_timeout_s", 0.0)))
    path_dir_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("path_dir", "export/paths")))

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
    add_tooltip(ticks_label, "Encoder ticks per wheel rotation for distance conversion.")
    add_tooltip(ticks_entry, "Encoder ticks per wheel rotation for distance conversion.")
    track_live_widget(ticks_entry)

    path_dir_frame = ttk.Frame(tabs["codegen"])
    path_dir_entry = ttk.Entry(path_dir_frame, textvariable=path_dir_var)
    path_dir_entry.pack(side="left", fill="x", expand=True)
    path_dir_btn = ttk.Button(path_dir_frame, text="Browse...", command=_browse_path_dir)
    path_dir_btn.pack(side="left", padx=4)
    _row(tabs["codegen"], 4, "Path export dir:", path_dir_frame, "Directory to save generated path files.")
    add_tooltip(path_dir_btn, "Directory to save generated path files.")
    track_live_widget(path_dir_entry)

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
        for w in list(live_update_widgets):
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
        available_optional = [c for c in optional_pool if c not in active_optional]

        # Mode selectors
        mode_row = ttk.Frame(tpl_panel)
        mode_row.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        ttk.Label(mode_row, text="Motion cmd:").pack(side="left")
        cb_motion = ttk.Combobox(mode_row, width=10, values=["move", "pose"], state="readonly", textvariable=motion_mode_var)
        cb_motion.pack(side="left", padx=3)
        cb_motion.bind("<<ComboboxSelected>>", _on_mode_change)
        ttk.Label(mode_row, text="Turn cmd:").pack(side="left", padx=(10, 3))
        cb_turn = ttk.Combobox(mode_row, width=12, values=["turn_global", "turn_local"], state="readonly", textvariable=turn_mode_var)
        cb_turn.pack(side="left")
        cb_turn.bind("<<ComboboxSelected>>", _on_mode_change)

        # Header
        hdr1 = ttk.Label(tpl_panel, text="Token", font=("Segoe UI", 10, "bold"))
        hdr2 = ttk.Label(tpl_panel, text="Template", font=("Segoe UI", 10, "bold"))
        hdr1.grid(row=1, column=0, sticky="w", padx=(0, 6))
        hdr2.grid(row=1, column=1, sticky="ew")
        tpl_panel.columnconfigure(1, weight=1)
        
        # Help text for each token
        help_map = {
            "wait": "{MS} or {S} delay tokens. TIMEOUT_MS/S include pad- and min floor.",
            "move": "{DIST_IN}/{X_IN},{Y_IN} {FORWARDS}; {MOVE_SPEED} optional ips override",
            "turn_global": "Field heading {HEADING_DEG}; {TURN_SPEED} optional dps",
            "turn_local": "Relative turn {TURN_DELTA_DEG}; {TURN_SPEED} optional dps",
            "pose": "{X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}; {FORWARDS}",
            "setpose": "Set initial pose {X_IN},{Y_IN},{HEADING_DEG}",
            "reshape": "{STATE} 1=normal, 2=reshaped",
            "reverse_on": "Toggle reverse drive ON",
            "reverse_off": "Toggle reverse drive OFF",
            "tbuffer": "{MS} or {S} for buffer wait",
            "path_follow": "Tokens: {PATH_NAME}, {PATH_FILE}, {PATH_ASSET}, {LOOKAHEAD}, {TIMEOUT_MS}, {FORWARDS}, {PATH_MIN_SPEED}, {PATH_MAX_SPEED}."
        }
        
        # Active command list honoring selected modes (pose and turn can coexist)
        motion_key = modes.get("motion") or "pose"
        turn_key = modes.get("turn") or ("turn_global" if style == "LemLib" else "turn_local")
        base_cmds = ["wait", motion_key, turn_key, "path_follow", "tbuffer"]
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

        row_idx = 0
        for key in active_cmds:
            if key not in tpl_vars:
                continue
            ttk.Label(list_panel, text=key).grid(row=row_idx, column=0, sticky="w", padx=(0, 6), pady=2)
            ent = ttk.Entry(list_panel, textvariable=tpl_vars[key])
            ent.grid(row=row_idx, column=1, sticky="ew", pady=2)
            add_tooltip(ent, help_map.get(key, ""))
            track_live_widget(ent)
            if key in optional_pool:
                ttk.Button(list_panel, text="Remove", command=lambda k=key: _remove_optional(k)).grid(row=row_idx, column=2, padx=4, pady=2)
            row_idx += 1
        list_panel.columnconfigure(1, weight=1)
        tpl_panel.rowconfigure(2, weight=1)
        
        # Add commands section (static)
        add_frame = ttk.Frame(tpl_panel)
        add_frame.grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(add_frame, text="Add commands:").pack(side="left")
        for opt in available_optional:
            ttk.Button(add_frame, text=opt, command=lambda k=opt: _add_optional(k)).pack(side="left", padx=3)
        
        # Action buttons (static)
        btns = ttk.Frame(tpl_panel)
        btns.grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Button(btns, text="Reset to Defaults", command=_reset_defaults).pack(side="left", padx=(0, 6))
        ttk.Button(btns, text="Save Templates", command=_persist_now).pack(side="left")
        tpl_panel.after(0, _bind_tpl_live_handlers)
        
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
            "min_timeout_s": float(min_s_var.get() or 0.0)
        })
        tpl_panel.after(0, _bind_tpl_live_handlers)

    def _on_style_change(_=None):
        """Handle style dropdown change."""
        _rebuild_tpl_panel()
    style_widget.bind("<<ComboboxSelected>>", _on_style_change)

    def _bind_live_handlers():
        """Attach live update events to tracked widgets."""
        for w in list(live_update_widgets):
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
            CFG["robot_physics"]["volts_straight"] = float(vs_var.get())
            CFG["robot_physics"]["volts_turn"] = float(vt_var.get())
            CFG["robot_physics"]["weight"] = float(w_var.get())
            CFG["robot_physics"]["t_buffer"] = float(tb_var.get())
            CFG["robot_physics"]["point_density_per_in"] = float(dens_var.get() or rp.get("point_density_per_in", 4.0))
            CFG["robot_physics"]["curvature_gain"] = float(curv_var.get() or rp.get("curvature_gain", 0.05))
            CFG["gear_ratio"] = float(gr_var.get())
            
            # Update UI
            CFG.setdefault("ui", {})["show_hitboxes"] = int(show_hitboxes_var.get())
            CFG.setdefault("ui", {})["show_field_objects"] = int(show_field_objects_var.get())
            
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
            pc["min_speed_ips"] = float(pc.get("min_speed_ips", 30.0))
            pc["max_speed_ips"] = float(pc.get("max_speed_ips", 127.0))
            pc["simulate_pursuit"] = int(path_lookahead_enabled)
            
            # Refresh lookahead globals
            path_lookahead_enabled = bool(pc.get("simulate_pursuit", 1))
            path_lookahead_px = max(0.0, float(pc.get("lookahead_in", auto_lookahead_in(CFG))) * PPI)
            last_lookahead_radius = path_lookahead_px
            
            # Update codegen
            CFG.setdefault("codegen", {})["style"] = codegen_style_var.get()
            CFG["codegen"].setdefault("opts", {}).update({
                "ticks_per_rotation": float(ticks_var.get() or 360),
                "pad_factor": float(pad_var.get() or 1.0),
                "min_timeout_s": float(min_s_var.get() or 0.0)
            })
            CFG["codegen"]["path_dir"] = path_dir_var.get().strip() or "export/paths"
            _save_tpl_vars_for(codegen_style_var.get())  # Save current templates
            
            # Save to disk
            save_config(CFG)
            
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
    global path_lookahead_enabled, path_lookahead_px, last_lookahead_point, last_lookahead_radius, last_heading_target, path_mirror_start, path_mirror_end
    global offset_dragging_idx, offset_drag_prev
    
    open_settings_window()
    
    running = True
    while running:
        clock.tick(60)
        pump_tk()
        
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
                       round(float(a.get("deg", 0.0)), 3)) if a.get("type") == "turn"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),)
                 for a in n.get("actions", [])
             ),
             (None if n.get("custom_lateral_ips") is None else round(float(n.get("custom_lateral_ips", 0.0)), 3)),
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
                    toggle_output_window()
                
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
                    tl = apply_tbuffer(CFG, compile_timeline(display_nodes, CFG, initial_state["heading"], fps=60))
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
                            if display_nodes[0].get("reshape_toggle", False): 
                                reshape_live = not reshape_live
                            for a in display_nodes[0].get("actions", []):
                                if a.get("type") in ("reshape", "geom"): 
                                    reshape_live = not reshape_live
                            timeline = apply_tbuffer(CFG, compile_timeline(display_nodes, CFG, initial_state["heading"], fps))
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
                
                elif event.key in (pygame.K_DELETE, pygame.K_BACKSPACE) and selected_idx is not None:
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
                        keys = pygame.key.get_pressed()
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
                                init_min = pd_cur.get("min_speed_ips", CFG.get("path_config", {}).get("min_speed_ips", 30.0))
                                init_max = pd_cur.get("max_speed_ips", CFG.get("path_config", {}).get("max_speed_ips", 127.0))
                                init_la = pd_cur.get("lookahead_in_override", CFG.get("path_config", {}).get("lookahead_in", auto_lookahead_in(CFG)))
                                inp = simpledialog.askstring("Path speeds / lookahead", "Enter min,max[,lookahead in]:", initialvalue=f"{init_min},{init_max},{init_la}")
                                if inp:
                                    parts = [p.strip() for p in inp.replace(";", ",").split(",")]
                                    if len(parts) >= 2:
                                        vmin = float(parts[0]); vmax = float(parts[1])
                                        pd = display_nodes[hit_seg].setdefault("path_to_next", {})
                                        pd["min_speed_ips"] = vmin
                                        pd["max_speed_ips"] = vmax
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
                                cmd = simpledialog.askstring(
                                    "Node actions",
                                    "Commands (comma/semicolon separated). Examples:\n"
                                    "  turn 45, wait 2, turn 90\n"
                                    "  wait 1.5; turn -30\n"
                                    "  offset 7   (custom offset)\n"
                                    "  reshape    (toggle geometry)\n"
                                    "  latspeed 50   (drive command override)\n"
                                    "  turnspeed 180 (deg/s override)\n",
                                    initialvalue=init_cmd
                                )
                                if cmd is not None:
                                    parse_and_apply_cmds(start_node, cmd, hit_seg)
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
                            cmd = simpledialog.askstring(
                                "Node actions",
                                "Commands (comma/semicolon separated). Examples:\n"
                                "  turn 45, wait 2, turn 90\n"
                                "  wait 1.5; turn -30\n"
                                "  offset 7   (custom offset)\n"
                                "  reshape    (toggle geometry)\n"
                                "  latspeed 50   (drive ips override)\n"
                                "  turnspeed 180 (deg/s override)\n"
                                "Angles use 0=left, 90=up, 180=right.",
                                initialvalue=init
                            )
                            if cmd is not None:
                                parse_and_apply_cmds(node, cmd, pick)
                            # Keep node anchored while editing commands
                            node["pos"] = node_pos_locked
                            display_nodes[pick]["pos"] = node_pos_locked
                            correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                            sync_all_path_endpoints()
                            util_push_undo_prev(undo_stack, prev)
                            last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                            last_path_sig = None
                            total_estimate_s = compute_total_estimate_s()

            if event.type == pygame.MOUSEMOTION:
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
                    shift_now = bool(mods_now & pygame.KMOD_SHIFT)
                    if shift_now and not constrain_active:
                        constrain_active = True
                        constrain_origin = tuple(display_nodes[selected_idx]["pos"])
                    elif not shift_now and constrain_active:
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
                    last_logged_seg = seg_i
                
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
                    elif seg_type == "turn":
                        robot_pos, robot_heading = seg["pos"], seg["target_heading"]
                    elif seg_type == "wait":
                        robot_pos = seg.get("pos", robot_pos)
                        robot_heading = seg.get("heading", robot_heading)
                    elif seg_type == "move":
                        robot_pos, robot_heading = seg["p1"], seg.get("facing", robot_heading)
                    elif seg_type == "reshape":
                        reshape_live = True if seg.get("state", 1) == 2 else False
                    seg_i += 1
                    t_local = 0.0
                else:
                    t_local = min(T, t_local + (1.0/60.0))
                    if seg_type == "move":
                        L_px = math.hypot(seg["p1"][0]-seg["p0"][0], seg["p1"][1]-seg["p0"][1])
                        L_in = L_px / PPI
                        drive_override = seg.get("drive_speed_ips")
                        s_in = sample_move_profile(t_local, L_in, CFG, v_override=drive_override)
                        frac = 0.0 if L_in <= 0 else max(0.0, min(1.0, s_in / max(1e-6, L_in)))
                        robot_pos = (seg["p0"][0] + frac*(seg["p1"][0]-seg["p0"][0]),
                                    seg["p0"][1] + frac*(seg["p1"][1]-seg["p0"][1]))
                        robot_heading = seg.get("facing", robot_heading)
                    
                    elif seg_type in ("path", "path_follow"):
                        path_points = seg.get("path_points", [])
                        use_pursuit = path_lookahead_enabled
                        la_px = seg.get("lookahead_px", path_lookahead_px)
                        path_speeds = seg.get("path_speeds")
                        if use_pursuit:
                            # Pure pursuit-style step toward lookahead
                            _, heading_to_look, look_pt = sample_path_position(
                                t_local, path_points, T, CFG,
                                lookahead_px=la_px,
                                use_pursuit=True,
                                path_speeds=path_speeds,
                                current_pos=robot_pos
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
                                speed_ips = seg.get("path_speeds", [None])[0] if path_speeds else CFG.get("path_config", {}).get("max_speed_ips", 60.0)
                                try:
                                    idx_progress = int(max(0, min(len(path_points)-1, (t_local / max(1e-6, T)) * (len(path_points)-1))))
                                    if path_speeds:
                                        speed_ips = path_speeds[idx_progress]
                                except Exception:
                                    pass
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
                                    path_speeds=path_speeds
                                )
                                last_heading_target = None
                            last_lookahead_radius = la_px
                        else:
                            robot_pos, heading_sample, look_pt = sample_path_position(
                                t_local, path_points, T, CFG,
                                lookahead_px=la_px,
                                use_pursuit=False,
                                path_speeds=path_speeds
                            )
                            if seg.get("reverse"):
                                heading_sample = (heading_sample + 180.0) % 360.0
                            robot_heading = clamp_heading_rate(robot_heading, heading_sample, CFG, dt=1.0/60.0)
                            last_lookahead_point = None
                            last_heading_target = None
                    
                    elif seg_type == "wait":
                        robot_pos = seg.get("pos", robot_pos)
                        robot_heading = seg.get("heading", robot_heading)
                    
                    elif seg_type == "turn":
                        robot_pos = seg["pos"]
                        robot_heading = sample_turn_heading_trap(t_local, seg["start_heading"], seg["target_heading"], CFG, rate_override=seg.get("turn_speed_dps"))
                    
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
                        elif seg_type == "reshape":
                            reshape_live = True if seg.get("state", 1) == 2 else False
                        seg_i += 1
                        t_local = 0.0
        
        #   Drawing
        screen.fill(BG_COLOR)
        draw_grid(screen, GRID_SIZE_PX)
        draw_field_objects(screen, CFG)
        draw_geometry_borders(screen, display_nodes, CFG, initial_state["heading"])
        draw_follow_geometry(screen, CFG, robot_pos, robot_heading, reshape_live)
        
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
            if not (path_data.get("use_path") and path_data.get("control_points")):
                # Draw straight line
                p0, p1 = display_nodes[i]["pos"], display_nodes[i+1]["pos"]
                pygame.draw.line(screen, NODE_COLOR, p0, p1, 2)
        
        # Draw nodes with proper arguments
        draw_nodes(screen, display_nodes, selected_idx, font, CFG, path_edit_mode)
        
        # PATH EDIT MODE OVERLAY
        if path_edit_mode and path_control_points:
            cp_preview = list(path_control_points)
            # Draw smooth path preview
            if PATH_FEATURES_AVAILABLE:
                smooth_path = generate_bezier_path(cp_preview, num_samples=50)
                draw_curved_path(screen, smooth_path, color=(255, 200, 100), width=4)
            
            # Draw control point handles
            draw_path_control_points(screen, path_control_points, selected_control_point)
            
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
