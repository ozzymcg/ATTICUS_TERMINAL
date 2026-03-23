import os, math, json, copy, re, random, sys, subprocess, importlib, importlib.machinery, threading, traceback, time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

APP_ROOT = os.path.dirname(os.path.abspath(__file__))
if getattr(sys, "frozen", False):
    try:
        APP_ROOT = os.path.dirname(os.path.abspath(sys.executable))
    except Exception:
        APP_ROOT = os.getcwd()
try:
    os.chdir(APP_ROOT)
except Exception:
    pass

if os.name == "nt" and not os.environ.get("SDL_VIDEODRIVER"):
    os.environ["SDL_VIDEODRIVER"] = "windows"

try:
    import tkinter as tk
    from tkinter import simpledialog, ttk, messagebox, filedialog
except ModuleNotFoundError:
    msg = (
        "Missing dependency: tkinter.\n\n"
        "Linux users typically need to install your distro tkinter package\n"
        "(for example: python3-tk), then rerun Atticus Terminal."
    )
    print(msg)
    raise SystemExit(1)

try:
    import pygame
except ModuleNotFoundError:
    py_cmd = "py" if os.name == "nt" else "python3"
    exe_hint = f"\"{sys.executable}\" -m pip install ."
    msg = (
        "Missing dependency: pygame.\n\n"
        "Install it with one of:\n"
        f"  {py_cmd} -m pip install .\n"
        f"  {py_cmd} -m pip install -r requirements.txt\n"
        f"  {exe_hint}\n\n"
        "Tip (Windows Store Python): use the `py -m pip ...` form."
    )
    try:
        root = tk.Tk()
        root.withdraw()
        messagebox.showerror("Atticus Terminal Startup Error", msg)
        root.destroy()
    except Exception:
        print(msg)
    raise SystemExit(1)

from mod.config import (
    WINDOW_WIDTH, WINDOW_HEIGHT, GRID_SIZE_PX, BG_COLOR, NODE_COLOR, TEXT_COLOR, PPI,
    save_config, reload_cfg, auto_lookahead_in, DEFAULT_CONFIG, MCL_CONFIG_VERSION
)
from mod.geom import (
    convert_heading_input, interpret_input_angle, heading_from_points,
    coords_str, constrain_to_8dirs, approach_unit, point_in_poly
)
from mod.sim import (
    snap_to_grid, compile_timeline,
    sample_turn_heading_trap, sample_move_profile, vmax_straight, accel_straight, _turn_accel_deg_s2,
    correct_nodes_inbounds, sample_path_position, path_time_with_curvature, turn_rate,
    boomerang_curve_points
)
from mod.draw import (
    draw_grid, draw_robot, draw_chevron, draw_nodes,
    draw_hover_box, draw_time_label, draw_constraint_visual,
    draw_field_objects, draw_geometry_borders, draw_follow_geometry,
    draw_curved_path, swing_arc_points, draw_path_control_points, draw_path_edit_overlay,
    draw_label, get_field_object_entries
)
from mod.storage import save_nodes, load_nodes, compile_log
from mod.util import (
    pros_convert_inches, build_compile_header, apply_tbuffer,
    snapshot as util_snapshot, push_undo_prev as util_push_undo_prev,
    get_node_offset_in
)
from mod.codegen import build_export_lines, pick_profile, SETTLE_BASE, VOLTAGE_SHAPES
from mod import ui
from mod import localizer_sim as mcl_mod

# macOS Tk/SDL guard:
# If SDL (pygame) initializes NSApp first, modern Tk can crash with
# NSInvalidArgumentException: '-[SDLApplication macOSVersion]'.
# Pre-initializing a hidden Tk root avoids that app-class collision.
if sys.platform == "darwin":
    try:
        ui.ensure_tk_root()
    except Exception:
        pass


try:
    from mod.pathing import generate_bezier_path, calculate_path_heading, resample_path_uniform
    from mod.pathing import export_lemlib_path, generate_path_asset_name
    PATH_FEATURES_AVAILABLE = True
except ImportError:
    PATH_FEATURES_AVAILABLE = False
    print("Warning: Path utilities not available. Path editing disabled.")
    def generate_bezier_path(points, num_samples=50, spline_type=None):
        """Handle generate bezier path."""
        return points
    def calculate_path_heading(points, index):
        """Handle calculate path heading."""
        return 0.0
    def export_lemlib_path(points, name, cfg):
        """Handle export lemlib path."""
        pass
    def generate_path_asset_name(routine, idx):
        """Handle generate path asset name."""
        return f"path_{idx}"

generate_bezier_path: Callable[[Any, int, Optional[str]], Any]
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
    ("B", "Cycle Atticus correction: off -> immediate -> wall trim -> rough wall"),
    ("P", "Toggle path edit mode for \n hovered/selected node"),
    ("U", "Toggle spline type in path edit mode\n(uniform/centripetal)."),
    ("T", "Toggle path look-ahead simulation \n(Pure Pursuit)"),
    ("SPACE / CTRL+SPACE", "Start-Pause / Reset"),
    ("Q", "Toggle Snap-to-Grid"),
    ("F", "Cycle offset: none -> 1 -> 2 -> 3"),
    ("R / G", "Toggle Reverse / Reshape Geometry at node"),
    ("Hold Backspace/Delete + Click", "Delete clicked node(s)"),
    ("Delete / Backspace", "Delete currently selected node\n(node 0 is protected)."),
    ("CTRL+SHIFT+Backspace/Delete", "Reset field to node 0"),
    ("CTRL+Z / CTRL+Y", "Undo / Redo"),
    ("S / L", "Save / Load routine"),
    ("C", "Export generated program"),
    ("O", "Toggle Output window"),
]

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

TEMPLATE_BG = "#f2f4f7"
ATTICUS_DSR_RECOMMENDED_STILL_MS = 20.0
ATTICUS_IMMEDIATE_HIGHLIGHT_MS = 220.0

_LOCKED_MECH_PRESET_SPECS = {
    "reshape": {"name": "reshape", "mode": "toggle", "template": "", "on": "", "off": "", "default": False},
    "dsr": {"name": "DSR", "mode": "action", "template": "", "on": "", "off": "", "default": False},
}


def _locked_mech_preset_key(name: object) -> Optional[str]:
    """Return the normalized locked-preset key for a built-in mechanism command."""
    key = str(name or "").strip().lower()
    return key if key in _LOCKED_MECH_PRESET_SPECS else None


def _locked_mech_preset_spec(key: str) -> dict:
    """Return a fresh locked-preset payload for the requested built-in command."""
    spec = _LOCKED_MECH_PRESET_SPECS.get(str(key or "").strip().lower())
    return dict(spec) if isinstance(spec, dict) else {}


def _ensure_locked_mech_presets(cfg: dict) -> list[dict]:
    """Keep built-in mechanism commands present, deduplicated, and uneditable."""
    codegen = cfg.setdefault("codegen", {}) if isinstance(cfg, dict) else {}
    presets = codegen.get("mech_presets", [])
    if not isinstance(presets, list):
        presets = []
        codegen["mech_presets"] = presets
    merged = []
    seen_locked = set()
    for item in presets:
        if not isinstance(item, dict):
            continue
        locked_key = _locked_mech_preset_key(item.get("name", ""))
        if locked_key is None:
            merged.append(item)
            continue
        if locked_key in seen_locked:
            continue
        merged.append(_locked_mech_preset_spec(locked_key))
        seen_locked.add(locked_key)
    for locked_key in ("reshape", "dsr"):
        if locked_key not in seen_locked:
            merged.append(_locked_mech_preset_spec(locked_key))
    presets[:] = merged
    return presets

def _default_attlib_sim_module_dir() -> str:
    """Best-effort default directory for the built attlib_sim module."""
    base = Path(APP_ROOT).resolve().parent
    candidates = [
        base / "Project Atticus" / "atticode" / "host" / "build313",
        base / "Project Atticus" / "atticode" / "host" / "build",
    ]
    ext_suffixes = tuple(importlib.machinery.EXTENSION_SUFFIXES or ())
    for path in candidates:
        if not path.exists():
            continue
        if any((path / f"attlib_sim{sfx}").exists() for sfx in ext_suffixes):
            return str(path)
    for path in candidates:
        if path.exists():
            return str(path)
    return str(candidates[-1])

def _default_attlib_sim_routine_script() -> str:
    """Best-effort default script used for external AttLibSim routine mode."""
    base = Path(APP_ROOT).resolve().parent
    return str(base / "Project Atticus" / "atticode" / "host" / "run_attlibsim.py")

CFG = reload_cfg()
_ensure_locked_mech_presets(CFG)

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
_ESTIMATE_CACHE_VERSION = 1
_estimate_total_cache = {}
_path_duration_cache = {}

_attlib_cfg = CFG.setdefault("codegen", {}).setdefault("attlib_sim", {})
def _attlib_cfg_get(name: str, default):
    """Read an AttLibSim config field with compatibility for wrapped values."""
    value = _attlib_cfg.get(name, default)
    if isinstance(value, dict) and "value" in value:
        return value.get("value", default)
    return value

attlib_sim_visual_run = bool(_attlib_cfg_get("visual_run", 0))
attlib_sim_module_dir = str(_attlib_cfg_get("module_dir", _default_attlib_sim_module_dir()))
attlib_sim_source = str(_attlib_cfg_get("source", "timeline"))
attlib_sim_routine_script = str(_attlib_cfg_get("routine_script", _default_attlib_sim_routine_script()))
attlib_sim_routine_func = str(_attlib_cfg_get("routine_function", "run_routine"))
attlib_sim_status = "idle"
attlib_sim_runtime = {
    "running": False,
    "module": None,
    "chassis": None,
    "mode": "timeline",
    "plan": [],
    "index": 0,
    "wait_elapsed_s": 0.0,
    "external_thread": None,
    "external_done": False,
    "external_error": None,
    "external_result": None,
}

path_lookahead_enabled = bool(CFG.get("path_config", {}).get("simulate_pursuit", 1))
path_lookahead_px = float(CFG.get("path_config", {}).get("lookahead_in", auto_lookahead_in(CFG))) * PPI
last_lookahead_point = None
last_lookahead_radius = path_lookahead_px
last_heading_target = None
path_draw_cache = {}  # cache for gradient-drawn path segments
insert_preview = None  # (segment_idx, (x,y), t) while shift-inserting

path_edit_mode = False
path_edit_segment_idx = None  # Which segment (node index) is being edited
path_control_points = []  # Current control points being edited
selected_control_point = None  # Index of selected control point
dragging_control_point = False
PATH_CONTROL_RADIUS = 8  # Click detection radius for control points
EDGE_MARKER_RADIUS = 7
EDGE_MARKER_HIT_RADIUS = 10
edge_marker_drag = None  # {"seg_idx": int, "event_idx": int, "prev": snapshot}
PATH_SPLINE_OPTIONS = ("uniform", "centripetal")

TUTORIAL_FLAG_DEFAULTS = {
    "added_node": False,
    "dragged_node": False,
    "axis_lock_used": False,
    "shift_insert": False,
    "right_click_prompt": False,
    "mech_marker": False,
    "entered_path_edit": False,
    "spline_toggled": False,
    "path_speed_prompt": False,
    "ran_sim": False,
    "opened_template_editor": False,
    "save_load_used": False,
    "compiled_once": False,
}

tutorial_state = {
    "win": None,
    "flags": dict(TUTORIAL_FLAG_DEFAULTS),
    "tabs_seen": set(),
    "active": False,
}

mcl_state = mcl_mod.MCLState()
mcl_enabled = bool(CFG.get("mcl", {}).get("enabled", 0))
atticus_runtime = {
    "wall_trim_ctx": None,
    "rough_wall_ctx": None,
    "highlight_mode": None,
    "highlight_sensors": [],
    "highlight_ms": 0.0,
    "sticky": False,
}

def _atticus_style_selected() -> bool:
    """True when the current export style is the Atticus localizer surface."""
    style = CFG.get("codegen", {}).get("style", "")
    if isinstance(style, dict):
        style = style.get("value", "")
    style = str(style).strip()
    if style == "AttLib":
        style = "Atticus"
    return style == "Atticus"


def _node_atticus_correction_mode(node: dict):
    """Return normalized per-node Atticus correction mode."""
    mode = str(node.get("atticus_correction_mode", "") or "").strip().lower()
    if mode in ("immediate", "wall_trim", "rough_wall"):
        return mode
    return None


def _set_node_atticus_correction_mode(node: dict, mode):
    """Store or clear the per-node Atticus correction mode."""
    if mode in ("immediate", "wall_trim", "rough_wall"):
        node["atticus_correction_mode"] = str(mode)
    else:
        node.pop("atticus_correction_mode", None)


def _clear_atticus_runtime():
    """Clear any active manual Atticus correction state/highlights."""
    atticus_runtime["wall_trim_ctx"] = None
    atticus_runtime["rough_wall_ctx"] = None
    atticus_runtime["highlight_mode"] = None
    atticus_runtime["highlight_sensors"] = []
    atticus_runtime["highlight_ms"] = 0.0
    atticus_runtime["sticky"] = False
    try:
        mcl_state.rough_wall_ctx = None
    except Exception:
        pass


def _set_atticus_ray_highlight(mode: str, sensor_names, hold_ms: float = 0.0, sticky: bool = False):
    """Track a temporary or sticky ray highlight for manual Atticus corrections."""
    names = sorted({str(name).strip() for name in (sensor_names or []) if str(name).strip()})
    if not names:
        if not sticky:
            atticus_runtime["highlight_mode"] = None
            atticus_runtime["highlight_sensors"] = []
            atticus_runtime["highlight_ms"] = 0.0
            atticus_runtime["sticky"] = False
        return
    atticus_runtime["highlight_mode"] = str(mode or "").strip().lower() or None
    atticus_runtime["highlight_sensors"] = names
    atticus_runtime["highlight_ms"] = max(0.0, float(hold_ms or 0.0))
    atticus_runtime["sticky"] = bool(sticky)


def _apply_atticus_ray_highlight(dt_ms: float = 0.0):
    """Project the active manual-correction highlight onto the latest sensor rays."""
    if not atticus_runtime.get("sticky"):
        try:
            atticus_runtime["highlight_ms"] = max(0.0, float(atticus_runtime.get("highlight_ms", 0.0)) - float(dt_ms or 0.0))
        except Exception:
            atticus_runtime["highlight_ms"] = 0.0
        if atticus_runtime.get("highlight_ms", 0.0) <= 1e-6:
            atticus_runtime["highlight_mode"] = None
            atticus_runtime["highlight_sensors"] = []
    mode = atticus_runtime.get("highlight_mode")
    names = set(atticus_runtime.get("highlight_sensors", []))
    for ray in mcl_state.last_rays:
        if not isinstance(ray, dict):
            continue
        if mode and ray.get("name") in names:
            ray["highlight_mode"] = mode
        else:
            ray.pop("highlight_mode", None)


def _atticus_sim_measurements():
    """Capture a fresh Atticus-localizer measurement snapshot at the current true pose."""
    if mcl_state.lf is None or not mcl_state.map_segments:
        mcl_mod.update_map_segments(mcl_state, CFG)
    mcl_pose = (robot_pos[0], robot_pos[1], _mcl_heading_from_internal(robot_heading))
    return mcl_mod.simulate_measurements(mcl_state, CFG, mcl_pose, add_noise=True)


def _atticus_apply_immediate_correction(sensor_tokens: Optional[object] = None):
    """Run a one-shot Atticus translation cleanup in the simulator."""
    if not mcl_enabled:
        return
    measurements = _atticus_sim_measurements()
    resolved_names = []
    if sensor_tokens is not None and str(sensor_tokens).strip() != "":
        resolved_names = mcl_mod.resolve_distance_sensor_names(CFG, sensor_tokens, max_sensors=2)
    if sensor_tokens is None or str(sensor_tokens).strip() == "" or not resolved_names:
        accepted = mcl_mod.apply_immediate_correction_auto(mcl_state, CFG, measurements)
        fallback_names = []
    else:
        accepted = mcl_mod.apply_immediate_correction_sensors(mcl_state, CFG, measurements, sensor_tokens)
        fallback_names = list(resolved_names)
    names = accepted
    if not names and fallback_names:
        names = list(fallback_names)
    if not names and isinstance(measurements, dict) and (sensor_tokens is None or str(sensor_tokens).strip() == ""):
        dist_meas = measurements.get("distance", {})
        if isinstance(dist_meas, dict):
            names = list(dist_meas.keys())
    _set_atticus_ray_highlight("immediate", names, hold_ms=ATTICUS_IMMEDIATE_HIGHLIGHT_MS, sticky=False)


def _atticus_start_wall_trim():
    """Arm a simulated theta-wall-alignment segment and highlight the chosen sensor ray."""
    if not mcl_enabled:
        return
    measurements = _atticus_sim_measurements()
    ctx = mcl_mod.start_theta_wall_alignment_auto(mcl_state, CFG, measurements)
    if ctx is None:
        _set_atticus_ray_highlight("wall_trim", [], hold_ms=0.0, sticky=False)
        atticus_runtime["wall_trim_ctx"] = None
        return
    atticus_runtime["wall_trim_ctx"] = ctx
    atticus_runtime["rough_wall_ctx"] = None
    mcl_state.rough_wall_ctx = None
    _set_atticus_ray_highlight("wall_trim", [ctx.get("sensor_name", "")], sticky=True)


def _atticus_end_wall_trim():
    """Stop the active simulated theta-wall-alignment segment."""
    atticus_runtime["wall_trim_ctx"] = None
    if atticus_runtime.get("highlight_mode") == "wall_trim":
        _set_atticus_ray_highlight("wall_trim", [], hold_ms=0.0, sticky=False)


def _atticus_start_rough_wall():
    """Arm a simulated rough wall-traverse segment and highlight the chosen sensors."""
    if not mcl_enabled:
        return
    measurements = _atticus_sim_measurements()
    ctx = mcl_mod.start_rough_wall_traverse_auto(mcl_state, CFG, measurements)
    if ctx is None:
        _set_atticus_ray_highlight("rough_wall", [], hold_ms=0.0, sticky=False)
        atticus_runtime["rough_wall_ctx"] = None
        mcl_state.rough_wall_ctx = None
        return
    atticus_runtime["rough_wall_ctx"] = ctx
    atticus_runtime["wall_trim_ctx"] = None
    mcl_state.rough_wall_ctx = ctx
    highlight_names = list(ctx.get("selected_sensor_names", []))
    if not highlight_names:
        highlight_names = [ctx.get("sensor_name", "")]
    _set_atticus_ray_highlight("rough_wall", highlight_names, sticky=True)


def _atticus_end_rough_wall():
    """Stop the active simulated rough wall-traverse segment."""
    atticus_runtime["rough_wall_ctx"] = None
    mcl_state.rough_wall_ctx = None
    if atticus_runtime.get("highlight_mode") == "rough_wall":
        _set_atticus_ray_highlight("rough_wall", [], hold_ms=0.0, sticky=False)

def _mcl_heading_from_internal(heading_deg: float) -> float:
    """Convert internal math-frame heading to MCL/UI convention (0=left, clockwise+)."""
    try:
        return convert_heading_input(heading_deg, None)
    except Exception:
        return float(heading_deg or 0.0)

def _internal_heading_from_mcl(heading_deg: float) -> float:
    """Convert MCL/UI heading to internal math-frame heading."""
    try:
        return interpret_input_angle(heading_deg)
    except Exception:
        return float(heading_deg or 0.0)

def _mcl_apply_cfg(reset_particles=False):
    """Apply the current Atticus config and optionally reset the live estimate."""
    global mcl_enabled
    mcl_cfg = CFG.get("mcl", {})
    mcl_enabled = int(mcl_cfg.get("enabled", 0)) == 1
    if not mcl_enabled:
        _clear_atticus_runtime()
        mcl_state.particles = []
        mcl_state.estimate = None
        mcl_state.ekf_pose = None
        mcl_state.ekf_P = None
        return
    if reset_particles:
        _clear_atticus_runtime()
    if reset_particles or mcl_state.estimate is None:
        mcl_state.particles = []
        mcl_heading = _mcl_heading_from_internal(robot_heading)
        mcl_mod.reset_state_to_pose(mcl_state, CFG, (robot_pos[0], robot_pos[1], mcl_heading))
    mcl_mod.update_map_segments(mcl_state, CFG)
    mcl_state.last_true_pose = (robot_pos[0], robot_pos[1], _mcl_heading_from_internal(robot_heading))
    mcl_state.motion_accum = 0.0
    mcl_state.sensor_accum = 0.0
    if mcl_state.ekf_pose is None:
        mcl_heading = _mcl_heading_from_internal(robot_heading)
        mcl_mod.ekf_reset_state(mcl_state, CFG, (robot_pos[0], robot_pos[1], mcl_heading))

def _draw_heading_marker(surface, pos, heading_deg, color, size=10, width=2):
    """Draw heading marker."""
    heading_internal = _internal_heading_from_mcl(heading_deg)
    rad = math.radians(heading_internal)
    tip = (pos[0] + math.cos(rad) * size, pos[1] - math.sin(rad) * size)
    left = (pos[0] + math.cos(rad + 2.5) * size * 0.7, pos[1] - math.sin(rad + 2.5) * size * 0.7)
    right = (pos[0] + math.cos(rad - 2.5) * size * 0.7, pos[1] - math.sin(rad - 2.5) * size * 0.7)
    pygame.draw.polygon(surface, color, [tip, left, right], width)

def _draw_covariance_ellipse(surface, mean, cov, color=(200, 80, 200), n_sigma=2.0):
    """Draw covariance ellipse."""
    var_x, var_y, cov_xy = cov
    if var_x <= 0.0 or var_y <= 0.0:
        return
    trace = var_x + var_y
    det = var_x * var_y - cov_xy * cov_xy
    if det <= 0.0:
        return
    root = max(0.0, (trace * trace) / 4.0 - det)
    eig1 = trace / 2.0 + math.sqrt(root)
    eig2 = trace / 2.0 - math.sqrt(root)
    if eig1 <= 0.0 or eig2 <= 0.0:
        return
    a = math.sqrt(eig1) * n_sigma
    b = math.sqrt(eig2) * n_sigma
    angle = 0.5 * math.atan2(2.0 * cov_xy, var_x - var_y)
    pts = []
    steps = 40
    ca = math.cos(angle)
    sa = math.sin(angle)
    for i in range(steps):
        t = (i / float(steps)) * 2.0 * math.pi
        x = a * math.cos(t)
        y = b * math.sin(t)
        xr = x * ca - y * sa
        yr = x * sa + y * ca
        pts.append((mean[0] + xr, mean[1] + yr))
    if len(pts) >= 3:
        pygame.draw.lines(surface, color, True, pts, 2)

def _draw_mcl_overlay(surface):
    """Draw the live Atticus localization overlay used by the simulator."""
    if not mcl_enabled:
        return
    mcl_cfg = CFG.get("mcl", {})
    ui_cfg = mcl_cfg.get("ui", {})
    show_estimate = int(ui_cfg.get("show_estimate", 1)) == 1
    show_cov = int(ui_cfg.get("show_covariance", 1)) == 1
    show_pred_rays = int(ui_cfg.get("show_rays", 1)) == 1
    show_gating = int(ui_cfg.get("show_gating", 1)) == 1

    if show_cov and mcl_state.estimate is not None:
        _draw_covariance_ellipse(surface, mcl_state.estimate, mcl_state.cov_xy, color=(200, 80, 200), n_sigma=2.0)

    if show_estimate and mcl_state.estimate is not None:
        _draw_heading_marker(surface, mcl_state.estimate, mcl_state.estimate[2], (255, 220, 60), size=12, width=2)

    if mcl_state.last_rays:
        for ray in mcl_state.last_rays:
            if not isinstance(ray, dict):
                continue
            gated = bool(ray.get("gated", False))
            highlight_mode = str(ray.get("highlight_mode", "") or "").strip().lower()
            if highlight_mode == "immediate":
                meas_color = (255, 170, 40)
                pred_color = (255, 220, 120)
            elif highlight_mode == "wall_trim":
                meas_color = (255, 90, 220)
                pred_color = (120, 245, 255)
            elif highlight_mode == "rough_wall":
                meas_color = (255, 120, 120)
                pred_color = (255, 210, 120)
            else:
                meas_color = (80, 200, 80)
                pred_color = (80, 160, 255)
            origin = ray.get("origin")
            meas_end = ray.get("meas_end")
            pred_origin = ray.get("pred_origin")
            pred_end = ray.get("pred_end")
            if origin is not None and meas_end is not None:
                pygame.draw.line(surface, meas_color, origin, meas_end, 2)
                pygame.draw.circle(surface, meas_color, (int(meas_end[0]), int(meas_end[1])), 4, 1)
                if gated and show_gating and highlight_mode not in ("immediate", "wall_trim", "rough_wall"):
                    pygame.draw.circle(surface, (200, 40, 40), (int(meas_end[0]), int(meas_end[1])), 6, 1)
            if show_pred_rays and pred_origin is not None and pred_end is not None:
                pygame.draw.line(surface, pred_color, pred_origin, pred_end, 1)
                pygame.draw.circle(surface, pred_color, (int(pred_end[0]), int(pred_end[1])), 3, 1)

def _edge_events_sig(node):
    """Handle edge events sig."""
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

def _normalize_spline_type(raw):
    """Handle normalize spline type."""
    raw = str(raw or "uniform").strip().lower()
    return "centripetal" if raw in ("centripetal", "cent", "c") else "uniform"

def _path_spline_type(path_data):
    """Handle path spline type."""
    if not isinstance(path_data, dict):
        return "uniform"
    return _normalize_spline_type(path_data.get("spline_type"))

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
        return (False, (), _edge_events_sig(node), chain_flag, chain_val, None)
    cps = pd.get("control_points") or []
    cps_sig = tuple((int(round(cp[0])), int(round(cp[1]))) for cp in cps)
    chain_flag = bool(node.get("chain_through", False) or node.get("chain", False))
    chain_val = node.get("chain_looseness")
    try:
        chain_val = None if chain_val is None else round(float(chain_val), 4)
    except Exception:
        chain_val = None
    spline_type = _path_spline_type(pd)
    return (
        bool(pd.get("use_path", False)),
        cps_sig,
        _edge_events_sig(node),
        chain_flag,
        chain_val,
        spline_type
    )

def _round_sig_scalar(value, digits=4):
    """Round a scalar for cache keys without throwing on bad values."""
    try:
        return round(float(value), digits)
    except Exception:
        return None if value is None else str(value)

def _sig_point(point, digits=2):
    """Convert a 2D point into a compact cache-key tuple."""
    if not isinstance(point, (list, tuple)) or len(point) < 2:
        return None
    try:
        return (round(float(point[0]), digits), round(float(point[1]), digits))
    except Exception:
        return None

def _cache_store(cache: dict, key, value, max_size: int):
    """Store a cache entry and hard-reset oversized caches."""
    if len(cache) >= max(8, int(max_size)):
        cache.clear()
    cache[key] = value

def _timing_actions_sig(node):
    """Keep only action fields that can change motion timing or chaining."""
    acts = node.get("actions")
    if not isinstance(acts, list):
        acts = node.get("actions_out")
    if not isinstance(acts, list):
        acts = []
    sig = []
    for act in acts:
        if not isinstance(act, dict):
            continue
        kind = str(act.get("type", "")).strip().lower()
        if kind == "geom":
            kind = "reshape"
        if kind in ("preset", "code"):
            continue
        if kind == "wait":
            sig.append(("wait", _round_sig_scalar(act.get("s", 0.0), 4)))
        elif kind == "turn":
            sig.append(("turn", _round_sig_scalar(act.get("deg", 0.0), 3)))
        elif kind == "swing":
            sig.append((
                "swing",
                _round_sig_scalar(act.get("deg", 0.0), 3),
                str(act.get("dir", "auto")).strip().lower(),
                bool(act.get("settle", False)),
            ))
        elif kind == "reverse":
            state = act.get("state", None)
            sig.append(("reverse", "toggle" if state is None else bool(state)))
        else:
            sig.append((kind,))
    return tuple(sig)

def _timing_sig_for_node(node):
    """Signature for fields that actually change estimate/runtime timing."""
    pd = node.get("path_to_next")
    if not isinstance(pd, dict):
        pd = {}
    cps = []
    for cp in pd.get("control_points") or []:
        pt_sig = _sig_point(cp)
        if pt_sig is not None:
            cps.append(pt_sig)
    path_sig = (
        bool(pd.get("use_path", False)),
        tuple(cps),
        _normalize_spline_type(pd.get("spline_type")),
        _round_sig_scalar(pd.get("lookahead_in_override"), 3),
        _round_sig_scalar(pd.get("speed_mult", 1.0), 3),
        _round_sig_scalar(pd.get("min_speed_cmd", pd.get("min_speed_ips")), 3),
        _round_sig_scalar(pd.get("max_speed_cmd", pd.get("max_speed_ips")), 3),
    )
    return (
        _sig_point(node.get("pos")),
        _round_sig_scalar(node.get("offset", 0.0), 3),
        _round_sig_scalar(node.get("offset_custom_in"), 3),
        bool(node.get("reverse", False)),
        bool(node.get("reshape_toggle", False)),
        str(node.get("atticus_correction_mode", "") or "").strip().lower(),
        bool(node.get("move_to_pose", False)),
        _round_sig_scalar(node.get("pose_heading_deg"), 3),
        _round_sig_scalar(node.get("pose_lead_in"), 3),
        str(node.get("turn_mode", "") or "").strip().lower(),
        str(node.get("swing_dir", "") or "").strip().lower(),
        bool(node.get("swing_settle", False)),
        _round_sig_scalar(node.get("swing_target_heading_deg"), 3),
        str(node.get("profile_override", "") or "").strip().lower(),
        _round_sig_scalar(_node_lateral_cmd(node), 3),
        bool(_node_lateral_reset(node)),
        _round_sig_scalar(node.get("custom_turn_dps"), 3),
        bool(node.get("chain_through", False) or node.get("chain", False)),
        _round_sig_scalar(node.get("chain_looseness"), 4),
        path_sig,
        _timing_actions_sig(node),
    )

def _estimate_cfg_sig():
    """Compact signature for config values that change time estimation."""
    rp = CFG.get("robot_physics", {})
    bd = CFG.get("bot_dimensions", {})
    pc = CFG.get("path_config", {})
    cal = CFG.get("codegen", {}).get("calibration", {})
    payload = {
        "version": _ESTIMATE_CACHE_VERSION,
        "fps": int(fps),
        "heading": _round_sig_scalar(initial_state.get("heading"), 3),
        "gear_ratio": _round_sig_scalar(CFG.get("gear_ratio"), 4),
        "robot_physics": {k: _round_sig_scalar(rp.get(k), 4) for k in (
            "rpm", "diameter", "volts_straight", "volts_turn", "weight",
            "t_buffer", "advanced_motion", "all_omni", "tracking_wheels",
            "point_density_per_in", "curvature_gain", "mu", "max_cmd"
        )},
        "bot_dimensions": {k: _round_sig_scalar(bd.get(k), 4) for k in ("dt_width", "width")},
        "path_config": {k: _round_sig_scalar(pc.get(k), 4) for k in (
            "lookahead_in", "min_speed_cmd", "max_speed_cmd", "simulate_pursuit", "curvature_gain"
        )},
        "physics_constants": {
            str(k): _round_sig_scalar(v, 5) if isinstance(v, (int, float)) else v
            for k, v in sorted((CFG.get("physics_constants", {}) or {}).items())
        },
        "calibration": json.loads(json.dumps(cal, sort_keys=True, default=str)),
        "lookahead_enabled": bool(path_lookahead_enabled),
        "lookahead_px": _round_sig_scalar(path_lookahead_px, 3),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))

def _estimate_graph_sig():
    """Signature for the current routine's timing-relevant state."""
    return (
        _estimate_cfg_sig(),
        tuple(_timing_sig_for_node(node) for node in display_nodes),
    )

def _path_duration_cache_key(seg, cfg_sig: str, use_pursuit: bool):
    """Key for offline pursuit-duration estimates."""
    pts = []
    for pt in seg.get("path_points", []) or []:
        pt_sig = _sig_point(pt)
        if pt_sig is not None:
            pts.append(pt_sig)
    speeds = []
    for speed in seg.get("path_speeds", []) or []:
        speeds.append(_round_sig_scalar(speed, 4))
    meta = seg.get("path_meta") or {}
    return (
        cfg_sig,
        str(seg.get("type", "")).strip().lower(),
        bool(use_pursuit),
        bool(seg.get("move_to_pose", False)),
        bool(seg.get("reverse", False)),
        _sig_point(seg.get("p0")),
        _sig_point(seg.get("p1")),
        _round_sig_scalar(seg.get("lookahead_px"), 3),
        _round_sig_scalar(seg.get("pose_heading"), 3),
        _round_sig_scalar(seg.get("pose_lead_in"), 3),
        _round_sig_scalar(seg.get("facing"), 3),
        tuple(pts),
        tuple(speeds),
        _round_sig_scalar(meta.get("total_px"), 3),
    )

def _path_speed_override_cmd(path_data, key):
    """Handle path speed override cmd."""
    if not path_data:
        return None
    val = path_data.get(key)
    if val is None:
        val = path_data.get(key.replace("_cmd", "_ips"))
    return val

def _path_speed_cmd(path_data, cfg, key, default):
    """Handle path speed cmd."""
    val = _path_speed_override_cmd(path_data, key)
    if val is None:
        pcfg = cfg.get("path_config", {})
        val = pcfg.get(key)
        if val is None:
            val = pcfg.get(key.replace("_cmd", "_ips"))
    return default if val is None else val

def _node_lateral_cmd(node):
    """Handle node lateral cmd."""
    val = node.get("custom_lateral_cmd")
    if val is None:
        val = node.get("custom_lateral_ips")
    return val


def _node_lateral_reset(node):
    """Return whether this node explicitly clears any carried latspeed override."""
    return bool(node.get("custom_lateral_reset", False))

def _clamp_cmd(val, default=0.0):
    """Handle clamp cmd."""
    try:
        return max(0.0, min(127.0, float(val)))
    except Exception:
        return float(default)

def _normalize_cmd_range(min_val, max_val):
    """Handle normalize cmd range."""
    min_cmd = _clamp_cmd(min_val)
    max_cmd = _clamp_cmd(max_val, default=127.0)
    if min_cmd > max_cmd:
        min_cmd, max_cmd = max_cmd, min_cmd
    return min_cmd, max_cmd

def _apply_profile_override_for_speed_overrides(node):
    """Handle apply profile override for speed overrides."""
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
    """Handle normalize speed units."""
    for node in nodes:
        if node.get("custom_lateral_cmd") is None and node.get("custom_lateral_ips") is not None:
            node["custom_lateral_cmd"] = node.pop("custom_lateral_ips")
        else:
            node.pop("custom_lateral_ips", None)
        if node.get("custom_lateral_cmd") is not None:
            node.pop("custom_lateral_reset", None)
        elif not bool(node.get("custom_lateral_reset", False)):
            node.pop("custom_lateral_reset", None)
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
    cps[0] = effective_node_pos(node_idx)
    cps[-1] = effective_node_pos(node_idx + 1)
    if not PATH_FEATURES_AVAILABLE:
        return [("plain", cps)]
    try:
        spline_type = _path_spline_type(pd)
        smooth_path = generate_bezier_path(cps, num_samples=50, spline_type=spline_type)
        if len(smooth_path) < 2:
            return None
        min_override = _path_speed_override_cmd(pd, "min_speed_cmd")
        max_override = _path_speed_override_cmd(pd, "max_speed_cmd")
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
        pd = display_nodes[i].get("path_to_next", {})
        if not pd.get("use_path", False) and (i + 1) < len(display_nodes):
            display_nodes[i + 1].pop("offset_ghost_angle", None)
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
    tl = compile_timeline(display_nodes, CFG, initial_state["heading"], fps)
    _retime_dynamic_motion_segments(tl, cfg_sig=_estimate_cfg_sig())
    tl = apply_tbuffer(CFG, tl)
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

def _path_length_in(path_points):
    """Return polyline length in inches."""
    if not path_points or len(path_points) < 2:
        return 0.0
    total_px = 0.0
    for idx in range(len(path_points) - 1):
        dx = path_points[idx + 1][0] - path_points[idx][0]
        dy = path_points[idx + 1][1] - path_points[idx][1]
        total_px += math.hypot(dx, dy)
    return total_px / PPI

def _closest_path_progress_px(path_points, pos):
    """Project a pose onto a path and return (distance_px, seg_idx, point)."""
    if not path_points:
        return 0.0, 0, (0.0, 0.0)
    best_d2 = float("inf")
    best_s = 0.0
    best_seg = 0
    best_pt = path_points[0]
    accum = 0.0
    for seg_idx in range(len(path_points) - 1):
        a = path_points[seg_idx]
        b = path_points[seg_idx + 1]
        abx = b[0] - a[0]
        aby = b[1] - a[1]
        seg_len = math.hypot(abx, aby)
        if seg_len <= 1e-9:
            continue
        apx = pos[0] - a[0]
        apy = pos[1] - a[1]
        t_proj = max(0.0, min(1.0, (apx * abx + apy * aby) / max(1e-9, seg_len * seg_len)))
        proj = (a[0] + abx * t_proj, a[1] + aby * t_proj)
        d2 = (proj[0] - pos[0]) ** 2 + (proj[1] - pos[1]) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = accum + seg_len * t_proj
            best_seg = seg_idx
            best_pt = proj
        accum += seg_len
    return best_s, best_seg, best_pt

def _speed_at_path_progress_ips(seg, path_points, progress_px):
    """Look up the commanded path speed at the current path progress."""
    speeds = seg.get("path_speeds")
    if not path_points or not speeds or len(speeds) != len(path_points):
        speed_ips = seg.get("drive_speed_ips")
        if speed_ips is not None:
            try:
                return float(speed_ips)
            except Exception:
                pass
        try:
            cfg_cmd = float(CFG.get("path_config", {}).get("max_speed_cmd", CFG.get("path_config", {}).get("max_speed_ips", 127.0)))
        except Exception:
            cfg_cmd = 127.0
        cfg_cmd = max(0.0, min(127.0, cfg_cmd))
        return vmax_straight(CFG) * (cfg_cmd / 127.0)

    total_px = 0.0
    try:
        total_px = float(seg.get("path_meta", {}).get("total_px", 0.0))
    except Exception:
        total_px = 0.0
    if total_px <= 1e-9:
        total_px = max(1e-9, _path_length_in(path_points) * PPI)

    ratio = max(0.0, min(1.0, progress_px / total_px))
    idx_f = ratio * max(0, len(speeds) - 1)
    idx0 = int(math.floor(idx_f))
    idx1 = min(len(speeds) - 1, idx0 + 1)
    frac = idx_f - idx0
    try:
        v0 = float(speeds[idx0])
        v1 = float(speeds[idx1])
    except Exception:
        v0 = v1 = 0.0
    return max(0.0, v0 + (v1 - v0) * frac)

def _segment_wait_distance_in(seg, path_points=None):
    """Return the distance basis used for marker waitUntil progress."""
    st = str(seg.get("type", "")).strip().lower()
    if st == "move":
        try:
            length_in = float(seg.get("length_in", 0.0))
        except Exception:
            length_in = 0.0
        if length_in > 1e-9:
            return length_in
        p0 = seg.get("p0", (0.0, 0.0))
        p1 = seg.get("p1", p0)
        return math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / PPI
    if st in ("path", "path_follow"):
        try:
            path_len_in = float(seg.get("path_length_in", 0.0))
        except Exception:
            path_len_in = 0.0
        if path_len_in > 1e-9:
            return path_len_in
        pts = path_points if path_points is not None else seg.get("path_points", [])
        return _path_length_in(pts)
    return 0.0

def _advance_path_segment(seg, pos, heading, cfg, dt_s, state=None, use_pursuit=True):
    """Run one path-following step. The estimate path and the playback path both use this."""
    state = dict(state or {})
    elapsed_s = max(0.0, float(state.get("elapsed_s", 0.0)))
    travel_in = max(0.0, float(state.get("travel_in", 0.0)))
    lookahead_px = max(1.0, float(seg.get("lookahead_px", path_lookahead_px)))
    reverse = bool(seg.get("reverse", False))
    move_to_pose = bool(seg.get("move_to_pose", False))

    if move_to_pose:
        pose_heading = float(seg.get("pose_heading", seg.get("facing", heading)))
        pose_lead = seg.get("pose_lead_in")
        path_points, _ = boomerang_curve_points(
            pos,
            seg.get("p1", pos),
            pose_heading,
            lead_in=pose_lead,
            reverse=reverse,
            num=max(24, int(max(1.0, float(seg.get("path_length_in", 0.0) or 0.0)) * 4.0)),
        )
        path_meta = None
        path_speeds = None
    else:
        path_points = seg.get("path_points", []) or []
        path_meta = seg.get("path_meta")
        path_speeds = seg.get("path_speeds")

    if len(path_points) < 2:
        state.update({
            "elapsed_s": elapsed_s + max(0.0, dt_s),
            "travel_in": travel_in,
            "progress_frac": 1.0,
            "done": True,
            "lookahead_point": None,
            "lookahead_radius": 0.0,
            "heading_target": None,
        })
        return seg.get("p1", pos), float(seg.get("facing", heading)) % 360.0, state

    total_wait_in = max(1e-6, _segment_wait_distance_in(seg, path_points))
    total_path_px = max(1e-6, _path_length_in(path_points) * PPI)
    next_pos = pos
    next_heading = heading
    look_pt = None
    heading_target = None

    if use_pursuit:
        progress_px, _seg_idx, _proj = _closest_path_progress_px(path_points, pos)
        _, heading_to_look, look_pt = sample_path_position(
            elapsed_s,
            path_points,
            max(1e-6, float(seg.get("T", 0.0))),
            cfg,
            lookahead_px=lookahead_px,
            use_pursuit=True,
            path_speeds=path_speeds,
            current_pos=pos,
            path_meta=path_meta,
        )
        if look_pt is None:
            look_pt = seg.get("p1", path_points[-1])
        target_heading = heading_to_look
        if move_to_pose:
            target_pos = seg.get("p1", path_points[-1])
            dist_end = math.hypot(target_pos[0] - pos[0], target_pos[1] - pos[1])
            if dist_end <= max(lookahead_px * 0.8, PPI * 1.5):
                target_heading = float(seg.get("pose_heading", seg.get("facing", heading)))
            if reverse:
                target_heading = (target_heading + 180.0) % 360.0
        else:
            dist_end = math.hypot(path_points[-1][0] - pos[0], path_points[-1][1] - pos[1])
            if dist_end <= max(lookahead_px * 0.8, PPI * 1.5):
                target_heading = calculate_path_heading(path_points, len(path_points) - 1)
            if reverse:
                target_heading = (target_heading + 180.0) % 360.0

        prev_target = state.get("heading_target")
        heading_target = smooth_angle(prev_target if prev_target is not None else heading, target_heading, 0.25)
        dx = look_pt[0] - pos[0]
        dy = look_pt[1] - pos[1]
        dist_px = math.hypot(dx, dy)
        speed_ips = _speed_at_path_progress_ips(seg, path_points, progress_px)
        step_px = min(dist_px, max(0.0, speed_ips) * PPI * max(0.0, dt_s))
        if dist_px > 1e-6 and step_px > 0.0:
            next_pos = (pos[0] + dx / dist_px * step_px, pos[1] + dy / dist_px * step_px)
        next_heading = clamp_heading_rate(heading, heading_target, cfg, dt=max(1e-6, dt_s))
    else:
        next_elapsed = min(max(0.0, float(seg.get("T", 0.0))), elapsed_s + max(0.0, dt_s))
        next_pos, heading_sample, _ = sample_path_position(
            next_elapsed,
            path_points,
            max(1e-6, float(seg.get("T", 0.0))),
            cfg,
            lookahead_px=lookahead_px,
            use_pursuit=False,
            path_speeds=path_speeds,
            path_meta=path_meta,
        )
        if reverse:
            heading_sample = (heading_sample + 180.0) % 360.0
        next_heading = clamp_heading_rate(heading, heading_sample, cfg, dt=max(1e-6, dt_s))
        elapsed_s = next_elapsed - max(0.0, dt_s)

    step_in = math.hypot(next_pos[0] - pos[0], next_pos[1] - pos[1]) / PPI
    travel_in += step_in
    elapsed_s += max(0.0, dt_s)

    target_heading = float(seg.get("pose_heading", seg.get("facing", next_heading))) if move_to_pose else calculate_path_heading(path_points, len(path_points) - 1)
    if reverse:
        target_heading = (target_heading + 180.0) % 360.0
    heading_err = abs(((target_heading - next_heading + 180.0) % 360.0) - 180.0)
    dist_end_px = math.hypot(path_points[-1][0] - next_pos[0], path_points[-1][1] - next_pos[1])
    progress_frac = max(0.0, min(1.0, travel_in / total_wait_in))
    done = progress_frac >= 0.999 and dist_end_px <= max(PPI * 0.5, lookahead_px * 0.2)
    if move_to_pose:
        done = done and heading_err <= 4.0
    elif not use_pursuit and elapsed_s >= max(0.0, float(seg.get("T", 0.0))) - 1e-9:
        done = True

    state.update({
        "elapsed_s": elapsed_s,
        "travel_in": travel_in,
        "progress_frac": progress_frac,
        "done": bool(done),
        "lookahead_point": look_pt if use_pursuit else None,
        "lookahead_radius": lookahead_px if use_pursuit else 0.0,
        "heading_target": heading_target if use_pursuit else None,
    })
    return next_pos, next_heading, state

def _estimate_path_segment_duration(seg, cfg, cfg_sig=None):
    """Estimate the duration of a pursuit-driven segment by running the same stepper offline."""
    pos = seg.get("p0", seg.get("path_points", [(0.0, 0.0)])[0] if seg.get("path_points") else (0.0, 0.0))
    path_points = seg.get("path_points", []) or []
    if len(path_points) >= 2:
        heading = calculate_path_heading(path_points, 0)
        if bool(seg.get("reverse", False)):
            heading = (heading + 180.0) % 360.0
    else:
        heading = float(seg.get("start_heading", seg.get("facing", seg.get("pose_heading", 0.0))))
    use_pursuit = bool(seg.get("move_to_pose")) or bool(path_lookahead_enabled)
    if not use_pursuit:
        return None
    cache_key = _path_duration_cache_key(seg, cfg_sig or _estimate_cfg_sig(), use_pursuit)
    cached = _path_duration_cache.get(cache_key)
    if cached is not None:
        return cached
    state = {}
    elapsed_s = 0.0
    base_T = max(0.0, float(seg.get("T", 0.0)))
    deadline_s = max(1.0, base_T * 2.5 + 1.0)
    dt_s = 1.0 / max(1.0, float(fps))
    while elapsed_s < deadline_s:
        pos, heading, state = _advance_path_segment(seg, pos, heading, cfg, dt_s, state=state, use_pursuit=use_pursuit)
        elapsed_s = float(state.get("elapsed_s", elapsed_s + dt_s))
        if state.get("done"):
            result = max(dt_s, elapsed_s)
            _cache_store(_path_duration_cache, cache_key, result, max_size=384)
            return result
    result = max(dt_s, base_T)
    _cache_store(_path_duration_cache, cache_key, result, max_size=384)
    return result

def _retime_dynamic_motion_segments(timeline_list, cfg_sig=None):
    """Replace stale path timings with the same pursuit model used during playback."""
    cfg_sig = cfg_sig or _estimate_cfg_sig()
    for seg in timeline_list:
        st = str(seg.get("type", "")).strip().lower()
        if st not in ("path", "path_follow"):
            continue
        est_s = _estimate_path_segment_duration(seg, CFG, cfg_sig=cfg_sig)
        if est_s is None:
            continue
        try:
            seg["T"] = max(0.0, float(est_s))
        except Exception:
            pass

def _attlib_sim_should_run() -> bool:
    """Whether SPACE simulation should run via attlib_sim."""
    if _attlib_sim_source_mode() == "external_script":
        return True
    style = str(CFG.get("codegen", {}).get("style", "")).strip()
    return bool(attlib_sim_visual_run) and style in ("Atticus", "AttLib")

def _attlib_sim_source_mode() -> str:
    """Return normalized AttLibSim source mode."""
    mode = str(attlib_sim_source or "timeline").strip().lower()
    if mode in ("external", "external_script", "script"):
        return "external_script"
    return "timeline"

def _attlib_sim_mode_reason() -> str:
    """Human-readable reason for current SPACE simulation mode."""
    if _attlib_sim_source_mode() == "external_script":
        return (
            "AttLibSim external routine mode enabled: "
            f"{attlib_sim_routine_script}::{attlib_sim_routine_func}"
        )
    style = str(CFG.get("codegen", {}).get("style", "")).strip()
    if style not in ("Atticus", "AttLib"):
        return f"AttLibSim disabled: output style is '{style}', expected 'Atticus' or legacy 'AttLib'."
    if not bool(attlib_sim_visual_run):
        return "AttLibSim disabled: visual mode toggle is off in General settings."
    return "AttLibSim timeline mode enabled."

def _attlib_sim_load_external_callable():
    """Load external routine callable from configured script path."""
    script_path = Path(str(attlib_sim_routine_script or "").strip())
    if not script_path.exists():
        raise FileNotFoundError(f"Routine script not found: {script_path}")

    module_name = f"_attlibsim_ext_{abs(hash((str(script_path), script_path.stat().st_mtime_ns)))}"
    spec = importlib.util.spec_from_file_location(module_name, str(script_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load script module: {script_path}")
    script_dir = str(script_path.parent)
    if script_dir and script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    ext_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ext_mod)
    fn_name = str(attlib_sim_routine_func or "run_routine").strip() or "run_routine"
    runner = getattr(ext_mod, fn_name, None)
    if runner is None or not callable(runner):
        raise AttributeError(f"Callable '{fn_name}' not found in {script_path}")
    return runner

def _attlib_sim_external_worker(runner, chassis, module):
    """Run external routine function in a side thread and capture terminal state."""
    result = None
    error = None
    try:
        try:
            result = runner(chassis, module)
        except TypeError:
            result = runner(chassis)
    except Exception:
        error = traceback.format_exc(limit=8)
    attlib_sim_runtime["external_done"] = True
    attlib_sim_runtime["external_error"] = error
    attlib_sim_runtime["external_result"] = result

class _AttlibSimChassisProxy:
    """
    Proxy that preserves default synchronous call style while keeping UI responsive.

    Motion methods are sent as async=True to attlib_sim, then locally waited by polling
    `is_in_motion()` with short sleeps so pygame/Tk can continue rendering pose updates.
    """

    def __init__(self, chassis):
        self._ch = chassis

    def __getattr__(self, name):
        return getattr(self._ch, name)

    def _wait_for_motion_completion(self, start_timeout_s=0.25):
        """
        Wait for a motion to start, then wait for it to finish.

        This avoids returning too early when async commands are queued but not yet active.
        """
        started = False
        start_deadline = time.monotonic() + max(0.02, float(start_timeout_s))
        while time.monotonic() < start_deadline:
            try:
                if self._ch.is_in_motion():
                    started = True
                    break
            except Exception:
                return
            time.sleep(0.005)

        if not started:
            return

        while True:
            try:
                if not self._ch.is_in_motion():
                    return
            except Exception:
                return
            time.sleep(0.005)

    def _wait_for_motion_ack(self, start_timeout_s=0.15):
        """Wait until a motion enters active state once (ack behavior)."""
        deadline = time.monotonic() + max(0.02, float(start_timeout_s))
        while time.monotonic() < deadline:
            try:
                if self._ch.is_in_motion():
                    return
            except Exception:
                return
            time.sleep(0.005)

    @staticmethod
    def _read_async(default_value, kwargs):
        if "async" in kwargs:
            return bool(kwargs.pop("async"))
        if "async_" in kwargs:
            return bool(kwargs.pop("async_"))
        return bool(default_value)

    def _read_params_and_async(self, rest, kwargs, async_default=False):
        """Parse the common (params?, async?) motion-call suffix used by host scripts."""
        async_req = self._read_async(async_default, kwargs)
        params = None
        if len(rest) == 1:
            if isinstance(rest[0], bool):
                async_req = bool(rest[0])
            else:
                params = rest[0]
        elif len(rest) >= 2:
            params = rest[0]
            async_req = bool(rest[1])
        return params, async_req

    def turn_to_heading(self, theta_deg, timeout_ms, *rest, async_=False, **kwargs):
        _params, async_req = self._read_params_and_async(rest, kwargs, async_)
        self._ch.turn_to_heading(theta_deg, timeout_ms, True)
        if not async_req:
            self._wait_for_motion_completion()

    def turn_to_point(self, x, y, timeout_ms, *rest, async_=False, **kwargs):
        _params, async_req = self._read_params_and_async(rest, kwargs, async_)
        self._ch.turn_to_point(x, y, timeout_ms, True)
        if not async_req:
            self._wait_for_motion_completion()

    def move_to_point(self, x, y, timeout_ms, *rest, async_=False, **kwargs):
        params, async_req = self._read_params_and_async(rest, kwargs, async_)
        if params is None:
            self._ch.move_to_point(x, y, timeout_ms, True)
        else:
            self._ch.move_to_point(x, y, timeout_ms, params, True)
        if not async_req:
            self._wait_for_motion_completion()

    def swing_to_heading(self, theta_deg, locked_side, timeout_ms, *rest, async_=False, **kwargs):
        _params, async_req = self._read_params_and_async(rest, kwargs, async_)
        self._ch.swing_to_heading(theta_deg, locked_side, timeout_ms, True)
        if not async_req:
            self._wait_for_motion_completion()

    def swing_to_point(self, x, y, locked_side, timeout_ms, *rest, async_=False, **kwargs):
        _params, async_req = self._read_params_and_async(rest, kwargs, async_)
        self._ch.swing_to_point(x, y, locked_side, timeout_ms, True)
        if not async_req:
            self._wait_for_motion_completion()

    def move_to_pose(self, x, y, theta_deg, timeout_ms, *rest, **kwargs):
        params, async_req = self._read_params_and_async(rest, kwargs, False)
        if params is None:
            self._ch.move_to_pose(x, y, theta_deg, timeout_ms, True)
        else:
            self._ch.move_to_pose(x, y, theta_deg, timeout_ms, params, True)
        if not async_req:
            self._wait_for_motion_completion()

    def follow(self, path, lookahead_in, timeout_ms, forwards=True, async_=False, **kwargs):
        async_req = self._read_async(async_, kwargs)
        self._ch.follow(path, lookahead_in, timeout_ms, forwards, True)
        if not async_req:
            self._wait_for_motion_completion()

    def tank127(self, left127, right127, async_=False, **kwargs):
        async_req = self._read_async(async_, kwargs)
        self._ch.tank127(left127, right127, True)
        if not async_req:
            self._wait_for_motion_ack()

    def tank127_for(self, left127, right127, duration_ms, async_=False, **kwargs):
        async_req = self._read_async(async_, kwargs)
        self._ch.tank127_for(left127, right127, duration_ms, True)
        if not async_req:
            self._wait_for_motion_completion()

    def wait_until_done(self):
        self._wait_for_motion_completion(start_timeout_s=0.4)

    def wait_until(self, dist_or_deg):
        self._ch.wait_until(float(dist_or_deg))

def _attlib_sim_px_to_in(pos):
    """Convert terminal pixel coordinates to Atticus field coordinates (x forward, y left)."""
    if int(CFG.get("field_centric", 1)) == 1:
        cx, cy = WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0
    else:
        cx, cy = initial_state["position"]
    x_forward_in = -(float(pos[1]) - cy) / PPI
    y_left_in = -(float(pos[0]) - cx) / PPI
    return (x_forward_in, y_left_in)

def _attlib_sim_in_to_px(x_in: float, y_in: float):
    """Convert Atticus field coordinates (x forward, y left) back to terminal pixels."""
    if int(CFG.get("field_centric", 1)) == 1:
        cx, cy = WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0
    else:
        cx, cy = initial_state["position"]
    px = cx - float(y_in) * PPI
    py = cy - float(x_in) * PPI
    return (px, py)

def _attlib_sim_timeout_ms(seg, pad_s: float = 0.4) -> int:
    """Create a safe timeout from timeline segment duration."""
    try:
        t_s = float(seg.get("T", 0.0))
    except Exception:
        t_s = 0.0
    return max(120, int((max(0.0, t_s) + max(0.0, pad_s)) * 1000.0))

def _attlib_sim_log(message: str):
    """Append an AttLibSim runtime line into the output panel."""
    if not message:
        return
    if not log_lines:
        log_lines.extend(build_compile_header(CFG, initial_state["heading"]))
    line = f"[AttLibSim] {message}"
    if not log_lines or log_lines[-1] != line:
        log_lines.append(line)
    ui.output_refresh(log_lines)

def _attlib_motion_profile(module, profile_name):
    """Map terminal profile names onto host MotionProfile enums."""
    motion_profile = getattr(module, "MotionProfile", None)
    if motion_profile is None:
        return None
    key = str(profile_name or "").strip().lower()
    attr = {
        "legacy": "LEGACY",
        "precise": "PRECISE",
        "fast": "FAST",
    }.get(key, "NORMAL")
    return getattr(motion_profile, attr, getattr(motion_profile, "NORMAL", None))

def _attlib_drive_params(module, seg):
    """Build host move params from a compiled terminal segment."""
    params = module.MoveToPoseParams() if bool(seg.get("move_to_pose")) else module.MoveToPointParams()
    profile_name = seg.get("profile_override") or pick_profile("drive", abs(float(seg.get("path_length_in", seg.get("length_in", 0.0)) or 0.0)), cfg=CFG)
    profile = _attlib_motion_profile(module, profile_name)
    if profile is not None:
        params.profile = profile
    params.forwards = not bool(seg.get("reverse", False))
    try:
        params.max_speed = float(seg.get("drive_speed_cmd", 127.0) if seg.get("drive_speed_cmd") is not None else 127.0)
    except Exception:
        params.max_speed = 127.0
    if bool(seg.get("chain_to_next")):
        try:
            params.min_speed = float(seg.get("chain_min_speed", 0.0) or 0.0)
        except Exception:
            params.min_speed = 0.0
        try:
            params.early_exit_range = float(seg.get("chain_early_exit", 0.0) or 0.0)
        except Exception:
            params.early_exit_range = 0.0
        params.allow_chaining = True
    else:
        params.min_speed = 0.0
        params.early_exit_range = 0.0
        params.allow_chaining = False
    if bool(seg.get("move_to_pose")):
        lead_in = seg.get("pose_lead_in")
        if lead_in is not None:
            try:
                params.lead = float(lead_in)
                params.auto_lead = False
            except Exception:
                pass
        else:
            params.auto_lead = True
    return params

def _attlib_marker_events(seg):
    """Convert edge markers into waitUntil-style progress targets for host playback."""
    raw_events = seg.get("edge_events", [])
    if not isinstance(raw_events, list) or not raw_events:
        return []
    total_in = max(0.0, _segment_wait_distance_in(seg))
    events = []
    for event in raw_events:
        if not isinstance(event, dict) or not event.get("enabled", True):
            continue
        try:
            t_val = float(event.get("t", 0.0))
        except Exception:
            t_val = 0.0
        t_val = max(0.0, min(1.0, t_val))
        events.append({
            "progress_in": total_in * t_val,
            "actions": list(event.get("actions", [])),
        })
    events.sort(key=lambda item: item.get("progress_in", 0.0))
    return events

def _attlib_sim_build_plan(tl, module):
    """Translate terminal timeline segments into attlib_sim command plan."""
    plan = []
    for seg in tl:
        seg_type = str(seg.get("type", "")).strip().lower()
        if seg_type == "move":
            x_in, y_in = _attlib_sim_px_to_in(seg.get("p1", (0.0, 0.0)))
            plan.append({
                "kind": "move_to_point",
                "x_in": x_in,
                "y_in": y_in,
                "timeout_ms": _attlib_sim_timeout_ms(seg),
                "params": _attlib_drive_params(module, seg),
                "marker_events": _attlib_marker_events(seg),
                "progress_mode": "linear_in",
                "started": False,
                "ticks_since_start": 0,
            })
        elif seg_type == "turn":
            plan.append({
                "kind": "turn_to_heading",
                "heading_deg": float(seg.get("target_heading", 0.0)),
                "timeout_ms": _attlib_sim_timeout_ms(seg),
                "progress_mode": "angle_deg",
                "started": False,
                "ticks_since_start": 0,
            })
        elif seg_type == "swing":
            swing_dir = str(seg.get("swing_dir", "auto")).strip().lower()
            if swing_dir == "cw":
                locked_side = "RIGHT"
            elif swing_dir == "ccw":
                locked_side = "LEFT"
            else:
                start_h = float(seg.get("start_heading", 0.0))
                target_h = float(seg.get("target_heading", 0.0))
                delta = ((target_h - start_h + 180.0) % 360.0) - 180.0
                locked_side = "LEFT" if delta >= 0.0 else "RIGHT"
            plan.append({
                "kind": "swing_to_heading",
                "heading_deg": float(seg.get("target_heading", 0.0)),
                "locked_side": locked_side,
                "timeout_ms": _attlib_sim_timeout_ms(seg),
                "progress_mode": "angle_deg",
                "started": False,
                "ticks_since_start": 0,
            })
        elif seg_type == "path":
            if bool(seg.get("move_to_pose", False)):
                x_in, y_in = _attlib_sim_px_to_in(seg.get("p1", (0.0, 0.0)))
                heading_deg = float(seg.get("pose_heading", seg.get("facing", 0.0)))
                plan.append({
                    "kind": "move_to_pose",
                    "x_in": x_in,
                    "y_in": y_in,
                    "heading_deg": heading_deg,
                    "timeout_ms": _attlib_sim_timeout_ms(seg, pad_s=0.7),
                    "params": _attlib_drive_params(module, seg),
                    "marker_events": _attlib_marker_events(seg),
                    "progress_mode": "linear_in",
                    "started": False,
                    "ticks_since_start": 0,
                })
            else:
                path_points = seg.get("path_points", []) or []
                if len(path_points) >= 2:
                    path_obj = module.Path()
                    pts = []
                    s_in = []
                    running_in = 0.0
                    for point in path_points:
                        path_point = module.PathPoint()
                        path_point.x_in, path_point.y_in = _attlib_sim_px_to_in(point)
                        pts.append(path_point)
                    for point_idx in range(len(path_points)):
                        if point_idx > 0:
                            dx = path_points[point_idx][0] - path_points[point_idx - 1][0]
                            dy = path_points[point_idx][1] - path_points[point_idx - 1][1]
                            running_in += math.hypot(dx, dy) / PPI
                        s_in.append(running_in)
                    path_obj.pts = pts
                    path_obj.s_in = s_in
                    lookahead_in = max(1.0, float(seg.get("lookahead_px", path_lookahead_px)) / PPI)
                    plan.append({
                        "kind": "follow",
                        "path_obj": path_obj,
                        "lookahead_in": lookahead_in,
                        "timeout_ms": _attlib_sim_timeout_ms(seg, pad_s=1.0),
                        "forwards": not bool(seg.get("reverse", False)),
                        "marker_events": _attlib_marker_events(seg),
                        "progress_mode": "linear_in",
                        "started": False,
                        "ticks_since_start": 0,
                    })
                else:
                    x_in, y_in = _attlib_sim_px_to_in(seg.get("p1", (0.0, 0.0)))
                    plan.append({
                        "kind": "move_to_point",
                        "x_in": x_in,
                        "y_in": y_in,
                        "timeout_ms": _attlib_sim_timeout_ms(seg),
                        "params": _attlib_drive_params(module, seg),
                        "marker_events": _attlib_marker_events(seg),
                        "progress_mode": "linear_in",
                        "started": False,
                        "ticks_since_start": 0,
                    })
        elif seg_type == "wait":
            if str(seg.get("role", "")).strip().lower() == "buffer":
                continue
            try:
                wait_s = float(seg.get("T", 0.0))
            except Exception:
                wait_s = 0.0
            if wait_s > 0.0:
                plan.append({"kind": "wait", "seconds": wait_s})
        elif seg_type == "reshape":
            plan.append({"kind": "reshape_toggle"})
        elif seg_type == "marker":
            plan.append({"kind": "marker", "actions": list(seg.get("actions", []))})
    return plan

def _attlib_sim_stop():
    """Stop and release current attlib_sim runtime resources."""
    global attlib_sim_runtime
    worker = attlib_sim_runtime.get("external_thread")
    chassis = attlib_sim_runtime.get("chassis")
    if chassis is not None:
        try:
            chassis.cancel_all_motions()
        except Exception:
            pass
        try:
            chassis.stop()
        except Exception:
            pass
    if worker is not None:
        try:
            worker.join(timeout=0.25)
        except Exception:
            pass
    attlib_sim_runtime = {
        "running": False,
        "module": None,
        "chassis": None,
        "mode": "timeline",
        "plan": [],
        "index": 0,
        "wait_elapsed_s": 0.0,
        "external_thread": None,
        "external_done": False,
        "external_error": None,
        "external_result": None,
    }

def _attlib_sim_start(tl) -> bool:
    """Initialize attlib_sim visual runner for the current timeline."""
    global attlib_sim_runtime, attlib_sim_status

    _attlib_sim_stop()

    module_dirs = []
    preferred = str(attlib_sim_module_dir or "").strip()
    if preferred:
        module_dirs.append(Path(preferred))
    module_dirs.extend([
        Path(_default_attlib_sim_module_dir()),
        Path(APP_ROOT).resolve().parent / "Project Atticus" / "atticode" / "host" / "build",
    ])

    ext_suffixes = tuple(importlib.machinery.EXTENSION_SUFFIXES or ())
    module_dir = None
    found_any = []
    found_incompatible = []
    for candidate in module_dirs:
        if not candidate.exists():
            continue
        binaries = sorted(candidate.glob("attlib_sim*.pyd"))
        if binaries:
            found_any.extend(str(p.name) for p in binaries)
        if any((candidate / f"attlib_sim{sfx}").exists() for sfx in ext_suffixes):
            module_dir = candidate
            break
        if binaries:
            found_incompatible.extend(str(p.name) for p in binaries)

    if module_dir is None:
        if found_incompatible:
            py_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
            attlib_sim_status = (
                f"AttLibSim binary mismatch (need {py_tag}). "
                f"Found: {', '.join(found_incompatible)}"
            )
        elif found_any:
            attlib_sim_status = f"AttLibSim module not loadable for this Python. Found: {', '.join(found_any)}"
        else:
            attlib_sim_status = "AttLibSim module not found. Build host/attlib_sim first."
        return False

    if str(module_dir) not in sys.path:
        sys.path.insert(0, str(module_dir))

    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        dll_dir = Path(r"C:\Strawberry\c\bin")
        if dll_dir.exists():
            try:
                os.add_dll_directory(str(dll_dir))
            except Exception:
                pass

    try:
        importlib.invalidate_caches()
        module = importlib.import_module("attlib_sim")
    except Exception as exc:
        attlib_sim_status = f"AttLibSim import failed: {exc}"
        return False

    try:
        sim_cfg = module.SimDriveIOConfig()
        sim_cfg.track_width_in = float(CFG.get("bot_dimensions", {}).get("dt_width", 11.5))
        sim_cfg.wheel_diameter_in = float(CFG.get("robot_physics", {}).get("diameter", 4.0))
        sim_cfg.max_wheel_speed_inps = float(vmax_straight(CFG))
        chassis = module.SimChassis(sim_cfg)
        x0_in, y0_in = _attlib_sim_px_to_in(display_nodes[0]["pos"])
        chassis.set_pose(x0_in, y0_in, float(initial_state["heading"]))
        plan = _attlib_sim_build_plan(tl, module)
    except Exception as exc:
        attlib_sim_status = f"AttLibSim setup failed: {exc}"
        return False

    mode = _attlib_sim_source_mode()
    if mode == "external_script":
        try:
            runner = _attlib_sim_load_external_callable()
        except Exception as exc:
            attlib_sim_status = f"External routine load failed: {exc}"
            return False

        script_chassis = _AttlibSimChassisProxy(chassis)

        worker = threading.Thread(
            target=_attlib_sim_external_worker,
            args=(runner, script_chassis, module),
            daemon=True,
            name="AttLibSimExternalRoutine",
        )
        attlib_sim_runtime = {
            "running": True,
            "module": module,
            "chassis": chassis,
            "mode": mode,
            "plan": [],
            "index": 0,
            "wait_elapsed_s": 0.0,
            "external_thread": worker,
            "external_done": False,
            "external_error": None,
            "external_result": None,
        }
        worker.start()
        attlib_sim_status = (
            "AttLibSim running external routine: "
            f"{attlib_sim_routine_script}::{attlib_sim_routine_func}"
        )
        _attlib_sim_log(attlib_sim_status)
        _attlib_sim_log("External routine proxy enabled: blocking calls are emulated without freezing visual updates.")
        return True

    if not plan:
        attlib_sim_status = "No executable timeline commands for AttLibSim."
        return False

    attlib_sim_runtime = {
        "running": True,
        "module": module,
        "chassis": chassis,
        "mode": mode,
        "plan": plan,
        "index": 0,
        "wait_elapsed_s": 0.0,
        "external_thread": None,
        "external_done": False,
        "external_error": None,
        "external_result": None,
    }
    attlib_sim_status = f"AttLibSim running ({len(plan)} cmds)"
    _attlib_sim_log(attlib_sim_status)
    return True

def _attlib_sim_step(dt_s):
    """Advance attlib_sim visual runtime and mirror pose into terminal view."""
    global moving, robot_pos, robot_heading, reshape_live, attlib_sim_status

    if not attlib_sim_runtime.get("running"):
        return

    chassis = attlib_sim_runtime.get("chassis")
    module = attlib_sim_runtime.get("module")
    plan = attlib_sim_runtime.get("plan", [])
    index = int(attlib_sim_runtime.get("index", 0))
    if chassis is None or module is None:
        moving = False
        _attlib_sim_stop()
        attlib_sim_status = "AttLibSim runtime missing chassis/module."
        _attlib_sim_log(attlib_sim_status)
        return

    try:
        pose = chassis.get_pose()
        robot_pos = _attlib_sim_in_to_px(float(pose.x), float(pose.y))
        robot_heading = float(pose.theta) % 360.0
    except Exception:
        pass

    if paused:
        return

    mode = str(attlib_sim_runtime.get("mode", "timeline"))
    if mode == "external_script":
        if bool(attlib_sim_runtime.get("external_done", False)):
            try:
                if chassis.is_in_motion():
                    return
            except Exception:
                pass
            moving = False
            err = attlib_sim_runtime.get("external_error")
            result = attlib_sim_runtime.get("external_result")
            _attlib_sim_stop()
            if err:
                attlib_sim_status = f"External routine failed:\n{err}"
            else:
                attlib_sim_status = f"External routine completed. result={result!r}"
            _attlib_sim_log(attlib_sim_status)
        return

    if index >= len(plan):
        moving = False
        _attlib_sim_stop()
        attlib_sim_status = "AttLibSim completed."
        _attlib_sim_log(attlib_sim_status)
        return

    cmd = plan[index]
    kind = cmd.get("kind")

    if kind == "wait":
        attlib_sim_runtime["wait_elapsed_s"] = float(attlib_sim_runtime.get("wait_elapsed_s", 0.0)) + max(0.0, float(dt_s))
        if float(attlib_sim_runtime["wait_elapsed_s"]) >= float(cmd.get("seconds", 0.0)):
            attlib_sim_runtime["index"] = index + 1
            attlib_sim_runtime["wait_elapsed_s"] = 0.0
        return

    if kind == "reshape_toggle":
        reshape_live = not reshape_live
        attlib_sim_runtime["index"] = index + 1
        return

    if kind == "marker":
        reshape_live = _marker_apply_reshape(cmd.get("actions", []), reshape_live)
        _marker_apply_atticus(cmd.get("actions", []))
        attlib_sim_runtime["index"] = index + 1
        return

    if not cmd.get("started", False):
        try:
            if kind == "move_to_point":
                params = cmd.get("params")
                if params is None:
                    chassis.move_to_point(float(cmd["x_in"]), float(cmd["y_in"]), int(cmd["timeout_ms"]), True)
                else:
                    chassis.move_to_point(float(cmd["x_in"]), float(cmd["y_in"]), int(cmd["timeout_ms"]), params, True)
            elif kind == "turn_to_heading":
                chassis.turn_to_heading(float(cmd["heading_deg"]), int(cmd["timeout_ms"]), True)
            elif kind == "swing_to_heading":
                locked_side = module.DriveSide.RIGHT if cmd.get("locked_side") == "RIGHT" else module.DriveSide.LEFT
                chassis.swing_to_heading(float(cmd["heading_deg"]), locked_side, int(cmd["timeout_ms"]), True)
            elif kind == "move_to_pose":
                params = cmd.get("params")
                if params is None:
                    chassis.move_to_pose(
                        float(cmd["x_in"]),
                        float(cmd["y_in"]),
                        float(cmd["heading_deg"]),
                        int(cmd["timeout_ms"]),
                        True,
                    )
                else:
                    chassis.move_to_pose(
                        float(cmd["x_in"]),
                        float(cmd["y_in"]),
                        float(cmd["heading_deg"]),
                        int(cmd["timeout_ms"]),
                        params,
                        True,
                    )
            elif kind == "follow":
                chassis.follow(
                    cmd["path_obj"],
                    float(cmd["lookahead_in"]),
                    int(cmd["timeout_ms"]),
                    bool(cmd.get("forwards", True)),
                    True,
                )
            else:
                attlib_sim_runtime["index"] = index + 1
                return
            cmd["started"] = True
            cmd["ticks_since_start"] = 0
            cmd["progress"] = 0.0
            cmd["marker_index"] = 0
            try:
                pose = chassis.get_pose()
                cmd["last_pose_xy"] = (float(pose.x), float(pose.y))
                cmd["last_heading_deg"] = float(pose.theta) % 360.0
            except Exception:
                cmd["last_pose_xy"] = None
                cmd["last_heading_deg"] = None
        except Exception as exc:
            moving = False
            _attlib_sim_stop()
            attlib_sim_status = f"AttLibSim command failed ({kind}): {exc}"
            _attlib_sim_log(attlib_sim_status)
            return

    cmd["ticks_since_start"] = int(cmd.get("ticks_since_start", 0)) + 1
    if cmd["ticks_since_start"] < 2:
        return

    try:
        pose = chassis.get_pose()
        current_xy = (float(pose.x), float(pose.y))
        current_heading = float(pose.theta) % 360.0
        progress_mode = str(cmd.get("progress_mode", "")).strip().lower()
        if progress_mode == "linear_in":
            prev_xy = cmd.get("last_pose_xy")
            if isinstance(prev_xy, tuple) and len(prev_xy) == 2:
                cmd["progress"] = float(cmd.get("progress", 0.0)) + math.hypot(current_xy[0] - prev_xy[0], current_xy[1] - prev_xy[1])
            cmd["last_pose_xy"] = current_xy
        elif progress_mode == "angle_deg":
            prev_heading = cmd.get("last_heading_deg")
            if prev_heading is not None:
                delta = ((current_heading - float(prev_heading) + 180.0) % 360.0) - 180.0
                cmd["progress"] = float(cmd.get("progress", 0.0)) + abs(delta)
            cmd["last_heading_deg"] = current_heading

        events = cmd.get("marker_events", [])
        marker_idx = int(cmd.get("marker_index", 0))
        progress_val = float(cmd.get("progress", 0.0))
        while marker_idx < len(events):
            target = float(events[marker_idx].get("progress_in", 0.0))
            if progress_val + 1e-6 < target:
                break
            actions = events[marker_idx].get("actions", [])
            reshape_live = _marker_apply_reshape(actions, reshape_live)
            _marker_apply_atticus(actions)
            marker_idx += 1
        cmd["marker_index"] = marker_idx

        if not chassis.is_in_motion():
            while marker_idx < len(events):
                actions = events[marker_idx].get("actions", [])
                reshape_live = _marker_apply_reshape(actions, reshape_live)
                _marker_apply_atticus(actions)
                marker_idx += 1
            cmd["marker_index"] = marker_idx
            attlib_sim_runtime["index"] = index + 1
            attlib_sim_runtime["wait_elapsed_s"] = 0.0
    except Exception as exc:
        moving = False
        _attlib_sim_stop()
        attlib_sim_status = f"AttLibSim poll failed: {exc}"
        _attlib_sim_log(attlib_sim_status)

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
    """Handle arrival heading for offset."""
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
                spline_type = _path_spline_type(prev_pd)
                pts = generate_bezier_path(cps, num_samples=20, spline_type=spline_type)
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
    force_pose_ghost = False
    if off_in != 0.0 and prev_node.get("move_to_pose"):
        pose_h = prev_node.get("pose_heading_deg")
        if pose_h is None:
            pose_h = heading_from_points(prev, node["pos"])
        try:
            pose_h = float(pose_h)
        except Exception:
            pose_h = heading_from_points(prev, node["pos"])
        ghost_ang = pose_h % 360.0
        node["offset_ghost_angle"] = ghost_ang
        force_pose_ghost = True
    if off_in == 0.0:
        return node["pos"]
    if force_pose_ghost:
        if ghost_ang is None:
            ghost_ang = heading_from_points(prev, node["pos"])
            node["offset_ghost_angle"] = ghost_ang
        if ghost_ang is not None:
            rad = math.radians(ghost_ang)
            return (
                node["pos"][0] - math.cos(rad) * off_in * PPI,
                node["pos"][1] + math.sin(rad) * off_in * PPI
            )
    if use_radial:
        try:
            prev_eff = display_nodes[idx - 1]["pos"] if idx - 1 >= 0 else prev
        except Exception:
            prev_eff = prev
        if not force_pose_ghost:
            arrival_heading = _arrival_heading_for_offset(prev_pd, prev_eff, node["pos"])
            if arrival_heading is not None:
                ghost_ang = arrival_heading
            elif ghost_ang is None:
                ghost_ang = heading_from_points(prev_eff, node["pos"])
            node["offset_ghost_angle"] = ghost_ang
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
        node.pop("offset_ghost_angle", None)
    ux, uy = approach_unit(prev, node["pos"])
    return (node["pos"][0] - ux * off_in * PPI, node["pos"][1] - uy * off_in * PPI)

def compute_total_estimate_s():
    """Compute total routine time estimate."""
    estimate_sig = _estimate_graph_sig()
    cached = _estimate_total_cache.get(estimate_sig)
    if cached is not None:
        return cached
    tl = build_timeline_with_buffers()
    total = compute_total_from_timeline(tl)
    _cache_store(_estimate_total_cache, estimate_sig, total, max_size=96)
    return total

def clamp_heading_rate(current_h, target_h, cfg, dt):
    """Limit heading change per frame to respect physical turn constraints."""
    max_rate = turn_rate(cfg)  # deg/s from physics
    max_step = max_rate * dt
    d = ((target_h - current_h + 180.0) % 360.0) - 180.0
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
    """Handle split edge events."""
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

def _fire_edge_events(seg, progress=None, reshape_state=False, drain_all=False):
    """Trigger marker actions when a segment crosses their waitUntil progress target."""
    events = seg.get("edge_events", [])
    if not isinstance(events, list) or not events:
        return reshape_state
    idx = int(seg.get("_edge_event_idx", 0))
    while idx < len(events):
        try:
            target = float(events[idx].get("t", 0.0))
        except Exception:
            target = 0.0
        if not drain_all:
            prog = 0.0 if progress is None else max(0.0, min(1.0, float(progress)))
            if prog + 1e-6 < target:
                break
        actions = events[idx].get("actions", [])
        label = _marker_actions_to_text(actions)
        reshape_state = _marker_apply_reshape(actions, reshape_state)
        _marker_apply_atticus(actions)
        log_action("marker", label=label)
        idx += 1
    seg["_edge_event_idx"] = idx
    return reshape_state

def _split_path_control_points(cps, insert_pt):
    """Handle split path control points."""
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
    """Handle edge polyline."""
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
        spline_type = _path_spline_type(pd)
        pts = generate_bezier_path(cps, num_samples=60, spline_type=spline_type)
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
    """Handle polyline lengths."""
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
    """Handle polyline point at."""
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
    """Handle polyline nearest."""
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
    """Handle polyline tangent."""
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
    """Handle edge marker hit."""
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
    """Handle edge marker preview."""
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
        if int(CFG.get("angle_units", 0)) == 1:
            chosen_val = chosen * (math.pi/180.0)
            to_face_val = to_face_deg * (math.pi/180.0)
            log_lines.append(f"  Swing: {chosen_val:.6f} rad dir={dir_tag} to {to_face_val:.6f} rad")
        else:
            log_lines.append(f"  Swing: {chosen:.3f}° dir={dir_tag} to {to_face_deg:.3f}°")
    elif kind == "wait":
        log_lines.append(f"  Wait: {kw['s']:.3f} s")
    elif kind == "reshape":
        log_lines.append(f"  Reshape: state {kw['state']}")
    elif kind == "reverse":
        log_lines.append(f"  Reverse: {'ON' if kw['state'] else 'OFF'}")
    elif kind == "atticus_immediate":
        log_lines.append(f"  Atticus: applyImmediateCorrectionAuto() [settled about {ATTICUS_DSR_RECOMMENDED_STILL_MS:g} ms]")
    elif kind == "atticus_wall_trim_start":
        log_lines.append("  Atticus: correctThetaFromWallAuto()")
    elif kind == "atticus_wall_trim_end":
        log_lines.append("  Atticus: endThetaWallAlignment()")
    elif kind == "atticus_rough_wall_start":
        log_lines.append("  Atticus: startRoughWallTraverseAuto()")
    elif kind == "atticus_rough_wall_end":
        log_lines.append("  Atticus: endRoughWallTraverse()")
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
    tutorial_state["flags"]["entered_path_edit"] = True

    node = display_nodes[segment_idx]

    path_data = node.get("path_to_next") or {}
    cps = None
    if (
        path_data is None
        or not path_data.get("use_path", False)
        or not path_data.get("control_points")
        or len(path_data.get("control_points", ())) < 2
    ):
        p0 = effective_node_pos(segment_idx)
        p1 = effective_node_pos(segment_idx + 1)
        mid_x = (p0[0] + p1[0]) // 2
        mid_y = (p0[1] + p1[1]) // 2

        cps = [p0, (mid_x, mid_y), p1]
        default_spline = _path_spline_type(path_data)
        node["path_to_next"] = {
            "use_path": True,
            "control_points": cps,
            "spline_type": default_spline
        }
        end_node = display_nodes[segment_idx + 1]
        if end_node.get("offset", 0) != 0 or end_node.get("offset_custom_in") is not None:
            if end_node.get("offset_ghost_angle") is None:
                if node.get("move_to_pose"):
                    pose_h = node.get("pose_heading_deg")
                    if pose_h is None:
                        pose_h = heading_from_points(p0, p1)
                    try:
                        pose_h = float(pose_h)
                    except Exception:
                        pose_h = heading_from_points(p0, p1)
                    pose_h = pose_h % 360.0
                    end_node["offset_ghost_angle"] = pose_h
                else:
                    arrival_heading = _arrival_heading_for_offset(path_data, p0, p1)
                    if arrival_heading is None:
                        arrival_heading = heading_from_points(p0, p1)
                    end_node["offset_ghost_angle"] = arrival_heading
    else:
        cps = list(path_data["control_points"])
        path_data.setdefault("spline_type", _path_spline_type(path_data))
    
    path_control_points = list(cps)
    selected_control_point = None

def exit_path_edit_mode():
    """Exit path editing mode and save changes."""
    global path_edit_mode, path_edit_segment_idx, path_control_points, selected_control_point
    global last_path_sig, last_snapshot, total_estimate_s, selected_idx
    global dragging, constrain_active, constrain_origin, dragging_control_point

    if path_edit_mode and path_edit_segment_idx is not None:
        node = display_nodes[path_edit_segment_idx]
        if path_control_points and len(path_control_points) > 2:
            cp = list(path_control_points)
            cp[0] = effective_node_pos(path_edit_segment_idx)
            cp[-1] = effective_node_pos(path_edit_segment_idx + 1)
            node.setdefault("path_to_next", {})
            node["path_to_next"]["control_points"] = cp
            node["path_to_next"]["use_path"] = True
        else:
            path_to_next = node.get("path_to_next")
            if path_to_next is not None:
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

    last_path_sig = None
    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
    total_estimate_s = compute_total_estimate_s()

def toggle_segment_curved(segment_idx):
    """Toggle whether a segment uses a curved path."""
    if segment_idx is None or segment_idx >= len(display_nodes) - 1:
        return
    
    node = display_nodes[segment_idx]
    
    if "path_to_next" in node and node["path_to_next"].get("use_path"):
        node["path_to_next"]["use_path"] = False
    else:
        enter_path_edit_mode(segment_idx)

def add_control_point_at_mouse(mouse_pos):
    """Add a control point at mouse position."""
    global path_control_points
    
    if not path_edit_mode or not path_control_points:
        return
    
    min_dist = float('inf')
    insert_idx = 1
    
    for i in range(len(path_control_points) - 1):
        p1 = path_control_points[i]
        p2 = path_control_points[i + 1]
        
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

    if point_idx <= 0 or point_idx >= len(path_control_points) - 1:
        return

    path_control_points.pop(point_idx)
    selected_control_point = None

def toggle_path_spline_type(segment_idx=None):
    """Toggle spline type between uniform and centripetal for a segment."""
    global last_path_sig, last_snapshot, total_estimate_s
    if segment_idx is None:
        segment_idx = path_edit_segment_idx
    if segment_idx is None or segment_idx >= len(display_nodes) - 1:
        return
    node = display_nodes[segment_idx]
    pd = node.setdefault("path_to_next", {})
    cur_type = _path_spline_type(pd)
    new_type = "centripetal" if cur_type == "uniform" else "uniform"
    pd["spline_type"] = new_type
    tutorial_state["flags"]["spline_toggled"] = True
    path_draw_cache.clear()
    last_path_sig = None
    last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
    total_estimate_s = compute_total_estimate_s()

def _split_command_parts(text):
    """Split command text on comma/semicolon, ignoring delimiters inside (), [], {}, and quotes."""
    if not text:
        return []
    out = []
    buf = []
    d_paren = d_brack = d_brace = 0
    in_single = False
    in_double = False
    esc = False
    for ch in str(text):
        if esc:
            buf.append(ch)
            esc = False
            continue
        if ch == "\\" and (in_single or in_double):
            buf.append(ch)
            esc = True
            continue
        if ch == "'" and not in_double:
            in_single = not in_single
            buf.append(ch)
            continue
        if ch == '"' and not in_single:
            in_double = not in_double
            buf.append(ch)
            continue
        if not in_single and not in_double:
            if ch == "(":
                d_paren += 1
            elif ch == ")":
                d_paren = max(0, d_paren - 1)
            elif ch == "[":
                d_brack += 1
            elif ch == "]":
                d_brack = max(0, d_brack - 1)
            elif ch == "{":
                d_brace += 1
            elif ch == "}":
                d_brace = max(0, d_brace - 1)
            elif ch in ",;" and d_paren == 0 and d_brack == 0 and d_brace == 0:
                part = "".join(buf).strip()
                if part:
                    out.append(part)
                buf = []
                continue
        buf.append(ch)
    part = "".join(buf).strip()
    if part:
        out.append(part)
    return out

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
        elif act.get("type") == "preset":
            name = str(act.get("name", "")).strip()
            if name:
                text = name
                state_raw = act.get("state", None)
                if isinstance(state_raw, bool):
                    state = "on" if state_raw else "off"
                elif state_raw is None:
                    state = ""
                else:
                    state = str(state_raw).strip().lower()
                value = _preset_value_text(act)
                if state:
                    text = f"{text} {state}"
                if value:
                    text = f"{text} {value}"
                parts.append(text)
        elif act.get("type") == "code":
            code = str(act.get("code", "")).strip()
            if code:
                parts.append(f"code {code}")
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
    if lat_cmd is not None:
        parts.append(f"latspeed {float(lat_cmd):g}")
    elif _node_lateral_reset(node):
        parts.append("latspeed off")
    if node.get("custom_turn_dps"):
        parts.append(f"turnspeed {node['custom_turn_dps']:g}")
    corr_mode = _node_atticus_correction_mode(node)
    if corr_mode == "immediate":
        parts.append("correct immediate")
    elif corr_mode == "wall_trim":
        parts.append("correct walltrim")
    elif corr_mode == "rough_wall":
        parts.append("correct roughwall")
    return ", ".join(parts)

def _normalize_preset_state_token(state):
    """Normalize preset toggle state to on/off/toggle/None."""
    if state is None:
        return None
    if isinstance(state, bool):
        return "on" if state else "off"
    s = str(state).strip().lower()
    if not s:
        return None
    if s in ("on", "1", "true", "yes", "enable", "enabled"):
        return "on"
    if s in ("off", "0", "false", "no", "disable", "disabled"):
        return "off"
    if s == "toggle":
        return "toggle"
    return None


def _normalize_reverse_action_state(state):
    """Normalize reverse action state to True/False/None(toggle)."""
    if state is None:
        return None
    if isinstance(state, bool):
        return state
    s = str(state).strip().lower()
    if not s or s == "toggle":
        return None
    if s in ("on", "1", "true", "yes", "enable", "enabled"):
        return True
    if s in ("off", "0", "false", "no", "disable", "disabled"):
        return False
    return None


def _looks_like_legacy_dsr_preset(name):
    """Detect old DSR preset actions that should become Atticus immediate nodes."""
    return str(name or "").strip().lower() == "dsr"


def _looks_like_legacy_dsr_code(code):
    """Detect old hard-coded DSR calls saved before node correction migration."""
    text = str(code or "").strip().lower()
    if not text:
        return False
    return (
        "distancesensorcorrection(" in text
        or "applyimmediatecorrectionauto(" in text
    )


def parse_and_apply_cmds(node, cmd_str, idx):
    """Parse and apply commands to node."""
    acts = []
    node.pop("swing_target_heading_deg", None)
    node["reverse"] = False
    has_reverse_action = False
    presets = _mech_presets()
    preset_names = [p["name"] for p in presets if str(p.get("name", "")).strip()]
    preset_names.sort(key=lambda name: len(name), reverse=True)
    preset_map = {str(name).strip().lower(): name for name in preset_names}

    reshape_alias = str(CFG.get("reshape_label", "Reshape")).strip().lower()
    if reshape_alias in ("reshape", "rs", "geom"):
        reshape_alias = ""

    def _num_token(tok):
        """Handle num token."""
        t = str(tok or "").strip().lower()
        t = t.replace("deg/s", "").replace("dps", "")
        t = t.replace("deg", "").replace("\u00b0", "")
        t = t.replace("sec", "").replace("s", "")
        t = re.sub(r"[^0-9eE+\-\.]", "", t)
        return t

    if cmd_str:
        for part in _split_command_parts(cmd_str):
            low = part.lower()
            tokens_raw = part.split()
            cmd = tokens_raw[0].lower() if tokens_raw else ""
            try:
                if cmd in ("wait", "w", "sleep", "pause"):
                    if len(tokens_raw) >= 2:
                        x = float(_num_token(tokens_raw[1]))
                    else:
                        x = 0.0
                    if x > 0.0:
                        acts.append({"type": "wait", "s": x})

                elif cmd in ("turn", "t", "rotate"):
                    if len(tokens_raw) >= 2:
                        raw = float(_num_token(tokens_raw[1]))
                        acts.append({"type": "turn", "deg": interpret_input_angle(raw)})

                elif cmd == "swingto":
                    if len(tokens_raw) >= 2:
                        raw = float(_num_token(tokens_raw[1]))
                        node["turn_mode"] = "swing"
                        node["swing_dir"] = "auto"
                        node["swing_target_heading_deg"] = interpret_input_angle(raw)

                elif cmd in ("settleswing", "swing"):
                    settle_flag = (cmd == "settleswing")
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
                        raw = float(_num_token(val_tok))
                        sd = dir_tok if dir_tok in ("cw", "ccw", "auto") else "auto"
                        acts.append({"type": "swing", "deg": interpret_input_angle(raw), "dir": sd, "settle": settle_flag})

                elif cmd in ("movetopose", "poseheading", "pose"):
                    tokens = tokens_raw
                    if len(tokens) >= 2 and tokens[1].lower() in ("off", "none", "clear"):
                        node["move_to_pose"] = False
                        node.pop("pose_heading_deg", None)
                        node.pop("pose_lead_in", None)
                    elif len(tokens) >= 2:
                        try:
                            node["move_to_pose"] = True
                            node["pose_heading_deg"] = interpret_input_angle(float(_num_token(tokens[1])))
                            if len(tokens) >= 3:
                                try:
                                    node["pose_lead_in"] = float(_num_token(tokens[2]))
                                except Exception:
                                    pass
                            else:
                                node.pop("pose_lead_in", None)
                        except Exception:
                            pass

                elif cmd == "offset":
                    if idx != 0 and int(node.get("offset", 0)) == 0 and len(tokens_raw) >= 2:
                        node["offset_custom_in"] = float(_num_token(tokens_raw[1]))

                elif cmd in ("reshape", "rs", "geom") or (reshape_alias and (low == reshape_alias or low.startswith(reshape_alias + " "))):
                    if cmd in ("reshape", "rs", "geom"):
                        rem = part[len(tokens_raw[0]):].strip() if tokens_raw else ""
                    else:
                        rem = part[len(reshape_alias):].strip()
                    rem_tokens = rem.split() if rem else []
                    if rem_tokens and rem_tokens[0].lower() in ("on", "off", "toggle"):
                        state = rem_tokens[0].lower()
                        tail = rem_tokens[1:]
                        value = " ".join(tail) if tail else None
                        acts.append({
                            "type": "preset",
                            "name": "reshape",
                            "state": _normalize_preset_state_token(state),
                            "value": value,
                            "values": tail[:3],
                        })
                    else:
                        acts.append({"type": "reshape"})

                elif cmd == "reverse":
                    tokens = low.split()
                    state = _normalize_reverse_action_state(tokens[1] if len(tokens) >= 2 else None)
                    acts.append({"type": "reverse", "state": state})
                    has_reverse_action = True

                elif cmd in ("latspeed", "lat", "drive_speed"):
                    if len(tokens_raw) >= 2:
                        token = tokens_raw[1].lower()
                        if token in ("off", "none", "clear", "default"):
                            node.pop("custom_lateral_cmd", None)
                            node.pop("custom_lateral_ips", None)
                            node["custom_lateral_reset"] = True
                        else:
                            if not token.endswith("ips"):
                                if token.endswith("cmd"):
                                    token = token[:-3]
                                val = max(0.0, min(127.0, float(_num_token(token))))
                                if val > 0:
                                    node["custom_lateral_cmd"] = val
                                    node.pop("custom_lateral_reset", None)
                                else:
                                    node.pop("custom_lateral_cmd", None)
                                    node["custom_lateral_reset"] = True
                                node.pop("custom_lateral_ips", None)

                elif cmd == "chain":
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
                                node["chain_looseness"] = max(0.0, min(1.0, float(tokens[1])))
                            except Exception:
                                pass

                elif cmd in ("turnspeed", "turnrate", "omega"):
                    if len(tokens_raw) >= 2:
                        token = tokens_raw[1].lower()
                        if token in ("off", "none", "clear"):
                            node.pop("custom_turn_dps", None)
                        else:
                            node["custom_turn_dps"] = float(_num_token(token))

                elif cmd in ("correct", "correction", "atticus"):
                    token = tokens_raw[1].lower() if len(tokens_raw) >= 2 else "immediate"
                    if token in ("off", "none", "clear", "disable"):
                        _set_node_atticus_correction_mode(node, None)
                    elif token in ("immediate", "oneshot", "one-shot", "now"):
                        _set_node_atticus_correction_mode(node, "immediate")
                    elif token in ("walltrim", "wall", "trim", "wallalign", "wallalignment", "thetawall"):
                        _set_node_atticus_correction_mode(node, "wall_trim")
                    elif token in ("roughwall", "rough", "park", "parking", "wallrough", "roughterrain"):
                        _set_node_atticus_correction_mode(node, "rough_wall")

                elif cmd in ("immediatecorrection", "immediatecorrect", "correctnow"):
                    _set_node_atticus_correction_mode(node, "immediate")

                elif cmd in ("walltrim", "wallalign", "wallalignment", "thetawall"):
                    if len(tokens_raw) >= 2 and tokens_raw[1].lower() in ("off", "none", "clear", "disable"):
                        _set_node_atticus_correction_mode(node, None)
                    else:
                        _set_node_atticus_correction_mode(node, "wall_trim")

                elif cmd in ("roughwall", "wallrough", "roughterrain", "parkmode", "parkingmode"):
                    if len(tokens_raw) >= 2 and tokens_raw[1].lower() in ("off", "none", "clear", "disable"):
                        _set_node_atticus_correction_mode(node, None)
                    else:
                        _set_node_atticus_correction_mode(node, "rough_wall")

                elif cmd == "code":
                    code = part[len(tokens_raw[0]):].strip() if tokens_raw else ""
                    if code:
                        acts.append({"type": "code", "code": code})

                elif cmd == "dsr":
                    if len(tokens_raw) >= 2 and tokens_raw[1].lower() in ("off", "none", "clear", "disable"):
                        _set_node_atticus_correction_mode(node, None)
                    else:
                        _set_node_atticus_correction_mode(node, "immediate")

                else:
                    matched_name = preset_map.get(cmd)
                    if matched_name is None:
                        for name in preset_names:
                            name_low = name.lower()
                            if low == name_low or low.startswith(name_low + " "):
                                matched_name = name
                                break
                    if matched_name:
                        rem = part[len(matched_name):].strip()
                        tokens = rem.split() if rem else []
                        state = None
                        value = None
                        values = []
                        if tokens and tokens[0].lower() in ("on", "off", "toggle"):
                            state = _normalize_preset_state_token(tokens[0])
                            tokens = tokens[1:]
                        if tokens:
                            value = " ".join(tokens)
                            values = tokens[:3]
                        acts.append({"type": "preset", "name": matched_name, "state": state, "value": value, "values": values})
                    else:
                        code = part.strip()
                        if code:
                            acts.append({"type": "code", "code": code})
            except Exception:
                pass
    if has_reverse_action:
        node["reverse"] = False
    node["actions"] = acts
    _normalize_node_mech_payload(node)

def _normalize_node_mech_payload(node):
    """Canonicalize node mechanism actions/events so save/load/export stay in sync."""
    if not isinstance(node, dict):
        return

    def _norm_preset_like(src):
        """Handle norm preset like."""
        name = str(src.get("name", "")).strip()
        if not name:
            return None
        state = _normalize_preset_state_token(src.get("state", None))
        value = src.get("value", None)
        values = src.get("values", [])
        if not isinstance(values, list):
            values = []
        values = [str(v) for v in values[:3]]
        if value in (None, "") and values:
            value = " ".join(values)
        return {
            "name": name,
            "state": state,
            "value": value,
            "values": values,
        }

    src_actions = node.get("actions")
    if not isinstance(src_actions, list):
        src_actions = node.get("actions_out")
    if not isinstance(src_actions, list):
        src_actions = []
    out_actions = []
    saw_legacy_dsr = False
    for act in src_actions:
        if not isinstance(act, dict):
            continue
        t = str(act.get("type", "")).strip().lower()
        if t == "geom":
            t = "reshape"
        if t == "preset":
            preset = _norm_preset_like(act)
            if preset is None:
                continue
            if _looks_like_legacy_dsr_preset(preset.get("name", "")):
                saw_legacy_dsr = True
                continue
            out_actions.append({"type": "preset", **preset})
            continue
        if t == "code":
            code = str(act.get("code", "")).strip()
            if _looks_like_legacy_dsr_code(code):
                saw_legacy_dsr = True
                continue
            if code:
                out_actions.append({"type": "code", "code": code})
            continue
        if t == "reverse":
            out_actions.append({"type": "reverse", "state": _normalize_reverse_action_state(act.get("state", None))})
            continue
        if t in ("turn", "wait", "swing", "reverse", "reshape"):
            out_actions.append(dict(act))
            continue
        out_actions.append(dict(act))
    node["actions"] = out_actions
    node["actions_out"] = list(out_actions)
    if saw_legacy_dsr and _node_atticus_correction_mode(node) is None:
        _set_node_atticus_correction_mode(node, "immediate")

    pd = node.get("path_to_next")
    if isinstance(pd, dict) and "edge_events" in pd and "edge_events" not in node:
        if isinstance(pd.get("edge_events"), list):
            node["edge_events"] = list(pd.get("edge_events"))

    raw_events = node.get("edge_events", [])
    if not isinstance(raw_events, list):
        raw_events = []
    norm_events = []
    for ev in raw_events:
        if not isinstance(ev, dict):
            continue
        try:
            t_val = max(0.0, min(1.0, float(ev.get("t", 0.0))))
        except Exception:
            t_val = 0.0
        enabled = bool(ev.get("enabled", True))
        raw_marker_actions = ev.get("actions", [])
        if not isinstance(raw_marker_actions, list):
            raw_marker_actions = []
        marker_actions = []
        for ma in raw_marker_actions:
            if not isinstance(ma, dict):
                continue
            kind = str(ma.get("kind", ma.get("type", ""))).strip().lower()
            if kind == "preset":
                preset = _norm_preset_like(ma)
                if preset is None:
                    continue
                marker_actions.append({"kind": "preset", **preset})
            elif kind == "code":
                code = str(ma.get("code", "")).strip()
                if code:
                    marker_actions.append({"kind": "code", "code": code})
        if marker_actions:
            norm_events.append({
                "t": t_val,
                "enabled": enabled,
                "actions": marker_actions,
            })
    if norm_events:
        node["edge_events"] = norm_events
    elif "edge_events" in node:
        node.pop("edge_events", None)

def _mech_presets():
    """Handle mech presets."""
    _ensure_locked_mech_presets(CFG)
    raw = CFG.get("codegen", {}).get("mech_presets", [])
    if not isinstance(raw, list):
        return []
    def _normalize_cases(items):
        """Handle normalize cases."""
        out_cases = []
        if isinstance(items, dict):
            items = [{"key": k, "template": v} for k, v in items.items()]
        if not isinstance(items, list):
            return out_cases
        for it in items:
            if isinstance(it, dict):
                key = str(it.get("key", "")).strip()
                tpl = str(it.get("template", "")).strip()
            elif isinstance(it, (list, tuple)) and len(it) >= 2:
                key = str(it[0]).strip()
                tpl = str(it[1]).strip()
            else:
                continue
            if not key:
                continue
            out_cases.append({"key": key, "template": tpl})
        return out_cases
    out = []
    for p in raw:
        if not isinstance(p, dict):
            continue
        name = str(p.get("name", "")).strip()
        if not name:
            continue
        mode = str(p.get("mode", "action")).strip().lower()
        if mode not in ("action", "toggle", "cases"):
            mode = "action"
        out.append({
            "name": name,
            "mode": mode,
            "template": str(p.get("template", "") or p.get("action", "") or ""),
            "on": str(p.get("on", "")),
            "off": str(p.get("off", "")),
            "default": bool(p.get("default", False)),
            "cases": _normalize_cases(p.get("cases", [])),
            "case_default": str(p.get("case_default", "") or ""),
        })
    return out

def _preset_value_text(act):
    """Handle preset value text."""
    value_raw = act.get("value", None)
    if value_raw is not None:
        return str(value_raw).strip()
    values = act.get("values", None)
    if isinstance(values, list):
        parts = []
        for v in values:
            v_str = str(v).strip()
            if v_str:
                parts.append(v_str)
        return " ".join(parts)
    return ""

def _marker_actions_to_text(actions):
    """Handle marker actions to text."""
    parts = []
    if not isinstance(actions, list):
        return ""
    for act in actions:
        if not isinstance(act, dict):
            continue
        kind = str(act.get("kind", "code")).lower()
        if kind == "preset":
            name = str(act.get("name", "")).strip()
            state_raw = act.get("state", None)
            if isinstance(state_raw, bool):
                state = "on" if state_raw else "off"
            elif state_raw is None:
                state = ""
            else:
                state = str(state_raw).strip().lower()
            value = _preset_value_text(act)
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
    """Handle marker apply reshape."""
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


def _marker_apply_atticus(actions):
    """Run built-in Atticus marker commands inside the simulator."""
    if not isinstance(actions, list):
        return
    for act in actions:
        if not isinstance(act, dict):
            continue
        if str(act.get("kind", "")).strip().lower() != "preset":
            continue
        if str(act.get("name", "")).strip().lower() != "dsr":
            continue
        values = act.get("values", [])
        if isinstance(values, list) and values:
            sensor_tokens = [str(v).strip() for v in values if str(v).strip()][:2]
        else:
            value = _preset_value_text(act)
            sensor_tokens = [tok for tok in re.split(r"[\s,;/|+]+", value) if tok][:2] if value else []
        _atticus_apply_immediate_correction(sensor_tokens or None)

def _marker_hover_text(actions):
    """Handle marker hover text."""
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
                label = reshape_label if name.strip().lower() == "reshape" else name
                text = label
                state_raw = act.get("state", None)
                if isinstance(state_raw, bool):
                    state = "on" if state_raw else "off"
                elif state_raw is None:
                    state = ""
                else:
                    state = str(state_raw).strip().lower()
                value = _preset_value_text(act)
                if state:
                    text = f"{text} {state}"
                if value:
                    text = f"{text} {value}"
                items.append(text)
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
    """Draw marker hover."""
    if not text:
        return
    draw_label(surface, mouse_pos, [text], font_small)

def _parse_marker_actions(text):
    """Handle parse marker actions."""
    actions = []
    if not text:
        return actions
    presets = _mech_presets()
    preset_names = [p["name"] for p in presets]
    preset_names.sort(key=lambda s: len(s), reverse=True)
    reshape_label = str(CFG.get("reshape_label", "Reshape")).strip()
    reshape_alias = reshape_label.lower() if reshape_label else ""
    if reshape_alias == "reshape":
        reshape_alias = ""
    for part in _split_command_parts(text):
        low = part.lower()
        if low.startswith("code "):
            actions.append({"kind": "code", "code": part[5:].strip()})
            continue
        matched_name = None
        rem = None
        if reshape_alias and (low == reshape_alias or low.startswith(reshape_alias + " ")):
            matched_name = "reshape"
            rem = part[len(reshape_label):].strip()
        else:
            for name in preset_names:
                if low == name.lower() or low.startswith(name.lower() + " "):
                    matched_name = name
                    rem = part[len(matched_name):].strip()
                    break
        if matched_name:
            tokens = rem.split() if rem else []
            state = None
            value = None
            values = []
            if tokens and tokens[0].lower() in ("on", "off", "toggle"):
                state = _normalize_preset_state_token(tokens[0])
                tokens = tokens[1:]
            if tokens:
                value = " ".join(tokens)
                values = tokens[:3]
            actions.append({"kind": "preset", "name": matched_name, "state": state, "value": value, "values": values})
        else:
            actions.append({"kind": "code", "code": part})
    return actions

def _marker_prompt_text():
    """Handle marker prompt text."""
    presets = _mech_presets()
    lines = [
        "Commands (comma/semicolon separated). Examples:",
        "  intake on, clamp toggle",
        "  lift 300",
        f"  DSR RIGHT LEFT   (explicit distance reset using up to two sensors; stay settled about {ATTICUS_DSR_RECOMMENDED_STILL_MS:g} ms)",
        "  DSR              (no value = auto immediate correction)",
        "  trap open   (works with Cases preset keys like words/numbers)",
        "  lift 100 200 300   (preset values -> {VALUE}/{VALUE1-3})",
        "  clamp on 1 2 3     (toggle + values, {STATE} on/off)",
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
    """Handle edit edge marker."""
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
    tutorial_state["flags"]["mech_marker"] = True
    return True
    if has_reverse_action:
        node["reverse"] = False
    _apply_profile_override_for_speed_overrides(node)

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
            """Handle ok."""
            resp["value"] = entry_var.get()
            try:
                top.destroy()
            except Exception:
                pass

        def _cancel():
            """Handle cancel."""
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
        "turn/swing/settleswing [CW/CCW] + optional movetopose [heading] [lead const] (comma/semicolon separated).\n"
        "Advanced vars/motion: profile precise|normal|fast|slam (non-path segments only).\n"
        "Examples: 'turn auto;'; 'swing cw; movetopose 180 0.6'; 'settleswing cw'; 'movetopose off'; 'profile precise'.\n"
        "Omit movetopose (or use 'movetopose off') to disable enforcing final heading.\n"
        "Angles use 0=left (+Y), 90=up (+X).",
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
    
    tabs = {
        "controls": ttk.Frame(notebook),
        "general": ttk.Frame(notebook),
        "physics": ttk.Frame(notebook),
        "geometry": ttk.Frame(notebook),
        "mcl": ttk.Frame(notebook),
        "codegen": ttk.Frame(notebook),
    }
    
    notebook.add(tabs["controls"], text="Controls")
    notebook.add(tabs["general"], text="General")
    notebook.add(tabs["physics"], text="Physics")
    notebook.add(tabs["geometry"], text="Geometry")
    notebook.add(tabs["mcl"], text="Atticus")
    notebook.add(tabs["codegen"], text="Export")
    
    for name in ("general", "geometry", "mcl", "codegen"):
        for c in range(2):
            tabs[name].columnconfigure(c, weight=1)

    general_tab = tabs["general"]
    general_tab.rowconfigure(0, weight=1)
    general_tab.columnconfigure(0, weight=1)
    general_tab.columnconfigure(1, weight=0)
    general_canvas = tk.Canvas(general_tab, borderwidth=0, highlightthickness=0)
    general_scroll = ttk.Scrollbar(general_tab, orient="vertical", command=general_canvas.yview)
    general_canvas.configure(yscrollcommand=general_scroll.set)
    general_body = ttk.Frame(general_canvas)
    general_win = general_canvas.create_window((0, 0), window=general_body, anchor="nw")
    general_body.columnconfigure(0, weight=1)
    general_body.columnconfigure(1, weight=1)
    general_canvas.grid(row=0, column=0, sticky="nsew")
    general_scroll.grid(row=0, column=1, sticky="ns")

    def _general_on_configure(_evt=None):
        """Update general-tab scroll region and content width."""
        try:
            general_canvas.configure(scrollregion=general_canvas.bbox("all"))
            general_canvas.itemconfigure(general_win, width=general_canvas.winfo_width())
        except Exception:
            pass

    def _general_mousewheel(event):
        """Handle mousewheel scrolling in General tab."""
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
            general_canvas.yview_scroll(delta, "units")
            return "break"

    general_body.bind("<Configure>", _general_on_configure)
    general_canvas.bind("<Configure>", _general_on_configure)
    for w in (general_canvas, general_body):
        w.bind("<MouseWheel>", _general_mousewheel)
        w.bind("<Button-4>", _general_mousewheel)
        w.bind("<Button-5>", _general_mousewheel)
    
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
        """Handle on controls configure."""
        controls_canvas.configure(scrollregion=controls_canvas.bbox("all"))
    
    controls_frame.bind("<Configure>", _on_controls_configure)
    
    def _on_mousewheel(event):
        """Handle on mousewheel."""
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
    
    dist_labels = ["Inches", "Encoder degrees", "Encoder rotations", "Ticks"]
    dist_map = {dist_labels[0]: 0, dist_labels[1]: 1, dist_labels[2]: 2, dist_labels[3]: 3}
    dist_inv = {0: dist_labels[0], 1: dist_labels[1], 2: dist_labels[2], 3: dist_labels[3]}
    
    ang_labels = ["Degrees", "Radians"]
    ang_map = {ang_labels[0]: 0, ang_labels[1]: 1}
    ang_inv = {0: ang_labels[0], 1: ang_labels[1]}
    
    dist_value = int(CFG.get("distance_units", 0))
    dist_value = 0 if dist_value not in (0, 1, 2, 3) else dist_value
    ang_value = int(CFG.get("angle_units", 0))
    ang_value = 0 if ang_value not in (0, 1) else ang_value
    
    dist_var = tk.StringVar(value=dist_inv[dist_value])
    ang_var = tk.StringVar(value=ang_inv[ang_value])
    
    init_heading_disp = convert_heading_input(initial_state["heading"], None)
    init_head = tk.StringVar(value=f"{init_heading_disp:.3f}")
    
    show_hitboxes_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_hitboxes", 1)))
    show_hitbox_conflicts_only_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_hitbox_conflicts_only", 0)))
    show_field_objects_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_field_objects", 1)))
    show_node_numbers_var = tk.IntVar(value=int(CFG.get("ui", {}).get("show_node_numbers", 1)))
    reshape_label_var = tk.StringVar(value=str(CFG.get("reshape_label", "Reshape")))
    auto_heading_node1_var = tk.IntVar(value=0)
    
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
    gr_var = tk.StringVar(value=str(CFG.get("gear_ratio", 1.0)))
    dens_var = tk.StringVar(value=str(rp.get("point_density_per_in", 4.0)))
    curv_var = tk.DoubleVar(value=float(rp.get("curvature_gain", 0.05)))
    
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
    def _sanitize_advanced_poly(points):
        """Normalize advanced geometry points to [[x_in, y_in], ...]."""
        if isinstance(points, dict):
            points = points.get("value", [])
        if not isinstance(points, (list, tuple)):
            return []
        out = []
        for pt in points:
            x = y = None
            if isinstance(pt, dict):
                x = pt.get("x", pt.get("x_in"))
                y = pt.get("y", pt.get("y_in"))
            elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                x, y = pt[0], pt[1]
            if x is None or y is None:
                continue
            try:
                out.append([round(float(x), 4), round(float(y), 4)])
            except Exception:
                continue
        return out
    adv_geom_raw = bd.get("advanced_geometry", {})
    if not isinstance(adv_geom_raw, dict):
        adv_geom_raw = {}
    adv_enabled_raw = adv_geom_raw.get("enabled", 0)
    if isinstance(adv_enabled_raw, dict):
        adv_enabled_raw = adv_enabled_raw.get("value", 0)
    try:
        adv_enabled = int(bool(int(adv_enabled_raw)))
    except Exception:
        adv_enabled = int(bool(adv_enabled_raw))
    adv_sym_raw = adv_geom_raw.get("symmetry", 0)
    if isinstance(adv_sym_raw, dict):
        adv_sym_raw = adv_sym_raw.get("value", 0)
    try:
        adv_symmetry = int(bool(int(adv_sym_raw)))
    except Exception:
        adv_symmetry = int(bool(adv_sym_raw))
    adv_points = _sanitize_advanced_poly(adv_geom_raw.get("points", []))
    adv_reshape_points = _sanitize_advanced_poly(adv_geom_raw.get("reshape_points", []))
    legacy_normal_pts = _sanitize_advanced_poly(adv_geom_raw.get("normal", []))
    legacy_reshape_pts = _sanitize_advanced_poly(adv_geom_raw.get("reshape", []))
    if not adv_points:
        adv_points = legacy_normal_pts if legacy_normal_pts else legacy_reshape_pts
    if not adv_reshape_points:
        adv_reshape_points = legacy_reshape_pts if legacy_reshape_pts else adv_points
    advanced_geometry_state = {
        "enabled": adv_enabled,
        "symmetry": adv_symmetry,
        "points": adv_points,
        "reshape_points": adv_reshape_points,
    }
    bd["advanced_geometry"] = {
        "enabled": int(advanced_geometry_state.get("enabled", 0)),
        "symmetry": int(advanced_geometry_state.get("symmetry", 0)),
        "points": [list(p) for p in advanced_geometry_state.get("points", [])],
        "reshape_points": [list(p) for p in advanced_geometry_state.get("reshape_points", [])],
    }
    
    off = CFG["offsets"]
    off1 = tk.StringVar(value=str(off.get("offset_1_in", 0)))
    off2 = tk.StringVar(value=str(off.get("offset_2_in", 0)))
    off3 = tk.StringVar(value=str(off.get("offset_3_in", 0)))
    pad = tk.StringVar(value=str(off.get("padding_in", 0)))

    mcl_cfg = CFG.get("mcl", {})
    mcl_interop_cfg = mcl_cfg.get("interop", {}) if isinstance(mcl_cfg, dict) else {}
    mcl_enabled_var = tk.IntVar(value=int(mcl_cfg.get("enabled", 0)))
    mcl_pose_convention_var = tk.StringVar(
        value=str(mcl_interop_cfg.get("pose_convention", "cw_zero_forward"))
    )
    mcl_pose_swap_xy_var = tk.IntVar(value=int(mcl_interop_cfg.get("swap_xy", 0)))
    mcl_pose_invert_x_var = tk.IntVar(value=int(mcl_interop_cfg.get("invert_x", 0)))
    mcl_pose_invert_y_var = tk.IntVar(value=int(mcl_interop_cfg.get("invert_y", 0)))
    mcl_motion_ms_var = tk.StringVar(value=str(mcl_cfg.get("motion_ms", 20)))
    mcl_sensor_ms_var = tk.StringVar(value=str(mcl_cfg.get("sensor_ms", 20)))
    motion_cfg = mcl_cfg.get("motion", {})
    tracking_cfg = mcl_cfg.get("tracking", {})
    mcl_motion_enabled_var = tk.IntVar(value=int(motion_cfg.get("enabled", 1)))
    mcl_horizontal_odom_var = tk.StringVar(
        value="enabled" if int(tracking_cfg.get("horizontal_enabled", 0)) == 1 else "none"
    )
    mcl_sigma_x_var = tk.StringVar(value=str(motion_cfg.get("sigma_x_in", 0.08)))
    mcl_sigma_y_var = tk.StringVar(value=str(motion_cfg.get("sigma_y_in", 0.08)))
    mcl_sigma_theta_var = tk.StringVar(value=str(motion_cfg.get("sigma_theta_deg", 0.7)))
    mcl_sigma_x_per_var = tk.StringVar(value=str(motion_cfg.get("sigma_x_per_in", 0.02)))
    mcl_sigma_y_per_var = tk.StringVar(value=str(motion_cfg.get("sigma_y_per_in", 0.03)))
    mcl_sigma_theta_per_var = tk.StringVar(value=str(motion_cfg.get("sigma_theta_per_deg", 0.05)))
    mcl_no_horizontal_scale_var = tk.StringVar(value=str(motion_cfg.get("no_horizontal_lateral_scale", 2.5)))
    mcl_turn_lateral_scale_var = tk.StringVar(value=str(motion_cfg.get("turn_lateral_scale", 2.0)))
    mcl_rough_wall_sigma_x_scale_var = tk.StringVar(value=str(motion_cfg.get("rough_wall_sigma_x_scale", 1.8)))
    mcl_rough_wall_sigma_y_scale_var = tk.StringVar(value=str(motion_cfg.get("rough_wall_sigma_y_scale", 3.5)))
    mcl_rough_wall_sigma_theta_scale_var = tk.StringVar(value=str(motion_cfg.get("rough_wall_sigma_theta_scale", 1.25)))
    mcl_set_pose_xy_var = tk.StringVar(value=str(mcl_cfg.get("set_pose_sigma_xy_in", 0.2)))
    mcl_set_pose_theta_var = tk.StringVar(value=str(mcl_cfg.get("set_pose_sigma_theta_deg", 2.0)))
    sensors_cfg = mcl_cfg.get("sensors", {})
    dist_cfg = sensors_cfg.get("distance", {})
    imu_cfg = sensors_cfg.get("imu", {})
    mcl_dist_enabled_var = tk.IntVar(value=int(dist_cfg.get("enabled", 1)))
    mcl_dist_sigma_var = tk.StringVar(value=str(dist_cfg.get("sigma_hit_mm", 15.0)))
    mcl_dist_w_hit_var = tk.StringVar(value=str(dist_cfg.get("w_hit", 0.9)))
    mcl_dist_w_rand_var = tk.StringVar(value=str(dist_cfg.get("w_rand", 0.1)))
    mcl_dist_w_short_var = tk.StringVar(value=str(dist_cfg.get("w_short", 0.0)))
    mcl_dist_w_max_var = tk.StringVar(value=str(dist_cfg.get("w_max", 0.0)))
    mcl_dist_lambda_short_var = tk.StringVar(value=str(dist_cfg.get("lambda_short", 0.1)))
    mcl_dist_max_range_var = tk.StringVar(value=str(dist_cfg.get("max_range_mm", 2000.0)))
    mcl_dist_min_range_var = tk.StringVar(value=str(dist_cfg.get("min_range_mm", 20.0)))
    mcl_dist_conf_min_var = tk.StringVar(value=str(dist_cfg.get("confidence_min", 0.0)))
    mcl_dist_obj_size_min_var = tk.StringVar(value=str(dist_cfg.get("object_size_min", 0.0)))
    mcl_dist_obj_size_max_var = tk.StringVar(value=str(dist_cfg.get("object_size_max", 0.0)))
    mcl_dist_innov_gate_var = tk.StringVar(value=str(dist_cfg.get("innovation_gate_mm", 180.0)))
    mcl_dist_median_window_var = tk.StringVar(value=str(dist_cfg.get("median_window", 3)))
    mcl_dist_ignore_max_var = tk.IntVar(value=int(dist_cfg.get("lf_ignore_max", 0)))
    mcl_dist_gate_var = tk.StringVar(value=str(dist_cfg.get("gate_mm", 180.0)))
    mcl_dist_gate_mode_var = tk.StringVar(value=str(dist_cfg.get("gate_mode", "hard")))
    mcl_dist_gate_penalty_var = tk.StringVar(value=str(dist_cfg.get("gate_penalty", 0.05)))
    mcl_dist_gate_reject_var = tk.StringVar(value=str(dist_cfg.get("gate_reject_ratio", 0.9)))
    lf_cfg = dist_cfg.get("likelihood_field", {})
    mcl_dist_lf_res_var = tk.StringVar(value=str(lf_cfg.get("resolution_in", 1.0)))
    mcl_imu_enabled_var = tk.IntVar(value=int(imu_cfg.get("enabled", 1)))
    mcl_imu_sigma_var = tk.StringVar(value=str(imu_cfg.get("sigma_deg", 1.0)))
    mcl_ekf_use_imu_var = tk.IntVar(value=int((mcl_cfg.get("ekf", {}) or {}).get("use_imu_update", 1)))
    geom_cfg = mcl_cfg.get("sensor_geometry", {})
    geom_sensors = geom_cfg.get("distance_sensors", [])
    if not isinstance(geom_sensors, list) or not geom_sensors:
        geom_sensors = mcl_mod.get_distance_sensors(CFG, include_disabled=True)
    mcl_sensor_vars = []
    for idx in range(4):
        entry = geom_sensors[idx] if idx < len(geom_sensors) else {}
        has_entry = isinstance(entry, dict) and bool(entry)
        enabled_default = int(entry.get("enabled", 1)) if has_entry else 0
        mcl_sensor_vars.append({
            "name": tk.StringVar(value=str(entry.get("name", ""))),
            "x_in": tk.StringVar(value=str(entry.get("x_in", 0.0))),
            "y_in": tk.StringVar(value=str(entry.get("y_in", 0.0))),
            "angle_deg": tk.StringVar(value=str(entry.get("angle_deg", 0.0))),
            "enabled": tk.IntVar(value=enabled_default),
            "bias_mm": tk.StringVar(value=str(entry.get("bias_mm", 0.0))),
            "angle_offset_deg": tk.StringVar(value=str(entry.get("angle_offset_deg", 0.0))),
            "min_range_mm": tk.StringVar(value=str(entry.get("min_range_mm", dist_cfg.get("min_range_mm", 20.0)))),
            "max_range_mm": tk.StringVar(value=str(entry.get("max_range_mm", dist_cfg.get("max_range_mm", 2000.0)))),
            "min_confidence": tk.StringVar(value=str(entry.get("min_confidence", dist_cfg.get("confidence_min", 0.0)))),
            "min_object_size": tk.StringVar(value=str(entry.get("min_object_size", dist_cfg.get("object_size_min", 0.0)))),
            "max_object_size": tk.StringVar(value=str(entry.get("max_object_size", dist_cfg.get("object_size_max", 0.0)))),
            "innovation_gate_mm": tk.StringVar(value=str(entry.get("innovation_gate_mm", dist_cfg.get("innovation_gate_mm", 180.0)))),
            "map_mode": tk.StringVar(value=str(entry.get("map_mode", "perimeter"))),
        })
    map_cfg = mcl_cfg.get("map_objects", {})
    mcl_map_perimeter_var = tk.IntVar(value=int(map_cfg.get("perimeter", 1)))
    mcl_map_long_goals_var = tk.IntVar(value=int(map_cfg.get("long_goals", 1)))
    mcl_map_long_goal_braces_var = tk.IntVar(value=int(map_cfg.get("long_goal_braces", 1)))
    mcl_map_center_goals_var = tk.IntVar(value=int(map_cfg.get("center_goals", 1)))
    mcl_map_matchloaders_var = tk.IntVar(value=int(map_cfg.get("matchloaders", 1)))
    mcl_map_park_zones_var = tk.IntVar(value=int(map_cfg.get("park_zones", 0)))
    mcl_object_vars = {}
    mcl_object_entries = []
    sensor_obj_vis_cfg = mcl_cfg.get("sensor_object_visibility", {})
    if not isinstance(sensor_obj_vis_cfg, dict):
        sensor_obj_vis_cfg = {}
    for entry in get_field_object_entries(CFG):
        obj_id = entry.get("id")
        if not obj_id:
            continue
        obj_label = str(entry.get("label", obj_id) or obj_id)
        default_on = 1 if mcl_mod._entry_default_enabled(CFG, entry) else 0
        sel = mcl_cfg.get("object_selection", {})
        mcl_object_vars[obj_id] = tk.IntVar(value=int(sel.get(obj_id, default_on)))
        mcl_object_entries.append((obj_id, obj_label))
    mcl_sensor_object_vars = {}
    for idx, sensor_vars in enumerate(mcl_sensor_vars):
        raw_name = str(sensor_vars["name"].get() or "").strip()
        sensor_name = raw_name if raw_name else f"sensor_{idx + 1}"
        vis_raw = sensor_obj_vis_cfg.get(str(idx), sensor_obj_vis_cfg.get(sensor_name, sensor_obj_vis_cfg.get(f"sensor_{idx + 1}", {})))
        if not isinstance(vis_raw, dict):
            vis_raw = {}
        per_sensor = {}
        for obj_id, _obj_label in mcl_object_entries:
            global_default = int(mcl_object_vars.get(obj_id).get()) if obj_id in mcl_object_vars else 1
            try:
                vis_default = int(vis_raw.get(obj_id, global_default))
            except Exception:
                vis_default = global_default
            per_sensor[obj_id] = tk.IntVar(value=1 if vis_default else 0)
        mcl_sensor_object_vars[idx] = per_sensor
    corr_cfg = mcl_cfg.get("correction", {})
    mcl_corr_enabled_var = tk.IntVar(value=int(corr_cfg.get("enabled", 1)))
    mcl_corr_min_conf_var = tk.StringVar(value=str(corr_cfg.get("min_confidence", 0.6)))
    mcl_corr_max_trans_var = tk.StringVar(value=str(corr_cfg.get("max_trans_jump_in", 4.0)))
    mcl_corr_max_theta_var = tk.StringVar(value=str(corr_cfg.get("max_theta_jump_deg", 8.0)))
    mcl_corr_writeback_alpha_var = tk.StringVar(value=str(corr_cfg.get("writeback_alpha", corr_cfg.get("alpha_max", 0.35))))
    mcl_corr_teleport_trans_var = tk.StringVar(value=str(corr_cfg.get("teleport_reset_trans_in", 18.0)))
    mcl_corr_teleport_theta_var = tk.StringVar(value=str(corr_cfg.get("teleport_reset_theta_deg", 45.0)))
    mcl_ui_cfg = mcl_cfg.get("ui", {})
    mcl_show_estimate_var = tk.IntVar(value=int(mcl_ui_cfg.get("show_estimate", 1)))
    mcl_show_cov_var = tk.IntVar(value=int(mcl_ui_cfg.get("show_covariance", 1)))
    mcl_show_rays_var = tk.IntVar(value=int(mcl_ui_cfg.get("show_rays", 1)))
    mcl_show_gating_var = tk.IntVar(value=int(mcl_ui_cfg.get("show_gating", 1)))
    def _init_margins():
        """Handle init margins."""
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
        return lbl, widget

    def _track_widgets(*widgets):
        """Handle track widgets."""
        for w in widgets:
            try:
                ui.track_live_widget(w)
            except Exception:
                continue

    def _set_row_visible(row, visible: bool) -> None:
        """Set row visible."""
        if not row:
            return
        lbl, widget = row
        try:
            if visible:
                lbl.grid()
                widget.grid()
            else:
                lbl.grid_remove()
                widget.grid_remove()
        except Exception:
            pass
    
    geometry_win = {"win": None}
    def open_geometry_visualizer():
        """Open a drag-to-edit geometry visualizer for drivetrain/full/reshape boxes."""
        if geometry_win["win"] is not None and geometry_win["win"].winfo_exists():
            geometry_win["win"].lift()
            geometry_win["win"].focus_force()
            return
        
        def _num(var, default=0.0):
            """Handle num."""
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
                """Handle parse."""
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
            avail_w = max(0.0, ow - dw)
            ml, mr = adjust_pair(avail_w, ml, mr, "left", "right", "normal")
            avail_l = max(0.0, ol - dl)
            mb, mf = adjust_pair(avail_l, mb, mf, "back", "front", "normal")
            avail_w_r = max(0.0, rw - dw)
            mlr, mrr = adjust_pair(avail_w_r, mlr, mrr, "left_r", "right_r", "reshape")
            avail_l_r = max(0.0, rl - dl)
            mbr, mfr = adjust_pair(avail_l_r, mbr, mfr, "back_r", "front_r", "reshape")
            w_full.set(f"{max(MIN_SIZE, dw + ml + mr):.3f}")
            l_full.set(f"{max(MIN_SIZE, dl + mb + mf):.3f}")
            full_off_y.set(f"{((ml - mr) / 2.0):.3f}")
            full_off_x.set(f"{((mf - mb) / 2.0):.3f}")
            rs_w.set(f"{max(MIN_SIZE, dw + mlr + mrr):.3f}")
            rs_l.set(f"{max(MIN_SIZE, dl + mbr + mfr):.3f}")
            rs_off_y.set(f"{((mlr - mrr) / 2.0):.3f}")
            rs_off_x.set(f"{((mfr - mbr) / 2.0):.3f}")
            margin_left_var.set(f"{ml:.3f}"); margin_right_var.set(f"{mr:.3f}")
            margin_back_var.set(f"{mb:.3f}"); margin_front_var.set(f"{mf:.3f}")
            margin_left_r_var.set(f"{mlr:.3f}"); margin_right_r_var.set(f"{mrr:.3f}")
            margin_back_r_var.set(f"{mbr:.3f}"); margin_front_r_var.set(f"{mfr:.3f}")
            draw()
        def _mark_margin(name, mode_tag):
            """Handle mark margin."""
            _apply_margins._last = {"mode": mode_tag, "name": name}
        _apply_margins._last = {"mode": None, "name": None}
        ttk.Button(sidebar, text="Apply margins", command=_apply_margins).grid(row=18, column=0, sticky="w", pady=(0, 10))
        advanced_status_var = tk.StringVar(value="")
        def _refresh_advanced_status():
            """Refresh advanced geometry summary line."""
            en = bool(int(advanced_geometry_state.get("enabled", 0)))
            sym = bool(int(advanced_geometry_state.get("symmetry", 0)))
            n_pts = len(advanced_geometry_state.get("points", []))
            n_r_pts = len(advanced_geometry_state.get("reshape_points", []))
            if en:
                advanced_status_var.set(
                    f"Advanced geometry ON (normal {n_pts}, reshape {n_r_pts}, symmetry {'ON' if sym else 'OFF'})"
                )
            else:
                advanced_status_var.set(
                    f"Advanced geometry OFF (normal {n_pts}, reshape {n_r_pts}, symmetry {'ON' if sym else 'OFF'})"
                )
        advanced_editor_win = {"win": None}
        def _open_advanced_geometry_editor():
            """Open node editor for the current mode's custom collision polygon."""
            if advanced_editor_win["win"] is not None and advanced_editor_win["win"].winfo_exists():
                advanced_editor_win["win"].lift()
                advanced_editor_win["win"].focus_force()
                return
            adv_top = tk.Toplevel(win)
            advanced_editor_win["win"] = adv_top
            adv_top.title("Advanced Geometry")
            adv_top.resizable(False, False)

            use_custom_var = tk.IntVar(value=int(bool(advanced_geometry_state.get("enabled", 0))))
            symmetry_var = tk.IntVar(value=int(bool(advanced_geometry_state.get("symmetry", 0))))
            active_adv_key = "reshape_points" if mode_var.get() == "reshape" else "points"
            active_mode_label = "Reshape" if active_adv_key == "reshape_points" else "Normal"
            poly_state = [list(p) for p in _sanitize_advanced_poly(advanced_geometry_state.get(active_adv_key, []))]
            selected = {"idx": None, "drag": False, "mirror_idx": None, "side": 0}
            SYM_EPS = 1e-4
            SYM_TOL_IN = 0.6
            adv_canvas_size = 360
            state = {"scale": 1.0, "cx": adv_canvas_size / 2.0, "cy": adv_canvas_size / 2.0}
            root = ttk.Frame(adv_top, padding=6)
            root.pack(fill="both", expand=True)
            left = ttk.Frame(root)
            left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
            right = ttk.Frame(root)
            right.grid(row=0, column=1, sticky="nw")
            root.columnconfigure(0, weight=1)
            root.rowconfigure(0, weight=1)
            adv_canvas = tk.Canvas(left, width=adv_canvas_size, height=adv_canvas_size, background="#f8f8f8",
                                   highlightthickness=1, highlightbackground="#cccccc")
            adv_canvas.pack(fill="both", expand=True)
            def _save_adv_geometry():
                """Persist advanced geometry immediately."""
                try:
                    save_config(CFG)
                except Exception:
                    pass
            def _on_toggle_adv_flag():
                """Handle advanced geometry toggles."""
                _commit_adv(save_now=True)
                _draw_adv()
                draw()
            ttk.Checkbutton(
                right, text="Use custom polygon", variable=use_custom_var,
                command=_on_toggle_adv_flag
            ).grid(row=0, column=0, sticky="w", pady=(0, 8))
            ttk.Checkbutton(
                right, text="Symmetry (mirror left/right)", variable=symmetry_var,
                command=_on_toggle_adv_flag
            ).grid(row=1, column=0, sticky="w", pady=(0, 6))
            ttk.Label(right, text=f"Editing: {active_mode_label} geometry").grid(row=2, column=0, sticky="w", pady=(0, 4))
            help_lbl = ttk.Label(
                right,
                text=(
                    "Left click point: select/drag\n"
                    "Left click edge: insert node + drag\n"
                    "Right click/Delete: remove point\n"
                    "Rect from outer: reset polygon to outer box"
                ),
                justify="left"
            )
            help_lbl.grid(row=3, column=0, sticky="w")
            points_var = tk.StringVar(value="")
            ttk.Label(right, textvariable=points_var, foreground="#555555").grid(row=4, column=0, sticky="w", pady=(6, 6))
            btn_row = ttk.Frame(right)
            btn_row.grid(row=5, column=0, sticky="w")
            def _active_dims_local():
                """Get outer dimensions/offsets for active advanced edit mode."""
                return _outer_vals("reshape" if active_adv_key == "reshape_points" else "normal")
            def _clamp_local_point(x_in, y_in):
                """Clamp local point to current outer geometry bounds."""
                ow, ol, _ox, _oy = _active_dims_local()
                x_lim = max(0.0, float(ol) * 0.5)
                y_lim = max(0.0, float(ow) * 0.5)
                return max(-x_lim, min(x_lim, x_in)), max(-y_lim, min(y_lim, y_in))
            def _to_canvas(x_in, y_in):
                """Convert local geometry coordinates to canvas coordinates."""
                s = max(1e-6, state["scale"])
                return (state["cx"] - float(y_in) * s, state["cy"] - float(x_in) * s)
            def _to_local(px, py):
                """Convert canvas coordinates to local geometry coordinates."""
                s = max(1e-6, state["scale"])
                return ((state["cy"] - float(py)) / s, (state["cx"] - float(px)) / s)
            def _symmetry_on():
                """Return whether mirror symmetry is active."""
                return bool(int(symmetry_var.get()))
            def _nearest_to_target(target_x, target_y, exclude_idx=None, tol_in=SYM_TOL_IN):
                """Find nearest point index to target local position."""
                best_idx = None
                best_d2 = 1e9
                for idx, pt in enumerate(poly_state):
                    if exclude_idx is not None and idx == exclude_idx:
                        continue
                    try:
                        px = float(pt[0]); py = float(pt[1])
                    except Exception:
                        continue
                    d2 = (px - float(target_x)) * (px - float(target_x)) + (py - float(target_y)) * (py - float(target_y))
                    if d2 < best_d2:
                        best_d2 = d2
                        best_idx = idx
                if best_idx is None:
                    return None
                return best_idx if best_d2 <= (float(tol_in) * float(tol_in)) else None
            def _side_from_y(y_in):
                """Map y sign to side id: +1 left, -1 right, 0 centerline."""
                try:
                    y = float(y_in)
                except Exception:
                    return 0
                if y > SYM_EPS:
                    return 1
                if y < -SYM_EPS:
                    return -1
                return 0
            def _find_partner_idx(idx):
                """Find mirrored partner index for a point."""
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return None
                try:
                    x_in = float(poly_state[idx][0])
                    y_in = float(poly_state[idx][1])
                except Exception:
                    return None
                if _side_from_y(y_in) == 0:
                    return None
                return _nearest_to_target(x_in, -y_in, exclude_idx=idx)
            def _ensure_partner_idx(idx):
                """Ensure mirrored partner exists; return (possibly shifted idx, partner_idx)."""
                partner_idx = _find_partner_idx(idx)
                if partner_idx is not None:
                    return idx, partner_idx
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return idx, None
                try:
                    x_in = float(poly_state[idx][0])
                    y_in = float(poly_state[idx][1])
                except Exception:
                    return idx, None
                side = _side_from_y(y_in)
                if side == 0:
                    return idx, None
                mirror_target = (x_in, -y_in)
                edge_info = _nearest_edge_insert_local(mirror_target[0], mirror_target[1])
                if edge_info:
                    insert_at = int(edge_info["insert_idx"])
                    mx, my = _clamp_local_point(float(edge_info["proj"][0]), float(edge_info["proj"][1]))
                    mirror_pt = [round(mx, 4), round(my, 4)]
                else:
                    mirror_pt = [round(x_in, 4), round(-y_in, 4)]
                    insert_at = (idx + 1) if side > 0 else idx
                poly_state.insert(insert_at, mirror_pt)
                if insert_at <= idx:
                    idx += 1
                return idx, insert_at
            def _apply_symmetry_with_partner(idx, partner_idx=None, lock_side=0):
                """Apply mirrored update while keeping point on its side."""
                if not _symmetry_on():
                    return idx, None
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return idx, None
                try:
                    x_in = float(poly_state[idx][0])
                    y_in = float(poly_state[idx][1])
                except Exception:
                    return idx, None
                try:
                    side_lock = int(lock_side)
                except Exception:
                    side_lock = 0
                side = side_lock if side_lock in (-1, 0, 1) else _side_from_y(y_in)
                if side == 1:
                    y_in = max(SYM_EPS, y_in)
                elif side == -1:
                    y_in = min(-SYM_EPS, y_in)
                else:
                    y_in = 0.0
                poly_state[idx] = [round(x_in, 4), round(y_in, 4)]
                if side == 0:
                    return idx, None
                if partner_idx is None or partner_idx < 0 or partner_idx >= len(poly_state) or partner_idx == idx:
                    idx, partner_idx = _ensure_partner_idx(idx)
                if partner_idx is None or partner_idx < 0 or partner_idx >= len(poly_state) or partner_idx == idx:
                    return idx, None
                poly_state[partner_idx] = [round(x_in, 4), round(-y_in, 4)]
                return idx, partner_idx
            def _delete_idx_with_symmetry(idx):
                """Delete a point and its mirrored partner when symmetry is enabled."""
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return
                if not _symmetry_on():
                    poly_state.pop(idx)
                    return
                mirror_idx = _find_partner_idx(idx)
                if mirror_idx is None:
                    poly_state.pop(idx)
                    return
                if mirror_idx > idx:
                    poly_state.pop(mirror_idx)
                    poly_state.pop(idx)
                else:
                    poly_state.pop(idx)
                    if 0 <= mirror_idx < len(poly_state):
                        poly_state.pop(mirror_idx)
            def _commit_adv(save_now=False):
                """Persist editor points to shared config state."""
                advanced_geometry_state["enabled"] = int(bool(use_custom_var.get()))
                advanced_geometry_state["symmetry"] = int(bool(symmetry_var.get()))
                advanced_geometry_state[active_adv_key] = _sanitize_advanced_poly(poly_state)
                CFG.setdefault("bot_dimensions", {})["advanced_geometry"] = {
                    "enabled": int(bool(advanced_geometry_state.get("enabled", 0))),
                    "symmetry": int(bool(advanced_geometry_state.get("symmetry", 0))),
                    "points": [list(p) for p in advanced_geometry_state.get("points", [])],
                    "reshape_points": [list(p) for p in advanced_geometry_state.get("reshape_points", [])],
                }
                _refresh_advanced_status()
                if save_now:
                    _save_adv_geometry()
            def _nearest_idx(px, py, hit_radius_px=11.0):
                """Return nearest point index within hit radius."""
                best_idx = None
                best_d2 = 1e9
                for idx, pt in enumerate(poly_state):
                    try:
                        sx, sy = _to_canvas(float(pt[0]), float(pt[1]))
                    except Exception:
                        continue
                    d2 = (sx - px) * (sx - px) + (sy - py) * (sy - py)
                    if d2 < best_d2:
                        best_d2 = d2
                        best_idx = idx
                return best_idx if best_d2 <= (float(hit_radius_px) * float(hit_radius_px)) else None
            def _nearest_edge_insert_local(x_in, y_in):
                """Return nearest edge insertion info for a local point."""
                n = len(poly_state)
                if n < 2:
                    return None
                qx = float(x_in)
                qy = float(y_in)
                seg_count = n if n >= 3 else n - 1
                best_d2 = 1e9
                best = None
                for i in range(seg_count):
                    j = (i + 1) % n
                    if n < 3 and j <= i:
                        continue
                    try:
                        ax = float(poly_state[i][0]); ay = float(poly_state[i][1])
                        bx = float(poly_state[j][0]); by = float(poly_state[j][1])
                    except Exception:
                        continue
                    vx = bx - ax
                    vy = by - ay
                    vv = vx * vx + vy * vy
                    if vv <= 1e-12:
                        continue
                    t = ((qx - ax) * vx + (qy - ay) * vy) / vv
                    t = max(0.0, min(1.0, t))
                    proj_x = ax + t * vx
                    proj_y = ay + t * vy
                    d2 = (proj_x - qx) * (proj_x - qx) + (proj_y - qy) * (proj_y - qy)
                    if d2 < best_d2:
                        best_d2 = d2
                        best = {
                            "insert_idx": n if j == 0 else (i + 1),
                            "proj": (proj_x, proj_y),
                        }
                return best
            def _nearest_edge_insert(px, py, hit_radius_px=12.0):
                """Return insertion index + projected local point on nearest edge."""
                x_in, y_in = _to_local(px, py)
                info = _nearest_edge_insert_local(x_in, y_in)
                if not info:
                    return None, None
                proj_local = info["proj"]
                proj_px = _to_canvas(proj_local[0], proj_local[1])
                d2_px = (proj_px[0] - px) * (proj_px[0] - px) + (proj_px[1] - py) * (proj_px[1] - py)
                if d2_px > (float(hit_radius_px) * float(hit_radius_px)):
                    return None, None
                return info["insert_idx"], proj_local
            def _set_outer_rect_points():
                """Set polygon to the current outer rectangle."""
                ow, ol, _ox, _oy = _active_dims_local()
                half_l = max(0.0, float(ol) * 0.5)
                half_w = max(0.0, float(ow) * 0.5)
                poly_state[:] = [
                    [-half_l, -half_w],
                    [half_l, -half_w],
                    [half_l, half_w],
                    [-half_l, half_w],
                ]
                selected.update({"idx": None, "drag": False, "mirror_idx": None, "side": 0})
                _commit_adv(save_now=True)
                _draw_adv()
                draw()
            def _clear_points():
                """Clear polygon points."""
                poly_state[:] = []
                selected.update({"idx": None, "drag": False, "mirror_idx": None, "side": 0})
                _commit_adv(save_now=True)
                _draw_adv()
                draw()
            ttk.Button(btn_row, text="Rect from outer", command=_set_outer_rect_points).pack(side="left")
            ttk.Button(btn_row, text="Clear", command=_clear_points).pack(side="left", padx=(6, 0))
            def _delete_selected(_evt=None):
                """Delete selected polygon point."""
                idx = selected.get("idx")
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return
                _delete_idx_with_symmetry(idx)
                selected.update({"idx": None, "drag": False, "mirror_idx": None, "side": 0})
                _commit_adv(save_now=True)
                _draw_adv()
                draw()
            def _draw_adv():
                """Render advanced polygon editor canvas."""
                adv_canvas.delete("all")
                ow, ol, ox, oy = _active_dims_local()
                dw, dl = _dt_vals()
                view_in = max(24.0, float(ow) + 8.0, float(ol) + 8.0, float(dw) + 10.0, float(dl) + 10.0)
                state["scale"] = (adv_canvas_size * 0.86) / max(1e-6, view_in)
                s = state["scale"]
                cx, cy = state["cx"], state["cy"]
                step = max(2, int(round(s)))
                start = int(cx - (view_in * 0.5) * s)
                end = int(cx + (view_in * 0.5) * s)
                for gx in range(start, end + step, step):
                    fill = "#ececec" if ((gx - start) // step) % 5 else "#d9d9d9"
                    adv_canvas.create_line(gx, start, gx, end, fill=fill)
                for gy in range(start, end + step, step):
                    fill = "#ececec" if ((gy - start) // step) % 5 else "#d9d9d9"
                    adv_canvas.create_line(start, gy, end, gy, fill=fill)
                adv_canvas.create_rectangle(start, start, end, end, outline="#c9c9c9")
                # Outer geometry (centered in local frame).
                ox0 = cx - float(ow) * 0.5 * s
                ox1 = cx + float(ow) * 0.5 * s
                oy0 = cy - float(ol) * 0.5 * s
                oy1 = cy + float(ol) * 0.5 * s
                adv_canvas.create_rectangle(ox0, oy0, ox1, oy1, outline="#3c6cc9", width=2, fill="#dfe9f7")
                # Drivetrain reference rectangle (offset relative to outer center).
                base_cx = cx + float(oy) * s
                base_cy = cy + float(ox) * s
                dx0 = base_cx - float(dw) * 0.5 * s
                dx1 = base_cx + float(dw) * 0.5 * s
                dy0 = base_cy - float(dl) * 0.5 * s
                dy1 = base_cy + float(dl) * 0.5 * s
                adv_canvas.create_rectangle(dx0, dy0, dx1, dy1, outline="#666666", width=2, dash=(4, 2))
                adv_canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#666666", outline="#333333")
                cpts = []
                for pt in poly_state:
                    try:
                        cpts.append(_to_canvas(float(pt[0]), float(pt[1])))
                    except Exception:
                        continue
                if len(cpts) >= 2:
                    for j in range(len(cpts) - 1):
                        adv_canvas.create_line(*cpts[j], *cpts[j + 1], fill="#b72d2d", width=2)
                    if len(cpts) >= 3:
                        adv_canvas.create_line(*cpts[-1], *cpts[0], fill="#b72d2d", width=2)
                for idx, (px, py) in enumerate(cpts):
                    r = 5 if idx == selected.get("idx") else 4
                    fill = "#f1a22e" if idx == selected.get("idx") else "#d86f00"
                    adv_canvas.create_oval(px - r, py - r, px + r, py + r, outline="#7a3d00", fill=fill)
                points_var.set(
                    f"{active_mode_label} points: {len(cpts)} ({'enabled' if bool(use_custom_var.get()) else 'disabled'}, symmetry {'on' if _symmetry_on() else 'off'})"
                )
            def _update_selected_point(px, py):
                """Update selected point using canvas position."""
                idx = selected.get("idx")
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return False
                x_in, y_in = _to_local(px, py)
                x_in, y_in = _clamp_local_point(x_in, y_in)
                side = int(selected.get("side", 0))
                if _symmetry_on() and side == 0:
                    side = _side_from_y(y_in)
                    if side != 0:
                        selected["side"] = side
                poly_state[idx] = [round(x_in, 4), round(y_in, 4)]
                if _symmetry_on():
                    idx, partner = _apply_symmetry_with_partner(idx, selected.get("mirror_idx"), lock_side=selected.get("side", 0))
                    selected["idx"] = idx
                    selected["mirror_idx"] = partner
                return True
            def _on_left_down(evt):
                """Select point or insert a node on nearest polygon edge."""
                if not poly_state:
                    _set_outer_rect_points()
                idx = _nearest_idx(evt.x, evt.y)
                if idx is None:
                    insert_idx, proj_local = _nearest_edge_insert(evt.x, evt.y)
                    if insert_idx is None or proj_local is None:
                        selected.update({"idx": None, "drag": False, "mirror_idx": None, "side": 0})
                        _draw_adv()
                        return
                    x_in, y_in = _clamp_local_point(float(proj_local[0]), float(proj_local[1]))
                    poly_state.insert(insert_idx, [round(x_in, 4), round(y_in, 4)])
                    idx = insert_idx
                side = 0
                if _symmetry_on():
                    try:
                        side = _side_from_y(float(poly_state[idx][1]))
                    except Exception:
                        side = 0
                selected.update({"idx": idx, "drag": True, "mirror_idx": None, "side": side})
                if _symmetry_on() and side != 0:
                    idx, partner = _ensure_partner_idx(idx)
                    selected["idx"] = idx
                    selected["mirror_idx"] = partner
                selected["idx"] = idx
                _update_selected_point(evt.x, evt.y)
                _commit_adv()
                _draw_adv()
                draw()
            def _on_motion(evt):
                """Drag selected point."""
                if not selected.get("drag"):
                    return
                if not _update_selected_point(evt.x, evt.y):
                    return
                _commit_adv()
                _draw_adv()
                draw()
            def _on_release(_evt):
                """Stop drag."""
                selected["drag"] = False
                _commit_adv(save_now=True)
            def _on_right_down(evt):
                """Remove nearest point."""
                idx = _nearest_idx(evt.x, evt.y, hit_radius_px=13.0)
                if idx is None or idx < 0 or idx >= len(poly_state):
                    return
                _delete_idx_with_symmetry(idx)
                selected.update({"idx": None, "drag": False, "mirror_idx": None, "side": 0})
                _commit_adv(save_now=True)
                _draw_adv()
                draw()
            adv_canvas.bind("<Button-1>", _on_left_down)
            adv_canvas.bind("<B1-Motion>", _on_motion)
            adv_canvas.bind("<ButtonRelease-1>", _on_release)
            adv_canvas.bind("<Button-3>", _on_right_down)
            adv_top.bind("<Delete>", _delete_selected)
            _draw_adv()
            def _close_adv():
                """Close advanced editor and push latest state."""
                _commit_adv(save_now=True)
                draw()
                adv_top.destroy()
            adv_top.protocol("WM_DELETE_WINDOW", _close_adv)
        ttk.Button(sidebar, text="Advanced geometry...", command=_open_advanced_geometry_editor).grid(row=19, column=0, sticky="w", pady=(0, 2))
        ttk.Label(sidebar, textvariable=advanced_status_var, foreground="#555555", wraplength=190, justify="left").grid(row=20, column=0, sticky="w", pady=(0, 8))
        _refresh_advanced_status()
        
        normal_margin_widgets = [lbl_left, m_left_entry, lbl_right, m_right_entry, lbl_back, m_back_entry, lbl_front, m_front_entry]
        reshape_margin_widgets = [lbl_lr_r, m_left_r_entry, m_right_r_entry, lbl_fb_r, m_back_r_entry, m_front_r_entry]
        normal_grid = {w: w.grid_info() for w in normal_margin_widgets}
        reshape_grid = {w: w.grid_info() for w in reshape_margin_widgets}
        
        def _current_margins(mode):
            """Handle current margins."""
            ow, ol, ox, oy = _outer_vals(mode)
            dw, dl = _dt_vals()
            ml = max(0.0, ow / 2.0 + oy - dw / 2.0)
            mr = max(0.0, ow / 2.0 - oy - dw / 2.0)
            mb = max(0.0, ol / 2.0 - ox - dl / 2.0)
            mf = max(0.0, ol / 2.0 + ox - dl / 2.0)
            return ml, mr, mb, mf
        
        def _sync_margin_vars(mode):
            """Handle sync margin vars."""
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
            """Set margin mode."""
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
            """Handle bind redraw."""
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
            """Handle outer vals."""
            m = active_mode or mode_var.get()
            if m == "reshape":
                return (_num(rs_w, 0), _num(rs_l, 0), _num(rs_off_x, 0), _num(rs_off_y, 0))
            return (_num(w_full, 0), _num(l_full, 0), _num(full_off_x, 0), _num(full_off_y, 0))
        
        def _set_outer(w=None, l=None, offx=None, offy=None, active_mode=None):
            """Set outer."""
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
            """Handle dt vals."""
            return (_num(dt_w, 0), _num(dt_l, 0))
        
        def _set_dt(w=None, l=None):
            """Set dt."""
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
            """Handle extents."""
            ow, ol, ox, oy = _outer_vals("normal")
            rw, rl, r_ox, r_oy = _outer_vals("reshape")
            dw, dl = _dt_vals()
            def span(w, l, offx, offy):
                """Handle span."""
                return (abs(offy) + w * 0.5, abs(offx) + l * 0.5)
            sx = max(span(ow, ol, ox, oy)[0], span(rw, rl, r_ox, r_oy)[0], dw * 0.5)
            sy = max(span(ow, ol, ox, oy)[1], span(rw, rl, r_ox, r_oy)[1], dl * 0.5)
            return sx, sy
        
        def draw():
            """Handle draw."""
            canvas.delete("all")
            handles.clear()
            base_cx = base_cy = canvas_size / 2
            vis_state["base_c"] = (base_cx, base_cy)
            sx, sy = _extents()
            view_in = 36.0
            scale = (canvas_size * 0.90) / view_in if view_in > 0 else 10.0
            vis_state["scale"] = scale
            
            mode = mode_var.get()
            ow, ol, ox, oy = _outer_vals(mode)
            dw, dl = _dt_vals()
            
            outer_cx = base_cx - oy * scale
            outer_cy = base_cy - ox * scale
            vis_state["outer_c"] = (outer_cx, outer_cy)
            
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
            
            x0 = outer_cx - (ow * 0.5 * scale)
            x1 = outer_cx + (ow * 0.5 * scale)
            y0 = outer_cy - (ol * 0.5 * scale)
            y1 = outer_cy + (ol * 0.5 * scale)
            canvas.create_rectangle(x0, y0, x1, y1, outline="#3c6cc9", width=3, fill="#dfe9f7")
            canvas.create_text((x0 + x1) / 2, y0 - 12, text=f"W: {ow:.2f} in", fill="#1f4b8f")
            canvas.create_text(x1 + 50, (y0 + y1) / 2, text=f"L: {ol:.2f} in", angle=90, fill="#1f4b8f")
            
            dx0 = base_cx - (dw * 0.5 * scale)
            dx1 = base_cx + (dw * 0.5 * scale)
            dy0 = base_cy - (dl * 0.5 * scale)
            dy1 = base_cy + (dl * 0.5 * scale)
            canvas.create_rectangle(dx0, dy0, dx1, dy1, outline="#666666", width=2, dash=(4,2), fill="")
            canvas.create_text(dx0 - 40, (dy0 + dy1) / 2, text=f"Track {dw:.2f}", angle=90, fill="#444444")
            try:
                _refresh_advanced_status()
                adv_enabled = bool(int(advanced_geometry_state.get("enabled", 0)))
            except Exception:
                adv_enabled = False
            if adv_enabled:
                adv_key = "reshape_points" if mode == "reshape" else "points"
                adv_pts = _sanitize_advanced_poly(advanced_geometry_state.get(adv_key, []))
                poly_px = []
                for pt in adv_pts:
                    try:
                        lx = float(pt[0]); ly = float(pt[1])
                    except Exception:
                        continue
                    px = outer_cx - ly * scale
                    py = outer_cy - lx * scale
                    poly_px.append((px, py))
                if len(poly_px) >= 2:
                    for j in range(len(poly_px) - 1):
                        canvas.create_line(*poly_px[j], *poly_px[j + 1], fill="#b72d2d", width=2)
                    if len(poly_px) >= 3:
                        canvas.create_line(*poly_px[-1], *poly_px[0], fill="#b72d2d", width=2)
                for (px, py) in poly_px:
                    canvas.create_oval(px - 3, py - 3, px + 3, py + 3, fill="#b72d2d", outline="#6f1a1a")
            
            handle_color = "#2b5fad"
            for (hx, hy, kind) in [
                (x0, outer_cy, "outer_left"),
                (x1, outer_cy, "outer_right"),
                (outer_cx, y0, "outer_top"),
                (outer_cx, y1, "outer_bottom"),
            ]:
                handles.append((hx, hy, kind))
                canvas.create_oval(hx-6, hy-6, hx+6, hy+6, fill=handle_color, outline="#0f305f")
            
            d_color = "#444444"
            for (hx, hy, kind) in [
                (dx0, base_cy, "drive_left"),
                (dx1, base_cy, "drive_right"),
                (base_cx, dy0, "drive_top"),
                (base_cx, dy1, "drive_bottom"),
            ]:
                handles.append((hx, hy, kind))
                canvas.create_rectangle(hx-5, hy-5, hx+5, hy+5, outline=d_color, fill="#e6e6e6")
            
            margin_left = max(0.0, ow / 2.0 + oy - dw / 2.0)
            margin_right = max(0.0, ow / 2.0 - oy - dw / 2.0)
            margin_back = max(0.0, ol / 2.0 - ox - dl / 2.0)
            margin_front = max(0.0, ol / 2.0 + ox - dl / 2.0)
            _sync_margin_vars(mode)
            info_var.set(f"Margins L/R: {margin_left:.3f} / {margin_right:.3f} in | Front/Back: {margin_front:.3f} / {margin_back:.3f} in")
        
        def _nearest_handle(x, y):
            """Handle nearest handle."""
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
            """Handle apply drag."""
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
            """Handle on press."""
            hit = _nearest_handle(event.x, event.y)
            drag_state["target"] = hit[2] if hit else None
        
        def on_motion(event):
            """Handle on motion."""
            if drag_state.get("target"):
                _apply_drag(drag_state["target"], event)
        
        def on_release(_event):
            """Handle on release."""
            drag_state["target"] = None
        
        canvas.bind("<Button-1>", on_press)
        canvas.bind("<B1-Motion>", on_motion)
        canvas.bind("<ButtonRelease-1>", on_release)
        
        win.protocol("WM_DELETE_WINDOW", lambda: win.destroy())
        _set_margin_mode()
        draw()
        def _close_and_apply():
            """Handle close and apply."""
            try:
                _apply_margins()
            except Exception:
                pass
            try:
                if advanced_editor_win["win"] is not None and advanced_editor_win["win"].winfo_exists():
                    advanced_editor_win["win"].destroy()
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
        off3_btn = ttk.Radiobutton(sel_frame, text="Offset 3", variable=offset_sel, value="3")
        off3_btn.pack(side="left", padx=(6,0))
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
            """Handle dims."""
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
            """Handle current offset."""
            sel = offset_sel.get()
            try:
                if sel == "1":
                    return float(off1.get())
                if sel == "2":
                    return float(off2.get())
                if sel == "3":
                    return float(off3.get())
                return float(custom_offset_var.get())
            except Exception:
                return 0.0
        
        def _set_offset(val):
            """Set offset."""
            sel = offset_sel.get()
            if sel == "1":
                off1.set(f"{val:.3f}")
            elif sel == "2":
                off2.set(f"{val:.3f}")
            elif sel == "3":
                off3.set(f"{val:.3f}")
            else:
                custom_offset_var.set(f"{val:.3f}")
            val_label.config(text=f"Offset: {val:.3f} in")
        
        def draw():
            """Handle draw."""
            canvas.delete("all")
            mode = mode_var.get()
            dw, dl, ow, ol, ox, oy = _dims(mode)
            rev = bool(reverse_var.get())
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
            max_off_y = max(0.0, (view_in / 2.0) - (ow / 2.0))
            max_off_x = max(0.0, (view_in / 2.0) - (ol / 2.0))
            oy = max(-max_off_y, min(max_off_y, oy))
            ox = max(-max_off_x, min(max_off_x, ox))
            base_cx = base_cy = canvas_size / 2
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
            x0 = outer_cx - (ow * 0.5 * scale)
            x1 = outer_cx + (ow * 0.5 * scale)
            y0 = outer_cy - (ol * 0.5 * scale)
            y1 = outer_cy + (ol * 0.5 * scale)
            canvas.create_rectangle(x0, y0, x1, y1, outline="#3c6cc9", width=2, fill="#e6eefb")
            dx0 = base_cx - (dw * 0.5 * scale)
            dx1 = base_cx + (dw * 0.5 * scale)
            dy0 = base_cy - (dl * 0.5 * scale)
            dy1 = base_cy + (dl * 0.5 * scale)
            if rev:
                dy0, dy1 = base_cy - (dl * 0.5 * scale), base_cy + (dl * 0.5 * scale)
            canvas.create_rectangle(dx0, dy0, dx1, dy1, outline="#444444", dash=(4,2), fill="#f7f7f7")
            val = _current_offset()
            node_pos_px[0] = base_cx
            node_pos_px[1] = base_cy - val * scale
            node_pos_px[1] = min(end, max(start, node_pos_px[1]))
            canvas.create_line(base_cx, start, base_cx, end, fill="#bbbbbb", dash=(3,3))
            canvas.create_oval(node_pos_px[0]-7, node_pos_px[1]-7, node_pos_px[0]+7, node_pos_px[1]+7, fill="#2b5fad", outline="#0f305f")
            val_label.config(text=f"Offset: {val:.3f} in")
        
        normal_btn.config(command=draw)
        reshape_btn.config(command=draw)
        off1_btn.config(command=draw)
        off2_btn.config(command=draw)
        off3_btn.config(command=draw)
        custom_btn.config(command=draw)
        
        def _on_drag(event):
            """Handle on drag."""
            base_cy = canvas_size / 2
            val = (base_cy - event.y) / scale
            max_abs = view_in / 2.0
            val = max(-max_abs, min(max_abs, val))
            _set_offset(val)
            draw()
        
        canvas.bind("<B1-Motion>", _on_drag)
        canvas.bind("<Button-1>", _on_drag)
        custom_offset_var.trace_add("write", lambda *_: draw())
        
        def _close_offset():
            """Handle close offset."""
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

        if path_edit_mode and path_edit_segment_idx == 0 and path_control_points:
            cps = list(path_control_points)
            if len(cps) >= 2:
                cps[0] = p0
                cps[-1] = p1
                try:
                    spline_type = _path_spline_type(display_nodes[0].get("path_to_next", {}))
                    path_pts = generate_bezier_path(cps, num_samples=50, spline_type=spline_type)
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
                            spline_type = _path_spline_type(pd)
                            path_pts = generate_bezier_path(cps, num_samples=50, spline_type=spline_type)
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
        
    heading_frame = ttk.Frame(general_body)
    heading_entry = ttk.Entry(heading_frame, textvariable=init_head, width=10)
    heading_entry.pack(side="left", fill="x", expand=True)
    
    def _heading_to_node1():
        """Handle heading to node1."""
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

    def _tutorial_reset_state():
        """Handle tutorial reset state."""
        tutorial_state["flags"] = dict(TUTORIAL_FLAG_DEFAULTS)
        tutorial_state["tabs_seen"].clear()

    def _select_tab(tab_key: str):
        """Handle select tab."""
        frame = tabs.get(tab_key)
        if frame is None:
            return
        try:
            notebook.select(frame)
            tutorial_state["tabs_seen"].add(tab_key)
        except Exception:
            pass

    def _note_active_tab():
        """Handle note active tab."""
        try:
            current = notebook.select()
        except Exception:
            return
        for name, frame in tabs.items():
            if str(frame) == current:
                tutorial_state["tabs_seen"].add(name)
                return

    def _has_offset():
        """Check whether has offset."""
        for i, node in enumerate(display_nodes):
            if i == 0:
                continue
            if int(node.get("offset", 0)) != 0 or node.get("offset_custom_in") is not None:
                return True
        return False

    def _has_reverse():
        """Check whether has reverse."""
        for node in display_nodes:
            if node.get("reverse", False):
                return True
            for act in node.get("actions", []):
                if act.get("type") == "reverse":
                    return True
        return False

    def _has_swing():
        """Check whether has swing."""
        for node in display_nodes:
            if str(node.get("turn_mode", "")).lower() == "swing":
                return True
            for act in node.get("actions", []):
                if act.get("type") == "swing":
                    return True
        return False

    def _has_pose():
        """Check whether has pose."""
        return any(bool(node.get("move_to_pose")) for node in display_nodes)

    def _has_markers():
        """Check whether has markers."""
        for node in display_nodes:
            events = node.get("edge_events", [])
            if isinstance(events, list) and events:
                return True
        return False

    def _open_tutorial_window():
        """Handle open tutorial window."""
        if tutorial_state["win"] is not None and tutorial_state["win"].winfo_exists():
            tutorial_state["win"].lift()
            tutorial_state["win"].focus_force()
            return
        _tutorial_reset_state()
        win = tk.Toplevel(top)
        tutorial_state["win"] = win
        tutorial_state["active"] = True
        win.title("Terminal Interactive Tutorial")
        win.geometry("820x610")
        win.minsize(700, 520)

        palette = {
            "bg": "#f5f7fb",
            "card": "#ffffff",
            "muted": "#5b6270",
            "ok": "#11845b",
            "warn": "#a35f00",
            "line": "#d8dfeb",
        }
        win.configure(bg=palette["bg"])

        outer = tk.Frame(win, bg=palette["bg"], padx=14, pady=14)
        outer.pack(fill="both", expand=True)

        top_card = tk.Frame(outer, bg=palette["card"], highlightbackground=palette["line"], highlightthickness=1)
        top_card.pack(fill="x")
        tk.Label(
            top_card,
            text="Interactive Terminal Tutorial",
            bg=palette["card"],
            fg="#111827",
            font=("Segoe UI", 16, "bold"),
            anchor="w",
        ).pack(fill="x", padx=14, pady=(12, 2))
        tk.Label(
            top_card,
            text=(
                "Follow the guided order from route creation to export and Atticus localization validation. "
                "Each step has live completion checks."
            ),
            bg=palette["card"],
            fg=palette["muted"],
            justify="left",
            anchor="w",
            wraplength=620,
        ).pack(fill="x", padx=14, pady=(0, 10))
        progress_var = tk.StringVar(value="0/0 complete")
        tk.Label(top_card, textvariable=progress_var, bg=palette["card"], fg=palette["muted"], anchor="w").pack(fill="x", padx=14)
        progress_bar = ttk.Progressbar(top_card, mode="determinate")
        progress_bar.pack(fill="x", padx=14, pady=(4, 12))

        body = tk.Frame(outer, bg=palette["bg"])
        body.pack(fill="both", expand=True, pady=(12, 0))

        nav_card = tk.Frame(body, bg=palette["card"], highlightbackground=palette["line"], highlightthickness=1)
        nav_card.pack(side="left", fill="y")
        tk.Label(
            nav_card,
            text="Roadmap",
            bg=palette["card"],
            fg="#111827",
            font=("Segoe UI", 11, "bold"),
            anchor="w",
        ).pack(fill="x", padx=10, pady=(10, 4))
        roadmap_list = tk.Listbox(
            nav_card,
            activestyle="none",
            exportselection=False,
            width=24,
            height=24,
            borderwidth=0,
            highlightthickness=0,
            selectborderwidth=0,
            font=("Segoe UI", 10),
        )
        roadmap_list.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        content = tk.Frame(body, bg=palette["card"], highlightbackground=palette["line"], highlightthickness=1)
        content.pack(side="left", fill="both", expand=True, padx=(12, 0))
        step_title_var = tk.StringVar(value="")
        step_status_var = tk.StringVar(value="")
        summary_var = tk.StringVar(value="")
        body_var = tk.StringVar(value="")

        title_row = tk.Frame(content, bg=palette["card"])
        title_row.pack(fill="x", padx=14, pady=(12, 8))
        tk.Label(
            title_row,
            textvariable=step_title_var,
            bg=palette["card"],
            fg="#111827",
            font=("Segoe UI", 14, "bold"),
            anchor="w",
        ).pack(side="left", fill="x", expand=True)
        status_chip = tk.Label(
            title_row,
            textvariable=step_status_var,
            bg="#fff2d6",
            fg=palette["warn"],
            padx=8,
            pady=4,
            font=("Segoe UI", 9, "bold"),
        )
        status_chip.pack(side="right")

        summary_msg = tk.Message(
            content,
            textvariable=summary_var,
            bg=palette["card"],
            fg="#1f2937",
            justify="left",
            anchor="w",
            width=500,
            font=("Segoe UI", 10),
        )
        summary_msg.pack(fill="x", padx=14)
        body_msg = tk.Message(
            content,
            textvariable=body_var,
            bg=palette["card"],
            fg=palette["muted"],
            justify="left",
            anchor="w",
            width=500,
            font=("Segoe UI", 10),
        )
        body_msg.pack(fill="x", padx=14, pady=(8, 0))

        tk.Label(content, text="Live completion checks", bg=palette["card"], fg="#111827", font=("Segoe UI", 11, "bold"), anchor="w").pack(fill="x", padx=14, pady=(10, 2))
        checks_holder = tk.Frame(content, bg=palette["card"])
        checks_holder.pack(fill="both", expand=True, padx=14, pady=(0, 8))

        action_row = tk.Frame(content, bg=palette["card"])
        action_row.pack(fill="x", padx=14, pady=(2, 8))
        focus_tab_btn = ttk.Button(action_row, text="Focus tab", width=12)
        focus_tab_btn.pack(side="left")
        action_btn = ttk.Button(action_row, text="Do this now")
        action_btn.pack(side="left", padx=(6, 0), fill="x", expand=True)

        nav_row = tk.Frame(content, bg=palette["card"])
        nav_row.pack(fill="x", padx=14, pady=(0, 12))
        back_btn = ttk.Button(nav_row, text="Back", width=10)
        next_btn = ttk.Button(nav_row, text="Next")
        back_btn.pack(side="left")
        next_btn.pack(side="left", padx=(6, 0), fill="x", expand=True)

        initial_heading = init_head.get()
        step_flags = {
            "heading_used": False,
            "opened_geometry_visualizer": False,
            "opened_offset_visualizer": False,
            "reviewed_mcl_tools": False,
            "reviewed_shortcuts": False,
        }

        def _step_heading():
            """Handle step heading."""
            step_flags["heading_used"] = True
            try:
                _heading_to_node1()
            except Exception:
                pass

        def _open_tutorial_reference():
            """Handle open tutorial reference."""
            return
            ref = tk.Toplevel(win)
            ref.title("Terminal Reference (VEX)")
            ref.geometry("860x620")
            ref.minsize(760, 520)

            shell = ttk.Frame(ref, padding=10)
            shell.pack(fill="both", expand=True)
            nb = ttk.Notebook(shell)
            nb.pack(fill="both", expand=True)

            pages = {
                "hotkeys": ttk.Frame(nb, padding=10),
                "rightclick": ttk.Frame(nb, padding=10),
                "commands": ttk.Frame(nb, padding=10),
                "windows": ttk.Frame(nb, padding=10),
                "systems": ttk.Frame(nb, padding=10),
                "workflow": ttk.Frame(nb, padding=10),
            }
            nb.add(pages["hotkeys"], text="Hotkeys")
            nb.add(pages["rightclick"], text="Right-click")
            nb.add(pages["commands"], text="Commands + Params")
            nb.add(pages["windows"], text="Windows")
            nb.add(pages["systems"], text="VEX Systems")
            nb.add(pages["workflow"], text="Full Workflow")

            def _make_readonly_text(parent, content: str):
                """Handle make readonly text."""
                holder = ttk.Frame(parent)
                holder.pack(fill="both", expand=True)
                w = tk.Text(holder, wrap="word")
                s = ttk.Scrollbar(holder, orient="vertical", command=w.yview)
                w.configure(yscrollcommand=s.set)
                w.pack(side="left", fill="both", expand=True)
                s.pack(side="left", fill="y")
                w.insert("1.0", content)
                w.configure(state="disabled")

            hotkeys_lines = ["All major controls:\n"]
            for key, desc in CONTROLS:
                hotkeys_lines.append(f"- {key}: {desc.replace(chr(10), ' ')}")
            hotkeys_lines.append("\nTip: open Settings -> Controls tab for the same list in-table.")
            _make_readonly_text(pages["hotkeys"], "\n".join(hotkeys_lines))

            right_click_text = (
                "Right-click functions are context-sensitive.\n\n"
                "Node actions prompt:\n"
                "- turn <deg>\n"
                "- wait <seconds>\n"
                "- swing <heading> [cw|ccw|auto]\n"
                "- settleswing <heading> [cw|ccw|auto]\n"
                "- offset <inches>\n"
                "- reshape\n"
                "- reverse [on|off]\n"
                "- latspeed <0-127>\n"
                "- turnspeed <deg/s>\n"
                "- chain [0-1|off]\n"
                "- mechanism preset commands like lift/clamp (template placeholders)\n\n"
                "Path-start right-click (curved segment):\n"
                "- set min/max path command speed\n"
                "- optional lookahead override\n\n"
                "Path edit mode right-click:\n"
                "- add control points\n\n"
                "Segment right-click (not on node):\n"
                "- add/edit mechanism markers along the segment timeline"
            )
            _make_readonly_text(pages["rightclick"], right_click_text)

            command_text = (
                "VEX-first command examples and parameters:\n\n"
                "1) turn <deg|auto>\n"
                "   - Purpose: re-orient robot before the next move.\n"
                "   - Example: turn 90\n\n"
                "2) swing <heading_deg> [cw|ccw|auto]\n"
                "   - Purpose: one-side swing turn (fast when space is tight).\n"
                "   - Example: swing 135 cw\n\n"
                "3) settleswing <heading_deg> [cw|ccw|auto]\n"
                "   - Purpose: swing with extra settle behavior for repeatability.\n"
                "   - Example: settleswing 180 ccw\n\n"
                "4) movetopose <heading_deg> [lead_in] / movetopose off\n"
                "   - Purpose: hold a target heading while driving into the node.\n"
                "   - lead_in is a unitless curvature-weight constant for boomerang shaping (not inches).\n"
                "   - Example: movetopose 180 0.6\n\n"
                "5) offset <inches>\n"
                "   - Purpose: shift endpoint along drivetrain centerline (clear goals/triballs/robots).\n"
                "   - Example: offset 6.5\n\n"
                "6) reverse [on|off], latspeed <0-127>, turnspeed <deg/s>, chain [0-1|off]\n"
                "   - Purpose: practical control over direction, command intensity, and chaining behavior.\n\n"
                "7) mechanism presets (template-defined)\n"
                "   - Purpose: emit team-specific commands like clamp/intake/lift at exact points.\n"
                "   - Example: clamp close, intake on, lift high\n\n"
                "Tip: keep commands simple and deterministic for skills/auton reliability."
            )
            _make_readonly_text(pages["commands"], command_text)

            windows_text = (
                "Main windows and tabs:\n\n"
                "- Field canvas (main): nodes, path edit, sim playback.\n"
                "- Output panel (O): generated compile/log output.\n"
                "- Settings window tabs:\n"
                "  * Controls: hotkey table.\n"
                "  * General: units, heading, labels, tutorial.\n"
                "  * Physics: speed/accel and sim behavior.\n"
                "  * Geometry: bot geometry visualizer + offset visualizer.\n"
                "  * Atticus: Atticus localizer config and simulator controls.\n"
                "  * Export: template system, style, path-file options.\n\n"
                "Popup tools:\n"
                "- Template Builder\n"
                "- Geometry Visual Editor\n"
                "- Offset Visual Editor\n"
                "- Localization integration notes"
            )
            _make_readonly_text(pages["windows"], windows_text)

            systems_text = (
                "How this maps to practical VEX systems:\n\n"
                "- Geometry tab:\n"
                "  Set robot footprint and offset behavior so path points match real bot clearances.\n\n"
                "- Physics tab:\n"
                "  Tune speed/accel behavior for realistic timing before localization tuning.\n\n"
                "- Export/templates:\n"
                "  Connect node commands to your library calls (LemLib, custom chassis APIs, subsystem wrappers).\n\n"
                "- Atticus tab:\n"
                "  Configure the Atticus local map-based corrector used by the simulator.\n\n"
                "- Mechanism markers:\n"
                "  Trigger intake/lift/clamp/etc at timeline points without hardcoded delays.\n\n"
                "Rule of thumb:\n"
                "  1) Drive/PID stable first, 2) path behavior second, 3) localization correction third."
            )
            _make_readonly_text(pages["systems"], systems_text)

            workflow_text = (
                "Recommended full terminal workflow:\n\n"
                "1) Build route: place/drag/insert nodes.\n"
                "2) Add node actions and mechanism markers.\n"
                "3) Path edit mode: control points + spline type.\n"
                "4) Tune path speeds/lookahead and run sim.\n"
                "5) Open Geometry tools and verify clearances/offsets.\n"
                "6) Enable Atticus localization and confirm sensor geometry.\n"
                "7) Validate raycast corrections in sim and tune gating/writeback.\n"
                "8) Open Export tab and confirm template style.\n"
                "9) Validate on robot.\n\n"
                "Rule: PID and base drive consistency come before localization correction."
            )
            _make_readonly_text(pages["workflow"], workflow_text)

        def _step_controls_tab():
            """Handle step controls tab."""
            _select_tab("controls")

        def _step_general_tab():
            """Handle step general tab."""
            _select_tab("general")

        def _step_physics_tab():
            """Handle step physics tab."""
            _select_tab("physics")

        def _step_geometry_tab():
            """Handle step geometry tab."""
            _select_tab("geometry")

        def _step_templates():
            """Handle step templates."""
            _select_tab("codegen")
            try:
                _open_template_builder()
            except Exception:
                pass

        def _step_geometry_visualizer():
            """Handle step geometry visualizer."""
            _select_tab("geometry")
            try:
                open_geometry_visualizer()
                step_flags["opened_geometry_visualizer"] = True
            except Exception:
                pass

        def _step_offset_visualizer():
            """Handle step offset visualizer."""
            _select_tab("geometry")
            try:
                open_offset_visualizer()
                step_flags["opened_offset_visualizer"] = True
            except Exception:
                pass

        def _step_mcl_tools_overview():
            """Handle step mcl tools overview."""
            _select_tab("mcl")
            step_flags["reviewed_mcl_tools"] = True
            try:
                messagebox.showinfo(
                    "Atticus Localizer",
                    "Use the Atticus tab to configure sensor geometry, range gating, and bounded writeback for the simulator."
                )
            except Exception:
                pass

        def _step_shortcuts_reviewed():
            """Handle step shortcuts reviewed."""
            step_flags["reviewed_shortcuts"] = True

        def _templates_done():
            """Handle templates done."""
            style_norm = str(codegen_style_var.get()).strip().lower()
            if style_norm in ("action list", "actionlist", "list"):
                return "codegen" in tutorial_state["tabs_seen"]
            return tutorial_state["flags"]["opened_template_editor"]

        steps = [
            {
                "title": "Controls tab",
                "summary": "Open Controls so your team is using the exact key map in this build.",
                "body": "Press the button or click Settings -> Controls. In VEX practice, verify Space, Ctrl+Space, P, M, S, L, C, O first.",
                "action_label": "Open Controls tab",
                "action": _step_controls_tab,
                "focus_tab": "controls",
                "checks": [("Controls tab visited", lambda: "controls" in tutorial_state["tabs_seen"])],
            },
            {
                "title": "Place first node",
                "summary": "Left-click to place Node 1 (Node 0 is your start).",
                "body": "Place at least one new node.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("At least one node added", lambda: len(display_nodes) > 1)],
            },
            {
                "title": "Set heading",
                "summary": "Set a deliberate start heading before route tuning.",
                "body": "Use the helper button or type heading manually. This should match how your robot is placed on the field tile.",
                "action_label": "Face Node 1",
                "action": _step_heading,
                "focus_tab": "general",
                "checks": [("Heading changed or helper used", lambda: step_flags["heading_used"] or init_head.get() != initial_heading)],
            },
            {
                "title": "Drag and lock",
                "summary": "Practice drag edits and Ctrl axis lock for quick clean geometry.",
                "body": "Drag once, then do at least one Ctrl-locked drag.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [
                    ("Node dragged", lambda: tutorial_state["flags"]["dragged_node"]),
                    ("Axis lock used", lambda: tutorial_state["flags"]["axis_lock_used"]),
                ],
            },
            {
                "title": "Shift insert",
                "summary": "Insert a node in an existing segment without rebuilding.",
                "body": "Hold Shift and click a segment.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Shift insert used", lambda: tutorial_state["flags"]["shift_insert"])],
            },
            {
                "title": "Node right-click actions",
                "summary": "Right-click node prompts are where turn/wait/swing/offset/reverse commands live.",
                "body": "Open any node prompt and enter a command. Example: 'turn 90; wait 0.2' or 'swing 135 cw'.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Node prompt used", lambda: tutorial_state["flags"]["right_click_prompt"])],
            },
            {
                "title": "Offsets and reverse",
                "summary": "Configure practical spacing and direction behavior for match tasks.",
                "body": "Set at least one offset and one reverse value. Offsets are useful when you want the bot to stop or align at a specific distance from its center, which makes matchloading, scoring, and approach setup much easier. Example: 'offset 6' then 'reverse on'.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Offset configured", _has_offset), ("Reverse configured", _has_reverse)],
            },
            {
                "title": "Swing and move-to-pose",
                "summary": "Cover both heading realization modes used in real auton routines.",
                "body": "Add at least one swing and one move-to-pose. Example: 'swing 180 ccw' then 'movetopose 180 0.6' to approach while holding heading.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Swing present", _has_swing), ("Move-to-pose present", _has_pose)],
            },
            {
                "title": "Mechanism markers",
                "summary": "Markers time subsystem actions directly on segment travel.",
                "body": "Add at least one marker with M or segment right-click. Use markers for intake/clamp/lift timing instead of fixed delays.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Marker added", lambda: _has_markers() or tutorial_state["flags"]["mech_marker"])],
            },
            {
                "title": "Path edit and spline",
                "summary": "Shape curved travel and compare spline behavior.",
                "body": "Enter path edit mode (P) and toggle spline type (U).",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [
                    ("Path edit used", lambda: tutorial_state["flags"]["entered_path_edit"]),
                    ("Spline toggled", lambda: tutorial_state["flags"]["spline_toggled"]),
                ],
            },
            {
                "title": "Path speed and sim",
                "summary": "Tune per-segment speed and validate with simulation.",
                "body": "Use path speed prompt and run simulation once. Example start values: min 45, max 95, then adjust for consistency before match.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [
                    ("Path speed prompt used", lambda: tutorial_state["flags"]["path_speed_prompt"]),
                    ("Simulation started", lambda: tutorial_state["flags"]["ran_sim"]),
                ],
            },
            {
                "title": "General tab",
                "summary": "Review units, heading, labels, and core routine settings.",
                "body": "Visit General tab.",
                "action_label": "Open General tab",
                "action": _step_general_tab,
                "focus_tab": "general",
                "checks": [("General tab visited", lambda: "general" in tutorial_state["tabs_seen"])],
            },
            {
                "title": "Physics tab",
                "summary": "Review simulation speed and behavior settings.",
                "body": "Visit Physics tab. Keep this aligned to your real drivetrain so ETA and settle behavior stay realistic.",
                "action_label": "Open Physics tab",
                "action": _step_physics_tab,
                "focus_tab": "physics",
                "checks": [("Physics tab visited", lambda: "physics" in tutorial_state["tabs_seen"])],
            },
            {
                "title": "Geometry tab",
                "summary": "Open geometry tools for practical drivetrain clearances.",
                "body": "Visit Geometry tab. Robot width/length and wheelbase assumptions directly affect practical path safety margins.",
                "action_label": "Open Geometry tab",
                "action": _step_geometry_tab,
                "focus_tab": "geometry",
                "checks": [("Geometry tab visited", lambda: "geometry" in tutorial_state["tabs_seen"])],
            },
            {
                "title": "Geometry visualizer",
                "summary": "Verify robot footprint behavior in the geometry visual editor.",
                "body": "Launch geometry visualizer from Geometry tab. This is where reshape geometry belongs: use it when a mechanism changes the robot hitbox (for example, an intake or matchloader deployed beyond the frame). If a change is rare and not likely to contact anything, you can usually skip modeling it and instead using a mechanism node (M/RClick Segment)",
                "action_label": "Open geometry visualizer",
                "action": _step_geometry_visualizer,
                "focus_tab": "geometry",
                "checks": [("Geometry visualizer opened", lambda: step_flags["opened_geometry_visualizer"])],
            },
            {
                "title": "Offset visualizer",
                "summary": "Set and validate practical offset presets for autonomous spacing.",
                "body": "Launch offset visualizer from Geometry tab.",
                "action_label": "Open offset visualizer",
                "action": _step_offset_visualizer,
                "focus_tab": "geometry",
                "checks": [("Offset visualizer opened", lambda: step_flags["opened_offset_visualizer"])],
            },
            {
                "title": "Atticus localizer",
                "summary": "Review the Atticus localization controls directly in-app.",
                "body": "Visit the Atticus tab and review sensor geometry, gating, and writeback settings.",
                "action_label": "Show Atticus overview",
                "action": _step_mcl_tools_overview,
                "focus_tab": "mcl",
                "checks": [
                    ("Atticus tab visited", lambda: "mcl" in tutorial_state["tabs_seen"]),
                    ("Atticus overview opened", lambda: step_flags["reviewed_mcl_tools"]),
                ],
            },
            {
                "title": "Export and templates",
                "summary": "Confirm export style and open template builder when needed.",
                "body": "Open Export tab and template editor.",
                "action_label": "Open template editor",
                "action": _step_templates,
                "focus_tab": "codegen",
                "checks": [("Export workflow visited", _templates_done)],
            },
            {
                "title": "Save/load and output",
                "summary": "Practice the repeatable team loop before you finish.",
                "body": "Use S and L once, press C once to compile, then acknowledge output/export hotkeys.",
                "action_label": "Acknowledge shortcut checklist",
                "action": _step_shortcuts_reviewed,
                "focus_tab": "controls",
                "checks": [
                    ("Save/load used", lambda: tutorial_state["flags"]["save_load_used"]),
                    ("Compiled once with C", lambda: tutorial_state["flags"]["compiled_once"]),
                    ("Output/export checklist acknowledged", lambda: step_flags["reviewed_shortcuts"]),
                ],
            },
            {
                "title": "Finish",
                "summary": "You covered full terminal usage from route creation to Atticus localization validation and export.",
                "body": "Press Finish to close. Reopen any time from General tab.",
                "action_label": None,
                "action": None,
                "focus_tab": None,
                "checks": [("Ready", lambda: True)],
            },
        ]

        state = {"idx": 0}
        roadmap_guard = {"busy": False}

        def _safe_check(fn):
            """Handle safe check."""
            try:
                return bool(fn())
            except Exception:
                return False

        def _check_rows(step):
            """Handle check rows."""
            return [(label, _safe_check(fn)) for label, fn in step.get("checks", [])]

        def _is_step_done(step):
            """Check whether is step done."""
            rows = _check_rows(step)
            return bool(rows) and all(ok for _, ok in rows)

        def _completion_frontier():
            """Handle completion frontier."""
            frontier = 0
            for i, step in enumerate(steps):
                if _is_step_done(step):
                    frontier = i + 1
                else:
                    break
            if frontier >= len(steps):
                return len(steps) - 1
            return frontier

        def _refresh_roadmap():
            """Handle refresh roadmap."""
            selected = state["idx"]
            frontier = _completion_frontier()
            roadmap_guard["busy"] = True
            roadmap_list.delete(0, "end")
            for i, step in enumerate(steps):
                done = _is_step_done(step)
                if i == selected:
                    prefix = ">"
                elif done:
                    prefix = "[x]"
                elif i <= frontier:
                    prefix = "[ ]"
                else:
                    prefix = "-"
                roadmap_list.insert("end", f"{prefix} {i + 1:02d}. {step['title']}")
                try:
                    if done:
                        roadmap_list.itemconfig(i, fg=palette["ok"])
                    elif i <= frontier:
                        roadmap_list.itemconfig(i, fg="#111827")
                    else:
                        roadmap_list.itemconfig(i, fg="#97a0b3")
                except Exception:
                    pass
            roadmap_list.selection_clear(0, "end")
            roadmap_list.selection_set(selected)
            roadmap_list.activate(selected)
            roadmap_guard["busy"] = False

        def _render_checks(step):
            """Handle render checks."""
            for child in checks_holder.winfo_children():
                child.destroy()
            for label, ok in _check_rows(step):
                row = tk.Frame(checks_holder, bg=palette["card"])
                row.pack(fill="x", pady=1)
                icon = "[x]" if ok else "[ ]"
                color = palette["ok"] if ok else palette["warn"]
                tk.Label(row, text=icon, bg=palette["card"], fg=color, font=("Segoe UI", 10, "bold")).pack(side="left")
                tk.Label(row, text=label, bg=palette["card"], fg="#1f2937", anchor="w").pack(side="left", fill="x", expand=True, padx=(6, 0))

        def _apply_step(idx: int):
            """Handle apply step."""
            idx = max(0, min(idx, len(steps) - 1))
            state["idx"] = idx
            step = steps[idx]
            step_title_var.set(step["title"])
            summary_var.set(step.get("summary", ""))
            body_var.set(step.get("body", ""))

            done = _is_step_done(step)
            if done:
                step_status_var.set("Complete")
                status_chip.configure(bg="#ddf7eb", fg=palette["ok"])
            else:
                step_status_var.set("In progress")
                status_chip.configure(bg="#fff2d6", fg=palette["warn"])

            if step["action"] is None:
                action_btn.configure(state="disabled", text="No action for this step", command=lambda: None)
            else:
                action_btn.configure(text=step["action_label"], state="normal", command=step["action"])

            focus_tab = step.get("focus_tab")
            if focus_tab:
                focus_tab_btn.configure(state="normal", command=lambda key=focus_tab: _select_tab(key))
            else:
                focus_tab_btn.configure(state="disabled", command=lambda: None)

            back_btn.configure(state=("normal" if idx > 0 else "disabled"))
            next_btn.configure(text=("Finish" if idx == len(steps) - 1 else "Next"))
            _render_checks(step)
            _refresh_roadmap()

        def _refresh_state():
            """Handle refresh state."""
            if not win.winfo_exists():
                return
            _note_active_tab()

            total = len(steps)
            completed = sum(1 for step in steps if _is_step_done(step))
            progress_var.set(f"{completed}/{total} complete")
            progress_bar.configure(value=(100.0 * completed / max(1, total)))

            step = steps[state["idx"]]
            next_btn.configure(state=("normal" if _is_step_done(step) else "disabled"))
            _apply_step(state["idx"])
            win.after(220, _refresh_state)

        def _back():
            """Handle back."""
            _apply_step(state["idx"] - 1)

        def _next():
            """Handle next."""
            if state["idx"] >= len(steps) - 1:
                _close()
                return
            _apply_step(state["idx"] + 1)

        def _on_roadmap_select(_event=None):
            """Handle on roadmap select."""
            if roadmap_guard["busy"]:
                return
            sel = roadmap_list.curselection()
            if not sel:
                return
            target = int(sel[0])
            if target <= _completion_frontier():
                _apply_step(target)
            else:
                _refresh_roadmap()

        def _on_resize(_event=None):
            """Handle on resize."""
            width = max(340, content.winfo_width() - 42)
            summary_msg.configure(width=width)
            body_msg.configure(width=width)

        def _close():
            """Handle close."""
            tutorial_state["active"] = False
            tutorial_state["win"] = None
            try:
                win.destroy()
            except Exception:
                pass

        roadmap_list.bind("<<ListboxSelect>>", _on_roadmap_select)
        content.bind("<Configure>", _on_resize)
        back_btn.configure(command=_back)
        next_btn.configure(command=_next)
        win.protocol("WM_DELETE_WINDOW", _close)
        _apply_step(0)
        _on_resize()
        win.update_idletasks()
        req_w = max(700, min(930, win.winfo_reqwidth() + 10))
        req_h = max(540, min(700, win.winfo_reqheight() + 10))
        win.geometry(f"{req_w}x{req_h}")
        _refresh_state()

    tutorial_btn = ttk.Button(general_body, text="Open tutorial", command=_open_tutorial_window)
    
    _row(general_body, 0, "Distance Units:", ttk.Combobox(general_body, textvariable=dist_var, values=dist_labels, state="readonly"), "Output units")
    _row(general_body, 1, "Angle Units:", ttk.Combobox(general_body, textvariable=ang_var, values=ang_labels, state="readonly"), "Angle units")
    _row(general_body, 2, "Initial heading (deg):", heading_frame, "Robot start heading")
    hitbox_row = ttk.Frame(general_body)
    hitbox_chk = ttk.Checkbutton(hitbox_row, variable=show_hitboxes_var)
    hitbox_chk.pack(side="left")
    hitbox_conflicts_lbl = ttk.Label(hitbox_row, text="Collisions only")
    hitbox_conflicts_lbl.pack(side="left", padx=(10, 4))
    hitbox_conflicts_chk = ttk.Checkbutton(hitbox_row, variable=show_hitbox_conflicts_only_var)
    hitbox_conflicts_chk.pack(side="left")
    _row(general_body, 3, "Show node hitboxes:", hitbox_row, "Toggle geometry boxes")
    def _refresh_hitbox_conflicts_state(*_):
        """Handle refresh hitbox conflicts state."""
        enabled = bool(show_hitboxes_var.get())
        state = "normal" if enabled else "disabled"
        try:
            hitbox_conflicts_chk.configure(state=state)
            hitbox_conflicts_lbl.configure(state=state)
        except Exception:
            pass
        if not enabled:
            show_hitbox_conflicts_only_var.set(0)
    _refresh_hitbox_conflicts_state()
    try:
        show_hitboxes_var.trace_add("write", _refresh_hitbox_conflicts_state)
    except Exception:
        pass
    field_obj_row = ttk.Frame(general_body)
    field_obj_chk = ttk.Checkbutton(field_obj_row, variable=show_field_objects_var)
    field_obj_chk.pack(side="left")
    node_nums_lbl = ttk.Label(field_obj_row, text="Node numbers")
    node_nums_lbl.pack(side="left", padx=(10, 4))
    node_nums_chk = ttk.Checkbutton(field_obj_row, variable=show_node_numbers_var)
    node_nums_chk.pack(side="left")
    ui.track_live_widget(hitbox_chk)
    ui.track_live_widget(hitbox_conflicts_chk)
    ui.track_live_widget(field_obj_chk)
    ui.track_live_widget(node_nums_chk)
    _row(general_body, 4, "Show field objects:", field_obj_row, "Draw field objects / toggle node labels")
    _row(general_body, 5, "Reshape label:", ttk.Entry(general_body, textvariable=reshape_label_var), "Custom label used anywhere reshape appears")
    _row(general_body, 6, "Flip routine:", ttk.Button(general_body, text="Flip", command=_flip_routine_horizontal), "Mirror horizontally")
    _row(general_body, 7, "Tutorial:", tutorial_btn, "Open the guided terminal tutorial.")

    attlib_sim_cfg_ui = CFG.get("codegen", {}).get("attlib_sim", {})
    attlib_sim_visual_var = tk.IntVar(value=int(attlib_sim_cfg_ui.get("visual_run", int(attlib_sim_visual_run))))
    attlib_sim_module_dir_var = tk.StringVar(value=str(attlib_sim_cfg_ui.get("module_dir", attlib_sim_module_dir)))
    attlib_sim_external_var = tk.IntVar(
        value=1 if str(attlib_sim_cfg_ui.get("source", attlib_sim_source)).strip().lower() in ("external", "external_script", "script") else 0
    )
    attlib_sim_script_var = tk.StringVar(value=str(attlib_sim_cfg_ui.get("routine_script", attlib_sim_routine_script)))
    attlib_sim_func_var = tk.StringVar(value=str(attlib_sim_cfg_ui.get("routine_function", attlib_sim_routine_func or "run_routine")))

    attlib_sim_frame = ttk.LabelFrame(general_body, text="AttLibSim Export Helper")
    attlib_sim_frame.grid(row=8, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 2))
    attlib_sim_frame.columnconfigure(0, weight=1)
    attlib_sim_text = (
        "AttLibSim can run either timeline replay or an external routine script.\n"
        "External routine mode is intended for library testing parity."
    )
    attlib_sim_lbl = ttk.Label(attlib_sim_frame, text=attlib_sim_text, justify="left", wraplength=760)
    attlib_sim_lbl.grid(row=0, column=0, sticky="w", padx=8, pady=(6, 4))

    attlib_sim_visual_chk = ttk.Checkbutton(
        attlib_sim_frame,
        text="Run SPACE visual sim through AttLibSim (Atticus/AttLib style)",
        variable=attlib_sim_visual_var,
    )
    attlib_sim_visual_chk.grid(row=1, column=0, sticky="w", padx=8, pady=(0, 4))

    attlib_sim_external_chk = ttk.Checkbutton(
        attlib_sim_frame,
        text="Use external routine script (ignore plotted routine nodes)",
        variable=attlib_sim_external_var,
    )
    attlib_sim_external_chk.grid(row=2, column=0, sticky="w", padx=8, pady=(2, 4))

    module_dir_row = ttk.Frame(attlib_sim_frame)
    module_dir_row.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 4))
    module_dir_row.columnconfigure(0, weight=1)
    module_dir_entry = ttk.Entry(module_dir_row, textvariable=attlib_sim_module_dir_var)
    module_dir_entry.grid(row=0, column=0, sticky="ew")

    def _browse_attlib_sim_module_dir():
        """Pick a folder containing attlib_sim*.pyd."""
        start_dir = attlib_sim_module_dir_var.get() or os.getcwd()
        chosen = filedialog.askdirectory(
            parent=top,
            initialdir=start_dir if os.path.isdir(start_dir) else os.getcwd(),
            title="Select attlib_sim module directory",
        )
        if chosen:
            attlib_sim_module_dir_var.set(chosen)
            on_update()

    ttk.Button(module_dir_row, text="Browse...", command=_browse_attlib_sim_module_dir).grid(
        row=0, column=1, padx=(6, 0)
    )

    script_row = ttk.Frame(attlib_sim_frame)
    script_row.grid(row=4, column=0, sticky="ew", padx=8, pady=(0, 4))
    script_row.columnconfigure(0, weight=1)
    script_entry = ttk.Entry(script_row, textvariable=attlib_sim_script_var)
    script_entry.grid(row=0, column=0, sticky="ew")

    def _browse_attlib_sim_script():
        """Pick an external AttLibSim routine script."""
        start_file = attlib_sim_script_var.get().strip()
        start_dir = os.path.dirname(start_file) if start_file and os.path.isfile(start_file) else os.getcwd()
        chosen = filedialog.askopenfilename(
            parent=top,
            initialdir=start_dir,
            title="Select AttLibSim routine script",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")],
        )
        if chosen:
            attlib_sim_script_var.set(chosen)
            on_update()

    ttk.Button(script_row, text="Browse...", command=_browse_attlib_sim_script).grid(
        row=0, column=1, padx=(6, 0)
    )

    func_row = ttk.Frame(attlib_sim_frame)
    func_row.grid(row=5, column=0, sticky="ew", padx=8, pady=(0, 6))
    ttk.Label(func_row, text="Routine function:").pack(side="left")
    func_entry = ttk.Entry(func_row, textvariable=attlib_sim_func_var, width=24)
    func_entry.pack(side="left", padx=(6, 0))

    def _refresh_attlib_external_state(*_):
        """Enable script/function inputs only for external routine mode."""
        state = "normal" if bool(attlib_sim_external_var.get()) else "disabled"
        try:
            script_entry.configure(state=state)
            func_entry.configure(state=state)
        except Exception:
            pass

    _refresh_attlib_external_state()
    try:
        attlib_sim_external_var.trace_add("write", _refresh_attlib_external_state)
    except Exception:
        pass

    def _copy_attlib_sim_snippet():
        """Copy a minimal AttLibSim terminal runner snippet."""
        snippet = (
            "import os, sys\n"
            "os.add_dll_directory(r\"C:\\\\Strawberry\\\\c\\\\bin\")  # if needed\n"
            "sys.path.insert(0, r\"host/build\")\n"
            "import attlib_sim\n\n"
            "cfg = attlib_sim.SimDriveIOConfig()\n"
            "ch = attlib_sim.SimChassis(cfg)\n"
            "ch.set_pose(0.0, 0.0, 0.0)\n\n"
            "# run exported routine calls here\n"
            "# default is synchronous (async=False)\n\n"
            "pose = ch.get_pose()\n"
            "stats = ch.get_runtime_stats()\n"
            "trace = ch.consume_control_trace()\n"
            "ch.stop()\n"
        )
        try:
            top.clipboard_clear()
            top.clipboard_append(snippet)
            cal_status_var.set("AttLibSim snippet copied to clipboard.")
        except Exception:
            pass

    attlib_sim_btns = ttk.Frame(attlib_sim_frame)
    attlib_sim_btns.grid(row=6, column=0, sticky="w", padx=8, pady=(0, 8))
    ttk.Button(attlib_sim_btns, text="Copy Python snippet", command=_copy_attlib_sim_snippet).pack(side="left")
    ui.track_live_widget(attlib_sim_visual_chk)
    ui.track_live_widget(attlib_sim_external_chk)
    ui.track_live_widget(module_dir_entry)
    ui.track_live_widget(script_entry)
    ui.track_live_widget(func_entry)
    
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
        """Handle physics scroll active."""
        return bool(adv_motion_var.get())

    def _refresh_physics_scrollbar():
        """Handle refresh physics scrollbar."""
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
        """Handle physics on configure."""
        try:
            physics_canvas.configure(scrollregion=physics_canvas.bbox("all"))
            physics_canvas.itemconfigure(physics_win, width=physics_canvas.winfo_width())
        except Exception:
            pass
        _refresh_physics_scrollbar()

    def _physics_mousewheel(event):
        """Handle physics mousewheel."""
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

    _row(physics_body, 0, "Wheel RPM:", ttk.Entry(physics_body, textvariable=rpm_var), "Affects vmax and accel")
    _row(physics_body, 1, "Wheel diameter (in):", ttk.Entry(physics_body, textvariable=diam_var), "Wheel size")
    _row(physics_body, 2, "Robot weight (lb):", ttk.Entry(physics_body, textvariable=w_var), "Robot mass")
    _row(physics_body, 3, "Buffer Time (s):", ttk.Entry(physics_body, textvariable=tb_var), "Pause per node")
    _row(physics_body, 4, "Gear ratio:", ttk.Entry(physics_body, textvariable=gr_var), "Motor:wheel ratio")
    _row(physics_body, 5, "Point density (/in):", ttk.Entry(physics_body, textvariable=dens_var), "Uniform resample density for curved paths")
    curv_disp = tk.StringVar(value=f"{curv_var.get():.3f}")
    def _curv_live(_=None):
        """Handle curv live."""
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
        """Handle phys const defaults."""
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
        """Handle derive cal constants from dynamics."""
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
        """Handle open physics constants editor."""
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
            """Handle load const values."""
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
            """Handle refresh dyn labels."""
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
                """Handle to float."""
                if isinstance(raw, dict):
                    raw = raw.get("value", None)
                try:
                    return float(raw)
                except Exception:
                    return None

            def _get_dyn_val(key):
                """Return dyn val."""
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
            """Handle refresh constants lock."""
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
            """Handle reset defaults."""
            defaults = _phys_const_defaults()
            for key, var in const_vars.items():
                var.set(str(defaults.get(key, 0.0)))

        def _apply_constants():
            """Handle apply constants."""
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
        """Handle motion profile defaults."""
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
        """Handle open motion profile editor."""
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
            """Handle profile on configure."""
            try:
                profile_canvas.configure(scrollregion=profile_canvas.bbox("all"))
                profile_canvas.itemconfigure(body_id, width=profile_canvas.winfo_width())
            except Exception:
                pass

        def _profile_mousewheel(event):
            """Handle profile mousewheel."""
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
            """Handle toggle guide."""
            if guide_visible.get():
                guide_box.pack_forget()
                guide_visible.set(False)
                guide_toggle_btn.configure(text="?")
            else:
                guide_box.pack(fill="x", pady=(0, 8), before=cap_lock_frame)
                guide_visible.set(True)
                guide_toggle_btn.configure(text="×")

        def _guide_mousewheel(event):
            """Handle guide mousewheel."""
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
            """Handle make json box."""
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
            """Handle pick single cap."""
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
            """Handle coerce single caps."""
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
            """Handle caps are single."""
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
            """Handle on single cap toggle."""
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
            """Handle load values."""
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
            """Handle reset defaults."""
            _load_values(_motion_profile_defaults())

        def _apply_profiles():
            """Handle apply profiles."""
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
    _row(physics_body, 8, "Advanced motion vars (EXPERIMENTAL):", adv_frame, "Enable JAR-style voltage/settle placeholders and profile hover info")

    cal_frame = None
    cal_export_btn = None

    def _refresh_adv_visibility():
        """Handle refresh adv visibility."""
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

    sensor_vis_win = {"win": None}
    _row(tabs["geometry"], 0, "Bot Geometry Visualizer:", ttk.Button(tabs["geometry"], text="Open visual editor", command=open_geometry_visualizer), "Drag-to-edit drivetrain and robot geometry, including reshape.")
    _row(tabs["geometry"], 1, "Offset 1 (in):", ttk.Entry(tabs["geometry"], textvariable=off1), "First preset")
    _row(tabs["geometry"], 2, "Offset 2 (in):", ttk.Entry(tabs["geometry"], textvariable=off2), "Second preset")
    _row(tabs["geometry"], 3, "Offset 3 (in):", ttk.Entry(tabs["geometry"], textvariable=off3), "Third preset")
    _row(tabs["geometry"], 4, "Wall padding (in):", ttk.Entry(tabs["geometry"], textvariable=pad), "Boundary margin")
    _row(tabs["geometry"], 5, "Offset Visualizer:", ttk.Button(tabs["geometry"], text="Open offset editor", command=open_offset_visualizer), "Drag node along centerline to set offsets.")

    tabs["mcl"].rowconfigure(0, weight=1)
    tabs["mcl"].columnconfigure(0, weight=1)
    mcl_canvas = tk.Canvas(tabs["mcl"], borderwidth=0, highlightthickness=0)
    mcl_body = ttk.Frame(mcl_canvas)
    mcl_body.columnconfigure(0, weight=1)
    mcl_body.columnconfigure(1, weight=1)
    mcl_scroll = ttk.Scrollbar(tabs["mcl"], orient="vertical", command=mcl_canvas.yview)
    mcl_canvas.configure(yscrollcommand=mcl_scroll.set)
    mcl_canvas.grid(row=0, column=0, sticky="nsew")
    mcl_scroll.grid(row=0, column=1, sticky="ns")
    mcl_win = mcl_canvas.create_window((0, 0), window=mcl_body, anchor="nw")

    def _mcl_on_configure(_evt=None):
        """Handle mcl on configure."""
        mcl_canvas.configure(scrollregion=mcl_canvas.bbox("all"))
        mcl_canvas.itemconfigure(mcl_win, width=mcl_canvas.winfo_width())

    def _mcl_mousewheel(event):
        """Handle mcl mousewheel."""
        try:
            if hasattr(event, "delta") and event.delta:
                delta = int(-event.delta / 120)
            elif getattr(event, "num", None) == 4:
                delta = -1
            elif getattr(event, "num", None) == 5:
                delta = 1
            else:
                delta = 0
            if delta:
                mcl_canvas.yview_scroll(delta, "units")
        except Exception:
            pass

    mcl_body.bind("<Configure>", _mcl_on_configure)
    mcl_canvas.bind("<Configure>", _mcl_on_configure)
    for w in (mcl_canvas, mcl_body):
        w.bind("<MouseWheel>", _mcl_mousewheel)
        w.bind("<Button-4>", _mcl_mousewheel)
        w.bind("<Button-5>", _mcl_mousewheel)

    core_frame = ttk.LabelFrame(mcl_body, text="Atticus Runtime")
    core_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 8))
    core_frame.columnconfigure(1, weight=1)
    r = 0
    _row(core_frame, r, "Enable Atticus:", ttk.Checkbutton(core_frame, variable=mcl_enabled_var),
         "Enable or disable the Atticus localizer in simulation."); r += 1
    _row(
        core_frame,
        r,
        "Export pose convention:",
        ttk.Combobox(
            core_frame,
            values=["cw_zero_forward", "atticus", "ccw_zero_forward"],
            textvariable=mcl_pose_convention_var,
            state="readonly",
        ),
        "Pose convention used when interpreting external pose adapters.",
    ); r += 1
    _row(
        core_frame,
        r,
        "External swap X/Y:",
        ttk.Checkbutton(core_frame, variable=mcl_pose_swap_xy_var),
        "Swap external pose X and Y axes before conversion into the internal localizer frame.",
    ); r += 1
    _row(
        core_frame,
        r,
        "External invert X:",
        ttk.Checkbutton(core_frame, variable=mcl_pose_invert_x_var),
        "Invert external X axis before conversion into the internal localizer frame.",
    ); r += 1
    _row(
        core_frame,
        r,
        "External invert Y:",
        ttk.Checkbutton(core_frame, variable=mcl_pose_invert_y_var),
        "Invert external Y axis before conversion into the internal localizer frame.",
    ); r += 1
    _row(
        core_frame,
        r,
        "Horizontal odom:",
        ttk.Combobox(
            core_frame,
            values=["none", "enabled"],
            textvariable=mcl_horizontal_odom_var,
            state="readonly",
        ),
        "Match the real Atticus runtime: choose whether the horizontal tracking wheel exists.",
    ); r += 1
    _row(core_frame, r, "Motion update ms:", ttk.Entry(core_frame, textvariable=mcl_motion_ms_var),
         "Prediction loop interval (ms). Real Atticus defaults to 20."); r += 1
    _row(core_frame, r, "Sensor update ms:", ttk.Entry(core_frame, textvariable=mcl_sensor_ms_var),
         "Distance/IMU correction interval (ms). Real Atticus defaults to 20."); r += 1

    motion_frame = ttk.LabelFrame(mcl_body, text="Odometry Prior")
    motion_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    motion_frame.columnconfigure(1, weight=1)
    r = 0
    _row(motion_frame, r, "Enable motion:", ttk.Checkbutton(motion_frame, variable=mcl_motion_enabled_var),
         "Enable odometry-delta prediction for the Atticus localizer."); r += 1
    mcl_row_sigma_x = _row(motion_frame, r, "Base sigma right (in):", ttk.Entry(motion_frame, textvariable=mcl_sigma_x_var),
         "Base lateral process sigma in the real Atticus body-right axis."); r += 1
    mcl_row_sigma_y = _row(motion_frame, r, "Base sigma forward (in):", ttk.Entry(motion_frame, textvariable=mcl_sigma_y_var),
         "Base forward process sigma in the real Atticus body-forward axis."); r += 1
    mcl_row_sigma_theta = _row(motion_frame, r, "Base sigma theta (deg):", ttk.Entry(motion_frame, textvariable=mcl_sigma_theta_var),
         "Base heading process sigma."); r += 1
    mcl_row_sigma_x_per = _row(motion_frame, r, "Right sigma / in:", ttk.Entry(motion_frame, textvariable=mcl_sigma_x_per_var),
         "Additional lateral sigma per inch of lateral body motion."); r += 1
    mcl_row_sigma_y_per = _row(motion_frame, r, "Forward sigma / in:", ttk.Entry(motion_frame, textvariable=mcl_sigma_y_per_var),
         "Additional forward sigma per inch of forward body motion."); r += 1
    mcl_row_sigma_theta_per = _row(motion_frame, r, "Theta sigma / deg:", ttk.Entry(motion_frame, textvariable=mcl_sigma_theta_per_var),
         "Additional heading sigma per degree turned."); r += 1
    mcl_row_no_horizontal_scale = _row(motion_frame, r, "No horizontal scale:", ttk.Entry(motion_frame, textvariable=mcl_no_horizontal_scale_var),
         "Inflate lateral uncertainty by this factor when no horizontal odom wheel exists."); r += 1
    mcl_row_turn_lateral_scale = _row(motion_frame, r, "Turn lateral scale:", ttk.Entry(motion_frame, textvariable=mcl_turn_lateral_scale_var),
         "Extra lateral inflation during sharp turns without horizontal odom."); r += 1
    mcl_row_rough_wall_sigma_x_scale = _row(motion_frame, r, "Rough wall right scale:", ttk.Entry(motion_frame, textvariable=mcl_rough_wall_sigma_x_scale_var),
         "Extra lateral inflation during rough wall-traverse segments."); r += 1
    mcl_row_rough_wall_sigma_y_scale = _row(motion_frame, r, "Rough wall forward scale:", ttk.Entry(motion_frame, textvariable=mcl_rough_wall_sigma_y_scale_var),
         "Extra forward inflation during rough wall-traverse segments."); r += 1
    mcl_row_rough_wall_sigma_theta_scale = _row(motion_frame, r, "Rough wall theta scale:", ttk.Entry(motion_frame, textvariable=mcl_rough_wall_sigma_theta_scale_var),
         "Extra heading-process inflation during rough wall-traverse segments."); r += 1
    _row(motion_frame, r, "Set pose sigma XY:", ttk.Entry(motion_frame, textvariable=mcl_set_pose_xy_var),
         "Sigma (in) when initializing around a pose."); r += 1
    _row(motion_frame, r, "Set pose sigma theta:", ttk.Entry(motion_frame, textvariable=mcl_set_pose_theta_var),
         "Sigma (deg) when initializing around a pose."); r += 1

    sensors_frame = ttk.LabelFrame(mcl_body, text="Range Sensors")
    sensors_frame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    sensors_frame.columnconfigure(1, weight=1)
    r = 0
    _row(sensors_frame, r, "Distance enabled:", ttk.Checkbutton(sensors_frame, variable=mcl_dist_enabled_var),
         "Enable distance sensors for Atticus map corrections."); r += 1
    mcl_row_dist_sigma = _row(sensors_frame, r, "Sigma hit (mm):", ttk.Entry(sensors_frame, textvariable=mcl_dist_sigma_var),
         "Distance sensor noise (mm)."); r += 1
    mcl_row_dist_w_hit = _row(sensors_frame, r, "W_hit:", ttk.Entry(sensors_frame, textvariable=mcl_dist_w_hit_var),
         "Mixture weight for hit component."); r += 1
    mcl_row_dist_w_rand = _row(sensors_frame, r, "W_rand:", ttk.Entry(sensors_frame, textvariable=mcl_dist_w_rand_var),
         "Mixture weight for random component."); r += 1
    mcl_row_dist_w_short = _row(sensors_frame, r, "W_short:", ttk.Entry(sensors_frame, textvariable=mcl_dist_w_short_var),
         "Mixture weight for short readings."); r += 1
    mcl_row_dist_w_max = _row(sensors_frame, r, "W_max:", ttk.Entry(sensors_frame, textvariable=mcl_dist_w_max_var),
         "Mixture weight for max range."); r += 1
    mcl_row_dist_lambda_short = _row(sensors_frame, r, "Lambda short:", ttk.Entry(sensors_frame, textvariable=mcl_dist_lambda_short_var),
         "Short reading exponential rate."); r += 1
    mcl_row_dist_max_range = _row(sensors_frame, r, "Max range (mm):", ttk.Entry(sensors_frame, textvariable=mcl_dist_max_range_var),
         "Distance sensor max range (mm)."); r += 1
    mcl_row_dist_min_range = _row(sensors_frame, r, "Min range (mm):", ttk.Entry(sensors_frame, textvariable=mcl_dist_min_range_var),
         "Ignore readings below this range (mm)."); r += 1
    mcl_row_dist_conf_min = _row(sensors_frame, r, "Min confidence (0-63):", ttk.Entry(sensors_frame, textvariable=mcl_dist_conf_min_var),
         "Minimum distance confidence to accept readings (PROS: 0–63; 0 disables)."); r += 1
    mcl_row_dist_obj_min = _row(sensors_frame, r, "Min object size:", ttk.Entry(sensors_frame, textvariable=mcl_dist_obj_size_min_var),
         "Minimum object size to accept readings (0 disables)."); r += 1
    mcl_row_dist_obj_max = _row(sensors_frame, r, "Max object size:", ttk.Entry(sensors_frame, textvariable=mcl_dist_obj_size_max_var),
         "Maximum object size to accept readings (0 disables)."); r += 1
    mcl_row_dist_innov = _row(sensors_frame, r, "Innovation gate (mm):", ttk.Entry(sensors_frame, textvariable=mcl_dist_innov_gate_var),
         "Reject a sensor if |meas - expected| exceeds this. Real Atticus defaults to 180 mm."); r += 1
    mcl_row_dist_median = _row(sensors_frame, r, "Median window:", ttk.Entry(sensors_frame, textvariable=mcl_dist_median_window_var),
         "Median filter window for distance sensors (1 disables)."); r += 1
    mcl_row_dist_ignore_max = _row(sensors_frame, r, "LF ignore max:", ttk.Checkbutton(sensors_frame, variable=mcl_dist_ignore_max_var),
         "Ignore max-range readings in likelihood field mode."); r += 1
    mcl_row_dist_gate = _row(sensors_frame, r, "Gate (mm):", ttk.Entry(sensors_frame, textvariable=mcl_dist_gate_var),
         "Legacy overlay threshold. Keep this aligned with the innovation gate."); r += 1
    mcl_row_dist_gate_mode = _row(sensors_frame, r, "Gate mode:", ttk.Combobox(sensors_frame, values=["hard", "soft"],
         textvariable=mcl_dist_gate_mode_var, state="readonly"), "Hard or soft gating."); r += 1
    mcl_row_dist_gate_penalty = _row(sensors_frame, r, "Gate penalty:", ttk.Entry(sensors_frame, textvariable=mcl_dist_gate_penalty_var),
         "Soft gating penalty multiplier."); r += 1
    mcl_row_dist_gate_reject = _row(sensors_frame, r, "Gate reject ratio:", ttk.Entry(sensors_frame, textvariable=mcl_dist_gate_reject_var),
         "Ignore a reading if this fraction of particles reject it."); r += 1
    mcl_row_dist_lf_res = _row(sensors_frame, r, "LF resolution (in):", ttk.Entry(sensors_frame, textvariable=mcl_dist_lf_res_var),
         "Likelihood field resolution (in)."); r += 1
    _row(sensors_frame, r, "IMU enabled:", ttk.Checkbutton(sensors_frame, variable=mcl_imu_enabled_var),
         "Enable IMU heading weighting."); r += 1
    mcl_row_imu_sigma = _row(sensors_frame, r, "IMU sigma (deg):", ttk.Entry(sensors_frame, textvariable=mcl_imu_sigma_var),
         "IMU heading noise (deg)."); r += 1
    _row(sensors_frame, r, "EKF use IMU:", ttk.Checkbutton(sensors_frame, variable=mcl_ekf_use_imu_var),
         "Fuse IMU heading into the EKF. Disable this when an external pose provider already uses the same IMU."); r += 1

    def _mcl_sensor_value(var, default=0.0):
        """Parse a float-like sensor field without breaking the editor."""
        try:
            return float(var.get())
        except Exception:
            try:
                return float(default)
            except Exception:
                return 0.0

    def _mcl_sensor_map_mode(var):
        """Clamp a per-sensor map-mode field to the supported Atticus values."""
        mode = str(var.get() or "perimeter").strip().lower()
        return mode if mode in ("both", "perimeter", "objects") else "perimeter"

    def _open_mcl_sensor_config(idx: int):
        """Open the advanced editor for one distance sensor."""
        sv = mcl_sensor_vars[idx]
        win = tk.Toplevel(top)
        win.title(f"Distance Sensor {idx + 1} Settings")
        win.resizable(False, False)
        body = ttk.Frame(win)
        body.pack(fill="both", expand=True, padx=8, pady=8)
        r = 0
        _row(body, r, "Bias (mm):", ttk.Entry(body, textvariable=sv["bias_mm"]),
             "Bias subtracted from the raw distance measurement."); r += 1
        _row(body, r, "Angle offset (deg):", ttk.Entry(body, textvariable=sv["angle_offset_deg"]),
             "Additional angle offset for this sensor."); r += 1
        _row(body, r, "Min range (mm):", ttk.Entry(body, textvariable=sv["min_range_mm"]),
             "Override minimum valid range for this sensor."); r += 1
        _row(body, r, "Max range (mm):", ttk.Entry(body, textvariable=sv["max_range_mm"]),
             "Override maximum valid range for this sensor."); r += 1
        _row(body, r, "Min confidence (0-63):", ttk.Entry(body, textvariable=sv["min_confidence"]),
             "Override minimum confidence for this sensor."); r += 1
        _row(body, r, "Min object size:", ttk.Entry(body, textvariable=sv["min_object_size"]),
             "Override minimum object size for this sensor."); r += 1
        _row(body, r, "Max object size:", ttk.Entry(body, textvariable=sv["max_object_size"]),
             "Override maximum object size for this sensor."); r += 1
        _row(body, r, "Innovation gate (mm):", ttk.Entry(body, textvariable=sv["innovation_gate_mm"]),
             "Per-sensor innovation gate (mm)."); r += 1
        _row(body, r, "Map mode:", ttk.Combobox(body, values=["both", "perimeter", "objects"],
             textvariable=sv["map_mode"], state="readonly"),
             "Choose map geometry this sensor should see; for object mode, use Sensor object view in Map Objects to pick object ids."); r += 1

        def _calibrate_bias():
            """Handle calibrate bias."""
            global robot_pos, robot_heading, mcl_state
            name = str(sv["name"].get() or f"sensor {idx + 1}")
            meas = simpledialog.askfloat("Calibrate bias",
                                         f"Enter measured distance (mm) for {name}:",
                                         parent=win)
            if meas is None:
                return
            try:
                mcl_mod.update_map_segments(mcl_state, CFG)
            except Exception:
                pass
            sensor_cfg = {
                "x_in": _mcl_sensor_value(sv["x_in"], 0.0),
                "y_in": _mcl_sensor_value(sv["y_in"], 0.0),
                "angle_deg": _mcl_sensor_value(sv["angle_deg"], 0.0),
                "angle_offset_deg": _mcl_sensor_value(sv["angle_offset_deg"], 0.0),
                "map_mode": _mcl_sensor_map_mode(sv["map_mode"]),
            }
            try:
                mcl_heading = _mcl_heading_from_internal(robot_heading)
                hit = mcl_mod._raycast_expected_measurement(
                    mcl_state,
                    CFG,
                    (robot_pos[0], robot_pos[1], mcl_heading),
                    sensor_cfg,
                )
                if hit is None:
                    messagebox.showerror("Calibrate bias",
                                         "No map intersection found for this sensor from the current pose.")
                    return
                expected_mm = float(hit.get("distance_mm", 0.0))
                bias = meas - expected_mm
                sv["bias_mm"].set(f"{bias:.3f}")
            except Exception as exc:
                messagebox.showerror("Calibrate bias", f"Calibration failed:\n{exc}")

        btn_frame = ttk.Frame(body)
        btn_frame.pack(fill="x", pady=(6, 0))
        ttk.Button(btn_frame, text="Calibrate bias from pose", command=_calibrate_bias).pack(side="left")
        ttk.Button(btn_frame, text="Close", command=win.destroy).pack(side="right")

    geom_frame = ttk.LabelFrame(mcl_body, text="Distance Sensor Geometry")
    geom_frame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    geom_frame.columnconfigure(1, weight=1)
    header = ttk.Frame(geom_frame)
    header.grid(row=0, column=1, sticky="w", padx=6)
    ttk.Label(header, text="Name").grid(row=0, column=0, padx=2)
    ttk.Label(header, text="X (in)").grid(row=0, column=1, padx=2)
    ttk.Label(header, text="Y (in)").grid(row=0, column=2, padx=2)
    ttk.Label(header, text="Angle (deg)").grid(row=0, column=3, padx=2)
    ttk.Label(header, text="On").grid(row=0, column=4, padx=2)
    ttk.Label(header, text="Cfg").grid(row=0, column=5, padx=2)
    mcl_geom_header = header
    mcl_geom_rows = []
    for i, sensor_vars in enumerate(mcl_sensor_vars):
        row_frame = ttk.Frame(geom_frame)
        name_entry = ttk.Entry(row_frame, textvariable=sensor_vars["name"], width=8)
        name_entry.grid(row=0, column=0, padx=2)
        x_entry = ttk.Entry(row_frame, textvariable=sensor_vars["x_in"], width=6)
        x_entry.grid(row=0, column=1, padx=2)
        y_entry = ttk.Entry(row_frame, textvariable=sensor_vars["y_in"], width=6)
        y_entry.grid(row=0, column=2, padx=2)
        ang_entry = ttk.Entry(row_frame, textvariable=sensor_vars["angle_deg"], width=8)
        ang_entry.grid(row=0, column=3, padx=2)
        en_check = ttk.Checkbutton(row_frame, variable=sensor_vars["enabled"])
        en_check.grid(row=0, column=4, padx=2)
        cfg_btn = ttk.Button(row_frame, text="Edit", command=lambda i=i: _open_mcl_sensor_config(i))
        cfg_btn.grid(row=0, column=5, padx=2)
        _track_widgets(name_entry, x_entry, y_entry, ang_entry, en_check, cfg_btn)
        mcl_geom_rows.append(_row(
            geom_frame,
            1 + i,
            f"Sensor {i+1}:",
            row_frame,
            "Distance sensor geometry in native Atticus coordinates: x=right of center, y=forward, angle 0=forward and +90=right."
        ))

    map_frame = ttk.LabelFrame(mcl_body, text="Map Objects")
    map_frame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    map_frame.columnconfigure(1, weight=1)
    cat_frame = ttk.Frame(map_frame)
    map_perim_check = ttk.Checkbutton(cat_frame, text="Perimeter", variable=mcl_map_perimeter_var)
    map_perim_check.grid(row=0, column=0, sticky="w", padx=4)
    map_long_check = ttk.Checkbutton(cat_frame, text="Long goals", variable=mcl_map_long_goals_var)
    map_long_check.grid(row=0, column=1, sticky="w", padx=4)
    map_long_braces_check = ttk.Checkbutton(cat_frame, text="Long goal braces", variable=mcl_map_long_goal_braces_var)
    map_long_braces_check.grid(row=0, column=2, sticky="w", padx=4)
    map_center_check = ttk.Checkbutton(cat_frame, text="Center goals", variable=mcl_map_center_goals_var)
    map_center_check.grid(row=1, column=0, sticky="w", padx=4)
    map_match_check = ttk.Checkbutton(cat_frame, text="Matchloaders", variable=mcl_map_matchloaders_var)
    map_match_check.grid(row=1, column=1, sticky="w", padx=4)
    map_park_check = ttk.Checkbutton(cat_frame, text="Park zones", variable=mcl_map_park_zones_var)
    map_park_check.grid(row=2, column=0, sticky="w", padx=4)
    _track_widgets(map_perim_check, map_long_check, map_long_braces_check, map_center_check, map_match_check, map_park_check)
    mcl_row_map_categories = _row(map_frame, 0, "Categories:", cat_frame, "Map object categories to include in raycasting.")
    sensor_sel_var = tk.StringVar(value="Global map objects")
    sensor_sel_frame = ttk.Frame(map_frame)
    sensor_sel_combo = ttk.Combobox(sensor_sel_frame, textvariable=sensor_sel_var, state="readonly")
    sensor_sel_combo.pack(side="left", fill="x", expand=True, padx=(0, 4))
    _track_widgets(sensor_sel_combo)
    mcl_row_map_sensor_view = _row(
        map_frame,
        1,
        "Sensor object view:",
        sensor_sel_frame,
        "Select Global map objects or an enabled sensor to edit which objects that sensor can raycast."
    )
    obj_frame = ttk.Frame(map_frame)
    obj_frame.columnconfigure(0, weight=1)
    mcl_row_map_objects = _row(map_frame, 2, "Individual objects:", obj_frame, "Override specific objects (by id).")

    sensor_sel_map = {}

    def _sensor_selector_values():
        """Build picker labels for the global and per-sensor object visibility views."""
        values = ["Global map objects"]
        for idx, sensor_vars in enumerate(mcl_sensor_vars):
            try:
                enabled = int(sensor_vars["enabled"].get()) == 1
            except Exception:
                enabled = False
            if not enabled:
                continue
            name = str(sensor_vars["name"].get() or "").strip() or f"sensor_{idx + 1}"
            label = f"Sensor {idx + 1}: {name}"
            values.append(label)
            sensor_sel_map[label] = idx
        return values

    def _selected_sensor_idx():
        """Map the current object-visibility picker choice back to a sensor index."""
        return sensor_sel_map.get(str(sensor_sel_var.get() or "").strip())

    def _rebuild_object_checks():
        """Rebuild the object checkbox list for the active global or per-sensor view."""
        for child in list(obj_frame.winfo_children()):
            try:
                child.destroy()
            except Exception:
                pass
        sensor_idx = _selected_sensor_idx()
        for row, (obj_id, obj_label) in enumerate(mcl_object_entries):
            if sensor_idx is None:
                var = mcl_object_vars.get(obj_id)
            else:
                var = mcl_sensor_object_vars.get(sensor_idx, {}).get(obj_id)
            if var is None:
                continue
            obj_text = f"{obj_label} ({obj_id})"
            obj_check = tk.Checkbutton(
                obj_frame,
                text=obj_text,
                variable=var,
                anchor="w",
                justify="left",
                wraplength=420,
            )
            obj_check.grid(row=row, column=0, sticky="ew", padx=4, pady=1)
            _track_widgets(obj_check)
            ui.add_tooltip(obj_check, f"Object id: {obj_id}")

    def _refresh_sensor_selector(*_):
        """Refresh the per-sensor object-visibility picker after name or enable changes."""
        sensor_sel_map.clear()
        vals = _sensor_selector_values()
        sensor_sel_combo.configure(values=vals)
        cur = str(sensor_sel_var.get() or "").strip()
        if cur not in vals:
            sensor_sel_var.set(vals[0] if vals else "Global map objects")
        _rebuild_object_checks()

    sensor_sel_combo.bind("<<ComboboxSelected>>", lambda _e: _rebuild_object_checks())
    for sensor_vars in mcl_sensor_vars:
        try:
            sensor_vars["enabled"].trace_add("write", _refresh_sensor_selector)
            sensor_vars["name"].trace_add("write", _refresh_sensor_selector)
        except Exception:
            pass
    _refresh_sensor_selector()

    corr_frame = ttk.LabelFrame(mcl_body, text="Bounded Writeback")
    corr_frame.grid(row=7, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    corr_frame.columnconfigure(1, weight=1)
    r = 0
    _row(corr_frame, r, "Enabled:", ttk.Checkbutton(corr_frame, variable=mcl_corr_enabled_var),
         "Enable bounded writeback from the fused Atticus pose back toward odometry."); r += 1
    mcl_row_corr_min_conf = _row(corr_frame, r, "Min confidence:", ttk.Entry(corr_frame, textvariable=mcl_corr_min_conf_var),
         "Minimum confidence to apply correction."); r += 1
    mcl_row_corr_max_trans = _row(corr_frame, r, "Max trans jump (in):", ttk.Entry(corr_frame, textvariable=mcl_corr_max_trans_var),
         "Clamp each writeback axis to this translation limit."); r += 1
    mcl_row_corr_max_theta = _row(corr_frame, r, "Max theta jump (deg):", ttk.Entry(corr_frame, textvariable=mcl_corr_max_theta_var),
         "Clamp heading writeback to this many degrees."); r += 1
    mcl_row_corr_writeback_alpha = _row(corr_frame, r, "Writeback alpha:", ttk.Entry(corr_frame, textvariable=mcl_corr_writeback_alpha_var),
         "Blend factor used by continuous writeback. Real Atticus defaults to 0.35."); r += 1
    mcl_row_corr_teleport_trans = _row(corr_frame, r, "Teleport reset trans (in):", ttk.Entry(corr_frame, textvariable=mcl_corr_teleport_trans_var),
         "Reset the fused estimate if one odom step jumps farther than this."); r += 1
    mcl_row_corr_teleport_theta = _row(corr_frame, r, "Teleport reset theta (deg):", ttk.Entry(corr_frame, textvariable=mcl_corr_teleport_theta_var),
         "Reset the fused estimate if one odom step turns farther than this."); r += 1

    viz_frame = ttk.LabelFrame(mcl_body, text="Localization Overlay")
    viz_frame.grid(row=8, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    viz_frame.columnconfigure(1, weight=1)
    r = 0
    _row(viz_frame, r, "Show estimate:", ttk.Checkbutton(viz_frame, variable=mcl_show_estimate_var),
         "Show estimated pose."); r += 1
    _row(viz_frame, r, "Show covariance:", ttk.Checkbutton(viz_frame, variable=mcl_show_cov_var),
         "Show covariance ellipse."); r += 1
    _row(viz_frame, r, "Show predicted rays:", ttk.Checkbutton(viz_frame, variable=mcl_show_rays_var),
         "Show the blue predicted map-hit ray. The green measured ray is always shown while Atticus sim is on."); r += 1
    _row(viz_frame, r, "Show gating:", ttk.Checkbutton(viz_frame, variable=mcl_show_gating_var),
         "Highlight gated sensor readings."); r += 1

    def _var_int(var, default=0):
        """Parse an int-like Tk variable without crashing the settings editor."""
        try:
            return int(var.get())
        except Exception:
            try:
                return int(float(var.get()))
            except Exception:
                return default

    def _var_float(var, default=0.0):
        """Parse a float-like Tk variable without crashing the settings editor."""
        try:
            return float(var.get())
        except Exception:
            return float(default)

    def _sync_geom_defaults() -> None:
        """Populate sensor geometry from the robot config when the editor is still blank."""
        if any(str(sv["name"].get() or "").strip() for sv in mcl_sensor_vars):
            return
        defaults = mcl_mod.get_distance_sensors(CFG, include_disabled=True)
        for idx, sv in enumerate(mcl_sensor_vars):
            if idx >= len(defaults):
                break
            entry = defaults[idx]
            sv["name"].set(str(entry.get("name", f"sensor_{idx + 1}")))
            sv["x_in"].set(str(entry.get("x_in", 0.0)))
            sv["y_in"].set(str(entry.get("y_in", 0.0)))
            sv["angle_deg"].set(str(entry.get("angle_deg", 0.0)))
            sv["enabled"].set(int(entry.get("enabled", 1)))

    def _refresh_mcl_visibility(*_):
        """Show only the Atticus controls that still change runtime behavior."""
        motion_on = _var_int(mcl_motion_enabled_var, 1) == 1
        _set_row_visible(mcl_row_sigma_x, motion_on)
        _set_row_visible(mcl_row_sigma_y, motion_on)
        _set_row_visible(mcl_row_sigma_theta, motion_on)
        _set_row_visible(mcl_row_sigma_x_per, motion_on)
        _set_row_visible(mcl_row_sigma_y_per, motion_on)
        _set_row_visible(mcl_row_sigma_theta_per, motion_on)
        _set_row_visible(mcl_row_no_horizontal_scale, motion_on)
        _set_row_visible(mcl_row_turn_lateral_scale, motion_on)
        _set_row_visible(mcl_row_rough_wall_sigma_x_scale, motion_on)
        _set_row_visible(mcl_row_rough_wall_sigma_y_scale, motion_on)
        _set_row_visible(mcl_row_rough_wall_sigma_theta_scale, motion_on)

        dist_on = _var_int(mcl_dist_enabled_var, 1) == 1
        _set_row_visible(mcl_row_dist_sigma, dist_on)
        _set_row_visible(mcl_row_dist_w_hit, False)
        _set_row_visible(mcl_row_dist_w_rand, False)
        _set_row_visible(mcl_row_dist_w_short, False)
        _set_row_visible(mcl_row_dist_w_max, False)
        _set_row_visible(mcl_row_dist_lambda_short, False)
        _set_row_visible(mcl_row_dist_max_range, dist_on)
        _set_row_visible(mcl_row_dist_min_range, dist_on)
        _set_row_visible(mcl_row_dist_conf_min, False)
        _set_row_visible(mcl_row_dist_obj_min, False)
        _set_row_visible(mcl_row_dist_obj_max, False)
        _set_row_visible(mcl_row_dist_innov, dist_on)
        _set_row_visible(mcl_row_dist_median, dist_on)
        _set_row_visible(mcl_row_dist_ignore_max, False)
        _set_row_visible(mcl_row_dist_gate, dist_on)
        _set_row_visible(mcl_row_dist_gate_mode, False)
        _set_row_visible(mcl_row_dist_gate_penalty, False)
        _set_row_visible(mcl_row_dist_gate_reject, False)
        _set_row_visible(mcl_row_dist_lf_res, False)

        imu_on = _var_int(mcl_imu_enabled_var, 1) == 1
        _set_row_visible(mcl_row_imu_sigma, imu_on)

        if dist_on:
            geom_frame.grid()
            map_frame.grid()
        else:
            geom_frame.grid_remove()
            map_frame.grid_remove()
        try:
            if dist_on:
                mcl_geom_header.grid()
            else:
                mcl_geom_header.grid_remove()
        except Exception:
            pass
        for row in mcl_geom_rows:
            _set_row_visible(row, dist_on)
        _set_row_visible(mcl_row_map_categories, dist_on)
        _set_row_visible(mcl_row_map_sensor_view, dist_on)
        _set_row_visible(mcl_row_map_objects, dist_on)

        corr_on = _var_int(mcl_corr_enabled_var, 1) == 1
        _set_row_visible(mcl_row_corr_min_conf, corr_on)
        _set_row_visible(mcl_row_corr_max_trans, corr_on)
        _set_row_visible(mcl_row_corr_max_theta, corr_on)
        _set_row_visible(mcl_row_corr_writeback_alpha, corr_on)
        _set_row_visible(mcl_row_corr_teleport_trans, corr_on)
        _set_row_visible(mcl_row_corr_teleport_theta, corr_on)
        try:
            _mcl_on_configure()
        except Exception:
            pass

    for var in (
        mcl_motion_enabled_var,
        mcl_dist_enabled_var,
        mcl_dist_gate_var,
        mcl_imu_enabled_var,
        mcl_corr_enabled_var,
    ):
        try:
            var.trace_add("write", _refresh_mcl_visibility)
        except Exception:
            pass
    _sync_geom_defaults()
    _refresh_mcl_visibility()

    integration_frame = ttk.LabelFrame(mcl_body, text="Atticus Integration")
    integration_frame.grid(row=9, column=0, columnspan=2, sticky="ew", padx=6, pady=(0, 8))
    integration_frame.columnconfigure(1, weight=1)
    integration_info = ttk.Label(
        integration_frame,
        text=(
            "Legacy export and tuning tools have been removed from the terminal.\n"
            "This tab now configures the Atticus-style local map corrector used in simulation:\n"
            "set sensor geometry, range gating, and bounded writeback here, then validate the result in sim."
        ),
        wraplength=420,
        justify="left",
    )
    _row(integration_frame, 0, "Info:", integration_info,
         "Atticus localization in the terminal is simulator-focused and no longer exports the old particle-filter package.")

    def _refresh_mcl_export_wrap(_evt=None):
        """Handle refresh mcl export wrap."""
        try:
            wrap_px = max(180, min(900, int(integration_frame.winfo_width()) - 210))
            integration_info.configure(wraplength=wrap_px)
        except Exception:
            pass

    integration_frame.bind("<Configure>", _refresh_mcl_export_wrap)
    top.after(0, _refresh_mcl_export_wrap)

    _stored_tpl_defaults = CFG.get("codegen", {}).get("templates", {})

    # JAR (non-advanced) defaults (locked in the UI): keep this lightweight and match
    # the non-advanced expectations (simple settle/timeout + motion call).
    _jar_defaults = {
        "wait": "task::sleep({MS});",
        "move": "chassis.drive_settle_error = {DRIVE_EARLY_EXIT};\nchassis.drive_timeout =  {TIMEOUT_MS};\nchassis.drive_distance({DIST_IN});",
        "turn_global": "chassis.turn_settle_error = {TURN_EARLY_EXIT};\nchassis.turn_timeout = {TIMEOUT_MS};\nchassis.turn_to_angle({HEADING_DEG});",
        "turn_local": "turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS});",
        "pose": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "pose_angle": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "swing": "chassis.swing_settle_error = {SWING_EARLY_EXIT};\nchassis.swing_timeout = {TIMEOUT_MS};\nchassis.{SIDE_LC}_swing_to_angle({HEADING_DEG});",
        "path_follow": 'followPath("{PATH_FILE}", {TIMEOUT_MS});',
        "reshape_on": "MLmech.off();",
        "reshape_off": "MLmech.on();",
        "reshape": "MLmech.on();",
        "reverse_on": "// reverse handled inline",
        "reverse_off": "// reverse handled inline",
        "tbuffer": "task::sleep({MS});",
        "marker_wait": "",
        "marker_wait_done": "",
        "setpose": "chassis.set_coordinates({X_IN},{Y_IN},{HEADING_DEG});",
    }

    # JAR (advanced) keeps the fully-parameterized placeholders.
    _jar_advanced_defaults = {
        "wait": "task::sleep({MS});",
        "move": "chassis.drive_distance({DIST_IN}, {HEADING_DEG},{DRIVE_MAX_V},{HEADING_MAX_V},{DRIVE_SETTLE_ERR},{DRIVE_SETTLE_TIME}, {TIMEOUT_MS});",
        "turn_global": "chassis.turn_to_angle({HEADING_DEG}, {TURN_MAX_V}, {TIMEOUT_MS});",
        "turn_local": "turnToAngle({TURN_DELTA_DEG}, {TIMEOUT_MS});",
        "pose": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "pose_angle": "chassis.holonomic_drive_to_point({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS});",
        "swing": "chassis.{SIDE}_swing_to_angle({HEADING_DEG}, {SWING_MAX_V}, {SWING_SETTLE_ERR}, {SWING_SETTLE_TIME}, {TIMEOUT_MS});",
        "path_follow": 'followPath("{PATH_FILE}", {TIMEOUT_MS});',
        "reshape_on": "matchload_state({STATE});",
        "reshape_off": "matchload_state({STATE});",
        "reshape": "matchload_state({STATE});",
        "reverse_on": "// reverse handled inline",
        "reverse_off": "// reverse handled inline",
        "tbuffer": "task::sleep({MS});",
        "marker_wait": "",
        "marker_wait_done": "",
        "setpose": "chassis.set_coordinates({X_IN}, {Y_IN}, {HEADING_DEG});"
    }

    codegen_defaults = {
        "AttLib": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, attlib::DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = attlib::AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
            "atticus_immediate": "localizer.applyImmediateCorrectionAuto();",
            "atticus_wall_trim_start": "localizer.correctThetaFromWallAuto();",
            "atticus_wall_trim_end": "localizer.endThetaWallAlignment();",
            "atticus_rough_wall_start": "localizer.startRoughWallTraverseAuto();",
            "atticus_rough_wall_end": "localizer.endRoughWallTraverse();",
            "reshape_on": "matchloadPistons.set_value({STATE});",
            "reshape_off": "matchloadPistons.set_value({STATE});",
            "reshape": "matchloadPistons.set_value({STATE});",
            "reverse_on": "// reverse handled per-command",
            "reverse_off": "// reverse handled per-command",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "chassis.waitUntilDone();",
            "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "Atticus": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
            "atticus_immediate": "localizer.applyImmediateCorrectionAuto();",
            "atticus_wall_trim_start": "localizer.correctThetaFromWallAuto();",
            "atticus_wall_trim_end": "localizer.endThetaWallAlignment();",
            "atticus_rough_wall_start": "localizer.startRoughWallTraverseAuto();",
            "atticus_rough_wall_end": "localizer.endRoughWallTraverse();",
            "reshape_on": "matchloadPistons.set_value({STATE});",
            "reshape_off": "matchloadPistons.set_value({STATE});",
            "reshape": "matchloadPistons.set_value({STATE});",
            "reverse_on": "// reverse handled per-command",
            "reverse_off": "// reverse handled per-command",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "chassis.waitUntilDone();",
            "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "LemLib": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
            "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
            "reshape_on": "matchloadPistons.set_value({STATE});",
            "reshape_off": "matchloadPistons.set_value({STATE});",
            "reshape": "matchloadPistons.set_value({STATE});",
            "reverse_on": "// reverse handled per-command",
            "reverse_off": "// reverse handled per-command",
            "tbuffer": "pros::delay({MS});",
            "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "chassis.waitUntilDone();",
            "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "JAR": dict(_jar_defaults),
        "JAR (advanced)": dict(_jar_advanced_defaults),
        "Custom": {
            "wait": "wait({S}, seconds);",
            "move": "chassis.drive_distance({DIST_IN}, {HEADING_DEG}, {.drive_timeout = {TIMEOUT_MS}});",
            "turn_global": "chassis.turn_to_angle({HEADING_DEG}, {.turn_timeout = {TIMEOUT_MS}});",
            "turn_local": "turn_relative({TURN_DELTA_DEG});",
            "pose": "pose({X_IN}, {Y_IN}, {HEADING_DEG});",
            "swing": "swing_to_heading({HEADING_DEG}, {DIR});",
            "reshape_on": "MLadapter({STATE});",
            "reshape_off": "MLadapter({STATE});",
            "reshape": "MLadapter({STATE});",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "wait ({S}, seconds);",
            "marker_wait": "waitUntil({MARKER_DIST_IN});",
            "marker_wait_done": "waitUntilDone();",
            "path_follow": 'follow_path("{PATH_NAME}", {TIMEOUT_MS}, {LOOKAHEAD});',
            "setpose": "chassis.set_coordinates({X_IN},{Y_IN},{HEADING_DEG});"
        }
    }
    _stored_tpl_defaults = CFG.get("codegen", {}).get("templates", {})
    if isinstance(_stored_tpl_defaults, dict):
        for _style_name, _style_map in _stored_tpl_defaults.items():
            if not isinstance(_style_map, dict):
                continue
            dst = codegen_defaults.setdefault(_style_name, {})
            for _k, _v in _style_map.items():
                if str(_k).startswith("__"):
                    continue
                if isinstance(_v, str):
                    dst[_k] = _v
        CFG.setdefault("codegen", {
            "style": "JAR",
            "templates": {},
            "opts": {
                "ticks_per_rotation": 360,
                "pad_factor": 1.0,
                "min_timeout_s": 0.0,
                "reshape_output": "1/2",
                "jar_pose_angle_overload": 0,
                # Passive chain defaults (inches): used when segments are not explicitly chained.
                "default_drive_early_exit": 0.0,
                "default_turn_early_exit": 0.0,
                # Keep historical behavior: swings default to a small early-exit window unless "settle swing" is used.
                "default_swing_early_exit": 7.0,
                # When "settle swing" is used, apply this early-exit window (0 by default).
                "default_swing_settle_early_exit": 0.0,
                "omit_defaults": 1
            },
            "path_dir": "export/paths",
            "path_columns": "{X}, {Y}, {COMMAND}",
            "mech_presets": [
            {"name": "reshape", "mode": "toggle", "template": "", "on": "", "off": "", "default": False},
            {"name": "DSR", "mode": "action", "template": "", "on": "", "off": "", "default": False}
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

    _ensure_locked_mech_presets(CFG)

    def _default_optional_for_style(style_name: str):
        """Handle default optional for style."""
        base = ["setpose", "path_follow"]
        if style_name not in ("LemLib", "Atticus", "AttLib"):
            base += ["pose", "swing"]
        return base

    opt_cfg = CFG["codegen"].setdefault("opts", {})
    # Migration/defaults for existing configs: avoid accidentally changing swing behavior
    # by introducing new opts with a 0 default.
    opt_cfg.setdefault("default_drive_early_exit", 0.0)
    opt_cfg.setdefault("default_turn_early_exit", 0.0)
    opt_cfg.setdefault("default_swing_early_exit", 7.0)
    opt_cfg.setdefault("default_swing_settle_early_exit", 0.0)
    # If this key existed from an earlier build where we defaulted it to 0,
    # migrate back to the historical swing behavior once.
    if not opt_cfg.get("default_swing_early_exit_migrated", 0):
        try:
            _v = opt_cfg.get("default_swing_early_exit", 7.0)
            if isinstance(_v, dict):
                _v = _v.get("value", 7.0)
            if abs(float(_v)) < 1e-9:
                opt_cfg["default_swing_early_exit"] = 7.0
        except Exception:
            opt_cfg["default_swing_early_exit"] = 7.0
        opt_cfg["default_swing_early_exit_migrated"] = 1
    migrate_path_follow = not bool(opt_cfg.get("path_follow_optional_migrated", False))
    migrate_reshape_split = not bool(opt_cfg.get("reshape_split_templates_migrated", False))
    migrate_async_syntax = not bool(opt_cfg.get("async_template_syntax_migrated", False))
    migrate_turn_swing_forwards = not bool(opt_cfg.get("turn_swing_forwards_template_migrated", False))
    legacy_turn_swing_tpls = {
        "AttLib": {
            "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, attlib::DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.direction = attlib::AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
        },
        "LemLib": {
            "turn_global": (
                "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            ),
            "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
            "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.direction = AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
        },
    }
    for _style, _tpl in codegen_defaults.items():
        CFG["codegen"].setdefault("templates", {}).setdefault(_style, dict(_tpl))
        CFG["codegen"]["templates"].setdefault(_style, {}).setdefault("__optional__", _default_optional_for_style(_style))
        tpl_map = CFG["codegen"]["templates"].get(_style, {})
        if migrate_turn_swing_forwards and isinstance(tpl_map, dict):
            legacy_map = legacy_turn_swing_tpls.get(_style, {})
            for _tpl_key, _legacy_val in legacy_map.items():
                _current = tpl_map.get(_tpl_key)
                if isinstance(_legacy_val, (tuple, list, set)):
                    _match = _current in _legacy_val
                else:
                    _match = (_current == _legacy_val)
                if _match:
                    tpl_map[_tpl_key] = _tpl.get(_tpl_key, _legacy_val)
        if migrate_async_syntax and isinstance(tpl_map, dict):
            for _tpl_key, _tpl_val in list(tpl_map.items()):
                if isinstance(_tpl_val, str) and "async = {ASYNC}" in _tpl_val:
                    tpl_map[_tpl_key] = _tpl_val.replace("async = {ASYNC}", "{ASYNC}")
        if migrate_path_follow:
            opt_list = CFG["codegen"]["templates"].get(_style, {}).get("__optional__", [])
            if isinstance(opt_list, list) and "path_follow" not in opt_list and "path_follow" in tpl_map:
                opt_list.append("path_follow")
        if migrate_reshape_split:
            if isinstance(tpl_map, dict):
                legacy = tpl_map.get("reshape")
                if legacy:
                    if not tpl_map.get("reshape_on"):
                        tpl_map["reshape_on"] = legacy
                    if not tpl_map.get("reshape_off"):
                        tpl_map["reshape_off"] = legacy
                opt_list = tpl_map.get("__optional__", [])
                if isinstance(opt_list, list) and "reshape" in opt_list:
                    opt_list[:] = [k for k in opt_list if k != "reshape"]
                    if "reshape_on" not in opt_list:
                        opt_list.append("reshape_on")
                    if "reshape_off" not in opt_list:
                        opt_list.append("reshape_off")
    if migrate_path_follow:
        opt_cfg["path_follow_optional_migrated"] = True
    if migrate_reshape_split:
        opt_cfg["reshape_split_templates_migrated"] = True
    if migrate_async_syntax:
        opt_cfg["async_template_syntax_migrated"] = True
    if migrate_turn_swing_forwards:
        opt_cfg["turn_swing_forwards_template_migrated"] = True

    if str(CFG.get("codegen", {}).get("style", "")).strip().lower() == "pros":
        CFG.setdefault("codegen", {})["style"] = "Custom"

    style_labels = ["Atticus", "Action List", "LemLib", "JAR", "JAR (advanced)", "Custom"]
    style_raw = CFG.get("codegen", {}).get("style", "JAR")
    if isinstance(style_raw, dict):
        style_raw = style_raw.get("value", "JAR")
    style_name = str(style_raw).strip() or "JAR"
    if style_name == "AttLib":
        style_name = "Atticus"
    if style_name not in style_labels:
        style_name = "JAR"
    codegen_style_var = tk.StringVar(value=style_name)

    base_tpl_keys = ["wait", "move", "turn_global", "turn_local", "pose", "pose_angle", "tbuffer"]
    optional_pool = ["reverse_on", "reverse_off", "reshape_on", "reshape_off", "setpose", "swing", "path_follow", "marker_wait", "marker_wait_done"]
    tpl_keys = base_tpl_keys + optional_pool
    tpl_vars = {k: tk.StringVar() for k in tpl_keys}
    motion_mode_var = tk.StringVar(value="move")
    turn_mode_var = tk.StringVar(value="turn_global")
    template_tokens = [
        "MS", "S", "TIMEOUT_MS", "TIMEOUT_S",
        "X_IN", "Y_IN", "DIST_IN", "DIST_ROT", "DIST_DEG", "DIST_TICKS",
        "HEADING_DEG", "HEADING_RAD", "TURN_DELTA_DEG", "TURN_DELTA_RAD",
        "TARGET_X_IN", "TARGET_Y_IN",
        "FORWARDS", "FORWARD_PARAM", "ASYNC", "DIR",
        "SIDE", "SIDE_LC", "LOCKED_SIDE", "LOCKED_SIDE_LC",
        "LOOKAHEAD",
        "PATH_NAME", "PATH_FILE", "PATH_ASSET",
        "MOVE_SPEED", "TURN_SPEED", "PATH_MIN_SPEED", "PATH_MAX_SPEED",
        "DRIVE_MAX_SPEED",
        "DRIVE_MAX_V", "HEADING_MAX_V", "TURN_MAX_V", "SWING_MAX_V",
        "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
        "TURN_SETTLE_ERR", "TURN_SETTLE_TIME",
        "SWING_SETTLE_ERR", "SWING_SETTLE_TIME",
        "DRIVE_MIN_SPEED", "TURN_MIN_SPEED", "SWING_MIN_SPEED",
        "DRIVE_EARLY_EXIT", "TURN_EARLY_EXIT", "SWING_EARLY_EXIT",
        "LEAD_IN", "STATE", "NAME", "CASE_KEY", "VALUE", "VALUE1", "VALUE2", "VALUE3",
        "MARKER_DIST_IN", "MARKER_FRAC", "MARKER_INDEX"
    ]

    ticks_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("ticks_per_rotation", 360)))
    pad_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("pad_factor", 1.0)))
    min_s_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("min_timeout_s", 0.0)))
    reshape_output_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("reshape_output", "1/2")))
    omit_defaults_var = tk.IntVar(value=int(CFG.get("codegen", {}).get("opts", {}).get("omit_defaults", 1)))
    default_drive_exit_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("default_drive_early_exit", 0.0)))
    default_turn_exit_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("default_turn_early_exit", 0.0)))
    default_swing_exit_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("default_swing_early_exit", 7.0)))
    default_settle_swing_exit_var = tk.StringVar(value=str(CFG.get("codegen", {}).get("opts", {}).get("default_swing_settle_early_exit", 0.0)))
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

    style_widget = ttk.Combobox(tabs["codegen"], textvariable=codegen_style_var, values=style_labels, state="readonly")
    _row(tabs["codegen"], 0, "Output style:", style_widget, "Select export style. Action List keeps current log.")
    _row(tabs["codegen"], 1, "Timeout pad ×:", 
        ttk.Entry(tabs["codegen"], textvariable=pad_var), 
        "Multiply projected time by constant for timeout margin.")
    _row(tabs["codegen"], 2, "Min timeout (s):", 
        ttk.Entry(tabs["codegen"], textvariable=min_s_var), 
        "Minimum timeout in seconds.")
    omit_defaults_chk = ttk.Checkbutton(tabs["codegen"], variable=omit_defaults_var)
    _row(tabs["codegen"], 3, "Omit default params:", omit_defaults_chk,
        "Remove default-valued named params from templates (disable to preserve full placeholder order).")

    # JAR-only settle/chain defaults (hide for other styles).
    jar_defaults_frame = ttk.LabelFrame(tabs["codegen"], text="JAR settle/chain defaults")
    jar_defaults_frame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=6, pady=(4, 6))
    jar_defaults_frame.columnconfigure(1, weight=1)
    drive_exit_entry = ttk.Entry(jar_defaults_frame, textvariable=default_drive_exit_var, width=10)
    _row(jar_defaults_frame, 0, "Default drive early-exit (in):", drive_exit_entry,
         "Passive chain default (inches). Used when a segment is not explicitly chained. 0 disables.")
    turn_exit_entry = ttk.Entry(jar_defaults_frame, textvariable=default_turn_exit_var, width=10)
    _row(jar_defaults_frame, 1, "Default turn early-exit (deg):", turn_exit_entry,
         "Passive chain default (degrees). Used when a segment is not explicitly chained. 0 disables.")
    swing_exit_entry = ttk.Entry(jar_defaults_frame, textvariable=default_swing_exit_var, width=10)
    _row(jar_defaults_frame, 2, "Default swing early-exit (deg):", swing_exit_entry,
         "Default swing chaining window (degrees) for non-settle swings. 0 disables chaining.")
    settle_swing_exit_entry = ttk.Entry(jar_defaults_frame, textvariable=default_settle_swing_exit_var, width=10)
    _row(jar_defaults_frame, 3, "Default settle swing early-exit (deg):", settle_swing_exit_entry,
         "Used only for Settle Swing. 0 means no early-exit window (fully settle).")

    def _refresh_jar_defaults_visibility(_evt=None):
        """Show JAR-only settle defaults only when a JAR style is selected."""
        try:
            is_jar = str(codegen_style_var.get()) in ("JAR", "JAR (advanced)")
            if is_jar:
                jar_defaults_frame.grid()
            else:
                jar_defaults_frame.grid_remove()
        except Exception:
            pass
    _refresh_jar_defaults_visibility()
    
    ticks_label = ttk.Label(tabs["codegen"], text="Ticks per rotation:")
    ticks_entry = ttk.Entry(tabs["codegen"], textvariable=ticks_var)
    ticks_label.grid(row=5, column=0, sticky="w", padx=6, pady=4)
    ticks_entry.grid(row=5, column=1, sticky="ew", padx=6, pady=4)
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
    _row(tabs["codegen"], 6, "Path export dir:", path_dir_frame, "Directory to save generated path files.")
    ui.add_tooltip(path_dir_btn, "Directory to save generated path files.")
    ui.add_tooltip(path_cols_btn, "Set output columns for path files.")
    ui.track_live_widget(path_dir_entry)

    def _cal_is_float(val):
        """Handle cal is float."""
        try:
            float(val)
            return True
        except Exception:
            return False

    def _cal_has_data(cal: dict) -> bool:
        """Handle cal has data."""
        profiles = cal.get("profiles", {}) if isinstance(cal, dict) else {}
        if not isinstance(profiles, dict):
            return False
        return any(bool(profiles.get(mv)) for mv in ("drive", "turn", "swing"))

    def _cal_bucket_count(cal: dict) -> int:
        """Handle cal bucket count."""
        profiles = cal.get("profiles", {}) if isinstance(cal, dict) else {}
        if not isinstance(profiles, dict):
            return 0
        def _walk(node):
            """Handle walk."""
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
        """Handle cal status text."""
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
        """Handle cal detail text."""
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
            """Handle walk."""
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
        """Handle refresh cal summary."""
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
        """Handle sync cal vars from cfg."""
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
        """Handle import calibration."""
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
        """Handle export calibration."""
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
        """Handle clear calibration."""
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
        """Handle reset calibration recommended."""
        cal_err_scale_var.set("1.2")
        cal_time_scale_var.set("1.2")
        cal_noise_mult_var.set("4.0")
        on_update()

    def _cal_bucket(move_type: str, magnitude: float):
        """Handle cal bucket."""
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
        """Handle cal p90."""
        vals = sorted(values)
        if not vals:
            return None
        idx = int(math.ceil(0.9 * len(vals))) - 1
        idx = max(0, min(len(vals) - 1, idx))
        return vals[idx]

    def _analyze_calibration_logs(log_text: str):
        """Handle analyze calibration logs."""
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
                """Handle median."""
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
        """Handle run calibration wizard."""
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
            """Handle wizard on configure."""
            try:
                wizard_canvas.configure(scrollregion=wizard_canvas.bbox("all"))
                wizard_canvas.itemconfigure(body_id, width=wizard_canvas.winfo_width())
            except Exception:
                pass

        def _wizard_mousewheel(event):
            """Handle wizard mousewheel."""
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
            """Handle safe auton ident."""
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

        style_options = ["Atticus", "JAR", "JAR (advanced)", "LemLib", "Custom"]
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
            """Handle refresh auton fn."""
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
            """Handle parse list."""
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
            """Handle parse caps."""
            caps = _parse_list(raw, float)
            if not caps:
                return []
            is_volts = (mode == "Volts")
            if is_volts or any(v > 1.5 for v in caps):
                return [max(0.1, min(1.2, v / 12.0)) for v in caps]
            return [max(0.1, min(1.2, v)) for v in caps]

        def _update_caps_interp(*_):
            """Update caps interp."""
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
            """Handle apply preset."""
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
            """Handle refresh drive err note."""
            if str(track_var.get()) == "0":
                drive_err_source_var.set("Tape measure")
                drive_err_note.configure(text="Drive settle error will be approximate unless you enter measured error.")
            else:
                if drive_err_source_var.get() == "Tape measure":
                    drive_err_source_var.set("Tracking odom")
                drive_err_note.configure(text="")

        def _export_plan():
            """Handle export plan."""
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
                        """Handle plan comment."""
                        raw = json.dumps(pl, indent=2)
                        return "\n".join(["// " + line for line in raw.splitlines()])

                    def _emit_code(pl):
                        """Handle emit code."""
                        def _cpp_ident(raw_name: str) -> str:
                            """Handle cpp ident."""
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
                            """Handle cpp string."""
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
                        turn_key = modes.get("turn", "turn_global" if style in ("LemLib", "Atticus", "AttLib") else "turn_local")
                        if move_key not in tpls:
                            move_key = "move" if "move" in tpls else next(iter(tpls.keys()))
                        if turn_key not in tpls:
                            turn_key = "turn_global" if "turn_global" in tpls else ("turn_local" if "turn_local" in tpls else move_key)

                        def _normalize_tpl(val):
                            """Handle normalize tpl."""
                            if isinstance(val, (list, tuple)):
                                return [str(v) for v in val if str(v).strip()]
                            if isinstance(val, str):
                                parts = [p.strip() for p in val.split("||")]
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

                        def _format_tpl_lines(tpl_val, tokens: dict):
                            """Handle format tpl lines."""
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
                            "ASYNC": "false",
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
                            "FORWARDS": "true",
                            "FORWARD_PARAM": "{.forwards = true}",
                            "ASYNC": "false",
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
            """Return log text."""
            raw = text.get("1.0", "end-1c")
            if raw.strip() == placeholder:
                return ""
            return raw

        def _update_key_status(raw_text: str):
            """Update key status."""
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
            """Handle on text modified."""
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
            """Handle load from file."""
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
            """Handle analyze."""
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
                """Handle fmt noise."""
                try:
                    return f"{float(val):.3f}"
                except Exception:
                    return "n/a"
            drive_noise = _fmt_noise(noise.get("drive_in", None))
            turn_noise = _fmt_noise(noise.get("turn_deg", None))
            def _median(vals):
                """Handle median."""
                vals = sorted([v for v in vals if v is not None])
                if not vals:
                    return None
                mid = len(vals) // 2
                if len(vals) % 2:
                    return vals[mid]
                return 0.5 * (vals[mid - 1] + vals[mid])

            def _bucket_summary(move, prof):
                """Handle bucket summary."""
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
            """Handle apply."""
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
            """Handle copy example."""
            try:
                win.clipboard_clear()
                win.clipboard_append(example_line)
            except Exception:
                pass

        placeholder = "Paste robot JSON log lines here..."
        text.insert("1.0", placeholder)
        text.configure(foreground="#888888")

        def _clear_placeholder(_evt=None):
            """Handle clear placeholder."""
            if text.get("1.0", "end-1c") == placeholder:
                text.delete("1.0", "end")
                text.configure(foreground="#000000")

        def _restore_placeholder(_evt=None):
            """Handle restore placeholder."""
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
    cal_frame.grid(row=12, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 2))
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
        """Handle refresh ticks visibility."""
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
    
    tabs["codegen"].rowconfigure(6, weight=1)
    tabs["codegen"].columnconfigure(1, weight=1)
    tpl_container = ttk.Frame(tabs["codegen"])
    tpl_container.grid(row=6, column=0, columnspan=2, sticky="nsew", padx=6, pady=6)
    tpl_container.rowconfigure(0, weight=1)
    tpl_container.columnconfigure(0, weight=1)
    tabs["codegen"].rowconfigure(6, weight=1)
    tabs["codegen"].columnconfigure(0, weight=1)

    tpl_panel = ttk.Frame(tpl_container, style="TFrame")
    tpl_panel.grid(row=0, column=0, sticky="nsew")
    tpl_canvas = tpl_panel  # placeholder to satisfy bindings
    tpl_win = None

    def _tpl_on_configure(event=None):
        """Handle tpl on configure."""
        pass

    def _tpl_mousewheel(event):
        """Handle tpl mousewheel."""
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
        """Handle active optional for."""
        stored = CFG.get("codegen", {}).get("templates", {}).get(style_name, {})
        active = stored.get("__optional__", [])
        if isinstance(active, list):
            return list(active)
        fallback = ["marker_wait", "marker_wait_done"]
        for key in optional_pool:
            if key in fallback:
                continue
            if key in stored:
                fallback.append(key)
        return fallback

    def _set_active_optional(style_name: str, lst):
        """Set active optional."""
        CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})["__optional__"] = list(lst)
        return list(lst)

    def _add_optional(key):
        """Handle add optional."""
        style = codegen_style_var.get()
        active = _active_optional_for(style)
        if key not in active:
            active.append(key)
            _set_active_optional(style, active)
            _rebuild_tpl_panel()

    def _remove_optional(key):
        """Handle remove optional."""
        style = codegen_style_var.get()
        active = _active_optional_for(style)
        if key in active:
            active.remove(key)
            _set_active_optional(style, active)
            _rebuild_tpl_panel()

    def _default_modes(style_name: str):
        """Handle default modes."""
        return {
            "motion": "move",
            "turn": "turn_global" if style_name in ("LemLib", "Atticus", "AttLib") else "turn_local"
        }

    def _current_modes(style_name: str):
        """Handle current modes."""
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
        """Set modes."""
        CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})["__modes__"] = dict(modes)
        motion_mode_var.set(modes.get("motion", "move"))
        turn_mode_var.set(modes.get("turn", "turn_global" if style_name in ("LemLib", "Atticus", "AttLib") else "turn_local"))
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

    _LOCKED_TEMPLATE_STYLES = {"JAR"}  # "JAR (advanced)" remains editable

    def _save_tpl_vars_for(style_name: str):
        """Save UI variables to config."""
        if style_name == "Action List":
            return
        locked = style_name in _LOCKED_TEMPLATE_STYLES
        tdict = CFG.setdefault("codegen", {}).setdefault("templates", {}).setdefault(style_name, {})
        active_opt = _active_optional_for(style_name)
        if not locked:
            for k in base_tpl_keys + optional_pool:
                if k in optional_pool and k not in active_opt:
                    tdict.pop(k, None)
                    continue
                tdict[k] = tpl_vars[k].get()
        tdict["__optional__"] = active_opt
        _set_modes(style_name, _current_modes(style_name))

    def _apply_mode_selection(style_name: str, motion_val=None, turn_val=None):
        """Handle apply mode selection."""
        modes = _current_modes(style_name)
        if motion_val in ("move", "pose"):
            modes["motion"] = motion_val
        if turn_val in ("turn_local", "turn_global"):
            modes["turn"] = turn_val
        _set_modes(style_name, modes)
        return modes

    def _on_mode_change(_=None):
        """Handle on mode change."""
        style = codegen_style_var.get()
        modes = _apply_mode_selection(style, motion_mode_var.get(), turn_mode_var.get())
        _set_modes(style, modes)
        _rebuild_tpl_panel()

    template_builder_win = {"win": None}
    mech_preset_win = {"win": None}

    def _open_template_builder():
        """Handle open template builder."""
        style = codegen_style_var.get()
        if style == "Action List":
            messagebox.showinfo("Edit templates", "Action List mode has no templates to edit.")
            return
        if style in _LOCKED_TEMPLATE_STYLES:
            messagebox.showinfo(
                "Templates locked",
                "The JAR template is locked to prevent accidental edits.\n\n"
                "Use 'JAR (advanced)' if you need a customizable JAR variant."
            )
            return
        _fill_tpl_vars_for(style)
        if template_builder_win["win"] is not None and template_builder_win["win"].winfo_exists():
            template_builder_win["win"].lift()
            template_builder_win["win"].focus_force()
            return

        win = tk.Toplevel(top)
        template_builder_win["win"] = win
        tutorial_state["flags"]["opened_template_editor"] = True
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
        opt_row.columnconfigure(1, weight=1)
        ttk.Label(opt_row, text="Optional commands:").grid(row=0, column=0, sticky="nw")
        opt_grid = ttk.Frame(opt_row)
        opt_grid.grid(row=0, column=1, sticky="w")
        opt_map = {
            "reverse_on": "Reverse on",
            "reverse_off": "Reverse off",
            "reshape_on": "Reshape on",
            "reshape_off": "Reshape off",
            "setpose": "Set pose",
            "swing": "Swing",
            "path_follow": "Path follow",
            "marker_wait": "Marker wait",
            "marker_wait_done": "Marker wait done"
        }
        active_optional = set(_active_optional_for(style))
        opt_vars = {}
        opt_widgets = []
        wrap_cols = 3
        for idx, (key, label) in enumerate(opt_map.items()):
            var = tk.BooleanVar(value=key in active_optional)
            opt_vars[key] = var
            chk = ttk.Checkbutton(opt_grid, text=label, variable=var)
            row = idx // wrap_cols
            col = idx % wrap_cols
            chk.grid(row=row, column=col, sticky="w", padx=(8, 0), pady=(0, 2))
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
            """Handle render parts."""
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
            """Handle refresh preview."""
            preview = "".join(
                f"{{{p['value']}}}" if p["kind"] == "token" else p["value"]
                for p in parts
            )
            preview_text.configure(state="normal")
            preview_text.delete("1.0", "end")
            preview_text.insert("1.0", preview)
            preview_text.configure(state="disabled")

        def _split_template(text):
            """Handle split template."""
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
            """Handle allowed tokens for cmd."""
            base = {
                "wait": ["MS", "S", "TIMEOUT_MS", "TIMEOUT_S"],
                "tbuffer": ["MS", "S", "TIMEOUT_MS", "TIMEOUT_S"],
                "move": [
                    "X_IN", "Y_IN", "DIST_IN", "DIST_ROT", "DIST_DEG", "DIST_TICKS",
                    "HEADING_DEG", "HEADING_RAD", "FORWARDS", "FORWARD_PARAM", "ASYNC", "MOVE_SPEED",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
                    "DRIVE_MIN_SPEED", "DRIVE_EARLY_EXIT", "LEAD_IN"
                ],
                "pose": [
                    "X_IN", "Y_IN", "DIST_IN", "DIST_ROT", "DIST_DEG", "DIST_TICKS",
                    "HEADING_DEG", "HEADING_RAD", "FORWARDS", "FORWARD_PARAM", "ASYNC", "LEAD_IN", "MOVE_SPEED",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
                    "DRIVE_MIN_SPEED", "DRIVE_EARLY_EXIT"
                ],
                "turn_global": [
                    "HEADING_DEG", "HEADING_RAD", "TURN_DELTA_DEG", "TURN_DELTA_RAD",
                    "TARGET_X_IN", "TARGET_Y_IN", "FORWARDS", "FORWARD_PARAM", "ASYNC", "TURN_SPEED",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "TURN_MAX_V", "TURN_SETTLE_ERR", "TURN_SETTLE_TIME",
                    "TURN_MIN_SPEED", "TURN_EARLY_EXIT"
                ],
                "turn_local": [
                    "TURN_DELTA_DEG", "TURN_DELTA_RAD", "HEADING_DEG", "HEADING_RAD",
                    "TARGET_X_IN", "TARGET_Y_IN", "FORWARDS", "FORWARD_PARAM", "ASYNC", "TURN_SPEED",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "TURN_MAX_V", "TURN_SETTLE_ERR", "TURN_SETTLE_TIME",
                    "TURN_MIN_SPEED", "TURN_EARLY_EXIT"
                ],
                "swing": [
                    "HEADING_DEG", "HEADING_RAD", "TURN_DELTA_DEG", "TURN_DELTA_RAD",
                    "TARGET_X_IN", "TARGET_Y_IN", "FORWARDS", "FORWARD_PARAM", "ASYNC", "DIR",
                    "SIDE", "SIDE_LC", "LOCKED_SIDE", "LOCKED_SIDE_LC",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "SWING_MAX_V", "SWING_SETTLE_ERR", "SWING_SETTLE_TIME",
                    "SWING_MIN_SPEED", "SWING_EARLY_EXIT"
                ],
                "path_follow": [
                    "PATH_NAME", "PATH_FILE", "PATH_ASSET", "LOOKAHEAD",
                    "TIMEOUT_MS", "TIMEOUT_S", "MS", "S",
                    "FORWARDS", "FORWARD_PARAM", "ASYNC", "PATH_MIN_SPEED", "PATH_MAX_SPEED",
                    "DRIVE_MAX_V", "HEADING_MAX_V", "DRIVE_SETTLE_ERR", "DRIVE_SETTLE_TIME",
                    "DRIVE_MIN_SPEED", "DRIVE_EARLY_EXIT"
                ],
                "setpose": ["X_IN", "Y_IN", "HEADING_DEG", "HEADING_RAD", "NAME"],
                "reshape_on": ["STATE", "NAME"],
                "reshape_off": ["STATE", "NAME"],
                "reverse_on": [],
                "reverse_off": [],
                "marker_wait": ["MARKER_DIST_IN", "MARKER_FRAC", "MARKER_INDEX", "NAME", "TIMEOUT_MS", "TIMEOUT_S"],
                "marker_wait_done": ["NAME"]
            }
            tokens = list(base.get(cmd_name, []))
            seen = set()
            out = []
            for tok in tokens:
                if tok not in seen:
                    seen.add(tok)
                    out.append(tok)
            return out

        def _refresh_palette(cmd_name):
            """Handle refresh palette."""
            palette_list.delete(0, "end")
            tokens = _allowed_tokens_for_cmd(cmd_name)
            if not tokens:
                tokens = list(template_tokens)
            for token in sorted(tokens):
                palette_list.insert("end", f"{{{token}}}")

        def _load_command(name):
            """Handle load command."""
            current_cmd["name"] = name
            tpl = tpl_vars.get(name).get()
            if not tpl:
                tpl = codegen_defaults.get(style, codegen_defaults["Custom"]).get(name, "")
            parts.clear()
            parts.extend(_split_template(tpl))
            _render_parts()
            _refresh_palette(name)

        def _active_cmds_for_builder():
            """Handle active cmds for builder."""
            turn_key = turn_choice_var.get() or ("turn_global" if style in ("LemLib", "Atticus", "AttLib") else "turn_local")
            base_cmds = ["wait", "move", "pose", turn_key, "tbuffer"]
            optional_cmds = [k for k in optional_pool if opt_vars.get(k) and opt_vars[k].get()]
            return base_cmds + optional_cmds

        def _refresh_cmd_list(select_name=None):
            """Handle refresh cmd list."""
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
            """Update optional."""
            active = [k for k, var in opt_vars.items() if var.get()]
            _set_active_optional(style, active)
            _refresh_cmd_list(current_cmd["name"])
            on_update()

        def _update_turn_choice():
            """Update turn choice."""
            modes = {"motion": motion_mode_var.get(), "turn": turn_choice_var.get()}
            _set_modes(style, modes)
            _refresh_cmd_list(current_cmd["name"])
            on_update()

        def _apply_current_command():
            """Handle apply current command."""
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
            """Handle apply and close."""
            _apply_current_command()
            win.destroy()

        def _add_text_part():
            """Handle add text part."""
            val = simpledialog.askstring("Add text", "Enter literal text to insert:", parent=win)
            if val is None:
                return
            parts.append({"kind": "text", "value": val})
            _render_parts()

        def _edit_text_part():
            """Handle edit text part."""
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
            """Handle remove part."""
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
            """Handle clear parts."""
            parts.clear()
            _render_parts()

        def _reset_command():
            """Handle reset command."""
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
            """Handle start drag palette."""
            idx = palette_list.nearest(event.y)
            if idx < 0:
                return
            val = palette_list.get(idx)
            drag_state.update({"source": "palette", "index": None, "token": val.strip("{}")})

        def _start_drag_parts(event):
            """Handle start drag parts."""
            idx = parts_list.nearest(event.y)
            if idx < 0:
                return
            parts_list.selection_clear(0, "end")
            parts_list.selection_set(idx)
            selected_part_idx["idx"] = idx
            drag_state.update({"source": "parts", "index": idx, "token": None})

        def _drop_on_parts_at(y_root=None, y_local=None):
            """Handle drop on parts at."""
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
            """Handle cancel drag."""
            drag_state.update({"source": None, "index": None, "token": None})

        def _insert_token_at_end(event):
            """Handle insert token at end."""
            idx = palette_list.nearest(event.y)
            if idx < 0:
                return
            token = palette_list.get(idx).strip("{}")
            parts.append({"kind": "token", "value": token})
            selected_part_idx["idx"] = len(parts) - 1
            _render_parts()

        def _drop_anywhere(event):
            """Handle drop anywhere."""
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
            """Handle on select cmd."""
            sel = cmd_list.curselection()
            if not sel:
                return
            name = cmd_list.get(sel[0])
            _load_command(name)

        cmd_list.bind("<<ListboxSelect>>", _on_select_cmd)
        _refresh_cmd_list()

    def _open_mech_preset_editor():
        """Handle open mech preset editor."""
        if mech_preset_win["win"] is not None and mech_preset_win["win"].winfo_exists():
            mech_preset_win["win"].lift()
            mech_preset_win["win"].focus_force()
            return
        presets = CFG.setdefault("codegen", {}).setdefault("mech_presets", [])
        if not isinstance(presets, list):
            presets = []
            CFG["codegen"]["mech_presets"] = presets
        _ensure_locked_mech_presets(CFG)
        for p in presets:
            if not isinstance(p, dict):
                continue
            locked_key = _locked_mech_preset_key(p.get("name", ""))
            if locked_key is None:
                continue
            p.clear()
            p.update(_locked_mech_preset_spec(locked_key))
            p["cases"] = []
            p["case_default"] = ""

        win = tk.Toplevel(top)
        mech_preset_win["win"] = win
        win.title("Mechanism presets")
        win.geometry("760x500")
        win.minsize(660, 420)

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
        right.rowconfigure(6, weight=1)

        preset_list = tk.Listbox(left, height=12, exportselection=False)
        preset_scroll = ttk.Scrollbar(left, orient="vertical", command=preset_list.yview)
        preset_list.configure(yscrollcommand=preset_scroll.set)
        preset_list.grid(row=0, column=0, sticky="nsew")
        preset_scroll.grid(row=0, column=1, sticky="ns")

        btn_row = ttk.Frame(left)
        btn_row.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        add_btn = ttk.Button(btn_row, text="Add", command=lambda: _add_preset())
        add_btn.pack(side="left")
        remove_btn = ttk.Button(btn_row, text="Remove", command=lambda: _remove_preset())
        remove_btn.pack(side="left", padx=(6, 0))

        name_var = tk.StringVar()
        mode_var = tk.StringVar(value="Action")
        default_var = tk.IntVar(value=0)
        current_idx = {"idx": None}

        ttk.Label(right, text="Name:").grid(row=0, column=0, sticky="w", pady=(0, 4))
        name_entry = ttk.Entry(right, textvariable=name_var)
        name_entry.grid(row=0, column=1, sticky="ew", pady=(0, 4))

        ttk.Label(right, text="Type:").grid(row=1, column=0, sticky="w", pady=(0, 4))
        mode_combo = ttk.Combobox(right, values=["Action", "Toggle", "Cases"], state="readonly", textvariable=mode_var, width=12)
        mode_combo.grid(row=1, column=1, sticky="w", pady=(0, 4))

        default_chk = ttk.Checkbutton(right, text="Default ON", variable=default_var)
        default_chk.grid(row=2, column=1, sticky="w", pady=(0, 6))

        action_lbl = ttk.Label(right, text="Action template:")
        action_lbl.grid(row=3, column=0, sticky="nw")
        action_text = tk.Text(right, height=4, wrap="word")
        action_text.grid(row=3, column=1, sticky="ew", pady=(0, 6))

        on_lbl = ttk.Label(right, text="Toggle ON template:")
        on_lbl.grid(row=4, column=0, sticky="nw")
        on_text = tk.Text(right, height=3, wrap="word")
        on_text.grid(row=4, column=1, sticky="ew", pady=(0, 6))

        off_lbl = ttk.Label(right, text="Toggle OFF template:")
        off_lbl.grid(row=5, column=0, sticky="nw")
        off_text = tk.Text(right, height=3, wrap="word")
        off_text.grid(row=5, column=1, sticky="ew", pady=(0, 6))

        cases_lbl = ttk.Label(right, text="Cases:")
        cases_lbl.grid(row=6, column=0, sticky="nw")
        cases_section = ttk.Frame(right)
        cases_section.grid(row=6, column=1, sticky="nsew", pady=(0, 6))
        cases_section.columnconfigure(0, weight=1)
        cases_section.rowconfigure(1, weight=1)

        cases_toolbar = ttk.Frame(cases_section)
        cases_toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        cases_toolbar.columnconfigure(0, weight=1)
        cases_help_lbl = ttk.Label(
            cases_toolbar,
            text="Add as many as needed. Trigger token goes above output template."
        )
        cases_help_lbl.grid(row=0, column=0, sticky="w")
        add_case_btn = ttk.Button(cases_toolbar, text="+ Add Case")
        add_case_btn.grid(row=0, column=1, sticky="e", padx=(8, 0))

        def _layout_cases_toolbar(_evt=None):
            """Handle layout cases toolbar."""
            try:
                w = max(120, int(cases_toolbar.winfo_width()))
            except Exception:
                w = 120
            wrap = max(120, w - 140)
            try:
                cases_help_lbl.configure(wraplength=wrap)
            except Exception:
                pass
            if w < 340:
                add_case_btn.grid_configure(row=1, column=0, sticky="e", padx=0, pady=(4, 0))
            else:
                add_case_btn.grid_configure(row=0, column=1, sticky="e", padx=(8, 0), pady=0)

        cases_toolbar.bind("<Configure>", _layout_cases_toolbar)

        cases_canvas = tk.Canvas(cases_section, height=170, highlightthickness=0)
        cases_scroll = ttk.Scrollbar(cases_section, orient="vertical", command=cases_canvas.yview)
        cases_canvas.configure(yscrollcommand=cases_scroll.set)
        cases_canvas.grid(row=1, column=0, sticky="nsew")
        cases_scroll.grid(row=1, column=1, sticky="ns")
        cases_rows_frame = ttk.Frame(cases_canvas)
        cases_rows_win = cases_canvas.create_window((0, 0), window=cases_rows_frame, anchor="nw")

        case_rows = []

        def _on_cases_frame_config(_evt=None):
            """Handle on cases frame config."""
            try:
                cases_canvas.configure(scrollregion=cases_canvas.bbox("all"))
                cases_canvas.itemconfigure(cases_rows_win, width=max(10, cases_canvas.winfo_width()))
            except Exception:
                pass

        cases_rows_frame.bind("<Configure>", _on_cases_frame_config)
        cases_canvas.bind("<Configure>", _on_cases_frame_config)

        def _cases_mousewheel(event):
            """Handle cases mousewheel."""
            try:
                if getattr(event, "delta", 0):
                    step = -1 if event.delta > 0 else 1
                elif getattr(event, "num", None) == 4:
                    step = -1
                elif getattr(event, "num", None) == 5:
                    step = 1
                else:
                    return
                cases_canvas.yview_scroll(step, "units")
            except Exception:
                pass

        def _bind_case_wheel(widget):
            """Handle bind case wheel."""
            try:
                widget.bind("<MouseWheel>", _cases_mousewheel, add="+")
                widget.bind("<Button-4>", _cases_mousewheel, add="+")
                widget.bind("<Button-5>", _cases_mousewheel, add="+")
            except Exception:
                pass

        _bind_case_wheel(cases_canvas)
        _bind_case_wheel(cases_rows_frame)

        def _clear_case_rows():
            """Handle clear case rows."""
            for row in case_rows:
                try:
                    row["frame"].destroy()
                except Exception:
                    pass
            case_rows.clear()
            _on_cases_frame_config()

        def _set_case_rows_enabled(enabled: bool):
            """Set case rows enabled."""
            st = "normal" if enabled else "disabled"
            try:
                add_case_btn.configure(state=st)
            except Exception:
                pass
            for row in case_rows:
                try:
                    row["trigger_entry"].configure(state=st)
                    row["template_text"].configure(state=st)
                    row["remove_btn"].configure(state=st)
                except Exception:
                    pass

        def _collect_case_rows():
            """Handle collect case rows."""
            out = []
            for row in case_rows:
                key = row["trigger_var"].get().strip()
                tpl = row["template_text"].get("1.0", "end-1c").strip()
                if not key:
                    continue
                out.append({"key": key, "template": tpl})
            return out

        def _add_case_row(trigger="", template="", auto_save=True):
            """Handle add case row."""
            row_idx = len(case_rows)
            row_frame = ttk.Frame(cases_rows_frame, padding=(0, 0, 0, 6))
            row_frame.grid(row=row_idx, column=0, sticky="ew")
            row_frame.columnconfigure(0, weight=1)

            top_line = ttk.Frame(row_frame)
            top_line.grid(row=0, column=0, sticky="ew", pady=(0, 3))
            top_line.columnconfigure(1, weight=1)
            trigger_lbl = ttk.Label(top_line, text="Trigger:")
            trigger_lbl.grid(row=0, column=0, sticky="w", padx=(0, 6))
            trig_var = tk.StringVar(value=str(trigger or ""))
            trig_entry = ttk.Entry(top_line, textvariable=trig_var)
            trig_entry.grid(row=0, column=1, sticky="ew")

            ttk.Label(row_frame, text="Output template:").grid(row=1, column=0, sticky="w")
            out_text = tk.Text(row_frame, height=3, wrap="word")
            out_text.grid(row=2, column=0, sticky="ew")
            out_text.insert("1.0", str(template or ""))
            remove_btn_case = ttk.Button(row_frame, text="Remove")
            remove_btn_case.grid(row=3, column=0, sticky="w", pady=(4, 0))

            row_data = {
                "frame": row_frame,
                "trigger_var": trig_var,
                "trigger_entry": trig_entry,
                "template_text": out_text,
                "remove_btn": remove_btn_case,
            }
            case_rows.append(row_data)

            def _remove_this():
                """Handle remove this."""
                if row_data not in case_rows:
                    return
                case_rows.remove(row_data)
                try:
                    row_frame.destroy()
                except Exception:
                    pass
                for idx2, rw in enumerate(case_rows):
                    rw["frame"].grid_configure(row=idx2)
                _on_cases_frame_config()
                _save_current()

            remove_btn_case.configure(command=_remove_this)
            trig_entry.bind("<FocusOut>", lambda _e: _save_current())
            trig_entry.bind("<KeyRelease>", lambda _e: _save_current())
            out_text.bind("<FocusOut>", lambda _e: _save_current())
            out_text.bind("<KeyRelease>", lambda _e: _save_current())
            _bind_case_wheel(trig_entry)
            _bind_case_wheel(out_text)
            _bind_case_wheel(row_frame)

            _on_cases_frame_config()
            if auto_save:
                _save_current()

        def _load_case_rows(cases):
            """Handle load case rows."""
            _clear_case_rows()
            loaded_any = False
            if isinstance(cases, list):
                for item in cases:
                    if not isinstance(item, dict):
                        continue
                    key = str(item.get("key", "")).strip()
                    tpl = str(item.get("template", "") or "")
                    if not key:
                        continue
                    _add_case_row(key, tpl, auto_save=False)
                    loaded_any = True
            if not loaded_any:
                _add_case_row("", "", auto_save=False)
            _on_cases_frame_config()

        add_case_btn.configure(command=lambda: _add_case_row("", ""))

        cases_default_lbl = ttk.Label(right, text="Cases default template:")
        cases_default_lbl.grid(row=7, column=0, sticky="nw")
        cases_default_text = tk.Text(right, height=3, wrap="word")
        cases_default_text.grid(row=7, column=1, sticky="ew", pady=(0, 6))

        hint_lbl = ttk.Label(
            right,
            text="Hint: {VALUE} = all values after the preset name; {VALUE1}/{VALUE2}/{VALUE3} = first three.\n"
                 "Toggle templates also get {STATE} (on/off). Cases use Trigger + Output rows.\n"
                 "Use \\n in template text to force a line break in exported output.\n"
                 f"Reshape and DSR are locked here. DSR emits a distance reset; use it while settled about {ATTICUS_DSR_RECOMMENDED_STILL_MS:g} ms."
        )
        hint_lbl.grid(row=8, column=0, columnspan=2, sticky="w", pady=(4, 0))

        def _preset_label(preset):
            """Handle preset label."""
            name = str(preset.get("name", "")).strip() or "(unnamed)"
            mode = str(preset.get("mode", "action")).strip().lower()
            return f"{name} ({mode})"

        def _refresh_list(select_idx=None):
            """Handle refresh list."""
            preset_list.delete(0, "end")
            for p in presets:
                preset_list.insert("end", _preset_label(p))
            if presets:
                idx = 0 if select_idx is None else max(0, min(select_idx, len(presets) - 1))
                preset_list.selection_set(idx)
                preset_list.see(idx)
                _load_selected(idx)

        def _refresh_list_row(idx):
            """Handle refresh list row."""
            if idx is None or idx < 0 or idx >= len(presets):
                return
            try:
                preset_list.delete(idx)
                preset_list.insert(idx, _preset_label(presets[idx]))
                preset_list.selection_clear(0, "end")
                preset_list.selection_set(idx)
                preset_list.see(idx)
            except Exception:
                pass

        def _set_text(widget, text_val):
            """Set text."""
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.insert("1.0", text_val)

        def _get_text(widget):
            """Return text."""
            return widget.get("1.0", "end-1c").strip()

        def _is_locked():
            """Check whether is locked."""
            idx = current_idx.get("idx")
            if idx is None or idx >= len(presets):
                return False
            return _locked_mech_preset_key(presets[idx].get("name", "")) is not None

        def _refresh_mode_visibility():
            """Handle refresh mode visibility."""
            mode = mode_var.get().strip().lower()
            is_toggle = mode == "toggle"
            is_cases = mode == "cases"
            locked = _is_locked()
            if locked:
                name_entry.configure(state="disabled")
                mode_combo.configure(state="disabled")
                default_chk.configure(state="disabled")
                action_text.configure(state="disabled")
                on_text.configure(state="disabled")
                off_text.configure(state="disabled")
                _set_case_rows_enabled(False)
                cases_default_text.configure(state="disabled")
                remove_btn.configure(state="disabled")
            else:
                name_entry.configure(state="normal")
                mode_combo.configure(state="readonly")
                default_chk.configure(state="normal" if is_toggle else "disabled")
                on_text.configure(state="normal" if is_toggle else "disabled")
                off_text.configure(state="normal" if is_toggle else "disabled")
                action_text.configure(state="normal" if (not is_toggle and not is_cases) else "disabled")
                _set_case_rows_enabled(is_cases)
                cases_default_text.configure(state="normal" if is_cases else "disabled")
                remove_btn.configure(state="normal")

            show_action = (mode == "action") and not locked
            show_toggle = (mode == "toggle") and not locked
            show_cases = (mode == "cases") and not locked
            if locked:
                show_action = show_toggle = show_cases = True
            if show_action:
                action_lbl.grid()
                action_text.grid()
            else:
                action_lbl.grid_remove()
                action_text.grid_remove()
            if show_toggle:
                on_lbl.grid()
                on_text.grid()
                off_lbl.grid()
                off_text.grid()
            else:
                on_lbl.grid_remove()
                on_text.grid_remove()
                off_lbl.grid_remove()
                off_text.grid_remove()
            if show_cases:
                cases_lbl.grid()
                cases_section.grid()
                cases_default_lbl.grid()
                cases_default_text.grid()
            else:
                cases_lbl.grid_remove()
                cases_section.grid_remove()
                cases_default_lbl.grid_remove()
                cases_default_text.grid_remove()
            if show_toggle or locked:
                default_chk.grid()
            else:
                default_chk.grid_remove()

        def _save_current(refresh_list_row=False):
            """Handle save current."""
            idx = current_idx.get("idx")
            if idx is None or idx >= len(presets):
                return
            preset = presets[idx]
            locked_key = _locked_mech_preset_key(preset.get("name", ""))
            if locked_key is not None:
                locked_spec = _locked_mech_preset_spec(locked_key)
                name = locked_spec.get("name", "")
                mode = str(locked_spec.get("mode", "action"))
                default = False
                template = ""
                on_tpl = ""
                off_tpl = ""
                mode_var.set("Toggle" if mode == "toggle" else "Action")
                default_var.set(0)
                _set_text(action_text, "")
                _set_text(on_text, "")
                _set_text(off_text, "")
                _load_case_rows([])
                _set_text(cases_default_text, "")
            else:
                name = name_var.get().strip() or f"Preset {idx + 1}"
                mode = mode_var.get().strip().lower()
                if mode not in ("action", "toggle", "cases"):
                    mode = "action"
                default = bool(default_var.get()) if mode == "toggle" else False
                template = _get_text(action_text) if mode == "action" else ""
                on_tpl = _get_text(on_text) if mode == "toggle" else ""
                off_tpl = _get_text(off_text) if mode == "toggle" else ""
            preset["name"] = name
            preset["mode"] = mode
            preset["default"] = default
            preset["template"] = template
            preset["on"] = on_tpl
            preset["off"] = off_tpl
            preset["cases"] = _collect_case_rows() if mode == "cases" else []
            preset["case_default"] = _get_text(cases_default_text) if mode == "cases" else ""
            if refresh_list_row:
                _refresh_list_row(idx)

        default_chk.configure(command=_save_current)

        def _load_selected(idx):
            """Handle load selected."""
            if idx is None or idx >= len(presets):
                return
            current_idx["idx"] = idx
            preset = presets[idx]
            name_var.set(str(preset.get("name", "")))
            mode_val = str(preset.get("mode", "action")).strip().lower()
            if mode_val == "toggle":
                mode_var.set("Toggle")
            elif mode_val == "cases":
                mode_var.set("Cases")
            else:
                mode_var.set("Action")
            default_var.set(1 if preset.get("default") else 0)
            _set_text(action_text, str(preset.get("template", "")))
            _set_text(on_text, str(preset.get("on", "")))
            _set_text(off_text, str(preset.get("off", "")))
            _load_case_rows(preset.get("cases", []))
            _set_text(cases_default_text, str(preset.get("case_default", "")))
            _refresh_mode_visibility()

        def _add_preset():
            """Handle add preset."""
            presets.append({
                "name": f"Preset {len(presets) + 1}",
                "mode": "action",
                "template": "",
                "on": "",
                "off": "",
                "default": False,
                "cases": [],
                "case_default": ""
            })
            _refresh_list(len(presets) - 1)

        def _remove_preset():
            """Handle remove preset."""
            sel = preset_list.curselection()
            if not sel:
                return
            idx = sel[0]
            if _locked_mech_preset_key(presets[idx].get("name", "")) is not None:
                return
            presets.pop(idx)
            current_idx["idx"] = None
            _refresh_list(idx if idx < len(presets) else len(presets) - 1)

        def _on_select(_evt=None):
            """Handle on select."""
            sel = preset_list.curselection()
            if not sel:
                return
            _save_current(refresh_list_row=True)
            _load_selected(sel[0])

        def _apply_and_close():
            """Handle apply and close."""
            _save_current(refresh_list_row=True)
            try:
                save_config(CFG)
            except Exception:
                pass
            win.destroy()

        mode_combo.bind("<<ComboboxSelected>>", lambda _e: (_refresh_mode_visibility(), _save_current(refresh_list_row=True)))
        preset_list.bind("<<ListboxSelect>>", _on_select)

        action_text.bind("<FocusOut>", lambda _e: _save_current())
        on_text.bind("<FocusOut>", lambda _e: _save_current())
        off_text.bind("<FocusOut>", lambda _e: _save_current())
        cases_default_text.bind("<FocusOut>", lambda _e: _save_current())
        name_entry.bind("<FocusOut>", lambda _e: _save_current())
        action_text.bind("<KeyRelease>", lambda _e: _save_current())
        on_text.bind("<KeyRelease>", lambda _e: _save_current())
        off_text.bind("<KeyRelease>", lambda _e: _save_current())
        cases_default_text.bind("<KeyRelease>", lambda _e: _save_current())
        name_entry.bind("<KeyRelease>", lambda _e: _save_current())

        _refresh_list(0 if presets else None)

        btns = ttk.Frame(container)
        btns.grid(row=1, column=0, columnspan=2, sticky="e", pady=(8, 0))
        ttk.Button(btns, text="Close", command=_apply_and_close).pack(side="right")
        win.protocol("WM_DELETE_WINDOW", _apply_and_close)

    def _rebuild_tpl_panel(*_):
        """Rebuild the template editor panel based on selected style."""
        for w in list(tpl_panel.children.values()):
            try: w.destroy()
            except Exception: pass
        
        style = codegen_style_var.get()
        if style == "Action List":
            ttk.Label(tpl_panel, text="Action List mode uses default log output.\nNo templates to configure.", 
                    justify="center").grid(row=0, column=0, columnspan=2, pady=20)
            return

        _fill_tpl_vars_for(style)
        modes = _current_modes(style)
        active_optional = _active_optional_for(style)

        hdr1 = ttk.Label(tpl_panel, text="CMD", font=("Segoe UI", 10, "bold"))
        hdr2 = ttk.Label(tpl_panel, text="Template", font=("Segoe UI", 10, "bold"))
        hdr1.grid(row=1, column=0, sticky="w", padx=(0, 6))
        hdr2.grid(row=1, column=1, sticky="ew")
        tpl_panel.columnconfigure(1, weight=1)
        
        help_map = {
            "wait": "{MS} or {S} delay tokens. TIMEOUT_MS/S include pad- and min floor.",
            "move": "{DIST_IN}/{X_IN},{Y_IN} {FORWARDS}; {HEADING_DEG} (global) optional; {ASYNC}; {MOVE_SPEED} optional cmd (0-127) override.",
            "turn_global": "Field heading {HEADING_DEG}; target point {TARGET_X_IN},{TARGET_Y_IN}; {FORWARDS}; {ASYNC}; {TURN_SPEED} optional dps.",
            "turn_local": "Relative turn {TURN_DELTA_DEG}; target point {TARGET_X_IN},{TARGET_Y_IN}; {FORWARDS}; {ASYNC}; {TURN_SPEED} optional dps.",
            "pose": "{X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}; {FORWARDS}; {ASYNC}.",
            "setpose": "Set initial pose {X_IN},{Y_IN},{HEADING_DEG}",
            "reshape_on": "Reshape ON command template; {STATE}=ON value for selected output mode.",
            "reshape_off": "Reshape OFF command template; {STATE}=OFF value for selected output mode.",
            "reverse_on": "Toggle reverse drive ON",
            "reverse_off": "Toggle reverse drive OFF",
            "tbuffer": "{MS} or {S} for buffer wait",
            "path_follow": "Tokens: {PATH_NAME}, {PATH_FILE}, {PATH_ASSET}, {LOOKAHEAD}, {TIMEOUT_MS}, {FORWARDS}, {ASYNC}, {PATH_MIN_SPEED}, {PATH_MAX_SPEED} (cmd 0-127).",
            "swing": "{HEADING_DEG} target, point {TARGET_X_IN},{TARGET_Y_IN}, {FORWARDS}, {ASYNC}, {DIR}=AUTO/CW_CLOCKWISE/CCW_COUNTERCLOCKWISE, {SIDE}=LEFT/RIGHT/AUTO, {LOCKED_SIDE}=RIGHT/LEFT/AUTO {TIMEOUT_MS}.",
            "marker_wait": "Edge marker wait: {MARKER_DIST_IN} inches (or use {MARKER_FRAC} 0-1).",
            "marker_wait_done": "Finish motion after markers (e.g. waitUntilDone)."
        }
        if style == "JAR (advanced)" or (adv_motion_var.get() and style == "JAR"):
            adv_drive_note = " Auto caps/settle: short=precise, long=fast; voltage caps scale with move size; settle err/time scale with move size and cap."
            adv_turn_note = " Auto caps/settle: small angles=precise, large=fast; voltage/settle scale with angle and cap."
            adv_swing_note = " Auto caps/settle: small swings=precise, large=fast; voltage/settle scale with swing angle and cap."
            help_map["move"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["pose"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["path_follow"] += " JAR: {DRIVE_MAX_V}, {HEADING_MAX_V}, {DRIVE_SETTLE_ERR}, {DRIVE_SETTLE_TIME}." + adv_drive_note
            help_map["turn_global"] += " JAR: {TURN_MAX_V}, {TURN_SETTLE_ERR}, {TURN_SETTLE_TIME}." + adv_turn_note
            help_map["turn_local"] += " JAR: {TURN_MAX_V}, {TURN_SETTLE_ERR}, {TURN_SETTLE_TIME}." + adv_turn_note
            help_map["swing"] += " JAR: {SWING_MAX_V}, {SWING_SETTLE_ERR}, {SWING_SETTLE_TIME}." + adv_swing_note
        
        turn_key = modes.get("turn") or ("turn_global" if style in ("LemLib", "Atticus", "AttLib") else "turn_local")
        base_cmds = ["wait", "move", "pose", turn_key, "tbuffer"]
        active_cmds = base_cmds + active_optional
        
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
            """Handle on list config."""
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
        
        btns = ttk.Frame(tpl_panel)
        btns.grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        locked = style in _LOCKED_TEMPLATE_STYLES
        edit_btn = ttk.Button(btns, text="Edit command templates...", command=_open_template_builder)
        edit_btn.pack(side="left", padx=(0, 6))
        if locked:
            try:
                edit_btn.configure(state="disabled")
            except Exception:
                pass

        ttk.Button(btns, text="Mechanism presets...", command=_open_mech_preset_editor).pack(side="left", padx=(0, 6))
        reset_btn = ttk.Button(btns, text="Reset to Defaults", command=_reset_defaults)
        reset_btn.pack(side="left")
        if locked:
            try:
                reset_btn.configure(state="disabled")
            except Exception:
                pass
        tpl_panel.after(0, _bind_tpl_live_handlers)

    try:
        adv_motion_var.trace_add("write", lambda *_: _rebuild_tpl_panel())
    except Exception:
        pass
        
    def _reset_defaults():
        """Handle reset defaults."""
        style = codegen_style_var.get()
        _set_active_optional(style, _default_optional_for_style(style))
        _set_modes(style, _default_modes(style))
        for k in base_tpl_keys + optional_pool:
            if k in codegen_defaults.get(style, codegen_defaults["Custom"]):
                tpl_vars[k].set(codegen_defaults.get(style, codegen_defaults["Custom"]).get(k, ""))
        _rebuild_tpl_panel()

    def _persist_now():
        """Handle persist now."""
        style = codegen_style_var.get()
        _save_tpl_vars_for(style)
        CFG.setdefault("codegen", {})["style"] = style
        def _f_or(var, default=0.0):
            try:
                return float(var.get() or default)
            except Exception:
                return float(default)
        CFG["codegen"].setdefault("opts", {}).update({
            "ticks_per_rotation": float(ticks_var.get() or 360),
            "pad_factor": float(pad_var.get() or 1.0),
            "min_timeout_s": float(min_s_var.get() or 0.0),
            "reshape_output": reshape_output_var.get() or "1/2",
            "default_drive_early_exit": _f_or(default_drive_exit_var, 0.0),
            "default_turn_early_exit": _f_or(default_turn_exit_var, 0.0),
            "default_swing_early_exit": _f_or(default_swing_exit_var, 0.0),
            "default_swing_settle_early_exit": _f_or(default_settle_swing_exit_var, 0.0),
            "omit_defaults": int(omit_defaults_var.get())
        })
        CFG["codegen"].setdefault("attlib_sim", {}).update({
            "visual_run": int(attlib_sim_visual_var.get()),
            "module_dir": attlib_sim_module_dir_var.get().strip() or _default_attlib_sim_module_dir(),
            "source": "external_script" if int(attlib_sim_external_var.get()) else "timeline",
            "routine_script": attlib_sim_script_var.get().strip() or _default_attlib_sim_routine_script(),
            "routine_function": attlib_sim_func_var.get().strip() or "run_routine",
        })
        tpl_panel.after(0, _bind_tpl_live_handlers)

    def _on_style_change(_=None):
        """Handle style dropdown change."""
        _rebuild_tpl_panel()
        _refresh_jar_defaults_visibility()
        _refresh_attlib_general_visibility()

    def _refresh_attlib_general_visibility(_evt=None):
        """Keep AttLibSim helper visible in General settings."""
        try:
            attlib_sim_frame.grid()
        except Exception:
            pass
    style_widget.bind("<<ComboboxSelected>>", _on_style_change)
    _refresh_attlib_general_visibility()

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
            global attlib_sim_visual_run, attlib_sim_module_dir, attlib_sim_source
            global attlib_sim_routine_script, attlib_sim_routine_func
            
            CFG["field_centric"] = 1
            CFG["distance_units"] = dist_map[dist_var.get()]
            CFG["angle_units"] = ang_map[ang_var.get()]
            
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
            
            CFG.setdefault("ui", {})["show_hitboxes"] = int(show_hitboxes_var.get())
            if not show_hitboxes_var.get():
                show_hitbox_conflicts_only_var.set(0)
            CFG.setdefault("ui", {})["show_hitbox_conflicts_only"] = int(show_hitbox_conflicts_only_var.get())
            CFG.setdefault("ui", {})["show_field_objects"] = int(show_field_objects_var.get())
            CFG.setdefault("ui", {})["show_node_numbers"] = int(show_node_numbers_var.get())
            CFG["reshape_label"] = reshape_label_var.get().strip() or "Reshape"
            
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
            bd["advanced_geometry"] = {
                "enabled": int(bool(advanced_geometry_state.get("enabled", 0))),
                "symmetry": int(bool(advanced_geometry_state.get("symmetry", 0))),
                "points": _sanitize_advanced_poly(advanced_geometry_state.get("points", [])),
                "reshape_points": _sanitize_advanced_poly(advanced_geometry_state.get("reshape_points", [])),
            }
            
            CFG["offsets"]["offset_1_in"] = float(off1.get())
            CFG["offsets"]["offset_2_in"] = float(off2.get())
            CFG["offsets"]["offset_3_in"] = float(off3.get())
            CFG["offsets"]["padding_in"] = float(pad.get())
            
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
            
            path_lookahead_enabled = bool(pc.get("simulate_pursuit", 1))
            path_lookahead_px = max(0.0, float(pc.get("lookahead_in", auto_lookahead_in(CFG))) * PPI)
            last_lookahead_radius = path_lookahead_px

            mcl = CFG.get("atticus", CFG.get("mcl", {}))
            if not isinstance(mcl, dict):
                mcl = {}
            CFG["atticus"] = mcl
            CFG["mcl"] = mcl
            prev_ekf = mcl.get("ekf") if isinstance(mcl.get("ekf"), dict) else {}
            def _mcl_f(var, default=0.0):
                """Parse a float-like settings field without aborting apply."""
                try:
                    return float(var.get())
                except Exception:
                    return float(default)

            def _mcl_i(var, default=0):
                """Parse an int-like settings field without aborting apply."""
                try:
                    return int(var.get())
                except Exception:
                    try:
                        return int(float(var.get()))
                    except Exception:
                        return int(default)

            mcl["enabled"] = _mcl_i(mcl_enabled_var, 0)
            mcl["config_version"] = int(MCL_CONFIG_VERSION)
            mcl["loop_ms"] = float(mcl_cfg.get("loop_ms", 20.0))
            mcl["stall_ms"] = float(mcl_cfg.get("stall_ms", max(1.0, 2.0 * mcl["loop_ms"])))
            mcl["motion_ms"] = _mcl_f(mcl_motion_ms_var, 20.0)
            mcl["sensor_ms"] = _mcl_f(mcl_sensor_ms_var, 20.0)
            mcl["motion"] = {
                "enabled": _mcl_i(mcl_motion_enabled_var, 1),
                "motion_model": "drive",
                "motion_source": "encoders",
                "sigma_x_in": _mcl_f(mcl_sigma_x_var, 0.08),
                "sigma_y_in": _mcl_f(mcl_sigma_y_var, 0.08),
                "sigma_theta_deg": _mcl_f(mcl_sigma_theta_var, 0.7),
                "sigma_x_per_in": _mcl_f(mcl_sigma_x_per_var, 0.02),
                "sigma_y_per_in": _mcl_f(mcl_sigma_y_per_var, 0.03),
                "sigma_theta_per_deg": _mcl_f(mcl_sigma_theta_per_var, 0.05),
                "no_horizontal_lateral_scale": _mcl_f(mcl_no_horizontal_scale_var, 2.5),
                "turn_lateral_scale": _mcl_f(mcl_turn_lateral_scale_var, 2.0),
                "rough_wall_sigma_x_scale": _mcl_f(mcl_rough_wall_sigma_x_scale_var, 1.8),
                "rough_wall_sigma_y_scale": _mcl_f(mcl_rough_wall_sigma_y_scale_var, 3.5),
                "rough_wall_sigma_theta_scale": _mcl_f(mcl_rough_wall_sigma_theta_scale_var, 1.25),
                "delta_guard_enabled": int(motion_cfg.get("delta_guard_enabled", 1)),
                "max_dx_in_per_tick": float(motion_cfg.get("max_dx_in_per_tick", 0.0)),
                "max_dy_in_per_tick": float(motion_cfg.get("max_dy_in_per_tick", 0.0)),
                "max_dtheta_deg_per_tick": float(motion_cfg.get("max_dtheta_deg_per_tick", 0.0)),
                "guard_vmax_in_s": float(motion_cfg.get("guard_vmax_in_s", 60.0)),
                "guard_wmax_deg_s": float(motion_cfg.get("guard_wmax_deg_s", 540.0)),
                "guard_margin_in": float(motion_cfg.get("guard_margin_in", 0.5)),
                "guard_margin_deg": float(motion_cfg.get("guard_margin_deg", 8.0)),
                "fault_inflate_cycles": int(motion_cfg.get("fault_inflate_cycles", 2)),
                "fault_noise_scale": float(motion_cfg.get("fault_noise_scale", 2.5)),
            }
            mcl["tracking"] = {
                "horizontal_enabled": 1 if str(mcl_horizontal_odom_var.get() or "").strip().lower() == "enabled" else 0,
            }
            mcl["set_pose_sigma_xy_in"] = _mcl_f(mcl_set_pose_xy_var, 0.2)
            mcl["set_pose_sigma_theta_deg"] = _mcl_f(mcl_set_pose_theta_var, 2.0)
            mcl["interop"] = {
                "pose_convention": str(mcl_pose_convention_var.get() or "cw_zero_forward").strip().lower(),
                "swap_xy": _mcl_i(mcl_pose_swap_xy_var, 0),
                "invert_x": _mcl_i(mcl_pose_invert_x_var, 0),
                "invert_y": _mcl_i(mcl_pose_invert_y_var, 0),
            }
            dist_fov_multi = int(dist_cfg.get("fov_multi_ray", 1))
            dist_rays_per_sensor = int(dist_cfg.get("rays_per_sensor", 3))
            mcl["sensors"] = {
                "distance": {
                    "enabled": _mcl_i(mcl_dist_enabled_var, 1),
                    "model": "likelihood_field",
                    "sigma_hit_mm": _mcl_f(mcl_dist_sigma_var, 15.0),
                    "sigma_far_scale": float(dist_cfg.get("sigma_far_scale", 0.05)),
                    "sigma_min_mm": float(dist_cfg.get("sigma_min_mm", 15.0)),
                    "sigma_max_mm": float(dist_cfg.get("sigma_max_mm", 100.0)),
                    "conf_sigma_scale": float(dist_cfg.get("conf_sigma_scale", 1.0)),
                    "min_sensor_weight": float(dist_cfg.get("min_sensor_weight", 1e-6)),
                    "w_hit": _mcl_f(mcl_dist_w_hit_var, 0.9),
                    "w_rand": _mcl_f(mcl_dist_w_rand_var, 0.1),
                    "w_short": _mcl_f(mcl_dist_w_short_var, 0.0),
                    "w_max": _mcl_f(mcl_dist_w_max_var, 0.0),
                    "lambda_short": _mcl_f(mcl_dist_lambda_short_var, 0.1),
                    "max_range_mm": _mcl_f(mcl_dist_max_range_var, 2000.0),
                    "min_range_mm": _mcl_f(mcl_dist_min_range_var, 20.0),
                    "confidence_min": _mcl_f(mcl_dist_conf_min_var, 0.0),
                    "object_size_min": _mcl_f(mcl_dist_obj_size_min_var, 0.0),
                    "object_size_max": _mcl_f(mcl_dist_obj_size_max_var, 0.0),
                    "innovation_gate_mm": _mcl_f(mcl_dist_innov_gate_var, 180.0),
                    "median_window": _mcl_f(mcl_dist_median_window_var, 3),
                    "batch_size": int(dist_cfg.get("batch_size", 3)),
                    "lf_ignore_max": _mcl_i(mcl_dist_ignore_max_var, 0),
                    "use_no_object_info": int(dist_cfg.get("use_no_object_info", 0)),
                    "fov_multi_ray": dist_fov_multi,
                    "rays_per_sensor": dist_rays_per_sensor,
                    "fov_half_deg_near": float(dist_cfg.get("fov_half_deg_near", 18.0)),
                    "fov_half_deg_far": float(dist_cfg.get("fov_half_deg_far", 12.0)),
                    "fov_switch_mm": float(dist_cfg.get("fov_switch_mm", 203.0)),
                    "raycast_bucket_in": float(dist_cfg.get("raycast_bucket_in", 12.0)),
                    "gate_mm": _mcl_f(mcl_dist_gate_var, 180.0),
                    "gate_mode": str(mcl_dist_gate_mode_var.get() or "hard").strip(),
                    "gate_penalty": _mcl_f(mcl_dist_gate_penalty_var, 0.05),
                    "gate_reject_ratio": _mcl_f(mcl_dist_gate_reject_var, 0.9),
                    "likelihood_field": {
                        "resolution_in": _mcl_f(mcl_dist_lf_res_var, 1.0),
                        "max_bytes": int((dist_cfg.get("likelihood_field", {}) or {}).get("max_bytes", 262144)),
                    }
                },
                "imu": {
                    "enabled": _mcl_i(mcl_imu_enabled_var, 1),
                    "sigma_deg": _mcl_f(mcl_imu_sigma_var, 1.0),
                    "check_calibrating": int(imu_cfg.get("check_calibrating", 1)),
                    "fallback_noise_scale": float(imu_cfg.get("fallback_noise_scale", 2.0)),
                }
            }
            dist_sensors = []
            for idx, sv in enumerate(mcl_sensor_vars):
                enabled = _mcl_i(sv["enabled"], 1)
                name = str(sv["name"].get() or "").strip()
                if not name:
                    name = f"sensor_{idx + 1}"
                dist_sensors.append({
                    "name": name,
                    "x_in": _mcl_f(sv["x_in"], 0.0),
                    "y_in": _mcl_f(sv["y_in"], 0.0),
                    "angle_deg": _mcl_f(sv["angle_deg"], 0.0),
                    "bias_mm": _mcl_f(sv["bias_mm"], 0.0),
                    "angle_offset_deg": _mcl_f(sv["angle_offset_deg"], 0.0),
                    "min_range_mm": _mcl_f(sv["min_range_mm"], 20.0),
                    "max_range_mm": _mcl_f(sv["max_range_mm"], 2000.0),
                    "min_confidence": _mcl_f(sv["min_confidence"], 0.0),
                    "min_object_size": _mcl_f(sv["min_object_size"], 0.0),
                    "max_object_size": _mcl_f(sv["max_object_size"], 0.0),
                    "innovation_gate_mm": _mcl_f(sv["innovation_gate_mm"], 180.0),
                    "map_mode": str(sv["map_mode"].get() or "perimeter").strip(),
                    "enabled": enabled,
                })
            mcl["sensor_geometry"] = {"distance_sensors": dist_sensors}
            mcl["map_objects"] = {
                "perimeter": _mcl_i(mcl_map_perimeter_var, 1),
                "long_goals": _mcl_i(mcl_map_long_goals_var, 1),
                "long_goal_braces": _mcl_i(mcl_map_long_goal_braces_var, 1),
                "center_goals": _mcl_i(mcl_map_center_goals_var, 1),
                "matchloaders": _mcl_i(mcl_map_matchloaders_var, 1),
                "park_zones": _mcl_i(mcl_map_park_zones_var, 0),
            }
            mcl["object_selection"] = {k: _mcl_i(v, 0) for k, v in mcl_object_vars.items()}
            sensor_object_visibility = {}
            for idx, sv in enumerate(mcl_sensor_vars):
                sensor_name = str(sv["name"].get() or "").strip() or f"sensor_{idx + 1}"
                vis_vars = mcl_sensor_object_vars.get(idx, {})
                vis_payload = {obj_id: _mcl_i(var, 1) for obj_id, var in vis_vars.items()}
                sensor_object_visibility[str(idx)] = dict(vis_payload)
                sensor_object_visibility[sensor_name] = dict(vis_payload)
            mcl["sensor_object_visibility"] = sensor_object_visibility
            mcl["correction"] = {
                "enabled": _mcl_i(mcl_corr_enabled_var, 1),
                "min_confidence": _mcl_f(mcl_corr_min_conf_var, 0.6),
                "max_trans_jump_in": _mcl_f(mcl_corr_max_trans_var, 4.0),
                "max_theta_jump_deg": _mcl_f(mcl_corr_max_theta_var, 8.0),
                "writeback_alpha": _mcl_f(mcl_corr_writeback_alpha_var, 0.35),
                "teleport_reset_trans_in": _mcl_f(mcl_corr_teleport_trans_var, 18.0),
                "teleport_reset_theta_deg": _mcl_f(mcl_corr_teleport_theta_var, 45.0),
                "safe_window_enabled": int((mcl_cfg.get("correction", {}) or {}).get("safe_window_enabled", 1)),
                "safe_max_speed_in_s": float((mcl_cfg.get("correction", {}) or {}).get("safe_max_speed_in_s", 8.0)),
                "safe_max_turn_deg_s": float((mcl_cfg.get("correction", {}) or {}).get("safe_max_turn_deg_s", 60.0)),
            }
            mcl["ui"] = {
                "show_estimate": _mcl_i(mcl_show_estimate_var, 1),
                "show_covariance": _mcl_i(mcl_show_cov_var, 1),
                "show_rays": _mcl_i(mcl_show_rays_var, 1),
                "show_gating": _mcl_i(mcl_show_gating_var, 1),
            }
            ekf_cfg_out = dict(prev_ekf)
            ekf_cfg_out["use_imu_update"] = _mcl_i(mcl_ekf_use_imu_var, 1)
            ekf_cfg_out["sigma_dx_in"] = mcl["motion"]["sigma_x_in"]
            ekf_cfg_out["sigma_dy_in"] = mcl["motion"]["sigma_y_in"]
            ekf_cfg_out["sigma_dtheta_deg"] = mcl["motion"]["sigma_theta_deg"]
            mcl["ekf"] = ekf_cfg_out
            for legacy_key in (
                "particles",
                "resample",
                "kld",
                "augmented",
                "random_injection",
                "recovery",
                "frame_sign_self_test",
                "region",
                "confidence",
                "mode_split",
                "cgr_lite",
                "tuning",
            ):
                mcl.pop(legacy_key, None)
            sensors_out = mcl.get("sensors", {})
            if isinstance(sensors_out, dict):
                sensors_out.pop("vision", None)
                mcl["sensors"] = sensors_out
            ui_out = mcl.get("ui", {})
            if isinstance(ui_out, dict):
                ui_out.pop("show_particles", None)
                ui_out.pop("show_region", None)
                ui_out.pop("max_particles_draw", None)
                mcl["ui"] = ui_out

            CFG.setdefault("codegen", {})["style"] = codegen_style_var.get()
            opts = CFG["codegen"].setdefault("opts", {})
            def _f_or(var, default=0.0):
                try:
                    return float(var.get() or default)
                except Exception:
                    return float(default)
            opts.update({
                "ticks_per_rotation": float(ticks_var.get() or 360),
                "pad_factor": float(pad_var.get() or 1.0),
                "min_timeout_s": float(min_s_var.get() or 0.0),
                "reshape_output": reshape_output_var.get() or "1/2",
                "default_drive_early_exit": _f_or(default_drive_exit_var, 0.0),
                "default_turn_early_exit": _f_or(default_turn_exit_var, 0.0),
                "default_swing_early_exit": _f_or(default_swing_exit_var, 0.0),
                "default_swing_settle_early_exit": _f_or(default_settle_swing_exit_var, 0.0),
                "omit_defaults": int(omit_defaults_var.get())
            })
            attlib_cfg = CFG["codegen"].setdefault("attlib_sim", {})
            attlib_cfg["visual_run"] = int(attlib_sim_visual_var.get())
            attlib_cfg["module_dir"] = attlib_sim_module_dir_var.get().strip() or _default_attlib_sim_module_dir()
            attlib_cfg["source"] = "external_script" if int(attlib_sim_external_var.get()) else "timeline"
            attlib_cfg["routine_script"] = attlib_sim_script_var.get().strip() or _default_attlib_sim_routine_script()
            attlib_cfg["routine_function"] = attlib_sim_func_var.get().strip() or "run_routine"
            attlib_sim_visual_run = bool(attlib_cfg["visual_run"])
            attlib_sim_module_dir = str(attlib_cfg["module_dir"])
            attlib_sim_source = str(attlib_cfg["source"])
            attlib_sim_routine_script = str(attlib_cfg["routine_script"])
            attlib_sim_routine_func = str(attlib_cfg["routine_function"])
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

            save_config(CFG)
            CFG.pop("_phys_cache", None)
            
            _attlib_sim_stop()
            moving = False; paused = False; show_chevron = False
            timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
            robot_pos = display_nodes[0]["pos"]; robot_heading = initial_state["heading"]
            _mcl_apply_cfg(reset_particles=True)
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
    global attlib_sim_status
    
    open_settings_window()
    _mcl_apply_cfg(reset_particles=True)
    
    running = True
    while running:
        clock.tick(60)
        dt_s = 1.0 / max(1.0, float(fps))
        dt_ms = dt_s * 1000.0
        ui.pump_tk()
        
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
                 else (("preset",
                        str(a.get("name", "")).strip().lower(),
                        (None if a.get("state", None) is None else str(a.get("state")).strip().lower()),
                        (_preset_value_text(a) or None))) if a.get("type") == "preset"
                 else (("code", str(a.get("code", "")).strip())) if a.get("type") == "code"
                 else (("reshape" if a.get("type") in ("reshape", "geom") else a.get("type")),)
                 for a in n.get("actions", [])
             ),
             (None if _node_lateral_cmd(n) is None else round(float(_node_lateral_cmd(n)), 3)),
             bool(_node_lateral_reset(n)),
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
        for event in pygame.event.get():
            if event.type == pygame.QUIT: 
                running = False
            
            if event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()
                
                if event.key == pygame.K_o:
                    _refresh_output_header()
                    ui.toggle_output_window(log_lines)
                
                if event.key == pygame.K_p and not (mods & pygame.KMOD_CTRL):
                    if path_edit_mode:
                        exit_path_edit_mode()
                    else:
                        target_idx = selected_idx if selected_idx is not None else hover_idx
                        if target_idx is not None and target_idx < len(display_nodes) - 1:
                            enter_path_edit_mode(target_idx)
                            selected_idx = target_idx

                elif event.key == pygame.K_ESCAPE and path_edit_mode:
                    exit_path_edit_mode()
                
                elif event.key == pygame.K_a and path_edit_mode and not (mods & pygame.KMOD_CTRL):
                    add_control_point_at_mouse(mouse_pos)
                
                elif event.key == pygame.K_x and path_edit_mode and selected_control_point is not None:
                    remove_control_point(selected_control_point)

                elif path_edit_mode and event.key == pygame.K_u:
                    toggle_path_spline_type()

                elif path_edit_mode and event.key == pygame.K_m:
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
                    tutorial_state["flags"]["compiled_once"] = True
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
                            st = seg.get("type")
                            if st == "marker":
                                label = _marker_actions_to_text(seg.get("actions", []))
                                log_action("marker", label=label)
                                continue
                            if st == "atticus_correction":
                                mode = str(seg.get("mode", "")).strip().lower()
                                if mode == "immediate":
                                    log_action("atticus_immediate")
                                elif mode == "wall_trim_start":
                                    log_action("atticus_wall_trim_start")
                                elif mode == "wall_trim_end":
                                    log_action("atticus_wall_trim_end")
                                continue
                            if T <= 0.0:
                                continue
                            if st == "move":
                                if seg.get("atticus_immediate"):
                                    log_action("atticus_immediate")
                                log_action("move", i0=seg.get("i0", ...), i1=seg.get("i1", ...), 
                                          p0=seg["p0"], p1=seg["p1"], reverse=seg.get("reverse", False))
                            elif st == "path":
                                if seg.get("atticus_immediate"):
                                    log_action("atticus_immediate")
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
                        if attlib_sim_runtime.get("running"):
                            attlib_sim_status = "Pause is not supported in AttLibSim visual mode. Use Ctrl+Space to reset."
                            _attlib_sim_log(attlib_sim_status)
                        else:
                            paused = not paused
                    else:
                        use_attlib = _attlib_sim_should_run()
                        use_external = (_attlib_sim_source_mode() == "external_script")
                        script_ready = bool(str(attlib_sim_routine_script or "").strip()) and Path(str(attlib_sim_routine_script)).exists()
                        if not use_external and len(display_nodes) < 2 and script_ready:
                            use_external = True
                            _attlib_sim_log(
                                "No plotted nodes; auto-switching to external routine mode "
                                f"({attlib_sim_routine_script}::{attlib_sim_routine_func})."
                            )
                        if not use_external and len(display_nodes) < 2:
                            _attlib_sim_log(
                                "SPACE ignored: need at least 2 nodes to simulate "
                                "(external routine mode is not active)."
                            )
                            continue

                        correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                        sync_all_path_endpoints()
                        log_lines.clear()
                        log_lines.extend(build_compile_header(CFG, initial_state["heading"]))
                        _attlib_sim_log(_attlib_sim_mode_reason())
                        ui.output_refresh(log_lines)
                        moving = True; paused = False; show_chevron = True
                        tutorial_state["flags"]["ran_sim"] = True
                        robot_pos = display_nodes[0]["pos"]
                        robot_heading = initial_state["heading"]
                        reshape_live = False
                        _mcl_apply_cfg(reset_particles=True)

                        if use_external:
                            timeline = []
                            total_estimate_s = 0.0
                            seg_i = 0; t_local = 0.0; last_logged_seg = -1
                            if not _attlib_sim_start([]):
                                moving = False
                                show_chevron = False
                                _attlib_sim_log(attlib_sim_status)
                        else:
                            timeline = build_timeline_with_buffers()
                            if not timeline:
                                moving = False
                                show_chevron = False
                                _attlib_sim_log("SPACE ignored: timeline is empty. Add at least one motion segment.")
                                continue
                            _attlib_sim_log(f"Timeline ready: {len(timeline)} segments.")
                            total_estimate_s = compute_total_from_timeline(timeline)
                            if total_estimate_s <= 1e-6:
                                _attlib_sim_log(
                                    "Timeline duration is near zero; motion may appear instantaneous/no visible movement."
                                )
                            seg_i = 0; t_local = 0.0; last_logged_seg = -1
                            if use_attlib:
                                if not _attlib_sim_start(timeline):
                                    _attlib_sim_log(f"{attlib_sim_status} Falling back to internal simulator.")
                            else:
                                _attlib_sim_log("Running internal simulator path.")
                
                elif event.key == pygame.K_SPACE and (mods & pygame.KMOD_CTRL):
                    _attlib_sim_stop()
                    moving = False; paused = False; show_chevron = False
                    robot_pos = display_nodes[0]["pos"]
                    robot_heading = initial_state["heading"]
                    reshape_live = False
                    _mcl_apply_cfg(reset_particles=True)
                    timeline.clear(); seg_i = 0; t_local = 0.0; last_logged_seg = -1
                
                elif event.key == pygame.K_f:
                    tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                    if tgt is not None and tgt >= 0 and tgt != 0:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        try:
                            off = int(display_nodes[tgt].get("offset", 0))
                        except Exception:
                            off = 0
                        if off not in (0, 1, 2, 3):
                            off = 0
                        new_off = (off + 1) % 4
                        display_nodes[tgt]["offset"] = new_off
                        if new_off != 0:
                            display_nodes[tgt].pop("offset_custom_in", None)
                            if display_nodes[tgt].get("offset_ghost_angle") is None and tgt > 0:
                                pd_prev = display_nodes[tgt - 1].get("path_to_next", {})
                                if pd_prev.get("use_path", False):
                                    ang = heading_from_points(display_nodes[tgt - 1]["pos"], display_nodes[tgt]["pos"])
                                    display_nodes[tgt]["offset_ghost_angle"] = ang
                                else:
                                    display_nodes[tgt].pop("offset_ghost_angle", None)
                        else:
                            display_nodes[tgt].pop("offset_ghost_angle", None)
                        util_push_undo_prev(undo_stack, prev)
                        last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                        last_path_sig = None
                        total_estimate_s = compute_total_estimate_s()

                elif event.key == pygame.K_b and not path_edit_mode:
                    if _atticus_style_selected():
                        tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                        if tgt is not None and tgt >= 0:
                            prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                            node = display_nodes[tgt]
                            current_mode = _node_atticus_correction_mode(node)
                            cycle = [None, "immediate", "wall_trim", "rough_wall"] if tgt < len(display_nodes) - 1 else [None, "immediate"]
                            try:
                                idx_mode = cycle.index(current_mode)
                            except Exception:
                                idx_mode = 0
                            next_mode = cycle[(idx_mode + 1) % len(cycle)]
                            _set_node_atticus_correction_mode(node, next_mode)
                            util_push_undo_prev(undo_stack, prev)
                            last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                            last_path_sig = None
                            total_estimate_s = compute_total_estimate_s()
                
                elif event.key == pygame.K_r:
                    tgt = selected_idx if selected_idx is not None else (len(display_nodes)-1 if display_nodes else None)
                    if tgt is not None and tgt >= 0:
                        prev = util_snapshot(display_nodes, robot_pos, robot_heading)
                        node = display_nodes[tgt]
                        acts = list(node.get("actions", []))
                        rev_idxs = [i for i, a in enumerate(acts) if a.get("type") == "reverse"]
                        if rev_idxs:
                            has_toggle = any(acts[i].get("state", None) is None for i in rev_idxs)
                            if has_toggle:
                                acts = [a for a in acts if a.get("type") != "reverse"]
                            else:
                                last_state = acts[rev_idxs[-1]].get("state", False)
                                new_state = not bool(last_state)
                                acts = [a for a in acts if a.get("type") != "reverse"]
                                acts.append({"type": "reverse", "state": new_state})
                            node["actions"] = acts
                            node["actions_out"] = list(acts)
                            node["reverse"] = False
                        else:
                            node["reverse"] = not node.get("reverse", False)
                        _normalize_node_mech_payload(node)
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
                    tutorial_state["flags"]["save_load_used"] = True
                    for _node in display_nodes:
                        _normalize_node_mech_payload(_node)
                    save_nodes(initial_state, display_nodes)
                
                elif event.key == pygame.K_l:
                    tutorial_state["flags"]["save_load_used"] = True
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
                            _normalize_node_mech_payload(n)
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
            

            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    if path_edit_mode:
                        hover_cp = None
                        for i, cp in enumerate(path_control_points):
                            if math.hypot(cp[0] - mouse_pos[0], cp[1] - mouse_pos[1]) <= PATH_CONTROL_RADIUS:
                                hover_cp = i
                                break
                        
                        if hover_cp is not None:
                            selected_control_point = hover_cp
                            dragging_control_point = True
                            continue
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
                                tutorial_state["flags"]["shift_insert"] = True
                                events = display_nodes[seg_idx].get("edge_events", [])
                                ev_left, ev_right = _split_edge_events(events, t_split)
                                if ev_left is not None:
                                    if ev_left:
                                        display_nodes[seg_idx]["edge_events"] = ev_left
                                    else:
                                        display_nodes[seg_idx].pop("edge_events", None)
                                    if ev_right:
                                        display_nodes[seg_idx + 1]["edge_events"] = ev_right
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
                                tutorial_state["flags"]["added_node"] = True
                                correct_nodes_inbounds(display_nodes, CFG, initial_state["heading"], WINDOW_WIDTH, WINDOW_HEIGHT)
                                sync_all_path_endpoints()
                                util_push_undo_prev(undo_stack, prev)
                                last_snapshot = util_snapshot(display_nodes, robot_pos, robot_heading)
                                last_path_sig = None
                                total_estimate_s = compute_total_estimate_s()
                
                elif event.button == 3:  # Right click
                    if path_edit_mode:
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
                                tutorial_state["flags"]["path_speed_prompt"] = True
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
                                start_node = display_nodes[hit_seg]
                                node_pos_locked = tuple(start_node.get("pos", (0, 0)))
                                init_cmd = compile_cmd_string(start_node, hit_seg)
                                tutorial_state["flags"]["right_click_prompt"] = True
                                cmd = _askstring_centered(
                                    "Node actions",
                                    "Commands (comma/semicolon separated). Examples:\n"
                                    "  turn 45, wait 2, turn 90\n"
                                    "  wait 1.5; turn -30\n"
                                    "  offset 7   (custom offset)\n"
                                    "  reshape / reshape on / reshape off\n"
                                    "  reverse    (toggle / reverse on/off)\n"
                                    f"  DSR (legacy alias for pink immediate correction; use settled about {ATTICUS_DSR_RECOMMENDED_STILL_MS:g} ms)\n"
                                    "  lift 100 200 300   (preset values -> {VALUE}/{VALUE1-3})\n"
                                    "  clamp on 1 2 3     (toggle preset + values, {STATE})\n"
                                    "  code Intake.spin(100, pct); (raw mechanism code)\n"
                                    "  latspeed 50   (drive cmd override 0-127)\n"
                                    "  turnspeed 180 (deg/s override)\n"
                                    "  correct immediate / correct walltrim / correct roughwall / correct off\n"
                                    "  chain [0-1/off] (chain through this node; optional looseness)\n",
                                    initialvalue=init_cmd
                                )
                                changed = False
                                if cmd is not None:
                                    parse_and_apply_cmds(start_node, cmd, hit_seg)
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
                            tutorial_state["flags"]["right_click_prompt"] = True
                            cmd = _askstring_centered(
                                "Node actions",
                                "Commands (comma/semicolon separated). Examples:\n"
                                "  turn 45, wait 2, turn 90\n"
                                "  wait 1.5; turn -30\n"
                                "  offset 7   (custom offset)\n"
                                "  reshape / reshape on / reshape off\n"
                                "  reverse    (toggle / reverse on/off)\n"
                                f"  DSR (legacy alias for pink immediate correction; use settled about {ATTICUS_DSR_RECOMMENDED_STILL_MS:g} ms)\n"
                                "  lift 100 200 300   (preset values -> {VALUE}/{VALUE1-3})\n"
                                "  clamp on 1 2 3     (toggle preset + values, {STATE})\n"
                                "  code Intake.spin(100, pct); (raw mechanism code)\n"
                                "  latspeed 50   (drive cmd override 0-127)\n"
                                "  turnspeed 180 (deg/s override)\n"
                                "  correct immediate / correct walltrim / correct roughwall / correct off\n"
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
                    tutorial_state["flags"]["dragged_node"] = True
                    mods_now = pygame.key.get_mods()
                    ctrl_now = bool(mods_now & pygame.KMOD_CTRL)
                    if ctrl_now and not constrain_active:
                        constrain_active = True
                        constrain_origin = tuple(display_nodes[selected_idx]["pos"])
                    elif not ctrl_now and constrain_active:
                        constrain_active = False
                        constrain_origin = None
                    if ctrl_now or constrain_active:
                        tutorial_state["flags"]["axis_lock_used"] = True
                    
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
            

        if not history_freeze and sig != last_path_sig:
            last_path_sig = sig
            moving = False; paused = False; show_chevron = False
            _clear_atticus_runtime()
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

        if not moving and attlib_sim_runtime.get("running"):
            _attlib_sim_stop()

        if moving and attlib_sim_runtime.get("running"):
            _attlib_sim_step(dt_s)
        elif moving and not paused:
            if seg_i >= len(timeline):
                _atticus_end_rough_wall()
                _atticus_end_wall_trim()
                moving = False
            else:
                seg = timeline[seg_i]
                T = seg["T"]
                motion_progress = None
                if seg_i != last_logged_seg and t_local <= 1e-9 and T >= 0:
                    if seg["type"] in ("move", "path", "path_follow") and seg.get("atticus_immediate"):
                        log_action("atticus_immediate")
                        _atticus_apply_immediate_correction()
                    if seg["type"] == "move":
                        log_action("move", i0=seg.get("i0", seg_i), i1=seg.get("i1", seg_i+1), 
                                  p0=seg["p0"], p1=seg["p1"], reverse=seg.get("reverse", False))
                    elif seg["type"] == "path":
                        log_action("path", i0=seg.get("i0", seg_i), i1=seg.get("i1", seg_i+1),
                                   p0=seg.get("p0", seg.get("path_points", [robot_pos])[0]),
                                   p1=seg.get("p1", seg.get("path_points", [robot_pos])[-1]),
                                   path_points=seg.get("path_points", []))
                    elif seg["type"] == "atticus_correction":
                        mode = str(seg.get("mode", "")).strip().lower()
                        if mode == "immediate":
                            log_action("atticus_immediate")
                        elif mode == "wall_trim_start":
                            log_action("atticus_wall_trim_start")
                        elif mode == "wall_trim_end":
                            log_action("atticus_wall_trim_end")
                        elif mode == "rough_wall_start":
                            log_action("atticus_rough_wall_start")
                        elif mode == "rough_wall_end":
                            log_action("atticus_rough_wall_end")
                    elif seg["type"] == "turn":
                        log_action("turn", h0=seg["start_heading"], h1=seg["target_heading"])
                    elif seg["type"] == "wait":
                        try:
                            _tbuf = float(CFG.get("robot_physics", {}).get("t_buffer", 0.0))
                        except Exception:
                            _tbuf = 0.0
                        if not (seg_i == len(timeline)-1 and abs(seg.get("T", 0.0) - _tbuf) <= 1e-6):
                            log_action("wait", s=seg["T"])
                    elif seg["type"] == "marker":
                        label = _marker_actions_to_text(seg.get("actions", []))
                        log_action("marker", label=label)
                    elif seg["type"] == "reshape":
                        log_action("reshape", state=seg["state"])
                    elif seg["type"] == "swing":
                        log_action("turn", h0=seg.get("start_heading"), h1=seg.get("target_heading"))
                    last_logged_seg = seg_i
                    if seg.get("edge_events"):
                        seg["_edge_event_idx"] = 0
                    seg.pop("_runtime_state", None)
                
                seg_type = seg.get("type")
                if T <= 0:
                    if seg_type in ("path", "path_follow"):
                        path_points = seg.get("path_points", [])
                        if path_points:
                            robot_pos = path_points[-1]
                            end_h = seg.get("facing")
                            if end_h is None and PATH_FEATURES_AVAILABLE:
                                end_h = calculate_path_heading(path_points, len(path_points) - 1)
                            if end_h is not None:
                                robot_heading = float(end_h) % 360.0
                                if seg.get("reverse") and not seg.get("facing_is_travel", False):
                                    robot_heading = (robot_heading + 180.0) % 360.0
                    elif seg_type == "swing":
                        robot_pos = seg.get("end_pos", robot_pos)
                        robot_heading = seg.get("target_heading", robot_heading)
                    elif seg_type == "turn":
                        robot_pos, robot_heading = seg["pos"], seg["target_heading"]
                    elif seg_type == "wait":
                        robot_pos = seg.get("pos", robot_pos)
                        robot_heading = seg.get("heading", robot_heading)
                    elif seg_type == "move":
                        robot_pos, robot_heading = seg["p1"], seg.get("facing", robot_heading)
                    elif seg_type == "atticus_correction":
                        mode = str(seg.get("mode", "")).strip().lower()
                        if mode == "immediate":
                            _atticus_apply_immediate_correction()
                        elif mode == "wall_trim_start":
                            _atticus_start_wall_trim()
                        elif mode == "wall_trim_end":
                            _atticus_end_wall_trim()
                        elif mode == "rough_wall_start":
                            _atticus_start_rough_wall()
                        elif mode == "rough_wall_end":
                            _atticus_end_rough_wall()
                    elif seg_type == "marker":
                        actions = seg.get("actions", [])
                        reshape_live = _marker_apply_reshape(actions, reshape_live)
                        _marker_apply_atticus(actions)
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
                        motion_progress = frac
                        robot_pos = (seg["p0"][0] + frac*(seg["p1"][0]-seg["p0"][0]),
                                    seg["p0"][1] + frac*(seg["p1"][1]-seg["p0"][1]))
                        robot_heading = seg.get("facing", robot_heading)
                    
                    elif seg_type in ("path", "path_follow"):
                        runtime_state = seg.get("_runtime_state", {})
                        use_pursuit = bool(seg.get("move_to_pose")) or bool(path_lookahead_enabled)
                        robot_pos, robot_heading, runtime_state = _advance_path_segment(
                            seg,
                            robot_pos,
                            robot_heading,
                            CFG,
                            1.0 / 60.0,
                            state=runtime_state,
                            use_pursuit=use_pursuit,
                        )
                        seg["_runtime_state"] = runtime_state
                        motion_progress = runtime_state.get("progress_frac")
                        last_lookahead_point = runtime_state.get("lookahead_point")
                        last_lookahead_radius = float(runtime_state.get("lookahead_radius", 0.0) or 0.0)
                        last_heading_target = runtime_state.get("heading_target")
                    
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
                    
                    if seg_type in ("move", "path", "path_follow") and seg.get("edge_events"):
                        try:
                            reshape_live = _fire_edge_events(seg, progress=motion_progress, reshape_state=reshape_live)
                        except Exception:
                            pass

                    path_state = seg.get("_runtime_state", {}) if seg_type in ("path", "path_follow") else {}
                    path_done = bool(path_state.get("done")) if seg_type in ("path", "path_follow") else False
                    path_elapsed = float(path_state.get("elapsed_s", t_local)) if seg_type in ("path", "path_follow") else t_local
                    path_timeout = (path_elapsed >= max(T, 1.0 / 60.0) + 0.5) if seg_type in ("path", "path_follow") else False
                    seg_complete = (path_done or path_timeout) if seg_type in ("path", "path_follow") else (t_local >= T - 1e-9)
                    if seg_complete:
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
                                    robot_heading = float(end_h) % 360.0
                                    if seg.get("reverse") and not seg.get("facing_is_travel", False):
                                        robot_heading = (robot_heading + 180.0) % 360.0
                                last_heading_target = robot_heading
                                last_lookahead_point = None
                                last_lookahead_radius = 0.0
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
                        if seg_type in ("move", "path", "path_follow") and seg.get("edge_events"):
                            reshape_live = _fire_edge_events(seg, reshape_state=reshape_live, drain_all=True)
                        seg_i += 1
                        t_local = 0.0
        if mcl_enabled:
            mcl_cfg = CFG.get("mcl", {})
            try:
                motion_ms = float(mcl_cfg.get("motion_ms", 20.0))
                sensor_ms = float(mcl_cfg.get("sensor_ms", 20.0))
            except Exception:
                motion_ms = 20.0
                sensor_ms = 20.0
            mcl_pose = (robot_pos[0], robot_pos[1], _mcl_heading_from_internal(robot_heading))
            if mcl_state.lf is None or not mcl_state.map_segments:
                mcl_mod.update_map_segments(mcl_state, CFG)
            mcl_state.motion_accum += dt_ms
            did_motion = False
            motion_input = None
            if mcl_state.motion_accum >= max(1.0, motion_ms):
                motion_input = mcl_mod.simulate_motion_input(
                    mcl_state, mcl_pose
                )
                mcl_state.motion_accum = 0.0
                if motion_input:
                    mcl_mod.motion_update(mcl_state, CFG, motion_input)
                mcl_mod.ekf_predict(mcl_state, CFG, motion_input)
                did_motion = True
            mcl_state.sensor_accum += dt_ms
            if mcl_state.sensor_accum >= max(1.0, sensor_ms):
                if not did_motion:
                    motion_input = mcl_mod.simulate_motion_input(
                        mcl_state, mcl_pose
                    )
                    mcl_state.motion_accum = 0.0
                    if motion_input:
                        mcl_mod.motion_update(mcl_state, CFG, motion_input)
                    mcl_mod.ekf_predict(mcl_state, CFG, motion_input)
                measurements = mcl_mod.simulate_measurements(
                    mcl_state, CFG, mcl_pose, add_noise=True
                )
                mcl_mod.sensor_update(mcl_state, CFG, measurements)
                ekf_cfg = mcl_cfg.get("ekf", {}) if isinstance(mcl_cfg, dict) else {}
                use_imu_update = int(ekf_cfg.get("use_imu_update", 1)) == 1
                if use_imu_update and isinstance(measurements, dict) and "imu" in measurements:
                    mcl_mod.ekf_update_imu(mcl_state, CFG, measurements["imu"])
                if atticus_runtime.get("rough_wall_ctx"):
                    mcl_mod.update_rough_wall_traverse(
                        mcl_state,
                        CFG,
                        measurements,
                        atticus_runtime.get("rough_wall_ctx"),
                        motion_input=motion_input,
                    )
                elif atticus_runtime.get("wall_trim_ctx"):
                    mcl_mod.update_theta_wall_alignment(
                        mcl_state,
                        CFG,
                        measurements,
                        atticus_runtime.get("wall_trim_ctx"),
                        motion_input=motion_input,
                    )
                if mcl_state.estimate is not None:
                    try:
                        conf_val = float(mcl_state.confidence)
                    except Exception:
                        conf_val = 0.0
                    mcl_mod.ekf_update_mcl(mcl_state, CFG, mcl_state.estimate, conf_val)
                mcl_state.sensor_accum = 0.0
        _apply_atticus_ray_highlight(dt_ms)
        insert_preview = None
        if not path_edit_mode and not moving:
            mods_now = pygame.key.get_mods()
            if mods_now & pygame.KMOD_SHIFT:
                insert_preview = _segment_insert_preview(mouse_pos)

        screen.fill(BG_COLOR)
        draw_grid(screen, GRID_SIZE_PX)
        draw_field_objects(screen, CFG)
        draw_geometry_borders(screen, display_nodes, CFG, initial_state["heading"])
        draw_follow_geometry(screen, CFG, robot_pos, robot_heading, reshape_live)
        _draw_mcl_overlay(screen)
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
                if not pose_preview:
                    pygame.draw.line(screen, NODE_COLOR, end_pos, next_eff, 2)
            if pose_preview and not pd.get("use_path", False):
                draw_curved_path(screen, pose_preview, color=(80, 220, 200), width=3)
        
        for i, node in enumerate(display_nodes[:-1]):
            path_data = node.get("path_to_next", {})
            if not (path_data.get("use_path") and path_data.get("control_points")):
                continue
            if path_edit_mode and i == path_edit_segment_idx:
                continue
            color = (100, 200, 255)
            width = 3
            if path_edit_mode:
                color = (120, 120, 120)
                width = 2
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
        
        for i in range(len(display_nodes) - 1):
            node = display_nodes[i]
            path_data = node.get("path_to_next", {})
            if path_data.get("pose_preview_points"):
                continue
            if path_data.get("swing_vis"):
                continue
            if not (path_data.get("use_path") and path_data.get("control_points")):
                p0, p1 = effective_node_pos(i), effective_node_pos(i + 1)
                pygame.draw.line(screen, NODE_COLOR, p0, p1, 2)

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
                tan = _polyline_tangent(pts, t_val)
                hole_r = EDGE_MARKER_RADIUS + 2
                hx0 = (p[0] - tan[0] * hole_r, p[1] - tan[1] * hole_r)
                hx1 = (p[0] + tan[0] * hole_r, p[1] + tan[1] * hole_r)
                pygame.draw.line(screen, BG_COLOR, hx0, hx1, 4)
                radius = EDGE_MARKER_RADIUS + (2 if marker_hover_idx == (i, j) else 0)
                pygame.draw.circle(screen, color, (int(p[0]), int(p[1])), radius, 2)

        for i in range(len(display_nodes) - 1):
            next_node = display_nodes[i + 1]
            off_in = get_node_offset_in(next_node, CFG, i + 1)
            if off_in == 0.0:
                continue
            end_pt = effective_node_pos(i + 1)
            if end_pt is None:
                continue
            pygame.draw.circle(screen, (140, 140, 140), (int(end_pt[0]), int(end_pt[1])), 6)

        draw_nodes(screen, display_nodes, selected_idx, font, CFG, path_edit_mode, draw_links=False)

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
        
        if path_edit_mode and path_control_points:
            cp_preview = list(path_control_points)
            if PATH_FEATURES_AVAILABLE:
                pd_preview = {}
                if path_edit_segment_idx is not None and path_edit_segment_idx < len(display_nodes):
                    pd_preview = display_nodes[path_edit_segment_idx].get("path_to_next", {}) or {}
                spline_type = _path_spline_type(pd_preview)
                smooth_path = generate_bezier_path(cp_preview, num_samples=50, spline_type=spline_type)
                draw_curved_path(screen, smooth_path, color=(255, 200, 100), width=4)
            
            draw_path_control_points(screen, path_control_points, selected_control_point, PATH_CONTROL_RADIUS)
            
            if len(cp_preview) >= 2:
                pygame.draw.lines(screen, (150, 150, 150), False, cp_preview, 1)
            
            draw_path_edit_overlay(screen, path_edit_segment_idx, display_nodes, font_small)
        
        if show_chevron:
            draw_chevron(screen, robot_pos, robot_heading)
        if not mcl_enabled:
            draw_robot(screen, robot_pos)

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
        
        if path_edit_mode:
            pass
        elif (hover_idx is not None and 0 <= hover_idx < len(display_nodes)
              and not dragging and selected_idx is None):
            draw_hover_box(screen, display_nodes[hover_idx], hover_idx, mouse_pos, CFG, initial_state["heading"], font_small)
        
        draw_time_label(screen, display_nodes, total_estimate_s, font_small)
        
        pygame.display.flip()
    
    pygame.quit()

if __name__ == "__main__":
    main()
