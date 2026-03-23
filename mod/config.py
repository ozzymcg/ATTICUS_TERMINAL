from __future__ import annotations
import json, os, sys
from typing import Optional

FIELD_WIDTH_2FT  = 6
FIELD_HEIGHT_2FT = 6
PIXELS_PER_2FT   = 100
WINDOW_WIDTH     = FIELD_WIDTH_2FT  * PIXELS_PER_2FT
WINDOW_HEIGHT    = FIELD_HEIGHT_2FT * PIXELS_PER_2FT
GRID_SIZE_PX     = PIXELS_PER_2FT
PPI              = PIXELS_PER_2FT / 24.0

BG_COLOR    = (30, 30, 30)
GRID_COLOR  = (50, 50, 50)
NODE_COLOR  = (50, 255, 50)
ROBOT_COLOR = (252, 3, 248)
ARROW_COLOR = (255, 255, 255)
TEXT_COLOR  = (255, 255, 255)
RED         = (255, 0, 0)
ORANGE      = (255, 165, 0)
GOLD        = (255, 215, 0)
DARK_RED    = (255, 245, 140)
SELECTED    = TEXT_COLOR
WHITE       = (255, 255, 255)
GREY        = (130, 130, 130)

CONFIG_FILENAME = "config.json"
APP_DIRNAME = "AtticusTerminal"

DEFAULT_ATTICUS = {
    "config_version": {"value": 8},
    "enabled": {"value": 0},
    "loop_ms": {"value": 20},
    "stall_ms": {"value": 40},
    "motion_ms": {"value": 20},
    "sensor_ms": {"value": 20},
    "motion": {
        "enabled": {"value": 1},
        "motion_model": {"value": "drive"},
        "motion_source": {"value": "encoders"},
        "sigma_x_in": {"value": 0.08},
        "sigma_y_in": {"value": 0.08},
        "sigma_theta_deg": {"value": 0.7},
        "sigma_x_per_in": {"value": 0.02},
        "sigma_y_per_in": {"value": 0.03},
        "sigma_theta_per_deg": {"value": 0.05},
        "no_horizontal_lateral_scale": {"value": 2.5},
        "turn_lateral_scale": {"value": 2.0},
        "rough_wall_sigma_x_scale": {"value": 1.8},
        "rough_wall_sigma_y_scale": {"value": 3.5},
        "rough_wall_sigma_theta_scale": {"value": 1.25},
        "delta_guard_enabled": {"value": 1},
        "max_dx_in_per_tick": {"value": 0.0},
        "max_dy_in_per_tick": {"value": 0.0},
        "max_dtheta_deg_per_tick": {"value": 0.0},
        "guard_vmax_in_s": {"value": 60.0},
        "guard_wmax_deg_s": {"value": 540.0},
        "guard_margin_in": {"value": 0.5},
        "guard_margin_deg": {"value": 8.0},
        "fault_inflate_cycles": {"value": 2},
        "fault_noise_scale": {"value": 2.5}
    },
    "set_pose_sigma_xy_in": {"value": 0.2},
    "set_pose_sigma_theta_deg": {"value": 2.0},
    "sensors": {
        "distance": {
            "enabled": {"value": 1},
            "model": {"value": "likelihood_field"},
            "sigma_hit_mm": {"value": 15.0},
            "sigma_far_scale": {"value": 0.05},
            "sigma_min_mm": {"value": 15.0},
            "sigma_max_mm": {"value": 100.0},
            "min_sensor_weight": {"value": 0.000001},
            "conf_sigma_scale": {"value": 1.0},
            "w_hit": {"value": 0.9},
            "w_rand": {"value": 0.1},
            "w_short": {"value": 0.0},
            "w_max": {"value": 0.0},
            "lambda_short": {"value": 0.1},
            "max_range_mm": {"value": 2000.0},
            "min_range_mm": {"value": 20.0},
            "confidence_min": {"value": 0.0},
            "object_size_min": {"value": 0.0},
            "object_size_max": {"value": 0.0},
            "innovation_gate_mm": {"value": 180.0},
            "median_window": {"value": 3},
            "batch_size": {"value": 3},
            "lf_ignore_max": {"value": 0},
            "use_no_object_info": {"value": 0},
            "fov_multi_ray": {"value": 1},
            "rays_per_sensor": {"value": 3},
            "fov_half_deg_near": {"value": 18.0},
            "fov_half_deg_far": {"value": 12.0},
            "fov_switch_mm": {"value": 203.0},
            "gate_mm": {"value": 180.0},
            "gate_mode": {"value": "hard"},
            "gate_penalty": {"value": 0.05},
            "raycast_bucket_in": {"value": 12.0},
            "likelihood_field": {
                "resolution_in": {"value": 1.0},
                "max_bytes": {"value": 262144}
            }
        },
        "imu": {
            "enabled": {"value": 1},
            "sigma_deg": {"value": 1.0},
            "check_calibrating": {"value": 1},
            "fallback_noise_scale": {"value": 2.0}
        }
    },
    "sensor_geometry": {
        "distance_sensors": []
    },
    "tracking": {
        "horizontal_enabled": {"value": 0}
    },
    "map_objects": {
        "perimeter": {"value": 1},
        "long_goals": {"value": 1},
        "long_goal_braces": {"value": 1},
        "center_goals": {"value": 1},
        "matchloaders": {"value": 1},
        "park_zones": {"value": 0}
    },
    "object_selection": {},
    "sensor_object_visibility": {},
    "interop": {
        "pose_convention": {"value": "cw_zero_forward"},
        "swap_xy": {"value": 0},
        "invert_x": {"value": 0},
        "invert_y": {"value": 0}
    },
    "correction": {
        "enabled": {"value": 1},
        "min_confidence": {"value": 0.6},
        "max_trans_jump_in": {"value": 4.0},
        "max_theta_jump_deg": {"value": 8.0},
        "writeback_alpha": {"value": 0.35},
        "teleport_reset_trans_in": {"value": 18.0},
        "teleport_reset_theta_deg": {"value": 45.0},
        "safe_window_enabled": {"value": 1},
        "safe_max_speed_in_s": {"value": 8.0},
        "safe_max_turn_deg_s": {"value": 60.0}
    },
    "ekf": {
        "enabled": {"value": 1},
        "localizer_min_conf": {"value": 0.6},
        "use_imu_update": {"value": 1},
        "sigma_dx_in": {"value": 0.08},
        "sigma_dy_in": {"value": 0.08},
        "sigma_dtheta_deg": {"value": 0.7},
        "imu_sigma_deg": {"value": 1.0},
        "pose_sigma_x_min": {"value": 0.2},
        "pose_sigma_x_max": {"value": 6.0},
        "pose_sigma_y_min": {"value": 0.2},
        "pose_sigma_y_max": {"value": 6.0},
        "pose_sigma_theta_min": {"value": 2.0},
        "pose_sigma_theta_max": {"value": 15.0},
        "pose_mahalanobis_gate": {"value": 11.34},
        "init_sigma_xy_in": {"value": 0.2},
        "init_sigma_theta_deg": {"value": 2.0}
    },
    "ui": {
        "show_estimate": {"value": 1},
        "show_covariance": {"value": 1},
        "show_rays": {"value": 1},
        "show_gating": {"value": 1}
    }
}

DEFAULT_CONFIG = {
    "reshape_label": {"value": "Reshape"},
    "field_objects": {
        "enabled": {"value": 1},
        "collide": {"value": 1},
        "thickness_in": {"value": 3.5},
        "long_goal_len_in": {"value": 48.8},
        "center_goal_len_in": {"value": 22.6},
        "matchloader_size_in": {"value": 3.5}
    },
    "robot_physics": {
        "weight":         {"value": 12.0},
        "rpm":            {"value": 400.0},
        "diameter":       {"value": 3.25},
        "volts_straight": {"value": 12.0},
        "volts_turn":     {"value": 12.0},
        "mu":             {"value": 0.9},
        "t_buffer":       {"value": 0.0},
        "all_omni":       {"value": 0},
        "tracking_wheels": {"value": 0},
        "advanced_motion": {"value": 0},
        "max_cmd":        {"value": 127.0},
        "point_density_per_in": {"value": 4.0},
        "curvature_gain": {"value": 0.05},
    },
    "offsets": {
        "offset_1_in": {"value": 5.0},
        "offset_2_in": {"value": 15.0},
        "offset_3_in": {"value": 25.0},
        "padding_in":  {"value": 2.5},
    },
    "ui": {
        "show_hitboxes": {"value": 1},
        "show_hitbox_conflicts_only": {"value": 0},
        "auto_oob_fix": {"value": 1},
        "time_ms":       {"value": 0},
        "show_field_objects": {"value": 1},
        "show_node_numbers": {"value": 1},
    },
    "path_config": {
        "lookahead_in": {"value": 15.0},
        "min_speed_cmd": {"value": 30.0},
        "max_speed_cmd": {"value": 127.0},
    },
    "field_centric": {"value": 1},
    "distance_units": {"value": 0},
    "angle_units": {"value": 0},
    "initial_heading_deg": {"value": 0.0},
    "gear_ratio":     {"value": 18.0},
    "bot_dimensions": {
        "width":            {"value": 14.0},
        "length":           {"value": 14.0},
        "dt_width":         {"value": 12.0},
        "dt_length":        {"value": 12.0},
        "full_offset_x_in": {"value": 0.0},
        "full_offset_y_in": {"value": 0.0},
        "reshape": {
            "width":       {"value": 14.0},
            "length":      {"value": 18.0},
            "offset_x_in": {"value": 4.0},
            "offset_y_in": {"value": 0.0},
        },
        "dtposition_offset": {
            "x": {"value": 0.0},
            "y": {"value": 1.5},
        },
        "advanced_geometry": {
            "enabled": 0,
            "symmetry": 0,
            "points": [],
            "reshape_points": [],
        },
    },
    "codegen": {
        "style": {"value": "Action List"},
        "templates": {
            "AttLib": {
                "wait": "pros::delay({MS});",
                "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "turn": "turn({HEADING_DEG});",
                "face": "turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
                "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "mech": "// MECH: {NAME}",
                "tbuffer": "pros::delay({MS});",
                "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
                "__optional__": [
                    "reshape_on",
                    "reshape_off",
                    "setpose",
                    "swing",
                    "path_follow",
                    "marker_wait",
                    "marker_wait_done"
                ],
                "__modes__": {
                    "motion": "move",
                    "turn": "turn_global"
                },
                "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});",
                "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, attlib::DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = attlib::AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
                "pose_angle": "",
                "atticus_immediate": "localizer.applyImmediateCorrectionAuto();",
                "atticus_wall_trim_start": "localizer.correctThetaFromWallAuto();",
                "atticus_wall_trim_end": "localizer.endThetaWallAlignment();",
                "atticus_rough_wall_start": "localizer.startRoughWallTraverseAuto();",
                "atticus_rough_wall_end": "localizer.endRoughWallTraverse();",
                "reshape": "matchloadPistons.set_value({STATE});",
                "reshape_on": "ml_mech.set_value(false);",
                "reshape_off": "ml_mech.set_value(true);",
                "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
                "marker_wait_done": "chassis.waitUntilDone();"
            },
            "Atticus": {
                "wait": "pros::delay({MS});",
                "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "turn": "turn({HEADING_DEG});",
                "face": "turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
                "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "mech": "// MECH: {NAME}",
                "tbuffer": "pros::delay({MS});",
                "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
                "__optional__": [
                    "reshape_on",
                    "reshape_off",
                    "setpose",
                    "swing",
                    "path_follow",
                    "marker_wait",
                    "marker_wait_done"
                ],
                "__modes__": {
                    "motion": "move",
                    "turn": "turn_global"
                },
                "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});",
                "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
                "pose_angle": "",
                "atticus_immediate": "localizer.applyImmediateCorrectionAuto();",
                "atticus_wall_trim_start": "localizer.correctThetaFromWallAuto();",
                "atticus_wall_trim_end": "localizer.endThetaWallAlignment();",
                "atticus_rough_wall_start": "localizer.startRoughWallTraverseAuto();",
                "atticus_rough_wall_end": "localizer.endRoughWallTraverse();",
                "reshape": "matchloadPistons.set_value({STATE});",
                "reshape_on": "ml_mech.set_value(true);",
                "reshape_off": "ml_mech.set_value(false);",
                "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
                "marker_wait_done": "chassis.waitUntilDone();"
            },
            "LemLib": {
                "wait": "pros::delay({MS});",
                "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "turn": "turn({HEADING_DEG});",
                "face": "turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
                "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .lead = {LEAD_IN}, .minSpeed = {DRIVE_MIN_SPEED}, .maxSpeed = {DRIVE_MAX_SPEED}, .earlyExitRange = {DRIVE_EARLY_EXIT}}, {ASYNC});",
                "mech": "// MECH: {NAME}",
                "tbuffer": "pros::delay({MS});",
                "path_follow": "chassis.follow(\"{PATH_NAME}\", {TIMEOUT_MS}, {LOOKAHEAD}, {.forwards = {FORWARDS}}, {ASYNC});",
                "__optional__": [
                    "reshape_on",
                    "reshape_off",
                    "setpose",
                    "swing",
                    "path_follow",
                    "marker_wait",
                    "marker_wait_done"
                ],
                "__modes__": {
                    "motion": "move",
                    "turn": "turn_global"
                },
                "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});",
                "turn_global": "chassis.turnToPoint({TARGET_X_IN}, {TARGET_Y_IN}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "turn_local": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .minSpeed = {TURN_MIN_SPEED}, .earlyExitRange = {TURN_EARLY_EXIT}}, {ASYNC});",
                "swing": "chassis.swingToPoint({TARGET_Y_IN}, {TARGET_X_IN}, DriveSide::{LOCKED_SIDE}, {TIMEOUT_MS}, {.forwards = {FORWARDS}, .direction = AngularDirection::{DIR}, .minSpeed = {SWING_MIN_SPEED}, .earlyExitRange = {SWING_EARLY_EXIT}}, {ASYNC});",
                "pose_angle": "",
                "reshape": "matchloadPistons.set_value({STATE});",
                "reshape_on": "ml_mech.set_value(false);",
                "reshape_off": "ml_mech.set_value(true);",
                "marker_wait": "chassis.waitUntil({MARKER_DIST_IN});",
                "marker_wait_done": "chassis.waitUntilDone();"
            }
        },
        "opts": {
            "ticks_per_rotation": 360,
            "pad_factor": 1.0,
            "min_timeout_s": 0.0,
            "reshape_output": "1/2"
        },
        "path_dir": {"value": "export/paths"},
        "path_columns": {"value": "{X}, {Y}, {COMMAND}"},
        "mech_presets": [
            {"name": "reshape", "mode": "toggle", "template": "", "on": "", "off": "", "default": False},
            {"name": "DSR", "mode": "action", "template": "", "on": "", "off": "", "default": False},
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
        },
        "motion_profiles": {},
        "attlib_sim": {
            "visual_run": 0,
            "module_dir": "",
            "source": "timeline",
            "routine_script": "",
            "routine_function": "run_routine"
        }
    },
    "physics_constants": {
        "load_factor": {"value": 0.9},
        "weight_exp": {"value": 0.06},
        "accel_mu_scale": {"value": 0.95},
        "accel_min": {"value": 60.0},
        "accel_max": {"value": 450.0},
        "t_to_v_base": {"value": 0.2},
        "t_to_v_min": {"value": 0.12},
        "t_to_v_max": {"value": 0.60},
        "vmax_min": {"value": 10.0},
        "vmax_max": {"value": 220.0},
        "turn_rate_scale": {"value": 0.8},
        "turn_rate_min": {"value": 120.0},
        "turn_rate_max": {"value": 900.0},
        "turn_accel_base": {"value": 0.20},
        "turn_accel_min": {"value": 0.12},
        "turn_accel_max": {"value": 0.30},
        "omni_scale": {"value": 0.9}
    },
    "atticus": DEFAULT_ATTICUS
}
DEFAULT_MCL = DEFAULT_ATTICUS
ATTICUS_CONFIG_VERSION = int(DEFAULT_ATTICUS.get("config_version", {}).get("value", 1))
MCL_CONFIG_VERSION = ATTICUS_CONFIG_VERSION
_ATTICUS_FUTURE_VERSION_WARNED = False


def _raw_value(v, default=None):
    """Extract possibly wrapped scalar value."""
    if isinstance(v, dict):
        return v.get("value", default)
    return v if v is not None else default

def _flatten(section: dict) -> dict:
    """Extract 'value' from nested dict structure."""
    flat = {}
    for k, v in section.items():
        flat[k] = v.get("value", v) if isinstance(v, dict) and "value" in v else v
    return flat

def _load_json(path: str) -> Optional[dict]:
    """Load JSON file, return None on failure."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None

def _save_json(path: str, data: dict) -> None:
    """Save JSON file with error handling."""
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
    except Exception:
        pass
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

def _runtime_root() -> str:
    """Handle runtime root."""
    if getattr(sys, "frozen", False):
        try:
            return os.path.dirname(sys.executable)
        except Exception:
            return os.getcwd()
    return os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))

def _user_config_root() -> str:
    """Return per-user writable app config directory."""
    home = os.path.expanduser("~")
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA")
        if not base:
            base = os.path.join(home, "AppData", "Local")
        return os.path.join(base, APP_DIRNAME)
    if sys.platform == "darwin":
        return os.path.join(home, "Library", "Application Support", APP_DIRNAME)
    xdg = os.environ.get("XDG_CONFIG_HOME")
    if not xdg:
        xdg = os.path.join(home, ".config")
    return os.path.join(xdg, APP_DIRNAME.lower())

def _can_write_file(path: str) -> bool:
    """Check if target file path is writable."""
    try:
        parent = os.path.dirname(path) or "."
        os.makedirs(parent, exist_ok=True)
        probe = os.path.join(parent, ".atticus_write_test")
        with open(probe, "w", encoding="utf-8") as f:
            f.write("ok")
        os.remove(probe)
        return True
    except Exception:
        return False

def _config_candidates() -> list:
    """Build ordered config file candidates."""
    here = os.path.dirname(__file__)
    root_candidate = os.path.normpath(os.path.join(here, os.pardir, CONFIG_FILENAME))
    mod_candidate = os.path.normpath(os.path.join(here, CONFIG_FILENAME))
    exe_candidate = os.path.normpath(os.path.join(_runtime_root(), CONFIG_FILENAME))
    user_candidate = os.path.normpath(os.path.join(_user_config_root(), CONFIG_FILENAME))
    env_cfg = os.environ.get("ATTICUS_CONFIG", "").strip()
    ordered = []
    if env_cfg:
        ordered.append(os.path.normpath(env_cfg))
    if getattr(sys, "frozen", False):
        ordered.extend([user_candidate, exe_candidate, root_candidate, mod_candidate])
    else:
        ordered.extend([root_candidate, mod_candidate, user_candidate, exe_candidate])
    out = []
    seen = set()
    for p in ordered:
        if not p:
            continue
        key = os.path.normcase(p)
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out

def load_config() -> dict:
    """Load config from root or mod directory, create default if missing."""
    candidates = _config_candidates()
    data = None
    source_path = None
    for path in candidates:
        data = _load_json(path)
        if data is not None:
            source_path = path
            break
    if data is None:
        data = _merge_defaults(DEFAULT_CONFIG, {})
        if isinstance(data, dict):
            data["atticus"] = _sanitize_atticus_cfg(data.get("atticus", {}))
        targets = [p for p in candidates if _can_write_file(p)]
        if not targets:
            targets = [os.path.normpath(os.path.join(_user_config_root(), CONFIG_FILENAME))]
        for path in targets[:2]:
            _save_json(path, data)
    else:
        if isinstance(data, dict):
            if "atticus" not in data and isinstance(data.get("mcl"), dict):
                data["atticus"] = data.get("mcl", {})
            data.pop("mcl", None)
            data["atticus"] = _sanitize_atticus_cfg(data.get("atticus", {}))
        # Keep one schema source of truth by deep-filling defaults at load time.
        merged = _merge_defaults(DEFAULT_CONFIG, data)
        if isinstance(merged, dict):
            merged["atticus"] = _sanitize_atticus_cfg(merged.get("atticus", {}))
        if merged != data:
            targets = [p for p in candidates if _can_write_file(p)]
            if not targets:
                fallback = source_path or os.path.normpath(os.path.join(_user_config_root(), CONFIG_FILENAME))
                targets = [fallback]
            for path in targets[:2]:
                try:
                    _save_json(path, merged)
                except Exception:
                    pass
        data = merged
    return data

def save_config(cfg_dict: dict) -> bool:
    """Save config dictionary to both root and mod directories."""
    try:
        """Handle wrap."""
        def wrap(v): return {"value": v}
        def _sanitize_poly_points(raw_points):
            """Sanitize advanced geometry points for serialization."""
            if isinstance(raw_points, dict):
                raw_points = raw_points.get("value", [])
            if not isinstance(raw_points, (list, tuple)):
                return []
            out = []
            for pt in raw_points:
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
        def _wrap_tree(obj):
            """Handle wrap tree."""
            if isinstance(obj, dict):
                if set(obj.keys()) == {"value"}:
                    return obj
                return {k: _wrap_tree(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [_wrap_tree(v) if isinstance(v, (dict, list)) else v for v in obj]
            return wrap(obj)
        bd = cfg_dict["bot_dimensions"]
        path_cfg = dict(cfg_dict.get("path_config", {}))
        if "min_speed_cmd" not in path_cfg and "min_speed_ips" in path_cfg:
            path_cfg["min_speed_cmd"] = path_cfg.pop("min_speed_ips")
        if "max_speed_cmd" not in path_cfg and "max_speed_ips" in path_cfg:
            path_cfg["max_speed_cmd"] = path_cfg.pop("max_speed_ips")
        path_cfg.pop("min_speed_ips", None)
        path_cfg.pop("max_speed_ips", None)
        adv_cfg = bd.get("advanced_geometry", {})
        if not isinstance(adv_cfg, dict):
            adv_cfg = {}
        raw = {
            "robot_physics": {k: wrap(float(v)) for k, v in cfg_dict["robot_physics"].items()},
            "bot_dimensions": {
                "width":     wrap(float(bd.get("width", 0.0))),
                "length":    wrap(float(bd.get("length", 0.0))),
                "dt_width":  wrap(float(bd.get("dt_width", 0.0))),
                "dt_length": wrap(float(bd.get("dt_length", 0.0))),
                "full_offset_x_in": wrap(float(bd.get("full_offset_x_in", 0.0))),
                "full_offset_y_in": wrap(float(bd.get("full_offset_y_in", 0.0))),
                "reshape": {
                    "width":       wrap(float(bd.get("reshape_width", 0.0))),
                    "length":      wrap(float(bd.get("reshape_length", 0.0))),
                    "offset_x_in": wrap(float(bd.get("reshape_offset_x_in", 0.0))),
                    "offset_y_in": wrap(float(bd.get("reshape_offset_y_in", 0.0))),
                },
                "advanced_geometry": {
                    "enabled": int(bool(adv_cfg.get("enabled", 0))),
                    "symmetry": int(bool(adv_cfg.get("symmetry", 0))),
                    "points": _sanitize_poly_points(adv_cfg.get("points", [])),
                    "reshape_points": _sanitize_poly_points(adv_cfg.get("reshape_points", [])),
                },
            },
            "offsets": {k: wrap(float(v)) for k, v in cfg_dict["offsets"].items()},
            "path_config": {k: wrap(float(v)) for k, v in path_cfg.items()},
            "field_centric":  wrap(int(cfg_dict["field_centric"])),
            "distance_units": wrap(int(cfg_dict["distance_units"])),
            "angle_units":    wrap(int(cfg_dict.get("angle_units", 0))),
            "initial_heading_deg": wrap(float(cfg_dict.get("initial_heading_deg", 0.0))),
            "gear_ratio":     wrap(float(cfg_dict.get("gear_ratio", 1.0))),
            "ui": {k: wrap(int(v)) for k, v in cfg_dict.get("ui", {}).items()},
            "reshape_label": wrap(str(cfg_dict.get("reshape_label", "Reshape"))),
            "codegen": {
                "style": {"value": cfg_dict.get("codegen", {}).get("style", "Action List")},
                "templates": cfg_dict.get("codegen", {}).get("templates", {}),
                "opts": cfg_dict.get("codegen", {}).get("opts", {}),
                "path_dir": {"value": cfg_dict.get("codegen", {}).get("path_dir", "export/paths")},
                "path_columns": {"value": cfg_dict.get("codegen", {}).get("path_columns", "{X}, {Y}, {COMMAND}")},
                "mech_presets": cfg_dict.get("codegen", {}).get("mech_presets", []),
                "calibration": cfg_dict.get("codegen", {}).get("calibration", {}),
                "motion_profiles": cfg_dict.get("codegen", {}).get("motion_profiles", {}),
                "attlib_sim": cfg_dict.get("codegen", {}).get("attlib_sim", {})
            },
            "physics_constants": {
                k: wrap(float(v))
                for k, v in cfg_dict.get("physics_constants", {}).items()
            },
            "atticus": _wrap_tree(
                _sanitize_atticus_cfg(
                    cfg_dict.get("atticus", cfg_dict.get("mcl", DEFAULT_ATTICUS))
                )
            ),
        }
        
        targets = []
        for path in _config_candidates():
            if _can_write_file(path):
                targets.append(path)
        if not targets:
            fallback = os.path.normpath(os.path.join(_user_config_root(), CONFIG_FILENAME))
            targets = [fallback]
        wrote = 0
        for path in targets:
            try:
                _save_json(path, raw)
                wrote += 1
            except Exception:
                pass
        if wrote <= 0:
            print("Failed to save config: no writable config path")
            return False
        print(f"Config saved to {targets[0]}")
        return True
    except Exception as e:
        print(f"Failed to save config: {e}")
        return False

def physics_flat(cfg: dict) -> dict:
    """Flatten robot_physics section."""
    return _flatten(cfg.get("robot_physics", {}))

def dims_flat(cfg: dict) -> dict:
    """Flatten bot_dimensions section."""
    bd = cfg.get("bot_dimensions", {})
    return {
        **_flatten(bd),
        "reshape": _flatten(bd.get("reshape", {})),
        "dtposition_offset": _flatten(bd.get("dtposition_offset", {}))
    }

def offsets_flat(cfg: dict) -> dict:
    """Flatten offsets section."""
    return _flatten(cfg.get("offsets", {}))


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


def _merge_defaults(defaults, raw):
    """Merge nested defaults with raw values (raw wins when present)."""
    if not isinstance(defaults, dict):
        return raw if raw is not None else defaults
    if not isinstance(raw, dict):
        raw = {}
    merged = {}
    for key, def_val in defaults.items():
        merged[key] = _merge_defaults(def_val, raw.get(key))
    for key, val in raw.items():
        if key not in merged:
            merged[key] = val
    return merged


def _unwrap_values(obj):
    """Recursively unwrap {"value": x} nodes to raw values."""
    if isinstance(obj, dict):
        if set(obj.keys()) == {"value"}:
            return obj.get("value")
        return {k: _unwrap_values(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_unwrap_values(v) for v in obj]
    return obj


def _sanitize_atticus_cfg(raw_atticus):
    """Strip config branches that only existed for the removed particle-filter path."""
    if not isinstance(raw_atticus, dict):
        raw_atticus = {}
    atticus = dict(raw_atticus)
    for key in (
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
        atticus.pop(key, None)
    sensors = atticus.get("sensors", {})
    if isinstance(sensors, dict):
        sensors = dict(sensors)
        sensors.pop("vision", None)
        atticus["sensors"] = sensors
    motion_cfg = atticus.get("motion", {})
    if isinstance(motion_cfg, dict):
        motion_cfg = dict(motion_cfg)
        keep_motion = {
            "enabled": motion_cfg.get("enabled", 1),
            "motion_model": motion_cfg.get("motion_model", "drive"),
            "motion_source": motion_cfg.get("motion_source", "encoders"),
            "sigma_x_in": motion_cfg.get("sigma_x_in", 0.08),
            "sigma_y_in": motion_cfg.get("sigma_y_in", 0.08),
            "sigma_theta_deg": motion_cfg.get("sigma_theta_deg", 0.7),
            "sigma_x_per_in": motion_cfg.get("sigma_x_per_in", 0.02),
            "sigma_y_per_in": motion_cfg.get("sigma_y_per_in", 0.03),
            "sigma_theta_per_deg": motion_cfg.get("sigma_theta_per_deg", 0.05),
            "no_horizontal_lateral_scale": motion_cfg.get("no_horizontal_lateral_scale", 2.5),
            "turn_lateral_scale": motion_cfg.get("turn_lateral_scale", 2.0),
            "rough_wall_sigma_x_scale": motion_cfg.get("rough_wall_sigma_x_scale", 1.8),
            "rough_wall_sigma_y_scale": motion_cfg.get("rough_wall_sigma_y_scale", 3.5),
            "rough_wall_sigma_theta_scale": motion_cfg.get("rough_wall_sigma_theta_scale", 1.25),
            "delta_guard_enabled": motion_cfg.get("delta_guard_enabled", 1),
            "max_dx_in_per_tick": motion_cfg.get("max_dx_in_per_tick", 0.0),
            "max_dy_in_per_tick": motion_cfg.get("max_dy_in_per_tick", 0.0),
            "max_dtheta_deg_per_tick": motion_cfg.get("max_dtheta_deg_per_tick", 0.0),
            "guard_vmax_in_s": motion_cfg.get("guard_vmax_in_s", 60.0),
            "guard_wmax_deg_s": motion_cfg.get("guard_wmax_deg_s", 540.0),
            "guard_margin_in": motion_cfg.get("guard_margin_in", 0.5),
            "guard_margin_deg": motion_cfg.get("guard_margin_deg", 8.0),
            "fault_inflate_cycles": motion_cfg.get("fault_inflate_cycles", 2),
            "fault_noise_scale": motion_cfg.get("fault_noise_scale", 2.5),
        }
        atticus["motion"] = keep_motion
    tracking_cfg = atticus.get("tracking", {})
    if isinstance(tracking_cfg, dict):
        tracking_cfg = dict(tracking_cfg)
    else:
        tracking_cfg = {}
    atticus["tracking"] = {
        "horizontal_enabled": tracking_cfg.get("horizontal_enabled", 0),
    }
    corr_cfg = atticus.get("correction", {})
    if isinstance(corr_cfg, dict):
        corr_cfg = dict(corr_cfg)
    else:
        corr_cfg = {}
    legacy_alpha = corr_cfg.get("alpha_max", corr_cfg.get("alpha_min", 0.35))
    atticus["correction"] = {
        "enabled": corr_cfg.get("enabled", 1),
        "min_confidence": corr_cfg.get("min_confidence", 0.6),
        "max_trans_jump_in": corr_cfg.get("max_trans_jump_in", 4.0),
        "max_theta_jump_deg": corr_cfg.get("max_theta_jump_deg", 8.0),
        "writeback_alpha": corr_cfg.get("writeback_alpha", legacy_alpha),
        "teleport_reset_trans_in": corr_cfg.get("teleport_reset_trans_in", 18.0),
        "teleport_reset_theta_deg": corr_cfg.get("teleport_reset_theta_deg", 45.0),
        "safe_window_enabled": corr_cfg.get("safe_window_enabled", 1),
        "safe_max_speed_in_s": corr_cfg.get("safe_max_speed_in_s", 8.0),
        "safe_max_turn_deg_s": corr_cfg.get("safe_max_turn_deg_s", 60.0),
    }
    ekf_cfg = atticus.get("ekf", {})
    if isinstance(ekf_cfg, dict):
        ekf_cfg = dict(ekf_cfg)
        keep = {
            "enabled": ekf_cfg.get("enabled", 1),
            "localizer_min_conf": ekf_cfg.get("localizer_min_conf", ekf_cfg.get("mcl_min_conf", 0.6)),
            "use_imu_update": ekf_cfg.get("use_imu_update", 1),
            "sigma_dx_in": ekf_cfg.get("sigma_dx_in", 0.08),
            "sigma_dy_in": ekf_cfg.get("sigma_dy_in", 0.08),
            "sigma_dtheta_deg": ekf_cfg.get("sigma_dtheta_deg", 0.7),
            "imu_sigma_deg": ekf_cfg.get("imu_sigma_deg", 1.0),
            "pose_sigma_x_min": ekf_cfg.get("pose_sigma_x_min", ekf_cfg.get("mcl_sigma_x_min", 0.2)),
            "pose_sigma_x_max": ekf_cfg.get("pose_sigma_x_max", ekf_cfg.get("mcl_sigma_x_max", 6.0)),
            "pose_sigma_y_min": ekf_cfg.get("pose_sigma_y_min", ekf_cfg.get("mcl_sigma_y_min", 0.2)),
            "pose_sigma_y_max": ekf_cfg.get("pose_sigma_y_max", ekf_cfg.get("mcl_sigma_y_max", 6.0)),
            "pose_sigma_theta_min": ekf_cfg.get("pose_sigma_theta_min", ekf_cfg.get("mcl_sigma_theta_min", 2.0)),
            "pose_sigma_theta_max": ekf_cfg.get("pose_sigma_theta_max", ekf_cfg.get("mcl_sigma_theta_max", 15.0)),
            "pose_mahalanobis_gate": ekf_cfg.get("pose_mahalanobis_gate", ekf_cfg.get("mcl_mahalanobis_gate", 11.34)),
            "init_sigma_xy_in": ekf_cfg.get("init_sigma_xy_in", 0.2),
            "init_sigma_theta_deg": ekf_cfg.get("init_sigma_theta_deg", 2.0),
        }
        atticus["ekf"] = keep
    ui_cfg = atticus.get("ui", {})
    if isinstance(ui_cfg, dict):
        ui_cfg = dict(ui_cfg)
        atticus["ui"] = {
            "show_estimate": ui_cfg.get("show_estimate", 1),
            "show_covariance": ui_cfg.get("show_covariance", 1),
            "show_rays": ui_cfg.get("show_rays", 1),
            "show_gating": ui_cfg.get("show_gating", 1),
        }
    return atticus


def reload_cfg() -> dict:
    """Reload configuration from disk and normalize nested values."""
    raw = load_config()
    robot_physics = physics_flat(raw)
    dims = dims_flat(raw)
    bd_raw = raw.get("bot_dimensions", {})
    reshape_raw = bd_raw.get("reshape", {})
    dims["full_offset_x_in"] = _num(bd_raw.get("full_offset_x_in", {"value": 0.0}), 0.0)
    dims["full_offset_y_in"] = _num(bd_raw.get("full_offset_y_in", {"value": 0.0}), 0.0)
    dims["reshape_width"] = _num(reshape_raw.get("width", {"value": dims.get("width", 0.0)}), dims.get("width", 0.0))
    dims["reshape_length"] = _num(reshape_raw.get("length", {"value": dims.get("length", 0.0)}), dims.get("length", 0.0))
    dims["reshape_offset_x_in"] = _num(reshape_raw.get("offset_x_in", {"value": 0.0}), 0.0)
    dims["reshape_offset_y_in"] = _num(reshape_raw.get("offset_y_in", {"value": 0.0}), 0.0)
    adv_raw = bd_raw.get("advanced_geometry", {})
    if not isinstance(adv_raw, dict):
        adv_raw = {}
    adv_enabled_raw = adv_raw.get("enabled", 0)
    if isinstance(adv_enabled_raw, dict):
        adv_enabled_raw = adv_enabled_raw.get("value", 0)
    try:
        adv_enabled = int(bool(int(adv_enabled_raw)))
    except Exception:
        adv_enabled = int(bool(adv_enabled_raw))
    adv_sym_raw = adv_raw.get("symmetry", 0)
    if isinstance(adv_sym_raw, dict):
        adv_sym_raw = adv_sym_raw.get("value", 0)
    try:
        adv_symmetry = int(bool(int(adv_sym_raw)))
    except Exception:
        adv_symmetry = int(bool(adv_sym_raw))
    def _adv_points(raw_points):
        """Normalize advanced geometry points."""
        if isinstance(raw_points, dict):
            raw_points = raw_points.get("value", [])
        if not isinstance(raw_points, (list, tuple)):
            return []
        out = []
        for pt in raw_points:
            x = y = None
            if isinstance(pt, dict):
                x = pt.get("x", pt.get("x_in"))
                y = pt.get("y", pt.get("y_in"))
            elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
                x, y = pt[0], pt[1]
            if x is None or y is None:
                continue
            try:
                out.append([float(x), float(y)])
            except Exception:
                continue
        return out
    adv_points = _adv_points(adv_raw.get("points", []))
    adv_reshape_points = _adv_points(adv_raw.get("reshape_points", []))
    legacy_normal = _adv_points(adv_raw.get("normal", []))
    legacy_reshape = _adv_points(adv_raw.get("reshape", []))
    if not adv_points:
        adv_points = legacy_normal if legacy_normal else legacy_reshape
    if not adv_reshape_points:
        adv_reshape_points = legacy_reshape if legacy_reshape else adv_points
    dims["advanced_geometry"] = {
        "enabled": adv_enabled,
        "symmetry": adv_symmetry,
        "points": adv_points,
        "reshape_points": adv_reshape_points,
    }

    ui_raw = raw.get("ui", {})
    ui = {k: v.get("value", v) if isinstance(v, dict) else v for k, v in ui_raw.items()}

    path_raw = raw.get("path_config", {})
    path_cfg = {
        "lookahead_in": _num(path_raw.get("lookahead_in", {"value": 15.0}), 15.0),
        "min_speed_cmd": _num(path_raw.get("min_speed_cmd", path_raw.get("min_speed_ips", {"value": 0.0})), 0.0),
        "max_speed_cmd": _num(path_raw.get("max_speed_cmd", path_raw.get("max_speed_ips", {"value": 127.0})), 127.0),
        "simulate_pursuit": _coerce_int_range(path_raw.get("simulate_pursuit", {"value": 1}), 1, (0, 1)),
    }
    try:
        min_cmd = max(0.0, min(127.0, float(path_cfg.get("min_speed_cmd", 0.0))))
        max_cmd = max(0.0, min(127.0, float(path_cfg.get("max_speed_cmd", 127.0))))
        if min_cmd > max_cmd:
            min_cmd, max_cmd = max_cmd, min_cmd
        path_cfg["min_speed_cmd"] = min_cmd
        path_cfg["max_speed_cmd"] = max_cmd
    except Exception:
        pass

    codegen_raw = raw.get("codegen", {})
    style_raw = codegen_raw.get("style", {"value": "Action List"})
    style_val = style_raw.get("value", "Action List") if isinstance(style_raw, dict) else (style_raw or "Action List")
    path_dir_raw = codegen_raw.get("path_dir", {"value": "export/paths"})
    if isinstance(path_dir_raw, dict):
        path_dir_val = path_dir_raw.get("value", "export/paths")
    else:
        path_dir_val = path_dir_raw or "export/paths"
    path_cols_raw = codegen_raw.get("path_columns", {"value": "{X}, {Y}, {COMMAND}"})
    if isinstance(path_cols_raw, dict):
        path_cols_val = path_cols_raw.get("value", "{X}, {Y}, {COMMAND}")
    else:
        path_cols_val = path_cols_raw or "{X}, {Y}, {COMMAND}"
    codegen = {
        "style": style_val,
        "templates": codegen_raw.get("templates", {}),
        "opts": codegen_raw.get("opts", {}),
        "path_dir": path_dir_val,
        "path_columns": path_cols_val,
        "mech_presets": codegen_raw.get("mech_presets", []),
        "calibration": codegen_raw.get("calibration", {}),
        "motion_profiles": codegen_raw.get("motion_profiles", {}),
        "attlib_sim": codegen_raw.get("attlib_sim", {}),
    }
    phys_raw = raw.get("physics_constants", {})
    phys_defaults = DEFAULT_CONFIG.get("physics_constants", {})
    physics_constants = {}
    for key, default_val in phys_defaults.items():
        default_num = _num(default_val, 0.0)
        physics_constants[key] = _num(phys_raw.get(key, {"value": default_num}), default_num)
    reshape_raw = raw.get("reshape_label", {"value": "Reshape"})
    if isinstance(reshape_raw, dict):
        reshape_label = reshape_raw.get("value", "Reshape")
    else:
        reshape_label = reshape_raw

    atticus_defaults = DEFAULT_CONFIG.get("atticus", {})
    atticus_raw = raw.get("atticus", raw.get("mcl", {}))
    if not isinstance(atticus_raw, dict):
        atticus_raw = {}
    loaded_atticus_version = _raw_value(atticus_raw.get("config_version", 1), 1)
    try:
        loaded_atticus_version = int(loaded_atticus_version)
    except Exception:
        loaded_atticus_version = 1
    global _ATTICUS_FUTURE_VERSION_WARNED
    if loaded_atticus_version > ATTICUS_CONFIG_VERSION and not _ATTICUS_FUTURE_VERSION_WARNED:
        print(
            f"Warning: unsupported atticus.config_version={loaded_atticus_version} "
            f"(supported={ATTICUS_CONFIG_VERSION}); using compatibility fallback."
        )
        _ATTICUS_FUTURE_VERSION_WARNED = True
    mcl = _sanitize_atticus_cfg(_unwrap_values(_merge_defaults(atticus_defaults, atticus_raw)))
    if loaded_atticus_version < 3:
        rates = mcl.get("rates", {})
        if isinstance(rates, dict):
            if "motion_ms" in rates:
                mcl["motion_ms"] = rates.get("motion_ms", mcl.get("motion_ms", 20.0))
            if "sensor_ms" in rates:
                mcl["sensor_ms"] = rates.get("sensor_ms", mcl.get("sensor_ms", 20.0))
        motion_noise = mcl.get("motion_noise", {})
        if isinstance(motion_noise, dict):
            motion_cfg = mcl.setdefault("motion", {})
            for key in ("sigma_x_in", "sigma_y_in", "sigma_theta_deg"):
                if key in motion_noise:
                    motion_cfg[key] = motion_noise.get(key, motion_cfg.get(key))
    if loaded_atticus_version < 8:
        def _legacy_num(value, default):
            """Coerce old scalar values without breaking config reload."""
            try:
                return float(value)
            except Exception:
                return float(default)
        if _legacy_num(mcl.get("loop_ms", 20.0), 20.0) == 10.0:
            mcl["loop_ms"] = 20.0
        if _legacy_num(mcl.get("motion_ms", 20.0), 20.0) == 10.0:
            mcl["motion_ms"] = 20.0
        if _legacy_num(mcl.get("sensor_ms", 20.0), 20.0) == 50.0:
            mcl["sensor_ms"] = 20.0
        motion_legacy = mcl.get("motion", {})
        if isinstance(motion_legacy, dict):
            if _legacy_num(motion_legacy.get("sigma_x_in", 0.08), 0.08) == 0.1275:
                motion_legacy["sigma_x_in"] = 0.08
            if _legacy_num(motion_legacy.get("sigma_y_in", 0.08), 0.08) == 0.1275:
                motion_legacy["sigma_y_in"] = 0.08
            if _legacy_num(motion_legacy.get("sigma_theta_deg", 0.7), 0.7) == 1.0:
                motion_legacy["sigma_theta_deg"] = 0.7
            motion_legacy.setdefault("sigma_x_per_in", 0.02)
            motion_legacy.setdefault("sigma_y_per_in", 0.03)
            motion_legacy.setdefault("sigma_theta_per_deg", 0.05)
            motion_legacy.setdefault("no_horizontal_lateral_scale", 2.5)
            motion_legacy.setdefault("turn_lateral_scale", 2.0)
            motion_legacy.setdefault("rough_wall_sigma_x_scale", 1.8)
            motion_legacy.setdefault("rough_wall_sigma_y_scale", 3.5)
            motion_legacy.setdefault("rough_wall_sigma_theta_scale", 1.25)
            mcl["motion"] = motion_legacy
        sensors_legacy = mcl.get("sensors", {})
        if isinstance(sensors_legacy, dict):
            dist_legacy = sensors_legacy.get("distance", {})
            if isinstance(dist_legacy, dict):
                if _legacy_num(dist_legacy.get("sigma_hit_mm", 15.0), 15.0) in (22.0, 15.0):
                    dist_legacy["sigma_hit_mm"] = 15.0
                if _legacy_num(dist_legacy.get("sigma_min_mm", 15.0), 15.0) in (8.0, 12.0, 15.0):
                    dist_legacy["sigma_min_mm"] = 15.0
                if _legacy_num(dist_legacy.get("sigma_max_mm", 100.0), 100.0) in (120.0, 90.0, 100.0):
                    dist_legacy["sigma_max_mm"] = 100.0
                if _legacy_num(dist_legacy.get("innovation_gate_mm", 180.0), 180.0) == 0.0:
                    dist_legacy["innovation_gate_mm"] = 180.0
                if _legacy_num(dist_legacy.get("gate_mm", 180.0), 180.0) == 150.0:
                    dist_legacy["gate_mm"] = 180.0
                sensors_legacy["distance"] = dist_legacy
            mcl["sensors"] = sensors_legacy
        tracking_cfg = mcl.get("tracking", {})
        if not isinstance(tracking_cfg, dict):
            tracking_cfg = {}
        if "horizontal_enabled" not in tracking_cfg:
            try:
                tracking_cfg["horizontal_enabled"] = 1 if int(robot_physics.get("tracking_wheels", 0)) >= 2 else 0
            except Exception:
                tracking_cfg["horizontal_enabled"] = 0
        mcl["tracking"] = tracking_cfg
        corr_cfg = mcl.get("correction", {})
        if not isinstance(corr_cfg, dict):
            corr_cfg = {}
        if "writeback_alpha" not in corr_cfg:
            alpha_max = corr_cfg.get("alpha_max", 0.25)
            alpha_min = corr_cfg.get("alpha_min", 0.05)
            if _legacy_num(alpha_max, 0.25) == 0.25 and _legacy_num(alpha_min, 0.05) == 0.05:
                corr_cfg["writeback_alpha"] = 0.35
            else:
                corr_cfg["writeback_alpha"] = alpha_max
        if _legacy_num(corr_cfg.get("max_trans_jump_in", 4.0), 4.0) == 8.0:
            corr_cfg["max_trans_jump_in"] = 4.0
        if _legacy_num(corr_cfg.get("max_theta_jump_deg", 8.0), 8.0) == 15.0:
            corr_cfg["max_theta_jump_deg"] = 8.0
        corr_cfg.setdefault("teleport_reset_trans_in", 18.0)
        corr_cfg.setdefault("teleport_reset_theta_deg", 45.0)
        mcl["correction"] = corr_cfg
        geom_cfg = mcl.get("sensor_geometry", {})
        if not isinstance(geom_cfg, dict):
            geom_cfg = {}
        sensors = geom_cfg.get("distance_sensors", [])
        if isinstance(sensors, list):
            migrated = []
            for sensor in sensors:
                if not isinstance(sensor, dict):
                    continue
                entry = dict(sensor)
                try:
                    legacy_x = float(entry.get("x_in", 0.0))
                except Exception:
                    legacy_x = 0.0
                try:
                    legacy_y = float(entry.get("y_in", 0.0))
                except Exception:
                    legacy_y = 0.0
                entry["x_in"] = -legacy_y
                entry["y_in"] = legacy_x
                migrated.append(entry)
            geom_cfg["distance_sensors"] = migrated
            mcl["sensor_geometry"] = geom_cfg
    motion_cfg = mcl.get("motion", {})
    if not isinstance(motion_cfg, dict):
        motion_cfg = {}
    # Runtime currently supports one deterministic odom model path.
    # Normalize legacy/unused selectors to prevent no-op ambiguity.
    motion_cfg["motion_model"] = "drive"
    motion_cfg["motion_source"] = "encoders"
    mcl["motion"] = motion_cfg
    sensors_cfg = mcl.get("sensors", {})
    if not isinstance(sensors_cfg, dict):
        sensors_cfg = {}
    dist_cfg = sensors_cfg.get("distance", {})
    if not isinstance(dist_cfg, dict):
        dist_cfg = {}
    try:
        sigma_hit = float(dist_cfg.get("sigma_hit_mm", 15.0))
        sigma_min = float(dist_cfg.get("sigma_min_mm", 15.0))
        sigma_max = float(dist_cfg.get("sigma_max_mm", 100.0))
        sigma_far = float(dist_cfg.get("sigma_far_scale", 0.05))
    except Exception:
        sigma_hit = sigma_min = sigma_max = sigma_far = None
    if sigma_hit == 22.0 and sigma_min == 12.0 and sigma_max == 90.0 and sigma_far == 0.05:
        dist_cfg["sigma_hit_mm"] = 15.0
        dist_cfg["sigma_min_mm"] = 15.0
        dist_cfg["sigma_max_mm"] = 100.0
    # The terminal only ships the Atticus likelihood-field path now.
    dist_cfg["model"] = "likelihood_field"
    sensors_cfg["distance"] = dist_cfg
    mcl["sensors"] = sensors_cfg
    mcl["config_version"] = ATTICUS_CONFIG_VERSION

    return {
        "robot_physics": robot_physics,
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
        "physics_constants": physics_constants,
        "reshape_label": str(reshape_label) if reshape_label is not None else "Reshape",
        "atticus": mcl,
        "mcl": mcl,
    }


def auto_lookahead_in(cfg: dict) -> float:
    """Auto-compute base lookahead (inches) from robot size."""
    bd = cfg.get("bot_dimensions", {})
    base = float(bd.get("dt_width", bd.get("width", 12.0)))
    base = max(8.0, min(24.0, base * 0.9))
    return base
