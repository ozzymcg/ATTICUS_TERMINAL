# mod/config.py
from __future__ import annotations
import json, os
from typing import Optional

# Window and field
FIELD_WIDTH_2FT  = 6
FIELD_HEIGHT_2FT = 6
PIXELS_PER_2FT   = 100
WINDOW_WIDTH     = FIELD_WIDTH_2FT  * PIXELS_PER_2FT
WINDOW_HEIGHT    = FIELD_HEIGHT_2FT * PIXELS_PER_2FT
GRID_SIZE_PX     = PIXELS_PER_2FT
PPI              = PIXELS_PER_2FT / 24.0

# Colors (RGB)
BG_COLOR    = (30, 30, 30)
GRID_COLOR  = (50, 50, 50)
NODE_COLOR  = (50, 255, 50)
ROBOT_COLOR = (252, 3, 248)
ARROW_COLOR = (255, 255, 255)
TEXT_COLOR  = (255, 255, 255)
RED         = (255, 0, 0)
ORANGE      = (255, 165, 0)
GOLD        = (255, 215, 0)
SELECTED    = TEXT_COLOR
WHITE       = (255, 255, 255)
GREY        = (130, 130, 130)

CONFIG_FILENAME = "config.json"

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
    # plane_mode removed; fixed heading convention
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
        }
    },
    "codegen": {
        "style": {"value": "Action List"},
        "templates": {},
        "opts": {
            "ticks_per_rotation": 360,
            "pad_factor": 1.0,
            "min_timeout_s": 0.0,
            "reshape_output": "1/2"
        },
        "path_dir": {"value": "export/paths"},
        "path_columns": {"value": "{X}, {Y}, {COMMAND}"},
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
        },
        "motion_profiles": {}
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
    }
}

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

def load_config() -> dict:
    """Load config from root or mod directory, create default if missing."""
    here = os.path.dirname(__file__)
    root_candidate = os.path.normpath(os.path.join(here, os.pardir, "config.json"))
    mod_candidate  = os.path.normpath(os.path.join(here, "config.json"))
    data = _load_json(root_candidate) or _load_json(mod_candidate)
    if data is None:
        data = DEFAULT_CONFIG
        _save_json(root_candidate, data)
        _save_json(mod_candidate, data)
    return data

def save_config(cfg_dict: dict) -> bool:
    """Save config dictionary to both root and mod directories."""
    try:
        def wrap(v): return {"value": v}
        bd = cfg_dict["bot_dimensions"]
        path_cfg = dict(cfg_dict.get("path_config", {}))
        if "min_speed_cmd" not in path_cfg and "min_speed_ips" in path_cfg:
            path_cfg["min_speed_cmd"] = path_cfg.pop("min_speed_ips")
        if "max_speed_cmd" not in path_cfg and "max_speed_ips" in path_cfg:
            path_cfg["max_speed_cmd"] = path_cfg.pop("max_speed_ips")
        path_cfg.pop("min_speed_ips", None)
        path_cfg.pop("max_speed_ips", None)
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
                "motion_profiles": cfg_dict.get("codegen", {}).get("motion_profiles", {})
            },
            "physics_constants": {
                k: wrap(float(v))
                for k, v in cfg_dict.get("physics_constants", {}).items()
            }
        }
        
        here = os.path.dirname(__file__)
        root_cfg = os.path.normpath(os.path.join(here, os.pardir, "config.json"))
        mod_cfg = os.path.normpath(os.path.join(here, "config.json"))
        _save_json(root_cfg, raw)
        _save_json(mod_cfg, raw)
        print(f"Config saved to {root_cfg}")
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


def reload_cfg() -> dict:
    """Reload configuration from disk and normalize nested values."""
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
        "motion_profiles": codegen_raw.get("motion_profiles", {})
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
        "physics_constants": physics_constants,
        "reshape_label": str(reshape_label) if reshape_label is not None else "Reshape",
    }


def auto_lookahead_in(cfg: dict) -> float:
    """Auto-compute base lookahead (inches) from robot size."""
    bd = cfg.get("bot_dimensions", {})
    base = float(bd.get("dt_width", bd.get("width", 12.0)))
    # Slightly tighter, more pragmatic lookahead for pure pursuit/path following
    base = max(8.0, min(24.0, base * 0.9))
    return base
