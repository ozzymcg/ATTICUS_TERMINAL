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
        "auto_oob_fix": {"value": 1},
        "time_ms":       {"value": 0},
        "show_field_objects": {"value": 1},
    },
    "path_config": {
        "lookahead_in": {"value": 15.0},
        "min_speed_ips": {"value": 30.0},
        "max_speed_ips": {"value": 127.0},
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
            "min_timeout_s": 0.0
        },
        "path_dir": {"value": "export/paths"}
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
            "path_config": {k: wrap(float(v)) for k, v in cfg_dict.get("path_config", {}).items()},
            "field_centric":  wrap(int(cfg_dict["field_centric"])),
            "distance_units": wrap(int(cfg_dict["distance_units"])),
            "angle_units":    wrap(int(cfg_dict.get("angle_units", 0))),
            "initial_heading_deg": wrap(float(cfg_dict.get("initial_heading_deg", 0.0))),
            "gear_ratio":     wrap(float(cfg_dict.get("gear_ratio", 1.0))),
            "ui": {k: wrap(int(v)) for k, v in cfg_dict.get("ui", {}).items()},
            "codegen": {
                "style": {"value": cfg_dict.get("codegen", {}).get("style", "Action List")},
                "templates": cfg_dict.get("codegen", {}).get("templates", {}),
                "opts": cfg_dict.get("codegen", {}).get("opts", {}),
                "path_dir": {"value": cfg_dict.get("codegen", {}).get("path_dir", "export/paths")}
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
