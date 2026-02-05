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
SELECTED    = TEXT_COLOR
WHITE       = (255, 255, 255)
GREY        = (130, 130, 130)

CONFIG_FILENAME = "config.json"
APP_DIRNAME = "AtticusTerminal"

DEFAULT_MCL = {
    "enabled": {"value": 1},
    "motion_ms": {"value": 20},
    "sensor_ms": {"value": 50},
    "particles": {
        "n": {"value": 200},
        "n_min": {"value": 200},
        "n_max": {"value": 400}
    },
    "motion": {
        "enabled": {"value": 1},
        "motion_model": {"value": "drive"},
        "motion_source": {"value": "encoders"},
        "use_alpha_model": {"value": 0},
        "sigma_x_in": {"value": 0.1275},
        "sigma_y_in": {"value": 0.1275},
        "sigma_theta_deg": {"value": 1.0},
        "alpha1": {"value": 0.05},
        "alpha2": {"value": 0.05},
        "alpha3": {"value": 0.05},
        "alpha4": {"value": 0.05}
    },
    "set_pose_sigma_xy_in": {"value": 0.2},
    "set_pose_sigma_theta_deg": {"value": 2.0},
    "sensors": {
        "distance": {
            "enabled": {"value": 1},
            "model": {"value": "likelihood_field"},
            "sigma_hit_mm": {"value": 8.5},
            "w_hit": {"value": 0.9},
            "w_rand": {"value": 0.1},
            "w_short": {"value": 0.0},
            "w_max": {"value": 0.0},
            "lambda_short": {"value": 0.1},
            "max_range_mm": {"value": 2000.0},
            "min_range_mm": {"value": 0.0},
            "confidence_min": {"value": 0.0},
            "object_size_min": {"value": 0.0},
            "object_size_max": {"value": 0.0},
            "innovation_gate_mm": {"value": 0.0},
            "median_window": {"value": 3},
            "lf_ignore_max": {"value": 0},
            "gate_mm": {"value": 150.0},
            "gate_mode": {"value": "hard"},
            "gate_penalty": {"value": 0.05},
            "likelihood_field": {
                "resolution_in": {"value": 2.0}
            }
        },
        "imu": {
            "enabled": {"value": 1},
            "sigma_deg": {"value": 1.0}
        },
        "vision": {
            "enabled": {"value": 0},
            "sigma_xy_in": {"value": 2.0},
            "sigma_theta_deg": {"value": 5.0},
            "confidence_min": {"value": 0.0},
            "sim_confidence": {"value": 1.0}
        }
    },
    "sensor_geometry": {
        "distance_sensors": []
    },
    "resample": {
        "method": {"value": "systematic"},
        "threshold": {"value": 0.5},
        "always": {"value": 0}
    },
    "kld": {
        "enabled": {"value": 0},
        "epsilon": {"value": 0.05},
        "delta": {"value": 0.99},
        "bin_xy_in": {"value": 2.0},
        "bin_theta_deg": {"value": 10.0}
    },
    "augmented": {
        "enabled": {"value": 0},
        "alpha_slow": {"value": 0.001},
        "alpha_fast": {"value": 0.1}
    },
    "random_injection": {"value": 0.01},
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
    "region": {
        "enabled": {"value": 1},
        "mode": {"value": "hard"},
        "type": {"value": "segment_path"},
        "update_mode": {"value": "segment_path"},
        "penalty": {"value": 0.2},
        "perimeter_gate": {"value": 1},
        "object_gate": {"value": 0},
        "object_mode": {"value": 1},
        "object_clip_free_in": {"value": 0.5},
        "object_clip_max_in": {"value": 2.0},
        "object_clip_sigma_in": {"value": 0.75},
        "grid_type": {"value": "quadrant"},
        "grid_x": {"value": 2},
        "grid_y": {"value": 2},
        "x_min_in": {"value": 0.0},
        "x_max_in": {"value": 144.0},
        "y_min_in": {"value": 0.0},
        "y_max_in": {"value": 144.0},
        "radius_in": {"value": 12.0},
        "slope_enabled": {"value": 0},
        "slope_sigma_deg": {"value": 20.0},
        "sample_attempts": {"value": 50}
    },
    "confidence": {
        "threshold": {"value": 0.0},
        "auto_reinit": {"value": 0},
        "reinit_mode": {"value": "global"}
    },
    "correction": {
        "enabled": {"value": 1},
        "min_confidence": {"value": 0.6},
        "max_trans_jump_in": {"value": 8.0},
        "max_theta_jump_deg": {"value": 15.0},
        "alpha_min": {"value": 0.05},
        "alpha_max": {"value": 0.25}
    },
    "ekf": {
        "enabled": {"value": 1},
        "mcl_min_conf": {"value": 0.6},
        "sigma_dx_in": {"value": 0.1275},
        "sigma_dy_in": {"value": 0.1275},
        "sigma_dtheta_deg": {"value": 1.0},
        "imu_sigma_deg": {"value": 1.0},
        "mcl_sigma_x_min": {"value": 0.2},
        "mcl_sigma_x_max": {"value": 6.0},
        "mcl_sigma_y_min": {"value": 0.2},
        "mcl_sigma_y_max": {"value": 6.0},
        "mcl_sigma_theta_min": {"value": 2.0},
        "mcl_sigma_theta_max": {"value": 15.0},
        "init_sigma_xy_in": {"value": 0.2},
        "init_sigma_theta_deg": {"value": 2.0}
    },
    "ui": {
        "show_particles": {"value": 1},
        "show_estimate": {"value": 1},
        "show_covariance": {"value": 1},
        "show_rays": {"value": 1},
        "show_region": {"value": 1},
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
    },
    "mcl": DEFAULT_MCL
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
    for path in candidates:
        data = _load_json(path)
        if data is not None:
            break
    if data is None:
        data = DEFAULT_CONFIG
        targets = [p for p in candidates if _can_write_file(p)]
        if not targets:
            targets = [os.path.normpath(os.path.join(_user_config_root(), CONFIG_FILENAME))]
        for path in targets[:2]:
            _save_json(path, data)
    return data

def save_config(cfg_dict: dict) -> bool:
    """Save config dictionary to both root and mod directories."""
    try:
        """Handle wrap."""
        def wrap(v): return {"value": v}
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
            },
            "mcl": _wrap_tree(cfg_dict.get("mcl", DEFAULT_MCL)),
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

    mcl_defaults = DEFAULT_CONFIG.get("mcl", {})
    mcl_raw = raw.get("mcl", {})
    mcl = _unwrap_values(_merge_defaults(mcl_defaults, mcl_raw))
    rates = mcl.get("rates", {})
    if isinstance(rates, dict):
        if "motion_ms" in rates:
            mcl["motion_ms"] = rates.get("motion_ms", mcl.get("motion_ms", 20.0))
        if "sensor_ms" in rates:
            mcl["sensor_ms"] = rates.get("sensor_ms", mcl.get("sensor_ms", 50.0))
    motion_noise = mcl.get("motion_noise", {})
    if isinstance(motion_noise, dict):
        motion_cfg = mcl.setdefault("motion", {})
        for key in ("sigma_x_in", "sigma_y_in", "sigma_theta_deg", "alpha1", "alpha2", "alpha3", "alpha4"):
            if key in motion_noise:
                motion_cfg[key] = motion_noise.get(key, motion_cfg.get(key))
    if "resample_method" in mcl or "resample_threshold" in mcl or "resample_always" in mcl:
        resample_cfg = mcl.get("resample", {})
        if not isinstance(resample_cfg, dict):
            resample_cfg = {}
        if "resample_method" in mcl:
            resample_cfg["method"] = mcl.get("resample_method", resample_cfg.get("method", "systematic"))
        if "resample_threshold" in mcl:
            resample_cfg["threshold"] = mcl.get("resample_threshold", resample_cfg.get("threshold", 0.5))
        if "resample_always" in mcl:
            resample_cfg["always"] = mcl.get("resample_always", resample_cfg.get("always", 0))
        mcl["resample"] = resample_cfg
    parts_cfg = mcl.get("particles", {})
    if isinstance(parts_cfg, dict) and "random_injection" in parts_cfg:
        mcl["random_injection"] = parts_cfg.get("random_injection", mcl.get("random_injection", 0.01))

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
        "mcl": mcl,
    }


def auto_lookahead_in(cfg: dict) -> float:
    """Auto-compute base lookahead (inches) from robot size."""
    bd = cfg.get("bot_dimensions", {})
    base = float(bd.get("dt_width", bd.get("width", 12.0)))
    base = max(8.0, min(24.0, base * 0.9))
    return base
