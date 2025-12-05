# mod/codegen.py
"""
Code generation module for PROS + LemLib with path file export.
Handles both straight segments and curved path segments.
"""

import os
import math
from typing import List, Tuple, Dict
from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT
from .path_export import export_lemlib_path, generate_path_asset_name, field_coords_in
from .util import pros_convert_inches, interpret_input_angle
from .geom import convert_heading_input


def _timeout_tokens(duration_s: float, pad_factor: float, min_s: float):
    """Calculate timeout tokens for code export."""
    raw_s = max(0.0, float(duration_s))
    pad_s = raw_s * pad_factor
    fin_s = max(min_s, pad_s)
    fin_ms = math.ceil(fin_s * 1000.0)
    return int(fin_ms), round(fin_s, 6)


def build_export_lines_with_paths(cfg, timeline, routine_name="autonomous", initial_heading=None):
    """
    Build code export lines from timeline, including path file generation.
    
    For curved segments (type="path"), generates .path files and emits
    LemLib path-following calls. For straight segments, uses original logic.
    
    Args:
        cfg: Configuration dictionary
        timeline: Compiled timeline from compile_timeline()
        routine_name: Name for path file generation
    
    Returns:
        List of code lines, or None for "Action List" style
    """
    style = str(cfg.get("codegen", {}).get("style", "Action List"))
    if style.strip().lower() in ("action list", "actionlist", "list"):
        return None

    # Normalize routine name to string for asset naming
    try:
        routine_name = str(routine_name)
    except Exception:
        routine_name = "autonomous"
    
        # Template defaults
    defaults = {
        "LemLib": {
            "wait": "pros::delay({MS});",
            "move": "chassis.moveToPoint({X_IN}, {Y_IN}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}}});",
            "turn_global": "chassis.turnToHeading({HEADING_DEG}, {TIMEOUT_MS});",
            "turn_local": "chassis.turnToAngle({HEADING_DEG}, {TIMEOUT_MS});",
            "pose": "chassis.moveToPose({X_IN}, {Y_IN}, {HEADING_DEG}, {TIMEOUT_MS}, {{.forwards = {FORWARDS}}});",
            "path_follow": "chassis.follow({PATH_ASSET}, {LOOKAHEAD}, {TIMEOUT_MS}, {FORWARDS});",
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse handled inline",
            "reverse_off": "// reverse handled inline",
            "tbuffer": "pros::delay({MS});",
            "setpose": "chassis.setPose({X_IN}, {Y_IN}, {HEADING_DEG});"
        },
        "PROS": {
            "wait": "pros::delay({MS});",
            "move": "drive_distance({DIST_IN});",
            "turn_global": "turn_to({HEADING_DEG});",
            "turn_local": "turn_angle({TURN_DELTA_DEG});",
            "pose": "// move_to_pose x={X_IN}, y={Y_IN}, h={HEADING_DEG}",
            "path_follow": 'follow_path("{PATH_FILE}", {LOOKAHEAD});',
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "setpose": "// set pose {X_IN},{Y_IN},{HEADING_DEG}"
        },
        "Custom": {
            "wait": "pros::delay({MS});",
            "move": "move({DIST_IN});",
            "turn_global": "turn_to_heading({HEADING_DEG});",
            "turn_local": "turn_relative({TURN_DELTA_DEG});",
            "pose": "pose({X_IN}, {Y_IN}, {HEADING_DEG});",
            "path_follow": 'follow_path("{PATH_NAME}", {TIMEOUT_MS}, {LOOKAHEAD});',
            "reshape": "// RESHAPE state={STATE}",
            "reverse_on": "// reverse ON",
            "reverse_off": "// reverse OFF",
            "tbuffer": "pros::delay({MS});",
            "setpose": "setpose({X_IN},{Y_IN},{HEADING_DEG});"
        }
    }
    tpls = dict(defaults.get(style, defaults["Custom"]))
    stored_tpl = cfg.get("codegen", {}).get("templates", {}).get(style, {})
    tpls.update(stored_tpl)
    optional_keys = ["reverse_on", "reverse_off", "reshape", "setpose"]
    active_opt = stored_tpl.get("__optional__", None)
    if active_opt is not None:
        active_opt = [opt for opt in active_opt if opt in optional_keys]
        for opt in optional_keys:
            if opt not in active_opt and opt in tpls:
                tpls.pop(opt, None)
    modes = stored_tpl.get("__modes__", {}) if isinstance(stored_tpl, dict) else {}
    
    opts = cfg.get("codegen", {}).get("opts", {})
    ticks_per_rotation = float(opts.get("ticks_per_rotation", 360))
    pad_factor = float(opts.get("pad_factor", 1.0) or 1.0)
    min_s = float(opts.get("min_timeout_s", 0.0) or 0.0)
    motion_mode = modes.get("motion", "move")
    turn_mode = modes.get("turn", "face" if style == "LemLib" else "face")
    
    # Path config
    path_cfg = cfg.get("path_config", {})
    lookahead_in = float(path_cfg.get("lookahead_in", 15.0))
    min_speed_ips = float(path_cfg.get("min_speed_ips", 30.0))
    max_speed_ips = float(path_cfg.get("max_speed_ips", 127.0))
    
    def dist_conversions(p0, p1, reverse=False):
        dist_in = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
        if reverse: 
            dist_in = -dist_in
        wheel_diam = float(cfg["robot_physics"].get("diameter", 2.75))
        rotations = 0.0 if wheel_diam == 0 else dist_in / (math.pi * wheel_diam)
        deg = rotations * 360.0
        ticks = rotations * ticks_per_rotation
        return dist_in, rotations, deg, ticks
    
    lines = [f"/* CODEGEN EXPORT: {style} */", ""]
    asset_declarations = []  # Track ASSET() declarations
    generated_paths = set()  # Track which paths we've generated
    
    # Determine initial pose (first known position)
    pose_pos = None
    for seg in timeline:
        if "p0" in seg:
            pose_pos = seg["p0"]
            break
        if "pos" in seg:
            pose_pos = seg["pos"]
            break
    pose_heading_internal = initial_heading
    if pose_heading_internal is None:
        # Try to infer from first segment heading if available
        for seg in timeline:
            pose_heading_internal = seg.get("start_heading") or seg.get("heading")
            if pose_heading_internal is not None:
                break
    if pose_heading_internal is None:
        # Fallback to config initial heading
        try:
            pose_heading_internal = interpret_input_angle(float(cfg.get("initial_heading_deg", 0.0) or 0.0))
        except Exception:
            pose_heading_internal = float(cfg.get("initial_heading_deg", 0.0) or 0.0)
    pose_heading_disp = convert_heading_input(pose_heading_internal, None)
    if style == "LemLib" and pose_pos is not None and "setpose" in tpls:
        x0_in, y0_in = field_coords_in(pose_pos)
        lines.append(f"chassis.setPose({x0_in:.6f}, {y0_in:.6f}, {pose_heading_disp:.6f});")
        lines.append("")
    
    def _normalize_tpl(val):
        if isinstance(val, (list, tuple)):
            return [str(v) for v in val if str(v).strip()]
        if isinstance(val, str):
            parts = [p.strip() for p in val.split("||")]
            return [p for p in parts if p]
        return []
    
    def emit(template_key, tokens):
        tpl_val = tpls.get(template_key, "")
        for part in _normalize_tpl(tpl_val):
            try:
                line = part.format(**tokens)
            except Exception as e:
                # Attempt to auto-escape lone braces (common with {.forwards = X})
                part_fixed = part.replace("{.forwards", "{{.forwards")
                part_fixed = part_fixed.replace("{FORWARDS}}", "{FORWARDS}}}")
                try:
                    line = part_fixed.format(**tokens)
                except Exception:
                    lines.append(f"// template error in {template_key}: {e}")
                    continue
            if line.strip():
                lines.append(line)
    
    def emit_first(keys, tokens):
        for k in keys:
            if k in tpls:
                emit(k, tokens)
                return
    
    i = 0
    tbuf = float(cfg.get("robot_physics", {}).get("t_buffer", 0.0) or 0.0)
    
    while i < len(timeline):
        seg = timeline[i]
        T = float(seg.get("T", 0.0))
        st = seg.get("type")
        if st == "turn":
            st = "face"
        # Do not inflate buffer waits with pad_factor
        eff_pad = 1.0 if seg.get("role") == "buffer" else pad_factor
        to_ms, to_s = _timeout_tokens(T, eff_pad, min_s)
        tokens_base = {
            "MS": to_ms, "S": to_s, "TIMEOUT_MS": to_ms, "TIMEOUT_S": to_s,
            "STATE": seg.get("state", 0), "NAME": seg.get("name", ""),
            "LOOKAHEAD": int(lookahead_in)
        }
        
        if st == "path":
            # CURVED PATH SEGMENT
            # Generate path file and emit follow call
            segment_idx = seg.get("segment_idx", i)
            path_asset_name = generate_path_asset_name(routine_name, segment_idx)
            path_points = seg.get("path_points", [])
            vels = seg.get("path_speeds_export") or seg.get("path_speeds") or []
            tokens_min = float(seg.get("min_speed_ips", min_speed_ips))
            tokens_max = float(seg.get("max_speed_ips", max_speed_ips))
            
            if path_points and len(path_points) >= 2 and path_asset_name not in generated_paths:
                # Export path file
                try:
                    init_v = vels[0] if vels else min_speed_ips
                    end_v = vels[-1] if vels else min_speed_ips
                    export_lemlib_path(
                        path_points,
                        path_asset_name,
                        cfg,
                        initial_velocity=init_v,
                        end_velocity=end_v,
                        velocities=vels if vels else None,
                        desired_min_speed=tokens_min,
                        desired_max_speed=tokens_max
                    )
                    generated_paths.add(path_asset_name)
                    
                    # Add ASSET declaration
                    asset_var = path_asset_name.replace(".txt", "").replace("-", "_")
                    asset_declarations.append(f'ASSET({asset_var}_txt);')
                except Exception as e:
                    print(f"Warning: Failed to export path {path_asset_name}: {e}")
            
            # Emit path following code
            tokens = dict(tokens_base)
            base_name = path_asset_name.replace(".txt", "")
            asset_var = base_name.replace("-", "_")
            forward_flag = "false" if seg.get("reverse") else "true"
            la_in_seg = lookahead_in
            if seg.get("lookahead_px") is not None:
                try:
                    la_in_seg = float(seg["lookahead_px"]) / PPI
                except Exception:
                    la_in_seg = lookahead_in
            tokens.update({
                "PATH_ASSET": f"{asset_var}_txt",
                "PATH_NAME": asset_var,
                "PATH_FILE": path_asset_name,
                "FORWARDS": forward_flag,
                "FORWARD_PARAM": f"{{.forwards = {forward_flag}}}",
                "PATH_MIN_SPEED": round(float(tokens_min), 3),
                "PATH_MAX_SPEED": round(float(tokens_max), 3),
                "LOOKAHEAD": round(la_in_seg, 6)
            })
            emit_first(("path_follow", "path"), tokens)
            
            i += 1
        elif st == "move":
            # STRAIGHT SEGMENT
            p0, p1 = seg["p0"], seg["p1"]
            x_in, y_in = field_coords_in(p1)
            dist_in, rotations, deg, ticks = dist_conversions(p0, p1, reverse=seg.get("reverse", False))
            facing_heading = seg.get("facing", seg.get("target_heading", 0.0))
            face_disp = convert_heading_input(facing_heading, None)
            tokens = dict(tokens_base)
            tokens.update({
                "X_IN": round(x_in, 6), "Y_IN": round(y_in, 6),
                "DIST_IN": round(dist_in, 6), "DIST_ROT": round(rotations, 6),
                "DIST_DEG": round(deg, 6), "DIST_TICKS": round(ticks, 6),
                "FORWARDS": "false" if seg.get("reverse") else "true",
                "FORWARD_PARAM": "{.forwards = " + ("false" if seg.get("reverse") else "true") + "}",
                "MOVE_SPEED": "" if seg.get("drive_speed_ips") is None else round(float(seg.get("drive_speed_ips")), 6),
                "HEADING_DEG": round(face_disp, 6),
                "HEADING_RAD": round(face_disp * math.pi / 180.0, 9),
            })
            
            emit("pose" if motion_mode == "pose" else "move", tokens)
            i += 1
        
        elif st == "face":
            role = seg.get("role")
            h_disp = convert_heading_input(seg["target_heading"], None)
            delta_internal = ((seg["target_heading"] - seg.get("start_heading", 0.0) + 180.0) % 360.0) - 180.0
            delta_heading = -delta_internal
            tokens = dict(tokens_base)
            tokens.update({
                "HEADING_DEG": round(h_disp, 6),
                "HEADING_RAD": round(h_disp * math.pi / 180.0, 9),
                "TURN_DELTA_DEG": round(delta_heading, 6),
                "TURN_DELTA_RAD": round(delta_heading * math.pi / 180.0, 9),
                "TURN_SPEED": "" if seg.get("turn_speed_dps") is None else round(float(seg.get("turn_speed_dps")), 6)
            })
            chosen_turn = "turn_global"
            if turn_mode == "turn_local":
                chosen_turn = "turn_local"
                # For local, emit delta instead of absolute heading if template expects it
                tokens["HEADING_DEG"] = tokens["TURN_DELTA_DEG"]
                tokens["HEADING_RAD"] = tokens["TURN_DELTA_RAD"]
            emit(chosen_turn, tokens)
            i += 1
        
        elif st == "wait":
            # Skip trailing buffer wait
            if not ((i == len(timeline)-1) and abs(T - tbuf) <= 1e-6 and seg.get("role") == "buffer"):
                temp_key = "tbuffer" if seg.get("role") == "buffer" else "wait"
                emit(temp_key, tokens_base)
            i += 1
        
        elif st == "reshape":
            emit("reshape", tokens_base)
            i += 1
        
        else:
            i += 1
    
    # Add time estimate comment
    total_t = sum(float(sg.get("T", 0.0)) for sg in timeline)
    if timeline and timeline[-1].get("type") == "wait" and timeline[-1].get("role") == "buffer":
        total_t = max(0.0, total_t - float(tbuf))
    
    lines.append("")
    lines.append(f"// Estimated total time: {total_t:.2f} s")
    
    # Prepend asset declarations
    if asset_declarations:
        header = [
            "// Path asset declarations",
            "// Add these near the top of your file:",
            ""
        ] + asset_declarations + ["", "// Autonomous routine:"]
        lines = header + lines
    
    return lines


def export_action_list(cfg, timeline, log_lines):
    """
    Generate action list output (original format) with path indicators.
    
    Args:
        cfg: Configuration dictionary
        timeline: Compiled timeline
        log_lines: List to append log output to
    """
    from .util import pros_convert_inches, coords_str
    from .geom import convert_heading_input
    
    tbuf = float(cfg.get("robot_physics", {}).get("t_buffer", 0.0) or 0.0)
    last_turn = None
    
    for idx, seg in enumerate(timeline):
        T = float(seg.get("T", 0.0))
        if T <= 0.0:
            continue
        
        st = seg.get("type")
        
        if st == "path":
            # Curved path segment
            p0, p1 = seg["p0"], seg["p1"]
            segment_idx = seg.get("segment_idx", idx)
            path_file = generate_path_asset_name("routine", segment_idx)
            
            log_lines.append(f"\n --- Path Segment {segment_idx} ---")
            log_lines.append(f"  Start: {coords_str(p0, cfg, (WINDOW_WIDTH//2, WINDOW_HEIGHT//2))}")
            log_lines.append(f"  End: {coords_str(p1, cfg, (WINDOW_WIDTH//2, WINDOW_HEIGHT//2))}")
            log_lines.append(f"  Path File: {path_file}")
            log_lines.append(f"  Duration: {T:.3f} s")
            
            # Calculate approximate distance
            if "path_points" in seg:
                path_points = seg["path_points"]
                total_dist_px = 0.0
                for i in range(len(path_points) - 1):
                    dx = path_points[i+1][0] - path_points[i][0]
                    dy = path_points[i+1][1] - path_points[i][1]
                    total_dist_px += math.hypot(dx, dy)
                dist_in = total_dist_px / PPI
                conv, unit = pros_convert_inches(cfg, dist_in)
                log_lines.append(f"  Path Length: {conv:.3f} {unit}")
        
        elif st == "move":
            p0, p1 = seg["p0"], seg["p1"]
            log_lines.append(f"\n --- Node {seg.get('i0', '?')} to Node {seg.get('i1', '?')} ---")
            inches = math.hypot(p1[0]-p0[0], p1[1]-p0[1]) / PPI
            if seg.get("reverse", False):
                inches = -inches
            conv, unit = pros_convert_inches(cfg, inches)
            log_lines.append(f"  Distance: {conv:.3f} {unit}")
        
        elif st == "face":
            h0, h1 = seg["start_heading"], seg["target_heading"]
            _key = (round(h0, 6), round(h1, 6), seg.get("role"))
            if _key != last_turn:
                delta_ccw = (h1 - h0 + 360.0) % 360.0
                if delta_ccw > 180.0:
                    delta_ccw -= 360.0
                chosen = float(-delta_ccw)
                dir_tag = "CCW" if chosen >= 0 else "CW"
                to_face_deg = convert_heading_input(h1, None)
                
                if int(cfg.get("angle_units", 0)) == 1:
                    chosen_val = chosen * (math.pi/180.0)
                    to_face_val = to_face_deg * (math.pi/180.0)
                    log_lines.append(f"  Turn: {chosen_val:.6f} rad ({dir_tag} +) | To face: {to_face_val:.6f} rad")
                else:
                    log_lines.append(f"  Turn: {chosen:.3f}° ({dir_tag} +) | To face: {to_face_deg:.3f}°")
                last_turn = _key
        
        elif st == "wait":
            if not (idx == len(timeline)-1 and abs(T - tbuf) <= 1e-6 and seg.get("role") == "buffer"):
                log_lines.append(f"  Wait: {T:.3f} s")
        
        elif st == "reshape":
            log_lines.append(f"  Reshape: state {seg.get('state', 0)}")
    
    # Total time
    total_t = sum(float(sg.get("T", 0.0)) for sg in timeline)
    if timeline and timeline[-1].get("type") == "wait" and timeline[-1].get("role") == "buffer":
        total_t = max(0.0, total_t - float(tbuf))
    log_lines.append(f"\nEstimated total time: {total_t:.2f} s")

def build_export_lines(cfg, timeline, routine_name="autonomous", initial_heading=None):
    """Build code export lines from timeline (path-aware wrapper)."""
    return build_export_lines_with_paths(cfg, timeline, routine_name, initial_heading)
