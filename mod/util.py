import math
import copy

# Support running without package context
if __package__ is None or __package__ == "":
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from geom import convert_heading_input  # type: ignore
else:
    from .geom import convert_heading_input


def get_node_offset_in(node, cfg, idx: int) -> float:
    """Get effective offset distance for node."""
    if idx == 0:
        return 0.0
    off = int(node.get("offset", 0))
    if off == 1:
        return float(cfg["offsets"].get("offset_1_in", 0.0) or 0.0)
    if off == 2:
        return float(cfg["offsets"].get("offset_2_in", 0.0) or 0.0)
    if off == 0 and node.get("offset_custom_in") is not None:
        try:
            return float(node["offset_custom_in"])
        except Exception:
            return 0.0
    return 0.0


def snapshot(display_nodes, robot_pos, robot_heading):
    """Create deep copy snapshot for undo."""
    return (copy.deepcopy(display_nodes), robot_pos, robot_heading)


def push_undo_prev(undo_stack, before_state):
    """Push state to undo stack with size limit."""
    undo_stack.append(before_state)
    if len(undo_stack) > 200:
        undo_stack.pop(0)


def pros_convert_inches(cfg, inches: float):
    """Convert inches to selected distance units."""
    wheel_c = math.pi * cfg["robot_physics"]["diameter"]
    gr = cfg.get("gear_ratio", 1.0) or 1.0
    mode = cfg["distance_units"]

    conversions = {
        1: ((inches / wheel_c) * 360.0 * gr, "encoder degrees"),
        2: ((inches / wheel_c) * gr, "encoder rotations"),
        3: ((inches / wheel_c) * 900.0 * gr, "ticks")
    }
    return conversions.get(mode, (inches, "inches"))


def build_compile_header(cfg, initial_heading_deg):
    """Generate compile output header."""
    rp = cfg["robot_physics"]
    unit_map = {0: "inches", 1: "encoder degrees", 2: "encoder rotations", 3: "ticks"}
    ang_map = {0: "degrees", 1: "radians"}

    return [
        "=== ATTICUS TERMINAL COMPILE ===",
        f"Initial heading: {convert_heading_input(initial_heading_deg, None):.3f} deg",
        "Heading: 0=left, 90=up, 180=right, 270=down",
        f"Distance Units: {unit_map.get(cfg['distance_units'], 'inches')}",
        f"Angle Units: {ang_map.get(cfg.get('angle_units', 0), 'degrees')}",
        f"Buffer Time = {rp.get('t_buffer', 0) * 1000} ms",
        "Coordinates: origin = field center (fixed); x=forward/up, y=left (right-handed)",
        ""
    ]


def apply_tbuffer(cfg, timeline_list):
    """Insert buffer wait after each move unless node has custom wait."""
    tb = float(cfg["robot_physics"].get("t_buffer", 0.0) or 0.0)
    if tb <= 1e-6:
        return timeline_list

    out = []
    for seg in timeline_list:
        out.append(seg)
        if seg.get("type") in ("move", "path", "path_follow") and not seg.get("skip_buffer", False):
            out.append({
                "type": "wait",
                "T": tb,
                "pos": seg["p1"],
                "heading": seg.get("facing", seg.get("heading", 0.0)),
                "role": "buffer",
                "post_edge_i": seg.get("i0", None)
            })
    return out
