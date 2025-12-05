# mod/util.py
import math, copy

# Support running without package context
if __package__ is None or __package__ == "":
    import os, sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT  # type: ignore
    from geom import convert_heading_input  # type: ignore
else:
    from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT
    from .geom import convert_heading_input

def interpret_input_angle(user_deg: float) -> float:
    """
    Convert user-facing heading (0=left, 90=up, 180=right, 270=down) into the
    internal math frame (0=right, 90=up) used for trig.
    """
    d = float(user_deg) % 360.0
    return (180.0 - d) % 360.0

def heading_from_points(p0, p1):
    """
    Calculate internal heading (0=right, 90=up standard math frame) from p0 to p1.
    Convert to display with convert_heading_input when showing to the user.
    """
    dx = p1[0] - p0[0]
    dy = -(p1[1] - p0[1])
    return math.degrees(math.atan2(dy, dx)) % 360.0

def world_offset_px(off_x_in: float, off_y_in: float, heading_deg: float):
    """Convert local offset to world pixel coordinates."""
    th = math.radians(heading_deg)
    rx_in = off_x_in * math.cos(th) - off_y_in * math.sin(th)
    ry_in = off_x_in * math.sin(th) + off_y_in * math.cos(th)
    return (rx_in * PPI, -ry_in * PPI)

def oriented_rect_corners_px(center_px, heading_deg, width_in, length_in, off_x_in=0.0, off_y_in=0.0):
    """Get rectangle corners with rotation and offset."""
    hw, hl = float(width_in) * 0.5, float(length_in) * 0.5
    local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    th = math.radians(heading_deg)
    s, c = math.sin(th), math.cos(th)
    off_dx_px, off_dy_px = world_offset_px(off_x_in, off_y_in, heading_deg)
    
    pts = []
    for lx, ly in local:
        rx_in = lx * c - ly * s
        ry_in = lx * s + ly * c
        pts.append((center_px[0] + off_dx_px + rx_in * PPI,
                    center_px[1] + off_dy_px - ry_in * PPI))
    return pts

def rect_oob(corners_px, pad_px: float, width_px: int, height_px: int):
    """Check if any corner is out of bounds."""
    for (x, y) in corners_px:
        if x < pad_px or x > (width_px - pad_px) or y < pad_px or y > (height_px - pad_px):
            return True
    return False

def approach_unit(prev_pos, curr_pos):
    """Get unit vector from prev to curr."""
    dx = curr_pos[0] - prev_pos[0]
    dy = curr_pos[1] - prev_pos[1]
    L = math.hypot(dx, dy)
    return (0.0, 0.0) if L <= 1e-9 else (dx / L, dy / L)

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
        f"Odometry Origin: {cfg['field_centric']} (0=bot-centric, 1=field-centric)",
        f"Distance Units: {unit_map.get(cfg['distance_units'], 'inches')}",
        f"Angle Units: {ang_map.get(cfg.get('angle_units', 0), 'degrees')}",
        f"Buffer Time = {rp.get('t_buffer', 0) * 1000} ms",
        "Coordinates: x=forward/up, y=left (right-handed)",
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

def polygons_intersect(polyA, polyB, eps=1e-9):
    """SAT collision test between convex polygons."""
    if not polyA or not polyB:
        return False
    
    def edges(poly):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            yield (x2 - x1, y2 - y1)
    
    def axis_normal(dx, dy):
        L = (dx*dx + dy*dy) ** 0.5
        return (0.0, 0.0) if L <= 1e-12 else (-dy / L, dx / L)
    
    def project(poly, ax, ay):
        vals = [x * ax + y * ay for (x, y) in poly]
        return (min(vals), max(vals))
    
    for poly in (polyA, polyB):
        for dx, dy in edges(poly):
            ax, ay = axis_normal(dx, dy)
            if ax == 0.0 and ay == 0.0:
                continue
            minA, maxA = project(polyA, ax, ay)
            minB, maxB = project(polyB, ax, ay)
            if maxA < minB - eps or maxB < minA - eps:
                return False
    return True

def coords_str(p, cfg, initial_pos):
    """Format coordinates as string based on reference frame."""
    if cfg["field_centric"] == 1:
        cx, cy = WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0
    else:
        cx, cy = initial_pos
    
    x_forward_in = -(p[1] - cy) / PPI
    y_left_in = -(p[0] - cx) / PPI
    return f"({x_forward_in:.3f} in, {y_left_in:.3f} in)"

def field_coords_in(p):
    """Convert pixel position to field-centric inches."""
    cx, cy = WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0
    x_forward_in = -(p[1] - cy) / PPI
    y_left_in = -(p[0] - cx) / PPI
    return x_forward_in, y_left_in

def constrain_to_8dirs(origin, target):
    """Project target onto nearest 8-directional axis from origin."""
    ox, oy = origin
    tx, ty = target
    dx, dy = tx - ox, ty - oy
    if dx == 0 and dy == 0:
        return origin
    
    dirs = [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0),
            (0.7071067811865476, 0.7071067811865476),
            (0.7071067811865476, -0.7071067811865476),
            (-0.7071067811865476, 0.7071067811865476),
            (-0.7071067811865476, -0.7071067811865476)]
    
    best_dot = -1e18
    best = (1.0, 0.0, 0.0)
    for ux, uy in dirs:
        dot = dx * ux + dy * uy
        if abs(dot) > best_dot:
            best_dot = abs(dot)
            best = (ux, uy, dot)
    
    ux, uy, dot = best
    return (ox + ux * dot, oy + uy * dot)
