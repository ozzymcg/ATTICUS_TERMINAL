from __future__ import annotations

import math

if __package__ is None or __package__ == "":
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT  # type: ignore
else:
    from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT


def convert_heading_input(heading_deg, _conventional_mode=None):
    """
    Convert internal (standard math frame, 0=right, CCW positive) to display
    convention (0=left, 90=up, 180=right, 270=down).
    """
    return (180.0 - float(heading_deg)) % 360.0


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


def polygons_intersect(polyA, polyB, eps=1e-9):
    """SAT collision test between convex polygons."""
    if not polyA or not polyB:
        return False

    def edges(poly):
        """Handle edges."""
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            yield (x2 - x1, y2 - y1)

    def axis_normal(dx, dy):
        """Handle axis normal."""
        L = (dx * dx + dy * dy) ** 0.5
        return (0.0, 0.0) if L <= 1e-12 else (-dy / L, dx / L)

    def project(poly, ax, ay):
        """Handle project."""
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


def point_in_poly(point, poly):
    """Ray-casting point-in-polygon test (poly as list of (x,y))."""
    x, y = point
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersects = ((yi > y) != (yj > y))
        if intersects:
            x_at_y = (xj - xi) * (y - yi) / max(1e-12, (yj - yi)) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside
