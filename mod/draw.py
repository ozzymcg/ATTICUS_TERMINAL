# mod/draw.py

from __future__ import annotations

import math, pygame

try:

    from pygame import gfxdraw

except Exception:

    gfxdraw = None



# Allow running as a script by falling back to absolute imports

if __package__ is None or __package__ == "":

    import os, sys

    here = os.path.dirname(os.path.abspath(__file__))

    sys.path.append(here)

    sys.path.append(os.path.dirname(here))

    from config import (

        GRID_COLOR, NODE_COLOR, ROBOT_COLOR, ARROW_COLOR, TEXT_COLOR,

        ORANGE, GOLD, RED, GREY, WHITE, WINDOW_WIDTH, WINDOW_HEIGHT, PPI, GRID_SIZE_PX

    )

    from util import (

        oriented_rect_corners_px, rect_oob, get_node_offset_in, 

        approach_unit, polygons_intersect, interpret_input_angle, heading_from_points

    )

    from geom import convert_heading_input

    from path_utils import generate_bezier_path, calculate_path_heading

else:

    from .config import (

        GRID_COLOR, NODE_COLOR, ROBOT_COLOR, ARROW_COLOR, TEXT_COLOR,

        ORANGE, GOLD, RED, GREY, WHITE, WINDOW_WIDTH, WINDOW_HEIGHT, PPI, GRID_SIZE_PX

    )

    from .util import (

        oriented_rect_corners_px, rect_oob, get_node_offset_in, 

        approach_unit, polygons_intersect, interpret_input_angle, heading_from_points

    )

    from .geom import convert_heading_input

    from .path_utils import generate_bezier_path, calculate_path_heading

def _reshape_label(cfg, lower=False):
    try:
        lbl = str(cfg.get("reshape_label", "Reshape"))
    except Exception:
        lbl = "Reshape"
    return lbl.lower() if lower else lbl

PATH_COLOR = (100, 180, 255)  # Light blue for paths
PATH_CONTROL_COLOR = (255, 200, 50)  # Gold for control points
PATH_EDIT_HIGHLIGHT = (255, 255, 100)  # Yellow highlight


def _aa_polygon(surface, color, pts, width=0):

    """Anti-aliased polygon with fallback."""

    if gfxdraw is not None:

        try:

            gfxdraw.aapolygon(surface, pts, color)

            if width == 0:

                gfxdraw.filled_polygon(surface, pts, color)

            else:

                pygame.draw.polygon(surface, color, pts, width)

            return

        except Exception:

            pass

    pygame.draw.polygon(surface, color, pts, width)



def draw_grid(surface, grid_size_px):

    """Draw field grid lines."""

    w, h = surface.get_width(), surface.get_height()

    for x in range(0, w, int(grid_size_px)):

        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, h))

    for y in range(0, h, int(grid_size_px)):

        pygame.draw.line(surface, GRID_COLOR, (0, y), (w, y))



def draw_robot(surface, pos):

    """Draw robot center dot."""

    pygame.draw.circle(surface, ROBOT_COLOR, pos, 8)



def draw_chevron(surface, pos, heading_deg, length=20, offset=45, arm=10):

    """Draw directional arrow chevron."""

    rad = math.radians(heading_deg)

    tip = (pos[0] + length * math.cos(rad), pos[1] - length * math.sin(rad))

    l = math.radians(heading_deg - offset)

    r = math.radians(heading_deg + offset)

    left = (tip[0] - arm * math.cos(l), tip[1] + arm * math.sin(l))

    right = (tip[0] - arm * math.cos(r), tip[1] + arm * math.sin(r))

    pygame.draw.line(surface, ARROW_COLOR, tip, left, 3)

    pygame.draw.line(surface, ARROW_COLOR, tip, right, 3)



def draw_multicolor_circle(surface, pos, radius, colors):

    """Draw circle with pie-slice colors."""

    if not colors:

        return

    if len(colors) == 1:

        pygame.draw.circle(surface, colors[0], pos, radius)

        return

    cx, cy = pos

    step = 360 / len(colors)

    npts = 24

    for i, color in enumerate(colors):

        a0 = math.radians(i * step)

        a1 = math.radians((i + 1) * step)

        pts = [(cx, cy)]

        for j in range(npts + 1):

            t = a0 + (a1 - a0) * (j / npts)

            pts.append((int(cx + radius * math.cos(t)), int(cy - radius * math.sin(t))))

        _aa_polygon(surface, color, pts, 0)



def draw_nodes(surface, nodes, selected_idx, font, cfg=None, path_edit_mode=False):

    """Draw path nodes with colors and labels."""

    # Draw connections (straight lines or path indicators)

    for i in range(len(nodes) - 1):

        p0, p1 = nodes[i]["pos"], nodes[i+1]["pos"]

        

        # Check if this segment uses a curved path

        path_data = nodes[i].get("path_to_next", {})

        if path_data.get("use_path", False):

            pygame.draw.line(surface, PATH_COLOR, p0, p1, 1)

        else:

            pygame.draw.line(surface, NODE_COLOR, p0, p1, 2)

    

    # Draw node circles

    for i, node in enumerate(nodes):

        colors = []

        if node.get("reverse", False): 

            colors.append(RED)

        off = node.get("offset", 0)

        if i != 0:

            if off == 1: colors.append(ORANGE)

            if off == 2: colors.append(GOLD)

        if node.get("reshape_toggle", False): 

            colors.append((0, 100, 0))

        

        # Add path indicator color if segment has path

        if i < len(nodes) - 1:

            path_data = node.get("path_to_next", {})

            if path_data.get("use_path", False):

                colors.append(PATH_COLOR)

        

        colors = colors or [NODE_COLOR]

        

        draw_multicolor_circle(surface, node["pos"], 6, colors)

        if i == selected_idx: 

            pygame.draw.circle(surface, TEXT_COLOR, node["pos"], 8, 3)

        label = font.render(str(i), True, TEXT_COLOR)

        surface.blit(label, label.get_rect(center=(node["pos"][0], node["pos"][1] - 15)))



        # Draw offset ghost circle/point for nodes with offset ghost angle

        # Draw offset ghost only in path edit mode

        try:

            if cfg is None or i == 0:

                continue

            off_type = int(node.get("offset", 0))

            if off_type == 0 and node.get("offset_custom_in") is None:

                continue

            prev_pd = nodes[i - 1].get("path_to_next", {}) if i > 0 else {}

            # Only render offset ghosts when editing a path segment
            if not (path_edit_mode and prev_pd.get("use_path", False)):
                continue

            off_in = get_node_offset_in(node, cfg, i)

            ghost_ang = node.get("offset_ghost_angle")

            if off_in and ghost_ang is None:

                prev_eff = nodes[i - 1]["pos"]

                ghost_ang = heading_from_points(prev_eff, node["pos"])

                node["offset_ghost_angle"] = ghost_ang

            if off_in and ghost_ang is not None:

                radius_px = float(off_in) * PPI

                rad = math.radians(ghost_ang)

                gx = node["pos"][0] + math.cos(rad) * radius_px

                gy = node["pos"][1] - math.sin(rad) * radius_px

                pygame.draw.circle(surface, GREY, (int(node["pos"][0]), int(node["pos"][1])), int(radius_px), 1)

                pygame.draw.circle(surface, (160,160,160), (int(gx), int(gy)), 6)

                pygame.draw.circle(surface, (120,120,120), (int(gx), int(gy)), 10, 1)

        except Exception:

            pass



def draw_path_curves(surface, nodes):

    """Draw smooth curves for all path segments."""

    for i in range(len(nodes) - 1):

        node = nodes[i]

        next_node = nodes[i + 1]

        path_data = node.get("path_to_next", {})

        

        if path_data.get("use_path", False):

            control_points = path_data.get("control_points", [])

            if len(control_points) >= 2:

                # Anchor endpoints to swing end/start overrides when present
                cps = list(control_points)
                # Anchor to node position (start of segment) so arc/path begins at the segment start
                cps[0] = node["pos"]
                cps[-1] = next_node["pos"]

                # Generate smooth curve
                curve_points = generate_bezier_path(cps, num_samples=50)

                # Draw the curve
                if len(curve_points) > 1:
                    pygame.draw.lines(surface, PATH_COLOR, False, curve_points, 3)



def draw_path_edit_mode(surface, nodes, segment_idx, dragging_idx, control_radius):

    """Draw path editing interface for the selected segment."""

    if segment_idx >= len(nodes) - 1:

        return

    

    node = nodes[segment_idx]

    next_node = nodes[segment_idx + 1]

    path_data = node.get("path_to_next", {})

    control_points = path_data.get("control_points", [])

    

    if not control_points:

        return

    

    # Draw curve with highlight

    curve_points = generate_bezier_path(control_points, num_samples=50)

    if len(curve_points) > 1:

        pygame.draw.lines(surface, PATH_EDIT_HIGHLIGHT, False, curve_points, 4)

    

    # Draw control points (middle points only - endpoints are nodes)

    for i, cp in enumerate(control_points):

        if 1 <= i < len(control_points) - 1:  # Skip first and last

            color = PATH_EDIT_HIGHLIGHT if i == dragging_idx else PATH_CONTROL_COLOR

            pygame.draw.circle(surface, color, (int(cp[0]), int(cp[1])), control_radius)

            pygame.draw.circle(surface, TEXT_COLOR, (int(cp[0]), int(cp[1])), control_radius, 2)

            

            # Draw lines connecting to path

            if i > 0:

                pygame.draw.line(surface, GREY, control_points[i-1], cp, 1)

            if i < len(control_points) - 1:

                pygame.draw.line(surface, GREY, cp, control_points[i+1], 1)



def draw_label(surface, anchor_xy, lines, font_small):

    """Draw text label box at position."""

    pad = 6

    text_surfs = [font_small.render(s, True, (220, 220, 220)) for s in lines]

    w = max(t.get_width() for t in text_surfs) + pad * 2

    h = sum(t.get_height() for t in text_surfs) + pad * 2

    x = min(max(0, anchor_xy[0] + 12), WINDOW_WIDTH - w)

    y = min(max(0, anchor_xy[1] + 12), WINDOW_HEIGHT - h)

    pygame.draw.rect(surface, (0, 0, 0), (x, y, w, h))

    cy = y + pad

    for t in text_surfs:

        surface.blit(t, (x + pad, cy))

        cy += t.get_height()




def draw_hover_box(surface, node, idx, mouse_pos, cfg, initial_heading, font_small):
    """Draw hover info for node."""
    lines = []
    adv_enabled = bool(cfg.get("robot_physics", {}).get("advanced_motion", 0))
    profile_info = node.get("profile_info") if adv_enabled else None
    if idx == 0:
        disp = convert_heading_input(initial_heading, None)
        lines.append(f"Initial heading {disp:.3f}°")
    else:
        off = int(node.get("offset", 0))
        if off == 1:
            lines.append(f"Offset1 {cfg['offsets']['offset_1_in']:g} in")
        elif off == 2:
            lines.append(f"Offset2 {cfg['offsets']['offset_2_in']:g} in")
        if off == 0 and node.get("offset_custom_in") is not None:
            lines.append(f"Custom offset {node['offset_custom_in']:g} in")

    for act in node.get("actions", []):
        t = act.get("type")
        if t == "turn":
            disp = convert_heading_input(act.get("deg", 0.0), None)
            lines.append(f"Turn to {disp:g}°")
        elif t == "wait":
            lines.append(f"Wait {act.get('s', 0):g}s")
        elif t in ("reshape", "geom"):
            lines.append(_reshape_label(cfg))

    path_data = node.get("path_to_next", {})
    if path_data.get("use_path", False):
        lines.append("Path: Curved")
        try:
            vmin = float(path_data.get("min_speed_ips", cfg.get("path_config", {}).get("min_speed_ips", 0.0)))
            vmax = float(path_data.get("max_speed_ips", cfg.get("path_config", {}).get("max_speed_ips", 127.0)))
            lines.append(f"Path speed {vmin:.1f}-{vmax:.1f} cmd")
        except Exception:
            pass
        if path_data.get("move_to_pose"):
            ph = path_data.get("pose_heading")
            pl = path_data.get("pose_lead_in")
            if ph is not None:
                suffix = f", lead={pl:g}\"" if pl not in (None, "") else ""
                lines.append(f"moveToPose h={convert_heading_input(ph, None):.1f}°{suffix}")
            else:
                lines.append("moveToPose (path)")
    else:
        if node.get("move_to_pose"):
            ph = node.get("pose_heading_deg")
            pl = node.get("pose_lead_in")
            if ph is not None:
                suffix = f", lead={pl:g}\"" if pl not in (None, "") else ""
                lines.append(f"moveToPose h={convert_heading_input(ph, None):.1f}°{suffix}")
            else:
                lines.append("moveToPose")
        else:
            lines.append("Straight")

    if adv_enabled and profile_info:
        prof_parts = []
        drv_prof = profile_info.get("drive_profile")
        if drv_prof:
            prof_parts.append(f"drive={drv_prof}")
        tm_mode = node.get("turn_mode", "turn")
        if tm_mode == "swing":
            sw_prof = profile_info.get("swing_profile")
            if sw_prof:
                prof_parts.append(f"swing={sw_prof}")
        else:
            turn_prof = profile_info.get("turn_profile")
            if turn_prof:
                prof_parts.append(f"turn={turn_prof}")
        if prof_parts:
            lines.append("Profile: " + ", ".join(prof_parts))

    tm = node.get("turn_mode", "turn")
    sd = node.get("swing_dir", "auto")
    if tm == "swing":
        lines.append(f"Heading change: swing ({sd})")
    else:
        lines.append("Heading change: turn in place")

    if node.get("reshape_toggle", False):
        lines.append(_reshape_label(cfg))
    if node.get("reverse", False):
        lines.append("Reverse")

    if lines:
        draw_label(surface, mouse_pos, lines, font_small)

def draw_time_label(surface, nodes, seconds, font_small):
    """Draw time estimate at last node."""
    if not nodes: 
        return
    pos = nodes[-1]["pos"]
    draw_label(surface, (pos[0] + 6, pos[1] - 12 - 30), [f"{seconds:.2f}s"], font_small)


def draw_constraint_visual(surface, dragging, selected_idx, nodes, constrain_origin):

    """Draw Shift-drag constraint guide lines."""

    if not (dragging and selected_idx is not None and constrain_origin is not None):

        return

    try:

        anchor_pt = constrain_origin

        curp = tuple(nodes[selected_idx]['pos'])

        ax, ay = int(anchor_pt[0]), int(anchor_pt[1])

        cx, cy = int(curp[0]), int(curp[1])

        mid = (cx, ay)

        g = (190, 190, 190)

        pygame.draw.line(surface, g, (ax, ay), mid, 1)

        pygame.draw.line(surface, g, mid, (cx, cy), 1)

        pygame.draw.circle(surface, g, (ax, ay), 5, 1)

    except Exception:

        pass



def _tile_center_px(tx, ty):

    """Convert tile coordinates to pixel center."""

    return (tx * GRID_SIZE_PX, WINDOW_HEIGHT - ty * GRID_SIZE_PX)



def _oriented_rect_poly(center_px, angle_deg, width_in, length_in):

    """Create rectangle polygon for field objects."""

    return oriented_rect_corners_px(center_px, angle_deg, width_in, length_in, 0.0, 0.0)



def get_field_object_polys(cfg):

    """Return list of (name, polygon) for field objects."""

    polys = []

    THICK, LONG_LEN, CENTER_LEN = 3.5, 48.8, 22.6
    ML_SIZE, PARK_W, PARK_D = 5.0, 18.87, 16.86

    

    # Long goals at x=1 and x=5

    for x in (1.0, 5.0):

        c = _tile_center_px(x, 3.0)

        polys.append(("long_goal", _oriented_rect_poly(c, 90.0, THICK, LONG_LEN)))

    

    # Center diagonal goals

    c = _tile_center_px(3.0, 3.0)

    polys.append(("center_goal", _oriented_rect_poly(c, 45.0, THICK, CENTER_LEN)))

    polys.append(("center_goal", _oriented_rect_poly(c, -45.0, THICK, CENTER_LEN)))

    

    # Matchloaders

    inset_in = ML_SIZE * 0.5

    for x in (1.0, 5.0):

        cb = _tile_center_px(x, 0.0)

        cb = (cb[0], cb[1] - inset_in * PPI)

        polys.append(("matchloader", _oriented_rect_poly(cb, 90.0, ML_SIZE, ML_SIZE)))

        ct = _tile_center_px(x, 6.0)

        ct = (ct[0], ct[1] + inset_in * PPI)

        polys.append(("matchloader", _oriented_rect_poly(ct, 90.0, ML_SIZE, ML_SIZE)))

    

    # Park zones

    w_px, d_px = PARK_W * PPI, PARK_D * PPI

    cx, _ = _tile_center_px(3.0, 0.0)

    x0, x1 = cx - w_px/2, cx + w_px/2

    polys.append(("park_zone", [(x0, WINDOW_HEIGHT - d_px), (x1, WINDOW_HEIGHT - d_px), 

                                 (x1, WINDOW_HEIGHT), (x0, WINDOW_HEIGHT)]))

    polys.append(("park_zone", [(x0, 0), (x1, 0), (x1, d_px), (x0, d_px)]))

    

    return polys



def draw_field_objects(surface, cfg):

    """Draw field objects if enabled."""

    if int(cfg.get("ui", {}).get("show_field_objects", 1)) != 1:

        return

    try:

        color_map = {

            'long_goal': (255, 180, 60),

            'center_goal': (70, 140, 255),

            'park_zone': (220, 60, 60),

            'matchloader': (245, 210, 60)

        }

        for name, poly in get_field_object_polys(cfg):

            col = color_map.get(name, (255, 255, 255))

            pygame.draw.polygon(surface, col, [(int(x), int(y)) for (x, y) in poly], 2)

    except Exception:

        pass



def draw_geometry_borders(surface, nodes, cfg, init_heading):

    """Draw geometry constraint boxes at nodes."""

    if int(cfg.get("ui", {}).get("show_hitboxes", 1)) != 1:

        return

    

    bd = cfg["bot_dimensions"]

    pad_px = float(cfg["offsets"].get("padding_in", 0.0)) * PPI

    

    # Compute effective centers (honor ghost offset angles)

    eff = []

    arrival_headings = []

    prev_pos = None

    def _arrival_heading(idx, prev_pd, prev_swing, prev_start, prev_eff, this_pos):

        """Heading coming into node idx from previous segment (paths/swing aware)."""

        heading = None

        if prev_swing and prev_swing.get("target_heading") is not None:

            heading = prev_swing.get("target_heading")
            if prev_swing.get("reverse"):
                heading = (heading + 180.0) % 360.0

        has_curve = prev_pd.get("use_path", False) or bool(prev_pd.get("pose_preview_points"))

        if heading is None and has_curve:

            pts = list(prev_pd.get("path_points") or prev_pd.get("pose_preview_points") or [])

            if not pts:

                cps = list(prev_pd.get("control_points", []))

                if len(cps) >= 2:

                    c0 = prev_start if prev_start is not None else prev_eff

                    cps[0] = c0 if c0 is not None else this_pos

                    cps[-1] = this_pos

                    pts = generate_bezier_path(cps, num_samples=20)

            if pts:

                heading = calculate_path_heading(pts, len(pts) - 1)

        if heading is None and prev_eff is not None:

            heading = heading_from_points(prev_eff, this_pos)

        return None if heading is None else (heading % 360.0)

    for i, node in enumerate(nodes):

        p = node["pos"]
        out_pd = node.get("path_to_next", {}) if i < len(nodes) - 1 else {}
        out_swing = out_pd.get("swing_vis")
        if out_swing and out_swing.get("start_pos") is not None:
            p = out_swing.get("start_pos")

        if i == 0:

            eff.append(p)

            arrival_headings.append(None)

            prev_pos = p

            continue

        node_offset_in = get_node_offset_in(node, cfg, i)

        ghost_ang = node.get("offset_ghost_angle")
        prev_pd = nodes[i - 1].get("path_to_next", {}) if i - 1 >= 0 else {}
        prev_swing = prev_pd.get("swing_vis")
        start_override = prev_pd.get("start_override")
        if prev_swing and prev_swing.get("end_pos") is not None:
            start_override = prev_swing.get("end_pos")
        prev_start = start_override if start_override is not None else prev_pos

        incoming_heading = _arrival_heading(i, prev_pd, prev_swing, prev_start, eff[i - 1] if eff else prev_pos, p)
        if incoming_heading is None:
            incoming_heading = _incoming_path_heading(i)

        arrival_headings.append(incoming_heading)

        if node_offset_in != 0.0 and ghost_ang is None and incoming_heading is not None:

            ghost_ang = incoming_heading

            node["offset_ghost_angle"] = ghost_ang

        if node_offset_in != 0.0 and ghost_ang is not None:

            rad = math.radians(ghost_ang)

            eff.append((p[0] - math.cos(rad) * node_offset_in * PPI,

                        p[1] + math.sin(rad) * node_offset_in * PPI))

        elif node_offset_in != 0.0 and prev_start is not None:

            ux, uy = approach_unit(prev_start, p)

            eff.append((p[0] - ux * node_offset_in * PPI, p[1] - uy * node_offset_in * PPI))

        else:

            eff.append(p)

        prev_pos = p

    # Preserve original effective centers; use overrides only when sampling headings
    eff_override = list(eff)

    

    reverse_state = reshape_state = False



    def _incoming_path_heading(idx: int):
        """Return final heading of path into node idx, if previous segment is curved."""
        if idx <= 0:
            return None
        pd = nodes[idx - 1].get("path_to_next", {})
        start_override = pd.get("start_override")
        end_override = pd.get("swing_vis", {}).get("end_pos") or pd.get("pose_preview_points", [None, None])[-1] if pd.get("pose_preview_points") else None
        has_curve = pd.get("use_path", False) or pd.get("pose_preview_points")
        if not has_curve:
            return None
        path_pts = list(pd.get("path_points") or pd.get("pose_preview_points") or [])
        if path_pts and len(path_pts) >= 2:
            path_pts[0] = start_override if start_override is not None else (eff_override[idx - 1] if idx - 1 < len(eff_override) else eff[idx - 1])
            path_pts[-1] = end_override if end_override is not None else (eff_override[idx] if idx < len(eff_override) else eff[idx])
            pts = path_pts
        else:
            cps = list(pd.get("control_points", []))
            if len(cps) < 2:
                return None
            cps[0] = start_override if start_override is not None else (eff_override[idx - 1] if idx - 1 < len(eff_override) else eff[idx - 1])
            cps[-1] = end_override if end_override is not None else (eff_override[idx] if idx < len(eff_override) else eff[idx])
            pts = generate_bezier_path(cps, num_samples=50)
        if not pts:
            return None
        return calculate_path_heading(pts, len(pts) - 1)
    

    def _shrink_poly(poly, inset_px=1.0):

        """Shrink polygon toward centroid to ignore stroke outlines."""

        if not poly or inset_px <= 0:

            return poly

        cx = sum(p[0] for p in poly) / len(poly)

        cy = sum(p[1] for p in poly) / len(poly)

        shrunk = []

        for (x, y) in poly:

            dx, dy = x - cx, y - cy

            dist = (dx * dx + dy * dy) ** 0.5

            if dist <= 1e-6:

                shrunk.append((x, y))

                continue

            scale = max(0.0, (dist - inset_px) / dist)

            shrunk.append((cx + dx * scale, cy + dy * scale))

        return shrunk



    for i, node in enumerate(nodes):

        # Compute outgoing heading (path-aware) from this node, if any

        outgoing_h = None
        swing_vis_here = nodes[i].get("path_to_next", {}).get("swing_vis")
        if i < len(nodes) - 1:
            next_pd = nodes[i].get("path_to_next", {})
            p0_override = swing_vis_here.get("end_pos") if swing_vis_here else next_pd.get("start_override")
            if swing_vis_here:
                # At swing start, orient to the entry heading (end of prior segment)
                outgoing_h = swing_vis_here.get("start_heading") or swing_vis_here.get("target_heading")
            if outgoing_h is None and (next_pd.get("use_path", False) or next_pd.get("pose_preview_points")):
                path_pts = list(next_pd.get("path_points") or next_pd.get("pose_preview_points") or [])
                if path_pts and len(path_pts) >= 2:
                    path_pts[0] = p0_override if p0_override is not None else eff_override[i]
                    path_pts[-1] = eff_override[i+1]
                    outgoing_h = calculate_path_heading(path_pts, 0)
                else:
                    cps = list(next_pd.get("control_points") or [])
                    if len(cps) >= 2:
                        cps[0] = p0_override if p0_override is not None else eff_override[i]
                        cps[-1] = eff_override[i+1]
                        pts = generate_bezier_path(cps, num_samples=20)
                        if pts:
                            outgoing_h = calculate_path_heading(pts, 0)
            if outgoing_h is None:
                p0_eff = p0_override if p0_override is not None else eff_override[i]
                outgoing_h = heading_from_points(p0_eff, eff_override[i+1])
        

        # Compute heading using effective centers:
        # - Intermediate nodes: face the outgoing segment/arc
        # - Final node: face the arrival (end of arc/path)

        incoming_h = arrival_headings[i] if i < len(arrival_headings) else None
        if i > 0 and incoming_h is None and nodes[i-1].get("move_to_pose") and nodes[i-1].get("pose_heading_deg") is not None:
            incoming_h = nodes[i-1].get("pose_heading_deg")

        if i == len(nodes) - 1:
            # Final node uses arrival heading if available
            if incoming_h is not None:
                base_h = incoming_h
            elif outgoing_h is not None:
                base_h = outgoing_h
            else:
                base_h = heading_from_points(eff[i-1], eff[i]) if i > 0 else 0.0
        else:
            # Intermediate node favors outgoing direction
            if outgoing_h is not None:
                base_h = outgoing_h
            elif incoming_h is not None:
                base_h = incoming_h
            else:
                base_h = heading_from_points(eff[i-1], eff[i]) if i > 0 else 0.0

        

        # Apply reverse state and upcoming reverse toggle for outgoing orientation

        outgoing_reverse = reverse_state

        if i < len(nodes) - 1 and nodes[i].get("reverse", False):

            outgoing_reverse = not outgoing_reverse

        eff_heading = (base_h + (180.0 if outgoing_reverse else 0.0)) % 360.0



        # Apply reverse toggle after processing this node so outgoing reflects it

        if node.get("reverse", False):

            reverse_state = not reverse_state

        

        # Update reshape state

        if node.get("reshape_toggle", False):

            reshape_state = not reshape_state

        for act in node.get("actions", []):

            if act.get("type") in ("reshape", "geom"):

                reshape_state = not reshape_state

        

        # Get dimensions

        if reshape_state:

            full_w = bd.get("reshape_width", bd.get("width", 0.0))

            full_l = bd.get("reshape_length", bd.get("length", 0.0))

            off_x = bd.get("reshape_offset_x_in", 0.0)

            off_y = bd.get("reshape_offset_y_in", 0.0)

        else:

            full_w = bd.get("width", 0.0)

            full_l = bd.get("length", 0.0)

            off_x = bd.get("full_offset_x_in", 0.0)

            off_y = bd.get("full_offset_y_in", 0.0)

        

        corners = oriented_rect_corners_px(eff[i], eff_heading, full_w, full_l, off_x, off_y)

        corners_inner = _shrink_poly(corners, inset_px=1.0)

        is_oob = rect_oob(corners_inner, pad_px, WINDOW_WIDTH, WINDOW_HEIGHT)

        

        # Check field object collisions

        try:

            if int(cfg.get('ui', {}).get('show_field_objects', 1)) == 1:

                for _name, _poly in get_field_object_polys(cfg):

                    if _name in ('park_zone', 'matchloader'):

                        continue

                    poly_inner = _shrink_poly(_poly, inset_px=1.0)

                    if polygons_intersect(corners_inner, poly_inner):

                        is_oob = True

                        break

        except Exception:

            pass

        

        color = RED if is_oob else GREY

        pygame.draw.polygon(surface, color, corners, 2)



def draw_follow_geometry(surface, cfg, pos_px, heading_deg, reshape_state_live):

    """Draw robot geometry boxes that follow motion."""

    bd = cfg["bot_dimensions"]

    pad_px = float(cfg["offsets"].get("padding_in", 0.0)) * PPI

    

    # Geometry box

    if reshape_state_live:

        gw = bd.get("reshape_width", bd.get("width", 0.0))

        gl = bd.get("reshape_length", bd.get("length", 0.0))

        off_x = bd.get("reshape_offset_x_in", 0.0)

        off_y = bd.get("reshape_offset_y_in", 0.0)

    else:

        gw = bd.get("width", 0.0)

        gl = bd.get("length", 0.0)

        off_x = bd.get("full_offset_x_in", 0.0)

        off_y = bd.get("full_offset_y_in", 0.0)

    

    g_corners = oriented_rect_corners_px(pos_px, heading_deg, gw, gl, off_x, off_y)

    # Shrink for OOB/object checks to ignore stroke outlines

    def _shrink_poly(poly, inset_px=1.0):

        if not poly or inset_px <= 0:

            return poly

        cx = sum(p[0] for p in poly) / len(poly)

        cy = sum(p[1] for p in poly) / len(poly)

        shrunk = []

        for (x, y) in poly:

            dx, dy = x - cx, y - cy

            dist = (dx * dx + dy * dy) ** 0.5

            if dist <= 1e-6:

                shrunk.append((x, y))

                continue

            scale = max(0.0, (dist - inset_px) / dist)

            shrunk.append((cx + dx * scale, cy + dy * scale))

        return shrunk

    g_corners_inner = _shrink_poly(g_corners, inset_px=1.0)

    g_oob = rect_oob(g_corners_inner, pad_px, WINDOW_WIDTH, WINDOW_HEIGHT)

    

    # Check collisions

    try:

        if int(cfg.get('ui', {}).get('show_field_objects', 1)) == 1:

            for _name, _poly in get_field_object_polys(cfg):

                if _name in ('park_zone', 'matchloader'):

                    continue

                poly_inner = _shrink_poly(_poly, inset_px=1.0)

                if polygons_intersect(g_corners_inner, poly_inner):

                    g_oob = True

                    break

    except Exception:

        pass

    

    pygame.draw.polygon(surface, RED if g_oob else GREY, g_corners, 2)

    

    # Drivetrain box

    dtw = bd.get("dt_width", bd.get("width", 0.0))

    dtl = bd.get("dt_length", bd.get("length", 0.0))

    dt_corners = oriented_rect_corners_px(pos_px, heading_deg, dtw, dtl, 0.0, 0.0)

    pygame.draw.polygon(surface, WHITE, dt_corners, 2)


