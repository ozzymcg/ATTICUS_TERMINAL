import pygame, math, sys, json, copy, os, pygame.gfxdraw
import tkinter as tk
from tkinter import filedialog, simpledialog
# python3-tk

# Solely Created by 15800A Atticus, Austin McGrath
icon_path = os.path.join(os.path.dirname(__file__), 'ATTICUS.png')
pygame.display.set_icon(pygame.image.load(icon_path))

undo_stack = []
redo_stack = []
dragging_node = False
drag_node_index = None
drag_start_pos = None

# --- Configuration and Globals ---
FIELD_WIDTH_2FT = 6     # Field is a 12ft x 12ft grid (6x6 in 24in^2 tiles)
FIELD_HEIGHT_2FT = 6
PIXELS_PER_2FT = 100 

CONFIG_FILENAME = "auton_config.json"
constant_k = 2000 

reverse_mode = False
current_segment_g = False

WINDOW_WIDTH = FIELD_WIDTH_2FT * PIXELS_PER_2FT
WINDOW_HEIGHT = FIELD_HEIGHT_2FT * PIXELS_PER_2FT

GRID_SIZE_FT = 1
GRID_SIZE_PX = GRID_SIZE_FT * PIXELS_PER_2FT

BG_COLOR = (30, 30, 30)
GRID_COLOR = (50, 50, 50)
NODE_COLOR = (50, 255, 50)
GOAL_COLOR = (255, 255, 0)
ROBOT_COLOR = (252, 3, 248)
ARROW_COLOR = (255, 255, 255) # Chevron indicator
TEXT_COLOR = (255, 255, 255)
RED = (255, 0, 0)
ORANGE = (255, 78, 2)
SELECTED = TEXT_COLOR  # Alternatively (173, 216, 230)

# --- Initialization ---
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("THE ATTICUS TERMINAL")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

print("""
                    Controls:
                    Q               - Toggle Snapless Grid
                    RClick/Delete   - Delete Node
                    S/L             - Save/Load Routine
                    H               - Set Initial Robot Heading (Unit Circle)
                    C               - Compile Directions + Coordinates After Running
                    SPACE           - Start/Pause Animation
                    CTRL + SPACE    - Stop Animation & Reset
                    V               - ESTIMATE AUTON TIME
                    B               - Curved Pathing (Brakeless/Coast)
                    F               - Bot Center Drives to an offset before node
                                        press a second time for wall stakes.
                    R               - Apply REVERSE to most recent node
                    G               - Toggle CLAMP to most recent mode

""")

# Initialize tkinter for dialogs and hide the window.
tk_root = tk.Tk()
tk_root.withdraw()

# <<---------------------------------------------------------------------------->>

# --- Helper: Convert pixel coordinates to logical coordinates ---
def pixel_to_logical(pos, rounded=True):
    # 6x6 24in tiles bottom-left is (0,0) and top-right is (6,6).
    x, y = pos
    if rounded == True:
        logical_x = round(x / PIXELS_PER_2FT, 3)
        logical_y = round((WINDOW_HEIGHT - y) / PIXELS_PER_2FT, 3)
    else:
        logical_x = x / PIXELS_PER_2FT
        logical_y = (WINDOW_HEIGHT - y) / PIXELS_PER_2FT

    return (logical_x, logical_y)


# --- Calculation Functions ---
def calc_distance(p1, p2):
    "Return distance in inches"
    pixel_distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    inches = (pixel_distance / PIXELS_PER_2FT) * 12 * 2
    return inches

def calc_angle(current_heading, p_start, p_target, node=None):
    "Returns calculated angle and angle difference"
    delta_x = p_target[0] - p_start[0]
    delta_y = p_target[1] - p_start[1]

    # For vertical movement, swap the degrees.
    if abs(delta_x) < 1e-6:
        desired = 270 if delta_y > 0 else 90
    elif abs(delta_y) < 1e-6: 
        desired = 0 if delta_x > 0 else 180
    else:
        desired = math.degrees(math.atan2(-delta_y, delta_x)) % 360
    
    if node and node.get("heading_flip", False): #Flip heading if in reverse
        flip_amount = node.get("flip_amount", 0)
        desired = (desired + flip_amount) % 360
    
    diff = (desired - current_heading + 360) % 360
    
    if diff > 180:
        diff -= 360

    return diff, desired
    # short for calculator guys im just using slang

def calc_adjustment(node, heading, corneronly=False, width=None, height=None):
    """Return the four corners of a square  centered on node,
       rotated by the given heading (in degrees)."""

    def getcorners(node, heading, width, height):
        "Return corner coordinates"
        center = node["pos"]
        pixels_per_inch = PIXELS_PER_2FT / 24  
        width_px = (width if width else bot_dimensions["width"]) * pixels_per_inch
        height_px = (height if height else bot_dimensions["height"]) * pixels_per_inch
        half_width = width_px / 2
        half_height = height_px / 2

        if width is None and height is None:
            # Read offsets (in inches) from config.
            dt_offset_x = config["dtposition_offset"]["x"]["value"]
            dt_offset_y = config["dtposition_offset"]["y"]["value"]
            # In our local robot frame, let:
            #   - dt_offset_y be the forward offset,
            #   - dt_offset_x be the lateral offset.
            # Convert these to pixels and rotate them by the heading.
            offset_px_x = dt_offset_y * pixels_per_inch * math.cos(math.radians(heading)) + dt_offset_x * pixels_per_inch * math.sin(math.radians(heading))
            offset_px_y = -dt_offset_y * pixels_per_inch * math.sin(math.radians(heading)) + dt_offset_x * pixels_per_inch * math.cos(math.radians(heading))
            center = (center[0] + offset_px_x, center[1] + offset_px_y)

        # Define rectangle corners relative to center:
        corners = [(-half_height, -half_width), (half_height, -half_width),
                   (half_height, half_width), (-half_height, half_width)]
        angle_rad = -math.radians(heading)
        rotated = []
        for dx, dy in corners:
            rx = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
            ry = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
            rotated.append((center[0] + rx, center[1] + ry))
        return rotated

    if corneronly == False:
        corners = getcorners(node, heading, width, height)
        
        rel_offsets = [(pt[0] - node["pos"][0], pt[1] - node["pos"][1]) for pt in corners]
        rel_xs = [dx for dx, dy in rel_offsets]
        rel_ys = [dy for dx, dy in rel_offsets]
        # Allowed center x: center_x + min(rel_xs) >= 0 and center_x + max(rel_xs) <= WINDOW_WIDTH
        allowed_min_x = -min(rel_xs)
        allowed_max_x = WINDOW_WIDTH - max(rel_xs)
        allowed_min_y = -min(rel_ys)
        allowed_max_y = WINDOW_HEIGHT - max(rel_ys)
            
        original_x, original_y = node["pos"]
        effective_x = max(allowed_min_x, min(original_x, allowed_max_x))
        effective_y = max(allowed_min_y, min(original_y, allowed_max_y))
            
        return (effective_x, effective_y)
    else:
        return getcorners(node, heading, width, height)

def calc_mogo(node, heading, clamp_offset_in=None, short_width_in=10, padding_in=None):
    """
        Given the intended mobile-goal node position and the robot's heading (in degrees),
    compute the effective target for the clamp using a hexagon that represents the clamp's
    extended border.
    
    The clamp mechanism puts the clamp inside the bot's front.
    Therefore, we first shift p_end backward by clamp_offset_in inches along the forward vector.
    Then we construct a hexagon (with a flat top) in local coordinates, rotate it so that
    its top flat side (which in standard local coords is horizontal) is aligned with the robot's front,
    and then adjust the hexagon's center (and thus the target) if any vertex lies out-of-bounds.
    
    Returns a tuple (adjusted_hex_center, adjusted_vertices) where adjusted_hex_center
    is used as the effective target for motion.
    
    Includes a consideration for customizable wall padding."""

    if clamp_offset_in is None:
        clamp_offset_in = offsets["clamp_offset_in"]

    if padding_in is None:
        padding_in = offsets["padding_in"]

    # Compute the normal adjusted target for the robot's square.
    normal_adj = calc_adjustment(node, heading)

    # Compute the unadjusted hexagon center.
    pixels_per_inch = PIXELS_PER_2FT / 24.0
    padding_offset_px = padding_in * pixels_per_inch
    clamp_offset_px = ((bot_dimensions["dt_height"] / 2) + 5 - clamp_offset_in) * pixels_per_inch # + padding_offset_px # border is 2px
    rad = math.radians(heading)
    forward = (math.cos(rad), -math.sin(rad))
    hex_center_unadj = (normal_adj[0] - clamp_offset_px * forward[0],
                        normal_adj[1] - clamp_offset_px * forward[1])
    
    # Define hexagon parameters in inches.
    apothem_in = short_width_in / 2.0         # 5 inches for a 10-inch short width
    circumradius_in = short_width_in / math.sqrt(3)  # ~5.77 inches
    R = circumradius_in
    A = apothem_in
    
    # Define the hexagon vertices in local (inch) coordinates.
    local_vertices_in = [
        ( R/2, -A),   # v0: one endpoint of the flat (clamp) face
        ( R,    0),   # v1
        ( R/2,  A),   # v2
        (-R/2,  A),   # v3
        (-R,    0),   # v4
        (-R/2, -A)    # v5: other endpoint of the clamp face
    ]
    
    # Convert local vertices to pixels.
    local_vertices_px = [(x * pixels_per_inch, y * pixels_per_inch) for (x, y) in local_vertices_in]
    
    # Rotate the hexagon so that the flat face is aligned with the robot's front.
    # Rotation angle: heading - 90°.
    rotation_angle = math.radians(heading - 90)
    cos_a = math.cos(rotation_angle)
    sin_a = math.sin(rotation_angle)
    rotated_vertices = []
    for (x, y) in local_vertices_px:
        xr = x * cos_a - y * sin_a
        yr = x * sin_a + y * cos_a
        rotated_vertices.append((hex_center_unadj[0] + xr, hex_center_unadj[1] + yr))
    
    # Compute the necessary offset to bring any out-of-bound vertices back in.
    offset_x = 0
    offset_y = 0
    min_vx = min(x for x, y in rotated_vertices)
    max_vx = max(x for x, y in rotated_vertices)
    min_vy = min(y for x, y in rotated_vertices)
    max_vy = max(y for x, y in rotated_vertices)
    
    # Compute offset for x-axis:
    offset_x = 0
    if min_vx < padding_offset_px:
        offset_x = padding_offset_px - min_vx
    if max_vx > WINDOW_WIDTH - padding_offset_px:
        offset_x = (WINDOW_WIDTH - padding_offset_px) - max_vx

    # Compute offset for y-axis:
    offset_y = 0
    if min_vy < padding_offset_px:
        offset_y = padding_offset_px - min_vy
    if max_vy > WINDOW_HEIGHT - padding_offset_px:
        offset_y = (WINDOW_HEIGHT - padding_offset_px) - max_vy


    # Apply the offset to the normal adjusted target.
    final_adjusted = (normal_adj[0] + offset_x, normal_adj[1] + offset_y)
    return final_adjusted


# --- Line & Animation Computation Helpers ---
def compute_segment_time(distance, V, a):
    """
    Compute time to travel a given distance (in inches) when starting from rest,
    accelerating at rate 'a' (inches/s^2) until reaching full speed V (inches/s).
    If distance is too short to reach full speed, returns time computed from s = 0.5*a*t^2.
    Otherwise, time = (time to accelerate) + (remaining distance at constant speed).
    """
    d_accel = 0.5 * (V**2) / a
    t_accel = V / a
    if distance >= d_accel:
        return t_accel + (distance - d_accel) / V
    else:
        return math.sqrt(2 * distance / a)

def segment_rstatus(nodes, segment_index):
    # Start with node 0's reverse flag.
    reverse_state = nodes[0].get("reverse", False)
    # For segments beyond node 0, toggle for nodes 1 through segment_index.
    for j in range(1, segment_index + 1):
        if nodes[j].get("reverse", False):
            reverse_state = not reverse_state
    return reverse_state

def segment_gstatus(nodes, segment_index):
    # Start with node 0 always off
    g_state = False  
    for j in range(1, segment_index + 1):
        if nodes[j].get("g", False):
            g_state = not g_state
    return g_state

def get_mogo_vertices(bot_center, heading, clamp_offset_in=None, short_width_in=10):
    """Returns coordinates of mogo. Used in drawing functions."""

    if clamp_offset_in is None:
        clamp_offset_in = offsets["clamp_offset_in"]
    pixels_per_inch = PIXELS_PER_2FT / 24.0
    clamp_offset_px = ((bot_dimensions["dt_height"] / 2) + 5 - clamp_offset_in) * pixels_per_inch
    rad_heading = math.radians(heading)
    forward = (math.cos(rad_heading), -math.sin(rad_heading))
    # Compute the hexagon center based on the bot's center.
    hex_center = (bot_center[0] - clamp_offset_px * forward[0],
                  bot_center[1] - clamp_offset_px * forward[1])
    # Hexagon parameters.
    apothem_in = short_width_in / 2.0
    circumradius_in = short_width_in / math.sqrt(3)
    R = circumradius_in
    A = apothem_in
    # Define the local vertices (flat-topped hexagon).
    local_vertices_in = [
        ( R/2, -A),
        ( R,    0),
        ( R/2,  A),
        (-R/2,  A),
        (-R,    0),
        (-R/2, -A)
    ]

    dx = bot_center[0] - hex_center[0]
    dy = bot_center[1] - hex_center[1]
    desired_deg = math.degrees(math.atan2(-dx, dy)) % 360
    rotation_angle = math.radians(desired_deg)
    cos_a = math.cos(rotation_angle)
    sin_a = math.sin(rotation_angle)

    vertices = []
    for (x_in, y_in) in local_vertices_in:
        x_px = x_in * pixels_per_inch
        y_px = y_in * pixels_per_inch
        x_rot = x_px * cos_a - y_px * sin_a
        y_rot = x_px * sin_a + y_px * cos_a
        vertices.append((hex_center[0] + x_rot, hex_center[1] + y_rot))
    return vertices


# --- Utility Drawing Functions ---
def draw_grid(surface):
    for x in range(0, WINDOW_WIDTH, int(GRID_SIZE_PX)):
        pygame.draw.line(surface, GRID_COLOR, (x, 0), (x, WINDOW_HEIGHT))
    for y in range(0, WINDOW_HEIGHT, int(GRID_SIZE_PX)):
        pygame.draw.line(surface, GRID_COLOR, (0, y), (WINDOW_WIDTH, y))

def snap_to_grid(pos, snap_enabled=True):
    """Snaps placed nodes to grid."""
    if not snap_enabled:
        return pos
    x, y = pos
    # Medium grid snap: half-tile increments.
    snapped_x = round(x / (GRID_SIZE_PX / 2)) * (GRID_SIZE_PX / 2)
    snapped_y = round(y / (GRID_SIZE_PX / 2)) * (GRID_SIZE_PX / 2)
    return (snapped_x, snapped_y)

def draw_nodes(surface, nodes):
    win_left = 0
    win_right = WINDOW_WIDTH
    win_top = 0
    win_bottom = WINDOW_HEIGHT

    def in_bounds(p):
        x, y = p
        return win_left <= x <= win_right and win_top <= y <= win_bottom

    def project_to_edge(x, y):
        """Project a point to the nearest valid edge while maintaining the curve's shape."""
        if x < win_left:
            return (win_left, y)
        elif x > win_right:
            return (win_right, y)
        elif y < win_top:
            return (x, win_top)
        elif y > win_bottom:
            return (x, win_bottom)
        return (x, y)

    for i, node in enumerate(nodes):
        center_for_border = node["pos"]

        if i == 0:
            node_heading = initial_state["heading"]
            default_border_color = GRID_COLOR
            
            # Regular robot border
            corners = calc_adjustment({"pos": center_for_border}, node_heading, True)
            pygame.draw.polygon(surface, default_border_color, corners, 2)  # Grey border for drivetrain

            # Drivetrain border (dt_width x dt_height)
            corners_dt = calc_adjustment({"pos": center_for_border}, node_heading, True, 
                                        width=bot_dimensions["dt_width"], height=bot_dimensions["dt_height"])
            pygame.draw.polygon(surface, (255, 255, 255), corners_dt, 2)  # Grey border for drivetrain

        else:
            default_border_color = GRID_COLOR
            if node.get("ring", False) or node.get("wall", False):
                # For mobile-goal (ring) nodes, simply use the node's position.
                center_for_border = node["pos"]
                # Compute node_heading from the previous node if available.
                if i > 0:
                    prev_node = nodes[i-1]
                    dx = node["pos"][0] - prev_node["pos"][0]
                    dy = node["pos"][1] - prev_node["pos"][1]
                    node_heading = math.degrees(math.atan2(-dy, dx)) % 360
                else:
                    node_heading = initial_state["heading"]

                if node.get("heading_flip", False):
                    drawing_heading = (node_heading - node.get("flip_amount", 0)) % 360
                else:
                    drawing_heading = node_heading

                # Compute landing target and apply offset.
                if node.get("ring", False):
                    offset_px = offsets["ring_node_offset_in"] * (PIXELS_PER_2FT / 24.0)
                elif node.get("wall", False):
                    offset_px = offsets["wall_stake_offset_in"] * (PIXELS_PER_2FT / 24.0)

                if node.get("ring", False) or node.get("wall", False):
                    final_target = (
                            node["pos"][0] - offset_px * math.cos(math.radians(drawing_heading)),
                            node["pos"][1] + offset_px * math.sin(math.radians(drawing_heading))
                    )
                else:
                    final_target = calc_adjustment({"pos": node["pos"]}, drawing_heading)
                
                node["adjusted_pos"] = final_target

                ghost_corners = calc_adjustment({"pos": final_target}, drawing_heading, True)

                if any(x < 0 or x > WINDOW_WIDTH or y < 0 or y > WINDOW_HEIGHT for (x, y) in ghost_corners):
                    border_color = RED
                else:
                    border_color = GRID_COLOR

                # Compute the border corners based on the offset target.
                corners = calc_adjustment({"pos": final_target}, drawing_heading, True)

            else:
                # For regular nodes, compute heading from the previous node.
                prev_node = nodes[i-1]
                dx = node["pos"][0] - prev_node["pos"][0]
                dy = node["pos"][1] - prev_node["pos"][1]
                node_heading = math.degrees(math.atan2(-dy, dx)) % 360
                center_for_border = node["pos"]
                corners = calc_adjustment({"pos": center_for_border}, node_heading, True)
                if any(x < 0 or x > WINDOW_WIDTH or y < 0 or y > WINDOW_HEIGHT for (x, y) in corners):
                    node["border_out"] = True
                    border_color = RED
                else:
                    node["border_out"] = False
                    border_color = GRID_COLOR
                corners = calc_adjustment({"pos": calc_adjustment({"pos": center_for_border}, node_heading)}, node_heading, True)

        # Draw the border using the computed color.
        if i != 0:
            if not (segment_rstatus(nodes, i-1) and segment_gstatus(nodes, i-1) and not segment_gstatus(nodes, i)):
                pygame.draw.polygon(surface, border_color, corners, 2)


    if len(nodes) < 2:
        return

    weight = robot_physics["weight"]        
    rpm = robot_physics["rpm"]               
    volts_turn = robot_physics["volts_turn"]      
    diameter = robot_physics["diameter"]     

    effective_rpm_turn = (volts_turn / 12.0) * rpm
    V_turn = (effective_rpm_turn * math.pi * diameter) / 60.0  
    a_turn = (volts_turn / 12.0) * (constant_k / weight)           
    R_turn = V_turn**2 / a_turn if a_turn != 0 else float('inf')

    prev_CP = None  

    for i in range(len(nodes) - 1):
        start = nodes[i]["pos"]
        end = nodes[i+1]["pos"]

        # Bezier Curve Computation
        # If the next node is marked as a bezier node, compute a quadratic Bezier control point.
        if nodes[i+1].get("bezier", False):
            # For consecutive bezier nodes, reflect the previous control point for smooth continuity.
            if i > 0 and nodes[i].get("bezier", False) and prev_CP is not None:
                CP = (2 * start[0] - prev_CP[0], 2 * start[1] - prev_CP[1])
            else:
                S = start  # Starting point of the segment
                T = end    # Ending point of the segment
                D = (T[0] - S[0], T[1] - S[1])  # Vector from start to end
                L = math.hypot(D[0], D[1])  # Distance between S and T

                # If the distance is almost zero, simply use the midpoint.
                if L < 1e-3:
                    CP = ((S[0] + T[0]) / 2, (S[1] + T[1]) / 2)
                else:
                    # Determine the desired heading for the curve.
                    # Use the initial robot heading for the first segment; otherwise, compute based on the previous node.
                    if i == 0:
                        theta = math.radians(initial_state["heading"])
                    else:
                        theta = math.radians(math.degrees(math.atan2(
                            -(S[1]-nodes[i-1]["pos"][1]), S[0]-nodes[i-1]["pos"][0])) % 360)
                    
                    # Compute the angle of the segment from S to T.
                    angle_D = math.atan2(-(T[1] - S[1]), T[0] - S[0])
                    # Beta is the angular difference between the segment's direction and the desired heading.
                    beta = (angle_D - theta + math.pi) % (2 * math.pi) - math.pi
                    sign = 1 if beta >= 0 else -1
                    # Compute a perpendicular unit vector based on the desired heading and the sign of beta.
                    n = (-math.sin(theta), math.cos(theta)) if sign >= 0 else (math.sin(theta), -math.cos(theta))
                    dot = D[0]*n[0] + D[1]*n[1]

                    # If the projection of D on n is negligible, fallback to using the midpoint.
                    if abs(dot) < 1e-3:
                        CP = ((S[0] + T[0]) / 2, (S[1] + T[1]) / 2)
                    else:
                        # Compute a geometric turning radius based on the chord (L) and its offset (dot product).
                        R_geo = (L**2) / (2 * abs(dot))
                        # R_turn is the robot's effective turning radius (precomputed elsewhere).
                        R_effective = max(R_turn, R_geo)
                        # Use a heuristic ratio to constrain the offset.
                        ratio = min(1, L / (2 * R_effective))
                        phi = 2 * math.asin(ratio)
                        # Base offset is determined from half the chord length scaled by the tangent of a quarter angle.
                        offset = (L / 2) * math.tan(phi / 4)
                        # Adjust offset based on the robot's turning capabilities.
                        speed_factor = min(1, effective_rpm_turn / 100.0)
                        offset *= speed_factor
                        # Scale further based on acceleration limits.
                        d_accel_turn = (0.5 * V_turn**2 / a_turn) if a_turn != 0 else L
                        accel_scale = min(1, L / d_accel_turn)
                        offset *= accel_scale
                        # Midpoint between S and T.
                        mid = ((S[0] + T[0]) / 2, (S[1] + T[1]) / 2)
                        # Normalize D.
                        D_unit = (D[0] / L, D[1] / L)
                        # Compute a vector perpendicular to D.
                        D_perp = (-D_unit[1], D_unit[0])
                        # Ensure the perpendicular vector points in the direction of n.
                        if (D_perp[0]*n[0] + D_perp[1]*n[1]) < 0:
                            D_perp = (-D_perp[0], -D_perp[1])
                        # Final control point is the midpoint offset in the perpendicular direction.
                        CP = (mid[0] + offset * D_perp[0], mid[1] + offset * D_perp[1])
            prev_CP = CP

            # Generate a series of points along the quadratic Bezier curve defined by start, CP, and end.
            points = []
            steps = 30
            for j in range(steps + 1):
                t = j / steps
                x = (1 - t)**2 * start[0] + 2 * (1 - t) * t * CP[0] + t**2 * end[0]
                y = (1 - t)**2 * start[1] + 2 * (1 - t) * t * CP[1] + t**2 * end[1]
                points.append((x, y))

            # Adjust any points that fall out-of-bounds to lie along the window edges.
            adjusted_points = []
            for x, y in points:
                if in_bounds((x, y)):
                    adjusted_points.append((int(x), int(y)))
                else:
                    edge_x, edge_y = project_to_edge(x, y)
                    adjusted_points.append((int(edge_x), int(edge_y)))

            # Draw the Bezier curve on the surface.
            pygame.draw.lines(surface, NODE_COLOR, False, adjusted_points, 2)

        else:
            pygame.draw.line(surface, NODE_COLOR, start, end, 2)
            prev_CP = None

    for i, node in enumerate(nodes):
        # Node color assignment
        if "colors" in node:
            node_colors = node["colors"]
        elif node.get("reverse", False) and (node.get("ring", False) or node.get("wall", False)) and (i > 0 and segment_gstatus(nodes, i-1) != segment_gstatus(nodes, i)):
            node_colors = [RED, ORANGE, GOAL_COLOR]  # Example: half red, half orange.
        elif node.get("reverse", False) and (node.get("ring", False) or node.get("wall", False)):
            node_colors = [RED, ORANGE]
        elif node.get("reverse", False) and (i > 0 and segment_gstatus(nodes, i-1) != segment_gstatus(nodes, i)):
            node_colors = [RED, GOAL_COLOR]
        elif (node.get("ring", False) or node.get("wall", False)) and (i > 0 and segment_gstatus(nodes, i-1) != segment_gstatus(nodes, i)):
            node_colors = [ORANGE, GOAL_COLOR]
        elif (node.get("ring", False) or node.get("wall", False)):
            node_colors = [ORANGE]
        elif node.get("reverse", False):
            node_colors = [RED]
        elif i > 0 and segment_gstatus(nodes, i-1) != segment_gstatus(nodes, i):
            node_colors = [GOAL_COLOR]
        else:
            node_colors = [NODE_COLOR]
        
        # Draw the node using the appropriate function.
        if len(node_colors) > 1:
            draw_multicolor_circle(surface, node["pos"], 6, node_colors)
        else:
            pygame.draw.circle(surface, node_colors[0], node["pos"], 6)
        
        if i == selected_node_index:
            pygame.draw.circle(surface, SELECTED, node["pos"], 8, 3)

        # Draw the node label as before.
        text = font.render(str(i), True, TEXT_COLOR)
        text_rect = text.get_rect(center=(node["pos"][0], node["pos"][1] - 15))
        surface.blit(text, text_rect)

        if node.get("left_mogo", False):
            # Use the actual landing position for the ring node.
            # Fall back to node["adjusted_pos"] if a dedicated left_mogo_pos isn't available.
            landing_pos = node.get("left_mogo_pos", node.get("adjusted_pos", node["pos"]))
            # Also use the stored heading if available, otherwise default to the node's drawing heading.
            landing_heading = node.get("left_mogo_heading", node.get("heading", initial_state["heading"]))
            vertices = get_mogo_vertices(landing_pos, landing_heading)
            pygame.draw.polygon(surface, GOAL_COLOR, vertices, 2)
        if "adjusted_pos" in node:
                del node["adjusted_pos"]

def draw_robot(surface, pos):
    pygame.draw.circle(surface, ROBOT_COLOR, pos, 8)

def draw_chevron_indicator(surface, pos, heading, length=20, angle_offset=45, chevron_length=10):
    # If in reverse mode, draw the robot facing 180° offset.
    effective_heading = heading
    rad = math.radians(effective_heading)
    tip_x = pos[0] + length * math.cos(rad)
    tip_y = pos[1] - length * math.sin(rad)
    left_rad = math.radians(effective_heading - angle_offset)
    right_rad = math.radians(effective_heading + angle_offset)
    left_x = tip_x - chevron_length * math.cos(left_rad)
    left_y = tip_y + chevron_length * math.sin(left_rad)
    right_x = tip_x - chevron_length * math.cos(right_rad)
    right_y = tip_y + chevron_length * math.sin(right_rad)
    pygame.draw.line(surface, ARROW_COLOR, (tip_x, tip_y), (left_x, left_y), 3)
    pygame.draw.line(surface, ARROW_COLOR, (tip_x, tip_y), (right_x, right_y), 3)

def draw_mogo_border(surface, bot_center, heading, clamp_offset_in=None, short_width_in=10, line_width=2):
    """
    Draw a regular, flat-topped hexagon representing the clamp.
    This version rotates the hexagon so that its designated flat face (between v0 and v5)
    faces the robot's center (bot_center) rather than the robot's heading.
    """
    if clamp_offset_in is None:
        clamp_offset_in = offsets["clamp_offset_in"]

    # Conversion: inches to pixels (PIXELS_PER_2FT is defined globally)
    pixels_per_inch = PIXELS_PER_2FT / 24.0
    clamp_offset_px = ((bot_dimensions["dt_height"] / 2) + 5 - clamp_offset_in) * pixels_per_inch

    # Distance from bot edge to mogo edge: short diagonal (10in) - clamp offset

    # Compute the forward vector from the current heading (used only for the clamp offset)
    rad_heading = math.radians(heading)
    forward = (math.cos(rad_heading), -math.sin(rad_heading))
    
    # Compute the clamp (hexagon) center by offsetting from the robot's center along the heading
    hex_center = (bot_center[0] - clamp_offset_px * forward[0],
                  bot_center[1] - clamp_offset_px * forward[1])
    
    # Hexagon measurements based on our proportions:
    # Short width (distance between parallel sides) = 10 in, so apothem = 5 in.
    # Side length = 10 / sqrt(3) ≈ 5.77 in.
    apothem_in = short_width_in / 2.0          # 5 inches
    circumradius_in = short_width_in / math.sqrt(3)  # ~5.77 inches
    R = circumradius_in
    A = apothem_in
    
    # Define vertices for a flat-topped hexagon centered at (0,0).
    # We designate the flat face formed by v0 and v5 as the clamp face.
    # In these local coordinates, v0 = (R/2, -A) and v5 = (-R/2, -A) share a horizontal edge.
    # The inward normal for this face (pointing toward the clamp) is (0, 1).
    local_vertices_in = [
        ( R/2, -A),   # v0: one endpoint of the flat (clamp) face
        ( R,    0),   # v1
        ( R/2,  A),   # v2
        (-R/2,  A),   # v3
        (-R,    0),   # v4
        (-R/2, -A)    # v5: other endpoint of the clamp face
    ]
    
    # To have the clamp face (with inward normal (0, 1)) point toward the robot's center,
    # compute the vector from hex_center to bot_center.
    dx = bot_center[0] - hex_center[0]
    dy = bot_center[1] - hex_center[1]
    # In our heading convention (0° = right, 90° = up), a unit vector is obtained via:
    #     unit = (cos(heading), -sin(heading))
    # When aligning (0,1) with the computed vector, solving R(theta)*(0,1) = (dx, dy)/d
    # leads to theta = arctan2(-dx, dy). This theta (in degrees) is our desired rotation.
    desired_deg = math.degrees(math.atan2(-dx, dy)) % 360
    rotation_angle = math.radians(desired_deg)
    
    # Convert each local vertex from inches to pixels, rotate by rotation_angle, and translate by hex_center.
    vertices = []
    for (x_in, y_in) in local_vertices_in:
        x_px = x_in * pixels_per_inch
        y_px = y_in * pixels_per_inch
        x_rot = x_px * math.cos(rotation_angle) - y_px * math.sin(rotation_angle)
        y_rot = x_px * math.sin(rotation_angle) + y_px * math.cos(rotation_angle)
        vertices.append((hex_center[0] + x_rot, hex_center[1] + y_rot))
    
    # Determine drawing color (red if any vertex is out-of-bounds)
    out_of_bounds = any(x < 0 or x > WINDOW_WIDTH or y < 0 or y > WINDOW_HEIGHT for (x, y) in vertices)
    color = (255, 0, 0) if out_of_bounds else GOAL_COLOR

    pygame.draw.polygon(surface, color, vertices, line_width)
    return vertices

def draw_multicolor_circle(surface, pos, radius, colors):
    """Draws a circle at pos divided into wedges using the provided list of colors."""
    cx, cy = pos
    n = len(colors)
    if n <= 0:
        return
    if n == 1:
        pygame.draw.circle(surface, colors[0], pos, radius)
        return

    # Calculate the angle (in degrees) for each wedge.
    angle_per_wedge = 360 / n
    num_points = 20  # Number of points along each arc for a smooth wedge.

    for i, color in enumerate(colors):
        # Define the start and end angles in degrees.
        start_angle = i * angle_per_wedge
        end_angle = (i + 1) * angle_per_wedge
        # Build the wedge polygon points.
        points = [pos]  # Start at the center.
        for j in range(num_points + 1):
            # Interpolate between the start and end angles.
            angle = math.radians(start_angle + (end_angle - start_angle) * j / num_points)
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((int(x), int(y)))
        pygame.draw.polygon(surface, color, points)


# --- Robot Animation Function ---
def move_robot_along(p_start, p_end, current_heading, fps=60, node=None, start_reverse=False, from_node_index=None, to_node_index=None):
    global odometry_pos
    # Compute basic turn angle.
    if from_node_index is not None and from_node_index == 0:
        diff, desired = calc_angle(initial_state['heading'], p_start, p_end)
    else:
        diff, desired = calc_angle(current_heading, p_start, p_end)
    # If this segment should run in reverse, add 180°.

    margin = 20  
    node_in_corner = False
    if node is not None:
        node_x, node_y = node["pos"]
        # Check if the node's position is near both a horizontal and vertical edge.
        node_in_corner = ((node_x < margin or node_x > WINDOW_WIDTH - margin) and
                            (node_y < margin or node_y > WINDOW_HEIGHT - margin))    

    if start_reverse:
        # Special-case for segment 0->1 (assuming p_start equals the initial position)
        if p_start == initial_state["position"]:
            reverse_heading = (desired + 180) % 360
            diff = (reverse_heading - current_heading + 360) % 360
            if diff > 180:
                diff -= 360
            computed_heading = reverse_heading
        else:
            computed_heading = (desired + 180) % 360
        
        # Use an adjusted target if the node is in a corner.
        if node_in_corner:
            base_target = calc_mogo({"pos": p_end}, computed_heading)
        else:
            base_target = calc_adjustment({"pos": p_end}, computed_heading)
        
        # Set left behind border if the node is in a corner and clamp toggle is off.
        if node is not None and node_in_corner and node.get("g", False):
            node["left_mogo"] = True  
            node["left_mogo_pos"] = base_target
            node["left_mogo_heading"] = computed_heading
    else:
        computed_heading = desired
        base_target = calc_adjustment({"pos": p_end}, computed_heading)

    # Calculate an initial base_target.
    effective_start = calc_adjustment({"pos": p_start}, computed_heading)

    if node and node.get("ring", False):
        # If it's a ring node, apply the ring offset and DO NOT recalc further.
        offset_px = offsets["ring_node_offset_in"] * (PIXELS_PER_2FT / 24.0)
        if start_reverse:
            base_target = (
                p_end[0] + offset_px * math.cos(math.radians(computed_heading)),
                p_end[1] - offset_px * math.sin(math.radians(computed_heading))
            )
        else:
            base_target = (
                p_end[0] - offset_px * math.cos(math.radians(computed_heading)),
                p_end[1] + offset_px * math.sin(math.radians(computed_heading))
            )
    elif node and node.get("wall", False):
        offset_px = offsets["wall_stake_offset_in"] * (PIXELS_PER_2FT / 24.0)
        if start_reverse:
            base_target = (
                p_end[0] + offset_px * math.cos(math.radians(computed_heading)),
                p_end[1] - offset_px * math.sin(math.radians(computed_heading))
            )
        else:
            base_target = (
                p_end[0] - offset_px * math.cos(math.radians(computed_heading)),
                p_end[1] + offset_px * math.sin(math.radians(computed_heading))
            )

    elif node is not None and node_in_corner and node.get("g", False):
        base_target = calc_mogo({"pos": p_end}, computed_heading)
    else:
        base_target = calc_adjustment({"pos": p_end}, computed_heading)

    # Update logical positions.
    logical_start = pixel_to_logical(effective_start)
    logical_end = pixel_to_logical(base_target)

    # Calculate distance (and reverse distance if needed).
    raw_distance = calc_distance(effective_start, base_target)
    phys_distance = raw_distance  # For physics.
    log_distance = raw_distance if not start_reverse else -raw_distance

    face_angle = convert_heading_input(computed_heading, config['plane_mode']['value'])

    if face_angle >= 0:
        alt_angle = face_angle - 360
    else:
        alt_angle = face_angle + 360

    if diff >= 0:
        alt_tangle = diff - 360
    else: 
        alt_tangle = diff + 360

    if from_node_index is not None and to_node_index is not None:
        print(f"\n --- Node {from_node_index} {logical_start} to Node {to_node_index} {logical_end}: ---")
        log_lines.append(f"\n --- Node {from_node_index} {logical_start} to Node {to_node_index} {logical_end}: ---")
    else:
        print(f"\n --- {logical_start} to {logical_end}: ---")
        log_lines.append(f"\n --- {logical_start} to {logical_end}: ---")

    log_lines.append(f"  Turn: {diff:.3f}° or {alt_tangle:.3f}° CCW | Facing: {face_angle:.3f}° or {alt_angle:.3f}°")
    print(f"  Turn: {diff:.3f}° or {alt_tangle:.3f}° CCW | Facing: {face_angle:.3f}° or {alt_angle:.3f}°")

    wheel_circumference = math.pi * robot_physics["diameter"]
    mode = config["pros_mode"]["value"]
    # Read the gear ratio from config.
    gear_ratio = config["gear_ratio"]["value"]
    # Define the base ticks per motor revolution (adjust as needed).
    ticks_per_rev = (gear_ratio / 2) * 100

    if mode == 1:
        # Convert inches to encoder degrees.
        converted = (log_distance / wheel_circumference) * 360 * gear_ratio
        log_lines.append(f"  Distance: {converted:.3f} encoder degrees")
        print(f"  Distance: {converted:.3f} encoder degrees")
    elif mode == 2:
        # Convert inches to encoder rotations.
        converted = (log_distance / wheel_circumference) * gear_ratio
        log_lines.append(f"  Distance: {converted:.3f} encoder rotations")
        print(f"  Distance: {converted:.3f} encoder rotations")
    elif mode == 3:
        # Compute encoder ticks per wheel revolution from gear ratio.
        converted = (log_distance / wheel_circumference) * gear_ratio * ticks_per_rev
        log_lines.append(f"  Distance: {converted:.3f} ticks")
        print(f"  Distance: {converted:.3f} ticks")
    else:
        # Default: output distance in inches.
        log_lines.append(f"  Distance: {log_distance:.3f} inches")
        print(f"  Distance: {log_distance:.3f} inches")


    
    # Now add extra logging for a wall stake node with a custom heading:
    if node and node.get("wall", False) and "wall_heading" in node:
        custom_heading = node["wall_heading"]
        # Compute the extra turn: difference between custom_heading and computed_heading
        extra_diff = custom_heading - computed_heading
        extra_diff = (extra_diff + 180) % 360 - 180  # normalize to [-180, 180]
        
        # Convert the custom heading to a face angle, just as with the computed heading.
        custom_face = convert_heading_input(custom_heading, config["plane_mode"]["value"])
        if custom_face >= 0:
            custom_alt = custom_face - 360
        else:
            custom_alt = custom_face + 360

        # Also compute an alternate extra turn (similar to alt_tangle for diff)
        if extra_diff >= 0:
            alt_extra_diff = extra_diff - 360
        else:
            alt_extra_diff = extra_diff + 360
            
        print(f"  Turn: {extra_diff:.3f}° or {alt_extra_diff:.3f}° CCW to Stake | Facing: {custom_face:.3f}° or {custom_alt:.3f}°")
        log_lines.append(f"  Turn: {extra_diff:.3f}° or {alt_extra_diff:.3f}° CCW to Stake | Facing: {custom_face:.3f}° or {custom_alt:.3f}°")


    # --- Physics-based interpolation for movement ---
    V = ((robot_physics["volts_straight"] / 12) * robot_physics["rpm"] * math.pi * robot_physics["diameter"]) / 60.0  
    a = ((robot_physics["volts_straight"] / 12) * (constant_k / robot_physics["weight"]))
        # robot_physics["volts_straight"] / 12 is unitless and K acts as N = lb * ms^-2
    d_accel = 0.5 * V**2 / a

    if phys_distance < d_accel:
        T_total = math.sqrt(2 * phys_distance / a)
    else:
        T_total = (V / a) + ((phys_distance - d_accel) / V)
    num_frames = max(math.ceil(T_total * fps), 1)

    positions = []
    headings = []
    t_accel = V / a

    for frame in range(num_frames + 1):
        if frame == num_frames:
            positions.append(base_target)
            headings.append(computed_heading)
        else:
            t = frame / fps
            t = min(t, T_total)
            if t < t_accel:
                traveled = 0.5 * a * t**2
            else:
                traveled = d_accel + V * (t - t_accel)
            traveled = min(traveled, phys_distance)
            fraction = traveled / phys_distance if phys_distance != 0 else 1
            x = effective_start[0] + fraction * (base_target[0] - effective_start[0])
            y = effective_start[1] + fraction * (base_target[1] - effective_start[1])
            positions.append((int(x), int(y)))
            headings.append(computed_heading)

    logical_p_end = pixel_to_logical(p_end, rounded=False)
    # Use field centric reference (logical center (3,3)) if enabled; otherwise, use the initial logical position.
    if config["field_centric"]["value"]:
        reference_logical = (3, 3)
    else:
        reference_logical = initial_logical

    odometry_pos = (
        24 * (logical_p_end[0] - reference_logical[0]),
        24 * (logical_p_end[1] - reference_logical[1])
    )

    log_lines.append(f"Odometry Position: ({odometry_pos[1]:.3f}, {odometry_pos[0]:.3f}) inches")
    print(f"Odometry Position: ({odometry_pos[1]:.3f}, {odometry_pos[0]:.3f}) inches (axis: (x,y):(vertical,horizontal))")
    return positions, computed_heading


# --- Misc. Utility Functions ---
def save_nodes(initial_state, nodes):
    filename = filedialog.asksaveasfilename(title="autonroutine", defaultextension=".json")
    if not filename:
        print("Save cancelled.")
        return
    data = {"initial": initial_state, "nodes": nodes}
    try:
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Saved path to {filename}")
    except Exception as e:
        print("Error saving file:", e)

def load_nodes():
    filename = filedialog.askopenfilename(title="Select JSON file", filetypes=[("JSON files", "*.json")])
    if not filename:
        print("No file selected.")
        return None, None
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        print(f"Loaded path from {filename}")
        return data["initial"], data["nodes"]
    except Exception as e:
        print("Error loading file:", e)
        return None, None

def compile_log():
    filename = filedialog.asksaveasfilename(title="autondirect", defaultextension=".txt")
    if not filename:
        print("Save cancelled.")
        return
    try:
        with open(filename, 'w') as f:
            for line in log_lines:
                f.write(line + "\n")
        print(f"Compiled log output to {filename}")
        log_lines.clear()
    except Exception as e:
        print("Error compiling log:", e)

def load_config():
    default_config = {
        "robot_physics": {
            "weight": {
                "value": 12,
                "description": "Robot weight (mass) in lbs"
            },
            "rpm": {
                "value": 400,
                "description": "Drive train RPM at 12V. This is not motor RPM."
            },
            "volts_straight": {
                "value": 9,
                "description": "Voltage #/12 for straight driving (V)"
            },
            "volts_turn": {
                "value": 8,
                "description": "Voltage #/12 for turning (V)"
            },
            "diameter": {
                "value": 3.25,
                "description": "Wheel diameter in inches"
            },
            "t_buffer": {
                "value": 0.2,
                "description": "Buffer time in seconds (stop time in go, stop, turn, stop, go when auton coded go, turn, go)"
            }
        },
        "offsets": {
            "clamp_offset_in": {
                "value": 3,
                "description": "Clamp offset in inches (dist from edge of bot to far edge of mogo inside bot)"
            },
            "ring_node_offset_in": {
                "value": 5,
                "description": "Picking up mogo/ring on field offset in inches; saves distance"
            },
            "wall_stake_offset_in": {
                "value": 15,  
                "description": "Offset for wall stakes in inches when F is pressed a second time."
            },
            "padding_in": {
                "value": 2.5,
                "description": "Added space between closest mogo point and wall when unclamping a mogo into a corner"
            }
        },
        "plane_mode": {
                "value": True,
                "description": "VEX Conventional (Up = 0deg, Right = 90deg) = True, Unit Circle (Up = 90deg, Right = 0deg) = False"
        },
        "field_centric": {
            "value": True,
            "description": "If True, compute odometry relative to center field (logical (3,3)); otherwise relative to robot's initial position."
        },
        "bot_dimensions": {
            "width": {
                "value": 16,
                "description": "Robot width in inches (Distance from left to rightmost point PRONE TO HITTING WALL)"
            },
            "height": {
                "value": 18,
                "description": "Robot length in inches (Distance from back to frontmost point PRONE TO HITTING WALL)"
            },
            "dt_width": {
                "value": 16,
                "description": "Base width; Length from edge to edge, left to right"
            },            
            "dt_height": {
                "value": 15,
                "description": "Base height; Length from edge edge to edge, front to back"
            }
        },
        "dtposition_offset": {
            "x": {
                "value": 0,
                "description": "Horizontal offset (in inches) from the robot's geometric center"
            },
            "y": {
                "value": 0,
                "description": "Vertical offset (in inches) from the robot's geometric center"
            }
        },
        "pros_mode": {
            "value": 0,
            "description": "Display unit: 0 = inches, 1 = encoder degrees, 2 = encoder rotations, 3 = encoder counts/ticks"
        },
        "gear_ratio": {
            "value": 18,
            "description": "11W V5 Smart Motor gear ratio. (18:1 gears = 18, etc.)"
        }
    }
    if not os.path.exists(CONFIG_FILENAME):
        with open(CONFIG_FILENAME, 'w') as f:
            json.dump(default_config, f, indent=4)
        return default_config
    else:
        with open(CONFIG_FILENAME, 'r') as f:
            return json.load(f)

def save_state():
    global undo_stack, redo_stack, nodes
    # Save a deep copy of nodes to the undo stack.
    undo_stack.append(copy.deepcopy(nodes))
    # Any new action clears the redo stack.
    redo_stack.clear()

def undo():
    global nodes, undo_stack, redo_stack
    if undo_stack:
        redo_stack.append(copy.deepcopy(nodes))
        nodes = undo_stack.pop()
        print("Undo performed.")

def redo():
    global nodes, undo_stack, redo_stack
    if redo_stack:
        undo_stack.append(copy.deepcopy(nodes))
        nodes = redo_stack.pop()
        print("Redo performed.")


# --- Coord. Plane & Orientation Mode Functions ---

def convert_heading_input(heading, conventional_mode):
    """Convert inputted heading based on the current orientation mode."""
    if conventional_mode:
            return (90 - heading) % 360  # Convert planes
        
    return heading

# <<---------------------------------------------------------------------------->>

# --- Robot Initial State ---
initial_state = {
    "position": (WINDOW_WIDTH // 2, WINDOW_HEIGHT // 2),  # center --> (3,3)
    "heading": 0  # in degrees; 0 means pointing right (unit circle)
}

robot_pos = initial_state["position"]
robot_heading = initial_state["heading"]
config = load_config()
robot_physics = config["robot_physics"]
bot_dimensions = config["bot_dimensions"]

# --- Data Structures & Dictionaries ---
nodes = [{"pos": robot_pos, "goal": False, "bezier": False}]
robot_physics = { key: config["robot_physics"][key]["value"] for key in config["robot_physics"] }
offsets = { key: config["offsets"][key]["value"] for key in config["offsets"] }
plane_mode = config["plane_mode"]["value"]
bot_dimensions = { key: config["bot_dimensions"][key]["value"] for key in config["bot_dimensions"] }

# --- Mode and Log Variables ---
snap_enabled = True      
selected_node_index = None 
log_lines = [] # Buffer for direction output
initial_logical = pixel_to_logical(initial_state["position"])
odometry_pos = []
circle_color = NODE_COLOR

# --- Main Loop Variables ---
moving = False     
move_positions = []         # Interpolated positions for current segment
move_index = 0
node_index = 0              # Current segment: nodes[node_index] -> nodes[node_index+1]
place_ring_goal = False
place_wall_stake = False
selected_node_index = None
manual_heading_set = False
target_node = None
place_bezier = False

# <<---------------------------------------------------------------------------->>

# --- Main Loop ---
def main():
    global robot_pos, robot_heading, moving, move_positions, move_index, node_index, manual_heading_set, initial_logical, log_lines, place_ring_goal, reverse_mode, bot_dimensions, constant_k
    global place_bezier, snap_enabled, selected_node_index, nodes, initial_state, dragging_node, drag_node_index, drag_start_pos, config, robot_physics, current_segment_g, place_wall_stake

    # Only compute a default heading if none was manually set.
    if len(nodes) > 1 and robot_heading == 0 and not manual_heading_set:
        _, default_heading = calc_angle(robot_heading, nodes[0]["pos"], nodes[1]["pos"])
        robot_heading = default_heading
        initial_state["heading"] = robot_heading
        print(f"Default heading set to {robot_heading:.2f}° toward first node.")

    while True:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # Key events
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    snap_enabled = not snap_enabled
                    print(f"Snap to grid is now {'ON' if snap_enabled else 'OFF'}")

                if event.key == pygame.K_h:
                    
                    new_heading = simpledialog.askfloat("Set Heading", "Face Heading (deg):")
                    if new_heading is not None:
                        config = load_config()
                        new_heading = convert_heading_input(new_heading, config["plane_mode"]["value"])
                        robot_heading = new_heading % 360
                        initial_state["heading"] = robot_heading
                        manual_heading_set = True
                        print(f"Set Initial Robot heading")

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if moving:
                            paused = not paused  # Toggle pause
                            print("Paused" if paused else "Resumed")
                        elif len(nodes) > 1:
                            log_lines.clear()
                            log_lines.append(f"Odometry axis key:(x,y):(vertical,horizontal)")
                            log_lines.append(f"Modes: Field Centric: {config['field_centric']['value']} | Conventional Orientation (0 Up, 270 Left): {config['plane_mode']['value']}")
                            log_lines.append(f"Initial Heading: {convert_heading_input(initial_state['heading'], config['plane_mode']['value'])}°")

                            moving = True
                            paused = False
                            robot_pos = nodes[0]["pos"]
                            # For the first segment (from node 0 to node 1), use segment index 0.
                            start_reverse = segment_rstatus(nodes, 0)
                            move_positions, robot_heading = move_robot_along(
                                nodes[0]["pos"],
                                nodes[1]["pos"],
                                robot_heading,
                                node=nodes[1],
                                start_reverse=start_reverse,
                                from_node_index=0,
                                to_node_index=1
                            )
                            move_index = 0
                            node_index = 1
          
                if event.key == pygame.K_SPACE and pygame.key.get_mods() & pygame.KMOD_CTRL:
                    moving = False
                    paused = False
                    robot_pos = nodes[0]["pos"]
                    node_index = 0
                    current_segment_g = False

                    config = load_config()

                    # Clear any left behind borders from previous runs
                    for node in nodes:
                        if "left_mogo" in node:
                            del node["left_mogo"]
                        if "left_mogo_pos" in node:
                            del node["left_mogo_pos"]
                        if "left_mogo_heading" in node:
                            del node["left_mogo_heading"]

                    print("Animation stopped and reset to node 0")

                if event.key == pygame.K_f:
                    if selected_node_index is not None:
                        save_state()
                        node = nodes[selected_node_index]
                        # Toggle: if neither ring nor wall are set, set ring.
                        # If ring is set, then switch to wall.
                        # If wall is set, clear both.
                        if not node.get("ring", False) and not node.get("wall", False):
                            node["ring"] = True
                            node["wall"] = False
                            print(f"Node {selected_node_index} set to Goal/Ring mode.")
                        elif node.get("ring", False):
                            node["ring"] = False
                            node["wall"] = True
                            print(f"Node {selected_node_index} toggled to Wall Stake mode.")
                        elif node.get("wall", False):
                            node["ring"] = False
                            node["wall"] = False
                            print(f"Node {selected_node_index} cleared of Goal/W/Stk mode.")
                    else:
                        # If no node is selected, fallback to setting flags for the next placed node.
                        if place_ring_goal:
                            place_wall_stake = True
                            place_ring_goal = False
                            print("Next placed node will be a Wall Stake node.")
                        else:
                            place_ring_goal = True
                            print("Next placed node will be a Goal/Ring node.")

                # For reverse nodes (red):
                if event.key == pygame.K_r:
                    if selected_node_index is not None:
                        save_state()
                        node = nodes[selected_node_index]
                        node["reverse"] = not node.get("reverse", False)
                        print(f"Reverse turned {'ON' if node['reverse'] else 'OFF'} for node {selected_node_index}")
                    else:
                        # Fallback to toggling the last node if none is selected.
                        if len(nodes) > 0:
                            save_state()
                            index = len(nodes) - 1
                            node = nodes[index]
                            node["reverse"] = not node.get("reverse", False)
                            print(f"Reverse turned {'ON' if node['reverse'] else 'OFF'} for node {index}")
                        else:
                            print("No node available to toggle reverse.")
                                
                if event.key == pygame.K_g:
                    # Use the selected node if available; otherwise, use the last node
                    index = selected_node_index if selected_node_index is not None else len(nodes) - 1
                    
                    if index == 0:
                        print("Node 0 cannot have clamp toggle (G is always off for node 0).")
                    else:
                        save_state()
                        new_state = not nodes[index].get("g", False)
                        nodes[index]["g"] = new_state
                        print(f"Clamped Mech Toggle for node {index}")

                if event.key == pygame.K_b:
                    if selected_node_index is not None:
                        # Toggle bezier flag on selected node
                        save_state()
                        current = nodes[selected_node_index].get("bezier", False)
                        nodes[selected_node_index]["bezier"] = not current
                        print(f"Node {selected_node_index} bezier flag set to {nodes[selected_node_index]['bezier']}")
                    else:
                        place_bezier = True
                        print("Next placed node will be a bezier node.")
                
                if event.key == pygame.K_s:
                    save_nodes(initial_state, nodes)
                
                if event.key == pygame.K_l:
                    loaded_initial, loaded_nodes = load_nodes()
                    if loaded_initial and loaded_nodes:
                        initial_state = loaded_initial
                        nodes = loaded_nodes
                        robot_pos = nodes[0]["pos"]
                        robot_heading = initial_state["heading"]
                        initial_logical = pixel_to_logical(initial_state["position"])
                        manual_heading_set = False  # allow a new heading to override the loaded one
                        print("Path loaded. Robot state reset.")
                
                if event.key == pygame.K_c:
                    compile_log()
                
                if event.key == pygame.K_p:
                    print("""
                    Controls:
                    Q               - Toggle Snapless Grid
                    RClick/Delete   - Delete Node
                    S/L             - Save/Load Routine
                    H               - Set Initial Robot Heading (Unit Circle)
                    C               - Compile Directions + Coordinates After Running
                    SPACE           - Start/Pause Animation
                    CTRL + SPACE    - Stop Animation & Reset
                    V               - ESTIMATE AUTON TIME
                    B               - Curved Pathing (Brakeless/Coast)
                    F               - Bot Center Drives to an offset before node
                                        press a second time for wall stakes.
                    R               - Apply REVERSE to most recent node
                    G               - Toggle CLAMP to most recent mode
                    """)
                
                if event.key == pygame.K_v:
                    config = load_config()
                    robot_physics = { key: config["robot_physics"][key]["value"] for key in config["robot_physics"] }
                    bot_dimensions = { key: config["bot_dimensions"][key]["value"] for key in config["bot_dimensions"] }
                    
                    rpm = robot_physics["rpm"]
                    diameter = robot_physics["diameter"]
                    # If config stores voltage values, adjust them as needed
                    volts_straight = robot_physics["volts_straight"] / 12.0
                    volts_turn = robot_physics["volts_turn"] / 12.0
                    weight = robot_physics["weight"]
                    t_buffer = robot_physics["t_buffer"]
                    print("Using physics parameters from config:")
                    print(robot_physics)

                    # Scale RPM linearly with voltage
                    effective_rpm_straight = volts_straight * rpm
                    effective_rpm_turn = volts_turn * rpm
                    V_straight = ((effective_rpm_straight * math.pi * diameter) / 60.0)
                    V_turn =     ((effective_rpm_turn * math.pi * diameter) / 60.0)

                    # Compute effective acceleration (in inches/sec^2)
                    # Using a constant so that for a 100 lb robot, a ~20 in/s^2 at 12V.
                    constant_k = constant_k
                    a_straight = volts_straight * (constant_k / weight)
                    a_turn = volts_turn * (constant_k / weight)
                    d_accel_turn = 0.5 * (V_turn ** 2) / a_turn if a_turn != 0 else pivot_arc_length

                    total_travel_time = 0.0
                    current_heading = initial_state["heading"]
                    for i in range(len(nodes)-1):
                        p_start = nodes[i]["pos"]
                        p_end = nodes[i+1]["pos"]
                        distance = calc_distance(p_start, p_end)
                        # Compute straight travel time (starting from 0 each segment)
                        t_straight = compute_segment_time(distance, V_straight, a_straight)
                        diff, desired_heading = calc_angle(current_heading, p_start, p_end)
                    
                        if nodes[i+1].get("heading_flip", False):
                            diff = abs(diff) + 180  # now diff is in degrees; ensure it's positive.
                        else:
                            diff = abs(diff)
                                
                        # Compute pivot arc length (arc length = radius * angle in radians)
                        pivot_angle_rad = diff * math.pi / 180.0
                        pivot_arc_length = (bot_dimensions["dt_width"] / 2) * pivot_angle_rad

                        if pivot_arc_length < d_accel_turn:
                            t_pivot = math.sqrt(2 * pivot_arc_length / a_turn)  # If pivot is too short to reach full speed
                        else:
                            t_pivot = (V_turn / a_turn) + ((pivot_arc_length - d_accel_turn) / V_turn)

                        if nodes[i + 1].get("bezier", False):
                            t_pivot = 0  # No stop, just a natural turn
                        else:
                            t_pivot = compute_segment_time(9 * pivot_angle_rad, V_turn, a_turn)

                        total_travel_time += t_straight + t_pivot + (t_buffer * 2) # (Buffer before and after move)
                        if nodes[i].get("bezier") == True:
                            total_travel_time -= (t_pivot + (t_buffer * 2))
                        # drifts instead of stops to turn, so no stop time added to total                        

                        current_heading = desired_heading
                        
                    print(f"Calculated Straight Velocity: {(V_straight/12):.2f} ft/s")
                    print(f"Calculated Turning Velocity: {(V_turn/12):.2f} ft/s")
                    print(f"Estimated Total Time (including acceleration and pivots): {total_travel_time:.2f} seconds")

                if event.key == pygame.K_DELETE and selected_node_index is not None:
                    if selected_node_index != 0:
                        if nodes[selected_node_index].get("heading_flip", False):
                            reverse_mode = False
                            print("Deleted red node. Reverse mode turned OFF.")
                        save_state()
                        print(f"Deleting node {selected_node_index}")
                        del nodes[selected_node_index]
                    selected_node_index = None

                if event.type == pygame.KEYDOWN:
                    # Check for Undo (Ctrl+Z)
                    if event.mod & pygame.KMOD_CTRL and event.key == pygame.K_z:
                        undo()
                        continue  # Skip other key processing

                    # Check for Redo (Ctrl+Y)
                    if event.mod & pygame.KMOD_CTRL and event.key == pygame.K_y:
                        redo()
                        continue

            # Mouse events (only when not moving)
            if event.type == pygame.MOUSEBUTTONDOWN and not moving:
                mouse_pos = pygame.mouse.get_pos()
                if event.button == 1:  # Left-click: select node for dragging if near one
                    found = False
                    for i, node in enumerate(nodes):
                        if calc_distance(mouse_pos, node["pos"]) <= 4:
                            selected_node_index = i
                            found = True
                            dragging_node = True
                            drag_node_index = i
                            drag_start_pos = node["pos"]
                            save_state()
                            break

                    if not found or selected_node_index is None:
                        selected_node_index = None
                        save_state()
                        node_pos = snap_to_grid(mouse_pos, snap_enabled)
                        node = {
                            "pos": node_pos,
                            "ring": place_ring_goal,
                            "wall": place_wall_stake,
                            "bezier": place_bezier,
                            "heading_flip": False
                        }
                        nodes.append(node)
                        print(f"Added node: {node_pos}, ring = {place_ring_goal}, bezier = {place_bezier}")
                        
                        # If this is a wall stake node, prompt the user for a heading change.
                        if place_wall_stake:
                            new_heading = simpledialog.askfloat("Wall Stake Node",
                                                                "Enter desired heading (deg) for wall stake node,\nor Cancel to keep automatic:")
                            if new_heading is not None:
                                # Store the new heading in the node (converted as needed)
                                node["wall_heading"] = convert_heading_input(new_heading, config["plane_mode"]["value"])
                                # Mark that the drive distance should be 0 at this node.
                                node["drive_distance"] = 0
                                print(f"Wall stake node will turn to heading {node['wall_heading']:.2f}° with 0 inches drive.")
                            else:
                                print("No custom heading set; using default drive behavior.")

                        place_ring_goal = False
                        place_wall_stake = False
                        place_bezier = False

                elif event.button == 3:  # Right-click: delete node if near one (except initial)
                    for i, node in enumerate(nodes):
                        if calc_distance(mouse_pos, node["pos"]) <= 4:
                            if i != 0:
                                save_state()
                                print(f"Right-click deletion: Removing node {i}")
                                del nodes[i]
                            break

            if event.type == pygame.MOUSEBUTTONUP and not moving:
                print("Node Registered")
                if selected_node_index == 0:
                    print(f"Initial position updated to: {initial_state['position']}")
                    initial_logical = pixel_to_logical(initial_state["position"])
                selected_node_index = None

            if event.type == pygame.MOUSEMOTION and not moving:
                if selected_node_index is not None:
                    new_pos = snap_to_grid(event.pos, snap_enabled)
                    nodes[selected_node_index]["pos"] = new_pos
                    if selected_node_index == 0:
                        initial_state["position"] = new_pos 

        # Drawing
        screen.fill(BG_COLOR)
        draw_grid(screen)
        draw_nodes(screen, nodes)
        if robot_pos:
            draw_robot(screen, robot_pos)
            if moving:
                if current_segment_g:
                    draw_mogo_border(screen, robot_pos, robot_heading)
                draw_chevron_indicator(screen, robot_pos, robot_heading)
            else:
                # In edit mode, always display the mogo.
                draw_mogo_border(screen, robot_pos, robot_heading)
        pygame.display.flip()

        # Animate robot movement along nodes if moving
        if moving and not paused:
            if move_index >= len(move_positions):
                if node_index < len(nodes) - 1:
                    p_start = robot_pos  # current position
                    # Use the aggregated g state, similar to how reverse is handled:
                    current_segment_g = segment_gstatus(nodes, node_index)
                    target_node = nodes[node_index+1]
                    p_end = target_node["pos"]

                    start_reverse = segment_rstatus(nodes, node_index)
                    move_positions, robot_heading = move_robot_along(
                        p_start, p_end, robot_heading, node=target_node, start_reverse=start_reverse, from_node_index=node_index, to_node_index=node_index+1
                    )
                    node_index += 1
                    move_index = 0

                else:
                    moving = False
                    print("Finished following all nodes.")
                    print("Final logical position:", pixel_to_logical(robot_pos))
                    print("Press C to compile directions.")

            if moving and move_positions:
                robot_pos = move_positions[move_index]
                # Print "Robot at:" for debugging, but these are not logged for compilation.
                # print("Robot at:", pixel_to_logical(robot_pos))
                move_index += 1

if __name__ == '__main__':
    # Run from terminal to see output
    main()
