# mod/path_utils.py
"""
Path utilities for curved segment generation and pure pursuit simulation.
Handles Catmull-Rom spline generation, Bezier curves, and path following math.
"""

import math
from typing import List, Tuple


def catmull_rom_spline(points: List[Tuple[float, float]], num_samples: int = 50) -> List[Tuple[float, float]]:
    """
    Generate a smooth Catmull-Rom spline through the control points.
    This creates a curve that passes through all control points naturally.
    
    Args:
        points: List of (x, y) control points
        num_samples: Number of interpolation samples per segment
    
    Returns:
        List of interpolated (x, y) points forming the smooth curve
    """
    if len(points) < 2:
        return points
    if len(points) == 2:
        return [points[0], points[1]]
    
    # Add phantom points at start/end for natural curve endpoints
    extended = [points[0]] + points + [points[-1]]
    
    result = []
    for i in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[i-1], extended[i], extended[i+1], extended[i+2]
        
        # Sample along this segment using Catmull-Rom formula
        for j in range(num_samples):
            t = j / num_samples
            t2 = t * t
            t3 = t2 * t
            
            # Catmull-Rom basis functions
            x = 0.5 * ((2 * p1[0]) +
                      (-p0[0] + p2[0]) * t +
                      (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                      (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)
            
            y = 0.5 * ((2 * p1[1]) +
                      (-p0[1] + p2[1]) * t +
                      (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                      (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
            
            result.append((x, y))
    
    result.append(points[-1])
    return result


def bezier_cubic(p0, p1, p2, p3, t):
    """
    Cubic Bezier interpolation between 4 control points.
    p0 and p3 are endpoints, p1 and p2 are control handles.
    """
    mt = 1 - t
    mt2 = mt * mt
    mt3 = mt2 * mt
    t2 = t * t
    t3 = t2 * t
    
    x = mt3 * p0[0] + 3 * mt2 * t * p1[0] + 3 * mt * t2 * p2[0] + t3 * p3[0]
    y = mt3 * p0[1] + 3 * mt2 * t * p1[1] + 3 * mt * t2 * p2[1] + t3 * p3[1]
    return (x, y)


def generate_bezier_path(control_points: List[Tuple[float, float]], num_samples: int = 50) -> List[Tuple[float, float]]:
    """
    Generate smooth path through control points using Catmull-Rom spline.
    This ensures the path passes through all control points naturally.
    
    Args:
        control_points: List of (x, y) points to interpolate
        num_samples: Interpolation density
    
    Returns:
        List of smooth path points
    """
    if len(control_points) < 2:
        return control_points
    
    if len(control_points) == 2:
        # Simple straight line
        return [control_points[0], control_points[1]]
    
    # Use Catmull-Rom for natural smooth curves through all points
    return catmull_rom_spline(control_points, num_samples)

def resample_path_uniform(path_points: List[Tuple[float, float]], spacing_px: float) -> List[Tuple[float, float]]:
    """Resample a polyline to uniform arc-length spacing."""
    if not path_points or len(path_points) < 2 or spacing_px <= 0:
        return path_points
    # Compute cumulative distances
    cum = [0.0]
    for i in range(1, len(path_points)):
        dx = path_points[i][0] - path_points[i-1][0]
        dy = path_points[i][1] - path_points[i-1][1]
        cum.append(cum[-1] + math.hypot(dx, dy))
    total = cum[-1]
    if total <= spacing_px:
        return path_points
    resampled = [path_points[0]]
    target = spacing_px
    idx = 1
    while target < total and idx < len(path_points):
        while idx < len(path_points) and cum[idx] < target:
            idx += 1
        if idx >= len(path_points):
            break
        prev_idx = idx - 1
        seg_len = cum[idx] - cum[prev_idx]
        if seg_len <= 1e-9:
            frac = 0.0
        else:
            frac = (target - cum[prev_idx]) / seg_len
        x = path_points[prev_idx][0] + frac * (path_points[idx][0] - path_points[prev_idx][0])
        y = path_points[prev_idx][1] + frac * (path_points[idx][1] - path_points[prev_idx][1])
        resampled.append((x, y))
        target += spacing_px
    if resampled[-1] != path_points[-1]:
        resampled.append(path_points[-1])
    return resampled


def calculate_path_heading(points: List[Tuple[float, float]], index: int) -> float:
    """
    Calculate heading (in degrees, math frame CCW+) at a point on the path.
    Uses forward difference for tangent direction.
    
    Args:
        points: Path points
        index: Index of point to calculate heading for
    
    Returns:
        Heading in degrees (0-360, math frame)
    """
    if index < len(points) - 1:
        p1 = points[index]
        p2 = points[index + 1]
    elif index > 0:
        p1 = points[index - 1]
        p2 = points[index]
    else:
        return 0.0
    
    dx = p2[0] - p1[0]
    dy = -(p2[1] - p1[1])  # Invert Y for screen coordinates
    return (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0


def pure_pursuit_lookahead(robot_pos: Tuple[float, float], 
                           path_points: List[Tuple[float, float]], 
                           current_index: int,
                           lookahead_dist: float) -> Tuple[int, Tuple[float, float]]:
    """
    Find the lookahead point on the path for pure pursuit control.
    
    Args:
        robot_pos: Current robot (x, y) position
        path_points: List of path points
        current_index: Current position on path
        lookahead_dist: Lookahead distance in pixels
    
    Returns:
        (new_index, lookahead_point) tuple
    """
    for i in range(current_index, len(path_points)):
        dist = math.hypot(path_points[i][0] - robot_pos[0], 
                         path_points[i][1] - robot_pos[1])
        if dist >= lookahead_dist:
            return i, path_points[i]
    
    # Return last point if no point beyond lookahead distance
    return len(path_points) - 1, path_points[-1]


def closest_point_on_path(robot_pos: Tuple[float, float], 
                          path_points: List[Tuple[float, float]]) -> int:
    """
    Find the index of the closest point on the path to the robot.
    
    Args:
        robot_pos: Current robot (x, y) position
        path_points: List of path points
    
    Returns:
        Index of closest point
    """
    min_dist = float('inf')
    closest_idx = 0
    
    for i, point in enumerate(path_points):
        dist = math.hypot(point[0] - robot_pos[0], point[1] - robot_pos[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    return closest_idx
