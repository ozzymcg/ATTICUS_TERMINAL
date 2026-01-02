"""
Path utilities and path-file export for curved segment generation.
"""

from __future__ import annotations

import math
import os
from typing import List, Tuple, Optional

# Support running without package context
if __package__ is None or __package__ == "":
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(here)
    sys.path.append(os.path.dirname(here))
    from geom import field_coords_in, convert_heading_input  # type: ignore
else:
    from .geom import field_coords_in, convert_heading_input


def catmull_rom_spline(points: List[Tuple[float, float]], num_samples: int = 50) -> List[Tuple[float, float]]:
    """
    Generate a smooth Catmull-Rom spline through the control points.
    This creates a curve that passes through all control points naturally.
    """
    if len(points) < 2:
        return points
    if len(points) == 2:
        return [points[0], points[1]]

    # Add phantom points at start/end for natural curve endpoints
    extended = [points[0]] + points + [points[-1]]

    result = []
    for i in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[i - 1], extended[i], extended[i + 1], extended[i + 2]

        # Sample along this segment using Catmull-Rom formula
        for j in range(num_samples):
            t = j / num_samples
            t2 = t * t
            t3 = t2 * t

            # Catmull-Rom basis functions
            x = 0.5 * ((2 * p1[0]) +
                      (-p0[0] + p2[0]) * t +
                      (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 +
                      (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3)

            y = 0.5 * ((2 * p1[1]) +
                      (-p0[1] + p2[1]) * t +
                      (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 +
                      (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)

            result.append((x, y))

    result.append(points[-1])
    return result


def generate_bezier_path(control_points: List[Tuple[float, float]], num_samples: int = 50) -> List[Tuple[float, float]]:
    """
    Generate smooth path through control points using Catmull-Rom spline.
    This ensures the path passes through all control points naturally.
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
        dx = path_points[i][0] - path_points[i - 1][0]
        dy = path_points[i][1] - path_points[i - 1][1]
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


def _vmax_straight(cfg: dict) -> float:
    """Local wrapper to avoid import cycles with sim."""
    if __package__ is None or __package__ == "":
        from sim import vmax_straight  # type: ignore
    else:
        from .sim import vmax_straight
    return vmax_straight(cfg)


def export_lemlib_path(path_points: List[Tuple[float, float]],
                      filename: str,
                      cfg: dict,
                      initial_velocity: float = 60.0,
                      end_velocity: float = 0.0,
                      velocities: Optional[List[float]] = None,
                      desired_min_speed: Optional[float] = None,
                      desired_max_speed: Optional[float] = None) -> str:
    """
    Export path points to LemLib/JerryIO-compatible .txt format.

    Format per line: configurable via codegen.path_columns (default "X, Y, COMMAND")
    - x, y: field coordinates in inches
    - velocity_cmd: speed command (0-127) scaled from ips using vmax_straight
    - desired_min_speed/desired_max_speed: clamp in command units (0-127)
    """
    # Create export directory (user-selectable; default relative to project root)
    export_dir = cfg.get("codegen", {}).get("path_dir", "export/paths")
    export_dir = os.path.expanduser(str(export_dir or "export/paths"))
    if not os.path.isabs(export_dir):
        base_root = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))
        export_dir = os.path.join(base_root, export_dir)
    os.makedirs(export_dir, exist_ok=True)

    filepath = os.path.join(export_dir, filename)

    # Vex/PROS command range conversion (ips -> 0-127 cmd units)
    max_cmd = float(cfg.get("robot_physics", {}).get("max_cmd", 127.0))
    vmax_ref = max(1e-6, _vmax_straight(cfg))

    def _ips_to_cmd(v_ips: float) -> float:
        return max(0.0, min(max_cmd, v_ips / vmax_ref * max_cmd))

    min_cmd = None if desired_min_speed is None else max(0.0, min(max_cmd, float(desired_min_speed)))
    max_cmd_cap = None if desired_max_speed is None else max(0.0, min(max_cmd, float(desired_max_speed)))
    if min_cmd is not None and max_cmd_cap is not None:
        min_cmd = min(min_cmd, max_cmd_cap)

    def _cap_cmd(v_cmd: float) -> float:
        if min_cmd is not None:
            v_cmd = max(min_cmd, v_cmd)
        if max_cmd_cap is not None:
            v_cmd = min(max_cmd_cap, v_cmd)
        return v_cmd

    fmt_default = "{X}, {Y}, {COMMAND}"
    fmt_raw = cfg.get("codegen", {}).get("path_columns", fmt_default)
    if isinstance(fmt_raw, dict):
        fmt_raw = fmt_raw.get("value", fmt_default)
    fmt = str(fmt_raw or fmt_default)
    if "{X" not in fmt or "{Y" not in fmt:
        fmt = fmt_default

    lines = []
    vel_list: Optional[List[float]] = [float(v) for v in velocities] if velocities else None
    for i, point in enumerate(path_points):
        # Convert to field inches
        x_in, y_in = field_coords_in(point)
        heading_disp = None
        if "{HEADING" in fmt:
            try:
                heading_internal = calculate_path_heading(path_points, i)
                heading_disp = convert_heading_input(heading_internal, None)
            except Exception:
                heading_disp = 0.0

        # Per-point velocity (use provided speeds if available), convert to cmd units
        if vel_list and i < len(vel_list):
            v_cmd = _ips_to_cmd(float(vel_list[i]))
        elif len(path_points) > 1:
            v_ips = initial_velocity + (end_velocity - initial_velocity) * (i / (len(path_points) - 1))
            v_cmd = _ips_to_cmd(v_ips)
        else:
            v_cmd = _ips_to_cmd(initial_velocity)
        v_cmd = _cap_cmd(v_cmd)

        # Customizable format
        tokens = {
            "X": x_in,
            "Y": y_in,
            "COMMAND": v_cmd,
            "HEADING": heading_disp if heading_disp is not None else 0.0,
            "HEADING_DEG": heading_disp if heading_disp is not None else 0.0,
        }
        try:
            lines.append(fmt.format(**tokens))
        except Exception:
            lines.append(f"{x_in:.3f}, {y_in:.3f}, {v_cmd:.3f}")

    # Write to file
    with open(filepath, "w") as f:
        f.write("\n".join(lines))

    print(f"Exported path to: {filepath}")
    return filepath


def generate_path_asset_name(routine_name: str, segment_index: int) -> str:
    """
    Generate a consistent asset name for a path segment.
    """
    safe_name = "".join(c if c.isalnum() else "_" for c in routine_name)
    return f"{safe_name}_seg{segment_index}_path.txt"
