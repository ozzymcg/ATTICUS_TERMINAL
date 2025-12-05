# mod/path_export.py
"""
Path file export for LemLib/JerryIO-style integration.
Generates .txt path files compatible with LemLib's pure pursuit system.
"""

import os
from typing import List, Tuple, Optional
from .config import PPI, WINDOW_WIDTH, WINDOW_HEIGHT
from .sim import vmax_straight


def field_coords_in(p: Tuple[float, float]) -> Tuple[float, float]:
    """
    Convert pixel position to field-centric inches.
    Field origin is at center, X = forward (up), Y = left.
    """
    cx, cy = WINDOW_WIDTH / 2.0, WINDOW_HEIGHT / 2.0
    x_forward_in = -(p[1] - cy) / PPI
    y_left_in = -(p[0] - cx) / PPI
    return x_forward_in, y_left_in


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
    
    Format per line: x_in, y_in, velocity_cmd
    - x, y: field coordinates in inches
    - velocity_cmd: speed command (0–127) scaled from ips using vmax_straight
    
    Args:
        path_points: List of (x, y) pixel coordinates
        filename: Output filename (e.g., "autonomous_seg0_path.txt")
        cfg: Configuration dictionary
        initial_velocity: Starting velocity in in/s
        end_velocity: Ending velocity in in/s
    
    Returns:
        Full path to the created file
    """
    # Create export directory (user-selectable; default relative to project root)
    export_dir = cfg.get("codegen", {}).get("path_dir", "export/paths")
    export_dir = os.path.expanduser(str(export_dir or "export/paths"))
    if not os.path.isabs(export_dir):
        base_root = os.path.normpath(os.path.join(os.path.dirname(__file__), os.pardir))
        export_dir = os.path.join(base_root, export_dir)
    os.makedirs(export_dir, exist_ok=True)
    
    filepath = os.path.join(export_dir, filename)
    
    def _cap_speed(v: float) -> float:
        if desired_min_speed is not None:
            v = max(desired_min_speed, v)
        if desired_max_speed is not None:
            v = min(desired_max_speed, v)
        return v

    # Vex/PROS command range conversion (ips -> 0-127 cmd units)
    max_cmd = float(cfg.get("robot_physics", {}).get("max_cmd", 127.0))
    vmax_ref = max(1e-6, vmax_straight(cfg))
    def _ips_to_cmd(v_ips: float) -> float:
        return max(0.0, min(max_cmd, v_ips / vmax_ref * max_cmd))

    lines = []
    capped_vels: Optional[List[float]] = None
    if velocities:
        # Mirror simulation speeds but cap to requested range when provided
        capped_vels = [_cap_speed(float(v)) for v in velocities]
    for i, point in enumerate(path_points):
        # Convert to field inches
        x_in, y_in = field_coords_in(point)

        # Per-point velocity (use provided speeds if available), convert to cmd units
        if capped_vels and i < len(capped_vels):
            v_cmd = _ips_to_cmd(float(capped_vels[i]))
        elif len(path_points) > 1:
            v_ips = initial_velocity + (end_velocity - initial_velocity) * (i / (len(path_points) - 1))
            v_ips = _cap_speed(v_ips)
            v_cmd = _ips_to_cmd(v_ips)
        else:
            v_cmd = _ips_to_cmd(_cap_speed(initial_velocity))

        # JerryIO-style format: x, y, command velocity
        lines.append(f"{x_in:.3f}, {y_in:.3f}, {v_cmd:.3f}")
    
    # Write to file
    with open(filepath, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Exported path to: {filepath}")
    return filepath


def generate_path_asset_name(routine_name: str, segment_index: int) -> str:
    """
    Generate a consistent asset name for a path segment.
    
    Args:
        routine_name: Name of the autonomous routine
        segment_index: Index of the segment (node index)
    
    Returns:
        Sanitized filename like "autonomous_seg0_path.txt"
    """
    # Sanitize routine name for filesystem and C++ identifiers
    safe_name = ''.join(c if c.isalnum() else '_' for c in routine_name)
    return f"{safe_name}_seg{segment_index}_path.txt"
