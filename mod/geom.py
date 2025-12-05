# mod/geom.py
from __future__ import annotations

def convert_heading_input(heading_deg, _conventional_mode=None):
    """
    Convert internal (standard math frame, 0=right, CCW positive) to display
    convention (0=left, 90=up, 180=right, 270=down).
    """
    return (180.0 - float(heading_deg)) % 360.0
