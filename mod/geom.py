# mod/geom.py
from __future__ import annotations

def convert_heading_input(heading_deg, conventional_mode):
    """Convert between unit circle and VEX heading conventions.
    
    conventional_mode=0: Unit circle (Right=0°, Up=90°)
    conventional_mode=1: VEX standard (Up=0°, Right=90°)
    """
    return (90 - heading_deg) % 360 if conventional_mode else heading_deg
