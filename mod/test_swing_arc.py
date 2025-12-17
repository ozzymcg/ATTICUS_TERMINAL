# Simple swing-arc sanity checks to guard cw/ccw and reverse handling.
import math
from .sim import _swing_segment, heading_from_points
from .config import PPI


_CFG = {
    "bot_dimensions": {"dt_width": 11.5, "width": 14.5},
    "robot_physics": {
        "volts_straight": 12.0,
        "rpm": 200.0,
        "diameter": 4.0,
        "mu": 0.9,
        "weight": 20.0,
        "max_cmd": 127.0,
        "volts_turn": 12.0,
    },
    "path_config": {"max_speed_ips": 127.0, "min_speed_ips": 30.0, "curvature_gain": 0.05},
}


def _ang_diff(a, b):
    d = ((a - b + 180.0) % 360.0) - 180.0
    return d


def _check_align(p0, p1, start_h, dirn, reverse):
    res = _swing_segment(p0, p1, start_h, dirn, _CFG, reverse=reverse)
    assert res is not None, "swing arc should be generated"
    seg, end_pos, target, _ = res
    desired = heading_from_points(end_pos, p1)
    if reverse:
        desired = (desired + 180.0) % 360.0
    err = _ang_diff(target, desired)
    assert abs(err) < 1e-3, f"heading misaligned: err={err}"
    if dirn.lower() == "cw":
        assert seg["delta_heading"] <= 0, "cw swing must rotate clockwise"
    if dirn.lower() == "ccw":
        assert seg["delta_heading"] >= 0, "ccw swing must rotate counter-clockwise"


def run():
    # Forward cw swing toward node above should lean into arc and align heading.
    _check_align((0.0, 0.0), (0.0, -24 * PPI), 270.0, "cw", False)
    # Reverse cw swing toward node to the right should still rotate clockwise.
    _check_align((0.0, 0.0), (24 * PPI, 0.0), 270.0, "cw", True)
    # Reverse ccw swing toward node below should rotate ccw.
    _check_align((0.0, 0.0), (0.0, 24 * PPI), 0.0, "ccw", True)


if __name__ == "__main__":
    run()
