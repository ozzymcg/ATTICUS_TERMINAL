"""Atticus Efficiency Pack code generation."""

from __future__ import annotations

import hashlib
import json
import os
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .atticus_export_validation import (
    ValidationIssue,
    validate_efficiency_pack_timeline,
)
from .config import WINDOW_HEIGHT, WINDOW_WIDTH, PPI


def _fmt(v: float) -> str:
    """C++ numeric formatter."""
    try:
        f = float(v)
    except Exception:
        f = 0.0
    out = f"{f:.6f}".rstrip("0").rstrip(".")
    if "." not in out:
        out += ".0"
    return out


def _cfg_get(cfg: dict, path: Sequence[str], default):
    """Safe nested config getter with dict-wrapped value support."""
    node = cfg
    for p in path:
        if not isinstance(node, dict):
            return default
        node = node.get(p, default)
    if isinstance(node, dict) and "value" in node:
        return node.get("value", default)
    return node if node is not None else default


def _project_hash(cfg: dict, timeline: Sequence[dict]) -> str:
    """Stable hash for generated pack parity checks."""
    payload = {
        "cfg": cfg,
        "timeline": timeline,
        "field_w_in": WINDOW_WIDTH / float(PPI),
        "field_h_in": WINDOW_HEIGHT / float(PPI),
    }
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _profile_idx(name: str) -> int:
    """Map profile string to enum index."""
    key = str(name or "").strip().lower()
    if key == "fast":
        return 0
    if key == "precise":
        return 2
    return 1


def _segment_type(seg: dict) -> str:
    """Map timeline segment to exported SegmentType enum literal."""
    st = str(seg.get("type", "")).strip().lower()
    if st in {"move"}:
        return "SegmentType::MoveToPoint"
    if st in {"pose"}:
        return "SegmentType::MoveToPose"
    if st in {"turn", "face"}:
        return "SegmentType::TurnToHeading"
    if st == "swing":
        return "SegmentType::SwingToHeading"
    if st in {"velnudge", "vel_nudge", "nudge"}:
        return "SegmentType::VelNudge"
    if st == "wait":
        return "SegmentType::WaitMs"
    raise ValueError(f"Unsupported segment type for Efficiency Pack export: '{st}'")


def _segment_flags(seg: dict, seg_type_literal: str) -> int:
    """Encode segment behavior flags for runtime."""
    flags = 0
    if seg_type_literal == "SegmentType::SwingToHeading":
        side_tok = str(
            seg.get("swing_side", seg.get("side", seg.get("drive_side", "left")))
        ).strip().lower()
        if side_tok in {"right", "r", "1", "cw_right"}:
            flags |= 1  # kSegFlagSwingRight
    return flags


def _trigger_unit_for_segment(seg_type_literal: str) -> str:
    """Default trigger progress unit by segment type."""
    if seg_type_literal in {"SegmentType::TurnToHeading", "SegmentType::SwingToHeading"}:
        return "TriggerUnit::Degrees"
    if seg_type_literal in {"SegmentType::MoveToPoint", "SegmentType::MoveToPose", "SegmentType::VelNudge"}:
        return "TriggerUnit::Inches"
    if seg_type_literal == "SegmentType::WaitMs":
        return "TriggerUnit::Milliseconds"
    return "TriggerUnit::None"


def _finalize_mode(seg: dict, is_last_motion: bool) -> str:
    """Compute finalize mode literal."""
    st = str(seg.get("type", "")).strip().lower()
    finalize_flag = bool(seg.get("finalize", False) or seg.get("role") == "finalize")
    if not finalize_flag and not is_last_motion:
        return "FinalizeMode::None"
    if st in {"pose", "swing"}:
        return "FinalizeMode::Composite"
    return "FinalizeMode::Simple"


def _corridor_for_segment(seg: dict, defaults: Dict[str, float]) -> Dict[str, float]:
    """Default corridor generator per segment."""
    p0 = seg.get("p0", seg.get("start_pos", (0.0, 0.0)))
    p1 = seg.get("p1", seg.get("end_pos", p0))
    try:
        x1, y1 = float(p0[0]), float(p0[1])
        x2, y2 = float(p1[0]), float(p1[1])
    except Exception:
        x1 = y1 = x2 = y2 = 0.0
    return {
        "enabled": int(defaults.get("enabled", 1)),
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "half_width_in": float(defaults.get("half_width_in", 8.0)),
        "soft_log_penalty": float(defaults.get("soft_log_penalty", 0.25)),
    }


def _schedule_table_hpp(cfg: dict) -> str:
    """Emit motion schedule table header."""
    lines: List[str] = []
    lines.append("#pragma once")
    lines.append('#include "atticus/auton_runner.hpp"')
    lines.append("")
    lines.append("namespace atticus_generated {")
    lines.append("enum class MotionProfile : int { FAST = 0, NORMAL = 1, PRECISE = 2 };")
    lines.append("")
    lines.append("static constexpr atticus::MotionScheduleEntry kScheduleTable[3][4][2] = {")
    # Profile ordering: FAST, NORMAL, PRECISE
    # Segment ordering: Move, Turn, Swing, Pose
    # Phase ordering: CHAINED, PRECISE
    table_defaults = {
        "FAST": {
            "move": ((95.0, 72.0, 8.0, 0.6, 0), (90.0, 0.0, 1.0, 0.6, 1)),
            "turn": ((95.0, 90.0, 15.0, 0.6, 0), (80.0, 0.0, 2.0, 0.6, 1)),
            "swing": ((95.0, 90.0, 15.0, 0.6, 0), (80.0, 0.0, 2.0, 0.6, 1)),
            "pose": ((95.0, 72.0, 8.0, 0.6, 0), (90.0, 0.0, 1.0, 0.6, 1)),
        },
        "NORMAL": {
            "move": ((90.0, 48.0, 5.0, 0.6, 0), (85.0, 0.0, 1.0, 0.6, 1)),
            "turn": ((90.0, 60.0, 10.0, 0.6, 0), (75.0, 0.0, 2.0, 0.6, 1)),
            "swing": ((90.0, 60.0, 10.0, 0.6, 0), (75.0, 0.0, 2.0, 0.6, 1)),
            "pose": ((90.0, 48.0, 5.0, 0.6, 0), (85.0, 0.0, 1.0, 0.6, 1)),
        },
        "PRECISE": {
            "move": ((75.0, 0.0, 0.0, 0.6, 1), (75.0, 0.0, 0.0, 0.6, 1)),
            "turn": ((70.0, 0.0, 0.0, 0.6, 1), (70.0, 0.0, 0.0, 0.6, 1)),
            "swing": ((70.0, 0.0, 0.0, 0.6, 1), (70.0, 0.0, 0.0, 0.6, 1)),
            "pose": ((75.0, 0.0, 0.0, 0.6, 1), (75.0, 0.0, 0.0, 0.6, 1)),
        },
    }
    for prof in ("FAST", "NORMAL", "PRECISE"):
        lines.append("  {")
        for key in ("move", "turn", "swing", "pose"):
            chain, precise = table_defaults[prof][key]
            lines.append(
                "    {{"
                f"{{{_fmt(chain[0])}, {_fmt(chain[1])}, {_fmt(chain[2])}, {_fmt(chain[3])}, {int(chain[4])}}}, "
                f"{{{_fmt(precise[0])}, {_fmt(precise[1])}, {_fmt(precise[2])}, {_fmt(precise[3])}, {int(precise[4])}}}"
                "}},"
            )
        lines.append("  },")
    lines.append("};")
    lines.append("")
    lines.append("} // namespace atticus_generated")
    return "\n".join(lines) + "\n"


def _config_hpp(cfg: dict, project_hash: str) -> str:
    """Emit generated runtime config header."""
    sched_ess = float(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "ess_resample_ratio"), 0.5))
    sched_batch = int(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "batch_size"), 3))
    sched_watchdog = float(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "watchdog_budget_ms"), 8.0))
    sched_kld = int(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "kld_enabled"), 1))
    sched_n_min = int(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "kld_n_min"), 250))
    sched_n_max = int(_cfg_get(cfg, ("codegen", "atticus_pack", "scheduling", "kld_n_max"), 500))
    cgr_enabled = int(_cfg_get(cfg, ("codegen", "atticus_pack", "cgr_lite", "enabled"), 1))
    cgr_top_k = int(_cfg_get(cfg, ("codegen", "atticus_pack", "cgr_lite", "top_k"), 8))
    cgr_iters = int(_cfg_get(cfg, ("codegen", "atticus_pack", "cgr_lite", "max_iters"), 2))
    cgr_budget = float(_cfg_get(cfg, ("codegen", "atticus_pack", "cgr_lite", "budget_ms"), 1.5))
    cgr_apply_on = str(_cfg_get(cfg, ("codegen", "atticus_pack", "cgr_lite", "apply_on"), "finalize_only")).strip().lower()
    cgr_apply_on_finalize = 1 if cgr_apply_on != "always" else 0
    corridor_enabled = int(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "enabled"), 1))
    corridor_half = float(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "half_width_in"), 8.0))
    corridor_pen = float(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "soft_log_penalty"), 0.25))

    lines: List[str] = []
    lines.append("#pragma once")
    lines.append('#include "atticus/addon.hpp"')
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("namespace atticus_generated {")
    lines.append(f'static constexpr const char* kProjectHash = "{project_hash}";')
    lines.append(f"static constexpr unsigned int kProjectHash32 = 0x{project_hash[:8]}u;")
    lines.append("")
    lines.append("inline atticus::AddonRuntimeConfig buildAddonRuntimeConfig() {")
    lines.append("  atticus::AddonRuntimeConfig cfg{};")
    lines.append(f"  cfg.localization_loop_ms = {int(_cfg_get(cfg, ('mcl', 'loop_ms'), 20))};")
    lines.append(f"  cfg.stall_ms = {int(_cfg_get(cfg, ('mcl', 'stall_ms'), 40))};")
    lines.append("  cfg.telemetry_period_ms = 100;")
    lines.append("  cfg.enable_internal_localization = false;")
    lines.append("  cfg.enable_pose_writeback = true;")
    lines.append("  cfg.enable_lcd_telemetry = false;")
    lines.append(f"  cfg.allow_pose_writeback_when_settled = {1 if int(_cfg_get(cfg, ('mcl', 'correction', 'safe_window_enabled'), 1)) else 0};")
    lines.append(f"  cfg.writeback_settled_speed_sum_rpm = {_fmt(_cfg_get(cfg, ('mcl', 'correction', 'safe_max_speed_in_s'), 8.0) * 3.0)};")
    lines.append(f"  cfg.writeback_settled_yaw_rate_dps = {_fmt(_cfg_get(cfg, ('mcl', 'correction', 'safe_max_turn_deg_s'), 60.0))};")
    lines.append(f"  cfg.writeback_max_jump_in = {_fmt(_cfg_get(cfg, ('mcl', 'correction', 'max_trans_jump_in'), 8.0))};")
    lines.append(f"  cfg.writeback_max_jump_deg = {_fmt(_cfg_get(cfg, ('mcl', 'correction', 'max_theta_jump_deg'), 15.0))};")
    lines.append(f"  cfg.external_writeback_min_confidence = {_fmt(_cfg_get(cfg, ('mcl', 'correction', 'min_confidence'), 0.6))};")
    lines.append("  cfg.external_sample_max_age_ms = 200;")
    lines.append("  cfg.reject_external_writeback_when_lost = true;")
    lines.append("  return cfg;")
    lines.append("}")
    lines.append("")
    lines.append("struct LocalizationSchedulePolicy {")
    lines.append("  float ess_resample_ratio;")
    lines.append("  std::int32_t batch_size;")
    lines.append("  float watchdog_budget_ms;")
    lines.append("  bool kld_enabled;")
    lines.append("  std::int32_t kld_n_min;")
    lines.append("  std::int32_t kld_n_max;")
    lines.append("};")
    lines.append("")
    lines.append("inline LocalizationSchedulePolicy buildLocalizationSchedulePolicy() {")
    lines.append("  LocalizationSchedulePolicy p{};")
    lines.append(f"  p.ess_resample_ratio = {_fmt(sched_ess)}F;")
    lines.append(f"  p.batch_size = {max(1, sched_batch)};")
    lines.append(f"  p.watchdog_budget_ms = {_fmt(max(0.0, sched_watchdog))}F;")
    lines.append(f"  p.kld_enabled = {1 if sched_kld else 0};")
    lines.append(f"  p.kld_n_min = {max(1, sched_n_min)};")
    lines.append(f"  p.kld_n_max = {max(max(1, sched_n_min), sched_n_max)};")
    lines.append("  return p;")
    lines.append("}")
    lines.append("")
    lines.append("struct CgrLitePolicy {")
    lines.append("  bool enabled;")
    lines.append("  std::int32_t top_k;")
    lines.append("  std::int32_t max_iters;")
    lines.append("  float budget_ms;")
    lines.append("  bool apply_on_finalize_only;")
    lines.append("};")
    lines.append("")
    lines.append("inline CgrLitePolicy buildCgrLitePolicy() {")
    lines.append("  CgrLitePolicy p{};")
    lines.append(f"  p.enabled = {1 if cgr_enabled else 0};")
    lines.append(f"  p.top_k = {max(1, cgr_top_k)};")
    lines.append(f"  p.max_iters = {max(1, cgr_iters)};")
    lines.append(f"  p.budget_ms = {_fmt(max(0.0, cgr_budget))}F;")
    lines.append(f"  p.apply_on_finalize_only = {cgr_apply_on_finalize};")
    lines.append("  return p;")
    lines.append("}")
    lines.append("")
    lines.append("struct CorridorDefaults {")
    lines.append("  bool enabled;")
    lines.append("  float half_width_in;")
    lines.append("  float soft_log_penalty;")
    lines.append("};")
    lines.append("")
    lines.append("inline CorridorDefaults buildCorridorDefaults() {")
    lines.append("  CorridorDefaults p{};")
    lines.append(f"  p.enabled = {1 if corridor_enabled else 0};")
    lines.append(f"  p.half_width_in = {_fmt(corridor_half)}F;")
    lines.append(f"  p.soft_log_penalty = {_fmt(corridor_pen)}F;")
    lines.append("  return p;")
    lines.append("}")
    lines.append("")
    lines.append("inline atticus::DriveLimiterConfig buildDriveLimiterConfig() {")
    lines.append("  atticus::DriveLimiterConfig cfg{};")
    lines.append("  cfg.max_voltage = 12000.0F;")
    lines.append("  cfg.max_delta_voltage_per_tick = 700.0F;")
    lines.append("  cfg.max_turn_ratio = 1.0F;")
    lines.append("  return cfg;")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace atticus_generated")
    return "\n".join(lines) + "\n"


def _segment_timeout_ms(seg: dict) -> int:
    """Convert timeline T seconds to timeout ms with conservative clamp."""
    try:
        t_s = float(seg.get("T", 0.0) or 0.0)
    except Exception:
        t_s = 0.0
    return max(100, int(round(t_s * 1000.0)))


def _angle_diff_deg(target: float, current: float) -> float:
    """Smallest signed angular residual in degrees."""
    return ((target - current + 180.0) % 360.0) - 180.0


def _segment_progress_extent(seg: dict, seg_type_literal: str) -> float:
    """Return progress magnitude in expected trigger units for a segment."""
    if seg_type_literal == "SegmentType::WaitMs":
        return float(_segment_timeout_ms(seg))

    if seg_type_literal in {"SegmentType::TurnToHeading", "SegmentType::SwingToHeading"}:
        try:
            a0 = float(seg.get("start_heading", 0.0) or 0.0)
            a1 = float(seg.get("target_heading", seg.get("facing", a0)) or a0)
            mag = abs(_angle_diff_deg(a1, a0))
            if mag > 1e-6:
                return mag
        except Exception:
            pass
        try:
            return abs(float(seg.get("delta_heading_deg", seg.get("turn_deg", 0.0)) or 0.0))
        except Exception:
            return 0.0

    # Linear progress segments (inches).
    for key in ("path_length_in", "length_in", "distance_in"):
        try:
            val = abs(float(seg.get(key, 0.0) or 0.0))
            if val > 1e-6:
                return val
        except Exception:
            continue

    p0 = seg.get("p0", seg.get("start_pos", (0.0, 0.0)))
    p1 = seg.get("p1", seg.get("end_pos", p0))
    try:
        x0, y0 = float(p0[0]), float(p0[1])
        x1, y1 = float(p1[0]), float(p1[1])
        dx = x1 - x0
        dy = y1 - y0
        return (dx * dx + dy * dy) ** 0.5
    except Exception:
        return 0.0


def _normalize_event_action(item: dict) -> Optional[dict]:
    """Normalize one action payload from event/action-list formats."""
    if not isinstance(item, dict):
        return None
    kind = str(item.get("kind", item.get("type", ""))).strip().lower()
    if not kind:
        if item.get("code") is not None:
            kind = "code"
        elif item.get("action") is not None:
            kind = "action"
        elif item.get("name") is not None:
            kind = "preset"
    if kind == "code":
        code = str(item.get("code", "")).strip()
        if code:
            return {"kind": "code", "code": code}
        return None
    if kind in {"preset", "action"}:
        name = str(item.get("name", item.get("action", ""))).strip()
        if name:
            return {
                "kind": "preset",
                "name": name,
                "state": item.get("state", None),
                "value": item.get("value", None),
                "values": item.get("values", []),
            }
        return None
    if kind == "action_id":
        try:
            action_id = int(item.get("action_id", item.get("id", 0)) or 0)
        except Exception:
            action_id = 0
        if action_id > 0:
            return {"kind": "action_id", "action_id": action_id}
        return None
    return None


def _event_action_list(ev: dict) -> List[dict]:
    """Extract zero or more normalized actions from an edge-event dict."""
    out: List[dict] = []
    acts = ev.get("actions", [])
    if isinstance(acts, list):
        for item in acts:
            norm = _normalize_event_action(item)
            if norm is not None:
                out.append(norm)
    if out:
        return out

    # Backward-compatible single-action event forms.
    inline = _normalize_event_action(ev)
    if inline is not None:
        out.append(inline)
    return out


def _event_threshold_and_unit(ev: dict, seg: dict, seg_type_literal: str) -> Tuple[float, str]:
    """Convert event threshold to deterministic runtime units."""
    expected = _trigger_unit_for_segment(seg_type_literal)
    if expected == "TriggerUnit::None":
        return 0.0, "TriggerUnit::None"

    extent = max(0.0, _segment_progress_extent(seg, seg_type_literal))
    unit = str(ev.get("unit", "")).strip().lower()
    has_value = ("value" in ev) or ("threshold" in ev)

    def _get_value(keys: Sequence[str], default: float = 0.0) -> float:
        for k in keys:
            if k not in ev:
                continue
            try:
                return float(ev.get(k, default) or default)
            except Exception:
                continue
        return default

    # Missing unit: interpret explicit value as expected-unit scalar, otherwise use t-fraction.
    if not unit:
        if has_value:
            raw = _get_value(("value", "threshold"), 0.0)
        else:
            t = _get_value(("t",), 0.0)
            t = max(0.0, min(1.0, t))
            raw = t * extent
        return max(0.0, raw), expected

    if unit == "none":
        return 0.0, "TriggerUnit::None"

    # Fraction input: convert deterministically to segment's expected runtime unit.
    if unit == "frac":
        f = _get_value(("value", "threshold", "t"), 0.0)
        f = max(0.0, min(1.0, f))
        return max(0.0, f * extent), expected

    if unit == "deg":
        unit_lit = "TriggerUnit::Degrees"
    elif unit == "ms":
        unit_lit = "TriggerUnit::Milliseconds"
    elif unit == "in":
        unit_lit = "TriggerUnit::Inches"
    else:
        # Unknown unit token: fall back to deterministic expected mapping.
        unit_lit = expected

    if has_value:
        raw = _get_value(("value", "threshold"), 0.0)
    else:
        t = _get_value(("t",), 0.0)
        t = max(0.0, min(1.0, t))
        raw = t * extent
    return max(0.0, raw), unit_lit


def _action_key(action: dict) -> str:
    """Stable action registry key."""
    return json.dumps(action, sort_keys=True, separators=(",", ":"), default=str)


def _cpp_comment_safe(text: str) -> str:
    """Sanitize text for single-line C++ comments."""
    return str(text or "").replace("\r", " ").replace("\n", " ").replace("*/", "* /")


def _build_plan_data(timeline: Sequence[dict], cfg: dict) -> Tuple[List[dict], List[dict], List[dict]]:
    """Compile raw timeline into export segment/trigger arrays."""
    segments: List[dict] = []
    triggers: List[dict] = []
    action_defs: List[dict] = []
    action_to_id: Dict[str, int] = {}
    corridor_defaults = {
        "enabled": int(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "enabled"), 1)),
        "half_width_in": float(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "half_width_in"), 8.0)),
        "soft_log_penalty": float(_cfg_get(cfg, ("codegen", "atticus_pack", "corridor_defaults", "soft_log_penalty"), 0.25)),
    }

    motion_indices = [
        i
        for i, s in enumerate(timeline)
        if str(s.get("type", "")).lower() in {"move", "pose", "turn", "face", "swing", "wait", "velnudge", "vel_nudge", "nudge"}
    ]
    last_motion_idx = motion_indices[-1] if motion_indices else -1

    for idx, seg in enumerate(timeline):
        st = str(seg.get("type", "")).strip().lower()
        if st in {"marker", "reverse", "reshape"}:
            continue
        seg_type = _segment_type(seg)
        p1 = seg.get("p1", seg.get("end_pos", seg.get("target_pos", (0.0, 0.0))))
        x = float(p1[0]) if isinstance(p1, (list, tuple)) and len(p1) >= 2 else 0.0
        y = float(p1[1]) if isinstance(p1, (list, tuple)) and len(p1) >= 2 else 0.0
        theta = float(seg.get("target_heading", seg.get("facing", 0.0) or 0.0))
        profile = str(seg.get("profile_override", "normal")).strip().upper()
        profile_idx = _profile_idx(profile)
        is_last_motion = idx == last_motion_idx
        finalize = _finalize_mode(seg, is_last_motion)
        corridor = _corridor_for_segment(seg, corridor_defaults)
        flags = _segment_flags(seg, seg_type)

        trig_begin = len(triggers)
        edge_events = seg.get("edge_events", [])
        if not isinstance(edge_events, list):
            edge_events = []
        for ev in edge_events:
            if not isinstance(ev, dict):
                continue
            threshold, unit_lit = _event_threshold_and_unit(ev, seg, seg_type)
            event_actions = _event_action_list(ev)

            if event_actions:
                for action in event_actions:
                    key = _action_key(action)
                    action_id = action_to_id.get(key)
                    if action_id is None:
                        action_id = len(action_defs) + 1
                        action_defs.append(action)
                        action_to_id[key] = action_id
                    triggers.append(
                        {
                            "type": "TriggerType::Progress",
                            "unit": unit_lit,
                            "threshold": threshold,
                            "action_id": action_id,
                            "flags": 0,
                        }
                    )
            else:
                # Marker with no action payload still preserves wait/progress chronology.
                triggers.append(
                    {
                        "type": "TriggerType::Progress",
                        "unit": unit_lit,
                        "threshold": threshold,
                        "action_id": 0,
                        "flags": 0,
                    }
                )
        trig_end = len(triggers)

        segments.append(
            {
                "type": seg_type,
                "x": x,
                "y": y,
                "theta": theta,
                "schedule_profile": profile_idx,
                "flags": flags,
                "timeout_ms": _segment_timeout_ms(seg),
                "trigger_begin": trig_begin,
                "trigger_end": trig_end,
                "finalize_mode": finalize,
                "corridor": corridor,
            }
        )

    return segments, triggers, action_defs


def _auton_plan_hpp(timeline: Sequence[dict], cfg: dict) -> str:
    """Emit plan array header."""
    segments, triggers, _action_defs = _build_plan_data(timeline, cfg)
    lines: List[str] = []
    lines.append("#pragma once")
    lines.append('#include "atticus/auton_runner.hpp"')
    lines.append("")
    lines.append("namespace atticus_generated {")
    lines.append("static constexpr atticus::Trigger kTriggers[] = {")
    if triggers:
        for tr in triggers:
            lines.append(
                "  {"
                f"{tr['type']}, {tr['unit']}, {_fmt(tr['threshold'])}, "
                f"static_cast<atticus::ActionId>({int(tr['action_id'])}), {int(tr['flags'])}"
                "},"
            )
    else:
        lines.append("  {atticus::TriggerType::None, atticus::TriggerUnit::None, 0.0, static_cast<atticus::ActionId>(0), 0},")
    lines.append("};")
    lines.append("")
    lines.append("static constexpr atticus::Segment kSegments[] = {")
    if segments:
        for seg in segments:
            c = seg["corridor"]
            lines.append(
                "  {"
                f"{seg['type']}, "
                f"{{{_fmt(seg['x'])}, {_fmt(seg['y'])}, {_fmt(seg['theta'])}}}, "
                f"{int(seg['schedule_profile'])}, {int(seg.get('flags', 0))}, {int(seg['timeout_ms'])}, "
                f"{int(seg['trigger_begin'])}, {int(seg['trigger_end'])}, "
                f"{seg['finalize_mode']}, "
                f"{{{int(c['enabled'])}, {_fmt(c['x1'])}, {_fmt(c['y1'])}, {_fmt(c['x2'])}, {_fmt(c['y2'])}, {_fmt(c['half_width_in'])}, {_fmt(c['soft_log_penalty'])}}}"
                "},"
            )
    else:
        lines.append(
            "  {atticus::SegmentType::WaitMs, {0.0,0.0,0.0}, 1, 0, 100, 0, 0, "
            "atticus::FinalizeMode::None, {0,0.0,0.0,0.0,0.0,8.0,0.25}},"
        )
    lines.append("};")
    lines.append("")
    lines.append("static constexpr atticus::PlanView kPlan{")
    lines.append("  kSegments,")
    lines.append("  static_cast<int>(sizeof(kSegments) / sizeof(kSegments[0])),")
    lines.append("  kTriggers,")
    lines.append("  static_cast<int>(sizeof(kTriggers) / sizeof(kTriggers[0]))")
    lines.append("};")
    lines.append("")
    lines.append("} // namespace atticus_generated")
    return "\n".join(lines) + "\n"


def _localizer_adapter_hpp() -> str:
    """Emit adapter skeleton header for ProsMCL -> ILocalizationSource."""
    return """#pragma once
#include "atticus/localizer.hpp"
#include "mcl_runtime.h"

namespace atticus_generated {

class ProsMclLocalizationSource final : public atticus::ILocalizationSource {
public:
  explicit ProsMclLocalizationSource(ProsMCL& runtime) : runtime_(runtime) {}

  bool read(atticus::LocalizerSample& out) const override {
    const MCLPose fused = runtime_.getFusedPose();
    const ProsMCL::DebugSnapshot snap = runtime_.getDebugSnapshot();
    const std::uint32_t flags = snap.event_flags;
    out.fused_pose = {static_cast<float>(fused.x), static_cast<float>(fused.y), static_cast<float>(fused.theta)};
    out.correction_pose = out.fused_pose;
    out.correction_suggested =
        (flags & MCL_EVENT_MCL_EKF_APPLIED) != 0u ||
        (flags & MCL_EVENT_ODOM_CORR_APPLIED) != 0u;
    const double neff = snap.mcl_neff;
    double ess = snap.mcl_ess_ratio;
    const double peak_raw = snap.mcl_peakedness;
    if (!(ess >= 0.0 && ess <= 1.0)) {
      ess = (peak_raw >= 0.0 && peak_raw <= 1.0) ? (1.0 - peak_raw) : 1.0;
    }
    if (ess < 0.0) ess = 0.0;
    if (ess > 1.0) ess = 1.0;
    const double peak = 1.0 - ess;
    out.lost = ((flags & MCL_EVENT_RECOVERY_ACTIVE) != 0u) || (peak < 0.15);
    out.neff = static_cast<float>(neff);
    out.ess_ratio = static_cast<float>(ess);
    out.peakedness = static_cast<float>(peak);
    out.particle_count = static_cast<float>((ess > 1e-9) ? (neff / ess) : 0.0);
    out.sample_time_ms = snap.time_ms;
    return true;
  }

  void setCorridorHint(const atticus::LocalizerCorridorHint* hint) override {
    if (hint == nullptr || !hint->enabled) {
      runtime_.clearSegmentBand();
      return;
    }
    MCLPose band[2] = {
      {hint->x1_in, hint->y1_in, 0.0},
      {hint->x2_in, hint->y2_in, 0.0}
    };
    runtime_.setSegmentBand(band, 2, hint->half_width_in);
  }

  void requestRelocalize(const atticus::LocalizerPose2D& around, float /*radius_in*/) override {
    runtime_.setPose(static_cast<double>(around.x_in), static_cast<double>(around.y_in), static_cast<double>(around.theta_deg));
    runtime_.requestRelocalize();
  }

private:
  ProsMCL& runtime_;
};

} // namespace atticus_generated
"""


def _auton_actions_cpp(action_defs: Sequence[dict]) -> str:
    """Emit action dispatch with generated action cases."""
    lines: List[str] = []
    lines.append('#include "atticus/auton_runner.hpp"')
    lines.append("")
    lines.append("namespace atticus_generated {")
    lines.append("")
    lines.append("void runAction(atticus::ActionId id) {")
    lines.append("  switch (static_cast<int>(id)) {")
    lines.append("    case 0:")
    lines.append("      break;")
    for idx, action in enumerate(action_defs, start=1):
        kind = str(action.get("kind", "unknown")).strip().lower()
        lines.append(f"    case {idx}: {{")
        if kind == "code":
            code = str(action.get("code", "")).strip()
            if code:
                lines.append("      // Generated marker code action.")
                for raw in code.splitlines():
                    line = raw.rstrip()
                    if line:
                        lines.append(f"      {line}")
            else:
                lines.append("      // Empty code action payload.")
        elif kind == "preset":
            name = _cpp_comment_safe(str(action.get("name", "")))
            state = _cpp_comment_safe(str(action.get("state", "")))
            value = _cpp_comment_safe(str(action.get("value", "")))
            lines.append(f"      // Preset action: name=\"{name}\" state=\"{state}\" value=\"{value}\".")
            lines.append("      // Hook your mechanism dispatcher here if desired.")
        elif kind == "action_id":
            act_id = int(action.get("action_id", 0) or 0)
            lines.append(f"      // Legacy action_id payload ({act_id}).")
            lines.append("      // Hook this case into your mechanism dispatcher if you use numeric action IDs.")
        else:
            raw = _cpp_comment_safe(json.dumps(action, sort_keys=True))
            lines.append(f"      // Unhandled action payload: {raw}")
        lines.append("      break;")
        lines.append("    }")
    lines.append("    default:")
    lines.append("      break;")
    lines.append("  }")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace atticus_generated")
    return "\n".join(lines) + "\n"


def _drive_ff_hpp(cfg: dict, lut_hash32: int = 0) -> str:
    """Emit generated drive feedforward constants."""
    rpm = float(_cfg_get(cfg, ("robot_physics", "rpm"), 400.0) or 400.0)
    if rpm <= 1e-6:
        rpm = 400.0
    k_v = 12000.0 / rpm
    k_s = float(_cfg_get(cfg, ("codegen", "calibration", "profiles", "drive_ff", "kS_mv"), 250.0) or 250.0)
    k_a = float(_cfg_get(cfg, ("codegen", "calibration", "profiles", "drive_ff", "kA_mv_per_rpm_s"), 0.0) or 0.0)
    lines: List[str] = []
    lines.append("#pragma once")
    lines.append("#include <cstdint>")
    lines.append("")
    lines.append("namespace atticus_generated {")
    lines.append("struct DriveFeedforwardModel {")
    lines.append("  float kS_mv;")
    lines.append("  float kV_mv_per_rpm;")
    lines.append("  float kA_mv_per_rpm_s;")
    lines.append("  bool has_lut;")
    lines.append("  std::uint32_t lut_hash32;")
    lines.append("};")
    lines.append("")
    lines.append("inline DriveFeedforwardModel buildDriveFeedforwardModel() {")
    lines.append("  DriveFeedforwardModel m{};")
    lines.append(f"  m.kS_mv = {_fmt(max(0.0, k_s))}F;")
    lines.append(f"  m.kV_mv_per_rpm = {_fmt(max(0.0, k_v))}F;")
    lines.append(f"  m.kA_mv_per_rpm_s = {_fmt(max(0.0, k_a))}F;")
    lines.append(f"  m.has_lut = {1 if int(lut_hash32) != 0 else 0};")
    lines.append(f"  m.lut_hash32 = 0x{int(lut_hash32) & 0xFFFFFFFF:08x}u;")
    lines.append("  return m;")
    lines.append("}")
    lines.append("")
    lines.append("} // namespace atticus_generated")
    return "\n".join(lines) + "\n"


def _extract_drive_lut(cfg: dict) -> List[Tuple[float, float]]:
    """Extract optional drive LUT points as (rpm, mv)."""
    lut = _cfg_get(cfg, ("codegen", "calibration", "profiles", "drive_ff_lut"), [])
    if not isinstance(lut, list):
        return []
    out: List[Tuple[float, float]] = []
    for row in lut:
        if isinstance(row, dict):
            rpm = row.get("rpm", row.get("x"))
            mv = row.get("mv", row.get("y"))
        elif isinstance(row, (list, tuple)) and len(row) >= 2:
            rpm, mv = row[0], row[1]
        else:
            continue
        try:
            rf = float(rpm)
            vf = float(mv)
        except Exception:
            continue
        out.append((rf, vf))
    out.sort(key=lambda p: p[0])
    return out


def _write_drive_lut_bin(path: str, lut: List[Tuple[float, float]]) -> int:
    """Write optional binary LUT file and return a 32-bit hash."""
    import struct

    if not lut:
        return 0
    payload = bytearray()
    payload += b"ATLU"
    payload += struct.pack("<HHI", 1, 0, len(lut))
    for rpm, mv in lut:
        payload += struct.pack("<ff", float(rpm), float(mv))
    h = hashlib.sha256(bytes(payload)).hexdigest()
    with open(path, "wb") as f:
        f.write(payload)
    return int(h[:8], 16)


def _autonomous_cpp() -> str:
    """Emit generated autonomous wrapper."""
    return """#include "atticus/auton_runner.hpp"
#include "atticus_generated/atticus_auton_plan.hpp"

namespace atticus_generated {

void runAutonomous(atticus::AutonScriptRunner& runner) {
  runner.run(kPlan);
}

} // namespace atticus_generated
"""


def _startup_cpp() -> str:
    """Emit generated startup helper."""
    return """#include "atticus_generated/atticus_config.hpp"
#include "atticus_generated/atticus_motion_schedule.hpp"
#include "atticus/addon.hpp"
#include "atticus/auton_runner.hpp"
#include <memory>

namespace atticus_generated {

void runAction(atticus::ActionId id);

std::unique_ptr<atticus::AtticusAddon> createAddon(
    const atticus::AtticusAddon::Context& ctx,
    atticus::ILocalizationSource* source) {
  auto addon = std::make_unique<atticus::AtticusAddon>(
      ctx, buildAddonRuntimeConfig(), buildDriveLimiterConfig());
  addon->setLocalizationSource(source);
  addon->start();
  return addon;
}

std::unique_ptr<atticus::AtticusAddon> createAddon(
    const atticus::AtticusAddon::Context& ctx) {
  // Normal flow: generated runtime config as source of truth, no manual edits.
  return createAddon(ctx, nullptr);
}

std::unique_ptr<atticus::AutonScriptRunner> createRunner(
    lemlib::Chassis& chassis,
    atticus::AtticusAddon& addon) {
  auto runner = std::make_unique<atticus::AutonScriptRunner>(chassis, addon);
  runner->setScheduleTable(&kScheduleTable[0][0][0], 3, 4, 2);
  runner->setActionCallback(&runAction);
  return runner;
}

} // namespace atticus_generated
"""


def write_efficiency_pack(
    cfg: dict,
    timeline: Sequence[dict],
    out_dir: str,
    *,
    routine_name: str = "autonomous",
    auto_convert_unsupported: bool = False,
    strict_validation: bool = True,
    emit_localizer_adapter: bool = True,
) -> Dict[str, object]:
    """Generate Atticus Efficiency Pack artifacts."""
    tl = list(timeline or [])
    conversion_issues: List[ValidationIssue] = []
    if auto_convert_unsupported:
        raise ValueError(
            "auto_convert_unsupported was removed. Resolve unsupported segments explicitly before export."
        )

    issues = validate_efficiency_pack_timeline(tl, strict=strict_validation)
    hard_errors = [i for i in issues if i.severity == "error"]
    if hard_errors:
        messages = "\n".join(f"[{i.code}] seg={i.segment_index}: {i.message}" for i in hard_errors)
        raise ValueError(f"Efficiency Pack validation failed:\n{messages}")

    os.makedirs(out_dir, exist_ok=True)
    inc_dir = os.path.join(out_dir, "include", "atticus_generated")
    src_dir = os.path.join(out_dir, "src", "atticus_generated")
    os.makedirs(inc_dir, exist_ok=True)
    os.makedirs(src_dir, exist_ok=True)

    project_hash = _project_hash(cfg, tl)
    _segments_preview, _triggers_preview, action_defs = _build_plan_data(tl, cfg)
    lut_points = _extract_drive_lut(cfg)
    lut_hash32 = 0
    lut_path = os.path.join(out_dir, "atticus_drive_lut.bin")
    if lut_points:
        lut_hash32 = _write_drive_lut_bin(lut_path, lut_points)

    files = {
        os.path.join(inc_dir, "atticus_config.hpp"): _config_hpp(cfg, project_hash),
        os.path.join(inc_dir, "atticus_motion_schedule.hpp"): _schedule_table_hpp(cfg),
        os.path.join(inc_dir, "atticus_auton_plan.hpp"): _auton_plan_hpp(tl, cfg),
        os.path.join(inc_dir, "atticus_drive_ff.hpp"): _drive_ff_hpp(cfg, lut_hash32),
        os.path.join(src_dir, "atticus_auton_actions.cpp"): _auton_actions_cpp(action_defs),
        os.path.join(src_dir, "atticus_autonomous.cpp"): _autonomous_cpp(),
        os.path.join(src_dir, "atticus_startup.cpp"): _startup_cpp(),
    }
    if emit_localizer_adapter:
        files[os.path.join(inc_dir, "atticus_localizer_adapter.hpp")] = _localizer_adapter_hpp()

    for path, content in files.items():
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)

    generated_files = sorted(files.keys())
    if lut_hash32 != 0:
        generated_files.append(lut_path)

    return {
        "out_dir": out_dir,
        "project_hash": project_hash,
        "drive_lut_hash32": f"0x{lut_hash32:08x}",
        "action_count": len(action_defs),
        "issues": [i.__dict__ for i in (conversion_issues + issues)],
        "generated_files": generated_files,
    }
