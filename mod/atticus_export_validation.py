"""Validation helpers for Atticus Efficiency Pack export."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Tuple


@dataclass
class ValidationIssue:
    """Single export validation issue."""

    severity: str
    code: str
    message: str
    segment_index: int = -1


SUPPORTED_SEGMENT_TYPES = {
    "move",
    "pose",
    "turn",
    "face",
    "swing",
    "wait",
    "marker",
    "reverse",
    "reshape",
}


def _is_motion_type(st: str) -> bool:
    """Return True when segment is a motion primitive."""
    return st in {"move", "pose", "turn", "face", "swing"}


def _segment_progress_unit(st: str) -> str:
    """Return progress unit expected by waitUntil-like triggers."""
    if st in {"turn", "face", "swing"}:
        return "deg"
    if st in {"move", "pose"}:
        return "in"
    if st == "wait":
        return "ms"
    return "none"


def _event_has_action_binding(ev: dict) -> bool:
    """Return True when event has at least one action payload binding."""
    if not isinstance(ev, dict):
        return False

    acts = ev.get("actions", [])
    if isinstance(acts, list):
        for item in acts:
            if not isinstance(item, dict):
                continue
            kind = str(item.get("kind", item.get("type", ""))).strip().lower()
            if kind == "code" and str(item.get("code", "")).strip():
                return True
            if kind in {"action", "preset"} and str(item.get("name", item.get("action", ""))).strip():
                return True
            if kind == "action_id":
                try:
                    if int(item.get("action_id", item.get("id", 0)) or 0) > 0:
                        return True
                except Exception:
                    pass

    if str(ev.get("code", "")).strip():
        return True
    if str(ev.get("name", ev.get("action", ""))).strip():
        return True
    try:
        if int(ev.get("action_id", 0) or 0) > 0:
            return True
    except Exception:
        pass
    return False


def validate_efficiency_pack_timeline(
    timeline: Iterable[dict],
    *,
    strict: bool = True,
) -> List[ValidationIssue]:
    """
    Validate timeline semantics for Efficiency Pack export.

    Rules enforced:
    - unsupported segment kinds
    - edge marker trigger unit compatibility
    - basic timeout sanity
    """
    issues: List[ValidationIssue] = []
    timeline_list = list(timeline)

    for idx, seg in enumerate(timeline_list):
        st = str(seg.get("type", "")).strip().lower()
        if st == "path":
            issues.append(
                ValidationIssue(
                    "error",
                    "path_not_supported_in_efficiency_pack",
                    "Path-follow segments are not supported by Efficiency Pack export. Convert to primitive segments first.",
                    idx,
                )
            )
            continue
        if st not in SUPPORTED_SEGMENT_TYPES:
            issues.append(
                ValidationIssue(
                    "error",
                    "unsupported_segment_type",
                    f"Segment type '{st}' is not supported by Efficiency Pack export.",
                    idx,
                )
            )
            continue

        if _is_motion_type(st):
            try:
                timeout_s = float(seg.get("T", 0.0) or 0.0)
            except Exception:
                timeout_s = 0.0
            if timeout_s <= 0.0:
                issues.append(
                    ValidationIssue(
                        "warning",
                        "non_positive_motion_time",
                        "Motion segment has non-positive duration; export will clamp timeout.",
                        idx,
                    )
                )

        if st == "swing":
            side = str(seg.get("swing_side", seg.get("side", seg.get("drive_side", "left")))).strip().lower()
            if side not in {"", "left", "right", "l", "r", "0", "1", "cw_left", "cw_right"}:
                issues.append(
                    ValidationIssue(
                        "warning",
                        "swing_side_unrecognized",
                        f"Swing side '{side}' is not recognized; export defaults to left unless explicitly right.",
                        idx,
                    )
                )

        expected_unit = _segment_progress_unit(st)
        edge_events = seg.get("edge_events", [])
        if not isinstance(edge_events, list):
            edge_events = []

        prev_abs = None
        prev_frac = None

        for ev in edge_events:
            if not isinstance(ev, dict):
                continue
            kind = str(ev.get("kind", "")).strip().lower()
            has_progress_fields = any(k in ev for k in ("unit", "value", "threshold", "t"))
            if kind in {"action", "preset", "code", "action_id"} and not _event_has_action_binding(ev):
                issues.append(
                    ValidationIssue(
                        "warning",
                        "missing_action_binding",
                        "Trigger action event has no usable action payload binding.",
                        idx,
                    )
                )
            # Pure marker without progress metadata is informational-only.
            if kind == "marker" and not has_progress_fields:
                continue

            unit = str(ev.get("unit", "")).strip().lower()
            if not unit:
                # Infer from segment, but emit warning so generated output is explicit.
                issues.append(
                    ValidationIssue(
                        "warning",
                        "trigger_unit_inferred",
                        f"Trigger unit missing; inferred '{expected_unit}' from segment type '{st}'.",
                        idx,
                    )
                )
                continue
            if expected_unit != "none" and unit not in {expected_unit, "frac"}:
                issues.append(
                    ValidationIssue(
                        "error",
                        "trigger_unit_mismatch",
                        (
                            f"Trigger unit '{unit}' is incompatible with segment type '{st}' "
                            f"(expected '{expected_unit}' or 'frac')."
                        ),
                        idx,
                    )
                )
            try:
                threshold = float(ev.get("value", ev.get("threshold", ev.get("t", 0.0))))
            except Exception:
                threshold = 0.0
            if threshold < 0.0:
                issues.append(
                    ValidationIssue(
                        "warning",
                        "negative_trigger_threshold",
                        "Trigger threshold is negative; runtime will clamp/wait unexpectedly.",
                        idx,
                    )
                )

            # Prevent ambiguous trigger ordering within a segment.
            # Compare absolute-threshold triggers and fraction-threshold triggers independently.
            if unit == "frac":
                frac = max(0.0, min(1.0, threshold))
                if prev_frac is not None and frac + 1e-9 < prev_frac:
                    issues.append(
                        ValidationIssue(
                            "error",
                            "trigger_order_nonmonotonic",
                            "Fraction trigger thresholds must be nondecreasing within a segment.",
                            idx,
                        )
                    )
                prev_frac = frac
            elif unit in {"", expected_unit}:
                if prev_abs is not None and threshold + 1e-9 < prev_abs:
                    issues.append(
                        ValidationIssue(
                            "error",
                            "trigger_order_nonmonotonic",
                            "Trigger thresholds must be nondecreasing within a segment.",
                            idx,
                        )
                    )
                prev_abs = threshold

    if strict:
        # If any hard errors exist, append a roll-up issue for UI display.
        if any(i.severity == "error" for i in issues):
            issues.append(
                ValidationIssue(
                    "error",
                    "export_blocked",
                    "Efficiency Pack export blocked until validation errors are resolved.",
                    -1,
                )
            )

    return issues


def convert_unsupported_timeline(
    timeline: Iterable[dict],
) -> Tuple[List[dict], List[ValidationIssue]]:
    """
    Legacy conversion hook for unsupported segments.

    Unsafe fallback conversion (unknown -> wait(0)) has been removed by design.
    Unsupported segments are preserved and returned with hard errors so export
    remains blocked unless the caller performs an explicit, user-confirmed
    conversion step.
    """
    converted: List[dict] = []
    issues: List[ValidationIssue] = []

    for idx, seg in enumerate(timeline):
        st = str(seg.get("type", "")).strip().lower()
        if st in SUPPORTED_SEGMENT_TYPES:
            converted.append(dict(seg))
            continue
        converted.append(dict(seg))
        issues.append(
            ValidationIssue(
                "error",
                "auto_conversion_removed",
                (
                    f"Unsupported segment '{st}' cannot be auto-converted. "
                    "Explicit conversion must be performed and confirmed before export."
                ),
                idx,
            )
        )

    return converted, issues
