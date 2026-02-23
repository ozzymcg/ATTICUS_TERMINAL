"""Replay diagnostics for Atticus Efficiency Pack runlogs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List

from .atticus_runlog import RunlogFrame

# Runlog flag convention shared with Atticus add-on runtime.
RUNLOG_FLAG_CORRECTION_APPLIED = 1 << 0
RUNLOG_FLAG_SLIP_ACTIVE = 1 << 1
RUNLOG_FLAG_LOST = 1 << 2
RUNLOG_FLAG_WRITEBACK_ATTEMPTED = 1 << 8
RUNLOG_FLAG_BLOCKED_INVALID = 1 << 9
RUNLOG_FLAG_BLOCKED_DISABLED = 1 << 10
RUNLOG_FLAG_BLOCKED_NO_CORRECTION = 1 << 11
RUNLOG_FLAG_BLOCKED_CONFIDENCE = 1 << 12
RUNLOG_FLAG_BLOCKED_LOST = 1 << 13
RUNLOG_FLAG_BLOCKED_SAFE_WINDOW = 1 << 14
RUNLOG_FLAG_BLOCKED_JUMP = 1 << 15
RUNLOG_FLAG_BLOCKED_STALE = 1 << 16
RUNLOG_BLOCKED_MASK = (
    RUNLOG_FLAG_BLOCKED_INVALID
    | RUNLOG_FLAG_BLOCKED_DISABLED
    | RUNLOG_FLAG_BLOCKED_NO_CORRECTION
    | RUNLOG_FLAG_BLOCKED_CONFIDENCE
    | RUNLOG_FLAG_BLOCKED_LOST
    | RUNLOG_FLAG_BLOCKED_SAFE_WINDOW
    | RUNLOG_FLAG_BLOCKED_JUMP
    | RUNLOG_FLAG_BLOCKED_STALE
)


def decode_correction_reasons(flags: int) -> List[str]:
    """Decode correction gate/applied reasons from runlog flag bits."""
    reasons: List[str] = []
    if flags & RUNLOG_FLAG_WRITEBACK_ATTEMPTED:
        reasons.append("writeback_attempted")
    if flags & RUNLOG_FLAG_BLOCKED_INVALID:
        reasons.append("blocked_invalid")
    if flags & RUNLOG_FLAG_BLOCKED_DISABLED:
        reasons.append("blocked_disabled")
    if flags & RUNLOG_FLAG_BLOCKED_NO_CORRECTION:
        reasons.append("blocked_no_correction")
    if flags & RUNLOG_FLAG_BLOCKED_CONFIDENCE:
        reasons.append("blocked_confidence")
    if flags & RUNLOG_FLAG_BLOCKED_LOST:
        reasons.append("blocked_lost")
    if flags & RUNLOG_FLAG_BLOCKED_SAFE_WINDOW:
        reasons.append("blocked_safe_window")
    if flags & RUNLOG_FLAG_BLOCKED_JUMP:
        reasons.append("blocked_jump")
    if flags & RUNLOG_FLAG_BLOCKED_STALE:
        reasons.append("blocked_stale")
    if (flags & RUNLOG_FLAG_CORRECTION_APPLIED) and (flags & RUNLOG_FLAG_WRITEBACK_ATTEMPTED):
        reasons.append("applied_gates_passed")
    elif flags & RUNLOG_FLAG_CORRECTION_APPLIED:
        reasons.append("applied")
    return reasons


def correction_applied_from_flags(flags: int, alpha: float) -> bool:
    """Return True when correction was applied in this frame."""
    return (flags & RUNLOG_FLAG_CORRECTION_APPLIED) != 0 or alpha > 1e-3


@dataclass
class DiagnosticIssue:
    """Single replay diagnostic finding."""

    severity: str
    code: str
    message: str
    time_ms: int
    seg_idx: int


def _moving_window(values: List[float], n: int) -> List[float]:
    """Simple trailing moving average."""
    if n <= 1:
        return list(values)
    out: List[float] = []
    acc = 0.0
    for i, v in enumerate(values):
        acc += v
        if i >= n:
            acc -= values[i - n]
        out.append(acc / float(min(i + 1, n)))
    return out


def run_diagnostics(frames: List[RunlogFrame]) -> List[DiagnosticIssue]:
    """Produce deterministic diagnostics from replay frames."""
    issues: List[DiagnosticIssue] = []
    if not frames:
        return issues

    peak_vals = [f.peakedness for f in frames]
    peak_ma = _moving_window(peak_vals, 10)

    low_conf_streak = 0
    high_late_streak = 0
    no_progress_streak = 0
    blocked_corr_streak = 0
    prev_blocked_mask = 0
    prev_corr_applied = False
    last_trig_idx = frames[0].trig_idx
    last_pose = (frames[0].fused_x, frames[0].fused_y)
    for i, f in enumerate(frames):
        peak = peak_ma[i]
        ess = f.ess_ratio
        # Canonical confidence invariant check.
        expect_peak = max(0.0, min(1.0, 1.0 - max(0.0, min(1.0, ess))))
        if abs(expect_peak - f.peakedness) > 0.08:
            issues.append(
                DiagnosticIssue(
                    "warning",
                    "confidence_invariant_violation",
                    "Peakedness diverges from 1-ESS_ratio; check adapter normalization.",
                    f.time_ms,
                    f.seg_idx,
                )
            )

        if peak < 0.2:
            low_conf_streak += 1
        else:
            if low_conf_streak >= 20:
                issues.append(
                    DiagnosticIssue(
                        "warning",
                        "low_confidence_streak",
                        f"Low peakedness streak lasted {low_conf_streak} frames.",
                        f.time_ms,
                        f.seg_idx,
                    )
                )
            low_conf_streak = 0

        if f.late_ms > 10.0:
            high_late_streak += 1
        else:
            if high_late_streak >= 8:
                issues.append(
                    DiagnosticIssue(
                        "warning",
                        "watchdog_pressure",
                        f"High loop lateness streak lasted {high_late_streak} frames.",
                        f.time_ms,
                        f.seg_idx,
                    )
                )
            high_late_streak = 0

        if f.alpha > 0.0 and abs(f.target_theta - f.fused_theta) > 20.0:
            issues.append(
                DiagnosticIssue(
                    "info",
                    "large_theta_correction",
                    "Large heading correction applied; consider tighter correction clamps.",
                    f.time_ms,
                    f.seg_idx,
                )
            )

        if f.slip > 0.5 and peak < 0.25:
            issues.append(
                DiagnosticIssue(
                    "info",
                    "slip_with_low_conf",
                    "Slip active while localization confidence is low.",
                    f.time_ms,
                    f.seg_idx,
                )
            )

        if i > 0:
            dx = f.fused_x - last_pose[0]
            dy = f.fused_y - last_pose[1]
            moved_in = math.hypot(dx, dy)
            dist_to_target = math.hypot(f.target_x - f.fused_x, f.target_y - f.fused_y)
            if f.seg_idx >= 0 and dist_to_target > 3.0 and moved_in < 0.03:
                no_progress_streak += 1
            else:
                if no_progress_streak >= 15:
                    issues.append(
                        DiagnosticIssue(
                            "warning",
                            "unintended_stop",
                            f"Robot showed no progress for {no_progress_streak} frames while segment remained active.",
                            f.time_ms,
                            f.seg_idx,
                        )
                    )
                no_progress_streak = 0

            if f.trig_idx != last_trig_idx and f.seg_idx >= 0:
                if dist_to_target < 1.0:
                    issues.append(
                        DiagnosticIssue(
                            "warning",
                            "late_trigger",
                            "Trigger fired after segment was already near completion.",
                            f.time_ms,
                            f.seg_idx,
                        )
                    )
                last_trig_idx = f.trig_idx

        correction_applied = correction_applied_from_flags(f.flags, f.alpha)
        blocked_mask = f.flags & RUNLOG_BLOCKED_MASK
        if blocked_mask != 0 and not correction_applied and blocked_mask != prev_blocked_mask:
            issues.append(
                DiagnosticIssue(
                    "info",
                    "correction_blocked_reason",
                    "Correction blocked due to: " + ", ".join(decode_correction_reasons(f.flags)),
                    f.time_ms,
                    f.seg_idx,
                )
            )
        if correction_applied and not prev_corr_applied:
            issues.append(
                DiagnosticIssue(
                    "info",
                    "correction_applied_reason",
                    "Correction applied with gate result: " + ", ".join(decode_correction_reasons(f.flags)),
                    f.time_ms,
                    f.seg_idx,
                )
            )
        prev_corr_applied = correction_applied
        prev_blocked_mask = blocked_mask if (blocked_mask != 0 and not correction_applied) else 0

        dist_to_target = math.hypot(f.target_x - f.fused_x, f.target_y - f.fused_y)
        if not correction_applied and peak >= 0.55 and dist_to_target > 4.0:
            blocked_corr_streak += 1
        else:
            if blocked_corr_streak >= 12:
                issues.append(
                    DiagnosticIssue(
                        "info",
                        "correction_blocked",
                        "Corrections stayed blocked while confidence was high and large target error remained; check jump/safe-window gates.",
                        f.time_ms,
                        f.seg_idx,
                    )
                )
            blocked_corr_streak = 0
        last_pose = (f.fused_x, f.fused_y)

    if no_progress_streak >= 15:
        last = frames[-1]
        issues.append(
            DiagnosticIssue(
                "warning",
                "unintended_stop",
                f"Robot showed no progress for {no_progress_streak} frames while segment remained active.",
                last.time_ms,
                last.seg_idx,
            )
        )
    if blocked_corr_streak >= 12:
        last = frames[-1]
        issues.append(
            DiagnosticIssue(
                "info",
                "correction_blocked",
                "Corrections stayed blocked while confidence was high and large target error remained; check jump/safe-window gates.",
                last.time_ms,
                last.seg_idx,
            )
        )

    return issues


def build_diagnostic_summary(frames: List[RunlogFrame]) -> Dict[str, float]:
    """Return compact summary for UI labels/reporting."""
    if not frames:
        return {
            "frames": 0.0,
            "avg_peakedness": 0.0,
            "avg_ess_ratio": 0.0,
            "avg_late_ms": 0.0,
            "corrections_applied": 0.0,
            "corrections_blocked_with_reason": 0.0,
        }
    n = float(len(frames))
    applied = 0.0
    blocked = 0.0
    for f in frames:
        if correction_applied_from_flags(f.flags, f.alpha):
            applied += 1.0
        elif (f.flags & RUNLOG_BLOCKED_MASK) != 0:
            blocked += 1.0
    return {
        "frames": n,
        "avg_peakedness": sum(f.peakedness for f in frames) / n,
        "avg_ess_ratio": sum(f.ess_ratio for f in frames) / n,
        "avg_late_ms": sum(f.late_ms for f in frames) / n,
        "corrections_applied": applied,
        "corrections_blocked_with_reason": blocked,
    }
