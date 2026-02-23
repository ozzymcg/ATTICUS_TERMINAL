from __future__ import annotations

import pathlib
import tempfile

if __package__:
    from . import atticus_diagnostics
    from . import atticus_export_validation
    from . import atticus_pack_codegen
    from . import atticus_runlog
    from . import config
else:
    import os
    import sys

    _here = os.path.dirname(__file__)
    _root = os.path.normpath(os.path.join(_here, os.pardir))
    if _root not in sys.path:
        sys.path.insert(0, _root)
    from mod import atticus_diagnostics
    from mod import atticus_export_validation
    from mod import atticus_pack_codegen
    from mod import atticus_runlog
    from mod import config


def _assert(cond: bool, msg: str, errors: list[str]) -> None:
    if not cond:
        errors.append(msg)


def _check_trigger_units(errors: list[str]) -> None:
    bad_tl = [
        {"type": "turn", "T": 1.0, "edge_events": [{"unit": "in", "value": 5.0}]},
        {"type": "turn", "T": 1.0, "edge_events": [{"kind": "code", "code": "/*x*/;", "unit": "in", "value": 5.0}]},
    ]
    issues = atticus_export_validation.validate_efficiency_pack_timeline(bad_tl, strict=True)
    _assert(any(i.code == "trigger_unit_mismatch" for i in issues), "missing trigger unit mismatch detection", errors)
    bad_order = [
        {
            "type": "move",
            "T": 1.0,
            "edge_events": [
                {"unit": "in", "value": 10.0, "kind": "code", "code": "/*a*/;"},
                {"unit": "in", "value": 5.0, "kind": "code", "code": "/*b*/;"},
            ],
        }
    ]
    issues = atticus_export_validation.validate_efficiency_pack_timeline(bad_order, strict=True)
    _assert(any(i.code == "trigger_order_nonmonotonic" for i in issues), "missing trigger order nonmonotonic detection", errors)


def _check_default_export_blocks_unsupported(errors: list[str]) -> None:
    cfg = config.reload_cfg()
    tl = [{"type": "unsupported_magic_segment", "T": 1.0}]
    with tempfile.TemporaryDirectory() as td:
        blocked = False
        try:
            atticus_pack_codegen.write_efficiency_pack(cfg, tl, td)
        except Exception:
            blocked = True
        _assert(blocked, "default export should block unsupported segments when auto-convert is disabled", errors)
    tl_path = [{"type": "path", "T": 1.0}]
    with tempfile.TemporaryDirectory() as td:
        blocked = False
        try:
            atticus_pack_codegen.write_efficiency_pack(cfg, tl_path, td)
        except Exception:
            blocked = True
        _assert(blocked, "path segments should be blocked for Efficiency Pack export by default", errors)
    path_issues = atticus_export_validation.validate_efficiency_pack_timeline(tl_path, strict=True)
    _assert(any(i.code == "path_not_supported_in_efficiency_pack" for i in path_issues), "missing explicit path unsupported validation code", errors)


def _check_corridor_defaults(errors: list[str]) -> None:
    cfg = config.reload_cfg()
    tl = [{"type": "move", "p0": (0.0, 0.0), "p1": (24.0, 0.0), "T": 2.0}]
    with tempfile.TemporaryDirectory() as td:
        rep = atticus_pack_codegen.write_efficiency_pack(cfg, tl, td)
        _assert(len(rep.get("generated_files", [])) > 0, "pack export produced no files", errors)
        plan_hpp = pathlib.Path(td) / "include" / "atticus_generated" / "atticus_auton_plan.hpp"
        txt = plan_hpp.read_text(encoding="utf-8")
        _assert("8.0" in txt, "corridor half width default not exported", errors)
        _assert("0.25" in txt, "corridor soft penalty default not exported", errors)


def _check_fraction_trigger_conversion(errors: list[str]) -> None:
    cfg = config.reload_cfg()
    tl = [
        {
            "type": "move",
            "p0": (0.0, 0.0),
            "p1": (24.0, 0.0),
            "length_in": 24.0,
            "T": 2.0,
            "edge_events": [{"unit": "frac", "value": 0.5, "actions": [{"kind": "code", "code": "/*noop*/;"}]}],
        },
        {
            "type": "turn",
            "start_heading": 0.0,
            "target_heading": 90.0,
            "T": 1.0,
            "edge_events": [{"unit": "frac", "value": 0.5, "actions": [{"kind": "code", "code": "/*noop2*/;"}]}],
        },
        {
            "type": "wait",
            "T": 2.0,
            "edge_events": [{"unit": "frac", "value": 0.5, "actions": [{"kind": "code", "code": "/*noop3*/;"}]}],
        },
    ]
    with tempfile.TemporaryDirectory() as td:
        atticus_pack_codegen.write_efficiency_pack(cfg, tl, td)
        plan_hpp = pathlib.Path(td) / "include" / "atticus_generated" / "atticus_auton_plan.hpp"
        actions_cpp = pathlib.Path(td) / "src" / "atticus_generated" / "atticus_auton_actions.cpp"
        plan_txt = plan_hpp.read_text(encoding="utf-8")
        act_txt = actions_cpp.read_text(encoding="utf-8")
        _assert("TriggerUnit::Fraction" not in plan_txt, "fraction trigger leaked into runtime plan", errors)
        _assert("TriggerUnit::Inches" in plan_txt, "fraction trigger not converted to inches", errors)
        _assert("TriggerUnit::Degrees" in plan_txt, "fraction trigger not converted to degrees", errors)
        _assert("TriggerUnit::Milliseconds" in plan_txt, "fraction trigger not converted to milliseconds", errors)
        _assert("12.0" in plan_txt, "fraction trigger threshold not converted by segment extent", errors)
        _assert("45.0" in plan_txt, "turn fraction threshold not converted by heading extent", errors)
        _assert("1000.0" in plan_txt, "wait fraction threshold not converted by timeout extent", errors)
        _assert("case 1" in act_txt, "edge action case not generated", errors)


def _check_swing_side_flags(errors: list[str]) -> None:
    cfg = config.reload_cfg()
    tl = [{"type": "swing", "target_heading": 90.0, "side": "right", "T": 1.0}]
    segs, _trigs, _acts = atticus_pack_codegen._build_plan_data(tl, cfg)
    _assert(len(segs) == 1, "unexpected swing segment count", errors)
    _assert(int(segs[0].get("flags", 0)) == 1, "swing-right flag did not export", errors)


def _check_direct_event_action_forms(errors: list[str]) -> None:
    cfg = config.reload_cfg()
    tl = [
        {
            "type": "move",
            "p0": (0.0, 0.0),
            "p1": (24.0, 0.0),
            "T": 2.0,
            "edge_events": [
                {"kind": "code", "code": "/*inline_code*/;", "unit": "in", "value": 4.0},
                {"kind": "preset", "name": "intake_on", "unit": "in", "value": 8.0},
                {"kind": "action", "action": "clamp_close", "unit": "in", "value": 12.0},
                {"kind": "action_id", "action_id": 7, "unit": "in", "value": 16.0},
            ],
        }
    ]
    issues = atticus_export_validation.validate_efficiency_pack_timeline(tl, strict=True)
    _assert(not any(i.code == "missing_action_binding" for i in issues), "direct event action forms flagged as missing bindings", errors)

    with tempfile.TemporaryDirectory() as td:
        atticus_pack_codegen.write_efficiency_pack(cfg, tl, td)
        actions_cpp = pathlib.Path(td) / "src" / "atticus_generated" / "atticus_auton_actions.cpp"
        txt = actions_cpp.read_text(encoding="utf-8")
        _assert("Generated marker code action." in txt, "inline code action was not emitted", errors)
        _assert("Preset action: name=\"intake_on\"" in txt, "inline preset action was not emitted", errors)
        _assert("Preset action: name=\"clamp_close\"" in txt, "inline action-name payload was not emitted", errors)
        _assert("Legacy action_id payload (7)." in txt, "inline action_id payload was not emitted", errors)


def _check_confidence_invariant_diagnostics(errors: list[str]) -> None:
    # Build an in-memory runlog with one frame violating peakedness = 1 - ESS.
    head = atticus_runlog.HEADER_STRUCT.pack(
        atticus_runlog.RUNLOG_MAGIC,
        atticus_runlog.RUNLOG_SCHEMA_MAJOR,
        atticus_runlog.RUNLOG_SCHEMA_MINOR,
        atticus_runlog.HEADER_STRUCT.size,
        atticus_runlog.FRAME_STRUCT.size,
        1,
        0,
        0,
        0,
    )
    frame = atticus_runlog.FRAME_STRUCT.pack(
        100,  # time
        0,    # seg
        0,    # trig
        0,    # flags
        0.0, 0.0, 0.0,  # pose
        0.0, 0.0, 0.0,  # fused
        0.0, 0.0, 0.0,  # target
        0.1, 0.2, 0.0, 0.0,  # alpha/conf/slip/late
        300.0, 120.0, 0.4, 0.9,  # N/Neff/ESS/peakedness (violates: expected 0.6)
    )
    with tempfile.TemporaryDirectory() as td:
        p = pathlib.Path(td) / "test.atlrun"
        p.write_bytes(head + frame)
        _, frames = atticus_runlog.parse_runlog(str(p))
        issues = atticus_diagnostics.run_diagnostics(frames)
        _assert(any(i.code == "confidence_invariant_violation" for i in issues), "missing confidence invariant diagnostic", errors)


def _check_extended_diagnostics(errors: list[str]) -> None:
    F = atticus_runlog.RunlogFrame
    frames = []
    # Trigger "unintended_stop" and "correction_blocked": no movement, far from target, high confidence.
    for i in range(20):
        frames.append(
            F(
                time_ms=100 + i * 20,
                seg_idx=1,
                trig_idx=0,
                flags=0,
                pose_x=0.0,
                pose_y=0.0,
                pose_theta=0.0,
                fused_x=0.0,
                fused_y=0.0,
                fused_theta=0.0,
                target_x=24.0,
                target_y=0.0,
                target_theta=0.0,
                alpha=0.0,
                confidence=0.8,
                slip=0.0,
                late_ms=0.0,
                particle_count=300.0,
                neff=120.0,
                ess_ratio=0.4,
                peakedness=0.6,
            )
        )
    # Trigger "late_trigger": trigger index increments near completion.
    frames.append(
        F(
            time_ms=600,
            seg_idx=1,
            trig_idx=1,
            flags=0,
            pose_x=0.0,
            pose_y=0.0,
            pose_theta=0.0,
            fused_x=23.4,
            fused_y=0.0,
            fused_theta=0.0,
            target_x=24.0,
            target_y=0.0,
            target_theta=0.0,
            alpha=0.0,
            confidence=0.8,
            slip=0.0,
            late_ms=0.0,
            particle_count=300.0,
            neff=120.0,
            ess_ratio=0.4,
            peakedness=0.6,
        )
    )
    issues = atticus_diagnostics.run_diagnostics(frames)
    codes = {i.code for i in issues}
    _assert("unintended_stop" in codes, "missing unintended_stop diagnostic", errors)
    _assert("late_trigger" in codes, "missing late_trigger diagnostic", errors)
    _assert("correction_blocked" in codes, "missing correction_blocked diagnostic", errors)


def _check_runlog_forward_compat(errors: list[str]) -> None:
    # Emit a frame with a larger frame_size (minor-schema growth) and ensure parser reads prefix.
    base = atticus_runlog.FRAME_STRUCT.pack(
        100, 0, 0, 0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.1, 0.2, 0.0, 0.0,
        300.0, 120.0, 0.4, 0.6,
    )
    frame_size = len(base) + 8
    head = atticus_runlog.HEADER_STRUCT.pack(
        atticus_runlog.RUNLOG_MAGIC,
        atticus_runlog.RUNLOG_SCHEMA_MAJOR,
        atticus_runlog.RUNLOG_SCHEMA_MINOR,
        atticus_runlog.HEADER_STRUCT.size,
        frame_size,
        1,
        0,
        0,
        0,
    )
    with tempfile.TemporaryDirectory() as td:
        p = pathlib.Path(td) / "compat.atlrun"
        p.write_bytes(head + base + (b"\x00" * 8))
        _, frames = atticus_runlog.parse_runlog(str(p))
        _assert(len(frames) == 1, "forward-compatible runlog frame parsing failed", errors)


def main() -> int:
    errors: list[str] = []
    _check_default_export_blocks_unsupported(errors)
    _check_trigger_units(errors)
    _check_corridor_defaults(errors)
    _check_fraction_trigger_conversion(errors)
    _check_swing_side_flags(errors)
    _check_direct_event_action_forms(errors)
    _check_confidence_invariant_diagnostics(errors)
    _check_extended_diagnostics(errors)
    _check_runlog_forward_compat(errors)
    if errors:
        print("Atticus pack checks FAILED:")
        for e in errors:
            print(f"- {e}")
        return 1
    print("Atticus pack checks PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
