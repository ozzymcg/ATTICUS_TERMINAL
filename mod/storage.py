from __future__ import annotations
import json, os, sys, subprocess
from tkinter import filedialog

def _open_path(path: str):
    """Open file with system default application."""
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", path])
        else:
            subprocess.Popen(["xdg-open", path])
    except Exception:
        pass

def _sanitize_preset_action(action: dict, key_name: str) -> dict | None:
    """Handle sanitize preset action."""
    name = str(action.get("name", "")).strip()
    if not name:
        return None
    values = action.get("values", [])
    if not isinstance(values, list):
        values = []
    values = [str(v) for v in values[:3]]
    value = action.get("value", None)
    if value in (None, "") and values:
        value = " ".join(values)
    return {
        key_name: "preset",
        "name": name,
        "state": action.get("state", None),
        "value": value,
        "values": values,
    }

def _sanitize_nodes_for_json(nodes):
    """Handle sanitize nodes for json."""
    out = []
    if not isinstance(nodes, list):
        return out
    for node in nodes:
        if not isinstance(node, dict):
            continue
        n = dict(node)
        src_actions = n.get("actions")
        if not isinstance(src_actions, list):
            src_actions = n.get("actions_out")
        if not isinstance(src_actions, list):
            src_actions = []
        actions = []
        for act in src_actions:
            if not isinstance(act, dict):
                continue
            t = str(act.get("type", "")).strip().lower()
            if t == "geom":
                t = "reshape"
            if t == "preset":
                preset = _sanitize_preset_action(act, "type")
                if preset is not None:
                    actions.append(preset)
                continue
            if t == "code":
                code = str(act.get("code", "")).strip()
                if code:
                    actions.append({"type": "code", "code": code})
                continue
            if t:
                a2 = dict(act)
                a2["type"] = t
                actions.append(a2)
        n["actions"] = actions
        n["actions_out"] = list(actions)

        events = n.get("edge_events", [])
        if not isinstance(events, list):
            events = []
        norm_events = []
        for ev in events:
            if not isinstance(ev, dict):
                continue
            try:
                t_evt = max(0.0, min(1.0, float(ev.get("t", 0.0))))
            except Exception:
                t_evt = 0.0
            enabled = bool(ev.get("enabled", True))
            raw_actions = ev.get("actions", [])
            if not isinstance(raw_actions, list):
                raw_actions = []
            ev_actions = []
            for ma in raw_actions:
                if not isinstance(ma, dict):
                    continue
                kind = str(ma.get("kind", ma.get("type", ""))).strip().lower()
                if kind == "preset":
                    preset = _sanitize_preset_action(ma, "kind")
                    if preset is not None:
                        ev_actions.append(preset)
                elif kind == "code":
                    code = str(ma.get("code", "")).strip()
                    if code:
                        ev_actions.append({"kind": "code", "code": code})
            if ev_actions:
                norm_events.append({"t": t_evt, "enabled": enabled, "actions": ev_actions})
        if norm_events:
            n["edge_events"] = norm_events
        else:
            n.pop("edge_events", None)

        out.append(n)
    return out

def save_nodes(initial_state, nodes):
    """Save routine to JSON file."""
    filename = filedialog.asksaveasfilename(
        title="Save routine", 
        defaultextension=".json",
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    if not filename:
        return
    clean_nodes = _sanitize_nodes_for_json(nodes)
    with open(filename, "w", encoding="utf-8") as f:
        json.dump({"initial": initial_state, "nodes": clean_nodes}, f, indent=4)

def load_nodes():
    """Load routine from JSON file."""
    filename = filedialog.askopenfilename(
        title="Load routine", 
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    if not filename:
        return None, None
    with open(filename, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data.get("initial", {}), _sanitize_nodes_for_json(data.get("nodes", []))

def compile_log(lines):
    """Export compiled output to text file."""
    filename = filedialog.asksaveasfilename(
        title="Export directives", 
        defaultextension=".txt",
        filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
    )
    if not filename:
        return
    with open(filename, "w", encoding="utf-8") as f:
        for line in lines:
            f.write(line + "\n")
    _open_path(filename)
    lines.clear()
