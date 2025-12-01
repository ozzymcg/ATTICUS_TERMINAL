# mod/storage.py
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

def save_nodes(initial_state, nodes):
    """Save routine to JSON file."""
    filename = filedialog.asksaveasfilename(
        title="Save routine", 
        defaultextension=".json",
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    if not filename:
        return
    with open(filename, "w", encoding="utf-8") as f:
        json.dump({"initial": initial_state, "nodes": nodes}, f, indent=4)

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
    return data["initial"], data["nodes"]

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
