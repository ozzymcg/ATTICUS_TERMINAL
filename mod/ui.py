"""
Tkinter helpers and output window utilities for the Atticus terminal.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

tk_root = None
tk_settings_win = None
tk_output_win = None
tk_output_text = None
live_update_widgets = []


def ensure_tk_root():
    """Initialize Tkinter root window with styling."""
    global tk_root
    if tk_root is None or not (hasattr(tk_root, "winfo_exists") and tk_root.winfo_exists()):
        tk_root = tk.Tk()
        try:
            style = ttk.Style(tk_root)
            if "clam" in style.theme_names():
                style.theme_use("clam")
            style.configure("TFrame", padding=6)
            style.configure("TLabel", padding=2)
            style.configure("TEntry", padding=2)
            style.configure("TButton", padding=4)
            style.configure("TNotebook.Tab", padding=(12, 6))
            style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"))
            style.configure("Help.TLabel", foreground="#777777")
        except Exception:
            pass
        tk_root.withdraw()
    return tk_root


def pump_tk():
    """Update Tkinter event loop."""
    global tk_root, tk_settings_win, tk_output_win
    if tk_root is None:
        return
    try:
        tk_root.update()
        if tk_settings_win is not None and not tk_settings_win.winfo_exists():
            tk_settings_win = None
        if tk_output_win is not None and not tk_output_win.winfo_exists():
            tk_output_win = None
    except tk.TclError:
        tk_root = None
        tk_settings_win = None
        tk_output_win = None


class _Tooltip:
    """Hover tooltip for widgets."""

    def __init__(self, widget, text):
        self.widget = widget
        self.text = text
        self.tip = None
        widget.bind("<Enter>", self._show)
        widget.bind("<Leave>", self._hide)

    def _show(self, _=None):
        if self.tip or not self.text:
            return
        x = self.widget.winfo_rootx() + 20
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 8
        self.tip = tk.Toplevel(self.widget)
        self.tip.wm_overrideredirect(True)
        self.tip.geometry(f"+{x}+{y}")
        frame = ttk.Frame(self.tip, padding=6, style="TFrame")
        frame.pack()
        label = ttk.Label(frame, text=self.text, justify="left", wraplength=360)
        label.pack()

    def _hide(self, _=None):
        if self.tip:
            try:
                self.tip.destroy()
            except Exception:
                pass
            self.tip = None


def add_tooltip(widget, text):
    """Add tooltip to widget."""
    if text:
        _Tooltip(widget, text)


def track_live_widget(widget):
    """Collect widgets that should trigger live updates."""
    if widget not in live_update_widgets:
        live_update_widgets.append(widget)


def output_refresh(lines):
    """Refresh output window content."""
    global tk_output_text
    if tk_output_win and tk_output_text:
        try:
            tk_output_text.config(state="normal")
            tk_output_text.delete("1.0", "end")
            tk_output_text.insert("end", "\n".join(lines))
            tk_output_text.see("end")
            tk_output_text.config(state="disabled")
        except Exception:
            pass


def toggle_output_window(lines):
    """Toggle output window visibility."""
    global tk_output_win, tk_output_text
    ensure_tk_root()
    if tk_output_win and tk_output_win.winfo_exists():
        try:
            tk_output_win.destroy()
        except Exception:
            pass
        tk_output_win = None
        tk_output_text = None
        return
    top = tk.Toplevel(tk_root)
    tk_output_win = top
    top.title("Output")
    top.geometry("700x500")
    top.resizable(True, True)
    txt = tk.Text(top, wrap="word")
    txt.pack(fill="both", expand=True)
    txt.insert("end", "\n".join(lines))
    txt.config(state="disabled")
    tk_output_text = txt
    top.protocol("WM_DELETE_WINDOW", lambda: top.destroy())
