# Atticus Terminal

Cross-platform path planning and simulation UI for VEX workflows.

## Quick Start (Git Clone)

1. Clone this repository.
2. Open a terminal in this folder.
3. Install dependencies:
   - Windows: `py -m pip install -r requirements.txt`
   - macOS/Linux: `python3 -m pip install -r requirements.txt`
4. Run environment checks:
   - `py doctor.py` (Windows) or `python3 doctor.py` (macOS/Linux)
5. Launch:
   - `py main.py` (Windows) or `python3 main.py` (macOS/Linux)

## Notes by Platform

- Windows Store Python works, but pip commands should usually use `py -m pip ...`.
- Linux may require a tkinter package from your distro (for example `python3-tk`).
- If pygame import fails, run dependency install again using the exact Python executable you launch with.

## Build (PyInstaller)

The included `atticus_terminal.spec` bundles project data (docs, pros templates, configs, assets) so exported builds are portable.

- Build command:
  - Install build deps: `py -m pip install -r requirements-build.txt` (or `python3 -m pip ...`)
  - `py -m PyInstaller atticus_terminal.spec` (Windows)
  - `python3 -m PyInstaller atticus_terminal.spec` (macOS/Linux)

## Configuration Behavior

- The app auto-selects a writable config path.
- In frozen builds it prefers a per-user config location.
- You can override config file path with environment variable `ATTICUS_CONFIG`.
