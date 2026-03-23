# <img width="597" height="596" alt="Screenshot 2026-01-02 235826" src="https://github.com/user-attachments/assets/96e6ef74-4857-4868-845e-cf7e54e407fc" />

# ATTICUS TERMINAL

Atticus Terminal is a VEX Robotics autonomous editor that lets you express **intent in field geometry** (nodes, headings, paths, offsets, and action triggers), then compiles that intent into an **execution timeline**. That compiled timeline is used consistently for **simulation** (with overlays + collision checks), **conservative time estimation**, and **instant export/code generation** into your own library. Once you give the program your bot's geometry and sensor locations, the terminal can also simulate the Atticus localization layer against the field map so you can validate sensor placement, gating, and writeback behavior before the robot touches the field.

## Purpose

- Building and iterating highly customizable, full autons visually and generating code **instantly**, saving weeks of manual tuning.
- Verifying clearances using a footprint of the robot's geometry and collision sampling before running on field.
- Estimating whether an auton fits inside a time window through physics-based, tunable time estimation.
- Exporting the same routine into different library styles (LemLib / JAR / Custom) using templates + mechanism presets.
- Built-in **Atticus localization** simulation tools for map-based drift correction around odometry using distance sensors and IMU heading.

---

## Feature overview

- **Nodes / edges / markers -> compiled segments:** drive, turn/face, swing/arcline, boomerang, path-following, wait/buffer, marker actions waituntil();, and state toggles.
- **Mechanism-first targeting:** per-node offsets so you can place a *mechanism contact point* (intake/clamp/etc) on the field target, making alignment easy.
- **Footprint + reshape legality:** customized robot geometry and physics with a collision checker for field objects.
- **Path file creation:** custom path file creation with tunable velocity curves and multiple Catmull-Rom Spline types.
- **Export templates + presets:** templates map compiled segments into a chosen library and mechanism presets with custom code blocks, generating routine code instantly.

---

## Atticus localization

Atticus Terminal includes an **Atticus tab** for configuring the simulator-side Atticus localizer used by the app. The localizer is an odometry-centered, map-based correction layer: distance sensors raycast against the field, translation is trimmed conservatively, heading remains IMU-dominant, and bounded writeback can be visualized directly in sim.

## Quick Start (git clone)

1. Clone this repository.
2. Open a terminal in this folder.
3. Install dependencies:
   - Windows: `py -m pip install .`
   - macOS/Linux: `python3 -m pip install .`
   - Alternate: `py -m pip install -r requirements.txt`
4. Run environment checks:
   - `py doctor.py` (Windows) or `python3 doctor.py` (macOS/Linux)
5. Launch:
   - `py main.py` (Windows) or `python3 main.py` (macOS/Linux)

### Notes by Platform
- On Windows, avoid the Microsoft Store "app execution alias" `python.exe` (it often breaks venv/build tooling). Prefer a python.org install and use `py ...`.
- Linux may require a tkinter package from your distro (for example `python3-tk`).
- On macOS, SDL (pygame) and Tk can conflict if SDL initializes first; this app now pre-initializes Tk on macOS to avoid `SDLApplication macOSVersion` crashes.
- If pygame import fails, rerun dependency install using the exact Python executable you launch with.

---

## Build (PyInstaller)

The included `atticus_terminal.spec` bundles project data so exported builds are portable.

- Install build deps: `py -m pip install -r requirements-build.txt` (or `python3 -m pip ...`)
- Build:
  - Windows: `py -m PyInstaller atticus_terminal.spec`
  - macOS/Linux: `python3 -m PyInstaller atticus_terminal.spec`

### Windows one-click build script

- `export\\build_exe.bat` (wraps `export\\build_exe.ps1`)
- Creates/uses a local `.venv`, installs build deps, runs PyInstaller, and outputs `ATTICUS_TERMINAL.exe` in the project root.

---

## Configuration behavior

- The app auto-selects a writable config path.
- In frozen builds it prefers a per-user config location.
- Override config file path with environment variable `ATTICUS_CONFIG`.

---

## License

See `LICENSE`.

## Support / Bug Reporting

Contact Ozzy of 15800A through most VEX Discord servers, or by @ozzymcgrath on Instragram.

