# <img width="597" height="596" alt="Screenshot 2026-01-02 235826" src="https://github.com/user-attachments/assets/96e6ef74-4857-4868-845e-cf7e54e407fc" />

# ATTICUS TERMINAL

Atticus Terminal is a VEX Robotics autonomous editor that lets you express **intent in field geometry** (nodes, headings, paths, offsets, and action triggers), then compiles that intent into an **execution timeline**. That compiled timeline is used consistently for **simulation** (with overlays + collision checks), **conservative time estimation**, and **instant export/code generation** into your own library. Once you give the program your bot's geometry and sensor locations, you can can optionally export a customized MCL system into your own PROS code and tune it automatically using the terminal's algorithm.

## Purpose

- Building and iterating highly customizable, full autons visually and generating code **instantly**, saving weeks of manual tuning.
- Verifying clearances using a footprint of the robot's geometry and collision sampling before running on field.
- Estimating whether an auton fits inside a time window through physics-based, tunable time estimation.
- Exporting the same routine into different library styles (LemLib / JAR / Custom) using templates + mechanism presets.
- Customizable **MCL + EKF localization** tools for drift correction/recovery, with import-based (microSD) tuning automation.

---

## Feature overview

- **Nodes / edges / markers → compiled segments:** drive, turn/face, swing/arcline, boomerang, path-following, wait/buffer, marker actions waituntil();, and state toggles.
- **Mechanism-first targeting:** per-node offsets so you can place a *mechanism contact point* (intake/clamp/etc) on the field target, making alignment easy.
- **Footprint + reshape legality:** customized robot geometry and physics with a collision checker for field objects.
- **Path file creation:** custom path file creation with tunable velocity curves and multiple Catmull-Rom Spline types.
- **Export templates + presets:** templates map compiled segments into a chosen library and mechanism presets with custom code blocks, generating routine code instantly.

---

## Custom PROS MCL + EKF localization

Atticus Terminal includes an **MCL tab** for configuring an optional **Monte Carlo Localization w/ Extended Kalman Filter** drift-correction system, plus an automatic tuning system via microSD that support importing logs and exporting configuration/runtime artifacts.
See pre-exported docs/API of this system in the ProsMCL folder.

---

## Quick Start (git clone)

1. Clone this repository.
2. Open a terminal in this folder.
3. Install dependencies:
   - Windows: `py -m pip install -r requirements.txt`
   - macOS/Linux: `python3 -m pip install -r requirements.txt`
4. Run environment checks:
   - `py doctor.py` (Windows) or `python3 doctor.py` (macOS/Linux)
5. Launch:
   - `py main.py` (Windows) or `python3 main.py` (macOS/Linux)

### Notes by Platform
- Windows Store Python works, but pip commands should usually use `py -m pip ...`.
- Linux may require a tkinter package from your distro (for example `python3-tk`).
- If pygame import fails, rerun dependency install using the exact Python executable you launch with.

---

## Build (PyInstaller)

The included `atticus_terminal.spec` bundles project data so exported builds are portable.

- Install build deps: `py -m pip install -r requirements-build.txt` (or `python3 -m pip ...`)
- Build:
  - Windows: `py -m PyInstaller atticus_terminal.spec`
  - macOS/Linux: `python3 -m PyInstaller atticus_terminal.spec`

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
