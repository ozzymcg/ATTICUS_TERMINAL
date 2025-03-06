![https://ibb.co/SCgxJgh](https://ibb.co/SCgxJgh)
# ATTICUS_TERMINAL

An advanced python-based interactive simulation and design tool for autonomous routines. It allows users to define a robot’s path by placing waypoints on a virtual VEX field, and then simulates realistic motion while providing visual, analytical feedback and detailed logging.

## Overview & Purpose

- **Path Definition:** Chart an autonomous path by placing nodes on a grid. Each node represents a waypoint where the robot can change direction, initiate a turn, or perform a special maneuver (e.g. picking up a mobile goal).
- **Visual Feedback & Logging:** Renders the robot’s position, heading, and key indicators (such as mobile-goal outlines) in real time. Detailed logs record distances, turning angles, travel times, and odometry data.
- **Automatic Routine Refinement:** The program refines nodes and paths so that the robot maximizes performance while staying within field boundaries.

For a complete breakdown of the simulation’s features and calculations, see the detailed [documentation](https://docs.google.com/document/d/1JHx0ViyM55vY7PmEMuhL2EYdnSzU5H1-IpEfy5h2yIw/edit?usp=sharing)

## Features

- **Grid-Based Node Placement:** Easily add, move, and delete nodes on a 12 ft × 12 ft virtual field.
- **Physics & Kinematics:** Compute acceleration, constant speed, and turning dynamics using standard kinematic equations.
- **Mobile-Goal Handling:** Calculate and adjust mobile-goal (mogo) positions using hexagon geometry.
- **PROS Mode Support:** Convert travel distances into different output units for integration with PROS (e.g. encoder degrees, rotations, and ticks).
- **Cross‑Platform Compatibility:** Works on both Mac and Linux using standard Python libraries.
- **Advanced Logging & State Management:** Includes undo/redo capabilities and detailed logging for autonomous routine analysis.

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/ozzymcg/atticus_terminal.git
   cd atticus_terminal
   ```

2. **Install Dependencies:**

   Ensure you have Python 3 installed. Then install the dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Running the Program

Start the simulation with:

```bash
python AUTONTERMINAL.py
```
Alternatively, create an executable:
```bash
python setup.py build`
```
The executable will be in the build folder.

## Basic Controls

- **Left-click:** Add or select a node.
- **Right-click/DELETE:** Remove a node.
- **H:** Set the initial robot heading.
- **SPACE:** Start or pause the simulation.
- **V:** Estimate autonomous routine time.
- **Q:** Toggle grid snapping.
- **B, F, R, G:** Access additional functions for curved pathing, mobile-goal actions, reverse mode, and clamp toggling.

## Configuration

A JSON configuration file (`auton_config.json`) is automatically created on first run. You can edit this file to adjust simulation parameters such as robot weight, drivetrain properties, and PROS mode options. It is highly recommended to configure these to be as accurate as possible.

## Information & Support

For detailed documentation—including the underlying physics, kinematics, PROS mode calculations, drivetrain offset adjustments, and advanced configuration options—please refer to the [Atticus Terminal Documentation](https://github.com/ozzymcg/atticus_terminal/blob/main/Documentation.md) or the provided documentation file.

For bug reporting or assistance, contact Austin/Ozzy of 15800A through most public VEX servers. (Feel free to mention this program in your notebook or interview, that would be much appreciated.)
