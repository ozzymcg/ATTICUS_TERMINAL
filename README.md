![Screenshot 2025-03-06 153523](https://github.com/user-attachments/assets/a6203b8f-e6f0-457e-a4ea-95d4199b1c20)
# ATTICUS TERMINAL: HIGH STAKES

An advanced python-based interactive simulation and design tool for autonomous routines. It allows users to define a robot’s path by placing waypoints on a virtual VEX field, and then simulates realistic motion while providing visual, analytical feedback and detailed logging.

## Overview & Purpose

- **Path Definition:** Chart an autonomous path by placing nodes on a grid. Each node represents a waypoint where the robot can change direction, initiate a turn, or perform a special maneuver (e.g. picking up a mobile goal).
- **Visual Feedback & Logging:** Renders the robot’s position, heading, and key indicators (such as mobile-goal outlines) in real time. Detailed logs record distances, turning angles, travel times, and odometry data.
- **Automatic Routine Refinement:** The program refines nodes and paths so that the robot maximizes performance while staying within field boundaries and considering customizable offsets.

![Screenshot 2025-03-06 155455](https://github.com/user-attachments/assets/73d57cd9-ebe1-4196-a9e1-2b4ef15fb90b)

## Features

- **Grid-Based Node Placement:** Easily add, move, and delete nodes on a 12 ft × 12 ft virtual field.
- **Mobile-Goal Handling:** Calculate and adjust mobile-goal (mogo) positions using hexagon geometry.
- **Bezier Curve Estimations:** Using physical variables, visually estimate brakeless node curves.
- **Optimization Offsetting:** Customizable offsets to acquire rings, mogos, and wall stakes perfectly.
- **PROS Mode Support:** Convert travel distances into different output units for integration with PROS (e.g. encoder degrees, rotations, and ticks).
- **Cross‑Platform Compatibility:** Works on both Mac and Linux using standard Python libraries.
- **Time Estimation:** Estimate time taken to physically run the routine through kinematics and advanced physical computations, allowing quick node rearrangement and routine optimizations.
- **Compiled Routine Directions & Data:** After running the routine, compile distances, angles, odometry, positions, etc. into a directions (.txt) file.

![Screenshot 2025-03-06 164258](https://github.com/user-attachments/assets/e6c44d22-49a3-4597-a8ee-2746fd1274e9)

## Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/ozzymcg/ATTICUS_TERMINAL.git
   cd ATTICUS_TERMINAL
   ```

2. **Install Dependencies:**

   Ensure you have Python 3 installed. Then install the dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Running the Program

Start the simulation with:

```bash
python ATTICUS_TERMINAL.py
```
Alternatively, create an executable:
```bash
python setup.py build
```
The executable will be in the build folder.

## Basic Controls

- **Left-click:** Add or select a node.
- **Right-click/DELETE:** Remove a node.
- **H:** Set the initial robot heading.
- **SPACE:** Start or pause the simulation.
- **V:** Estimate autonomous routine time.
- **Q:** Toggle grid snapping.
- **L/S** Load/Save a (.json) auton file.
- **C** After running a routine, compile directions and data into a (.txt) file.
- **B, F, R, G:** Access additional functions for curved pathing, ring/mogo/stake actions, reverse mode, and clamp toggling.

## Configuration

A JSON configuration file (`auton_config.json`) is automatically created on first run. You can edit this file to adjust simulation parameters.

### Configuration File Variable Descriptions

- **Physical Variables:**
  - **weight (lbs):** The robot's weight, which influences acceleration.
  - **rpm:** Drivetrain rotations per minute at 12 V (this is not the motor RPM).
  - **volts_straight:** Voltage used (# out of 12) during straight-line maneuvers.
  - **volts_turn:** Voltage used (# out of 12) during turning
  - **diameter (inches):** Wheel diameter
  - **t_buffer (seconds):** Delay buffer added before and after each move. Estimate this the best that you can.

- **Offsets:**
  - **clamp_offset_in (inches):** Distance from the robot’s closest drivetrain edge to the furthest internal edge of the mobile goal during clamp.
  - **ring_node_offset_in (inches):** Correction factor for ring/field-mogo nodes to prevent overshooting.
  - **wallstake_node_offset_in (inches):** Correction factor for wall stake nodes.
  - **padding_in (inches):** Space between mobile-goals and corner field boundaries to prevent collisions.

- **Orientation Settings:**
  - **plane_mode (Boolean):**
    - `True`: VEX conventional (Up = 0°, Right = 90°).
    - `False`: Unit circle (Up = 90°, Right = 0°).
  - **field_centric (Boolean):**
    - If `true`, odometry is computed relative to the field center (logical (3,3)); otherwise, relative to the initial robot position.

- **Bot Dimensions:**
  - **width & length (inches):** Physical dimensions of the robot.
  - **dt_width & dt_length (inches):** Drivetrain dimensions used for turning calculations.

- **Drivetrain Positioning:**
  - **x (inches) and y (inches):** Offsets to adjust the robot’s center relative to the drivetrain.

- **PROS Support:**
  - **pros_mode:** Determines output units:
    - `0`: Inches
    - `1`: Encoder degrees
    - `2`: Encoder rotations
    - `3`: Encoder counts/ticks
  - **gear_ratio:** The gear ratio of the drivetrain (e.g., 18 for an 18:1 gear setup).

## Information & Support

For detailed documentation—including the underlying physics, kinematics, calculations, offset adjustments, and advanced configuration options—please refer to the [Atticus Terminal Documentation](https://docs.google.com/document/d/1JHx0ViyM55vY7PmEMuhL2EYdnSzU5H1-IpEfy5h2yIw/edit?usp=sharing). This document additionally includes numerous LaTeX representations of underlying equations and computations.

For bug reporting or assistance, contact Austin/Ozzy of 15800A through most public VEX servers. (Feel free to mention this program in your notebook or interview, that would be much appreciated.)
