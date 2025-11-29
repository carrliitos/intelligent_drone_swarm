# NEST - Networked Ensemble of Swarm Things

*A Vision-Based Verification of Crazyflie Formations*

## Team

- Benzon Carlitos Salazar (salazarbc24@uww.edu)
- Kevin King (kingk20@uww.edu)
- Anjali Gupta (guptaam23@uww.edu)
- Griffin Polly (pollygs14@uww.edu)

## Project Overview

NEST is a proof-of-concept **Crazyflie 2.x drone control system** that couples a Python ground-control application with 
an overhead **OpenCV vision module** to **verify** whether commanded formations and motions are actually achieved.

Instead of a hardware “mothership” drone, a **laptop-based mothership process** acts as the coordinator:

- It sends open-loop motion and formation commands to one or more Crazyflies (via the Crazyflie Python API).
- An overhead webcam and **ArUco-based detector** track the drones in the image plane.
- The vision module computes simple metrics (e.g., displacement, spacing, formation geometry) and reports **PASS/FAIL** 
for specific maneuvers.

The capstone demo focuses on:

- A **single-drone dynamic back-and-forth motion test**, where the drone flies forward and backward by a fixed distance 
  while the overhead camera verifies the commanded displacement.
- **Multi-drone formations** (line, triangle, square, oscillation, spin) that can be triggered from the ground-control 
  UI and are visualized in the same vision pipeline.

NEST is intentionally **verification-first**: vision is used to validate what the drones did, and can optionally be used 
for image-based “click-to-go” control, but the core contribution is the **vision-based evaluation of formations and 
maneuvers**.

## Objectives

- **Vision-Based Formation Verification**  
  Use OpenCV and ArUco markers to detect Crazyflie positions in the overhead camera view and compute simple metrics for:
  - Single-drone displacement along a commanded axis.
  - Multi-drone line, triangle, and square formations.
  - PASS/FAIL outcomes based on configurable tolerances.
- **Swarm and Single-Drone Motion Layer**  
  Build a Python controller around the Crazyflie Python API to support:
  - Single-drone hover, motion primitives, and scripted tests.
  - Multi-drone synchronized takeoff/landing.
  - Simple swarm formations (line, triangle, square, oscillation, spin).
- **Interactive Ground-Control UI**  
  Provide a PyGame-based UI that:
  - Shows the live camera feed with ArUco/vision overlays.
  - Displays keyboard hints and status (mode, formation, PASS/FAIL).
  - Supports manual control, swarm formations, click-to-go (IBVS mode), and the back-and-forth verification routine.
- **Reproducible Experiments and Logging**  
  Log telemetry, vision detections, and PASS/FAIL outcomes with timestamps to enable reproducible experiments and later 
  analysis.

## Demo Scenarios

### 1. Single-Drone Back-and-Forth Motion Test

The primary capstone demo is a **single Crazyflie** executing a scripted, vision-verified maneuver:

- Press **`B`** in the UI to trigger the **back-and-forth motion test**:
  - The drone takes off (if needed) to a configured hover height.
  - Moves **forward** by a target distance (default ≈ 0.5 m).
  - Dwells for a few seconds.
  - Moves **backward** by the same distance.
  - (Optionally) repeats for N cycles.
- The vision module:
  - Tracks the drone’s ArUco marker.
  - Measures image-plane displacement between “pre-move” and “post-move” positions for each forward/backward leg.
  - Compares measured displacement to the target with a tolerance.
  - Reports **PASS/FAIL** per leg.
- The UI:
  - Tints the video **green** on PASS and **red** on FAIL.
  - Shows a small HUD line with cycle index, leg (forward/backward), and
    measured displacement.

### 2. Swarm Formations

When running in swarm mode, you can command simple preset formations from thekeyboard.

During swarm mode:

- **`S`** – SWARM takeoff (all connected drones).
- **`K`** – SWARM land / exit swarm mode.

Press **`1`–`5`** (during swarm mode) to trigger preset formations:

- **`1`** – Line formation (horizontal)
- **`2`** – Triangle formation
- **`3`** – Square formation
- **`4`** – Oscillation formation
- **`5`** – Spinning formation

These are executed open-loop (no closed-loop CV control), but the same vision stack can log positions for later analysis.

## Controls (High-Level)

Single-drone commands:

- **`H`** – Autonomous hover (one-shot).
- **`G`** – Take off (enter manual mode).
- **`L`** – Land / exit manual mode.
- **`B`** – Back-and-forth motion test (vision-verified).

Swarm commands:

- **`S`** – SWARM takeoff (all drones).
- **`K`** – SWARM land / exit swarm mode.
- **`1`–`5`** – Line / Triangle / Square / Oscillation / Spinning formations.

Manual movement (single or swarm, when enabled):

- **Arrow keys** – Forward, Back, Left, Right.
- **`A` / `D`** – Yaw left / right.
- **`R` / `F`** – Altitude up / down.

Vision (when `vision` mode is enabled):

- **Left click** – Set waypoint / click-to-go (IBVS mode, when `control` is enabled).
- **`V`** – Toggle click-delta overlay.
- **`C`** – Clear click point.
- **`P`** – Start/stop video recording.
- Back-and-forth: video tints **green** on PASS, **red** on FAIL.

Exit:

- **Backspace** – Quit program.

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/carrliitos/intelligent_drone_swarm.git
cd intelligent_drone_swarm
````

### 2. Create and Activate a Virtual Environment

Linux/macOS:

```bash
python3 -m venv venv
source venv/bin/activate
```

Windows (PowerShell):

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

### 3. Install in Editable Mode

Dependencies are defined in `pyproject.toml`.

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
```

This sets up the `fly` command in your venv.

### 4. Run the Drone CLI

Basic usage examples:

```bash
# Single drone over UDP (no vision)
fly udp

# Single drone on radio channel 7 (no vision)
fly radio 7

# Single drone with vision-only (for verification, click-to-go disabled)
fly radio 7 vision

# Single drone with vision + IBVS “control” (click-to-go enabled)
fly radio 7 vision control

# Two-drone swarm on channels 7 and 8 (no vision)
fly swarm 7 8

# Two-drone swarm with vision (for logging/verification)
fly swarm 7 8 vision
```

For the **capstone demo**, the most relevant configuration is:

```bash
# Single-drone back-and-forth motion test with vision verification
fly radio 7 vision
```

Then press **`B`** in the UI to run the back-and-forth routine and watch the vision-based PASS/FAIL overlay.
