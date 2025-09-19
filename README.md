# NEST - Networked Ensemble of Swarm Things

## Team

- Benzon Carlitos Salazar (salazarbc24@uww.edu)
- Kevin King (kingk20@uww.edu)
- Anjali Gupta (guptaam23@uww.edu)
- Griffin Polly (pollygs14@uww.edu)

## Project Overview

This capstone project develops a proof-of-concept **Crazyflie 2.x drone swarm system**, where multiple drones equipped 
with the **Flowdeck V2** fly in coordinated patterns. Instead of a hardware mothership drone, an external **computer vision 
"mothership"** (implemented with a webcam and OpenCV) detects objects and motion in the environment, processes this 
information, and issues commands to the swarm. The objective is to demonstrate real-time swarm behaviors guided by 
vision-based input, enabling applications such as mapping, object tracking, and search-and-rescue scenarios.

## Objectives

- **Swarm Control Framework**: Build a Python-based swarm manager using the Crazyflie Python API to control multiple 
Crazyflie drones simultaneously, supporting hover, synchronized takeoff/landing, and simple formation behaviors.

- **Computer Vision Integration**: Implement an OpenCV pipeline to perform object detection and motion tracking through 
a webcam, serving as the centralized "mothership" brain for the swarm.

- **Vision-Guided Coordination**: Translate vision detections into coordinated drone actions, allowing the swarm to react 
autonomously to external stimuli (e.g., following a moving target or converging on a detected object).

- **Proof-of-Concept Demonstration**: Validate a centralized architecture where the external CV system issues commands to 
multiple Crazyflies, showcasing potential use cases in autonomous coordination.

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

Basic usage:

```bash
fly udp
fly radio 7
fly radio 8 vision
```
