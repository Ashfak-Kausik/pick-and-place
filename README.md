# YOLOv12-Guided Fastener Sorting — Franka Panda ROS2 Simulation

A proof-of-concept robotic integration study demonstrating end-to-end deployment of YOLOv12 for autonomous fastener segregation using a Franka Panda 7-DOF manipulator in ROS2 Humble + Ignition Gazebo.

> Companion repository to the research paper:
> *"Zero False-Negatives Under Data Scarcity: Can YOLO Architectures Bridge the Reliability Gap in Resource-Constrained Industrial Inspections?"* (under review)
>
> Dataset: [IFDD on Roboflow Universe](https://universe.roboflow.com/manufacturing-bot/manufacturing-industrial-robot)

---
## Simulation Screenshots

![YOLO Detection](media/Screenshot%20from%202026-04-25%2004-29-29.png)
*YOLOv11 detecting fastener classes*

![Fastener Detector](media/Screenshot%20from%202026-04-25%2004-29-11.png)
*Fastener detection overlay*

![Terminal Output](media/Screenshot%20from%202026-04-25%2004-28-35.png)
*Detection terminal output*

![Arm at Pick Position](media/Screenshot%20from%202026-04-25%2007-04-05.png)
*Franka Panda arm reaching grasp position*

![Arm Motion](media/Screenshot%20from%202026-04-25%2007-04-28.png)
*Franka Panda arm executing pick motion*

![Gazebo Simulation](media/Screenshot%20from%202026-04-25%2006-16-51.png)
*Gazebo simulation environment*


## Overview

This simulation validates that YOLOv12's detection capability — trained on a minimal 1,442-instance dataset — can drive class-conditional robotic pick-and-place operations with zero misclassification errors across all attempted placements. The system detects all three fastener classes simultaneously and autonomously routes each to its designated bin.

**Key result from the integration study:**
- 858 detections logged across a single session
- 10 successful pick-and-place operations completed
- **100% correct bin assignment** — zero objects placed in wrong bin
- 3 planning failures due to workspace boundary violations (not detection errors)
- Full detection log available: [`pick_place_detections.txt`](./pick_place_detections.txt)

> **Simulation note:** Object transport is implemented via Ignition Gazebo's native `/world/empty_world/set_pose` service due to the absence of a compatible rigid-body attachment plugin for ROS2 Humble. This is a known simulation constraint and does not affect the detection or motion planning results.

---

## Tech Stack

| Tool | Version | Purpose |
|---|---|---|
| ROS2 | Humble | Robot middleware |
| Ignition Gazebo | 6 (Fortress) | Physics simulation |
| MoveIt2 | Humble | Motion planning |
| YOLOv12 (Ultralytics) | Latest | Fastener detection |
| Franka Panda | 7-DOF | Robotic manipulator |
| Python | 3.10 | Implementation |

---

## Detection Performance (YOLOv12 on IFDD)

| Metric | Value |
|---|---|
| mAP@0.5 | 98.98% |
| mAP@0.5:0.95 | 96.22% |
| Recall | 100.00% |
| Cohen's Kappa (κ) | 1.000 |
| Classes | Clip, Rivet, Screw |
| Training instances | 1,317 |
| Validation instances | 125 |

---

## Integration Study Results

| Metric | Value |
|---|---|
| Total detection messages | 858 |
| Detection rate | ~8.3 Hz |
| Pick attempts | 13 |
| Successful placements | 10 (76.9%) |
| Correct bin assignments | 10 / 10 (100%) |
| Wrong-bin placements | 0 |
| Planning failures | 3 |

---

## Project Structure

├── panda_bringup/          # Master launch file
├── panda_vision/           # YOLOv12 detection node
│   └── panda_vision/
│       └── fastener_detector.py   # Inference + /color_coordinates publisher
├── panda_description/      # Robot URDF, Gazebo world SDF
├── panda_moveit/           # MoveIt2 configuration
├── panda_controller/       # Joint and gripper controllers
├── pymoveit2/              # Pick-and-place execution
│   └── examples/
│       └── pick_and_place.py      # Autonomous multi-class sorting node
├── pick_place_detections.txt      # Full detection log from integration study
└── Dockerfile

---

## Setup

```bash
# Create workspace
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/Ashfak-Kausik/pick-and-place.git
cd ~/panda_ws

# Install dependencies
pip3 install "numpy<2" transforms3d ultralytics
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

---

## Run

```bash
# Terminal 1 — Launch simulation (Gazebo + RViz + detector)
ros2 launch panda_bringup pick_and_place.launch.py

# Terminal 2 — Autonomous pick-and-place (runs after Gazebo loads)
ros2 run pymoveit2 pick_and_place.py

# Optional — Log detections
ros2 topic echo /color_coordinates | tee detections.txt
```

The system runs fully autonomously. No target class parameter needed — all three classes are detected and sorted simultaneously.

---

## How It Works

Camera image → YOLOv12 inference →
Class + bounding box →
Pixel-to-world coordinate conversion →
MoveIt2 motion planning →
Pre-grasp → Grasp → Lift → Bin placement

Each detected fastener is queued and processed sequentially. The correct destination bin is determined entirely by YOLOv12's class prediction — no hardcoded position mapping.

---

## Authors

**MD Ashfakul Karim Kausik** — MIST
**Tahsin Ahmed Refat** — MIST
**Md. Saif Alvi** — KUET

---

## Acknowledgements

Extended from [Franka Panda Color Sorting Robot](https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot) by MechaMind-Labs, replacing HSV color detection with a custom-trained YOLOv12 model for semantic fastener classification.
