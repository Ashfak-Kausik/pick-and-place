# Franka Panda Fastener Detection & Sorting

A ROS2 + Gazebo simulation of a Franka Panda robotic arm that uses YOLOv12 to detect and sort industrial fasteners (clips, rivets, screws) into separate bins — replacing traditional HSV color detection with deep learning.

> 🔬 Based on research: *"Automated Industrial Fastener Detection: Systematic Benchmarking of YOLOv11 versus YOLOv12"* (under review)

## Demo
*Demo video coming soon*

## Tech Stack
| Tool | Purpose |
|------|---------|
| ROS2 Humble | Robot middleware |
| Gazebo | Physics simulation |
| MoveIt2 | Motion planning |
| YOLOv11 (Ultralytics) | Fastener detection |
| Franka Panda | 7-DOF robotic arm |
| Python 3 | Implementation |

## Detection Performance
| Metric | Score |
|--------|-------|
| mAP@0.5 | 98.98% |
| Recall | 100% |
| Classes | Clip, Rivet, Screw |
| Dataset | 630 images, 1442 instances |

## Project Structure
```
├── panda_bringup/       # Master launch file
├── panda_vision/        # YOLOv12 fastener detector node
│   └── panda_vision/
│       ├── fastener_detector.py   # YOLOv12 inference + ROS2 publisher
│       └── color_detector.py      # Original HSV detector (backup)
├── panda_description/   # Robot URDF, Gazebo world
├── panda_moveit/        # MoveIt2 config
├── panda_controller/    # Joint & gripper controllers
└── pymoveit2/           # Pick and place execution
    └── examples/
        └── pick_and_place.py      # YOLOv12-guided sorting
```

## Setup
```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src
git clone https://github.com/Ashfak-Kausik/franka-panda-fastener-detection.git
cd ~/panda_ws
pip3 install "numpy<2" transforms3d ultralytics
rosdep install --from-paths src --ignore-src -r -y
colcon build

source install/setup.bash
```

## Run
```bash
# Terminal 1 — Launch simulation
ros2 launch panda_bringup pick_and_place.launch.py

# Terminal 2 — YOLOv12 fastener detector
ros2 run panda_vision fastener_detector --ros-args -p target_fastener:=Screw

# Terminal 3 — Pick and place
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=Screw
```

Supported targets: `Clip`, `Rivet`, `Screw`

## Author
**MD Ashfakul Karim Kausik**  

## Acknowledgements
This project extends the Franka Panda Color Sorting Robot 
by [MechaMind-Labs](https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot),
replacing HSV color detection with a custom-trained YOLOv12 model 
for industrial fastener classification.
