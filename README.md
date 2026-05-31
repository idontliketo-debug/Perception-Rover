# perception-rover

An autonomous waste collection rover running on ROS 2 Jazzy. It patrols a 12x12m arena, detects colored targets with an onboard camera, navigates toward them, and removes them from the environment.

## What it does

- Covers the arena in a lawnmower pattern using Nav2 waypoints
- Detects green waste targets through YOLOv5 ONNX inference with an HSV color-space fallback
- Combines camera detections with 360-degree LiDAR scans to estimate 3D world positions
- Drives to detected targets, pauses for simulated collection, then continues patrol
- Removes collected targets from the simulation through the Gazebo API

## Stack

| Component | Technology |
|-----------|-----------|
| ROS 2 | Jazzy |
| Navigation | Nav2 (NavFn planner, Regulated Pure Pursuit controller) |
| Mapping | Pre-built occupancy grid |
| Perception | YOLOv5-nano ONNX + OpenCV HSV thresholding |
| Sensor fusion | Camera-LiDAR angular correspondence |
| Simulation | Gazebo Harmonic (gz-sim) |
| Deployment | Docker + noVNC |

## Architecture

```
Camera /image → vision_node → /waste_bbox (green detection)
                                   ↓
LiDAR  /scan  → ──────────────→ lidar_node → /target (3D world pos)
                                                  ↓
Coverage waypoints → mission_orchestrator ←───────┘
                           ↓
                    Nav2 /navigate_to_pose
                           ↓
                    Gazebo (stop, collect, delete)
```

## Running

### Docker (recommended)

```bash
docker build -t perception_rover .
docker run -d --name perception_rover -p 6080:6080 -p 5901:5901 --privileged perception_rover \
  bash -c "vncserver :1 -geometry 1280x800 -depth 24 -xstartup /usr/bin/xterm && \
           websockify --web /usr/share/novnc 6080 localhost:5901 & tail -f /dev/null"
```

Open http://localhost:6080/vnc.html in your browser.

### Inside the container

```bash
cd /ros2_ws
colcon build --packages-select perception_rover
source install/setup.bash

# Full system (Gazebo + Nav2 + perception + mission)
ros2 launch perception_rover rover_navigation.launch.py
```

## Project structure

```
perception_rover/
├── perception_rover/          # Python source
│   ├── vision_node.py         # YOLOv5 + HSV detection
│   ├── lidar_node.py          # Camera-LiDAR sensor fusion
│   └── mission_orchestrator.py # State machine
├── launch/
│   └── rover_navigation.launch.py
├── config/
│   ├── bridge.yaml            # ros_gz_bridge topic mapping
│   └── nav2_params.yaml       # Nav2 controller/planner config
├── worlds/
│   └── perception_world.sdf   # Gazebo arena
├── maps/
│   ├── arena_map.yaml         # Pre-built occupancy grid
│   └── arena_map.pgm
└── models/
    ├── yolov5n.onnx           # YOLOv5 nano model
    └── tb3/
        └── tb3_waffle_rgb.sdf # TurtleBot3 Waffle with RGB camera
```

## Notes

- Uses a pre-built map instead of SLAM for predictable, deterministic navigation
- Waste boxes are placed at known positions; a detection triggers an intercept toward the nearest one
- Collection is simulated by teleporting the model underground using Gazebo's set_pose service
