# auto_docking

ROS2 automatic docking package using LiDAR reflective strip detection + Nav2 opennav_docking.

## Requirements

```bash
sudo apt install ros-$ROS_DISTRO-opennav-docking
```

## Physical Setup

Mount two reflective strips on the dock, spaced **0.375m** apart. Your LiDAR must publish intensity values.

## Configuration

| File | Purpose |
|------|---------|
| `config/docking_server.yaml` | opennav_docking parameters |
| `config/dock_database.yaml` | Dock pose in map frame |

Edit `config/dock_database.yaml` to set your dock's `[x, y, theta]` in the map frame.

## Build

```bash
cd ~/workspace/MangoMobileRobot
colcon build --packages-select auto_docking
source install/setup.bash
```

## Run

**Launch all nodes (detector + controller + docking server):**
```bash
ros2 launch auto_docking auto_docking.launch.py
```

**Trigger docking:**
```bash
ros2 run auto_docking dock_robot
```

## Node Overview

| Node | Subscribes | Publishes |
|------|-----------|-----------|
| `dock_detector_node` | `/scan` | `/dock_pose`, `/dock_markers` |
| `dock_controller_node` | `/dock_pose`, `/odom` | `/cmd_vel` |
| `docking_server` | Nav2 stack | `/dock_robot` action |

## Detection Flow

```
/scan (LiDAR intensity) → dock_detector → /dock_pose → dock_controller → /cmd_vel
```

The detector finds two reflective clusters ~0.32m apart and computes the dock center pose.

## Tuning Parameters

In `launch/auto_docking.launch.py`:

- `intensity_threshold` — raise if false positives, lower if dock not detected
- `expected_strip_spacing` — must match physical strip distance on your dock
- `target_reverse_distance` — how far the robot reverses into the dock
- `reverse_speed` — approach speed (default 0.15 m/s)

## Troubleshooting

**Dock not detected:**
```bash
ros2 topic echo /scan | grep intensities
```
If `intensities: []` → your LiDAR doesn't publish intensity. Use AprilTag detection instead.

**Check dock pose is being published:**
```bash
ros2 topic echo /dock_pose
```

**Visualize in RViz:** Add `MarkerArray` display on topic `/dock_markers`.
