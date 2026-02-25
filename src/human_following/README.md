# Human Following Package

This package enables the robot to detect and follow humans using YOLOv8 and camera input.

## Features
- Real-time human detection using YOLOv8n
- Tracks the largest detected person
- Divides camera view into 3 zones (left, middle, right)
- Publishes velocity commands to `/cmd_vel` to follow the human

## Dependencies
Install Python dependencies:
```bash
pip install ultralytics opencv-python torch torchvision numpy
```

## Usage

### Terminal 1: Start micro-ROS Agent (if using ESP32)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/YOUR_DEVICE
```

### Terminal 2: Start Human Following Node
```bash
cd ~/thien_ws
source install/setup.bash
ros2 run human_following human_following_node
```

Or use the launch file:
```bash
ros2 launch human_following human_following.launch.py
```

### Terminal 3: Monitor (optional)
```bash
ros2 topic echo /cmd_vel
```

## How It Works
1. Camera captures frames at 640x480
2. YOLOv8 detects humans (class 0)
3. Tracker identifies the largest person and determines their zone
4. Controller computes velocities:
   - **Left zone**: Move forward + turn left
   - **Middle zone**: Move forward straight  
   - **Right zone**: Move forward + turn right
   - **No detection**: Stop
5. Publishes to `/cmd_vel` (same topic as joystick)

## Notes
- This node publishes to the same `/cmd_vel` topic as your joystick
- To use joystick control, stop this node first
- Camera source defaults to `/dev/video0` (webcam)
- Adjust speeds in `controller.py` if needed (default: linear=0.3, angular=0.5)
