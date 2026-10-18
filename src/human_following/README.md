# Human following / D435i

Startup → **Theo dõi** opens a popup above startup, like Destinations / New Map.
The popup shows the map, camera/detector/tracking status, camera source and motion
controls. It has no conversation panel, system-log panel or position sidebar.
ROS diagnostic output remains in `/tmp/tracking-*.log` and the terminal.
Closing the popup (including Escape) stops its owned launch group and releases
its own ROS context; startup stays open. Reopening starts a fresh tracking session.
A failed node stops the launch; missing status also stops it (60 seconds at
startup, 5 seconds after the first status). Restart after fixing the problem.

Startup → **Kiểm tra camera** opens a separate live colour-camera preview. This
launch only opens the selected camera and publishes frames; it does not load the
detector/tracker or publish robot motion. Closing the window stops its camera
process and releases the selected device.

## Python and build

The Python interpreter used by the installed ROS executable needs `numpy`,
`opencv-python` (or distro OpenCV), `ultralytics`, `torch`, `torchvision`, and
`pyrealsense2` for the RealSense backend. Installing these only in the UI venv
is insufficient if the ROS entry point uses `/usr/bin/python3`. Verify with that
interpreter before launching. The model is installed from `yolov8n.pt`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select human_following --symlink-install
source install/setup.bash
ros2 launch human_following system.launch.py camera_url:=realsense enable_motion:=false
```

RealSense uses the SDK colour stream, BGR8, 640×480 at 30 fps. It does not select
`/dev/video0` or assume a fixed video-node number. Select a device with
`camera_serial:=SERIAL` if multiple cameras are connected. Connection status
requires a real three-channel frame; detector readiness requires successful
model loading and inference. SDK/open/read/inference errors stop tracking,
release the camera, and appear in status and logs.

## Missing SDK on the ROS interpreter

The installed entry point in this workspace uses `/usr/bin/python3`, even when
startup runs in a venv. Check the actual interpreter:

```bash
head -1 install/human_following/lib/human_following/human_following_node
/usr/bin/python3 -c "import pyrealsense2 as rs; print(len(rs.context().query_devices()))"
```

The development machine now has `pyrealsense2==2.59.1.11053` installed for that
interpreter in the user's site-packages. A result of `0` means the SDK sees no
camera; installing the package alone does not establish a camera connection.

## Without the Python RealSense SDK

Find the **colour capture** interface and its supported modes first:

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl -d /dev/v4l/by-id/YOUR_COLOUR_DEVICE --list-formats-ext
ros2 launch human_following system.launch.py \
  camera_url:=/dev/v4l/by-id/YOUR_COLOUR_DEVICE \
  camera_format:=YUYV camera_width:=640 camera_height:=480 camera_fps:=30 \
  enable_motion:=false
```

Replace the placeholder with the verified colour interface; a by-id suffix alone
does not prove it carries colour. Use a supported format/resolution/fps from the
listing. Depth-only frames are rejected. Explicit numeric indices and URLs remain
supported for other cameras, but numeric indices are not stable across reconnects.

The UI source field applies on **Khởi động lại theo dõi**. Environment variables
`TRACKING_CAMERA_URL` and `TRACKING_CAMERA_SERIAL` set its initial source/serial.
Other camera parameters are available through the launch command above.

## Motion and acceptance checks

Motion defaults to **off**, including in the UI. To enable it, check **Cho phép robot
di chuyển**, then restart; CLI uses `enable_motion:=true`. Test initially with the
robot mechanically secured and the motion option off.

`system.launch.py` uses `use_nav2=False` and launches only the following node.
It publishes velocity exclusively to `/cmd_vel_sources/following`. Run exactly one
velocity arbiter (navigation/mapping bringup includes it; standalone:
`ros2 launch view_robot_pkg velocity_arbiter.launch.py`). The arbiter owns `/cmd_vel`,
requires health, gives teleop higher priority, and expires following commands
after 0.5 seconds. Following camera loss publishes a stop before releasing resources.

```bash
ros2 topic echo /tracking_status
ros2 topic echo /human_lock_status
ros2 topic echo /cmd_vel_sources/following
ros2 topic info /cmd_vel --verbose
```

Check these cases on hardware:

1. Click Theo dõi: colour camera connected, detector ready; no person → searching.
2. Present a person: locked status; velocities remain zero with motion disabled.
3. Unplug camera: visible error, zero velocity, launch exits; no leftover camera owner.
4. Reconnect and restart: colour frames and inference resume.
5. Close button, Escape and window close: launch/node gone, camera reusable, startup remains open.
6. Only after securing the robot, enable motion and verify the arbiter output and stop.

## Verification in the development session

The package builds. Automated tests cover colour-source selection, depth/no-frame
rejection, missing SDK, launch failure, descendant cleanup, motion gating and UI
return paths. Startup close-veto tests and popup cleanup/reopen tests are covered. A real ROS launch
reported missing `pyrealsense2` and terminated its node. This environment exposed
no `/dev/video*`; real D435i frames, YOLO inference, physical button interaction
and robot motion still require the hardware acceptance checks above.
