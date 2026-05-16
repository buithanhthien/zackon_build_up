# Zackon Phone App

Flutter app to view the robot map and live pose over 4G.

## Structure

```
zackon_build_up/
├── tool/
│   ├── robot_bridge_node.py   ← runs on robot PC (ROS 2 node)
│   └── vps_server.py          ← runs on VPS
└── phone_app/                 ← this Flutter project
    ├── lib/main.dart
    └── pubspec.yaml
```

## Setup

### 1. VPS server
```bash
pip install fastapi uvicorn python-multipart
uvicorn vps_server:app --host 0.0.0.0 --port 8000
```
Open port 8000 in your VPS firewall.

### 2. Robot bridge (on robot PC)
```bash
pip install numpy pillow requests websocket-client
python3 tool/robot_bridge_node.py --vps http://YOUR_VPS_IP:8000
```
Requires ROS 2 sourced and Nav2 running (`/map` + `/amcl_pose` active).

### 3. Phone app
1. Edit `lib/main.dart` — replace `YOUR_VPS_IP:8000` with your VPS IP.
2. Build and install:
```bash
cd phone_app
flutter pub get
flutter run          # or: flutter build apk
```
