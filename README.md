# ROS2 Vision Hand Teleop

A ROS2-based vision-driven teleoperation system that maps real-time hand gestures (via MediaPipe) to a simulated humanoid robotic hand in RViz.

This project integrates:
- Computer vision-based hand tracking using MediaPipe
- Gesture-to-joint mapping pipeline
- ROS2-based communication and control
- RViz visualization of a fully articulated robotic hand

---

## System Overview

The system consists of three main components:

### 1. Vision Layer (MediaPipe Node)
- Captures live webcam feed
- Extracts 21 hand landmarks using MediaPipe
- Publishes landmarks as ROS2 messages `/hand/landmarks`

### 2. Teleoperation Layer (Teleop Node)
- Receives hand landmarks
- Computes finger curl angles
- Maps gestures to 15 joint positions
- Publishes `/joint_states`

### 3. Visualization Layer (RViz + Robot State Publisher)
- Loads URDF hand model
- Visualizes joint states in real-time in RViz

---

## ROS2 Launch Modes

### Display Mode (Manual Joint Testing)
File: `display.launch.py`

Purpose:
Used for debugging and manual joint testing using GUI sliders.

Launches:
- robot_state_publisher
- joint_state_publisher_gui
- rviz2

Run:
```bash
ros2 launch robot_bringup display.launch.py
```

---

### MuJoCo Mode
File: `sim.launch.py`

```bash
ros2 launch robot_bringup sim.launch.py fake:=true   # synthetic hand, no camera
ros2 launch robot_bringup sim.launch.py              # real webcam
```

### Teleoperation Mode (Vision Control)
File: `teleop.launch.py`

Purpose:
Full vision-based teleoperation pipeline using MediaPipe.

Launches:
- robot_state_publisher
- robot_vision (MediaPipe node)
- robot_teleop (teleop node)
- rviz2

Run:
```bash
ros2 launch robot_bringup teleop.launch.py fake:=true   # no camera needed
ros2 launch robot_bringup teleop.launch.py
```

Calibrate once with your hand open, then in a fist:
```bash
ros2 service call /calibrate_open std_srvs/srv/Trigger
ros2 service call /calibrate_closed std_srvs/srv/Trigger
```

---

## Robot Model

URDF: `robot_description/urdf/robot_hand.urdf`

Features:
- 5-finger humanoid hand
- 15 revolute joints
- Realistic joint limits
- Simplified geometry for RViz visualization

---

## Build Instructions

### 1. Clone repository
```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/XDAlexSun/Oculus-Manus.git
cd ~/ws
```

### 2. Install dependencies
```bash
sudo apt install ros-$ROS_DISTRO-rviz2 \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui
```

Python dependencies:
```bash
pip install mediapipe opencv-python numpy mujoco
```

### 3. Build workspace
```bash
colcon build
```

### 4. Source setup
```bash
source install/setup.bash
```

---

## Topic Architecture

| Topic | Type | Description |
|------|------|-------------|
| /hand/world_landmarks | Float64MultiArray | 21 metric landmarks (63 floats); empty = no hand |
| /hand/finger_curl | Float64MultiArray | 5 curl values, 0 (open) to 1 (closed) |
| /hand/joint_command | JointState | 15 target joint angles for MuJoCo |
| /joint_states | JointState | same angles, used in RViz mode (robot_state_publisher) |
| /hand/sim_joint_states | JointState | angles the MuJoCo simulation actually reached |

---

## Package Structure

robot_bringup - Launch files  
robot_control - Control logic  
robot_description - URDF, RViz config  
robot_sim - MuJoCo bridge and node  
robot_teleop - Landmark, joint mapping  
robot_vision - MediaPipe vision node  

---

## How It Works

1. Webcam captures hand movement  
2. MediaPipe extracts 21 landmarks  
3. Finger curl estimation  
4. Mapping to joint angles  
5. Publish to ROS2 `/joint_states`  
6. RViz renders humanoid hand motion  

---

### Working Demo Video
![Demo](./images/mediapipe-ros2-demo.gif)


Vision Hand Teleop Demo: https://www.linkedin.com/feed/update/urn:li:activity:7461981548058144768/
