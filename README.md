# ROS2 Vision Hand Teleop

A ROS2-based vision-driven teleoperation system that maps real-time hand gestures (via MediaPipe) to a simulated humanoid robotic hand in RViz. Originally forked from: https://github.com/kimsniper/ros2_vision_hand_teleop

New integrations:
- MuJoCo Functionality

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

