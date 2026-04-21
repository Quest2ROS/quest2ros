# Quest2ROS — ROS2

ROS2 package for [Quest2ROS](https://github.com/Quest2ROS/quest2ros). Enables communication between a Meta Quest VR headset and ROS2: receive controller pose/velocity/input data and send haptic feedback.

> **ROS1 version** is on the `main` branch.


---

## Setup

### Prerequisites
- ROS2 (Humble or newer)
- [ros_tcp_endpoint](https://github.com/MWelle77/ROS-TCP-Endpoint) — patched ROS2 branch (`main-ros2`)

### 1 — Create a workspace and clone

```bash
mkdir -p ~/quest2ros_ws/src && cd ~/quest2ros_ws/src

# This package
git clone -b ros2 https://github.com/Quest2ROS/quest2ros.git

# ROS TCP Endpoint (ROS2 branch)
git clone -b main-ros2 https://github.com/MWelle77/ROS-TCP-Endpoint/tree/main-ros2
```

### 2 — Build

```bash
cd ~/quest2ros_ws
colcon build
source install/setup.bash
```

### 3 — Run the TCP endpoint

Replace `YOUR_MACHINE_IP` with your Ubuntu machine's IP address (the one reachable from the Quest over WiFi).

```bash
ros2 run ros_tcp_endpoint default_server_endpoint \
  --ros-args -p ROS_IP:=YOUR_MACHINE_IP -p ROS_TCP_PORT:=10000
```

### 4 — Run the demo node

```bash
source ~/quest2ros_ws/install/setup.bash
ros2 run quest2ros ros2quest.py
```

### 5 — Configure the Quest app

On the Meta Quest, open the Quest2ROS app:
1. Select ROS2 in the screen
1. Enter your Ubuntu machine's IP address
2. Set port to `10000`
4. Press **Apply**


---

## Topics

### Published by the Quest (subscribe on the ROS2 side)

| Topic | Type | Description |
|---|---|---|
| `q2r_right_hand_pose` | `geometry_msgs/PoseStamped` | Right controller position & orientation |
| `q2r_right_hand_twist` | `geometry_msgs/Twist` | Right controller linear & angular velocity |
| `q2r_right_hand_inputs` | `quest2ros/OVR2ROSInputs` | Right controller buttons & triggers |
| `q2r_left_hand_pose` | `geometry_msgs/PoseStamped` | Left controller position & orientation |
| `q2r_left_hand_twist` | `geometry_msgs/Twist` | Left controller linear & angular velocity |
| `q2r_left_hand_inputs` | `quest2ros/OVR2ROSInputs` | Left controller buttons & triggers |

### Subscribed by the Quest (publish from the ROS2 side)

| Topic | Type | Description |
|---|---|---|
| `q2r_right_hand_haptic_feedback` | `quest2ros/OVR2ROSHapticFeedback` | Right controller vibration |
| `q2r_left_hand_haptic_feedback` | `quest2ros/OVR2ROSHapticFeedback` | Left controller vibration |
| `dice_twist` | `geometry_msgs/Twist` | Velocity command for the Dice object in VR |
| `q2r_twist` | `geometry_msgs/Twist` | Velocity command for the Q2R object in VR |

---

## Custom Messages

**`OVR2ROSInputs`**
```
bool button_upper
bool button_lower
float32 thumb_stick_horizontal
float32 thumb_stick_vertical
float32 press_index
float32 press_middle
```

**`OVR2ROSHapticFeedback`**
```
float32 frequency
float32 amplitude
```

---

## Cordinate frame allignment

By pressing A + B on the right and X+Y on the left the relative cordinatframe gets alligned to the current controller position.
To use this for robot teleop allign the controller with the base frame of the robot and press the buttons.

---

## Bibtex

```
@inproceedings{@software{q2r2023,
  title={Quest2ROS: An App to Facilitate Teleoperating Robots},
  author={Welle, Michael C and Ingelhag, Nils and Lippi, Martina and Wozniak, Maciej K. and Gasparri, Andrea and Kragic, Danica},
  url = {https://quest2ros.github.io/q2r-web/},
  version = {1.0},
  date={2023-12-01}
}
```