# 02_6dof_industrial_arm: 6-DOF Industrial Manipulator with Parallel Gripper

A fully articulated 6-DOF industrial robot manipulator (UR5 / KUKA style) equipped with an articulated wrist and a functional two-finger parallel gripper. Designed for ROS 2 Humble, Gazebo Fortress (Ignition), and MoveIt 2 motion planning.

---

## Kinematics & Joint Structure

| Joint | Type | Axis | Parent -> Child | Motion | Limits |
| :--- | :---: | :---: | :---: | :--- | :---: |
| `joint_1` | Revolute | `[0, 0, 1]` (Z) | `base_link` -> `link_1` | Shoulder Pan (Yaw base rotation) | [-3.1416, +3.1416] rad |
| `joint_2` | Revolute | `[0, 1, 0]` (Y) | `link_1` -> `link_2` | Shoulder Lift (Pitch elevation) | [-2.094, +2.094] rad |
| `joint_3` | Revolute | `[0, 1, 0]` (Y) | `link_2` -> `link_3` | Elbow Flex (Forearm pitch) | [-2.618, +2.618] rad |
| `joint_4` | Revolute | `[0, 0, 1]` (Z) | `link_3` -> `link_4` | Wrist Roll 1 | [-3.1416, +3.1416] rad |
| `joint_5` | Revolute | `[0, 1, 0]` (Y) | `link_4` -> `link_5` | Wrist Pitch | [-2.094, +2.094] rad |
| `joint_6` | Revolute | `[0, 0, 1]` (Z) | `link_5` -> `link_6` | Wrist Roll 2 (Flange rotation) | [-6.2832, +6.2832] rad |
| `joint_tool0` | Fixed | - | `link_6` -> `tool0` | Industrial standard tool flange | - |
| `gripper_base_joint` | Fixed | - | `tool0` -> `gripper_base_link` | Gripper chassis mounting joint | - |
| `left_finger_joint` | Prismatic | `[0, 1, 0]` (Y) | `gripper_base_link` -> `left_finger_link` | Left finger sliding open/close | [0.0, 0.035] m |
| `right_finger_joint` | Prismatic | `[0, 1, 0]` (Y) | `gripper_base_link` -> `right_finger_link` | Right finger symmetric mimic motion | [-0.035, 0.0] m |
| `tcp_joint` | Fixed | - | `gripper_base_link` -> `tool_link` | Tool Center Point (TCP) frame | - |

---

## Controllers & ros2_control

Configured in [`config/controllers.yaml`](file:///media/bumble/DATA_DISK/robotics/projects/02_6dof_industrial_arm/config/controllers.yaml):
- `joint_state_broadcaster`: Broadcasts joint states at 100 Hz (`/joint_states`).
- `arm_controller`: `joint_trajectory_controller` for `joint_1`, `joint_2`, `joint_3`, `joint_4`, `joint_5`, `joint_6` (MoveIt trajectory execution interface).
- `gripper_controller`: `joint_trajectory_controller` for `left_finger_joint`, `right_finger_joint` (MoveIt gripper action interface).
- `joint_position_controller` & `gripper_position_controller`: Forward group position controllers.

---

## Package Setup & Build

This package is configured as a standard ROS 2 `ament_cmake` package (`arm_6dof_description`).

```bash
cd /media/bumble/DATA_DISK/robotics/projects/02_6dof_industrial_arm

# Build package
colcon build

# Source overlay
source install/setup.bash
```

---

## How to Run

### 1. View in RViz2 with Joint Control Sliders

```bash
# Direct launch
ros2 launch launch/display.launch.py

# Or via ROS 2 package
ros2 launch arm_6dof_description display.launch.py
```

### 2. Launch in Gazebo Simulator (Ignition / Fortress Physics + ros2_control)

```bash
# Direct launch
ros2 launch launch/gazebo.launch.py

# Or via ROS 2 package
ros2 launch arm_6dof_description gazebo.launch.py
```

---

## Verification Tests

```bash
# 1. URDF structure, arm joints, gripper joints & TCP validation
python3 test_robot.py

# 2. Physics sanity, inertia, limits, and PyBullet FK verification
python3 test_extra.py
```
