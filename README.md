# 02_6dof_industrial_arm: 6-DOF Industrial Manipulator with Parallel Gripper

A fully articulated 6-DOF industrial robot manipulator (UR5 / KUKA style) equipped with an articulated wrist and a functional two-finger parallel gripper. Designed for ROS 2 Humble, Gazebo Fortress (Ignition), and MoveIt 2 motion planning with complete Pick-and-Place simulation and physics validation.

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
- `arm_controller`: `joint_trajectory_controller` for `joint_1` through `joint_6` (MoveIt trajectory execution interface).
- `gripper_controller`: `joint_trajectory_controller` for `left_finger_joint` and `right_finger_joint` (MoveIt gripper action interface).
- `joint_position_controller` & `gripper_position_controller`: Forward group position controllers.

---

## Package Setup & Build

This package is configured as a standard ROS 2 `ament_cmake` package (`arm_6dof_description`) along with its MoveIt configuration package (`arm_moveit_config`).

```bash
cd /media/bumble/DATA_DISK/robotics/projects/02_6dof_industrial_arm

# Build packages
colcon build --base-paths . arm_moveit_config

# Source overlay
source install/setup.bash
```

---

## Gazebo Pick-and-Place Simulation

### 1. Launch Pick-and-Place Cell in Gazebo

Spawns the 6-DOF industrial manipulator mounted at a dedicated workstation featuring:
- **Pick Table**: Blue workstation table (`x=0.45, y=0.22, z=0.075`, top at `z=0.15m`) with visual alignment target.
- **Place Table & Tray**: Green destination workstation table (`x=0.45, y=-0.22, z=0.075`) with a yellow containment tray.
- **Target Graspable Cube**: Crimson physics object (`0.035 x 0.035 x 0.045m`, mass = 0.05kg) with high-friction contact surfaces.
- **ros2_control Hardware Bridge**: Automatic activation of `joint_state_broadcaster`, `arm_controller`, and `gripper_controller`.

```bash
# Launch Gazebo simulation in GUI mode
ros2 launch arm_6dof_description gazebo_pick_and_place.launch.py

# Launch Gazebo in headless mode (server only, low resource usage)
ros2 launch arm_6dof_description gazebo_pick_and_place.launch.py headless:=true

# Launch Gazebo + RViz2 together
ros2 launch arm_6dof_description gazebo_pick_and_place.launch.py rviz:=true
```

### 2. Run the Automated Pick-and-Place Action Node

In a separate terminal (with overlay sourced), execute the automated pick-and-place sequence:

```bash
source install/setup.bash

# Run single pick-and-place cycle
python3 scripts/pick_and_place_node.py

# Run with custom speed multiplier (e.g. 2x speed)
python3 scripts/pick_and_place_node.py --speed 2.0

# Run continuous pick-and-place loop
python3 scripts/pick_and_place_node.py --loop
```

The node sends synchronized `control_msgs/action/FollowJointTrajectory` actions to the arm and gripper controllers through 10 stages:
1. Move to `HOME` pose.
2. Open parallel gripper fingers (span = 70 mm).
3. Pre-pick trajectory above pick table (`z = 0.30m`).
4. Descend to envelope target cube (`z = 0.165m`).
5. Close gripper firmly on cube (`width = 32mm`).
6. Post-pick vertical lift (`z = 0.35m`).
7. Smooth workspace transit via midpoint clearance waypoint (`x = 0.45, y = 0.0, z = 0.38m`).
8. Descend into destination place tray (`x = 0.45, y = -0.22, z = 0.170m`).
9. Open gripper to release object inside tray.
10. Vertical retract and safe return to `HOME`.

---

## Standalone Visualization & MoveIt 2

### RViz2 Joint State Sliders

```bash
ros2 launch arm_6dof_description display.launch.py
```

### MoveIt 2 Motion Planning Demo

```bash
ros2 launch arm_moveit_config demo.launch.py
```

MoveIt planning states include:
- `home`: Upright ready pose
- `pick_ready`: Oriented above pick table
- `pick_grasp`: Positioned at grasp altitude
- `transit`: Clearance transit posture
- `place_ready`: Oriented above destination table
- `place_drop`: Positioned inside place tray
- `open` / `close`: Gripper finger states

---

## Verification Tests

```bash
# 1. URDF structure, arm joints, gripper joints & TCP validation
python3 test_robot.py

# 2. Physics sanity, inertia, limits, and PyBullet FK verification
python3 test_extra.py

# 3. Headless PyBullet Physics Pick-and-Place validation (with 9 snapshot captures)
python3 test_pick_and_place.py
```

All 9 step verification frames are automatically saved to `screenshots/pnp_6dof_step*.png`.
