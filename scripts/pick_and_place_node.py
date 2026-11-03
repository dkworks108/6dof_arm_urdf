#!/usr/bin/env python3
"""
6-DOF Industrial Arm Pick and Place ROS 2 Controller Node

Controls the arm_controller and gripper_controller via FollowJointTrajectory actions
to execute a full pick-and-place sequence in Gazebo simulation.
"""

import math
import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PickAndPlaceController(Node):
    ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    GRIPPER_JOINTS = ['left_finger_joint', 'right_finger_joint']

    # Robot link lengths for closed-form downward IK
    L2 = 0.40       # Upper arm length
    L3 = 0.40       # Forearm link 3 + link 4 length to wrist pitch joint 5
    L_TOOL = 0.234   # Wrist pitch joint 5 to TCP tool center point

    def __init__(self, speed=1.0, loop=False):
        super().__init__('pick_and_place_controller')
        self.speed = max(0.2, min(5.0, speed))
        self.loop = loop

        self.get_logger().info('Initializing 6-DOF Pick-and-Place Controller Node...')

        # Action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Connect to action servers
        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory action server...')
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm trajectory action server not available! Is Gazebo / controller_manager running?')
            raise RuntimeError('Arm action server unavailable')

        self.get_logger().info('Waiting for /gripper_controller/follow_joint_trajectory action server...')
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper trajectory action server not available!')
            raise RuntimeError('Gripper action server unavailable')

        self.get_logger().info('Action servers successfully connected!')

    def solve_ik_down(self, x, y, z):
        """Analytical closed-form inverse kinematics for tool pointing straight down."""
        j1 = math.atan2(y, x)
        rw = math.sqrt(x*x + y*y)
        zw = z + self.L_TOOL
        dz = zw - 0.30  # shoulder joint 2 height
        D = math.sqrt(rw*rw + dz*dz)
        if D > (self.L2 + self.L3) * 0.999 or D < abs(self.L2 - self.L3):
            self.get_logger().error(f'Position ({x:.3f}, {y:.3f}, {z:.3f}) out of reach!')
            return None
        beta = math.atan2(dz, rw)
        cos_gamma = (self.L2*self.L2 + D*D - self.L3*self.L3) / (2 * self.L2 * D)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)
        j2 = (math.pi/2 - beta) - gamma
        cos_elbow = (self.L2*self.L2 + self.L3*self.L3 - D*D) / (2 * self.L2 * self.L3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_ext = math.acos(cos_elbow)
        j3 = math.pi - elbow_ext
        j5 = math.pi - j2 - j3
        return [j1, j2, j3, 0.0, j5, 0.0]

    def send_arm_trajectory(self, joint_angles, duration_sec):
        """Send trajectory goal to arm controller and wait for completion."""
        adjusted_duration = duration_sec / self.speed
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in joint_angles]
        point.velocities = [0.0] * len(self.ARM_JOINTS)
        sec = int(adjusted_duration)
        nanosec = int((adjusted_duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        goal_msg.trajectory.points.append(point)

        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal was rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def send_gripper_goal(self, left_finger_pos, duration_sec=1.5):
        """Send trajectory goal to parallel gripper controller."""
        adjusted_duration = duration_sec / self.speed
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.GRIPPER_JOINTS

        point = JointTrajectoryPoint()
        # Symmetric motion: left finger pos in [0, 0.035], right finger in [-0.035, 0]
        point.positions = [float(left_finger_pos), -float(left_finger_pos)]
        point.velocities = [0.0, 0.0]
        sec = int(adjusted_duration)
        nanosec = int((adjusted_duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        goal_msg.trajectory.points.append(point)

        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal was rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def execute_sequence(self):
        """Execute full 10-step pick and place sequence."""
        home_joints = [0.0, -0.7854, 1.5708, 0.0, 0.7854, 0.0]

        ik_pre_pick = self.solve_ik_down(0.45, 0.22, 0.30)
        ik_pick = self.solve_ik_down(0.45, 0.22, 0.165)
        ik_lift = self.solve_ik_down(0.45, 0.22, 0.35)
        ik_transit = self.solve_ik_down(0.45, 0.0, 0.38)
        ik_pre_place = self.solve_ik_down(0.45, -0.22, 0.35)
        ik_place = self.solve_ik_down(0.45, -0.22, 0.170)
        ik_retract_high = self.solve_ik_down(0.45, -0.22, 0.40)

        cycle = 1
        while rclpy.ok():
            self.get_logger().info(f'========== Starting Pick-and-Place Cycle #{cycle} ==========')

            # Step 1: Move to Home
            self.get_logger().info('[Step 1/10] Moving arm to HOME pose...')
            self.send_arm_trajectory(home_joints, duration_sec=3.0)

            # Step 2: Open Gripper
            self.get_logger().info('[Step 2/10] Opening gripper fingers (span = 70 mm)...')
            self.send_gripper_goal(0.035, duration_sec=1.5)

            # Step 3: Move to Pre-Pick
            self.get_logger().info('[Step 3/10] Moving arm to PRE-PICK pose above pick table...')
            self.send_arm_trajectory(ik_pre_pick, duration_sec=3.0)

            # Step 4: Descend to Pick Target
            self.get_logger().info('[Step 4/10] Descending gripper to envelope target object...')
            self.send_arm_trajectory(ik_pick, duration_sec=2.0)

            # Step 5: Close Gripper to Grasp Object
            self.get_logger().info('[Step 5/10] Grasping target object firmly...')
            self.send_gripper_goal(0.016, duration_sec=1.5)

            # Step 6: Lift Object
            self.get_logger().info('[Step 6/10] Lifting object vertically off pick table...')
            self.send_arm_trajectory(ik_lift, duration_sec=2.0)

            # Step 7: Transit to Destination
            self.get_logger().info('[Step 7/10] Smoothly transiting object across workspace...')
            self.send_arm_trajectory(ik_transit, duration_sec=2.5)
            self.send_arm_trajectory(ik_pre_place, duration_sec=2.5)

            # Step 8: Lower Object into Destination Place Tray
            self.get_logger().info('[Step 8/10] Lowering object onto place workstation...')
            self.send_arm_trajectory(ik_place, duration_sec=2.0)

            # Step 9: Open Gripper to Release Object
            self.get_logger().info('[Step 9/10] Releasing object inside target bin...')
            self.send_gripper_goal(0.035, duration_sec=1.5)

            # Step 10: Retract and Return Home
            self.get_logger().info('[Step 10/10] Retracting arm vertically and returning HOME...')
            self.send_arm_trajectory(ik_retract_high, duration_sec=2.0)
            self.send_arm_trajectory(ik_transit, duration_sec=2.0)
            self.send_arm_trajectory(home_joints, duration_sec=3.0)

            self.get_logger().info(f'========== Pick-and-Place Cycle #{cycle} COMPLETED ==========')

            if not self.loop:
                break
            cycle += 1
            time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(description='6-DOF Arm Pick and Place ROS 2 Controller')
    parser.add_argument('--speed', type=float, default=1.0, help='Execution speed multiplier (default: 1.0)')
    parser.add_argument('--loop', action='store_true', help='Loop pick and place sequence continuously')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    try:
        controller = PickAndPlaceController(speed=args.speed, loop=args.loop)
        controller.execute_sequence()
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
