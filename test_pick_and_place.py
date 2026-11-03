#!/usr/bin/env python3
"""
6-DOF Industrial Arm: Physics Pick-and-Place Validation Suite

Performs automated headless physics simulation of the 6-DOF industrial manipulator
executing a complete pick, lift, transit, and place routine on a tabletop setup.
Captures verification snapshots at each milestone of the sequence.
"""

import os
import sys
import math
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image


def run_pick_and_place():
    print("=" * 60)
    print("  02_6dof_industrial_arm: Physics Pick-and-Place Validation  ")
    print("=" * 60)

    cur_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(cur_dir, 'urdf', 'robot.urdf')
    shots_dir = os.path.join(cur_dir, 'screenshots')
    os.makedirs(shots_dir, exist_ok=True)

    # Initialize PyBullet simulation in direct headless mode
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    # 1. Load Ground Plane
    plane_id = p.loadURDF("plane.urdf", [0, 0, 0])

    # 2. Load 6-DOF Robot Manipulator
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

    # Map joint names and link indices
    joint_name_to_id = {}
    link_name_to_id = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        j_name = info[1].decode('utf-8')
        l_name = info[12].decode('utf-8')
        joint_name_to_id[j_name] = i
        link_name_to_id[l_name] = i

    arm_joints = [
        joint_name_to_id['joint_1'],
        joint_name_to_id['joint_2'],
        joint_name_to_id['joint_3'],
        joint_name_to_id['joint_4'],
        joint_name_to_id['joint_5'],
        joint_name_to_id['joint_6']
    ]
    lf_joint = joint_name_to_id['left_finger_joint']
    rf_joint = joint_name_to_id['right_finger_joint']
    tool_link_idx = link_name_to_id['tool_link']

    # Set high friction on gripper finger pads
    for fid in [lf_joint, rf_joint]:
        p.changeDynamics(robot_id, fid, lateralFriction=5.0, spinningFriction=0.1, rollingFriction=0.01)

    # 3. Create Pick & Place Workstation Tables
    # Pick Table at (0.45, 0.22, 0.075), top surface at z = 0.15 m
    col_tbl1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.16, 0.16, 0.075])
    vis_tbl1 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.16, 0.16, 0.075], rgbaColor=[0.2, 0.35, 0.55, 1.0])
    pick_tbl_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_tbl1,
        baseVisualShapeIndex=vis_tbl1,
        basePosition=[0.45, 0.22, 0.075]
    )

    # Place Table at (0.45, -0.22, 0.075), top surface at z = 0.15 m
    col_tbl2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.16, 0.16, 0.075])
    vis_tbl2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.16, 0.16, 0.075], rgbaColor=[0.25, 0.50, 0.35, 1.0])
    place_tbl_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col_tbl2,
        baseVisualShapeIndex=vis_tbl2,
        basePosition=[0.45, -0.22, 0.075]
    )

    # Target Graspable Object: cylinder (radius 15mm, height 45mm, mass 40g)
    pick_x = 0.45
    pick_y = 0.22
    obj_z = 0.1725  # on top of pick table surface at 0.15m
    col_obj = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.015, height=0.045)
    vis_obj = p.createVisualShape(p.GEOM_CYLINDER, radius=0.015, length=0.045, rgbaColor=[0.9, 0.15, 0.15, 1.0])
    obj_id = p.createMultiBody(
        baseMass=0.04,
        baseCollisionShapeIndex=col_obj,
        baseVisualShapeIndex=vis_obj,
        basePosition=[pick_x, pick_y, obj_z]
    )
    p.changeDynamics(obj_id, -1, lateralFriction=5.0, spinningFriction=0.1, rollingFriction=0.01)

    # Robot kinematic constants
    L2 = 0.40
    L3 = 0.40
    L_tool = 0.234

    def solve_ik_down(x, y, z):
        """Analytical closed-form downward inverse kinematics."""
        j1 = math.atan2(y, x)
        rw = math.sqrt(x*x + y*y)
        zw = z + L_tool
        dz = zw - 0.30
        D = math.sqrt(rw*rw + dz*dz)
        if D > (L2 + L3) * 0.999 or D < abs(L2 - L3):
            return None
        beta = math.atan2(dz, rw)
        cos_gamma = (L2*L2 + D*D - L3*L3) / (2 * L2 * D)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)
        j2 = (math.pi/2 - beta) - gamma
        cos_elbow = (L2*L2 + L3*L3 - D*D) / (2 * L2 * L3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_ext = math.acos(cos_elbow)
        j3 = math.pi - elbow_ext
        j5 = math.pi - j2 - j3
        return [j1, j2, j3, 0.0, j5, 0.0]

    # Initialize robot to HOME pose
    home_pose = [0.0, -0.7854, 1.5708, 0.0, 0.7854, 0.0]
    for idx, j in enumerate(arm_joints):
        p.resetJointState(robot_id, j, home_pose[idx])
    p.resetJointState(robot_id, lf_joint, 0.035)
    p.resetJointState(robot_id, rf_joint, -0.035)

    def capture_frame(filename, camera_eye=[1.0, 0.7, 0.65], camera_target=[0.40, 0.0, 0.25]):
        view_mat = p.computeViewMatrix(camera_eye, camera_target, [0, 0, 1])
        proj_mat = p.computeProjectionMatrixFOV(55.0, 4.0/3.0, 0.1, 5.0)
        _, _, rgb, _, _ = p.getCameraImage(800, 600, view_mat, proj_mat, renderer=p.ER_TINY_RENDERER)
        img = Image.fromarray(np.reshape(rgb, (600, 800, 4))[:, :, :3])
        img.save(os.path.join(shots_dir, filename))

    current_arm = list(home_pose)
    current_grip = 0.035

    def move_to(arm_target, grip_target, steps=150):
        nonlocal current_arm, current_grip
        for s in range(1, steps + 1):
            alpha = s / steps
            for idx, j in enumerate(arm_joints):
                val = current_arm[idx] + alpha * (arm_target[idx] - current_arm[idx])
                p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, val, force=80.0, maxVelocity=2.0)
            g_val = current_grip + alpha * (grip_target - current_grip)
            p.setJointMotorControl2(robot_id, lf_joint, p.POSITION_CONTROL, g_val, force=40.0, maxVelocity=0.2)
            p.setJointMotorControl2(robot_id, rf_joint, p.POSITION_CONTROL, -g_val, force=40.0, maxVelocity=0.2)
            p.stepSimulation()
        current_arm = list(arm_target)
        current_grip = grip_target

    # Compute key trajectory waypoints
    ik_pre_pick = solve_ik_down(0.45, 0.22, 0.30)
    ik_pick = solve_ik_down(0.45, 0.22, 0.165)
    ik_lift = solve_ik_down(0.45, 0.22, 0.35)
    ik_transit = solve_ik_down(0.45, 0.0, 0.38)
    ik_pre_place = solve_ik_down(0.45, -0.22, 0.35)
    ik_place = solve_ik_down(0.45, -0.22, 0.170)
    ik_retract_high = solve_ik_down(0.45, -0.22, 0.40)

    # Let physics settle
    for _ in range(60):
        p.stepSimulation()

    print("\n[Step 1/9] Arm at HOME upright position...")
    capture_frame("pnp_6dof_step1_home.png")

    print("[Step 2/9] Move arm to PRE-PICK pose above pick table...")
    move_to(ik_pre_pick, 0.035, steps=120)
    capture_frame("pnp_6dof_step2_pre_pick.png")

    print("[Step 3/9] Descend arm to envelope target object...")
    move_to(ik_pick, 0.035, steps=120)
    capture_frame("pnp_6dof_step3_approach.png")

    print("[Step 4/9] Close gripper firmly around object...")
    move_to(ik_pick, 0.005, steps=150)
    contacts = p.getContactPoints(robot_id, obj_id)
    print(f" -> Contact points between fingers and object: {len(contacts)}")
    capture_frame("pnp_6dof_step4_grasp.png")

    print("[Step 5/9] Lift object off pick table...")
    move_to(ik_lift, 0.005, steps=150)
    lifted_pos, _ = p.getBasePositionAndOrientation(obj_id)
    print(f" -> Object height after lift: Z = {lifted_pos[2]:.3f} m (initial: {obj_z:.3f} m)")
    lift_success = lifted_pos[2] > 0.24
    print(f" -> Physical lift successful (>24 cm altitude): {lift_success}")
    capture_frame("pnp_6dof_step5_lift.png")

    print("[Step 6/9] Transiting object across workspace...")
    move_to(ik_transit, 0.005, steps=150)
    move_to(ik_pre_place, 0.005, steps=150)
    transit_pos, _ = p.getBasePositionAndOrientation(obj_id)
    print(f" -> Object position in transit: X={transit_pos[0]:.3f}, Y={transit_pos[1]:.3f}, Z={transit_pos[2]:.3f}")
    capture_frame("pnp_6dof_step6_transit.png")

    print("[Step 7/9] Lower object to place destination table...")
    move_to(ik_place, 0.005, steps=150)
    capture_frame("pnp_6dof_step7_place.png")

    print("[Step 8/9] Open gripper and release object...")
    move_to(ik_place, 0.035, steps=120)
    capture_frame("pnp_6dof_step8_release.png")

    print("[Step 9/9] Retract arm vertically and return to HOME...")
    move_to(ik_retract_high, 0.035, steps=120)
    move_to(ik_transit, 0.035, steps=120)
    move_to(home_pose, 0.035, steps=150)
    capture_frame("pnp_6dof_step9_retract.png")

    # Final placement evaluation
    for _ in range(60):
        p.stepSimulation()

    final_pos, _ = p.getBasePositionAndOrientation(obj_id)
    transport_dist = math.sqrt((final_pos[0] - pick_x)**2 + (final_pos[1] - pick_y)**2)
    print(f"\nFinal object placement coordinates: X={final_pos[0]:.3f}, Y={final_pos[1]:.3f}, Z={final_pos[2]:.3f}")
    print(f"Total transport distance: {transport_dist:.3f} m")

    # Criteria:
    # 1. Lift was achieved (> 0.24m)
    # 2. Object transported across (> 0.35m from initial pick table)
    # 3. Object settled safely on place table surface (z between 0.14m and 0.20m, y close to -0.22m)
    place_success = (transport_dist > 0.35) and (0.14 <= final_pos[2] <= 0.20) and (abs(final_pos[1] - (-0.22)) < 0.10)

    p.disconnect()

    if lift_success and place_success:
        print("\n" + "=" * 60)
        print(" PASS: Complete 6-DOF Industrial Arm Pick-and-Place Succeeded! ")
        print("=" * 60)
        return True
    else:
        print("\nFAIL: Pick-and-place sequence did not complete successfully.")
        return False


if __name__ == '__main__':
    ok = run_pick_and_place()
    sys.exit(0 if ok else 1)
