# 6dof industrial arm

trying to make a 6 axis arm like a ur5 or kuka robot.
all 6 joints are revolute so it can reach any 3d position and angle.

joints:
- base to shoulder (pan)
- shoulder to upper arm (pitch)
- elbow (pitch)
- wrist 1, 2, 3
- tool0 (fixed frame at the end for attaching grippers)

how to run:
ros2 launch launch/display.launch.py

checked with check_urdf and all the limits work fine.
