import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # find urdf path (relative or from installed share)
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'robot.urdf')
    if not os.path.exists(urdf_path):
        try:
            from ament_index_python.packages import get_package_share_directory
            urdf_path = os.path.join(get_package_share_directory('arm_6dof_description'), 'urdf', 'robot.urdf')
        except Exception:
            pass

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rviz_config = os.path.join(os.path.dirname(__file__), '..', 'config', 'rviz.rviz')
    if not os.path.exists(rviz_config):
        try:
            from ament_index_python.packages import get_package_share_directory
            rviz_config = os.path.join(get_package_share_directory('arm_6dof_description'), 'config', 'rviz.rviz')
        except Exception:
            pass

    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args
        )
    ])
