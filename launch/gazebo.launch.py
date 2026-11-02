import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package directory (supports both local git source and installed share)
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.xacro')
    controllers_yaml = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    if not os.path.exists(xacro_path):
        try:
            pkg_share = get_package_share_directory('arm_6dof_description')
            xacro_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
            controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
        except Exception:
            pass

    # Expand xacro to URDF string
    robot_desc = subprocess.check_output([
        'xacro', xacro_path,
        f'controllers_yaml:={controllers_yaml}',
        'gazebo_system:=ignition'
    ]).decode('utf-8')

    # 1. Robot State Publisher with use_sim_time=True
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 2. Launch Gazebo Fortress (Ignition) via ros_gz_sim
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_arm_6dof',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm_6dof',
            '-z', '0.0'
        ]
    )

    # 4. ROS-Gazebo parameter bridge for /clock (simulation time sync)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # 5. Spawner for Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 6. Spawner for Arm Trajectory Controller (6-DOF arm)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 7. Spawner for Gripper Controller (parallel gripper)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay arm and gripper controller spawning until joint_state_broadcaster is active
    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner]
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_sim_launch,
        spawn_entity,
        bridge_clock,
        joint_state_broadcaster_spawner,
        delay_controllers_spawner
    ])
