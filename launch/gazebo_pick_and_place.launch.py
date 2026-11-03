import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve package directory (supports both direct source path and installed share)
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xacro_path = os.path.join(pkg_dir, 'urdf', 'robot.xacro')
    controllers_yaml = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    default_world = os.path.join(pkg_dir, 'worlds', 'pick_and_place.sdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz.rviz')

    if not os.path.exists(xacro_path):
        try:
            pkg_share = get_package_share_directory('arm_6dof_description')
            xacro_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
            controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
            default_world = os.path.join(pkg_share, 'worlds', 'pick_and_place.sdf')
            rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
        except Exception:
            pass

    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (server only, no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 along with Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to SDF world file'
        )
    ]

    headless_arg = LaunchConfiguration('headless')
    rviz_arg = LaunchConfiguration('rviz')
    world_arg = LaunchConfiguration('world')

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

    # 2. Gazebo Fortress (Ignition) via ros_gz_sim
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Construct gz_args dynamically based on headless argument
    gz_args = PythonExpression([
        "'-s -r ' + '", world_arg, "' if '", headless_arg, "' == 'true' else '-r ' + '", world_arg, "'"
    ])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # 3. Spawn robot entity into the pick and place world
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_arm_6dof',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'arm_6dof',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ]
    )

    # 4. ROS-Gazebo parameter bridge for /clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Sequence controllers after joint_state_broadcaster finishes initialization
    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner]
        )
    )

    # 6. Optional RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz_arg)
    )

    return LaunchDescription(
        declared_args + [
            node_robot_state_publisher,
            gz_sim_launch,
            spawn_entity,
            bridge_clock,
            joint_state_broadcaster_spawner,
            delay_controllers_spawner,
            rviz_node
        ]
    )

