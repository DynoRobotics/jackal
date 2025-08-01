from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ROS2 Controllers
    control_group_action = GroupAction([

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/jackal/controller_manager'],
            output='screen',
        ),

        # Velocity Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['velocity_controller', '-c', '/jackal/controller_manager'],
            output='screen',
        )
    ])

    teleop_joy_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_control'), 'launch', 'teleop_joy.launch.py']
            )
        )
    )

    ld = LaunchDescription()
    ld.add_action(control_group_action)
    ld.add_action(teleop_joy_control)
    return ld
