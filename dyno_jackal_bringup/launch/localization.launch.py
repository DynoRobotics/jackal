import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # # Launch configurations
    is_simulation = LaunchConfiguration('is_simulation', default=True)
    mapping = LaunchConfiguration('mapping', default=True)

    # # Launch arguments
    is_simulation_launch_argument = DeclareLaunchArgument(
        "is_simulation",
        default_value="true",
        description="Decides if the system should be launched in simulation mode.")

    mapping_argument = DeclareLaunchArgument(
        "mapping",
        default_value="true",
        description="Create new SLAM map or use existing map")


    # # Configs
    bringup_dir = get_package_share_directory('dyno_jackal_bringup')
    config_jackal_ekf = os.path.join(bringup_dir, "params", "ekf.yaml")
    config_jackal_localization = os.path.join(bringup_dir, 'params', 'localization.yaml')


    # Localization
    localization_group_action = GroupAction([
        # Extended Kalman Filter
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_jackal_ekf],
        ),
    ])

    slam_toolbox_mapping_or_localization = LaunchDescription([
        Node(
            condition=IfCondition(mapping),
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox_mapping',
            output='screen',
            parameters=[{'use_sim_time': is_simulation},
                        config_jackal_localization]),
                        
        Node(
        condition=UnlessCondition(mapping),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox_localization',
        output='screen',
        parameters=[{'use_sim_time': is_simulation},
                    config_jackal_localization])
    ])


    ld = LaunchDescription()

    ld.add_action(is_simulation_launch_argument)
    ld.add_action(mapping_argument)

    ld.add_action(localization_group_action)
    # ld.add_action(slam_toolbox_mapping_or_localization)

    return ld
