import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ## Declare Launch arguments
    is_simulation_launch_arg = DeclareLaunchArgument(
        "is_simulation",
        default_value="true",
        description="Decides if the system should be launched in simulation mode."
    )

    declare_namespace_launch_arg = DeclareLaunchArgument(
        "namespace",
        default_value="jackal",
        description="Namespace for all topics and some params in ekf node"
    )

    declare_odom_frame_name = DeclareLaunchArgument(
        "odom_frame",
        default_value=[LaunchConfiguration("namespace"), "/odom"],
        description="The odom frame name (parameter value) that will be set for ekf_node",
    )

    declare_base_frame_name = DeclareLaunchArgument(
        "base_link_frame",
        default_value=[LaunchConfiguration("namespace"), "/base_link"],
        description="The odom frame name (parameter value) that will be set for ekf_node",
    )

    ## Configs
    config_jackal_ekf = os.path.join(
        get_package_share_directory('dyno_jackal_bringup'),
        "params",
        "ekf.yaml"
    )

    # Localization
    localization_group_action = GroupAction([
        Node( # Extended Kalman Filter
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                config_jackal_ekf,
                {
                    "odom_frame": LaunchConfiguration("odom_frame"), 
                    "base_link_frame": LaunchConfiguration("base_link_frame"),
                    "world_frame": LaunchConfiguration("odom_frame"),
                    "use_sim_time": LaunchConfiguration("is_simulation")
                },
            ],
        ),
    ])

    # Connect Jackal to Rviz frame.
    static_transform = GroupAction([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", LaunchConfiguration("odom_frame")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_world",
            arguments=["0", "0", "0", "0", "0", "0", "map", "world"],
        )
    ])

    # # Launch configurations
    # is_simulation = LaunchConfiguration('is_simulation', default=True)
    # mapping = LaunchConfiguration('mapping', default=True)

    # mapping_argument = DeclareLaunchArgument(
    #     "mapping",
    #     default_value="true",
    #     description="Create new SLAM map or use existing map")

    # slam_toolbox_mapping_or_localization = LaunchDescription([
    #     Node(
    #         condition=IfCondition(mapping),
    #         package='slam_toolbox',
    #         executable='sync_slam_toolbox_node',
    #         name='slam_toolbox_mapping',
    #         output='screen',
    #         parameters=[{'use_sim_time': is_simulation},
    #                     config_jackal_localization]),
    #         params_file,
    #         {
    #             "map_file_name": map_file_name,
    #             # "map_start_pose": [0.0, 0.0, 0.0],  # added this to avoid error print, but not sure if it breaks anything
    #             "map_start_at_dock": True,
    #             "use_lifecycle_manager": use_lifecycle_manager,
    #             "use_sim_time": sim,
    #             "mode": "localization",
    #         },

                        
    #     Node(
    #     condition=UnlessCondition(mapping),
    #     package='slam_toolbox',
    #     executable='localization_slam_toolbox_node',
    #     name='slam_toolbox_localization',
    #     output='screen',
    #     parameters=[{'use_sim_time': is_simulation},
    #                 config_jackal_localization])
    # ])


    ld = LaunchDescription()
    ld.add_action(declare_namespace_launch_arg)
    ld.add_action(declare_base_frame_name)
    ld.add_action(declare_odom_frame_name)
    ld.add_action(is_simulation_launch_arg)
    ld.add_action(localization_group_action)
    ld.add_action(static_transform)
    
    # ld.add_action(mapping_argument)
    # ld.add_action(slam_toolbox_mapping_or_localization)

    return ld
