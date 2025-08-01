import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_yaml = os.path.join(
        get_package_share_directory("dyno_jackal_bringup"),
        "params",
        "nav2_params.yaml",
    )

    map_file = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "maps",
        "warehouse.yaml",
    )

    return LaunchDescription([

        Node(  # Hosting the map
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=
            [{
                "use_sim_time": True,
                "yaml_filename": map_file
            }],
        ),
        Node(  # controller
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_yaml], 
            remappings=[
                # TODO: Namespacing that is not hard-coded?
                ("cmd_vel", "jackal/nav_vel"),
            ]
        ),
        Node(  # Planner
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_yaml],
        ),
        Node(  # Behavior
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_yaml],
        ),
        Node(  # Navigator
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_yaml],
        ),

        Node(  # Lifecycles, handle the nodes inactive/active states
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"autostart": True},
                {
                    "node_names":
                    [
                        "controller_server",  
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "map_server",
                    ]
                },
            ],
        ),

    ])
