import os.path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from dyno_utils.launch_utils import DynoWaitFor
import rclpy.qos

import std_msgs.msg
import sensor_msgs.msg
import rosgraph_msgs.msg

def generate_launch_description():

    pkg_dyno_jackal_bringup = get_package_share_directory("dyno_jackal_bringup")
    pkg_jackal_control = get_package_share_directory("jackal_control")

    twist_mux_params = os.path.join(get_package_share_directory("dyno_jackal_bringup"), "params", "twist_mux.yaml")

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    jackal_manual_control = LaunchConfiguration("jackal_manual_control", default=False)
    location = LaunchConfiguration("location", default="dyno_office_indoors")

    jackal_manual_control_launch_argument = DeclareLaunchArgument(
        "jackal_manual_control",
        default_value="False",
        description="To launch jackal keyboard steering or not")

    jackal_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "gazebo.launch.py")))
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jackal_control, "launch", "control.launch.py")))

    keyboard_steering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "keyboard_steering.launch.py")),
            condition=IfCondition(jackal_manual_control))
    
    twist_mux = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('cmd_vel_out', 'velocity_controller/cmd_vel_unstamped')},
            parameters=[twist_mux_params])

    

    jackal_with_namespace_and_remapping = GroupAction(
     actions=[
        PushRosNamespace('jackal'),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),
        jackal_gazebo,
        control,
        twist_mux,
        keyboard_steering
      ]
   )
    
    # TODO: Jackal namespace is for some reason given to other robots spawned
    # *after* the Jackal. Until this is sorted out, we will launch the Jackal last
    ld = LaunchDescription()
    ld.add_action(
        DynoWaitFor(
            name="jackal_launched_last",
            message_on_topics=[
                ("/clock", rosgraph_msgs.msg.Clock, rclpy.qos.qos_profile_sensor_data), # Wait for Gazebo to launch
                ("/static_agents/robot_description", std_msgs.msg.String, rclpy.qos.qos_profile_system_default), # Wait for static agents to launch
                ("/scan", sensor_msgs.msg.LaserScan, rclpy.qos.qos_profile_sensor_data), # Wait for infobot to launch
            ],
            actions=[
                jackal_with_namespace_and_remapping,
                ],
        )
    )

    return ld

def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
