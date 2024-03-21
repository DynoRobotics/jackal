import os.path
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():

    pkg_dyno_jackal_bringup = get_package_share_directory("dyno_jackal_bringup")
    pkg_jackal_control = get_package_share_directory("jackal_control")

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    location = LaunchConfiguration("location", default="dyno_office_indoors")

    twist_mux_params = os.path.join(get_package_share_directory("dyno_jackal_bringup"), "params", "twist_mux.yaml")

    jackal_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "gazebo.launch.py")))
    
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jackal_control, "launch", "control.launch.py")))
    
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
        twist_mux
      ]
   )
    
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/jackal -r cmd_vel:=key_vel

    ld = LaunchDescription()
    ld.add_action(jackal_with_namespace_and_remapping)

    return ld

def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
