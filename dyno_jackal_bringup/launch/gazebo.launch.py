import os.path
import os

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(
        get_package_share_directory("jackal_description"),
        "meshes"
    )
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gazebo_models_path

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("jackal_description"),
                    "urdf",
                    "jackal.urdf.xacro",
                ]
            ),
            " ",
            "name:=jackal",
            " ",
            "prefix:=jackal",
            " ",
            "is_sim:=true"
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=
        [{
            "use_sim_time": True,
            "robot_description": ParameterValue(robot_description_content)
        }],
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_jackal",
        arguments=[
            "-name","jackal",
            '-x',"0.0",
            '-y','0.0',
            '-z','0.2',
            "-topic","robot_description",
            "-robot_namespace","jackal",
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)

    return ld

def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
