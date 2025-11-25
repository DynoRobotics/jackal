from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # lc = LaunchContext()
    # joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='logitech')
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="True", 
        description="Whether or not simulation time should be used"
    )

    # filepath_config_joy = PathJoinSubstitution(
    #     [FindPackageShare('jackal_control'), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    # )
    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),'config','joy_bt.yaml']
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # pkg_jackal_control = get_package_share_directory("jackal_control")
    # joy2twist_params = os.path.join(pkg_jackal_control, "config", "joy_bt.yaml")

    start_joy2twist_node = Node(
        package="joy2twist",
        executable="joy2twist",
        name="joy2twist_node",
        parameters=[
            filepath_config_joy,
            {"cmd_vel_stamped": True, "use_sim_time": use_sim_time} # Jazzy uses stamped twist messages
        ],
        output={"screen"},
        remappings={("cmd_vel", "joy_vel")},
        emulate_tty="true",
    )

    start_joy_linux_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        output={"screen"},
        emulate_tty="true",
    )


    # node_joy = Node(
    #     namespace='joy_teleop',
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     name='joy_node',
    #     parameters=[filepath_config_joy]
    # )

    # node_teleop_twist_joy = Node(
    #     namespace='joy_teleop',
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     output='screen',
    #     name='teleop_twist_joy_node',
    #     parameters=[filepath_config_joy]
    # )


    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(start_joy2twist_node)
    ld.add_action(start_joy_linux_node)
    # ld.add_action(node_joy)
    # ld.add_action(node_teleop_twist_joy)
    return ld
