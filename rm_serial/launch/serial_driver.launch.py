# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     rm_serial_up_node = Node(
#         package='rm_serial_ext',
#         executable='rm_serial_ext_up_node',
#         namespace='',
#         output='screen',
#         emulate_tty=True,
#     )
#     rm_serial_down_node = Node(
#         package='rm_serial_ext',
#         executable='rm_serial_ext_down_node',
#         namespace='',
#         output='screen',
#         emulate_tty=True,
#     )

#     return LaunchDescription([rm_serial_up_node, rm_serial_down_node])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    robot_name = LaunchConfiguration("robot_name")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="cuhksz2025_infantry_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_pb2025_robot_description_dir,
                        "launch",
                        "robot_description_launch.py",
                    )
                ),
                launch_arguments={
                    "params_file": params_file,
                    "robot_name": robot_name,
                    "use_rviz": use_rviz,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
            Node(
                package='rm_serial',
                executable='rm_serial_node',
                name="rm_serial_ext",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="rm_serial",
                executable="gimbal_manager_node",
                name="gimbal_manager",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of nodes
    ld.add_action(bringup_cmd_group)

    return ld
