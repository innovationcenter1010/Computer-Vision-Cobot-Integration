"""Full bin-picking stack bringup — replaces the legacy main.py.

    ros2 launch cobot_bringup robot.launch.py
    ros2 launch cobot_bringup robot.launch.py cobot:=false   # e.g. no robot attached
    ros2 launch cobot_bringup robot.launch.py runtime_dir:=/ws/runtime_config

runtime_dir is the runtime-WRITABLE per-cell state (bind-mounted, never baked
into the image): trajectories.json (remembered positions, cobot_operation),
tooling_state.yaml + controller_profile/ (cobot_control).

Each node group can be toggled (camera / vision / cobot / tasks / web / health). All nodes read
config/cobot_params.yaml; respawn gives per-node crash isolation inside the
single container.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("cobot_bringup"))
    params_file = str(bringup_share / "config" / "cobot_params.yaml")
    # Packaged v2 SEED for the remembered positions; the live file is
    # <runtime_dir>/trajectories.json (copied from the seed on first boot).
    trajectories_default_file = str(bringup_share / "config" / "trajectories.json")
    # Named IO, motion profiles and tool profiles shared by cobot_control and
    # cobot_operation (bind-mounted with the rest of config/).
    setup_file = str(bringup_share / "config" / "cobot_setup.yaml")
    # Written by the calibration routine; lives in the bind-mounted config dir
    # so it survives container rebuilds. Missing file = not calibrated.
    calibration_file = str(bringup_share / "config" / "calibration.yaml")

    runtime_dir = LaunchConfiguration("runtime_dir")
    trajectories_file = PathJoinSubstitution([runtime_dir, "trajectories.json"])

    args = [
        DeclareLaunchArgument("runtime_dir", default_value="/ws/runtime_config"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("cobot", default_value="true"),
        DeclareLaunchArgument("tasks", default_value="true"),
        DeclareLaunchArgument("web", default_value="true"),
        DeclareLaunchArgument("health", default_value="true"),
    ]

    nodes = [
        Node(
            package="cobot_camera",
            executable="camera_node",
            name="camera_node",
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration("camera")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_detector",
            executable="detector_node",
            name="detector_node",
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration("vision")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_vision",
            executable="localization_node",
            name="localization_node",
            parameters=[
                params_file,
                {"calibration_file": calibration_file, "setup_file": setup_file},
            ],
            condition=IfCondition(LaunchConfiguration("vision")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_control",
            executable="cobot_control_node",
            name="cobot_control",
            parameters=[params_file, {"setup_file": setup_file, "runtime_dir": runtime_dir}],
            condition=IfCondition(LaunchConfiguration("cobot")),
            respawn=True,
            respawn_delay=5.0,
            output="screen",
        ),
        Node(
            package="cobot_operation",
            executable="cobot_operation_node",
            name="cobot_operation",
            parameters=[
                params_file,
                {
                    "setup_file": setup_file,
                    "trajectories_file": trajectories_file,
                    "trajectories_default_file": trajectories_default_file,
                },
            ],
            condition=IfCondition(LaunchConfiguration("cobot")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_tasks",
            executable="task_manager_node",
            name="task_manager",
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration("tasks")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_web",
            executable="web_node",
            name="web_node",
            parameters=[
                params_file,
                {
                    # /api/config and /dev per-param edits persist to this file.
                    "params_file": params_file,
                    # /dev cell-setup edits (applied on node restart).
                    "setup_file": setup_file,
                    # /dev trajectories view (writes go through cobot_operation).
                    "trajectories_file": trajectories_file,
                },
            ],
            condition=IfCondition(LaunchConfiguration("web")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_health",
            executable="health_monitor",
            name="health_monitor",
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration("health")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
    ]

    return LaunchDescription(args + nodes)
