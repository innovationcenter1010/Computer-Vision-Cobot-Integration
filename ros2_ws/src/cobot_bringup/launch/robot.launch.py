"""Full bin-picking stack bringup — replaces the legacy main.py.

    ros2 launch cobot_bringup robot.launch.py
    ros2 launch cobot_bringup robot.launch.py cobot:=false   # e.g. no robot attached

Each node group can be toggled (camera / vision / cobot / web). All nodes read
config/cobot_params.yaml; respawn gives per-node crash isolation inside the
single container.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("cobot_bringup"))
    params_file = str(bringup_share / "config" / "cobot_params.yaml")
    trajectories_file = str(bringup_share / "config" / "trajectories.json")
    # Written by the calibration routine; lives in the bind-mounted config dir
    # so it survives container rebuilds. Missing file = not calibrated.
    calibration_file = str(bringup_share / "config" / "calibration.yaml")

    args = [
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("cobot", default_value="true"),
        DeclareLaunchArgument("tasks", default_value="true"),
        DeclareLaunchArgument("web", default_value="true"),
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
            package="cobot_vision",
            executable="yolo_node",
            name="yolo_node",
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
            parameters=[params_file, {"calibration_file": calibration_file}],
            condition=IfCondition(LaunchConfiguration("vision")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
        Node(
            package="cobot_control",
            executable="cobot_node",
            name="cobot_node",
            parameters=[params_file, {"trajectories_file": trajectories_file}],
            condition=IfCondition(LaunchConfiguration("cobot")),
            respawn=True,
            respawn_delay=5.0,
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
            parameters=[params_file, {"params_file": params_file}],
            condition=IfCondition(LaunchConfiguration("web")),
            respawn=True,
            respawn_delay=2.0,
            output="screen",
        ),
    ]

    return LaunchDescription(args + nodes)
