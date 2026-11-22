from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("go1_description"))
    urdf = (share / "urdf/go1_model.urdf").read_text(encoding="utf-8")
    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="go1_state_publisher", output="screen",
            parameters=[{"robot_description": ParameterValue(urdf, value_type=str),
                         "publish_frequency": 50.0}],
        ),
    ])
