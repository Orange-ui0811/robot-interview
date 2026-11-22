from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("go1_bringup"))
    description = Path(get_package_share_directory("go1_description"))
    sim = Node(
        package="go1_mock", executable="go1_mock_node", output="screen",
        parameters=[LaunchConfiguration("mock_config")],
    )
    bridge = Node(
        package="go1_bringup", executable="go1_mcp_node", output="screen",
        arguments=[
            "--config", str(share / "config/mcp.json"),
            "--port", LaunchConfiguration("mcp_port"),
            "--core-url", LaunchConfiguration("core_url"),
            "--advertise-host", LaunchConfiguration("advertise_host"),
        ],
        condition=IfCondition(LaunchConfiguration("mcp")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mcp", default_value="true"),
        DeclareLaunchArgument("mcp_port", default_value="15795"),
        DeclareLaunchArgument(
            "core_url", default_value=EnvironmentVariable("AGENT_CORE_URL", default_value="disabled"),
        ),
        DeclareLaunchArgument("advertise_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("mock_config", default_value=str(share / "config/mock.yaml")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(description / "launch/state_publisher.launch.py"))),
        RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[
            EmitEvent(event=Shutdown(reason="Go1 simulator exited")),
        ])),
        RegisterEventHandler(OnProcessExit(target_action=bridge, on_exit=[
            EmitEvent(event=Shutdown(reason="Go1 MCP bridge exited")),
        ])),
        sim,
        bridge,
        Node(
            package="rviz2", executable="rviz2", arguments=["-d", str(share / "rviz/go1.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ])
