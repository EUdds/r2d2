from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    host_id = LaunchConfiguration("host_id")
    dest_id = LaunchConfiguration("dest_id")
    publish_per_node = LaunchConfiguration("publish_per_node")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    subscribe_per_node = LaunchConfiguration("subscribe_per_node")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("host_id", default_value="0"),
            DeclareLaunchArgument("dest_id", default_value="255"),
            DeclareLaunchArgument("publish_per_node", default_value="true"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8090"),
            DeclareLaunchArgument("subscribe_per_node", default_value="true"),
            Node(
                package="gateway",
                executable="r2bus_gateway",
                name="r2bus_gateway",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "baud": ParameterValue(baud, value_type=int),
                        "host_id": ParameterValue(host_id, value_type=int),
                        "dest_id": ParameterValue(dest_id, value_type=int),
                        "publish_per_node": ParameterValue(publish_per_node, value_type=bool),
                    }
                ],
            ),
            Node(
                package="gateway",
                executable="web_bridge",
                name="r2bus_web_bridge",
                output="screen",
                parameters=[
                    {
                        "web_host": web_host,
                        "web_port": ParameterValue(web_port, value_type=int),
                        "subscribe_per_node": ParameterValue(subscribe_per_node, value_type=bool),
                        "cors_origins": ParameterValue(["*"], value_type=List[str]),
                    }
                ],
            ),
        ]
    )
