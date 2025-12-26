from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    host_id = LaunchConfiguration("host_id")
    dest_id = LaunchConfiguration("dest_id")
    publish_per_node = LaunchConfiguration("publish_per_node")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    subscribe_per_node = LaunchConfiguration("subscribe_per_node")

    gateway_launch = Path(get_package_share_directory("gateway")) / "launch" / "r2bus_bridge.launch.py"
    dome_launch = Path(__file__).parent / "dome_launch.py"

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gateway_launch)),
                launch_arguments={
                    "port": port,
                    "baud": baud,
                    "host_id": host_id,
                    "dest_id": dest_id,
                    "publish_per_node": publish_per_node,
                    "web_host": web_host,
                    "web_port": web_port,
                    "subscribe_per_node": subscribe_per_node,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(dome_launch)),
            ),
        ]
    )
