from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch = LaunchDescription()

    res_node = Node(
        package = "res",
        executable = "res",
        respawn=True,
        respawn_delay=1
    )

    launch.add_action(res_node)

    return launch