import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('pid'),
        'config',
        'param.yaml'
        )

    node=Node(
        package = 'pid',
        name = 'pid_node',
        executable = 'pid_node.py',
        parameters = [config]
    )
    ld.add_action(node)
    return ld
