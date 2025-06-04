from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package = 'team8_pkg'
    node_name = 'target_node'

    ld = LaunchDescription()

    """target_node = Node(
        package=package,
        executable=node_name,
        output='screen')"""
  
    """depth_node = Node(
        package=package,
        executable='depth_node',
        output='screen'
    )"""

    lidar_node = Node(
        package=package,
        executable='lidar_node',
        output='screen'
    )
        
    ld.add_action(lidar_node)
    return ld

