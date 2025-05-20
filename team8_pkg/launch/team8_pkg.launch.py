from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package = 'team8_pkg'
    node_name = 'target_node'

    ld = LaunchDescription()

    calibration_node = Node(
        package=package,
        executable=node_name,
        output='screen')
        
    ld.add_action(calibration_node)
    return ld

