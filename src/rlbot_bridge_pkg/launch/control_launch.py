from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='simple_controller_pkg',
             executable='SimpleControllerNode',
             name='controller_sim'),
        Node(package='trajectory_generator_pkg',
             executable='TrajectoryGeneratorNode',
             name='controller_sim'),
    ])