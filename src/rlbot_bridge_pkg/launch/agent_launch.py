from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rlbot_bridge_pkg', 
             executable='AgentNode',
             name='rlbot_basic_sim')
    ])