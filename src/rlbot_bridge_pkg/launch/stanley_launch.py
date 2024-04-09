from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from rlbot_msgs.msg import PIDGains

def generate_launch_description():
    gains = PIDGains()
    gains.kp=1.0
    gains.ki=0.0
    gains.kd=0.0
    gains.dt=0.0
    ld = LaunchDescription([
        Node(package='simple_controller_pkg', 
             executable='StanleyControllerNode'),
        Node(package='simple_controller_pkg',
             executable='ReferenceGeneratorNode'),
        Node(package='simple_controller_pkg',
             executable='SimpleControllerNode')
    ])
    return ld