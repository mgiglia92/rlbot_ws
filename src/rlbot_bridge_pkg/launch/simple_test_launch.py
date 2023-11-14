#! /usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from rlbot_msgs.srv import TwistSetpoint, ResetGameState, SetGains
from rlbot_msgs.msg import PIDGains, RigidBodyTick, State
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from rosidl_runtime_py import message_to_yaml

def generate_launch_description():
    gains = PIDGains()
    gains.kp = 10.0
    setgains = SetGains.Request()
    setgains.gains = gains
    gains_yaml = message_to_yaml(setgains)
    ld = LaunchDescription([
        Node(package='simple_controller_pkg',
             executable='SimpleControllerNode',
             name='controller_sim',
            #  arguments=['-v', '1000', '-w', '2']
             ),

    ])
    ld.add_action(
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call",
                " /simple_controller_pkg",
                " rlbot_msgs/srv/SetGains ",
                str(gains_yaml)
            ]],
            shell=True
        ),        
    )
    return ld

generate_launch_description()