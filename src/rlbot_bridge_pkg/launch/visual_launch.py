#! /usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from rlbot_msgs.srv import TwistSetpoint, ResetGameState, SetGains
from rlbot_msgs.msg import PIDGains, RigidBodyTick, State
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
from rosidl_runtime_py import message_to_yaml

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        ExecuteProcess(
            cmd=[[FindExecutable(name='ros2'),
                    " run plotjuggler plotjuggler --layout $RLBOT_WS_DIR/plotjuggler/stanley.xml"
            ]],
            shell=True
            )
    )
    return ld

generate_launch_description()