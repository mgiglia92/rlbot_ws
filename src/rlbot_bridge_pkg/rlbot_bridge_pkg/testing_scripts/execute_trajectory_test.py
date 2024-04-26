# import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rlbot_msgs.msg import State, ActiveTraitsMsg
from rlbot_msgs.srv import SetGains
from rlbot_msgs.action import ExecuteTrajectory
from trajectory_generator_pkg.sample_trajectory_generator import ActiveTraits
from std_msgs.msg import Int32MultiArray, Int32, Float32
# import argparse
import numpy as np

class MinimalActionClientAsync(Node):

    def __init__(self):
        super().__init__('execute_trajetory_test')
        self._action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory_action')

    def send_goal(self, goal: ActiveTraitsMsg):
        goal_msg = ExecuteTrajectory.Goal()
        traits = ActiveTraitsMsg()
        # traits.active.append(1)
        # traits.values.append(float(1))

        #TODO: Turn into utility
        for each in [1,1,0,0,1,0,1]:
            a=Int32()
            a.data = each
            traits.active.append(a)

        for each in [2000., 2000., 0., 0., 0., 0., 0.]: 
            a=Float32()
            a.data = each
            traits.values.append(a)

        goal_msg.desired = traits

        while not self._action_client.wait_for_server(5):
            print("Action Server not active")

        return self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, msg: State):
        self.get_logger().info(f"Action Feedback: {msg}")


def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClientAsync()
    future = action_client.send_goal(ActiveTraits)
    rclpy.spin_until_future_complete(action_client, future)
    action_client.get_logger().info(
       f"success: {future}")

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
