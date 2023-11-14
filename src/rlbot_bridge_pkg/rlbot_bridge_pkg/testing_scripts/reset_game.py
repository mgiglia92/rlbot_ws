import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rlbot_msgs.srv import ResetGameState

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('reset_game_node')
        self.cli = self.create_client(ResetGameState, 'reset_game_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ResetGameState.Request()

        # Hard coded reset state
        self.req.rigid_body_tick.ball_state.pose.position.x = 1000.0
        self.req.rigid_body_tick.ball_state.pose.position.z = 1000.0
        self.req.rigid_body_tick.bot_state.pose.position.x = -2500.
        self.req.rigid_body_tick.bot_state.pose.position.y = 0.

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
       f"success: {response.success}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
