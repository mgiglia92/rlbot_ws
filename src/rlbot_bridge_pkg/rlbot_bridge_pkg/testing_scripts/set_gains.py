import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rlbot_msgs.srv import SetGains
import argparse


parser = argparse.ArgumentParser(
                    prog='Set Gains to Simple Controller',
                    description='Controls agent controller gains',
                    epilog='No epilog')

parser.add_argument('-g', '--gains', type=float, nargs=4, default=[1.0, 0.0, 0.0, 0.0])
args = parser.parse_args()


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('reset_game_node')
        self.cli = self.create_client(SetGains, 'simple_controller/set_gains')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetGains.Request()
        self.req.gains.kp = args.gains[0]
        self.req.gains.ki = args.gains[1]
        self.req.gains.kd = args.gains[2]
        self.req.gains.dt = args.gains[3]

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
