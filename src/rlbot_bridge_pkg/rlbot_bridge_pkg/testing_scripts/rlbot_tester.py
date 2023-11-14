# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import argparse

parser = argparse.ArgumentParser(
                    prog='RlbotTester',
                    description='Controls Rlbot Agent',
                    epilog='No epilog')

parser.add_argument('-t', '--throttle', type=float)
parser.add_argument('-s', '--steer', type=float)
args = parser.parse_args()


def main():
    rclpy.init()

    node = rclpy.create_node('rlbot_tester')
    publisher = node.create_publisher(Twist, '/cmd_vel', 2)

    msg = Twist()
    i = 0
    inc = True
    throttle1= args.throttle
    throttle2 = 0.0

    def timer_callback():
        nonlocal i
        nonlocal inc
        # scale = 1000
        # msg.linear.x = i/scale - 1
        # if(i == 2*scale): inc = False
        # elif(i == 0): inc = True
        # if inc: i += 1 
        # else: i -= 1
        if(i<500 and i>0):
            msg.linear.x=throttle1
        elif(i>500 and i<1000):
            msg.linear.x=throttle2
        elif(i>=1000):
            i=0
            msg.linear.x = throttle1
        i+=1
        
        node.get_logger().info('Publishing: "%s"' % msg.linear)
        publisher.publish(msg)

    timer_period = 1/500  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
