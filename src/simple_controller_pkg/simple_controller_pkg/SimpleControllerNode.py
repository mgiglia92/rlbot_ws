import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
import numpy as np
from pyquaternion import Quaternion
from simple_controller_pkg.controller_util import get_best_steering_and_throttle
import argparse

parser = argparse.ArgumentParser(
                    prog='SimplerController',
                    description='Controls Rlbot Agent',
                    epilog='No epilog')

parser.add_argument('-v', '--velocity', type=float)
parser.add_argument('-w', '--angvel', type=float)
args = parser.parse_args()

class SimpleController(Node):

    def __init__(self):
        super().__init__('simple_controller')
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription_ = self.create_subscription(RigidBodyTickMsg, "/player0/RigidBodyTick", self.listener_callback, 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription_

        # Control algorithm memory vars for differentail calcs
        self.prev_time = 0
        self.prev_vmag = 0
        self.prev_err = 0
        self.integrand = 0
        if args.velocity is not None:
            self.des = args.velocity
        else:
            self.des = 0
        if args.angvel is not None:
            self.des = args.angvel
        else:
            self.des_w = 0 
        self.des_w = args.angvel

    def listener_callback(self, msg):
        # Do nasty controls math all clobbered up 
        tnow = self.get_clock().now().nanoseconds
        dt = tnow - self.prev_time
        vec = to_numpy(msg.bot_state.twist.linear)
        q = to_numpy(msg.bot_state.pose.orientation)
        quat = Quaternion(q).unit
        body_vel = quat.inverse.rotate(vec)
        sign = np.sign(np.dot(body_vel, np.array([1,0,0])))
        body_vel = body_vel     
        vmag = msg.bot_state.vmag
        kp = 1.0
        ki = 0
        kd = 0.001
        err = 1000-vmag

        self.integrand = np.clip(self.integrand + (err*dt/1e9), -1, 1)

        des_a = kp*(err) + kd*(err - self.prev_err / (dt/1e9)) + ki*self.integrand

        u_t, u_s = get_best_steering_and_throttle(vmag, des_a, self.des_w)
        twist = Twist()
        twist.linear.x = u_t
        twist.angular.z = u_s
        self.publisher_.publish(twist)
        self.get_logger().info(f"des:{self.des}, v:{vmag}, u_t:{u_t}, desa:{des_a} err:{err}")
        self.prev_time = tnow
        self.prev_vmag = body_vel
        self.prev_err = err
    

    def norm(self, vec) -> np.array:
        return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2) * np.sign()

def to_numpy(v):
    try:
        if type(v) == Vector3:
            return np.array([v.x, v.y, v.z])
        if type(v) == QMsg:
            return np.array([v.w, v.x, v.y, v.z])
    except:
        print("Type error")
        return np.array([0,0,0,0])

def main(args=None):
    rclpy.init(args=args)

    controller = SimpleController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()