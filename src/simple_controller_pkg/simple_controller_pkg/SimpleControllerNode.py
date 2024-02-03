import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.srv import SetGains, TwistSetpoint
import numpy as np
from pyquaternion import Quaternion
from simple_controller_pkg.controller_util import get_best_steering_and_throttle, PIDStruct
import argparse

# parser = argparse.ArgumentParser(
#                     prog='SimplerController',
#                     description='Controls Rlbot Agent',
#                     epilog='No epilog')

# parser.add_argument('-v', '--velocity', type=float, default=1000.0)
# parser.add_argument('-w', '--angvel', type=float, default=2.0)

# args, unknown = parser.parse_known_args()

class SimpleController(Node):
    gains = PIDStruct()
    def __init__(self, node_name="simple_controller", **kwargs):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscription_ = self.create_subscription(ControllerReference, "/controller_reference", self.listener_callback, 10)
        self.services_ = [self.create_service(SetGains, "/simple_controller/set_gains", self.service_callback),
                          self.create_service(TwistSetpoint, "/simple_controller/twist_setpoint", self.setpoint_callback)]
        self.i = 0
        self.subscription_

        self.prev_time = 0.0
        self.integrand = 0.0
        self.prev_err = 0.0

    def service_callback(self, request, response):
        self.gains.set_from_ros_msg(request.gains)
        self.get_logger().info(f"Set gains: {request.gains}")
        response.success = True
        return response

    def setpoint_callback(self, request, response: bool):
        self.des = request.setpoint.linear.x
        self.des_w = request.setpoint.angular.z
        self.get_logger().info(f"Set Twist sepoint: {request.setpoint}")
        response.success = True
        return response

    def listener_callback(self, msg):
        # Do nasty controls math all clobbered up 
        tnow = self.get_clock().now().nanoseconds
        dt = tnow - self.prev_time
        vec = to_numpy(msg.rbt.bot_state.twist.linear)
        q = to_numpy(msg.rbt.bot_state.pose.orientation)
        quat = Quaternion(q).unit
        body_vel = quat.inverse.rotate(vec)
        sign = np.sign(np.dot(body_vel, np.array([1,0,0])))
        body_vel = body_vel
        vmag = msg.rbt.bot_state.vmag
        kp = self.gains.kp
        ki = self.gains.ki
        kd = self.gains.kd
        err = msg.v_desired - vmag
        self.integrand = np.clip(self.integrand + (err*dt/1e9), -1400, 1400)

        des_a = kp*(err) + kd*(err - self.prev_err / (dt/1e9)) + ki*self.integrand

        u_t, u_s = get_best_steering_and_throttle(vmag, des_a, msg.w_desired)
        twist = Twist()
        twist.linear.x = u_t
        twist.angular.z = u_s
        self.prev_time = tnow
        self.prev_vmag = body_vel
        self.prev_err = err
        self.publisher_.publish(twist)

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