import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import TrajectoryReference
from rlbot_msgs.srv import SetGains, TwistSetpoint
from transforms3d.euler import quat2euler
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

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arctan2(np.dot(np.cross(v1_u, v2_u), [0,0,1]), np.dot(v1_u, v2_u))

class StanleyControllerNode(Node):
    gains = PIDStruct()
    def __init__(self, node_name="stanley_controller", **kwargs):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(ControllerReference, "/controller_reference", 10)
        self.subscription_ = self.create_subscription(TrajectoryReference, "/trajectory_reference", self.stanley_callback, 10)
        # self.services_ = [self.create_service(SetGains, "/simple_controller/set_gains", self.service_callback),
        #                   self.create_service(TwistSetpoint, "/simple_controller/twist_setpoint", self.setpoint_callback)]
        self.i = 0
        # self.subscription_

        # Control algorithm memory vars for differentail calcs
        # self.prev_time = 0
        # self.prev_vmag = 0
        # self.prev_err = 0
        # self.integrand = 0
        # if args.velocity is not None:
        #     self.des = args.velocity
        # else:
        #     self.des_w = 0
        # if args.angvel is not None:
        #     self.des_w = args.angvel
        # else:
        #     self.des_w = 0 
        # self.des_w = args.angvel
    
    # def service_callback(self, request, response):
    #     self.gains.set_from_ros_msg(request.gains)
    #     self.get_logger().info(f"Set gains: {request.gains}")
    #     response.success = True
    #     return response

    # def setpoint_callback(self, request, response: bool):
    #     self.des = request.setpoint.linear.x
    #     self.des_w = request.setpoint.angular.z
    #     self.get_logger().info(f"Set Twist sepoint: {request.setpoint}")
    #     response.success = True
    #     return response

    def stanley_callback(self, msg: TrajectoryReference):
        
        cr = ControllerReference()
        x = msg.rbt.bot_state.pose.position.x
        y = msg.rbt.bot_state.pose.position.y
        o = msg.rbt.bot_state.pose.orientation
        cte = np.sqrt((msg.xr - x)**2 + (msg.yr - y)**2)
        r = -1 * quat2euler([o.w, o.x, o.y, o.z])[-1]
        he = msg.thetar - r


        cr.rbt = msg.rbt
        cr.v_desired = float(100)
        cr.w_desired = float(np.clip(he, -5.5, 5.5))
        self.publisher_.publish(cr)
        self.get_logger().info(f"Published: cte:{cte} | he: {he} | angle:{r}")

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

    controller = StanleyControllerNode()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()