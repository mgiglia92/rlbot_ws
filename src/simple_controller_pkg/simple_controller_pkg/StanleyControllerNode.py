import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import TrajectoryReference
from rlbot_msgs.srv import SetGains, TwistSetpoint
from transforms3d.quaternions import rotate_vector
from transforms3d.euler import quat2euler
from transforms3d.utils import normalized_vector
from transforms3d.axangles import axangle2mat
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
    return -1*np.arctan2(np.dot(np.cross(v1_u, v2_u), [0,0,1]), np.dot(v1_u, v2_u))

class StanleyControllerNode(Node):
    gains = PIDStruct()
    def __init__(self, node_name="stanley_controller", **kwargs):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(ControllerReference, "/controller_reference", 10)
        self.publisher2_ = self.create_publisher(Twist, "/internals_stanley", 10)
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
        xp = msg.rbt.bot_state.pose.position.x
        yp = msg.rbt.bot_state.pose.position.y
        o = msg.rbt.bot_state.pose.orientation
        vmag = msg.rbt.bot_state.vmag
        quat = np.array([o.w, o.x, o.y, o.z])
        forward = rotate_vector(np.array([1,0,0]), quat, False)
        right = rotate_vector(np.array([0,1,0]), quat, False)
        up = rotate_vector(np.array([0,0,1]), quat, False)

        # Sanitize Heading input
        # q = msg.rbt.bot_state.pose.orientation
        # quat = np.array([q.w,q.x,q.y,q.z])
        # x = np.array([1,0,0])
        # hvec = rotate_vector(x,quat)
        # hvec[2]=0.0
        # yaw = angle_between(hvec,x)

        # roll,pitch,yaw = quat2euler([o.w, o.x, o.y, o.z], 'sxyz')
        v = msg.rbt.bot_state.twist.linear
        vel = np.array([v.x, v.y, v.z])
        # Heading error
        vx = msg.vxr
        vy = msg.vyr
        yaw_desired = msg.thetar

        # Cross Track Error
        vec_to_path_world = np.array([msg.xr, msg.yr, 0]) - np.array([xp, yp, 0])
        R = axangle2mat([0,0,1], np.pi/2)
        ctvec_body = np.dot(R,vec_to_path_world)
        cte = np.linalg.norm(vec_to_path_world)
        trajectory_velocity = np.array([msg.vxr, msg.vyr, 0])

        if(angle_between(unit_vector(vel), unit_vector(vec_to_path_world)) < 0):
            cte = -1*cte

        # he = angle_between([vx, vy, 0], [np.cos(yaw), np.sin(yaw), 0])
        # he = yaw_desired-yaw
        he = angle_between(unit_vector(vel), unit_vector(trajectory_velocity))
        # if(he > np.pi):
        #     he = he - np.pi

        twist = Twist()
        twist.angular.x = he
        twist.angular.y = cte
        twist.angular.z = 0.0

        unit_vel = unit_vector(vel)
        cr.headingx = unit_vel[0]
        cr.headingy = unit_vel[1]
        cr.rbt = msg.rbt
        cr.he = he
        cr.cte = cte
        cr.correction = he# np.arctan2(cte, (0.001+vmag))
        cr.v_desired = float(300)
        cr.w_desired = float(np.clip(cte, -1,1))
        # cr.w_desired = float(np.clip(he, -5.5, 5.5))
        self.publisher_.publish(cr)
        self.publisher2_.publish(twist)
        self.get_logger().info(f"Published: cte:{cte} | he: {he} | angle:{ctvec_body}")

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