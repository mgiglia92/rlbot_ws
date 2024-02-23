import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import TrajectoryReference
from rlbot_msgs.srv import SetGains, TwistSetpoint
import numpy as np
from pyquaternion import Quaternion
from simple_controller_pkg.controller_util import get_best_steering_and_throttle, PIDStruct
import argparse
import casadi



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

class MinDist:
    def __init__(self):    
        #Symbolic expression
        self.radius = 1000
        self.xc = casadi.SX.sym('xc')
        self.x1 = casadi.SX.sym('x1')
        self.yc = casadi.SX.sym('yc')
        self.y1 = casadi.SX.sym('y1')
        self.F = casadi.Function('F', [self.xc], [casadi.sqrt(self.radius**2-self.xc**2)])
        self.J = self.F.jacobian()
        self.dist = casadi.Function('dist', [self.xc, self.x1, self.yc, self.y1], [casadi.sqrt((self.xc-self.x1)**2 + (self.yc-self.y1)**2)])

        self.opti = casadi.Opti()
        self.xpos = self.opti.parameter()
        self.ypos = self.opti.parameter()
        self.xmin = self.opti.variable(1)
        self.ymin = self.F(self.xmin)
        self.opti.set_initial(self.xmin, 0.99)
        self.opti.subject_to(self.xmin>=-1*(self.radius-1))
        self.opti.subject_to(self.xmin<=(self.radius-1))
        self.opti.minimize(self.dist(self.xmin, self.xpos, self.ymin, self.ypos))
        opts = {'ipopt.print_level': 2, 'print_time': 0, 'ipopt.sb': 'yes'}
        self.opti.solver('ipopt', opts)
        self.sol = None
    
    def update_position(self, x, y):
        self.opti.set_value(self.xpos, x)
        self.opti.set_value(self.ypos, y)
    
    def solve(self):
        self.sol = self.opti.solve()
        return self.opti.value(self.xmin), self.opti.value(self.ymin)
        

class ReferenceGeneratorNode(Node):
    gains = PIDStruct()
    def __init__(self, node_name="reference_generator", **kwargs):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(TrajectoryReference, "/trajectory_reference", 10)
        self.subscription_ = self.create_subscription(RigidBodyTickMsg, "/player0/RigidBodyTick", self.reference_callback, 10)
        # self.services_ = [self.create_service(SetGains, "/simple_controller/set_gains", self.service_callback),
        #                   self.create_service(TwistSetpoint, "/simple_controller/twist_setpoint", self.setpoint_callback)]
        self.i = 0
        # self.subscription_
        self.min_dist = MinDist()


    
    # def service_callback(self, request, response):
    #     self.gains.set_from_ros_msg(request.gains)
    #     self.get_logger().info(f"Set gains: {request.gains}")
    #     response.success = True
    #     return response

    # def setpoint_callback(self, request, response: bool):
    #     self.des = request.setpoint.linear.x
    #     self.des_w = request.setpoint.angular.z
    #     self.get_logger().info(f"Set Twist setpoint: {request.setpoint}")
    #     response.success = True
    #     return response
    

    
    def reference_callback(self, msg: RigidBodyTickMsg):
        # Get time from the message, and get reference state from hard coded trajectory
        # time = msg.time
        # f = (1/20)
        # xr = 500*np.cos(2*np.pi*f*time)
        # yr = 500*np.sin(2*np.pi*f*time)
        # vxr = -2*500*np.pi*f*np.sin(2*np.pi*f*time)
        # vyr = 2*500*np.pi*f*np.cos(2*np.pi*f*time)
        
        self.min_dist.update_position(msg.bot_state.pose.position.x, msg.bot_state.pose.position.y)
        xr, yr = self.min_dist.solve()
        #TODO: Parametrize the path so that we get a vector indicating direction to replace vxr and vyr
        slope = self.min_dist.J(xr, self.min_dist.xc)
        vxr = float(1)
        vyr = float(slope)
        thetar = angle_between([vxr, vyr, 0], [1, 0, 0])

        trajr = TrajectoryReference()
        trajr.rbt = msg
        trajr.xr = xr
        trajr.yr = yr
        trajr.vxr = vxr
        trajr.vyr = vyr
        trajr.thetar = thetar

        self.publisher_.publish(trajr)
        # self.get_logger().info(f"Published: {trajr}")

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

    controller = ReferenceGeneratorNode()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()