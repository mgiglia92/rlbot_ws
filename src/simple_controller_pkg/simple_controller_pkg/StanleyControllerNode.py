import rclpy
from rclpy.node import Node
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import TrajectoryReference
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from simple_controller_pkg.util.math_util import ControllerReference_from_TrajectoryReference
from simple_controller_pkg.controller_util import PIDStruct


# parser = argparse.ArgumentParser(
#                     prog='SimplerController',
#                     description='Controls Rlbot Agent',
#                     epilog='No epilog')

# parser.add_argument('-v', '--velocity', type=float, default=1000.0)
# parser.add_argument('-w', '--angvel', type=float, default=2.0)

# args, unknown = parser.parse_known_args()


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
        
        
        # cr.w_desired = float(np.clip(he, -5.5, 5.5))
        (cr, twist) = ControllerReference_from_TrajectoryReference(msg)
        self.publisher_.publish(cr)
        self.publisher2_.publish(twist)
        self.get_logger().info(f"Published: {cr}")

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