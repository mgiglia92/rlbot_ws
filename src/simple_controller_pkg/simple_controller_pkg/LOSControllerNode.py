import rclpy
import numpy as np
from rclpy.node import Node
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import TrajectoryReference
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from simple_controller_pkg.util.math_util import ControllerReference_from_TrajectoryReference
from simple_controller_pkg.controller_util import PIDStruct
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg


class LOSControllerNode(Node):
    def __init__(self, node_name="los_controller", **kwargs):
        super().__init__(node_name)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(RigidBodyTickMsg, f"/player0/RigidBodyTick", self.control, 10)

    def control(self, msg: RigidBodyTickMsg):        
        p_missile = msg.bot_state.pose.position
        r_m = np.array([p_missile.x, p_missile.y, 0])
        v_missile = msg.bot_state.twist.linear
        v_m = np.array([v_missile.x, v_missile.y, 0])
        
        p_targ = msg.ball_state.pose.position
        r_t = np.array([p_targ.x, p_targ.y, 0])
        v_targ = msg.ball_state.twist.linear
        v_t = np.array([v_targ.x, v_targ.y, 0])
        
        los_vector = np.subtract(r_t, r_m)
        v_r = np.subtract(v_t, v_m)

        # Compute the LOS angle
        omega = np.cross(los_vector, v_r) / np.dot(los_vector, los_vector)
        
        N = 3
        # Compute proportional accelleration
        a_r = np.cross(-N * np.abs(v_r) * (np.linalg.norm(los_vector)), omega)   
 
        out = Twist()
        out.angular.z = -1*a_r[0]
        out.linear.x = 1.0
        out.linear.y = 1.0
        self.pub.publish(out)
    


def main(args=None):
    rclpy.init(args=args)

    controller = LOSControllerNode()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()