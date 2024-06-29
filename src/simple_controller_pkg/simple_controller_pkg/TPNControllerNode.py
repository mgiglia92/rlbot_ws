import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from simple_controller_pkg.util.math_util import angle_between
import numpy as np
import ros2_numpy as rnp

class LOS_State:
    def __init__(self, time: float, los: Vector3, los_rate: float):
        self.los = los
        self.los_rate = los_rate
        self.time = time

class TPNController(Node):
    def __init__(self, node_name="TPNController", **kwargs):
        super().__init__(node_name)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.internals_publisher = self.create_publisher(Twist, "/tpn_internals", 10)
        self.subscription_ = self.create_subscription(RigidBodyTickMsg, "/player0/RigidBodyTick", self.calculate_controls, 10)
        self.previous = LOS_State(0.0, Vector3(), 0.0)
        self.current = LOS_State(0.0, Vector3(), 0.0)
    @staticmethod
    def get_angle(vec1, vec2):
        inner = np.dot(vec1, vec2)
        cross = np.cross(vec1,vec2)
        norm = np.linalg.norm(cross)
        if(norm < 1e-3): return 0.0
        cross_norm = cross/norm
        angle = np.arctan2(cross_norm[2], inner)
        return angle

    # Callback for rigid body tick topic. At each frame calculate controls and send it to bot.
    def calculate_controls(self, msg: RigidBodyTickMsg):
    # Update prev LOS calc

    # Calculate LOS Vector (Global Frame) update current LOS state
        ball = msg.ball_state.pose.position
        car =  msg.bot_state.pose.position
        self.current.los = Vector3()
        self.current.los.x = ball.x-car.x
        self.current.los.y = ball.y-car.y
        self.current.los.z = ball.z-car.z
        self.current.time = msg.time
        
    # Calculate LOS Rotation Rate (Global Frame)
        # Get normalized version of LOS vectors
        nlos_prev = rnp.numpify(self.previous.los)/np.linalg.norm(rnp.numpify(self.previous.los))
        nlos_cur = rnp.numpify(self.current.los)/np.linalg.norm(rnp.numpify(self.current.los))
        dt = float(self.current.time - self.previous.time)
        dTheta = angle_between(nlos_cur, nlos_prev)
        los_rate = dTheta/dt
        self.current.los_rate = los_rate
        los_rate_rate = (self.current.los_rate - self.previous.los_rate) / dt
        

    # Determine steering input ()

        # Determine acceleration input

        # Fill out Twist message to publish
        input = Twist()
        input.linear.x = 1.
        input.linear.y = 1.
        input.angular.z = (2*los_rate) + (los_rate_rate)# Steering input based on calculated los_rate

        # Publish message
        self.publisher.publish(input)

        # Publish internals messages
        internals = Twist()
        internals.angular.y = los_rate_rate
        internals.angular.z = los_rate
        internals.linear.x = self.current.los.x
        internals.linear.y = self.current.los.y
        internals.linear.z = self.current.los.z
        self.internals_publisher.publish(internals)

        # Update previous value for next iteration
        self.previous = LOS_State(self.current.time, self.current.los, self.current.los_rate)
        pass

def main(args=None):
    rclpy.init(args=args)

    controller = TPNController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()