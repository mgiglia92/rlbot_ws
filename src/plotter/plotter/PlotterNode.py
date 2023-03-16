import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from rlbot_msgs.msg import RigidBodyTick, State
from geometry_msgs.msg import Vector3, Twist
from geometry_msgs.msg import Quaternion as QMsg
from pyquaternion import Quaternion

class PhaseSpacePlotterNode(Node):
    def __init__(self):
        super().__init__('phase_space_plotter')
        self.subscription_ = self.create_subscription(RigidBodyTick, '/player0/RigidBodyTick', self.listener_callback, 10)
        self.subscription2_ = self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')
        plt.title("Phase Space of System in Driving State")
        plt.xlabel("steer")
        plt.ylabel("Angular_velocity")
        self.length = 100
        self.vmag_history = []
        self.w_history = []
        self.throttle_history = []
        self.steer_history = []
        self.throttle=0.0
        self.steer=0.0
    
    def timer_callback(self):
        plt.cla()
        self.ax.plot(self.steer_history[-100:-1], self.w_history[-100:-1], self.vmag_history[-100:-1],'r.')
        plt.draw()
        plt.pause(0.001)
    
    def cmdvel_callback(self, msg:Twist):
        self.throttle = msg.linear.x
        self.steer = msg.angular.z

    def listener_callback(self, msg:RigidBodyTick):
        # Get state and inputs
        vmag = msg.bot_state.vmag
        w = msg.bot_state.twist.angular.z
        throttle = float(msg.bot_state.throttle)
        steer = float(msg.bot_state.steer)
        self.vmag_history.append(vmag)
        self.w_history.append(w)
        self.throttle_history.append(self.throttle)
        self.steer_history.append(self.steer)
        # self.get_logger().info(f"vmag: {vmag:0.2f} w:{w:0.2f} thr:{throttle:0.4f} steer:{steer:0.4f}")

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

    plotter = PhaseSpacePlotterNode()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
