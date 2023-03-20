import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from rlbot_msgs.msg import RigidBodyTick, State, Polynomial3
from geometry_msgs.msg import Vector3, Twist
from geometry_msgs.msg import Quaternion as QMsg
from pyquaternion import Quaternion

class PhaseSpacePlotterNode(Node):
    def __init__(self):
        super().__init__('phase_space_plotter')
        self.subscription_ = self.create_subscription(RigidBodyTick, '/player0/RigidBodyTick', self.listener_callback, 10)
        self.subscription2_ = self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 10)
        self.subscription3_ = self.create_subscription(Polynomial3, '/current_trajectory', self.traj_callback,10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        self.fig0 = plt.figure(0)
        self.fig1 = plt.figure(1)
        self.ax = self.fig0.add_subplot(projection='3d')
        self.ax1 = self.fig1.add_subplot()
        self.length = 100
        self.x_history = []
        self.y_history = []
        self.vmag_history = []
        self.w_history = []
        self.throttle_history = []
        self.steer_history = []
        self.throttle=0.0
        self.steer=0.0
        self.xpoly = np.poly1d([0,1,2,3])
        self.ypoly = np.poly1d([0,1,2,3])
        self.tf = 1

    def timer_callback(self):
        plt.cla()
        self.ax.plot(self.x_history[-100:-1], self.y_history[-100:-1], self.vmag_history[-100:-1],'r.')

        self.ax.axes.set_xlim3d(left=-2000, right=2000) 
        self.ax.axes.set_ylim3d(bottom=-2000, top=2000) 
        self.ax.axes.set_zlim3d(bottom=0, top=1400)

        teval = np.linspace(0, self.tf, 101)
        self.ax1.plot(self.xpoly(teval), self.ypoly(teval))
        self.ax1.axes.set_xlim(0, np.max(self.xpoly(teval)))
        self.ax1.axes.set_ylim(0, np.max(self.ypoly(teval)))
        plt.draw()
        plt.pause(0.001)
    
    def cmdvel_callback(self, msg:Twist):
        self.throttle = msg.linear.x
        self.steer = msg.angular.z
    
    def traj_callback(self, msg: Polynomial3):
        self.xpoly = np.poly1d(msg.px)
        self.ypoly = np.poly1d(msg.py)
        self.tf = msg.tf
        self.get_logger().info(f"Incoming trajectory: {msg}")

    def listener_callback(self, msg:RigidBodyTick):
        # Get state and inputs
        vmag = msg.bot_state.vmag
        w = msg.bot_state.twist.angular.z
        throttle = float(msg.bot_state.throttle)
        steer = float(msg.bot_state.steer)
        x = float(msg.bot_state.pose.position.x)
        y = float(msg.bot_state.pose.position.y)
        self.x_history.append(x)
        self.y_history.append(y)
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
