import rclpy
from rclpy.node import Node
from trajectory_generator_pkg.sample_trajectory_generator import TrajectoryOpti, ActiveTraits
from scipy.interpolate import CubicSpline, PPoly
import matplotlib.pyplot as plt
from rlbot_msgs.msg import Polynomial3, RigidBodyTick, DiscretizedTrajectoryReference
from rlbot_msgs.srv import GetOptimalTrajectory
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, Float32
from random import random
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self, IC=ActiveTraits(), FC=ActiveTraits()):
        super().__init__('trajectory_generator_node')
        # Topic stuff
        self.publisher_ = self.create_publisher(DiscretizedTrajectoryReference, '/current_trajectory', 10)
        self.internals_publisher_ = self.create_publisher(Float32MultiArray, '/trajectory_internals', 10)
        self.subscription_ = self.create_subscription(RigidBodyTick, "/player0/RigidBodyTick", self.update_bot_data, 10) 
        self.trajectory_service = self.create_service(GetOptimalTrajectory, "/get_trajectory", self.get_trajectory_service_callback)
        # Optimizer stuff
        self.optimizer = TrajectoryOpti()
        self.bot_rbt = RigidBodyTick()
        self.sol = None
        self.current_trajectory = CubicSpline([0,1,2,3], [1,2,3,4])
        # self.publish_trajectory(IC,FC)
        # self.init_optimizer()
        #TODO: Change callback to not take args, use class vars instead
        # self.timer = self.create_timer(1, self.init_optimizer)
        # Service stuff

    def get_trajectory_service_callback(self, request: GetOptimalTrajectory.Request, response: GetOptimalTrajectory.Response):
        IC = ActiveTraits(request.ic.active, request.ic.values)
        FC = ActiveTraits(request.fc.active, request.fc.values)
        trajectory = self.calculate_trajectory(IC, FC)
        response.trajectory = response.trajectory = trajectory
        return response

    def update_bot_data(self, msg: RigidBodyTick):
        self.bot_rbt = msg
    
# Obsolete/Deprecated?
    def init_optimizer(self):
        pos = self.bot_rbt.bot_state.pose.position
        vel = self.bot_rbt.bot_state.twist.linear
        vmag = self.bot_rbt.bot_state.vmag
        yaw = self.bot_rbt.yaw
        omega = self.bot_rbt.bot_state.twist.angular
        ball_pos = self.bot_rbt.ball_state.pose.position
        
        IC = ActiveTraits([1,1,1,1,1,1,1], [pos.x, pos.y, vel.x, vel.y, yaw, omega.z, vmag])
        FC = ActiveTraits([1,1,0,0,0,0,0], [2000, 2000, 0,0,0,0, 0])
        self.publish_trajectory(IC, FC)

    def calculate_trajectory(self, \
                        IC = ActiveTraits([1,1,1,1,1,1,1],[0, 0, 0, 0, 1.5, 0,0]),\
                        FC = ActiveTraits([1,1,0,0,1 ,0,0], [1000, 1000, 0, 0, 1.5, 0, 0])):
        try:
            self.sol = self.optimizer.reset_optimizer(IC, FC)

            sol = self.sol
            tf = sol.value(self.optimizer.T)
            t = np.linspace(0,tf, self.optimizer.N+1)
            x = sol.value(self.optimizer.X[0,:])
            y = sol.value(self.optimizer.X[1,:])
            xdot = sol.value(self.optimizer.X[2,:])
            ydot = sol.value(self.optimizer.X[3,:])
            theta = sol.value(self.optimizer.X[4,:])
            thetadot = sol.value(self.optimizer.X[5,:])
            # v = np.sqrt(xdot**2 + ydot**2)
            throttle = sol.value(self.optimizer.U[0,:])
            steer = sol.value(self.optimizer.U[1,:])
            # self.current_trajectory = CubicSpline(t, np.vstack((x,y)).T)
            # z = np.polyfit(t, np.vstack((x,y)).T, deg=3)
            # newspline = PPoly.construct_fast(self.current_trajectory.c, self.current_trajectory.x)
            # teval = np.linspace(0,tf,101)
            # xpoly = np.poly1d(z[:,0])
            # ypoly = np.poly1d(z[:,1])

            msg = DiscretizedTrajectoryReference()
            msg.x              = x.tolist()
            msg.y              = y.tolist()
            msg.xdot           = xdot.tolist()
            msg.ydot           = ydot.tolist()
            msg.theta          = theta.tolist()
            msg.thetadot       = thetadot.tolist()
            msg.vmag           = sol.value(self.optimizer.X[-1,:]).tolist()
            msg.acceleration   = throttle.tolist()
            msg.steer          = steer.tolist()
            msg.tf             = tf

        except:
            import traceback
            traceback.print_exc()

    def publish_trajectory(self, msg: DiscretizedTrajectoryReference):
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publish DiscretizedTrajectoryReference: {msg}")
            
def main(args=None):
    rclpy.init(args=args)
    IC = ActiveTraits([1,1,1,1,1,1,1],[0, 0, 0, 0, 1.5, 0,0])
    FC = ActiveTraits([1,1,0,0,1 ,0,0], [1000, 1000, 0, 0, 1.5, 0, 0])
    node = TrajectoryGenerator(IC, FC)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
