import rclpy
from rclpy.node import Node
from trajectory_generator_pkg.sample_trajectory_generator import *
from scipy.interpolate import CubicSpline, PPoly
import matplotlib.pyplot as plt
from rlbot_msgs.msg import Polynomial3

class TrajectoryGenerator(Node):
    def __init__(self, IC=ActiveTraits(), FC=ActiveTraits()):
        super().__init__('trajectory_generator_node')
        # Topic stuff
        self.publisher_ = self.create_publisher(Polynomial3, '/current_trajectory', 10)
        
        # Optimizer stuff
        self.optimizer = TrajectoryOpti()
        self.sol = None
        self.current_trajectory = CubicSpline([0,1,2,3], [1,2,3,4])
        self.init_optimizer(IC,FC)
        #TODO: Change callback to not take args, use class vars instead
        self.timer = self.create_timer(1, self.init_optimizer)
        # Service stuff


    def init_optimizer(self, \
                        IC = ActiveTraits([1,1,1,1,1,1,1],[0, 0, 0, 0, 1.5, 0,0]),\
                        FC = ActiveTraits([1,1,0,0,1 ,0,0], [1000, 1000, 0, 0, 1.5, 0, 0])):
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
        v = np.sqrt(xdot**2 + ydot**2)
        throttle = sol.value(self.optimizer.U[0,:])
        steer = sol.value(self.optimizer.U[1,:])
        self.current_trajectory = CubicSpline(t, np.vstack((x,y)).T)
        z = np.polyfit(t, np.vstack((x,y)).T, deg=3)
        newspline = PPoly.construct_fast(self.current_trajectory.c, self.current_trajectory.x)
        teval = np.linspace(0,tf,101)
        xpoly = np.poly1d(z[:,0])
        ypoly = np.poly1d(z[:,1])
        
        # plt.figure(1)
        # plt.plot(xpoly(teval), ypoly(teval), 'r.')
        # plt.plot(self.current_trajectory(teval)[:,0], self.current_trajectory(teval)[:,1], 'b.')
        # plt.show(block=False)
        # plt.pause(0.01)

        msg = Polynomial3()
        msg.px = z[:,0].astype(np.float32)
        msg.py = z[:,1].astype(np.float32)
        msg.deg = 3
        msg.tf = tf
        self.get_logger().info(f"Publish Polynomial: {msg}")

        self.publisher_.publish(msg)
        

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
