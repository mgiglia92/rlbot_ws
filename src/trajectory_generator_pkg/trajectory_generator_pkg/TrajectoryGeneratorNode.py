import rclpy
from rclpy.node import Node
from trajectory_generator_pkg.sample_trajectory_generator import *
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from rlbot_msgs.msg import TrajectorySeed
from rlbot_msgs.srv import GetOptimalTrajectory
import random

class TrajectoryMsgHelper(TrajectorySeed):
    spline_trajectory: CubicSpline
    populated= False

    @staticmethod
    def init_from_solution(sol, optimizer: TrajectoryOpti):
        tf = sol.value(optimizer.T)
        t = np.linspace(0,tf, optimizer.N+1)
        x = sol.value(optimizer.X[0,:])
        y = sol.value(optimizer.X[1,:])
        # xdot = sol.value(optimizer.X[2,:])
        # ydot = sol.value(optimizer.X[3,:])
        theta = sol.value(optimizer.X[4,:])
        thetadot = sol.value(optimizer.X[5,:])
        vmag = sol.value(optimizer.X[6,:])
        accel = sol.value(optimizer.U[0,:])
        steer = sol.value(optimizer.U[1,:])

        seed = TrajectoryMsgHelper()
        seed.populate(t*tf,x,y,vmag,theta,thetadot,steer,accel, optimizer.IC, optimizer.FC)
        return seed

    # Populate the message
    def init_from_seed(self, seed: TrajectorySeed):
        self.t =        seed.t
        self.x =        seed.x
        self.y =        seed.y
        self.vmag =     seed.vmag
        self.theta =    seed.theta
        self.thetadot = seed.thetadot
        self.steer =    seed.steer
        self.accel =    seed.accel
        # self.IC =       seed.IC
        # self.FC =       seed.FC
        self.populated = True

    def populate(self, t, x, y, vmag, theta, thetadot, steer, accel, IC, FC):
        for i in [t,x,y,vmag,theta,thetadot]:
            assert len(i) == TrajectorySeed().N+1
        assert len(steer) == TrajectorySeed().N
        assert len(accel) == TrajectorySeed().N

        self.t = t
        self.x = x
        self.y = y
        self.vmag = vmag
        self.theta = theta
        self.thetadot = thetadot
        self.steer = steer
        self.accel = accel
        self.IC = IC
        self.FC = FC

        self.populated = True

    def construct_spline(self):
        if(self.populated):
            self.spline_trajectory = CubicSpline(self.t, np.vstack((self.x,self.y, self.vmag, self.theta, self.thetadot)).T)
        else:
            print("Trajectory Not Populated")

    def plot_trajectory(self):
        plt.figure(1)
        # state = self.spline_trajectory(self.t)
        plt.plot(self.x, self.y,'g.-', markersize=10, label='xy path')
        try:
            plt.plot(self.spline_trajectory(self.t)[:,0], self.spline_trajectory(self.t)[:,1], 'bo', markersize=1, label='[x,y]=poly(t)')
        except:
            print("Failed to plot spline trajectory")
        plt.plot(self.IC.values[0],self.IC.values[1],'y*-', label='initial conditions')
        plt.plot(self.FC.values[0], self.FC.values[1], 'b*-', label='final conditions')
        # plt.legend()
        plt.figure(2)
        plt.plot(self.t, self.vmag, 'g.-', label='vel(t)')
        plt.legend()
        plt.figure(3)
        plt.plot(self.t, self.thetadot, 'b.-', label='thetadot')
        plt.legend()
        plt.figure(4)
        plt.plot(self.t, self.theta, 'r*-', label='theta')
        plt.legend()
        plt.show(block=False)
        plt.pause(1)
    
    def get_trajectory_seed(self):
        if self.populated:
            seed = TrajectorySeed()
            seed.t = self.t
            seed.x = self.x
            seed.y = self.y
            seed.vmag = self.vmag
            seed.theta = self.theta
            seed.thetadot = self.thetadot
            seed.steer = self.steer
            seed.accel = self.accel

            return seed
        raise Exception('Trajectory Seed not populated')
    
class TrajectoryGenerator(Node):
    def __init__(self, IC=ActiveTraits(), FC=ActiveTraits()):
        super().__init__('trajectory_generator_node')
        # Topic stuff
        self.publisher_ = self.create_publisher(TrajectorySeed, '/current_trajectory', 10)
        self.trajectory_generation_service = self.create_service(GetOptimalTrajectory, '/get_optimal_trajectory', self.get_optimal_trajectory)
        
        self.IC = IC
        self.FC = FC
        # Optimizer stuff
        self.optimizer = TrajectoryOpti()
        self.sol = None
        self.current_trajectory = CubicSpline([0,1,2,3], [1,2,3,4])
        # self.init_optimizer()
        # self.timer = self.create_timer(1, self.init_optimizer)
        # Service stuff

    def get_optimal_trajectory(self, req: GetOptimalTrajectory.Request, res: GetOptimalTrajectory.Response):
        IC = ActiveTraits(req.ic.active, req.ic.values)
        FC = ActiveTraits(req.fc.active, req.fc.values)
        self.update_active_traits(IC,FC)
        self.sol = self.optimizer.reset_optimizer(self.IC, self.FC, TrajectorySeed().N)
        trajectory = TrajectoryMsgHelper.init_from_solution(self.sol, self.optimizer)
        res.trajectory = trajectory.get_trajectory_seed()
        return res

    def set_random_active_traits(self):
        # self.IC.values = np.array(self.IC.values) * random.random()
        self.FC.values[4] = np.mod(self.FC.values[4] - 0.3, 2*np.pi)#random.uniform(-1*np.pi, np.pi)

    def update_active_traits(self, IC: ActiveTraits, FC: ActiveTraits):
        self.IC = IC
        self.FC = FC

    def init_optimizer(self):
        self.set_random_active_traits()
        try:
            self.sol = self.optimizer.reset_optimizer(self.IC, self.FC, TrajectorySeed().N)
            print("")
            seed = TrajectoryMsgHelper.init_from_solution(self.sol, self.optimizer)
        except:
            debug = self.optimizer.debug
            seed = TrajectoryMsgHelper.init_from_solution(debug, self.optimizer)

        seed.construct_spline()
        seed.plot_trajectory()
        self.get_logger().info(f"Publish Trajectory Seed: {seed}")

        self.publisher_.publish(seed.get_trajectory_seed())
        

def main(args=None):
    rclpy.init(args=args)
    IC = ActiveTraits([1,1,1,1,1,0,1],[0, 0, 0, 0, 0, 0, 200])
    FC = ActiveTraits([1,1,0,0,1,0,1], [500, 1000, 0, 0, 3*np.pi/2, 0, 1200])
    node = TrajectoryGenerator(IC, FC)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
