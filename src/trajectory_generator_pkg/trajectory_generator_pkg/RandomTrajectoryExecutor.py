import rclpy
from rclpy.node import Node
from trajectory_generator_pkg.sample_trajectory_generator import ActiveTraits
from rlbot_msgs.msg import TrajectorySeed
from rlbot_msgs.msg import ActiveTraits as ActiveTraitsMsg
from rlbot_msgs.srv import GetOptimalTrajectory
from trajectory_generator_pkg.TrajectoryGeneratorNode import TrajectoryMsgHelper
import numpy as np
from rlbot_msgs.srv import ResetGameState

class RandomTrajectoryExecutor(Node):
    req = GetOptimalTrajectory.Request()
    
    def __init__(self):
        super().__init__('random_trajectory_executor')
        self.trajectory_publisher = self.create_publisher(TrajectorySeed, '/optimal_trajectory', 10)
        self.traj_cli = self.create_client(GetOptimalTrajectory, '/get_optimal_trajectory')
        self.reset_cli = self.create_client(ResetGameState, '/reset_game_state')
        while not self.traj_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetOptimalTrajectory service not available, waiting again...')
        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ResetGameState service not available, waiting again...')
        
        self.traj_req = GetOptimalTrajectory.Request()
        self.reset_req= ResetGameState.Request()
    
    def execute(self, IC: ActiveTraits, FC: ActiveTraits) -> TrajectorySeed:
        # Send reqest to service
        ic = ActiveTraitsMsg()
        fc = ActiveTraitsMsg()
        ic.active = IC.active
        ic.values = IC.values
        fc.active = FC.active
        fc.values = FC.values
        self.req.ic = ic
        self.req.fc = fc
        self.future = self.traj_cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        # Publish Trajectory to Stanley Controller
        result = self.future.result()

        return result.trajectory

def reset_game_request_from_active_traits(IC: ActiveTraits, FC: ActiveTraits) -> ResetGameState.Request:
    request = ResetGameState.Request()
    request.rigid_body_tick.bot_state.pose.position.x   = float(IC.values[0])
    request.rigid_body_tick.bot_state.pose.position.y   = float(IC.values[1])
    request.rigid_body_tick.bot_state.pose.position.z   = float(0.0)
    request.rigid_body_tick.roll                        = float(0.0)
    request.rigid_body_tick.pitch                       = float(0.0)
    request.rigid_body_tick.yaw                         = float(IC.values[4])
    request.rigid_body_tick.bot_state.vmag              = float(IC.values[6])
    request.rigid_body_tick.bot_state.twist.linear.x    = float(IC.values[6] * np.cos(IC.values[4]))
    request.rigid_body_tick.bot_state.twist.linear.y    = float(IC.values[6] * np.sin(IC.values[4]))

    request.rigid_body_tick.ball_state.pose.position.x  = float(FC.values[0])
    request.rigid_body_tick.ball_state.pose.position.y  = float(FC.values[1])
    request.rigid_body_tick.ball_state.twist.linear.x   = float(np.cos(FC.values[4])*FC.values[6])
    request.rigid_body_tick.ball_state.twist.linear.y   = float(np.sin(FC.values[4])*FC.values[6])

    return request

def main():
    IC = ActiveTraits([1,1,0,0,1,0,0], [0,0,0,0,np.pi/2,0,0])
    FC = ActiveTraits([1,1,0,0,1,0,1], [500,1000,0,0,3*np.pi/2,0,1200])

    rclpy.init()
    exec = RandomTrajectoryExecutor()
    # Get trajectory
    response = exec.execute(IC,FC)
    # Publish new trajectory to topic
    trajectory = TrajectoryMsgHelper()
    trajectory.init_from_seed(response)
    exec.trajectory_publisher.publish(trajectory.get_trajectory_seed())
    # Reset Game State

    future = exec.reset_cli.call_async(reset_game_request_from_active_traits(IC, FC))
    rclpy.spin_until_future_complete(exec, future)
    result = future.result()
    
    print("DEBUG")

if __name__ == "__main__":
    main()