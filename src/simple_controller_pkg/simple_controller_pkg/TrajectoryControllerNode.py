import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from rlbot_msgs.msg import TrajectorySeed, RigidBodyTick
from trajectory_generator_pkg.TrajectoryGeneratorNode import TrajectoryMsgHelper
from simple_controller_pkg.controller_util import AccelerationRelationship, SteeringRelationship

import numpy as np

class TrajectoryHistory:
    state_history:      list[RigidBodyTick]
    controller_history: list[Twist]
    def __init__(self):
        pass

    def push(self, state: RigidBodyTick, control: Twist):
        self.state_history.append(state)
        self.controller_history.append(control)
    
    def clear(self):
        self.state_history = []
        self.controller_history = []

class TrajectoryControllerStateMachine:
    t0: np.float32
    t1: np.float32
    trajectory = TrajectoryMsgHelper()
    state: np.int32
    control_input = Twist()
    trajectory_history = TrajectoryHistory()

    def __init__(self):
        self.state=2 # Start on dwell state
        self.trajectory = TrajectoryMsgHelper()
    
    def reset_state_machine(self, traj: TrajectoryMsgHelper):
        self.state = 0
        self.trajectory = traj
        self.trajectory.construct_spline()


    def update(self, rbt: RigidBodyTick):
        self.t1 = rbt.time
        tf = self.trajectory.t[-1] # Get last timestamp from trajectory

        if self.state == 0: # Prep/update
            self.state = 1
            self.t0 = rbt.time
            self.trajectory_history.clear() 
        elif self.state == 1:
            if(self.t1 - self.t0 > tf):
                self.state = 2
            else:
                self.update_control(rbt)
                self.trajectory_history.push(rbt, self.control_input)
        elif self.state == 2: # Dwell
            self.control_input = Twist()
            pass
    
    def update_control(self, rbt: RigidBodyTick):
        # Feed forward only
        tn = self.t1-self.t0
        spline = self.trajectory.input_trajectory
        input = spline(tn)[-2:]
        accel = input[0]
        steer = self.trajectory.spline_trajectory(tn)[4]
        vmag = rbt.bot_state.vmag
        accel_norm = accel/AccelerationRelationship.getAmax(vmag)
        steer_norm = steer/SteeringRelationship.getWMax(vmag)
        if(accel_norm > 1.0):
            pass
            # Get desired vel 0.1 seconds in future
            v_desired = self.trajectory.spline_trajectory(tn+0.1)[2]
            # Forward simulate current velocity with 0.1 sec boost
            v_predict = vmag + (0.1*991.666)
            # If prediction is < 1.1*desired then apply boost
            if(v_predict < (1.05*v_desired)):
                self.control_input.linear.y = 1.0
            else:
                self.control_input.linear.y = 0.0
        self.control_input.linear.x = np.clip(accel_norm, -1, 1)
        self.control_input.angular.z = np.clip(steer_norm, -1, 1)
        

class TrajectoryControllerNode(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.state_subscription = self.create_subscription(RigidBodyTick ,'/player0/RigidBodyTick', self.control_loop, 10)
        self.trajectory_subscriber = self.create_subscription(TrajectorySeed, '/optimal_trajectory', self.update_trajectory, 10)
        self.input_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_machine = TrajectoryControllerStateMachine()

    def update_trajectory(self, seed: TrajectorySeed):
        traj = TrajectoryMsgHelper()
        traj.init_from_seed(seed)
        self.state_machine.reset_state_machine(traj)

    def control_loop(self, rbt: RigidBodyTick):
        self.state_machine.update(rbt)
        self.input_publisher.publish(self.state_machine.control_input)

def main(args=None):
    rclpy.init(args=args)
    t = TrajectoryControllerNode()
    rclpy.spin(t)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()