import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Future

from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from std_msgs.msg import Int32, Float32
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import DiscretizedTrajectoryReference, ActiveTraitsMsg
from rlbot_msgs.srv import SetGains, TwistSetpoint, GetOptimalTrajectory
from rlbot_msgs.action import ExecuteTrajectory
from rlbot_msgs.action._execute_trajectory import ExecuteTrajectory_Result
from threading import Thread

from transforms3d.quaternions import rotate_vector
from transforms3d.euler import quat2euler
from transforms3d.utils import normalized_vector
from transforms3d.axangles import axangle2mat
import numpy as np
from pyquaternion import Quaternion
from simple_controller_pkg.controller_util import get_best_steering_and_throttle, PIDStruct
import argparse

# parser = argparse.ArgumentParser(
#                     prog='SimplerController',
#                     description='Controls Rlbot Agent',
#                     epilog='No epilog')

# parser.add_argument('-v', '--velocity', type=float, default=1000.0)
# parser.add_argument('-w', '--angvel', type=float, default=2.0)

# args, unknown = parser.parse_known_args()

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return -1*np.arctan2(np.dot(np.cross(v1_u, v2_u), [0,0,1]), np.dot(v1_u, v2_u))

class StanleyControllerNode(Node):
    gains = PIDStruct()
    trajectory = DiscretizedTrajectoryReference()
    latest_body_state = RigidBodyTickMsg()
    def __init__(self, node_name="stanley_controller", **kwargs):
        super().__init__(node_name)

        self.action_callback_group = ReentrantCallbackGroup()

        self.publisher_ = self.create_publisher(ControllerReference, "/controller_reference", 10)
        self.publisher2_ = self.create_publisher(Twist, "/internals_stanley", 10)
        self.subscription_ = self.create_subscription(DiscretizedTrajectoryReference, "/current_trajectory", 
                                                      self.update_trajectory, 10)
        self.body_subscription_ = self.create_subscription(RigidBodyTickMsg, "/player0/RigidBodyTick", 
                                                           self.stanley_callback, 10, callback_group=self.action_callback_group)
        self.trajectory_service_client = self.create_client(GetOptimalTrajectory,"/get_trajectory", callback_group=self.action_callback_group)
        self.execute_trajectory_action_server = ActionServer(self, 
                                                             ExecuteTrajectory,
                                                             '/execute_trajectory_action',
                                                             self.action_callback,

                                                             callback_group=self.action_callback_group)
        # self.services_ = [self.create_service(SetGains, "/simple_controller/set_gains", self.service_callback),
        #                   self.create_service(TwistSetpoint, "/simple_controller/twist_setpoint", self.setpoint_callback)]
        self.i = 0


        # MultiThreaded Executor allows ros to call callbacks in parallel and use callback groups to define execution semantics
        self.exec = MultiThreadedExecutor()
        self.exec.add_node(self)
        self.executor_thread = Thread(target = self.executor_func, daemon=True)
        self.executor_thread.start()

    def executor_func(self):
        try:
            while True:
                self.exec.spin_once()
        
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            raise rclpy.executors.ExternalShutdownException

    def get_ic_from_rigid_body_tick(self, rbt: RigidBodyTickMsg) -> ActiveTraitsMsg:
        pos = rbt.bot_state.pose.position
        vmag = rbt.bot_state.vmag
        yaw = rbt.yaw
        vx = vmag*np.cos(rbt.yaw)
        vy = vmag*np.sin(rbt.yaw)

        #TODO: Turn this process into a utility

        ic = ActiveTraitsMsg()
        for each in [1,1,0,0,1,0,1]:
            a=Int32()
            a.data=int(each)
            ic.active.append(a)
        for each in [pos.x, pos.y, vx, vy, yaw, 0.0, vmag]:
            a=Float32()
            a.data=float(each)
            ic.values.append(a)
        return ic

    def generate_random_final_constraints(self) -> ActiveTraitsMsg:
        pos = self.latest_body_state.bot_state.pose.position
        vmag = self.latest_body_state.bot_state.vmag
        fc = ActiveTraitsMsg()
        for each in [1,1,0,0,0,0,1]:
            a=Int32()
            a.data=int(each)
            fc.active.append(a)
        for each in [-1*pos.x, -1*pos.y, 0.0, 0.0, 0.0, 0.0, np.random.random(1)*vmag]:
            a=Float32()
            a.data=float(each)
            fc.values.append(a)
        return fc

    def action_callback(self, goal_handle: ServerGoalHandle):
        # Try calling service
        while not self.trajectory_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{self.trajectory_service_client.srv_name} not available, trying again...")

        self.req = GetOptimalTrajectory.Request()
        self.req.ic = self.get_ic_from_rigid_body_tick(self.latest_body_state)
        #TODO: Use goal lhandle here
        self.req.fc = self.generate_random_final_constraints()
        self.trajectory_service_future = Future()
        self.trajectory_service_future = self.trajectory_service_client.call_async(self.req)
        import time
        while not self.trajectory_service_future.done():
            self.get_logger().info(f"{self.trajectory_service_client.srv_name} service not completed yet")
        GoalResponse.ACCEPT
        trajectory = DiscretizedTrajectoryReference()
        trajectory = self.trajectory_service_future.result().trajectory

        self.update_trajectory(trajectory)
        goal_handle.succeed()

        ret = ExecuteTrajectory_Result()
        ret.succeed = True
        self.get_logger().info("ACTION COMPLETED")
        return ret


    def update_trajectory(self, msg: DiscretizedTrajectoryReference):
        self.trajectory = msg

    def stanley_callback(self, msg: RigidBodyTickMsg):
        # Update local data
        self.latest_body_state = msg
        # Exit if trajetory has no data
        if(len(self.trajectory.x) == 0):
            return
        # Put relevant data into local variables for ease of reading

        bot_state = msg.bot_state
        pos = np.array([msg.bot_state.pose.position.x, msg.bot_state.pose.position.y, 0])
        vel = np.array([msg.bot_state.twist.linear.x, msg.bot_state.twist.linear.y, 0])
        vmag = bot_state.vmag

        # Trajectory Position
        x = np.array([self.trajectory.x.tolist()])
        y = np.array([self.trajectory.y.tolist()])
        z = np.zeros(x.shape)
        pos_t = np.concatenate([x,y,z],axis=0).T # Transpose to get xyz vector on trailing axis for subtraction
        # Trajectory Velocity
        vx = np.array([self.trajectory.xdot.tolist()])
        vy = np.array([self.trajectory.ydot.tolist()])
        vz = np.zeros(vx.shape)
        vel_t = np.concatenate([vx,vy,vz],axis=0).T
        

        # Find point on trajectory closest to current body position
        dist_vec = pos_t-pos
        dist_mag = np.linalg.norm(dist_vec, axis=1) # Get the norm of the matrix along the trailing axis (magnitude of each |xyz| vector)
        min_index = np.argmin(dist_mag)
        pos_t_min = pos_t[min_index]

        # Calculate the Cross Track Error (CTE)
        vec_to_path_world = dist_vec[min_index]
        R = axangle2mat([0,0,1], np.pi/2)
        ctvec_body = np.dot(R,vec_to_path_world)
        cte = np.linalg.norm(vec_to_path_world)
        trajectory_velocity = vel_t[min_index]
        if(angle_between(unit_vector(vel), unit_vector(vec_to_path_world)) > 0):
            cte = -1*cte

        # Calculate the Heading Error (HE)
        he = angle_between(unit_vector(trajectory_velocity), unit_vector(vel))
        # if(he > np.pi):
        #     he = he - np.pi

        twist = Twist()
        twist.angular.x = he
        twist.angular.y = cte
        twist.angular.z = 0.0

        unit_vel = unit_vector(vel)
        # Put data into ControllerReference msg for publishing
        cr = ControllerReference()
        cr.headingx = unit_vel[0]
        cr.headingy = unit_vel[1]
        cr.rbt = msg
        cr.he = he
        cr.cte = cte
        cr.correction = 2*he + np.arctan2(3*cte, (0.001+vmag))
        cr.v_desired = float(1000)
        cr.w_desired = float(np.clip(2*he + np.arctan2(3*cte, (0.001+vmag)), -5.5, 5.5))
        cr.desired_pos = Vector3(x=pos_t_min[0], y=pos_t_min[1], z=pos_t_min[2])
        # cr.w_desired = float(np.clip(he, -5.5, 5.5))
        self.publisher_.publish(cr)
        self.publisher2_.publish(twist)
        self.get_logger().info(f"Published: cte:{cte} | he: {he} | angle:{ctvec_body}")

    def norm(self, vec) -> np.array:
        return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2) * np.sign()

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

    # rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    import time
    while controller.executor_thread.is_alive():
        time.sleep(1)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()