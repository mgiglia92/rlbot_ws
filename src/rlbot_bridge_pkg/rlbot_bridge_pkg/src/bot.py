from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.game_state_util import GameState, Physics, Rotator, Vector3, CarState, BallState, GameInfoState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.rigid_body_struct import RigidBodyTick

from typing import Optional

from threading import Thread
import os
import sys
import traceback
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from rclpy.serialization import serialize_message
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from ros2node.api import get_node_names
import rosbag2_py
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
# from rclpy.qos import QoSProfile
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.srv import ResetGameState
from rlbot_msgs.action import ControlVelocity
import numpy as np
import time

# if(not rclpy.ok()):
#     rclpy.init()

class AgentLifecycleNode(LifecycleNode):


    def __init__(self, node_name, **kwargs):
        self._count: int = 0
        self._pub: Optional[LifecyclePublisher] = None
        self._timer: Optional[Timer] = None
        self._srv = None
        self.ros_controls = SimpleControllerState()

        super().__init__(node_name, **kwargs)

        self.publisher_callback_group = ReentrantCallbackGroup()
        self.subscriber_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.exec = MultiThreadedExecutor()
        print(type(self.exec))
        self.exec.add_node(self)
        # self.executor.add_node(self.control_velocity_action_server)
        self.kill_executor_thread = False
        self.executor_thread = Thread(target=self.executor_func, daemon=True)
        self.executor_thread.start()

    def publish(self):
        msg = String()
        msg.data = f"I\'m a lifecycle node publishing right now, here my count: {self._count}"
        self._count += 1

        # Print the current state for demo purposes
        if self._pub is None or not self._pub.is_activated:
            self.get_logger().info('Lifecycle publisher is inactive. Messages are not published.')
        else:
            self.get_logger().info(f'Lifecycle publisher is active. Publishing: [{msg.data}]')

        # 
        if self._pub is not None:
            self._pub.publish(msg)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self._pub = self.create_lifecycle_publisher(RigidBodyTickMsg, f"/player0/RigidBodyTick", 10, callback_group=self.publisher_callback_group)
        self._sub = self.create_subscription(Twist, '/cmd_vel', self.update_controls, 10)
        self._srv = self.create_service(ResetGameState, 'reset_game_state', self.reset_game_state, callback_group=self.service_callback_group)
        self.get_logger().info('configure called')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('activate called')
        return super().on_activate(state)
    
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('deactivate called')
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('cleanup called')
        # self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)
        if self._srv is not None: self.destroy_service(self._srv)
        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('shutdown called')
        # self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)
        if self._srv is not None: self.destroy_service(self._srv)
        self.kill_executor_thread = True     
        return super().on_cleanup(state)

    def executor_func(self):
        try:
            while True:
                self.exec.spin_once()
                # self.get_logger().info("EXEC SPIN")
                if self.kill_executor_thread:
                    self.get_logger().info("THREAD KILL")
                    return
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            raise rclpy.executors.ExternalShutdownException

    def update_controls(self, msg: Twist):
        self.get_logger().debug('I heard: "%s"' % msg.linear.x)
        self.ros_controls.throttle = np.clip(msg.linear.x, -1, 1)
        self.ros_controls.steer = np.clip(msg.angular.z, -1 ,1)

    def reset_game_state(self, request, response):
        try:
            self.get_logger().info(f"Incoming request: {request}")
            car = request.rigid_body_tick.bot_state
            carp = car.pose.position
            caro = car.pose.orientation
            carv = car.twist.linear
            carw = car.twist.angular
            ball = request.rigid_body_tick.ball_state.pose.position
            car_state = CarState(boost_amount=100,
                                physics=Physics(location = Vector3(x=carp.x, y=carp.y), velocity=Vector3(x=carv.x,y=carv.y), rotation=Rotator(0, 0, 0),
                                angular_velocity=Vector3(carw.x, carw.y, carw.z)))
            ball_state = BallState(Physics(location=Vector3(ball.x, ball.y, ball.z)))
            game_info_state = GameInfoState(world_gravity_z=-660)
            game_state = GameState(ball=ball_state, cars={self.index: car_state}, game_info=game_info_state)
            self.set_game_state(game_state)
            response.success = True
            return response
        
        except Exception as e:
            response.success = False
            import traceback
            self.get_logger().warn(f"{traceback.print_exc()}")
            
        return response


def main():
    executor = rclpy.executors.SingleThreadedExecutor()
    node = AgentLifecycleNode('agent_lifecycle')
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        raise rclpy.executors.ExternalShutdownException
    
if __name__ == '__main__':
    main()


class MyBot(BaseAgent, AgentLifecycleNode):
    

    def __init__(self, name, team, index):
        if(not rclpy.ok()):
            rclpy.init()
        print("INITIALIZING MyBot")
        # Get list of nodes and searchr ename if necessary 
        ros2namespace = get_node_names(node=Node("temp"))
        node_list = []
        for n in ros2namespace:
            node_list.append(n.name)
        k=0
        for k, r in enumerate(node_list):
            if "rlbot_node"+str(k) in node_list:
                    k+=1
        self.node_name = "rlbot_node" + str(k)
        AgentLifecycleNode.__init__(self, self.node_name)
        BaseAgent.__init__(self, name, team, index)

        try:
            ret = os.system(f"ros2 lifecycle set /{self.node_name} configure")
            if ret > 0: raise SystemError
            ret = os.system(f"ros2 lifecycle set /{self.node_name} activate")
            if ret > 0: raise SystemError
        except SystemError:
            traceback.print_exc()
    
    def retire(self):
        try:
            ret = os.system(f"ros2 lifecycle set /{self.node_name} deactivate")
            if ret > 0: raise SystemError
            ret = os.system(f"ros2 lifecycle set /{self.node_name} cleanup")
            if ret > 0: raise SystemError
            ret = os.system(f"ros2 lifecycle set /{self.node_name} shutdown")
            if ret > 0: raise SystemError
                           
            self.destroy_node()
            self.exec.shutdown()
            self.executor_thread.join()
        except SystemError:
            traceback.print_exc()

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        rotator = packet.game_cars[self.index].physics.rotation
        msg = self.populate_rigid_body_tick_message(packet.game_info.seconds_elapsed, rotator)
        if self._pub is not None:
            # self.get_logger().info(f"get_output called")
            self._pub.publish(msg)
        return self.ros_controls

    def populate_rigid_body_tick_message(self, time: float, rotation: Rotator):
        gtp = RigidBodyTick()
        gtp = self.get_rigid_body_tick()
        msg = RigidBodyTickMsg()
        q = gtp.players[self.index].state.rotation
        p = gtp.players[self.index].state.location
        v = gtp.players[self.index].state.velocity
        w = gtp.players[self.index].state.angular_velocity
        throttle = gtp.players[self.index].input.throttle
        steer = gtp.players[self.index].input.steer
        msg.bot_state.pose.orientation.x = q.x
        msg.bot_state.pose.orientation.y = q.y
        msg.bot_state.pose.orientation.z = q.z
        msg.bot_state.pose.orientation.w = q.w
        msg.bot_state.pose.position.x = p.x
        msg.bot_state.pose.position.y = p.y
        msg.bot_state.pose.position.z = p.z
        msg.bot_state.twist.linear.x = v.x
        msg.bot_state.twist.linear.y = v.y
        msg.bot_state.twist.linear.z = v.z
        msg.bot_state.twist.angular.x = w.x
        msg.bot_state.twist.angular.y = w.y
        msg.bot_state.twist.angular.z = w.z
        # self.get_logger().info(f"angvel: {w.z}")
        msg.bot_state.throttle = throttle
        msg.bot_state.steer = steer
        msg.bot_state.vmag = np.linalg.norm([v.x, v.y, v.z])

        msg.roll = rotation.roll
        msg.pitch = rotation.pitch
        msg.yaw = rotation.yaw

        msg.time = time
        
        return msg

    def initialize_agent(self):
        if(not rclpy.ok()):
            rclpy.init()


class MyBot2(BaseAgent, Node):
    node: Node

    def __init__(self, name, team, index):
        self.test = False
        self.rlbotneeds = type('',(),{})
        self.rlbotneeds.name = name
        self.rlbotneeds.team = team
        self.rlbotneeds.index = index
        self.test = True
        rclpy.shutdown()
        if(not rclpy.ok()):
            rclpy.init()
                # Check ros name to not make multiple nodes with same name
        ros2namespace = get_node_names(node=Node("temp"))
        node_list = []
        for n in ros2namespace:
            node_list.append(n.name)
        k=0
        for k, r in enumerate(node_list):
            if "rlbot_node"+str(k) in node_list:
                    k+=1
        node_name = "rlbot_node" + str(k)
        # super().__init__(name, team, index)
        BaseAgent.__init__(self, self.rlbotneeds.name, self.rlbotneeds.team, self.rlbotneeds.index)
        self.node = Node(node_name)
        self.kill_node=False

    def ros_init(self):
        # self = Node("rlbot_node")
        self.publisher_callback_group = ReentrantCallbackGroup()
        self.subscriber_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
    # Topics
        self.publisher_ = self.node.create_publisher(RigidBodyTickMsg, f"/player0/RigidBodyTick", 10, callback_group=self.publisher_callback_group)
        self.subscriber_ = self.node.create_subscription(Twist, "/cmd_vel", self.listener_callback, 10, callback_group=self.subscriber_callback_group)
        self.subscriber_
    # Services
        self.srv = self.node.create_service(ResetGameState, 'reset_game_state', self.run_srv, callback_group=self.service_callback_group)
    # Actions
        self.control_velocity_action_server = \
            ActionServer(self.node, ControlVelocity, 'ctrl_vel', self.control_velocity_action, callback_group=self.action_callback_group)
        # self.publisher_ = self.create_publisher(GameTickPacketMsg, '/GameTickPacket',10)
    # RosBag
        self.write_active = False
        self.bag_writer = rosbag2_py.SequentialWriter()
        # Change names cause rosbag wont overwrite
        i=0
        while(os.path.exists('bags/my_bag'+str(i))):
            i += 1
        storage_options = rosbag2_py.StorageOptions(
            uri='bags/my_bag'+str(i), storage_id='sqlite3')
        
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.bag_writer.open(storage_options, converter_options)
        topic_info = rosbag2_py.TopicMetadata(
            name = "/cmd_vel_bag",
            type="geometry_msgs/msg/Twist",
            serialization_format='cdr')
        topic_info2 = rosbag2_py.TopicMetadata(
            name = "/rbt_bag",
            type = "rlbot_msgs/msg/RigidBodyTick",
            serialization_format='cdr'
        )
        self.bag_writer.create_topic(topic_info)
        self.bag_writer.create_topic(topic_info2)

        self.exec = MultiThreadedExecutor()
        print(type(self.exec))
        self.exec.add_node(self.node)
        # self.executor.add_node(self.control_velocity_action_server)
        self.thread = Thread(target=self.rclpy_executor, args=[self.node], daemon=False)

        if(not self.thread.is_alive()):
            self.thread.start()


    def retire(self):
        try:
            print(f"Attempting to shutdown node: {self.node.get_name()}")
            self.kill_node = True
            print("Shutdown Complete")
        except Exception as e:
            traceback.print_exc()
            print("Shutdown failed!")

    # Thread that spins the node
    def rclpy_executor(self, node: Node):
        try:
            # rclpy.spin(self, self.exec)
            # rclpy.spin(self.control_velocity_action_server)
            self.exec.spin()
            if self.kill_node:
                self.exec.shutdown()
                self.node.destroy_node()      
                return # Stop thread
        except Exception as e:
            traceback.print_exc()
            print("DEBUG")

    def control_velocity_action(self, goal_handle):
        # self.get_logger().info('Executing goal...')
        self.write_active = True
        goal_handle.succeed()
        result = ControlVelocity.Result()
        result.trajectory.append(self.populate_rigid_body_tick_message())
        for i in range(500):
            time.sleep(0.01)
            if(self.write_active): self.bag_writer.write('/rbt_bag', serialize_message(self.populate_rigid_body_tick_message()), self.get_clock().now().nanoseconds)
        self.write_active=False
        return result   

    def run_srv(self, request, response):
        try:
            self.node.get_logger().info(f"Incoming request: {request}")
            car = request.rigid_body_tick.bot_state
            carp = car.pose.position
            caro = car.pose.orientation
            carv = car.twist.linear
            carw = car.twist.angular
            ball = request.rigid_body_tick.ball_state.pose.position
            car_state = CarState(boost_amount=100,
                                physics=Physics(location = Vector3(x=carp.x, y=carp.y), velocity=Vector3(x=carv.x,y=carv.y), rotation=Rotator(0, 0, 0),
                                angular_velocity=Vector3(carw.x, carw.y, carw.z)))
            ball_state = BallState(Physics(location=Vector3(ball.x, ball.y, ball.z)))
            game_info_state = GameInfoState(world_gravity_z=0)
            game_state = GameState(ball=ball_state, cars={self.index: car_state}, game_info=game_info_state)
            self.set_game_state(game_state)
            response.success = True
            return response
        except Exception as e:
            response.success = False
            import traceback
            self.get_logger().warn(f"{traceback.print_exc()}")
            
        return response

    def listener_callback(self, msg: Twist):
        self.node.get_logger().debug('I heard: "%s"' % msg.linear.x)
        self.ros_controls.throttle = np.clip(msg.linear.x, -1, 1)
        self.ros_controls.steer = np.clip(msg.angular.z, -1 ,1)
        if(self.write_active): self.bag_writer.write('/cmd_vel_bag', serialize_message(msg), self.get_clock().now().nanoseconds)
        

    def initialize_agent(self):

        self.throttle = 0
        # if()
        self.ros_init()
        self.ros_controls = SimpleControllerState()
    
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc... and return controls to drive your car.
        """
        
        # self.publisher_.publish(msg)
        msg = self.populate_rigid_body_tick_message()
        if not self.kill_node: 
            self.publisher_.publish(msg)
        else: 
            self.thread.join()
        # else: self.publisher_.destroy()
        # self.get_logger().info(f"Throttle: {self.ros_controls.throttle}, vmag: {msg.bot_state.vmag}, angvel: {msg.bot_state.twist.angular.z}")
        return self.ros_controls

    def populate_rigid_body_tick_message(self):
        gtp = RigidBodyTick()
        gtp = self.get_rigid_body_tick()
        msg = RigidBodyTickMsg()
        q = gtp.players[self.index].state.rotation
        p = gtp.players[self.index].state.location
        v = gtp.players[self.index].state.velocity
        w = gtp.players[self.index].state.angular_velocity
        throttle = gtp.players[self.index].input.throttle
        steer = gtp.players[self.index].input.steer
        msg.bot_state.pose.orientation.x = q.x
        msg.bot_state.pose.orientation.y = q.y
        msg.bot_state.pose.orientation.z = q.z
        msg.bot_state.pose.orientation.w = q.w
        msg.bot_state.pose.position.x = p.x
        msg.bot_state.pose.position.y = p.y
        msg.bot_state.pose.position.z = p.z
        msg.bot_state.twist.linear.x = v.x
        msg.bot_state.twist.linear.y = v.y
        msg.bot_state.twist.linear.z = v.z
        msg.bot_state.twist.angular.x = w.x
        msg.bot_state.twist.angular.y = w.y
        msg.bot_state.twist.angular.z = w.z
        # self.get_logger().info(f"angvel: {w.z}")
        msg.bot_state.throttle = throttle
        msg.bot_state.steer = steer
        msg.bot_state.vmag = np.linalg.norm([v.x, v.y, v.z])
        
        return msg

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)
