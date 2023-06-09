from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.game_state_util import GameState, Physics, Rotator, Vector3, CarState, BallState, GameInfoState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.rigid_body_struct import RigidBodyTick

from threading import Thread
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
# from rclpy.qos import QoSProfile
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
import numpy as np
import time

if(not rclpy.ok()):
    rclpy.init()

class MyBot(BaseAgent):

    def __init__(self, name, team, index):

        super().__init__(name, team, index)
        # super(BaseAgent).__init__(name, team, index)
        # super(Node).__init__("rlbot_node")
        self.throttle = 0
        # if()
        self.ros_init()
        self.ros_controls = SimpleControllerState()
        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(self.node)
        self.initialize_agent()

    def ros_init(self):
        self.node = Node("rlbot_node")
        self.thread = Thread(target=self.rclpy_executor, args=[self.node], daemon=True)
        # self.timer = self.node.create_timer(1.0, self.idle)
        self.publisher_ = self.node.create_publisher(RigidBodyTickMsg, f"/player0/RigidBodyTick", 10)
        self.subscriber_ = self.node.create_subscription(Twist, "/cmd_vel", self.listener_callback, 10)
        self.subscriber_
        # self.publisher_ = self.node.create_publisher(GameTickPacketMsg, '/GameTickPacket',10)
        if(not self.thread.is_alive()):
            self.thread.start()

    def retire(self):
        try:
            print(f"Attempting to shutdown node: {self.node.get_name()}")
            import time
            time.sleep(0.5)
            print("waiting")
            self.node.destroy_node()
            self.thread.join()
            print("Shutdown Complete")
        except:
            print("Shutdown failed!")

    # Thread that spins the node
    def rclpy_executor(self, node):
        try:
            rclpy.spin(node)
        except Exception as e:
            print(e)
            rclpy.shutdown()

    def listener_callback(self, msg: Twist):
        self.node.get_logger().debug('I heard: "%s"' % msg.linear.x)
        self.ros_controls.throttle = np.clip(msg.linear.x, -1, 1)
        self.ros_controls.steer = np.clip(msg.angular.z, -1 ,1)

    def initialize_agent(self):
        try:
            time.sleep(1)
            car_state = CarState(boost_amount=100,
                                physics=Physics(location = Vector3(x=0, y=0), velocity=Vector3(x=0,y=0), rotation=Rotator(0, 0, 0),
                                angular_velocity=Vector3(0, 0, 0)))
            ball_state = BallState(Physics(location=Vector3(1000, 1000, None)))
            game_info_state = GameInfoState()
            game_state = GameState(ball=ball_state, cars={self.index: car_state}, game_info=game_info_state)
            time.sleep(1)
            self.set_game_state(game_state)
            time.sleep(1)
            self.set_game_state(game_state)
        except Exception as e:
            print(e)
    
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """
        
        # self.publisher_.publish(msg)
        msg = self.populate_rigid_body_tick_message()
        self.publisher_.publish(msg)
        self.node.get_logger().info(f"Throttle: {self.ros_controls.throttle}, vmag: {msg.bot_state.vmag}, angvel: {msg.bot_state.twist.angular.z}")
        return self.ros_controls

    def populate_rigid_body_tick_message(self, gametick):
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
        self.node.get_logger().info(f"angvel: {w.z}")
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
