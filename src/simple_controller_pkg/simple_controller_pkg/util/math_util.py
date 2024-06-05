from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Quaternion as QMsg
from rlbot_msgs.msg import RigidBodyTick as RigidBodyTickMsg
from rlbot_msgs.msg import ControllerReference
from rlbot_msgs.msg import TrajectoryReference
from rlbot_msgs.srv import SetGains, TwistSetpoint
from transforms3d.quaternions import rotate_vector
from transforms3d.euler import quat2euler
from transforms3d.utils import normalized_vector
from transforms3d.axangles import axangle2mat
import numpy as np
from pyquaternion import Quaternion
from simple_controller_pkg.controller_util import get_best_steering_and_throttle, PIDStruct
import argparse

def norm(self, vec) -> np.array:
    return np.sqrt(vec.x**2 + vec.y**2 + vec.z**2) * np.sign()

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

def ControllerReference_from_TrajectoryReference(msg: TrajectoryReference):
        cr = ControllerReference()
        xp = msg.rbt.bot_state.pose.position.x
        yp = msg.rbt.bot_state.pose.position.y
        o = msg.rbt.bot_state.pose.orientation
        vmag = msg.rbt.bot_state.vmag
        quat = np.array([o.w, o.x, o.y, o.z])
        forward = rotate_vector(np.array([1,0,0]), quat, False)
        right = rotate_vector(np.array([0,1,0]), quat, False)
        up = rotate_vector(np.array([0,0,1]), quat, False)

        # Sanitize Heading input
        # q = msg.rbt.bot_state.pose.orientation
        # quat = np.array([q.w,q.x,q.y,q.z])
        # x = np.array([1,0,0])
        # hvec = rotate_vector(x,quat)
        # hvec[2]=0.0
        # yaw = angle_between(hvec,x)

        # roll,pitch,yaw = quat2euler([o.w, o.x, o.y, o.z], 'sxyz')
        v = msg.rbt.bot_state.twist.linear
        vel = np.array([v.x, v.y, v.z])
        # Heading error
        vx = msg.vxr
        vy = msg.vyr
        yaw_desired = msg.thetar

        # Cross Track Error
        vec_to_path_world = np.array([msg.xr, msg.yr, 0]) - np.array([xp, yp, 0])
        R = axangle2mat([0,0,1], np.pi/2)
        ctvec_body = np.dot(R,vec_to_path_world)
        cte = np.linalg.norm(vec_to_path_world)
        trajectory_velocity = np.array([msg.vxr, msg.vyr, 0])

        if(angle_between(unit_vector(vel), unit_vector(vec_to_path_world)) > 0):
            cte = -1*cte

        # he = angle_between([vx, vy, 0], [np.cos(yaw), np.sin(yaw), 0])
        # he = yaw_desired-yaw
        he = angle_between(unit_vector(trajectory_velocity), unit_vector(vel))
        # if(he > np.pi):
        #     he = he - np.pi

        twist = Twist()
        twist.angular.x = he
        twist.angular.y = cte
        twist.angular.z = 0.0

        unit_vel = unit_vector(vel)
        cr.headingx = unit_vel[0]
        cr.headingy = unit_vel[1]
        cr.rbt = msg.rbt
        cr.he = he
        cr.cte = cte
        cr.correction = 2*he + np.arctan2(3*cte, (0.001+vmag))
        cr.v_desired = float(1800)
        cr.w_desired = float(np.clip(2*he + np.arctan2(3*cte, (0.001+vmag)), -5.5, 5.5))

        return (cr, twist)