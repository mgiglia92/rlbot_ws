from scipy.interpolate import UnivariateSpline, BSpline, splrep
from geometry_msgs.msg import Vector3, Twist
from rlbot_msgs.msg import PIDGains
import matplotlib.pyplot as plt
import numpy as np

###------------------------------------------------
class PIDStruct:
    kp=0.0
    ki=0.0
    kd=0.0
    dt=1
    def set_from_ros_msg(self, msg:PIDGains):
        self.kp=msg.kp
        self.ki=msg.ki
        self.kd=msg.kd
        self.dt=msg.dt
###------------------------------------------------

class AccelerationRelationship:
    vel = np.array([0, 1400, 1410, 2300])
    amax = np.array([1600, 160, 0, 0])
    veval = np.linspace(0,2300, 501)
    aeval = np.interp(veval, vel, amax)
    
    # Get throttle accelerations max
    getAmax = UnivariateSpline(veval, aeval, k=5)
    aCoeffs = np.polyfit(veval, aeval, deg=5)
    (t,c,k) = splrep(veval, aeval, k=3)
    aspline = BSpline(t,c,k)
    print("DEBUG")

class SteeringRelationship:
    vel = np.array([0, 500, 1000, 1500, 1750, 2300])
    k = np.array([0.0069, 0.00398, 0.00235, 0.001375, 0.0011, 0.00088])
    w = vel*k
    
    # Spline to get Kmin at diff velocities
    getKMin = UnivariateSpline(vel, k)
    # Get Angular velocity max for driving
    getWMax = UnivariateSpline(vel, w, k=4)

###_----------------------------------------------
# Get open loop estimations for controler input based on a desired acceleration and angularvelocity
def get_best_steering_and_throttle(vmag, des_a, des_w):
    # des_a = twist.linear.x
    # des_w = twist.angular.z
    #TODO: Set this up for driving backwards throttles
    a_max = AccelerationRelationship.getAmax(vmag)
    a_min = -3500
    w_max = SteeringRelationship.getWMax(vmag)
    coast = 0.01182
    u_t = 0.0
    if des_a > 0.0:
        u_t = np.clip(des_a/a_max, coast, 1.0)
    # elif (des_a >-500) and (des_a < 0):
    #     u_t = np.interp(des_a, [-500,0], [-coast, coast])
    elif des_a <= -500.0:
        u_t = np.clip(-1*des_a/a_min, -1.0, coast)
    u_s = np.clip(des_w/w_max, -1.0, 1.0)
    return u_t, u_s

def to_numpy(v):
    try:
        if type(v) == Vector3:
            return np.array([v.x, v.y, v.z])
        if type(v) == QMsg:
            return np.array([v.w, v.x, v.y, v.z])
    except:
        print("Type error")
        return np.array([0,0,0,0])

if __name__ == "__main__":
    plt.figure(0)
    plt.plot(SteeringRelationship.vel, SteeringRelationship.k, 'r.', label='k=f(v)')
    #Calculate many points using spline
    vs = np.linspace(0, 2300, 101)
    ks = SteeringRelationship.getKMin(vs)
    ws = SteeringRelationship.getWMax(vs)
    amax = AccelerationRelationship.getAmax(vs)
    amaxCoeffs = np.polyval(AccelerationRelationship.aCoeffs, AccelerationRelationship.veval)
    plt.plot(vs,ks,'b.')
    plt.legend()
    plt.figure(1)
    plt.plot(SteeringRelationship.vel, SteeringRelationship.w, 'r*', label = 'w=g(v)')
    plt.plot(vs, ws, 'b.')
    plt.legend()
    plt.figure(2)
    plt.plot(AccelerationRelationship.vel, AccelerationRelationship.amax, 'b')
    plt.plot(AccelerationRelationship.veval, amaxCoeffs, 'r')
    plt.plot(vs, amax, 'b.', label = 'amax = h(v)')
    plt.legend()
    plt.show()