from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3, Twist
import matplotlib.pyplot as plt
import numpy as np

###------------------------------------------------

class AccelerationRelationship:
    vel = np.array([0, 1400, 1410, 2300])
    amax = np.array([1600, 160, 0, 0])
    veval = np.linspace(0,2300, 501)
    aeval = np.interp(veval, vel, amax)
    
    # Get throttle accelerations max
    getAmax = UnivariateSpline(veval, aeval, k=5)

class SteeringRelationship:
    vel = np.array([0, 500, 1000, 1500, 1750, 2300])
    k = np.array([0.0069, 0.00398, 0.00235, 0.001375, 0.0011, 0.00088])
    w = vel*k
    
    # Spline to get Kmin at diff velocities
    getKMin = UnivariateSpline(vel, k)
    # Get Angular velocity max for driving
    getWMax = UnivariateSpline(vel, w, k=4)

###_----------------------------------------------
def get_best_steering_and_throttle(vmag, des_a, des_w):
    # des_a = twist.linear.x
    # des_w = twist.angular.z
    a_max = AccelerationRelationship.getAmax(vmag)
    a_min = -3500
    w_max = SteeringRelationship.getWMax(vmag)
    coast = 0.0125
    if des_a > 0.0:
        u_t = np.clip(des_a/a_max, coast, 1)
    elif (des_a >-500) and (des_a < 0):
        u_t = np.interp(des_a, [-500,0], [-coast, coast])
    elif des_a <= -500:
        u_t = np.clip(-1*des_a/a_min, -1, coast)
    u_s = np.clip(des_w/w_max, -1, 1)
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
    plt.plot(vs,ks,'b.')
    plt.legend()
    plt.figure(1)
    plt.plot(SteeringRelationship.vel, SteeringRelationship.w, 'r*', label = 'w=g(v)')
    plt.plot(vs, ws, 'b.')
    plt.legend()
    plt.figure(2)
    plt.plot(AccelerationRelationship.vel, AccelerationRelationship.amax)
    plt.plot(vs, amax, 'b.', label = 'amax = h(v)')
    plt.legend()
    plt.show()