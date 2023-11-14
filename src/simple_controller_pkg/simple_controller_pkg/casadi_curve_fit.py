import casadi
from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3, Twist
from rlbot_msgs.msg import PIDGains
import matplotlib.pyplot as plt
import numpy as np


from controller_util import AccelerationRelationship
ar = AccelerationRelationship

try:
    deg = 5
    numcoeff = deg + 1
    N=501

    vel = np.array([0, 1400, 1410, 2300])
    amax = np.array([1600, 160, 0, 0])
    veval = np.linspace(0,2300, N)
    aeval = np.interp(veval, vel, amax)

    #Symbolic expressions
    x = casadi.SX.sym('x')
    y = casadi.SX.sym('y')
    d = casadi.SX.sym('d', numcoeff)
    c = casadi.SX.sym('c', numcoeff)
    f = casadi.SX.sym('f', numcoeff)
    v = casadi.SX.sym('v')

    F = casadi.Function('F', [v, c, d], [c*(v**d)])
    dist = casadi.Function('dist', [x,y], [(x-y)**2])

    opti = casadi.Opti()
    c = opti.variable(numcoeff)
    v = opti.parameter(N)
    a = opti.parameter(N)
    d = opti.parameter(numcoeff)
    vconstrain = opti.parameter(N)
    aconstrain = opti.parameter(N)

    cinit = np.flip(ar.aCoeffs)
    opti.set_initial(c, np.polyfit(veval, aeval, deg=deg))
    # for i in range(len(amax)):
        # opti.subject_to(opti.bounded(-100, (F(vconstrain[i], c, d) - amax[i])**2, 100))
        # opti.subject_to(F(vel[i],c,d)==amax[i])
    for i in range(N):
        opti.set_value(v[i], veval[i])
        opti.set_value(a[i], aeval[i])
    for i in range(len(amax)):
    #     opti.minimize(casadi.sumsqr((F(vel[i],c, d) - amax[i])))
        opti.minimize(casadi.sum1(dist(F(vel[i],c,d),amax[i]*np.ones(6).T)))
    opti.set_value(vconstrain, veval)
    opti.set_value(aconstrain, aeval)
    opti.set_value(d, range(numcoeff))

    opti.solver('ipopt')
    sol = opti.solve()

    plt.figure(0)
    cval = sol.value(c)
    plt.plot(veval, aeval, 'b.')
    plt.plot(veval, AccelerationRelationship.getAmax(veval), 'g.')
    plt.plot(veval, np.polyval(cval,veval), 'r.')
    plt.show(block=False)
    # plt.show()
    plt.pause(0.001)
    print(ar.aCoeffs-cval)
    print("DEBUG")
except Exception as e:
    import traceback
    print(f"{traceback.print_exc()}")
    print("DEBUG")
    