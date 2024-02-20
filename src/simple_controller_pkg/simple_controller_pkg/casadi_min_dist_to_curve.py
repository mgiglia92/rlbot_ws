import casadi
from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3, Twist
# from rlbot_msgs.msg import PIDGains
import matplotlib.pyplot as plt
import numpy as np


# from controller_util import AccelerationRelationship
# ar = AccelerationRelationship

try:
    # deg = 5
    # numcoeff = deg + 1
    # N=501

    # vel = np.array([0, 1400, 1410, 2300])
    # amax = np.array([1600, 160, 0, 0])
    # veval = np.linspace(0,2300, N)
    # aeval = np.interp(veval, vel, amax)

    # Robot Position
    x=1.1
    y=1

    #Symbolic expressions
    xc = casadi.SX.sym('xc')
    x1 = casadi.SX.sym('x1')
    yc = casadi.SX.sym('yc')
    y1 = casadi.SX.sym('y1')
    # d = casadi.SX.sym('d', numcoeff)
    # c = casadi.SX.sym('c', numcoeff)
    # f = casadi.SX.sym('f', numcoeff)
    # v = casadi.SX.sym('v')

    # F = casadi.Function('F', [v, c, d], [c*(v**d)])
    F = casadi.Function('F', [xc], [casadi.sqrt(1-xc**2)])
    dist = casadi.Function('dist', [xc, x1, yc, y1], [casadi.sqrt((xc-x1)**2 + (yc-y1)**2)])

    opti = casadi.Opti()
    xmin = opti.variable(1)
    ymin = F(xmin)
    opti.set_initial(xmin, 0.99)
    opti.minimize(dist(xmin, x, ymin, y))
    opti.solver('ipopt')
    xsol=[]
    ysol=[]
    xp=[]
    yp=[]

    for i in range(1,10):
        x=(i/10)
        y=2*((i/10)**2)
        xp.append(x)
        yp.append(y)
        opti.minimize(dist(xmin, x, ymin, y))
        sol = opti.solve()
        xsol.append(opti.value(xmin))
        ysol.append(opti.value(ymin))
    plt.figure(0)
    xcurve = np.linspace(-1, 1, 101)
    ycurve = F(xcurve)
    plt.plot(xcurve, ycurve, 'b.')
    plt.plot(xp, yp, 'g.')
    plt.plot(xsol, ysol, 'r.')
    plt.axis('equal')
    plt.show(block=False)
    # plt.show()
    plt.pause(0.001)
    print("DEBUG")
except Exception as e:
    import traceback
    print(f"{traceback.print_exc()}")
    print("DEBUG")
    