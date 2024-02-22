import casadi
from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3, Twist
# from rlbot_msgs.msg import PIDGains
import matplotlib.pyplot as plt
import numpy as np


# from controller_util import AccelerationRelationship
# ar = AccelerationRelationship

try:

    #Symbolic expressions
    xc = casadi.SX.sym('xc')
    x1 = casadi.SX.sym('x1')
    yc = casadi.SX.sym('yc')
    y1 = casadi.SX.sym('y1')
    F = casadi.Function('F', [xc], [casadi.sqrt(10000-xc**2)])
    dist = casadi.Function('dist', [xc, x1, yc, y1], [casadi.sqrt((xc-x1)**2 + (yc-y1)**2)])

    opti = casadi.Opti()
    xpos = opti.parameter()
    ypos = opti.parameter()
    xmin = opti.variable(1)
    ymin = F(xmin)
    opti.set_initial(xmin, 0.99)
    opti.subject_to(xmin>=-99)
    opti.subject_to(xmin<=99)
    opti.minimize(dist(xmin, xpos, ymin, ypos))
    opts = {}#{'ipopt.print_level': 2, 'print_time': 0, 'ipopt.sb': 'yes'}
    opti.solver('ipopt', opts)
    xsol=[]
    ysol=[]
    xp=[]
    yp=[]

    # for i in range(1,10):
    x=196
    y=-507
    opti.set_value(xpos,x)
    opti.set_value(ypos, y)
    xp.append(x)
    yp.append(y)
    sol = opti.solve()
    xsol.append(opti.value(xmin))
    ysol.append(opti.value(ymin))
    plt.figure(0)
    xcurve = np.linspace(-100, 100, 101)
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
    