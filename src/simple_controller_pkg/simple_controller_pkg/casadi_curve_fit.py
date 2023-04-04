import casadi
from scipy.interpolate import UnivariateSpline
from geometry_msgs.msg import Vector3, Twist
from rlbot_msgs.msg import PIDGains
import matplotlib.pyplot as plt
import numpy as np

deg = 4
N=501

vel = np.array([0, 1400, 1410, 2300])
amax = np.array([1600, 160, 0, 0])
veval = np.linspace(0,2300, N)
aeval = np.interp(veval, vel, amax)

#Symbolic expressions
x = casadi.SX.sym('x')
y = casadi.SX.sym('y')
coeff = casadi.SX.sym('coeff', deg+1)
v = casadi.SX.sym('v', 1, 1)
fs=[]
for i in range(deg):
    fs.append((v**i)*coeff[i])
f = v*coeff[4] + (v**1)*coeff[3] + (v**2)*coeff[2] + (v**3)*coeff[1] + (v**4)*coeff[0]
F = casadi.Function('F', [v, coeff], [f])
dist = casadi.Function('dist', [x,y], [(y-x)**2])
opti = casadi.Opti()
coeff = opti.variable(deg+1)
v = opti.parameter(N)
a = opti.parameter(N)

cinit = [-4.29856970e-11,  4.46795682e-07, -8.19030390e-04, -6.03873482e-01,
        1.55619827e+03]
opti.set_initial(coeff, cinit)

# for i in range(deg+1):
    # opti.set_initial(coeff[i], cinit[i])
for i in range(N):
    opti.set_value(v[i], veval[i])
    opti.set_value(a[i], aeval[i])
    opti.minimize(casadi.norm_1(a[i] - F(v[i],coeff)))

opti.solver('ipopt')
sol = opti.solve()

plt.figure(0)
c = sol.value(coeff)
for i in range(N):
    plt.plot(veval[i], F(veval[i],c))
plt.legend()
plt.show(block=False)
plt.pause(0.001)
print("DEBUG")