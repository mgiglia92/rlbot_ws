import casadi
from casadi import Function, SX, MX
from casadi import poly_coeff, poly_roots, polyval, vertcat, sqrt, sumsqr
import numpy as np
from rlbot_msgs.msg import Polynomial3

class MinPoly:
    def __init__(self):    
        #Symbolic expression
        self.degree = 3
        t = casadi.SX.sym('t')
        p = casadi.SX.sym('p')
        c = casadi.SX.sym('pow', self.degree+1)
        tau = casadi.SX.sym('tau', self.degree+1)
        F = Function('F', [t, p], [casadi.power(t, p)], ['time', 'power'], ['tau']) # Generate a vector for tau of this
        J = F.jacobian()
        G = Function('G', [tau, c], [casadi.dot(tau,c)], ['tau', 'coeff'], ['position'])
        x = SX.sym('x') # X of body
        y = SX.sym('y')
        xt = SX.sym('xt') # X on traj
        yt = SX.sym('yt')
        self.dist = casadi.Function('dist', [x,xt,y,yt], [(xt-x)**2 + (yt-y)**2])

        self.opti = casadi.Opti()
        self.t = self.opti.variable(1)
        # self.p = self.opti.parameter(self.degree+1)
        # self.taumin = self.opti.variable(self.degree+1)
        self.x = self.opti.parameter() # Position of body
        self.y = self.opti.parameter()
        self.xcoeff = self.opti.parameter(self.degree + 1) # Coeff of trajectory
        self.ycoeff = self.opti.parameter(self.degree + 1)
        self.xmin = self.opti.variable(1)
        self.ymin = self.opti.variable(1)

        self.opti.subject_to(self.xmin == polyval(self.xcoeff, self.t)) # Posiion along traejctory that is minimum
        self.opti.subject_to(self.ymin == polyval(self.ycoeff, self.t))

        self.opti.subject_to(self.opti.bounded(0, self.t, 10))

        self.opti.minimize(self.dist(self.xmin, self.x, self.ymin, self.y))
        # self.opti.minimize(self.xmin + self.ymin)

        # opts = {'ipopt.print_level': 2, 'print_time': 0, 'ipopt.sb': 'yes'}
        self.opti.solver('ipopt')#, opts)
        self.sol = None
    
    def update_parameters(self, x, y, coeffs: Polynomial3):
        self.opti.set_value(self.x, x)
        self.opti.set_value(self.y, y)
        xcoeff =  coeffs.px
        ycoeff = coeffs.py
        self.opti.set_value(self.xcoeff, xcoeff)
        self.opti.set_value(self.ycoeff, ycoeff)
        self.opti.set_initial(self.t, coeffs.tf)
        self.opti.set_initial(self.xmin, x)
        self.opti.set_initial(self.ymin, y)
        # self.opti.set_initial(self.taumin, [1,1,1,1])
        # Initialize exponent vector
        # for i in range(self.degree+1):
            # self.opti.set_value(self.p, self.degree-i) # Set the power in the vector (0th degree first) may need to flip

    
    def solve(self):
        self.sol = self.opti.solve()
        return self.opti.value(self.xmin), self.opti.value(self.ymin), self.opti.value(self.t)
    
if __name__ == "__main__":
    m = MinPoly()
    p = Polynomial3()
    p.px = [-4.3259823e-01,  8.0080902e+02, -3.8988417e-01,  3.6566865e-02]
    p.py = [-4.3259823e-01,  8.0080902e+02, -3.8988417e-01,  3.6566865e-02]
    p.tf = 2.0
    m.update_parameters(-250, 1500, p)
    try:
        xmin, ymin, tmin = m.solve()
    except:
        pass

    print("Debug")