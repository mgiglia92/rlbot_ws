from casadi import *
import matplotlib.pyplot as plt
import numpy as np

class ActiveTraits:
    #TODO: Make default values work for any sizes
    def __init__(self, active=[0,0,0,0,0,0,0], values=[0,0,0,0,0,0,0]):
        assert len(active) == len(values), f"Mismatched lengths: {len(active)} {len(values)}"
        self.active = active
        self.values = values
        self.length = lambda: len(self.active)

class TrajectoryOpti(Opti):
    # # Just a basic tester to test rk4 integrarion and plot
    # def testrk4(self):
    #     N = self.N
    #     T = self.T
    #     X = np.zeros((7,N+1))
    #     X[:,0] = np.array([0,0,0,0,0,0,0])    # x, y, xdot, ydot, theta, thetadot, vmag
    #                                     # 0,   1, 2,    3,    4,       5,      6
    #     U = np.ones((2,N))
    #     f = self.fsim
    #     dt = T/N
    #     for k in range(0,N):
    #         k1 = f(X[:,k], U[:,k])
    #         k2 = f(X[:,k]+(k1*dt/2), U[:,k])
    #         k3 = f(X[:,k]+(k2*dt/2), U[:,k])
    #         k4 = f(X[:,k]+(k3*dt), U[:,k])
    #         X[:,k+1] = np.reshape(X[:,k] + (dt/6)*(k1 + (2*k2) + (2*k3) + k4), (7))
        
    #     plt.figure(1)
    #     plt.plot(X[0],X[1])
    #     plt.plot(X[2], X[3])
    #     plt.quiver(X[0], X[1], X[2], X[3], angles='xy')
    #     plt.show(block=False)
    #     plt.pause(0.01)
    #     return 0

    # Simulate nunmerically rk4
    def sim_rk4(self,x,u,T,N):
        f = self.fsim
        dt = T/N
        k1 = f(x,u)
        k2 = f(x+(k1*dt/2), u)
        k3 = f(x+(k2*dt/2), u)
        k4 = f(x+(k3*dt), u)
        x_next = x + (dt/6)*(k1 + (2*k2) + (2*k3) + k4)
        return x_next
    
    # Set rk4 constraints for opti
    def set_rk4(self,X,U,T,N):
        f = self.f
        dt = T/N
        for k in range(0,N):
            k1 = f(X[:,k], U[:,k])
            k2 = f(X[:,k]+(k1*dt/2), U[:,k])
            k3 = f(X[:,k]+(k2*dt/2), U[:,k])
            k4 = f(X[:,k]+(k3*dt), U[:,k])
            x_next = X[:,k] + (dt/6)*(k1 + (2*k2) + (2*k3) + k4)
            self.subject_to(X[:,k+1] == x_next)

    def f(self,x,u):
        # return vertcat(0,0,0,0,0)
        return vertcat( x[6]*cos(x[4]+x[5])          ,     #xdot
                        x[6]*sin(x[4]+x[5]),     #ydot
                        x[6]*cos(x[5]),     #xddot
                        x[6]*sin(x[4]),     #yddot
                        u[1]*x[6]*0.01,               #thetadot
                        0,                  #theatddot
                        u[0])               #vmagdot
    # Numerically evaluate xdot
    def fsim(self,x,u):
        # return vertcat(0,0,0,0,0)
        return np.array([x[2]          ,     #xdot
                        x[3]          ,     #ydot
                        x[6]*cos(x[4]),     #xddot
                        x[6]*sin(x[4]),     #yddot
                        x[5],               #thetadot
                        u[1]*x[6]*0.01,                  #theatddot
                        u[0]])


    def set_vars(self, xsize, usize, N):
        self.N = N
        self.xsize = xsize
        self.usize = usize
        self.X = self.variable(xsize, self.N+1) # [x, y, xdot, ydot, theta, thetadot, vmag]
        x = self.X[0,:]
        y = self.X[1,:]
        vx = self.X[2,:]
        vy = self.X[3,:]
        theta = self.X[4,:]
        thetadot = self.X[5,:]
        vmag = self.X[6,:]
        self.theta = self.X[4,:]
        self.U = self.variable(usize, self.N)
        self.T = self.variable()
        # self.R = MX(2,2,self.N)
    
    def set_constraints(self):
        self.set_rk4(self.X,self.U,self.T,self.N)
        self.subject_to(self.bounded(-1, self.U[1,:], 1)) # Bound U
        self.subject_to(self.bounded(-1400, self.U[0,:],1400)) # BOund U
        self.subject_to(self.bounded(0.1, self.T, 100)) # Bound Time
        # self.subject_to(self.X[])
        # self.subject_to(self.X[2,-1]*sin(-1*self.X[4,-1]) + self.X[3,-1]*cos(-1*self.X[4,-1]) == 0)
        # self.subject_to(self.U[1,:] == self.X[5,:d-1])
        
    def set_objectives(self):
        self.minimize(self.T)

        # self.minimize((self.X[2,-1]*sin(-1*self.X[4,-1]) + self.X[3,-1]*cos(-1*self.X[4,-1]))**2)


    
    def set_initialization_finalization_constraints(self, IC:ActiveTraits, FC:ActiveTraits):
        assert IC.length() == self.xsize, f"ICs not same length as X"
        assert FC.length() == self.xsize, f"FCs not same length as X"
        for i in range(self.xsize):
            if IC.active[i]: self.subject_to(self.X[i,0] == vertcat(IC.values[i]))
            if FC.active[i]: self.subject_to(self.X[i,-1] == vertcat(FC.values[i]))

    def initial_guess(self):
        ti = 1
        self.set_initial(self.T, ti)
        x = np.array([1, 1, 1, 1, 1, 1, 1])
        u = np.array([1000,0])
        for i in range(1, self.N):
            self.set_initial(self.X[:,i], x)
            self.set_initial(self.U[:,i], u)
            x = self.sim_rk4(x,u, ti, self.N)
            print(f"i:{i} x: {x}")
        pass

    def reset_optimizer(self, IC, FC):
        self.set_vars(7,2,21)
        self.subject_to() # Reset constraints
        self.set_constraints()
        self.set_initialization_finalization_constraints(IC, FC)
        self.set_objectives()
        self.initial_guess()
        options = dict()
        options["expand"]=True
        options["qpsol"] = 'qrqp'
        options["qpsol_options"] = {"print_iter": False, "print_header": False}
        options["print_iteration"] = False
        options["print_header"] = False
        options["print_status"] = False
        self.solver('ipopt')#,options)
        # self.solver("ipopt")
        self.sol = self.solve()
        return self.sol

opti = TrajectoryOpti()
# opti.testrk4()
IC = ActiveTraits([1,1,0,0,1,1,1],[0, 0, 100, 0, 0, 0,0])
FC = ActiveTraits([1,1,0,0,1 ,0,1], [1000, 1000, 0, 0, 0, 0, 0])
sol = opti.reset_optimizer(IC, FC)

# IC = ActiveTraits([1,1,1,1,1,1,1],[0, 0, 0,0,1.5,0,0])
# FC = ActiveTraits([1,1,0,0,1,0,0], [2000, 1000, 0, 0, 0, 0, 0])
# sol = opti.reset_optimizer(IC, FC)

sol.value(opti.X)
tf = sol.value(opti.T)
t = np.linspace(0,tf, opti.N+1)
x = sol.value(opti.X[0,:])
y = sol.value(opti.X[1,:])
xdot = sol.value(opti.X[2,:])
ydot = sol.value(opti.X[3,:])
theta = sol.value(opti.X[4,:])
thetadot = sol.value(opti.X[5,:])
v = np.sqrt(xdot**2 + ydot**2)
throttle = sol.value(opti.U[0,:])
steer = sol.value(opti.U[1,:])

plt.figure(1)

plt.plot(x,y, 'r.', label="pos")
for i in range(len(x)): 
    # plt.quiver(x[i],y[i],xdot[i],ydot[i], angles='xy', color='g')
    plt.quiver(x[i],y[i], 2*np.cos(theta[i]), 2*np.sin(theta[i]), angles='xy', scale_units='xy', color='r')
    if(i<len(throttle)):
        plt.quiver(x[i],y[i],throttle[i]*np.cos(theta[i]), throttle[i]*np.sin(theta[i]), angles='xy', color='black')
    
plt.legend()
plt.figure(2)
plt.plot(t[:-1], throttle/1400, 'r-', label='throttle')
plt.plot(t[:-1], steer, 'b', label='steer')
plt.legend()


from scipy.interpolate import CubicSpline

spline = CubicSpline(t,np.vstack((x,y)).T)
teval = np.linspace(0,t[-1],1)
plt.figure(3)
plt.plot(spline(teval)[:,0], spline(teval)[:,1], 'b*')
plt.plot(x,y,'r.')

fig = plt.figure(4)
ax = fig.subplots(3,1)
ax[0].plot(t[:-1], throttle, 'r.')
ax[1].plot(t[:-1],steer, 'b.')
ax[0].set_ylim(-1400,1400)
ax[1].set_ylim(-1,1)
ax[2].plot(t[:-1], v[:-1], 'k')
ax[2].set_ylim(-1400, 1400)
plt.show(block=False)
plt.pause(0.01)
print("DEBUG")