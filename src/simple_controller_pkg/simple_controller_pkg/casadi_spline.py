from casadi import *
import pylab as plt

# ====================================
#    Data to be interpolated
#
# =====================================


N = 20

# Generate some data to be interpolated
vel = np.array([0, 1400, 1410, 2300])
amax = np.array([1600, 160, 0, 0])
veval = np.linspace(0,2300, N)
aeval = np.interp(veval, vel, amax)
xs = veval
ys = aeval

# ====================================
#    Spline Basis function 
#
# =====================================

# second-order B-spline Basis
x = SX.sym("x")
basis = Function('basis',[x],[if_else(x<=0,x**2/2+x+0.5,if_else(x<=1,-x**2+x+0.5,x**2/2-2*x+2))])


# ====================================
#    Spline function = sum of bases
#
# =====================================


# Spline coefficients
C = MX.sym("C",N)

# Create spline: sum of adjacent basis functions
X = MX.sym("X")

# Anchor points of adjacent bases
ffX = floor(X-0.5)
fX  = floor(X+0.5)
cX  = floor(X+1.5)

# This is similar to symbolic indexing, but with O(N) complexity.
# Can be replaced when implemented in CasADi
index = DM(range(N))
ffc = dot((index==ffX),C)
fc = dot((index==fX),C)
cc = dot((index==cX),C)

# Sum of adjacent bases
e = ffc*basis(X+0.5-ffX)+fc*basis(X+0.5-fX)+cc*basis(X+0.5-cX)

# Construct the spline function.
# Function of evaluation point x and coefficient list C
spline = Function('spline',[X,C],[e])

# ====================================
#    Offline fit to data
#
# =====================================


# Fitting the coefficients to the data
spline_eval_at_nodes = spline.map('eval','serial',N,[1],[])

err = spline_eval_at_nodes(xs,C).T - ys

solver = nlpsol('solver','ipopt',{"f":dot(err,err),"x": C})
Csol = solver(x0=ys)["x"]

# ====================================
#    Plot the interpolated result
#
# =====================================


Ns = 1000
ts = np.linspace(0,N,Ns)

spline_eval = spline.map('eval','serial',Ns,[1],[])

plt.plot(ts,spline_eval(ts,Csol).T)
plt.plot(xs,ys,'x')
plt.show()