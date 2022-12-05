import os
import numpy as np
import matplotlib.pyplot as plt  # MATLAB-like plotting functions
import control as ct

def feed_foward(x0, xf, A, B, K, N):
    x0 = np.reshape(x0, (6, 1))
    xf = np.reshape(xf, (6, 1))
    # print("A: ", A.shape)
    # print("B: ", B.shape)
    x = x0
    for i in range(N):
        x_err = x - xf
        # print(x_err)
        u = -K.dot(x_err)
        # print("X ERR: ", x_err.shape)
        # print("OPT U: ", u)
        x += (A.dot(x) + B.dot(u))*.001
        # print("x new: ", x.shape)
    return x_err

# System parameters
m = 4       # mass of aircraft
J = 0.0475  # inertia around pitch axis
r = 0.25    # distance to center of force
g = 9.8     # gravitational constant
c = 0.05    # damping factor (estimated)

# State space dynamics
xe = [0, 0, 0, 0, 0, 0]  # equilibrium point of interest
ue = [0, m * g]  # (note these are lists, not matrices)

# TODO: The following objects need converting from np.matrix to np.array
# This will involve re-working the subsequent equations as the shapes
# See below.

A = np.array(
    [[0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 1],
     [0, 0, (-ue[0]*np.sin(xe[2]) - ue[1]*np.cos(xe[2]))/m, -c/m, 0, 0],
     [0, 0, (ue[0]*np.cos(xe[2]) - ue[1]*np.sin(xe[2]))/m, 0, -c/m, 0],
     [0, 0, 0, 0, 0, 0]]
)

# Input matrix
B = np.array(
    [[0, 0], [0, 0], [0, 0],
     [np.cos(xe[2])/m, -np.sin(xe[2])/m],
     [np.sin(xe[2])/m, np.cos(xe[2])/m],
     [r/J, 0]]
)

# Output matrix 
C = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])
D = np.array([[0, 0], [0, 0]])

#
# Construct inputs and outputs corresponding to steps in xy position
#
# The vectors xd and yd correspond to the states that are the desired
# equilibrium states for the system.  The matrices Cx and Cy are the 
# corresponding outputs.
#
# The way these vectors are used is to compute the closed loop system
# dynamics as
#
#   xdot = Ax + B u  =>  xdot = (A-BK)x + K xd
#      u = -K(x - xd)       y = Cx
#
# The closed loop dynamics can be simulated using the "step" command, 
# with K*xd as the input vector (assumes that the "input" is unit size,
# so that xd corresponds to the desired steady state.
#

xf = np.array([[1.], [0.], [0.], [0.], [0.], [0.]])
x0 = np.array([[0.], [1.], [0.], [0.], [0.], [0.]])

# Start with a diagonal weighting
Qx1 = np.diag([1, 1, 1, 1, 1, 1])
Qu1a = np.diag([1, 1])*.001
K1a, X, E = ct.lqr(A, B, Qx1, Qu1a)

print(K1a)
print(x0 - xf)
x_err = feed_foward(x0, xf, A, B, K1a, 10000)
print(x_err)