import numpy as np
from numpy import array, dot, sqrt, inf
from math import erf
from scipy.special import erfinv
from datetime import datetime

g = 9.8

T = 50 # timesteps
T_length = .1 # Time represented by each timestep

###################### QUADROTOR ############################
A = np.identity(13)
# Specs taken from DJI Mini 2
m = .242
# Model as a disk of radius .0107 m
Ixx = 6.8*10**-6
Iyy = 6.8*10**-6
Izz = 1.6*10**-5
A[5, 6] = g*T_length
A[9, 10] = -g*T_length
A[0, 1] = 1*T_length
A[2, 3] = 1*T_length
A[4, 5] = 1*T_length
A[6, 7] = 1*T_length
A[8, 9] = 1*T_length
A[10, 11] = 1*T_length
A[1, 12] = -.1

B = np.zeros((13, 4))
B[1, 0] = 1/m*T_length
B[3, 1] = 1/Ixx*T_length
B[7, 2] = 1/Iyy*T_length
B[11, 3] = 1/Izz*T_length

z_lims = (0, 10)
dz_lims = (-10, 10)
psi_lims = (-.2, .2)
dpsi_lims = (-1, 1)

x_lims = (0, 6)
dx_lims = (-2, 2)
phi_lims = (-.2, .2)
dphi_lims = (-1, 1)

y_lims = (0, 6)
dy_lims = (-2, 2)
th_lims = (-.2, .2)
dth_lims = (-1, 1)

g_lims = (g - .001, g + .001)

x_lims = [z_lims, dz_lims, psi_lims, dpsi_lims, x_lims, dx_lims, phi_lims, dphi_lims, y_lims, dy_lims, th_lims, dth_lims, g_lims]
u_lims = [(-3, 3), (-10, 10), (-10, 10), (-10, 10)]

####################### FREE DYNAMICS ###########################
# A = np.identity(13)
# # Specs taken from DJI Mini 2
# m = .242
# # Model as a disk of radius .0107 m
# Ixx = 6.8*10**-6
# Iyy = 6.8*10**-6
# Izz = 1.6*10**-5
# A[5, 6] = g*T_length
# A[9, 10] = -g*T_length
# A[0, 1] = 1*T_length
# A[2, 3] = 1*T_length
# A[4, 5] = 1*T_length
# A[6, 7] = 1*T_length
# A[8, 9] = 1*T_length
# A[10, 11] = 1*T_length
# A[1, 12] = -.1

# B = np.identity(13)

# z_lims = (0, 5)
# dz_lims = (-10, 10)
# psi_lims = (-.2, .2)
# dpsi_lims = (-1, 1)

# x_lims = (0, 6)
# dx_lims = (-2, 2)
# phi_lims = (-.2, .2)
# dphi_lims = (-1, 1)

# y_lims = (0, 6)
# dy_lims = (-2, 2)
# th_lims = (-.2, .2)
# dth_lims = (-1, 1)

# g_lims = (g - .001, g + .001)

# x_lims = [z_lims, dz_lims, psi_lims, dpsi_lims, x_lims, dx_lims, phi_lims, dphi_lims, y_lims, dy_lims, th_lims, dth_lims, g_lims]
# u_lims = [(-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10), (-10, 10)]

###########################################

print(A)
print(B)

# state x = [z, dz, psi, dpsi, x, dx, phi, dphi, y, dy, th, dth]
x0_bar = array([1, 0, 0, 0, 2.5, 0, 0, 0, 2.5, 0, 0, 0, g]) # The mean of x at timestep 0
xf_bar = array([4, 0, 0, 0, 5, 0, 0, 0, 2, 0, 0, 0, g])
Sigma_x0 = .05*np.eye(13) # Covariance matrix of x at timestep 0
# Sigma_x0[0, 0] = .05
# Sigma_x0[1, 1] = .05
# Sigma_x0[2, 2] = .05
Sigma_w = 0 # 0.001*np.identity(6)# Noise covariance added to x at each timestep
Delta = 0.05 # Probability that we will collide with obstacle
print(Sigma_x0)