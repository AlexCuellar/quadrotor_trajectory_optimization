from robot_model import *

def next_step(x, u):
    return A.dot(x) + B.dot(u)

def run_steps(x0, u_all):
    x_vals = [x0]
    for u in u_all:
        x_vals.append(next_step(x_vals[-1], u)) 
    return x_vals

u_all = []
u_all.append(np.array([0, 0, .001, 0]))
u_all.append(np.array([0, 0, 0, 0]))
u_all.append(np.array([0, 0, 0, 0]))
u_all.append(np.array([0, 0, 0, 0]))
u_all.append(np.array([0, 0, -.001, 0]))
x_vals = run_steps(x0_bar, u_all)
for x in x_vals:
    print(x)
