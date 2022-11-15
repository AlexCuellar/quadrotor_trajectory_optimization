from RMPC_robot import RMPC, LCC
from visualize_environment import visualize_3d
from robot_model import *

robot_plan = RMPC(T, 13, x_lims, 4, u_lims, A, B, x0_bar, Sigma_x0, Sigma_w, Delta, xf_bar)

# # Define Obstacle
objects = []
objects.append(((3, 1.5, 2.5), (6, 3, 1)))
objects.append(((3, 5, 2.5), (6, 2, 1)))
objects.append(((1.5, 3.5, 2.5), (3, 1, 1)))
objects.append(((5, 3.5, 2.5), (2, 1, 1)))

obj_counter = 0
for object in objects:
    robot_plan.add_object(object[0], object[1], None, obj_counter)
    obj_counter += 1

# Distribute risk to each chance constraint
for i in range(len(robot_plan.lcc)):
    robot_plan.lcc[i].delta = robot_plan.Delta/len(robot_plan.lcc)

# Find Plan
start = datetime.now()
robot_plan.determinize_and_solve()
end = datetime.now()
print("time taken: ", (end - start).microseconds)

for i in range(T):
    print("u[",i,"]: ", robot_plan.u(i))
print()
for i in range(T+1):
    print("x[",i,"]: ", robot_plan.xbar(i))
print()
for i in range(T+1):
    print("M[",i,"]: ", [x for x in robot_plan.M(i)])
print("Optimized objective: J = {}".format(robot_plan.objective()))

xbar = [robot_plan.xbar(i) for i in range(T)]
xbar_plot_3d = [[x[4], x[8], x[0]] for x in xbar]

visualize_3d(xbar_plot_3d, objects)
