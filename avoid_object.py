from RMPC_robot import RMPC, LCC
from visualize_environment import visualize_3d
from robot_model import *

xdim = A.shape[1]
udim = B.shape[1]
robot_plan = RMPC(T, xdim, x_lims, udim, u_lims, A, B, x0_bar, Sigma_x0, Sigma_w, Delta, xf_bar)

# # Define Obstacle
# objects = []
# objects.append(((3, 1.5, 2.5), (6, 3, 1)))
# objects.append(((3, 5, 2.5), (6, 2, 1)))
# objects.append(((1.5, 3.5, 2.5), (3, 1, 1)))
# objects.append(((5, 3.5, 2.5), (2, 1, 1)))

objects = []
# objects.append(([array([0, 1, 3]), array([4, 2, 4]), array([5, 0, 2]), array([1, 1, 1])], "tetrahedron")) # this one worketh not
objects.append(([array([0, 3, 2]), array([6, 0, 2]), array([0, 0, 3]), array([0, 0, 2])], "parallelopiped")) # yas
# objects.append(([array([0, 1, 0]), array([1, 0, 0]), array([0, 0, 1]), array([0, 0, 0])], "tetrahedron")) # dis one also borked
objects.append(([array([0, 1, 0]), array([1, 0, 0]), array([1, 1, 1]), array([0, 0, 0])], "parallelopiped")) # yas pt 2

obj_counter = 0
for object in objects:
    robot_plan.add_object(object[0], object[1], None, obj_counter)
    obj_counter += 1

visualize_3d([(2.5, 2.5, 1), (4, 5, 2)], objects) # prints objects and also start and end points so i can be justifiably ANGY

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
for i in range(T):
    print("|x|[",i,"]: ", robot_plan.dx_abs(i))
print("Optimized objective: J = {}".format(robot_plan.objective()))

xbar = [robot_plan.xbar(i) for i in range(T)]
xbar_plot_3d = [[x[4], x[8], x[0]] for x in xbar]

visualize_3d(xbar_plot_3d, objects)
