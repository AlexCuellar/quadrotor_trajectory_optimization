from RMPC_robot import RMPC, LCC
from visualize_environment import visualize_3d
from robot_model import *

def plan_trajectory(T, t_length, objects, use_free, verbose=False, graph=True):
    A, B, x_lims, u_lims = get_dynamics(use_free, t_length)

    xdim = A.shape[1]
    udim = B.shape[1]
    robot_plan = RMPC(T, xdim, x_lims, udim, u_lims, A, B, x0_bar, Sigma_x0, Sigma_w, Delta, xf_bar)

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
    if verbose:
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

    if graph:
        xbar = [robot_plan.xbar(i) for i in range(T)]
        xbar_plot_3d = [[x[4], x[8], x[0]] for x in xbar]

        visualize_3d(xbar_plot_3d, objects)
    
    return robot_plan.objective(), [robot_plan.xbar(i)for i in range(T)], sum([np.linalg.norm(robot_plan.u(i), 1) for i in range(T)])*t_length, end - start
    # visualize_3d(xbar_plot_3d, [])

# objects = []
# objects.append(((3.75, 2.25, 3.5), (2, 2, 4)))
# plan_trajectory(50, .1, objects, False, verbose=False, graph=True)