from RMPC_robot import RMPC, LCC
from visualize_environment import visualize_3d
from robot_model import *
import control as ct
import scipy.linalg
from visualize_environment import *
from avoid_object import plan_trajectory

def get_effort(u):
    u[0] += m*g
    return np.linalg.norm(u, 1)*.001

def feed_foward(x0, xf, A, B, K):
    x0 = np.reshape(x0, (12, 1))
    xf = np.reshape(xf, (12, 1))
    # print("A: ", A.shape)
    # print("B: ", B.shape)
    x_err = x0 - xf
    # print(x_err)
    u = -K.dot(x_err)
    # print("X ERR: ", x_err.shape)
    # print("OPT U: ", u)
    x = x0 + (A.dot(x0) + B.dot(u))*.001
    # print("x new: ", x.shape)
    return x, u

def get_shortest_path():
    A, B, x_lims, u_lims = get_dynamics(True)

    xdim = A.shape[1]
    udim = B.shape[1]
    robot_plan = RMPC(T, xdim, x_lims, udim, u_lims, A, B, x0_bar, Sigma_x0, Sigma_w, Delta, xf_bar)

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
    # print("time taken: ", (end - start).microseconds)


    print("Optimized objective: J = {}".format(robot_plan.objective()))

    xbar = [robot_plan.xbar(i) for i in range(T)]
    xbar_flat = [np.array([x[0], 0, 0, 0, x[4], 0, 0, 0, x[8], 0, 0, 0]) for x in xbar]
    return xbar_flat

def follow_shortest_path(A, B, Q, R, path, verbose=False):
    t = 0
    K1a, X, E = ct.lqr(A, B, Q, R)
    real_path = []
    x = path[0]
    effort = 0
    for i in range(1000000):
        # print(t)
        # print(x)
        # real_path.append(x)
        x_goal = path[t + 1]
        x, u = feed_foward(x, x_goal, A, B, K1a)
        effort += get_effort(u)
        dist_to_goal = ((x[0] - x_goal[0])**2 + (x[4] - x_goal[4])**2 + (x[8] - x_goal[8])**2)**.5
        if verbose:
            print(dist_to_goal, i, t)
        if dist_to_goal < .1:
            real_path.append(x)
            t = t + 1
        if t == len(path) - 1:
            return real_path, effort
    return False, np.inf
        

def shortest_path(T, t_length, objects, verbose=False, graph=True):
    start = datetime.now()
    J, X, effort, time = plan_trajectory(T, t_length, objects, True, verbose=False, graph=False)
    xbar_flat = [np.array([x[0], 0, 0, 0, x[4], 0, 0, 0, x[8], 0, 0, 0]) for x in X]
    A_real, B_real, x_lims_real, u_lims_real = get_dynamics(False, 1)

    A_real_no_gravity = A_real[:-1, :-1] - np.eye(12)
    B_real_no_gravity = B_real[:-1, :]
    # print(A_real_no_gravity)
    # print(B_real_no_gravity)
    R = np.eye(4)*1000
    Q = np.diag([100, .1, .1, .1, 10, .1, .1, .1, 10, .1, .1, .1])

    real_path, effort = follow_shortest_path(A_real_no_gravity, B_real_no_gravity, Q, R, xbar_flat, verbose=verbose)
    end = datetime.now()
    real_path_coordinates = [[x[4][0], x[8][0], x[0][0]] for x in real_path]
    shortest_path_coordinates = [[x[4], x[8], x[0]] for x in xbar_flat]
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    if graph:
        ax.scatter([i[0] for i in real_path_coordinates], [i[1] for i in real_path_coordinates], [i[2] for i in real_path_coordinates], color="r")
        ax.scatter([i[0] for i in shortest_path_coordinates], [i[1] for i in shortest_path_coordinates], [i[2] for i in shortest_path_coordinates], color="b")

        #display plot
        plt.show()
    
    return X, effort, end - start


# objects = []
# objects.append(((3.75, 2.25, 3.5), (2, 2, 4)))
# X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
# print(effort)