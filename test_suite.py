from avoid_object import plan_trajectory
from shortest_path_approach import shortest_path
from numpy import array


def environment_suite(T, t_length):
    #Empty boi
    print("empty boi")
    objects = []
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = False)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

    #One boi no need to get out da way
    print("One boi no need to get out da way")
    objects = []
    objects.append(([array([0, 3.25, 1.5]), array([2, 1.25, 1.5]), array([0, 1.25, 5.5]), array([0, 1.25, 1.5])], "parallelopiped"))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = True)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=True)
    print("T: ", T, " t_length: ", t_length, " Env: 2  runtime: ", time, " effort: ", effort)

    #One boi get out da way
    print("One boi get out da way")
    objects = []
    objects.append(([array([2, 3.25, 1.5]), array([4, 1.25, 1.5]), array([2, 1.25, 5.5]), array([2, 1.25, 1.5])], "parallelopiped"))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = True)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=True)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

    # Hole boi no need to get out da way
    print("Hole boi no need to get out da way")
    objects = []
    objects.append(([array([6, 0, 5.5]), array([0, 3, 5.5]), array([0, 0, 6.5]), array([0, 0, 5.5])], "parallelopiped"))
    objects.append(([array([6, 4, 5.5]), array([0, 6, 5.5]), array([0, 4, 6.5]), array([0, 4, 5.5])], "parallelopiped"))
    objects.append(([array([3, 3, 5.5]), array([0, 4, 5.5]), array([0, 3, 6.5]), array([0, 3, 5.5])], "parallelopiped"))
    objects.append(([array([6, 3, 5.5]), array([4, 4, 5.5]), array([4, 3, 6.5]), array([4, 3, 5.5])], "parallelopiped"))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = True)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=True)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

    # Hole boi get out da way
    print("Hole boi get out da way")
    objects = []
    objects.append(([array([6, 0, 2]), array([0, 3, 2]), array([0, 0, 3]), array([0, 0, 2])], "parallelopiped"))
    objects.append(([array([6, 4, 2]), array([0, 6, 2]), array([0, 4, 3]), array([0, 4, 2])], "parallelopiped"))
    objects.append(([array([3, 3, 2]), array([0, 4, 2]), array([0, 3, 3]), array([0, 3, 2])], "parallelopiped"))
    objects.append(([array([6, 3, 2]), array([4, 4, 2]), array([4, 3, 3]), array([4, 3, 2])], "parallelopiped"))
    J, X, effort, time = plan_trajectory(50, .1, objects, False, graph = True)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=True)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

environment_suite(50, .1)