from avoid_object import plan_trajectory
from shortest_path_approach import shortest_path

#Empty boi
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
    objects.append(((1, 2.25, 3.5), (2, 2, 4)))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = False)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
    print("T: ", T, " t_length: ", t_length, " Env: 2  runtime: ", time, " effort: ", effort)

    #One boi get out da way
    print("One boi get out da way")
    objects = []
    objects.append(((3.75, 2.25, 3.5), (2, 2, 4)))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = False)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

    # Hole boi no need to get out da way
    print("Hole boi no need to get out da way")
    objects = []
    objects.append(((3, 1.5, 6), (6, 3, 1)))
    objects.append(((3, 5, 6), (6, 2, 1)))
    objects.append(((1.5, 3.5, 6), (3, 1, 1)))
    objects.append(((5, 3.5, 6), (2, 1, 1)))
    J, X, effort, time = plan_trajectory(T, t_length, objects, False, graph = False)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

    # Hole boi get out da way
    print("Hole boi get out da way")
    objects = []
    objects.append(((3, 1.5, 2.5), (6, 3, 1)))
    objects.append(((3, 5, 2.5), (6, 2, 1)))
    objects.append(((1.5, 3.5, 2.5), (3, 1, 1)))
    objects.append(((5, 3.5, 2.5), (2, 1, 1)))
    J, X, effort, time = plan_trajectory(50, .1, objects, False, graph = False)
    print("T: ", T, " t_length: ", t_length, " J: ", J, " runtime: ", time, " effort: ", effort)
    X, effort, time = shortest_path(50, .1, objects, verbose=False, graph=False)
    print("T: ", T, " t_length: ", t_length, " runtime: ", time, " effort: ", effort)

environment_suite(50, .1)