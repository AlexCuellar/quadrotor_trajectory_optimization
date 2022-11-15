from turtle import color
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
import numpy as np
from pulp import value
from scipy.special import erfinv

def process_object(obj_pos, obj_size):
    x, y, z = np.indices((7, 7, 7))
    # draw cuboids in the top left and bottom right corners, and a link between them
    cube1 = (x <  obj_pos[0] + obj_size[0]/2) & (y <  obj_pos[1] + obj_size[1]/2) & (z <  obj_pos[2] + obj_size[2]/2) \
          & (x >= obj_pos[0] - obj_size[0]/2) & (y >= obj_pos[1] - obj_size[1]/2) & (z >= obj_pos[2] - obj_size[2]/2)
    # combine the objects into a single boolean array
    voxels = cube1
    # set the colors of each object
    colors = np.empty(voxels.shape, dtype=object)
    colors[cube1] = 'blue'
    return voxels, colors    

def visualize_3d(pts, objects):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    voxels_and_colors = []
    for object in objects:
      voxels_and_colors.append(process_object(object[0], object[1]))

    ax.scatter([i[0] for i in pts], [i[1] for i in pts], [i[2] for i in pts], color="r")
    for v_and_c in voxels_and_colors:
      ax.voxels(v_and_c[0], facecolors=v_and_c[1], edgecolor='k')
    #display plot
    plt.show()
