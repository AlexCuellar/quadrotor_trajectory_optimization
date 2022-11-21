from turtle import color
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
import numpy as np
from pulp import value
from scipy.special import erfinv

from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt

def process_object(obj_pos, obj_size):
    xlb, xub = obj_pos[0] - obj_size[0]/2, obj_pos[0] + obj_size[0]/2
    ylb, yub = obj_pos[1] - obj_size[1]/2, obj_pos[1] + obj_size[1]/2
    zlb, zub = obj_pos[2] - obj_size[2]/2, obj_pos[2] + obj_size[2]/2
    x1, y1, z1 = [xlb,xlb,xlb,xlb], [ylb,ylb,yub,yub], [zlb,zub,zub,zlb]
    x2, y2, z2 = [xlb,xlb,xub,xub], [ylb,ylb,ylb,ylb], [zlb,zub,zub,zlb]
    x3, y3, z3 = [xlb,xlb,xub,xub], [ylb,yub,yub,ylb], [zlb,zlb,zlb,zlb]
    x4, y4, z4 = [xub,xub,xub,xub], [ylb,ylb,yub,yub], [zlb,zub,zub,zlb]
    x5, y5, z5 = [xlb,xlb,xub,xub], [yub,yub,yub,yub], [zlb,zub,zub,zlb]
    x6, y6, z6 = [xlb,xlb,xub,xub], [ylb,yub,yub,ylb], [zub,zub,zub,zub]
    verts = [list(zip(x1,y1,z1)), list(zip(x2,y2,z2)), list(zip(x3,y3,z3)), list(zip(x4,y4,z4)), list(zip(x5,y5,z5)), list(zip(x6,y6,z6))]
    return verts    

def visualize_3d(pts, objects):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    all_verts = []
    for object in objects:
      all_verts += process_object(object[0], object[1])

    ax.scatter([i[0] for i in pts], [i[1] for i in pts], [i[2] for i in pts], color="r")
    ax.add_collection3d(Poly3DCollection(all_verts))
    #display plot
    plt.show()
