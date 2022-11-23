# -*- coding: utf-8 -*-
# @Time    : 2021/7/13 16:08
# @Author  : Xinyi Yu

import matplotlib.pyplot as plt
from gurobipy import *
import matplotlib.hatch
from Parameter import *
from matplotlib.patches import Rectangle
import numpy as np

def printSolution(path_sequence):
    path = []
    position_x = []
    position_y = []

    for i in range(Stl_Hrizon + 1):
        path.append([path_sequence[i][0], path_sequence[i][1]])
        position_x.append(path_sequence[i][0])
        position_y.append(path_sequence[i][1])
    path = np.array(path)

    # draw the map
    a1 = Rectangle((xA1[0], yA1[0]), xA1[1] - xA1[0], yA1[1] - yA1[0], facecolor ="royalblue", alpha=0.3)
    a2 = Rectangle((xA2[0], yA2[0]), xA2[1] - xA2[0], yA2[1] - yA2[0], facecolor ="royalblue", alpha=0.3)
    a3 = Rectangle((xA3[0], yA3[0]), xA3[1] - xA3[0], yA3[1] - yA3[0], facecolor ="mediumblue", alpha=0.6)
    ax = plt.gca()
    ax.add_patch(a1)
    ax.add_patch(a2)
    ax.add_patch(a3)

    plt.text(9, 9, r'A1', family='Calibri', fontsize=22, color='black')
    plt.text(0.3, 0.8, r'A2', family='Calibri', fontsize=22, color='black')
    plt.text(9, 0.8, r'A3', family='Calibri', fontsize=22, color='black')
    # draw the line and dots
    plt.plot(position_x, position_y, color='black', linewidth=1.3)
    plt.scatter(path[:, 0], path[:, 1], c='b')
    plt.scatter(path[0, 0], path[0, 1], c='r')
    plt.xlabel('x/m')
    plt.ylabel("y/m")
    plt.axis([0, 10, 0, 10])

    plt.show()


