
import matplotlib.pyplot as plt
from gurobipy import *
from Parameter import *
from matplotlib.patches import Rectangle
import numpy as np
from matplotlib import rc


def printSolution(path_sequence):
    path1 = []
    path2 = []
    position_x1 = []
    position_y1 = []
    position_x2 = []
    position_y2 = []
    # dots and path copy
    for i in range(spl_t[1] + 1):
        path1.append([path_sequence[i][0], path_sequence[i][1]])
        position_x1.append(path_sequence[i][0])
        position_y1.append(path_sequence[i][1])
    for i in range(spl_t[1], Stl_Hrizon + 1):
        path2.append([path_sequence[i][0], path_sequence[i][1]])
        position_x2.append(path_sequence[i][0])
        position_y2.append(path_sequence[i][1])
    path1 = np.array(path1)
    path2 = np.array(path2)

    # draw the map
    a1 = Rectangle((xA1[0], yA1[0]), xA1[1] - xA1[0], yA1[1] - yA1[0], facecolor ="royalblue", alpha=0.3)
    a2 = Rectangle((xA2[0], yA2[0]), xA2[1] - xA2[0], yA2[1] - yA2[0], facecolor ="royalblue", alpha=0.3)
    a3 = Rectangle((xA3[0], yA3[0]), xA3[1] - xA3[0], yA3[1] - yA3[0], facecolor ="mediumblue", alpha=0.6)
    terminal = Rectangle((x_t[0], y_t[0]), x_t[1] - x_t[0], y_t[1] - y_t[0], label=r"$\hat{X}_{12,12}^{\{2,3\}}$", hatch="///", edgecolor="red", linewidth=0.3, alpha=0.35, fill=False)
    ax = plt.gca()
    ax.add_patch(a1)
    ax.add_patch(a2)
    ax.add_patch(a3)
    ax.add_patch(terminal)
    plt.legend(loc="upper left")
    # display the annotation
    x_tmp = path_sequence[spl_t[1]][0]
    y_tmp = path_sequence[spl_t[1]][1]
    ax.annotate(r"$x_{12}$", xy=(x_tmp, y_tmp), xytext=(x_tmp - 1, y_tmp + 1), style="italic", fontsize=13, arrowprops=dict(arrowstyle="->", color="black", linewidth=2), color="black")
    plt.text(9, 9, r'A1', family='Calibri', fontsize=22, color='black')
    plt.text(0.3, 0.8, r'A2', family='Calibri', fontsize=22, color='black')
    plt.text(9, 0.8, r'A3', family='Calibri', fontsize=22, color='black')

    # draw the line and dots
    plt.plot(position_x1, position_y1, color='blue', linewidth=1.3)
    plt.plot(position_x2, position_y2, color='black', linewidth=1.3, linestyle="--")
    plt.scatter(path1[:, 0], path1[:, 1], color='blue', linewidth =0.1)
    plt.scatter(path1[0, 0], path1[0, 1], color='red', linewidth =0.5, marker="s")
    plt.scatter(path2[:, 0], path2[:, 1], color='black', linewidth =0.1)
    plt.scatter(path2[0, 0], path2[0, 1], color='black', linewidth =0.5, marker="s")
    plt.xlabel('x/m')
    plt.ylabel('y/m')
    plt.axis([0, 10, 0, 10])
    plt.show()


