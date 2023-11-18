# coding=UTF-8
# HW3 of EECE5550
# Zhexin Xu, xu.zhex@northeastern.edu
# 11/11
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
import numpy as np
import sys

path1 = sys.argv[1]
path_data1 = np.loadtxt(path1, delimiter=',')

path2 = sys.argv[2]
path_data2 = np.loadtxt(path2, delimiter=',')

fig = plt.figure()
ax = Axes3D(fig)
# modification 
path_data1[:, 2:4] *= -1
path_data2[:, 2:4] *= -1
ax.invert_zaxis()
ax.invert_xaxis()

ax.plot(path_data1[:, 1], path_data1[:, 2], path_data1[:, 3], linewidth=5, label='Optimized Trajectory')

start_point = path_data1[0, 1:4]
end_point = path_data1[-1, 1:4]
ax.scatter(start_point[0], start_point[1], start_point[2], s=50, marker='o', c='b', label='Start')
ax.scatter(end_point[0], end_point[1], end_point[2], s=50, marker='o', c='g', label='End')

ax.text(start_point[0] + 0.007, start_point[1], start_point[2], 'Start', color='r', fontsize=15, ha='center', va='bottom')
ax.text(end_point[0] + 0.005, end_point[1], end_point[2], 'End', color='g', fontsize=15, ha='center', va='bottom')
ax.tick_params(axis='x', pad=5)
ax.tick_params(axis='y', pad=5)
ax.tick_params(axis='z', pad=5)

tag_legend = Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=10, label='apriltag')

handles, labels = ax.get_legend_handles_labels()

handles.append(tag_legend)
labels.append('apriltag')

ax.legend(handles=handles, labels=labels)

for i, (x, y, z) in enumerate(zip(path_data2[:, 1], path_data2[:, 2], path_data2[:, 3])):
    ax.scatter(x, y, z, s=50, marker='o', c='r')
    ax.text(x, y - 0.002, z - 0.002, 'tag' + str(int(path_data2[i, 0])), color='black', fontsize=10)

ax.set_xlabel('X', fontsize=15)
ax.set_ylabel('Y', fontsize=15)
ax.set_zlabel('Z', fontsize=15)

plt.title('Optimized trajectory and AprilTag poses')
plt.show()

fig.savefig('../pic/plot_new1.png', dpi=300, bbox_inches='tight') 


exit(1)
