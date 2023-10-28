# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb
if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Please input valid file')
        exit(1)
    else:
        path1 = sys.argv[1]
        path_data1 = np.loadtxt(path1)
        path2 = sys.argv[2]
        path_data2 = np.loadtxt(path2)
        path3 = sys.argv[2]
        path_data3 = np.loadtxt(path3)
        fig = plt.figure()
        ax = Axes3D(fig)

        # ax.scatter(path_data1[:, 0], path_data1[:, 1], path_data1[:, 2], c='r', s=4, label = "source")
        ax.scatter(path_data2[:, 0], path_data2[:, 1], path_data2[:, 2], c= 'r',cmap='viridis',alpha = 0.5, s=1, label = "target")
        ax.scatter(path_data3[:, 0], path_data3[:, 1], path_data3[:, 2], c= 'b', cmap='viridis',alpha = 0.5, s=1, label = "result")
        plt.legend(prop={'family': 'Times New Roman', 'size': 16})
        plt.title('ICP ')
        plt.show()
        fig.savefig("../pic/ICP_matplot.png", dpi=300, bbox_inches="tight")

        exit(1)
