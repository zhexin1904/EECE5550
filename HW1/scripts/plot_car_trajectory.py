# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 格式：t x y z qx qy qz qw
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('please inenter trajectory.txt !')
        exit(1)
    else:
        path1 = sys.argv[1] # time=30s

        path_data1 = np.genfromtxt(path1, delimiter=',')
        # fig = plt.subplots()
        fig = plt.figure(figsize=(12.0, 12.0))
        ax = Axes3D(fig)
        ax.plot(path_data1[:, 0], path_data1[:, 1], path_data1[:, 2], color = 'r', linewidth=3, label="time=5s")
        plt.legend(prop={'family' : 'Times New Roman', 'size'   : 16})
        ax.set_xlabel("x", fontsize=18, labelpad=22)
        ax.set_ylabel("y", fontsize=18, labelpad=22)
        ax.set_zlabel("z", fontsize=18, labelpad=22)
        fig.suptitle("SE3  car trajectory", size = 20)

        
        plt.show()
    
        
        

        exit(1)
