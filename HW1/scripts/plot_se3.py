# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 格式：t x y z qx qy qz qw
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('please inenter trajectory.txt !')
        exit(1)
    else:
        path1 = sys.argv[1] # time=30s
        path2 = sys.argv[2] # time=1s

        path_data1 = np.genfromtxt(path1, delimiter=',')
        path_data2 = np.genfromtxt(path2, delimiter=',')
        # fig = plt.subplots()
        fig = plt.figure(figsize=(12.0, 12.0))
        ax = fig.add_subplot(111, projection='3d')
        # ax = Axes3D(fig)
        plt.plot(path_data1[:, 0], path_data1[:, 1], path_data1[:, 2], color = 'r', linewidth=3, label="time=30s")
        plt.plot(path_data2[:, 0], path_data2[:, 1], path_data2[:, 2], color = 'b', linewidth=3, label="time=1s")
        plt.legend(prop={'family' : 'Times New Roman', 'size'   : 16})
        ax.set_xlabel("x", fontsize=18, labelpad=22)
        ax.set_ylabel("y", fontsize=18, labelpad=22)
        ax.set_zlabel("z", fontsize=18, labelpad=22)
        fig.suptitle("SE3  trajectory", size = 20)

        
        plt.show()
        
        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # plt.rcParams['figure.figsize'] = (16.0, 12.0)
        # plt.title('3D path comparison')
        # # colorList = ['darkred', 'red', 'darkblue', 'blue']
        # colorList = ['r','g','b','y']

        # for i in range(4):
        #     path = sys.argv[i + 1]
        #     path_data = np.genfromtxt(path, delimiter=',')

        #     ax.plot(path_data[:, 0], path_data[:, 1], path_data[:, 2], color = colorList[i])
        #     # plt.title('3D path comparison')
        #     # plt.legend()
        # plt.show()

        
        

        exit(1)
