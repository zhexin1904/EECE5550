# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pdb
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('please inenter trajectory.txt !')
        exit(1)
    else:
        path1 = sys.argv[1] # original
        path2 = sys.argv[2] # pf

        fig = plt.figure()
    
        path_data1 = np.genfromtxt(path1, delimiter=',')

        fig = plt.figure(figsize=(12.0, 12.0))
        plt.legend(prop={'family' : 'Times New Roman', 'size'   : 16})
        plt.xlabel("x", fontsize=18, labelpad=22)
        plt.ylabel("y", fontsize=18, labelpad=22)
        
        with open(path2, "r") as file:
            lines = file.readlines()



        

        x = []  
        y = [] 

        current_color = 'b'  # original color
        legend_handles = []
        for line in lines:
            if line.strip() == '*':
                # detect *
                scatter = plt.scatter(x, y, c=current_color, label='t = 10s') 
                legend_handles.append(scatter)
               
                # pdb.set_trace()/

                x = []
                y = []
            else:
                # 
                parts = line.strip().split(',')
                if len(parts) == 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))

        plt.plot(path_data1[:, 0], path_data1[:, 1], color = 'black', linewidth=4, label="No noise trajectory")
        groundtruth = plt.Line2D([0], [0], color='black', linewidth=4, label='No noise trajectory')
        legend_handles.append(groundtruth)
        label=['t = 10s']
        plt.legend(handles=legend_handles, labels= label + ['No noise trajectory'], prop={'family': 'Times New Roman', 'size': 16})
        plt.xlim(-3, 4)
        plt.ylim(-1, 6)
        
        plt.title("Propogation_10s", fontsize=20)
        plt.show()
        fig.savefig("../pic/Propogation_10s.png", dpi=300, bbox_inches="tight")

        exit(1)
