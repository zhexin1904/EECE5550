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

        path_data1 = np.genfromtxt(path1, delimiter=',')

        fig = plt.figure(figsize=(12.0, 12.0))
        # ax = Axes3D(fig),.
        plt.legend(prop={'family' : 'Times New Roman', 'size'   : 16})
        plt.xlabel("x", fontsize=18, labelpad=22)
        plt.ylabel("y", fontsize=18, labelpad=22)

    
        
        
        with open(path2, "r") as file:
            lines = file.readlines()

        color_to_label = {
            'b': 't = 5s',
            'g': 't = 10s',
            'r': 't = 15s',
            'c': 't = 20s',
        }

        

        x = []  
        y = [] 

        current_color = 'b'  # original color
        legend_handles = []
        for line in lines:
            if line.strip() == '*':
                # detect *
                label_text = color_to_label[current_color]
                scatter = plt.scatter(x, y, c=current_color, label=label_text)                
                legend_handles.append(scatter)
                # pdb.set_trace()/

                x = []
                y = []
                # color
                if current_color == 'b':
                    current_color = 'g'
                elif current_color == 'g':
                    current_color = 'r'
                elif current_color == 'r':
                    current_color = 'c'
                elif current_color == 'c':
                    current_color = 'm'
                else:
                    current_color = 'b'
            else:
                # 
                parts = line.strip().split(',')
                if len(parts) == 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))


        plt.plot(path_data1[:, 0], path_data1[:, 1], color = 'black', linewidth=4, label="No noise trajectory")
        groundtruth = plt.Line2D([0], [0], color='black', linewidth=4, label='No noise trajectory')
        legend_handles.append(groundtruth)

        labels1 = ['t = 5s',
        't = 10s',
        't = 15s',
        't = 20s']

        plt.legend(handles=legend_handles, labels= labels1 + ['No noise trajectory'], prop={'family': 'Times New Roman', 'size': 16})
        plt.xlim(-3, 3)
        plt.ylim(-1, 5)
        
        plt.title("Distribution_after_updated", fontsize=20)
        plt.show()
        fig.savefig("../pic/Distribution_after_updated.png", dpi=300, bbox_inches="tight")

        exit(1)
