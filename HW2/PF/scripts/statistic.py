import numpy as np
import sys

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('please enter trajectory.txt!')
        exit(1)
    else:
        path1 = sys.argv[1]

        with open(path1, "r") as file:
            lines = file.readlines()

        x_values = []
        y_values = []

        means_x = []
        means_y = []
        covariances = []

        current_x = []
        current_y = []

        for line in lines:
            if "*" in line:
                if current_x and current_y:
                    x_data = np.array(current_x)
                    y_data = np.array(current_y)
                    mean_x = np.mean(x_data)
                    mean_y = np.mean(y_data)
                    covariance = np.cov(x_data, y_data, rowvar=False)
                    means_x.append(mean_x)
                    means_y.append(mean_y)
                    covariances.append(covariance)

                current_x = []
                current_y = []
            else:
                parts = line.strip().split(',')
                if len(parts) == 2:
                    x_value, y_value = map(float, parts)
                    current_x.append(x_value)
                    current_y.append(y_value)

        if current_x and current_y:
            x_data = np.array(current_x)
            y_data = np.array(current_y)
            mean_x = np.mean(x_data)
            mean_y = np.mean(y_data)
            covariance = np.cov(x_data, y_data, rowvar=False)
            means_x.append(mean_x)
            means_y.append(mean_y)
            covariances.append(covariance)

        for i, (mean_x, mean_y, covariance) in enumerate(zip(means_x, means_y, covariances)):
            print(f"Segment {i + 1}:")
            print(f"Mean of x: {mean_x}")
            print(f"Mean of y: {mean_y}")
            print("Covariance Matrix:")
            print(covariance)
            print()

        exit(0)
