import numpy as np
import matplotlib.pyplot as plt

class Pcontroller:
    def __init__(self, Kp):
        self.m = 0.065  
        self.g = 9.81  
        self.kT = 5.276e-4
        self.r = 1  
        self.Kp = Kp
        self.dt = 0.05
        self.time = np.arange(0, 100, self.dt)
        self.h = 0
        self.velo = 0
        self.h_list = []
        self.velo_list = []

    def intergartion(self):
        for _ in self.time:
            u = self.Kp * (self.r - self.h) + (self.m * self.g) / (4 * self.kT)
            velo_dot = (4 * self.kT * u - self.m * self.g) / self.m
            self.velo += velo_dot * self.dt
            self.h += self.velo * self.dt
            self.h_list.append(self.h)
            self.velo_list.append(self.velo)

    def visualize(self):
        plt.figure()
        # plt.plot(self.time, self.h_list, linewidth=2, color='r')

        # plt.xlabel('Timestamp')
        # plt.ylabel('Hieght/m')

        plt.plot(self.time, self.velo_list, linewidth=2, color='r')
        plt.xlabel('Timestamp')
        plt.ylabel('Hieght-velocity/ m/s')
        plt.title(f'Kp = {self.Kp}')
        plt.savefig('kp=5_1.png', dpi=300)
        plt.show()

def main():
    # system1 = Pcontroller(5)
    # system2 = Pcontroller(15)
    system3 = Pcontroller(50)
    # numerical integration
    # system1.intergartion()
    # system2.intergartion()
    system3.intergartion()

    # Plot 
    # system1.visualize()
    # system2.visualize()
    system3.visualize()

if __name__ == "__main__":
    main()
