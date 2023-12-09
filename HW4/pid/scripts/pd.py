import numpy as np
import matplotlib.pyplot as plt

class PDController:
    def __init__(self, Kp, Kd):
        self.m = 0.065  
        self.g = 9.81  
        self.kT = 5.276e-4
        self.ref = 1  
        self.Kp = Kp
        self.Kd = Kd
        self.dt = 0.05
        self.time = np.arange(0, 100, self.dt)
        self.h = 0
        self.velo = 0
        self.h_values = []
        self.velo_values = []

    def intergartion(self):
        for _ in self.time:
            u = self.Kp * (self.ref - self.h) + self.Kd * (-self.velo) + (self.m * self.g) / (4 * self.kT)
            velo_dot = (4 * self.kT * u - self.m * self.g) / self.m
            self.velo += velo_dot * self.dt
            self.h += self.velo * self.dt
            self.h_values.append(self.h)
            self.velo_values.append(self.velo)

    def plot(self):
        plt.figure()
        # plt.plot(self.time, self.h_values, label='Height / m', linewidth=2, color='r')
        # plt.axhline(y=self.ref, color='b', linestyle='--', label='ref')
        # plt.xlabel('Timestamp')
        # plt.legend()
        # plt.title('overdamped')
        plt.plot(self.time, self.velo_values, label='Vertical velocity', linewidth=2, color='r')
        plt.legend()
        plt.xlabel('Timestamp')
        plt.title('overdamped')
        plt.tight_layout()
        plt.show()

def main():
    system = PDController(10, 66)
    system.intergartion()
    system.plot()

if __name__ == "__main__":
    main()
