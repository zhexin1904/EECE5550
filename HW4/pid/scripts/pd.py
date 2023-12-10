import numpy as np
import matplotlib.pyplot as plt

class PDController:
    def __init__(self, Kp, Kd):
        self.m = 0.065  
        self.ref = 1  
        self.Kp = Kp
        self.Kd = Kd
        self.dt = 0.05
        self.g = 9.81  
        self.kT = 5.276e-4
        self.altitude = 0
        self.velo = 0
        self.time = np.arange(0, 100, self.dt)
        self.altitude_values = []
        self.velo_values = []

    def intergartion(self):
        for _ in self.time:
            output = self.Kp * (self.ref - self.altitude) + self.Kd * (-self.velo) + (self.m * self.g) / (4 * self.kT)
            velo_dot = (4 * self.kT * output - self.m * self.g) / self.m
            self.velo += velo_dot * self.dt
            self.altitude += self.velo * self.dt
            self.altitude_values.append(self.altitude)
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
