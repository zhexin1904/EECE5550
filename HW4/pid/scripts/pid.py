import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, discont):
        self.m = 0.065  
        self.kT = 5.276e-4  
        self.g = 9.81  
        self.r = 1  
        self.h = 0  
        self.velo = 0  
        self.dt = 0.05  
        self.time = np.arange(0, 150, self.dt)
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.discont = discont
        self.altitude_pid = []
        self.velocity_pid = []
        self.integral = 0

    def intergartion(self):
        for _ in self.time:
            error = self.r - self.h
            self.integral += error * self.dt
            u = self.Kp * error + self.Ki * self.integral + self.Kd * (-self.velo) + (self.m * self.g) / (4 * self.kT)
            u_actual = self.discont * u  
            # System dynamics
            acc = (4 * self.kT * u_actual - self.m * self.g) / self.m
            self.velo += acc * self.dt
            self.h += self.velo * self.dt
            # Store results
            self.altitude_pid.append(self.h)
            self.velocity_pid.append(self.velo)

    def plot_results(self):
        # plt.plot(self.time, self.altitude_pid, label='Height / m', linewidth=2, color='r')
        # plt.axhline(y=self.r, color='b', linestyle='--', label='ref')
        # plt.legend()
        # plt.xlabel('Timestamp')
        # plt.title('PID')
        plt.plot(self.time, self.velocity_pid, label='Vertical velocity', linewidth=2, color='r')
        plt.legend()
        plt.xlabel('Timestamp')
        plt.tight_layout()
        plt.show()

def main():
    # Create an instance of the PIDControlSystem class
    system = PIDController(10, 1.8, 14, 0.95)

    # Run the simulation
    system.intergartion()

    # Plot results
    system.plot_results()

if __name__ == "__main__":
    main()
