import math
import numpy as np
import matplotlib.pyplot as plt


class VehicleSimulation:
    def __init__(self, num=10, time=1000, length=400, d_init=15, d_safe=7, t_safe=1.5, v_init=8, v_max=15, v_min=0,
                 a_init=0, a_max=2, a_min=-4, step=0.1):
        self.num = num
        self.time = time
        self.length = length
        self.d_init = d_init
        self.d_safe = d_safe
        self.t_safe = t_safe
        self.v_init = v_init
        self.v_max = v_max
        self.v_min = v_min
        self.a_init = a_init
        self.a_max = a_max
        self.a_min = a_min
        self.step = step

        self.x = np.zeros((num, time))
        self.v = np.zeros((num, time))
        self.a = np.zeros((num, time))

    def init_vehicles(self):
        for i in range(self.num):
            self.x[i][0] = (self.num - i - 1) * self.d_init
            self.v[i][0] = self.v_init
            self.a[i][0] = self.a_init

    def update_vehicle(self, i, t):
        dx = self.x[i - 1][t - 1] - self.x[i][t - 1] if i != 0 else 99999
        dv = self.v[i][t - 1] - self.v[i - 1][t - 1]
        vt = self.v[i][t - 1]
        sn = self.d_safe + vt * self.t_safe + vt * dv / (2 * math.sqrt(self.a_max * abs(self.a_min)))
        result = self.a_max * (1 - pow(vt / self.v_max, 4) - pow(sn / dx, 2))
        return result

    def simulate(self):
        self.init_vehicles()

        for t in range(1, self.time):
            for i in range(self.num):
                self.a[i][t] = self.update_vehicle(i, t)
                self.v[i][t] = self.v[i][t - 1] + self.a[i][t] * self.step
                self.x[i][t] = self.x[i][t - 1] + self.v[i][t - 1] * self.step + 0.5 * self.a[i][
                    t] * self.step * self.step

    def plot_result(self):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        axes = axes.flatten()

        for n in range(self.num):
            axes[0].plot(range(self.time), self.x[n], label=f"Vehicle {n + 1}")
            axes[1].plot(range(self.time), self.v[n], label=f"Vehicle {n + 1}")
            axes[2].plot(range(self.time), self.a[n], label=f"Vehicle {n + 1}")

        axes[0].set_xlabel('$time(s)$')
        axes[0].set_ylabel('$space(m)$')
        axes[1].set_xlabel('$time(s)$')
        axes[1].set_ylabel('$velocity(m/s)$')
        axes[2].set_xlabel('$time(s)$')
        axes[2].set_ylabel('$acceleration(m/s^2)$')

        for ax in axes:
            ax.legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    simulator = VehicleSimulation()
    simulator.simulate()
    simulator.plot_result()
