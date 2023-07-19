# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# plt.rcParams['font.sans-serif'] = ['SimHei']
# plt.rcParams['axes.unicode_minus'] = False

# write to csv
# velocities = [4.3, 4.5]
# df = pd.DataFrame([[velocities[0], velocities[1]]], columns=['obs_1', 'obs_2'])
# df.to_csv('/home/next/cartesian_planner/src/data/obs_state.csv', mode="a", index=False, header=True)

# read to csv
speed_data = pd.read_csv("/home/next/cartesian_planner/src/data/path.csv", usecols=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
                         names=None)
speed = speed_data.values.tolist()
# actual_speed = []
# expected_speed = []
# opt_x = []
# opt_y = []
# steer_angle = []
# opt_kappa = []

actual_speed = []
expected_speed = []
coarse_x = []
coarse_y = []
coarse_kappa = []
coarse_velocity = []
coarse_theta = []
opt_x = []
opt_y = []
steer_angle = []
opt_kappa = []

for item in speed:
    actual_speed.append(item[0])
    if item[1] != 0:
        expected_speed.append(item[1])
    coarse_x.append(item[2])
    coarse_y.append(item[3])
    coarse_kappa.append(item[4])
    coarse_velocity.append(item[5])
    coarse_theta.append(item[6])
    opt_x.append(item[7])
    opt_y.append(item[8])
    steer_angle.append(item[9])
    opt_kappa.append(item[10])


# 计算动作
def action(s):
    a1 = []
    for i in range(len(s) - 1):
        if s[i + 1] - s[i] > 0:
            if i == 0:
                a1.append(10)
            a1.append(10)
        elif s[i + 1] - s[i] < 0:
            if i == 0:
                a1.append(15)
            a1.append(15)
        else:
            if i == 0:
                a1.append(5)
            a1.append(5)
    return a1


t1 = range(0, 15, 1)
t2 = np.linspace(0, 15, len(expected_speed))
t3 = np.linspace(0, 15, len(actual_speed))
t4 = np.linspace(0, 150, len(opt_kappa))

a = action(expected_speed)

plt.figure(figsize=(6, 4))
grid = plt.GridSpec(14, 3)
# 纵向决策
# plt.subplot(311)
plt.subplot(grid[0:3, :])
plt.step(t2, a, color='white')
plt.xticks(t1)
plt.yticks([5, 10, 15], [r'$LCL$', r'$LCR$', r'$LK$'], fontsize=10)
plt.xlabel('时间(s)', fontsize=10)
plt.ylabel('纵向动作决策', fontsize=11)
plt.grid(visible='true', which='major', axis='x', color='grey', linestyle='--')
# 横向决策
# plt.subplot(312)
plt.subplot(grid[4:7, :])
plt.step(t2, a, color='white')
plt.xticks(t1)
plt.yticks([5, 10, 15], [r'$KS$', r'$ACC$', r'$DEC$'], fontsize=10)
plt.xlabel('时间(s)', fontsize=10)
plt.ylabel('横向动作决策', fontsize=11)
plt.grid(visible='true', which='major', axis='x', color='grey', linestyle='--')
#
# plt.subplot(313)
plt.subplot(grid[8:, :])
xe = [0, 1.5, 5.3, 7.2, 8.2, 11.1, 15]
ye = [18, 25.3, 25.3, 15.8, 15.8, 25.3, 25.3]
points = np.array([[0, 18], [2.4, 23], [2.9, 23], [3.2, 21.8], [4.2, 21.8], [8.6, 10.0], [12.05, 10.0], [15, 17.2]])
xe1, ye1 = points[:, 0], points[:, 1]
points1 = np.array(
    [[0, 18], [0.5, 18], [1, 16.85], [2.4, 16.85], [4.3, 20.5], [4.9, 20.5], [5.4, 20], [5.8, 20], [7.1, 16.8],
     [15.0, 16.8]])
xe2, ye2 = points1[:, 0], points1[:, 1]
points2 = np.array(
    [[0, 18], [1.5, 25], [5.3, 25], [6.8, 22], [7.9, 22], [9.8, 25], [15, 25]])
xe3, ye3 = points2[:, 0], points2[:, 1]
# plt.plot(xe3, ye3, label='期望速度')
plt.plot(t2, expected_speed, color='white')
plt.plot(t3, actual_speed, color='white')
# plt.legend(loc='best')
plt.xticks(t1)
plt.xlabel('时间(s)', fontsize=10)
plt.ylabel(r'速度$(km/h)$', fontsize=11)
plt.grid(visible='true', which='major', axis='x', color='grey', linestyle='--')
plt.tight_layout()
# plt.savefig('/home/next/planning/Notes/python_test/13.png', dpi=500)

# plt.figure()
# plt.subplot(211)
# plt.plot(opt_x, opt_y, '--', label='Trajectory')
# plt.legend(loc='best')
# # plt.axis('equal')
# plt.xticks()
# plt.xlabel('X(m)', fontsize=12)
# plt.ylabel('Y(m)', fontsize=12)
# plt.grid(visible='true', which='major', axis='x', color='grey', linestyle='--')

# plt.subplot(212)

# plt.subplot(212)
# plt.plot(t3, steer_angle, color='g')
# # plt.legend(loc=4)
# plt.xticks(t1)
# plt.xlabel('Time(s)', fontsize=12)
# plt.ylabel('Steering angle(rad)', fontsize=13)
# plt.grid(visible='true', which='major', axis='x', color='grey', linestyle='--')


plt.show()

