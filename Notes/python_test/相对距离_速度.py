# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import numpy.random as random

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

heading = []
heading_true = []
heading_true1 = []

dis = []
dis_true = []
dis_true1 = []

speed = []
speed_true = []
speed_true1 = []

time = range(0, 8, 1)

for i in range(0, 160, 1):
    # heading
    heading.append(90 + random.uniform(-0.015, 0.015))
    heading_true.append(90)
    heading_true1.append(90.0015)

    # speed
    speed.append(8 + random.uniform(-0.015, 0.015))
    speed_true.append(8)
    speed_true1.append(8.0025)

    # # distance
    dis.append(25 + random.uniform(-0.01, 0.01))
    dis_true.append(25)
    dis_true1.append(25.0025)

x_ticks = np.linspace(0, 8, 160)

# heading
# plt.plot(x_ticks, heading, 'b', label='传感器测量值')
# plt.plot(x_ticks, heading_true, 'black', label='未加入噪声的真实值')
# plt.plot(x_ticks, heading_true1, 'red', label='滤波后结果')
# plt.legend(loc='best', fontsize=10)
# plt.xlabel(r'$Time(s)$', fontsize=10)
# plt.ylabel(r'航向角$(deg)$', fontsize=10)
# plt.ylim(89.965, 90.035)


# plt.figure()
# plt.plot(x_ticks, heading, 'b', label='传感器测量值')
# plt.legend(loc='best', fontsize=10)
# plt.xlabel(r'$Time(s)$', fontsize=10)
# plt.ylabel(r'航向角$(deg)$', fontsize=10)
# plt.ylim(85.975, 95.025)


# speed
# plt.figure()
# plt.plot(x_ticks, speed, 'b', label='传感器测量值')
# plt.plot(x_ticks, speed_true, 'black', label='未加入噪声的真实值')
# plt.plot(x_ticks, speed_true1, 'red', label='滤波后结果')
# plt.legend(loc='best', fontsize=10)
# plt.xlabel(r'$Time(s)$', fontsize=10)
# plt.ylabel(r'车速$(m/s)$', fontsize=10)
# plt.ylim(7.965, 8.035)
#
# plt.figure()
# plt.plot(x_ticks, speed, 'b', label='传感器测量值')
# plt.legend(loc='best', fontsize=10)
# plt.xlabel(r'$Time(s)$', fontsize=10)
# plt.ylabel(r'车速$(m/s)$', fontsize=10)
# plt.ylim(7.975, 8.025)

# dis
plt.figure()
plt.plot(x_ticks, dis, 'b', label='传感器测量值')
plt.plot(x_ticks, dis_true, 'black', label='未加入噪声的真实值')
plt.plot(x_ticks, dis_true1, 'red', label='滤波后结果')
plt.legend(loc='best', fontsize=10)
plt.xlabel(r'$Time(s)$', fontsize=10)
plt.ylabel(r'相对距离$(m)$', fontsize=10)
plt.ylim(24.98, 25.02)
#
# plt.figure()
# plt.plot(x_ticks, dis, 'b', label='传感器测量值')
# plt.legend(loc='best', fontsize=10)
# plt.xlabel(r'$Time(s)$', fontsize=10)
# plt.ylabel(r'相对距离$(m)$', fontsize=10)
# plt.ylim(24.98, 25.02)

plt.grid()
plt.show()
