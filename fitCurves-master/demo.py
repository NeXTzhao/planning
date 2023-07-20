import matplotlib.pyplot as plt
import numpy as np
from fitCurves import *


def generate_data_points():
    x = []
    y = []

    tdata1 = np.linspace(-60, -10, 5)
    tdata2 = np.linspace(-10, -60, 5)

    for t1 in tdata1:
        x.append(100)
        y.append(t1)

    tdata = np.linspace(0, np.pi, 30)
    for t in tdata:
        x.append(100 * np.cos(t))
        y.append(100 * np.sin(t))

    for t2 in tdata2:
        x.append(-100)
        y.append(t2)

    x = x[::-1]
    y = y[::-1]

    print("x = ", x)
    print("y = ", y)
    points = np.column_stack((x, y))
    return points


def generate_arc_points(center, start_radius, end_radius, start_angle, end_angle, num_points):
    theta = np.linspace(start_angle, end_angle, num_points)
    radius = np.linspace(start_radius, end_radius, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    # print('x = ', x)
    # print('y = ', y)
    points = np.column_stack((x, y))
    return points


def calculate_curvature(points):
    dx_dt = np.gradient(points[:, 0])
    dy_dt = np.gradient(points[:, 1])
    d2x_dt2 = np.gradient(dx_dt)
    d2y_dt2 = np.gradient(dy_dt)
    curvature = np.abs(d2x_dt2 * dy_dt - dx_dt * d2y_dt2) / (dx_dt ** 2 + dy_dt ** 2) ** 1.5
    return curvature


if __name__ == '__main__':
    # 原始数据点
    center = (0, 0)  # 中心点坐标
    start_radius = 30  # 起始半径
    end_radius = 150  # 终止半径
    start_angle = 0  # 起始角度（弧度）
    end_angle = np.pi / 2  # 终止角度（弧度）
    num_points = 40  # 离散点数量

    points = generate_arc_points(center, start_radius, end_radius, start_angle, end_angle, num_points)
    curvature = calculate_curvature(points)
    print('curvature = ', curvature)
    # points = generate_data_points()
    max_error = 1

    plt.scatter(points[:, 0], points[:, 1], c='red', label='Original Points')
    # 拟合曲线
    beziers = fitCurve(points, max_error)

    # 绘制拟合的曲线，每个曲线使用不同颜色
    colors = ['blue', 'orange', 'green', 'purple', 'red', 'yellow']  # 定义颜色列表
    for i, bezier in enumerate(beziers):
        x = [point[0] for point in bezier]  # 提取所有点的 x 值
        y = [point[1] for point in bezier]  # 提取所有点的 y 值
        color = colors[i % len(colors)]  # 根据索引选择颜色
        plt.plot(x, y, color=color, label=f'fit{i + 1} points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend()
    plt.title('Fitted Bezier Curves')
    plt.show()
