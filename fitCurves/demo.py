import matplotlib.pyplot as plt
import numpy as np
from fitCurves import *


def generate_arc_points(center, radius, start_angle, end_angle, num_points):
    theta = np.linspace(start_angle, end_angle, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    points = np.column_stack((x, y))
    return points


if __name__ == '__main__':
    # 原始数据点
    # points = np.array([[0.0, 0.0], [1.2, 1.1], [3.1, 2.1], [4.1, 6.1], [5.1, 8.9], [8.5, 3.3]])
    # 弧线参数
    center = (0, 0)  # 中心点坐标
    radius = 10.0  # 半径
    start_angle = 0  # 起始角度（弧度）
    end_angle = np.pi / 2  # 终止角度（弧度）
    num_points = 100  # 离散点数量

    # 生成弧线的离散点
    points = generate_arc_points(center, radius, start_angle, end_angle, num_points)

    max_error = 0.001
    plt.scatter(points[:, 0], points[:, 1], c='red', label='Original Points')
    # 拟合曲线
    beziers = fitCurve(points, max_error)

    # 绘制拟合的曲线，每个曲线使用不同颜色
    colors = ['blue', 'orange', 'green', 'purple', 'red', 'yellow']  # 定义颜色列表
    for i, bezier in enumerate(beziers):
        x = [point[0] for point in bezier]  # 提取所有点的 x 值
        y = [point[1] for point in bezier]  # 提取所有点的 y 值
        color = colors[i % len(colors)]  # 根据索引选择颜色
        plt.plot(x, y, color=color, label=f'fit{i+1} points')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Fitted Bezier Curves')
    plt.show()
