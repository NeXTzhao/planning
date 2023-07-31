import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# 二阶贝塞尔曲线参数方程
def bezier(t, p0, p1, p2):
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * p1 + t ** 2 * p2


# 目标函数：最小化曲线与样本点之间的距离误差
def objective_function(p1, t_values, points, samples):
    error = 0
    for i, t in enumerate(t_values):
        x_sample, y_sample = samples[i]
        x_curve, y_curve = bezier(t, points[0], p1, points[1])
        error += (x_sample - x_curve) ** 2 + (y_sample - y_curve) ** 2
    return error


# 通过最小二乘法找到最佳的中间控制点 P1
def fit_bezier_curve(points, t_values, samples):
    initial_p1 = np.mean(points, axis=0)  # 初始值可以取为贝塞尔曲线两端点的平均值
    result = minimize(objective_function, initial_p1, args=(t_values, points, samples))
    p1 = result.x
    return p1


# 绘制贝塞尔曲线
def plot_bezier_curve(points, p1, num_points=100):
    t_values = np.linspace(0, 1, num_points)
    x_curve = []
    y_curve = []
    for t in t_values:
        x, y = bezier(t, points[0], p1, points[1])
        x_curve.append(x)
        y_curve.append(y)

    plt.figure()
    plt.plot([points[0][0], p1[0], points[1][0]], [points[0][1], p1[1], points[1][1]], 'ro-', label='Control Points')
    plt.plot(x_curve, y_curve, 'b-', label='Fitted Bezier Curve')
    plt.scatter(samples[:, 0], samples[:, 1], color='green', label='Samples')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Fitted Bezier Curve with Samples')
    plt.grid()
    plt.show()


# 示例使用：
# 给定的贝塞尔曲线两端点
p0 = np.array([100, 315])
p2 = np.array([300, 495])

# 样本点，这里假设已知曲线上的几个点的函数值
samples = np.array([(100, 315), (150, 400), (200, 430), (250, 460), (300, 495)])

# 拟合二阶贝塞尔曲线的中间控制点 P1
points = np.array([p0, p2])
t_values = np.linspace(0, 1, len(samples))  # 生成对应的 t 值

p1 = fit_bezier_curve(points, t_values, samples)

# 输出拟合得到的中间控制点 P1
print("Fitted Control Point P1:", p1)

# 可视化拟合效果
plot_bezier_curve(points, p1)
