import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splrep, splev
from sympy import symbols, Matrix, expand, det, lambdify, simplify


def fit_bspline_curve(points, degree, num_control_points, num_samples):
    # Generate a parameterization for the points
    t = np.linspace(0, 1, len(points))

    # Fit the B-spline curve to the points
    tck = splrep(t, points[:, 0], k=degree, s=0)

    # Evaluate the B-spline curve at the specified number of samples
    t_eval = np.linspace(0, 1, num_samples)
    curve_points_x = splev(t_eval, tck)

    tck = splrep(t, points[:, 1], k=degree, s=0)

    # Evaluate the B-spline curve at the specified number of samples
    curve_points_y = splev(t_eval, tck)

    curve_points = np.column_stack((curve_points_x, curve_points_y))

    return curve_points


# Generate some random discrete points
np.random.seed(20)
num_points = 4
points = np.random.rand(num_points, 2) * 10

# Fit a B-spline curve to the points
degree = 3  # Degree of the B-spline curve
num_control_points = 6  # Number of control points to fit
num_samples = 100  # Number of samples to evaluate the B-spline curve

curve_points = fit_bspline_curve(points, degree, num_control_points, num_samples)

# 定义符号变量
a3, a2, a1, a0 = symbols('a3 a2 a1 a0')
b3, b2, b1, b0 = symbols('b3 b2 b1 b0')
x_sym, y_sym = symbols('x y')

# 构建 Sylvester 矩阵
S = Matrix([
    [a3, a2, a1, a0 - x_sym, 0, 0],
    [0, a3, a2, a1, a0 - x_sym, 0],
    [0, 0, a3, a2, a1, a0 - x_sym],
    [b3, b2, b1, b0 - y_sym, 0, 0],
    [0, b3, b2, b1, b0 - y_sym, 0],
    [0, 0, b3, b2, b1, b0 - y_sym],
])

# 计算行列式并展开化简
determinant_expr = det(S)
simplified_expr = simplify(determinant_expr)

# Define the function using lambdify
determinant_function = lambdify((a3, a2, a1, a0, b3, b2, b1, b0, x_sym, y_sym), simplified_expr)


def implicit_eq(x, y):
    a3, a2, a1, a0, b3, b2, b1, b0 = 1, 1, 1, 1, 1, 1, 1, 1,
    return (b0 - y) * (pow(a3, 3) * pow(b0 - y, 2) + b3 * (
            pow(a1, 3) * b3 + pow(a2, 2) * (a1 * b1 + b2 * (a0 - x)) - a1 * a2 * (
            a1 * b2 + 2 * b3 * (a0 - x)) + pow(a2, 3) * (-b0 + y)) + pow(a3, 2) * (
                               a2 * b1 * (-b0 + y) + a1 * (pow(b1, 2) - 2 * b0 * b2 + 2 * b2 * y) + (a0 - x) * (
                               b1 * b2 - 2 * b0 * b3 + 2 * b3 * y)) + a3 * (
                               pow(a1, 2) * (pow(b2, 2) - 2 * b1 * b3) + a1 * b2 * b3 * (a0 - x) + pow(b3,
                                                                                                       2) * pow(
                           a0 - x, 2) + pow(a2, 2) * b2 * (b0 - y) - a2 * (
                                       a1 * b1 * b2 + pow(b2, 2) * (a0 - x) + 3 * a1 * b3 * (-b0 + y)))) - (
            a0 - x) * (pow(a3, 2) * (pow(b1, 3) + b3 * pow(b0 - y, 2) + 2 * b1 * b2 * (-b0 + y)) + b3 * (
            a2 * (pow(b2, 2) - 2 * b1 * b3) * (a0 - x) + b3 * (
            pow(a1, 2) * b1 + b3 * pow(a0 - x, 2) + a1 * b2 * (-a0 + x)) - a1 * a2 * (
                    b1 * b2 - b0 * b3 + b3 * y) + pow(a2, 2) * (pow(b1, 2) + b2 * (-b0 + y))) + a3 * (
                               a1 * b1 * (pow(b2, 2) - 2 * b1 * b3) - (a0 - x) * (
                               pow(b2, 3) - 3 * b1 * b2 * b3 + 2 * pow(b3, 2) * (b0 - y)) - a2 * (
                                       pow(b1, 2) * b2 + pow(b2, 2) * (-b0 + y) + b1 * b3 * (
                                       -b0 + y))))


# 定义距离场函数

# 定义样条线的控制点
# control_points = np.array([[1, 1], [2, 3], [4, 2], [5, 4]])

# 计算样条线参数化
# t_values = np.linspace(0, 1, 300)
# spl = CubicSpline(range(len(control_points)), control_points.T, axis=1)
# sampled_points = spl(t_values)

# print(sampled_points.size)
# 提取样条线上的所有 x 值
# x_range = sampled_points[0]
# y_range = sampled_points[1]
# X, Y = np.meshgrid(x_range, y_range)
x_range = curve_points[:, 0]
y_range = curve_points[:, 1]
X, Y = np.meshgrid(x_range, y_range)
# Z = np.zeros((len(x_range), len(y_range)))

# 计算距离场
# for i, x in enumerate(x_range):
#     for j, y in enumerate(y_range):
#         Z[i, j] = determinant_function(1, 1, 1, 1, 1, 1, 1, 1, x, y)
Z = implicit_eq(X, Y)
# Z = implicit_eq(X, Y)levels=10
# 可视化样条线和距离场
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# 绘制样条线
ax1.scatter(points[:, 0], points[:, 1], color='red', label='Original Points')
ax1.plot(curve_points[:, 0], curve_points[:, 1], 'b-', label='Fitted B-spline Curve')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('样条线')
ax1.legend()

# 绘制距离场
contour = ax2.contourf(X, Y, Z, cmap='viridis')
fig.colorbar(contour, ax=ax2)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_title('距离场')

plt.tight_layout()
plt.show()
