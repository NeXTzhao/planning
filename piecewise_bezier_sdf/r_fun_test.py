from bezir_to_poly import *
from implicit_fun import *
from r_fun import *
from r_fun_test2 import *

# 贝塞尔曲线的参数
P0 = np.array([4, 5])
P1 = np.array([7, 7])
P2 = np.array([12, 7])
P3 = np.array([20, 5])
size = 50
sample = 100

from math import sqrt


def ho(x, y, line):
    x1, y1, x2, y2 = line
    f = ((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    d = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)  # length of the line
    t = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / d
    h_fun = (f ** 2 + ((t ** 2 + f ** 4) ** 0.5 - t) ** 2) / 4
    return h_fun


def h(x, y, line):
    x1, y1, x2, y2 = line
    f = ((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    d = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    xc = (x1 + x2) / 2
    yc = (y1 + y2) / 2
    t = (d ** 2 / 4 - (x - xc) ** 2 - (y - yc) ** 2) / d
    h_fun = (f ** 2 + ((t ** 2 + f ** 4) ** 0.5 - t) ** 2) / 4
    return h_fun


def normalize_sdf(sdf):
    sdf_min = np.min(sdf)
    sdf_max = np.max(sdf)
    if sdf_max == sdf_min:
        return np.zeros_like(sdf)
    normalized_sdf = (sdf - sdf_min) / (sdf_max - sdf_min)
    return normalized_sdf


# line0 = [P0[0], P0[1], P1[0], P1[1]]
# line1 = [P1[0], P1[1], P2[0], P2[1]]
# line2 = [P2[0], P2[1], P3[0], P3[1]]
# line3 = [P3[0], P3[1], P0[0], P0[1]]
#
# x_imp = np.linspace(-size, size, sample)
# y_imp = np.linspace(-size, size, sample)
# h0 = h(x_imp, y_imp, line0)
# h1 = h(x_imp, y_imp, line1)
# h2 = h(x_imp, y_imp, line2)
# h3 = h(x_imp, y_imp, line3)
#
# # 计算 h(x, y, line) 函数的最大值和最小值，并进行归一化
# h0_normalized = normalize_sdf(h0)
# h1_normalized = normalize_sdf(h1)
# h2_normalized = normalize_sdf(h2)
# h3_normalized = normalize_sdf(h3)
#
# # # 计算 h(x, y, line) 函数的最大值和最小值
# # h_max = max(h0.max(), h1.max(), h2.max(), h3.max())
# # h_min = min(h0.min(), h1.min(), h2.min(), h3.min())
# #
# # # 将 h(x, y, line) 的值进行线性归一化
# # h0_normalized = (h0 - h_min) / (h_max - h_min)
# # h1_normalized = (h1 - h_min) / (h_max - h_min)
# # h2_normalized = (h2 - h_min) / (h_max - h_min)
# # h3_normalized = (h3 - h_min) / (h_max - h_min)

rectangle_sdf = create_rectangle_sdf(size, sample, P0, P1, P2, P3)

control_points = [P0, P1, P2, P3]
px, py, x, y = bezier_to_poly(control_points)
x_imp = np.linspace(-size, size, sample)
y_imp = np.linspace(-size, size, sample)
X, Y = np.meshgrid(x_imp, y_imp)
evalxy = implicit_fun(px, py).eval(X, Y)

# con = [h0_normalized, h1_normalized, h2_normalized, h3_normalized]
# con = [h0, h1, h2, h3]

# con_sdf = combined_r_function_n(con)
con_cur = [evalxy, rectangle_sdf]
con_cur_sdf = combined_r_function_n(con_cur)
con_cur_sdf_normalized = normalize_sdf(con_cur_sdf)

x_imp1 = np.linspace(-size, size, sample)
y_imp1 = np.linspace(-size, size, sample)
X, Y = np.meshgrid(x_imp1, y_imp1)
# 提取所有 x 和 y 坐标
x_coords = np.array([point[0] for point in control_points])
y_coords = np.array([point[1] for point in control_points])
x_coords = np.append(x_coords, x_coords[0])
y_coords = np.append(y_coords, y_coords[0])
# plt.legend()
fig, axes = plt.subplots(2, 2, figsize=(10, 8))

# Plot the original control points and Bezier curve


fig, axes = plt.subplots(2, 2, figsize=(10, 8))

# Plot the original control points and Bezier curve
axes[0, 0].scatter(x_coords, y_coords, c='red', marker='o', edgecolor='black', s=30, label='Original Control Points')
axes[0, 0].plot(x_coords, y_coords, '--', color='gray', label='Bezier Curve (Dashed)')
axes[0, 0].plot(x, y, color='blue', label=f'Poly Curve', linewidth=2)
axes[0, 0].set_xlabel('X')
axes[0, 0].set_ylabel('Y')
axes[0, 0].legend()

# Plot the isopotential surface of the rectangle SDF
cs3 = axes[0, 1].contour(X, Y, rectangle_sdf, levels=10, cmap='coolwarm')
axes[0, 1].set_xlabel('X')
axes[0, 1].set_ylabel('Y')
axes[0, 1].set_title('Rectangle Isopotential Surface')
plt.clabel(cs3, fmt='%1.1f')  # Add contour labels

# Plot the isopotential surface of the Bezier curve SDF
cs1 = axes[1, 0].contour(X, Y, evalxy, levels=10, cmap='coolwarm')
axes[1, 0].set_xlabel('X')
axes[1, 0].set_ylabel('Y')
axes[1, 0].set_title('Bezier Curve Isopotential Surface')
plt.clabel(cs1, fmt='%1.1f')  # Add contour labels

# Plot the isopotential surface of the combined SDF
cs2 = axes[1, 1].contour(X, Y, con_cur_sdf_normalized, levels=10, cmap='coolwarm')
axes[1, 1].set_xlabel('X')
axes[1, 1].set_ylabel('Y')
axes[1, 1].set_title('Combined Isopotential Surface (Normalized)')
plt.clabel(cs2, fmt='%1.1f')  # Add contour labels

# plt.axis('equal')
plt.tight_layout()
plt.show()

