from bezir_to_poly import *
from implicit_fun import *
from quadrilateral_sdf import *
from r_fun import *

# 贝塞尔曲线的参数
P0 = np.array([4, 2])
P1 = np.array([1, 3])
P2 = np.array([2, 5])
P3 = np.array([5, 6])
lower = -15
upper = 40
sample = 500
x_imp = np.linspace(lower, upper, sample)
y_imp = np.linspace(lower, upper, sample)
X, Y = np.meshgrid(x_imp, y_imp)


def normalize_sdf(sdf):
    sdf_min = np.min(sdf)
    sdf_max = np.max(sdf)
    if sdf_max == sdf_min:
        return np.zeros_like(sdf)
    normalized_sdf = (sdf - sdf_min) / (sdf_max - sdf_min)
    return normalized_sdf


rectangle_sdf = create_rectangle_sdf(X, Y, P0, P1, P2, P3)
rectangle_sdf = normalize_sdf(rectangle_sdf)

control_points = [P0, P1, P2, P3]
px, py, x, y = bezier_to_poly(control_points)

evalxy = implicit_fun(px, py).eval(X, Y)
evalxy = normalize_sdf(evalxy)

# con_cur = [evalxy, rectangle_sdf]
# con_cur_sdf = combined_r_function_n(con_cur)
con_cur_sdf = trim(evalxy, rectangle_sdf)
con_cur_sdf  = normalize_sdf(con_cur_sdf)

# 提取所有 x 和 y 坐标
x_coords = np.array([point[0] for point in control_points])
y_coords = np.array([point[1] for point in control_points])
x_coords = np.append(x_coords, x_coords[0])
y_coords = np.append(y_coords, y_coords[0])

# plt
fig, axes = plt.subplots(2, 3, figsize=(12, 10))

# Plot the original control points and Bezier curve
axes[0, 1].scatter(x_coords, y_coords, c='red', marker='o', edgecolor='black', s=50, label='Original Control Points')
axes[0, 1].plot(x_coords, y_coords, '--', color='gray', label='Bezier Curve (Dashed)')
axes[0, 1].plot(x, y, color='blue', label='Poly Curve', linewidth=2)
axes[0, 1].set_xlabel('X')
axes[0, 1].set_ylabel('Y')
axes[0, 1].legend()

# Plot the isopotential surface of the rectangle SDF
cs3 = axes[1, 0].contourf(X, Y, rectangle_sdf, cmap='coolwarm')
axes[1, 0].set_xlabel('X')
axes[1, 0].set_ylabel('Y')
axes[1, 0].set_title('Rectangle Isopotential Surface')
plt.colorbar(cs3, ax=axes[1, 0], label='Normalized SDF')

# Plot the isopotential surface of the Bezier curve SDF
cs1 = axes[1, 1].contourf(X, Y, evalxy, cmap='coolwarm')
axes[1, 1].set_xlabel('X')
axes[1, 1].set_ylabel('Y')
axes[1, 1].set_title('Bezier Curve Isopotential Surface')
plt.colorbar(cs1, ax=axes[1, 1], label='Normalized SDF')

# Plot the isopotential surface of the combined SDF
cs2 = axes[1, 2].contourf(X, Y, con_cur_sdf, cmap='coolwarm')
axes[1, 2].set_xlabel('X')
axes[1, 2].set_ylabel('Y')
axes[1, 2].set_title('Combined Isopotential Surface (Normalized)')
plt.colorbar(cs2, ax=axes[1, 2], label='Normalized SDF')

# Set equal aspect ratio for all subplots
for ax in axes.flat:
    ax.set_aspect('equal')

plt.tight_layout()
plt.show()
