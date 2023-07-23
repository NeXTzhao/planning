from bezir_to_poly import *
from fitCurves import *
from implicit_fun import *
from quadrilateral_sdf import *


# from r_fun import *


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

    # print("x = ", x)
    # print("y = ", y)
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
    points = generate_data_points()
    max_error = 1.5

    # 拟合曲线
    control_points, curves = fitCurve(points, max_error)

    colors = ['blue', 'orange', 'green', 'purple', 'red', 'yellow']  # 定义颜色列表

    lower = -135
    upper = 135
    sample = 100
    x_imp = np.linspace(lower, upper, sample)
    y_imp = np.linspace(lower, upper, sample)
    X, Y = np.meshgrid(x_imp, y_imp)
    # 设置子图的行数和列数
    rows = 2
    cols = 3
    fig, axes = plt.subplots(rows, cols, figsize=(12, 8))
    sdf_results = []
    for i, control_point in enumerate(control_points):
        px, py, x, y = bezier_to_poly(control_point)
        color = colors[i % len(colors)]  # 根据索引选择颜色
        # 计算等值线数据
        evalxy = implicit_fun(px, py).eval(X, Y)
        # sdf_results.append(evalxy)
        rectangle = create_rectangle_sdf(lower, upper, sample, control_point[0], control_point[1], control_point[2],
                                         control_point[3])
        trim_sdf = trim(evalxy, rectangle)
        # trim_sdf = normalize_sdf(trim_sdf)
        sdf_results.append(trim_sdf)

        # 在对应的子图中绘制等值线和拟合曲线
        ax = axes[i // cols, i % cols]
        cs = ax.contour(X, Y, evalxy, colors='gray', alpha=0.6)
        # ax.contour(X, Y, trim_sdf, colors='red', alpha=0.6)
        ax.plot(x, y, color=color, label=f'poly{i + 1} curve', linewidth=2)
        plt.clabel(cs, fmt='%1.1f')  # 添加等值线标签
    # 在一个子图中绘制所有拟合曲线
    ax_fit = axes[1, 1]
    ax_fit.scatter(points[:, 0], points[:, 1], c='red', marker='o', edgecolor='black', s=30, label='Original Points')

    for i, control_point in enumerate(control_points):
        px, py, x, y = bezier_to_poly(control_point)
        color = colors[i % len(colors)]  # 根据索引选择颜色
        label = f'poly_Curve {i + 1}'
        ax_fit.plot(x, y, color=color, label=label, linewidth=2, linestyle='-', alpha=0.8)
        # 设置图例样式
        ax_fit.legend(loc='upper right', fontsize=12, frameon=True, edgecolor='black')

        # 添加网格线
        ax_fit.grid(color='gray', linestyle='--', linewidth=0.5)

    integrated_sdf = combined_r_function_n(sdf_results)
    integrated_sdf = normalize_sdf(integrated_sdf)
    cs = plt.contour(X, Y, integrated_sdf, cmap='coolwarm')
    plt.colorbar()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.title('Integrated SDF Field')
    # plt.clabel(cs, fmt='%1.1f')  # 添加等值线标签

    plt.tight_layout()
    plt.show()

    # 绘制拟合的曲线，每个曲线使用不同颜色
    # for i, bezier in enumerate(curves):
    #     x = [point[0] for point in bezier]  # 提取所有点的 x 值
    #     y = [point[1] for point in bezier]  # 提取所有点的 y 值
    #     color = colors[i % len(colors)]  # 根据索引选择颜色
    #     plt.plot(x, y, color=color, label=f'fit{i + 1} curve')
