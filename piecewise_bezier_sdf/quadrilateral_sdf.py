# def sdf_line_segment(p1, p2, points):
#     # 计算线段SDF
#     v = p2 - p1
#     w = points - p1
#
#     c1 = np.dot(w, v)
#     c2 = np.dot(v, v)
#
#     b = c1 / c2
#
#     # 计算点到线段的距离
#     dist = np.linalg.norm(w - np.outer(b, v), axis=1)
#
#     # 判断点在线段的位置
#     mask1 = (c1 <= 0)
#     mask2 = (c2 <= c1)
#     dist[mask1] = np.linalg.norm(w[mask1], axis=1)
#     dist[mask2] = np.linalg.norm(points[mask2] - p2, axis=1)
#
#     return dist
#
#
# def normalize_sdf(sdf):
#     max_val = np.max(sdf)
#     min_val = np.min(sdf)
#     return (sdf - min_val) / (max_val - min_val)
#
#
# def r_function(sdf_a, sdf_b, blend):
#     # R函数：将两个SDF场进行混合拼接
#     p = 2
#     denominator = np.sqrt((sdf_a ** p + sdf_b ** p) + 1e-6)
#     return sdf_a + sdf_b - denominator
# def r_function_n(sdf_fields, blend):
#     p = 2
#     result = sdf_fields[0]
#     for sdf in sdf_fields[1:]:
#         denominator = np.sqrt((result ** p + sdf ** p) + 1e-6)
#         result = result + sdf - denominator
#     return result
#
#
# def create_rectangle_sdf(p1, p2, p3, p4):
#     # 生成坐标网格
#     x = np.linspace(-2, 5, 500)
#     y = np.linspace(-2, 2, 500)
#     X, Y = np.meshgrid(x, y)
#     points = np.stack((X.flatten(), Y.flatten()), axis=-1)
#
#     # 计算四个线段的SDF场
#     sdf_line_segment_1 = sdf_line_segment(p1, p2, points)
#     sdf_line_segment_2 = sdf_line_segment(p2, p3, points)
#     sdf_line_segment_3 = sdf_line_segment(p3, p4, points)
#     sdf_line_segment_4 = sdf_line_segment(p4, p1, points)
#
#     # 使用R函数进行混合拼接
#     result_sdf = r_function(sdf_line_segment_1, sdf_line_segment_2, 0.5)
#     result_sdf = r_function(result_sdf, sdf_line_segment_3, 0.5)
#     result_sdf = r_function(result_sdf, sdf_line_segment_4, 0.5)
#
#     # 归一化SDF场
#     sdf_line_segment_1 = normalize_sdf(sdf_line_segment_1)
#     sdf_line_segment_2 = normalize_sdf(sdf_line_segment_2)
#     sdf_line_segment_3 = normalize_sdf(sdf_line_segment_3)
#     sdf_line_segment_4 = normalize_sdf(sdf_line_segment_4)
#     result_sdf = normalize_sdf(result_sdf)
#
#     # 重新整形结果
#     sdf_line_segment_1 = np.reshape(sdf_line_segment_1, X.shape)
#     sdf_line_segment_2 = np.reshape(sdf_line_segment_2, X.shape)
#     sdf_line_segment_3 = np.reshape(sdf_line_segment_3, X.shape)
#     sdf_line_segment_4 = np.reshape(sdf_line_segment_4, X.shape)
#     result_sdf = np.reshape(result_sdf, X.shape)
#
#     return X, Y, sdf_line_segment_1, sdf_line_segment_2, sdf_line_segment_3, sdf_line_segment_4, result_sdf
#
#
# def bezier_curve(t, control_points):
#     # 贝塞尔曲线生成函数
#     n = len(control_points) - 1
#     p = np.zeros_like(control_points[0])
#     for i in range(n + 1):
#         p += control_points[i] * np.math.comb(n, i) * (1 - t) ** (n - i) * t ** i
#     return p
#
#
# def create_curve_sdf(control_points):
#     # 生成曲线等势面的SDF场
#     x_imp = np.linspace(-2, 3, 500)
#     y_imp = np.linspace(-2, 3, 500)
#     X, Y = np.meshgrid(x_imp, y_imp)
#
#     # 在这里根据贝塞尔曲线的控制点生成曲线等势面的SDF场
#     sdf_curve = np.zeros_like(X)
#     for t in np.linspace(0, 1, 100):
#         px, py = bezier_curve(t, control_points)
#         sdf_curve += np.exp(-((X - px) ** 2 + (Y - py) ** 2) / 0.1 ** 2)
#
#     return sdf_curve
#
#
#
#
# # 设置四个点的参数
# p1 = np.array([0.5, 0.2])
# p2 = np.array([1.0, 0.7])
# p3 = np.array([1.8, 0.9])
# p4 = np.array([2.0, 0.5])
#
# # 生成矩形的SDF场
# X, Y, _, _, _, _, result_sdf_rect = create_rectangle_sdf(p1, p2, p3, p4)
#
# # 生成曲线等势面的SDF场，使用样条曲线的控制点
# curve_control_points = np.array([[0.5, 0.2], [1.0, 0.7], [1.8, 0.9], [2.0, 0.5]])
# curve_sdf = create_curve_sdf(curve_control_points)
# # 归一化SDF场
# result_sdf_rect = normalize_sdf(result_sdf_rect)
# curve_sdf = normalize_sdf(curve_sdf)
#
# # 使用矩阵距离场和曲线等势面的SDF场进行混合拼接
# result_sdf_combined = r_function_n([result_sdf_rect, curve_sdf], 0.5)
#
# plt.figure(figsize=(12, 8))
#
# # 绘制矩形等势面
# plt.subplot(2, 2, 1)
# cs_rect = plt.contour(X, Y, result_sdf_rect, levels=10, cmap='coolwarm')
# plt.colorbar()
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Rectangle Isopotential Surface')  # 矩形等势面
# plt.clabel(cs_rect, fmt='%1.1f')  # 添加等值线标签
#
# # 绘制曲线等势面
# plt.subplot(2, 2, 2)
# cs_curve = plt.contour(X, Y, curve_sdf, levels=10, cmap='coolwarm')
# plt.colorbar()
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Curve Isopotential Surface')  # 曲线等势面
# plt.clabel(cs_curve, fmt='%1.1f')  # 添加等值线标签
#
# # 绘制合并后的等势面
# plt.subplot(2, 1, 2)
# cs_combined = plt.contour(X, Y, result_sdf_combined, levels=10, cmap='coolwarm')
# plt.colorbar()
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Combined Isopotential Surface')  # 合并后的等势面
# plt.clabel(cs_combined, fmt='%1.1f')  # 添加等值线标签
#
# plt.tight_layout()
# plt.show()

import math

import numpy as np


def trim(f, t):
    # return np.sqrt(f ** 2 + (np.abs(t) - t) ** 2 * 0.25)
    f = normalize_sdf(f)
    t = normalize_sdf(t)
    return np.sqrt(f ** 2 + (np.sqrt(t ** 2 + f ** 4) - t) ** 2 * 0.25)


def normalize_sdf(sdf):
    max_val = np.max(sdf)
    min_val = np.min(sdf)
    return (sdf - min_val) / (max_val - min_val)


def combined_r_function_n(sdf_fields, p=2):
    normalized_sdf_fields = [normalize_sdf(sdf) for sdf in sdf_fields]

    result = normalized_sdf_fields[0]
    for sdf in normalized_sdf_fields[1:]:
        denominator = np.sqrt((result ** p + sdf ** p) + 1e-6)
        result = result + sdf - denominator
    return result


def create_rectangle_sdf(lower, upper, sample, p1, p2, p3, p4):
    def sdf_line_segment(x, y, start_point, end_point):
        # Calculate line segment SDF
        # v = p2 - p1
        # w = points - p1
        #
        # c1 = np.dot(w, v)
        # c2 = np.dot(v, v)
        #
        # b = c1 / c2
        #
        # # Calculate distance from points to the line segment
        # dist = np.linalg.norm(w - np.outer(b, v), axis=1)
        #
        # # Check if points are outside the line segment
        # mask1 = (c1 <= 0)
        # mask2 = (c2 <= c1)
        # dist[mask1] = np.linalg.norm(w[mask1], axis=1)
        # dist[mask2] = np.linalg.norm(points[mask2] - p2, axis=1)
        # return dist

        # 计算隐式方程
        xi, yi = start_point
        xi1, yi1 = end_point
        hi = ((x - xi) * (yi1 - yi) - (y - yi) * (xi1 - xi)) / math.sqrt((xi1 - xi) ** 2 + (yi1 - yi) ** 2)
        return hi

    def norm_sdf(sdf):
        max_val = np.max(sdf)
        min_val = np.min(sdf)
        return (sdf - min_val) / (max_val - min_val)

    # def r_function(sdf_a, sdf_b, blend):
    #     # R function: blend two SDF fields
    #     p = 2
    #     denominator = np.sqrt((sdf_a ** p + sdf_b ** p) + 1e-6)
    #     return sdf_a + sdf_b - denominator
    #     # return np.sqrt(sdf_a ** 2 + (np.abs(sdf_b) - sdf_b) ** 2 * 0.25)
    #
    def r_function(sdf_a, sdf_b, blend):
        # 计算 R-合取操作
        return sdf_a + sdf_b - np.sqrt(sdf_a ** 2 + sdf_b ** 2)

    # Generate coordinate grid
    x = np.linspace(lower, upper, sample)
    y = np.linspace(lower, upper, sample)
    X, Y = np.meshgrid(x, y)

    # Calculate SDF for four line segments of the rectangle
    sdf_line_segment_1 = sdf_line_segment(X, Y, p1, p2)
    sdf_line_segment_2 = sdf_line_segment(X, Y, p2, p3)
    sdf_line_segment_3 = sdf_line_segment(X, Y, p3, p4)
    sdf_line_segment_4 = sdf_line_segment(X, Y, p4, p1)

    # Use R function to blend the SDF fields
    result_sdf = r_function(sdf_line_segment_1, sdf_line_segment_2, 0.5)
    result_sdf = r_function(result_sdf, sdf_line_segment_3, 0.5)
    result_sdf = r_function(result_sdf, sdf_line_segment_4, 0.5)

    # Normalize the SDF fields
    result_sdf = norm_sdf(result_sdf)

    # Reshape the results
    # sdf_line_segment_1 = np.reshape(sdf_line_segment_1, X.shape)
    # sdf_line_segment_2 = np.reshape(sdf_line_segment_2, X.shape)
    # sdf_line_segment_3 = np.reshape(sdf_line_segment_3, X.shape)
    # sdf_line_segment_4 = np.reshape(sdf_line_segment_4, X.shape)
    result_sdf = np.reshape(result_sdf, X.shape)

    # return X, Y, sdf_line_segment_1, sdf_line_segment_2, sdf_line_segment_3, sdf_line_segment_4, result_sdf
    return result_sdf
