import numpy as np
import matplotlib.pyplot as plt


# 计算线段的有向距离（SDF）
def sdf_line_segment(p1, p2, points):
    v = p2 - p1
    w = points - p1

    c1 = np.dot(w, v)
    c2 = np.dot(v, v)

    b = c1 / c2

    dist = np.linalg.norm(w - np.outer(b, v), axis=1)

    mask1 = (c1 <= 0)
    mask2 = (c2 <= c1)
    dist[mask1] = np.linalg.norm(w[mask1], axis=1)
    dist[mask2] = np.linalg.norm(points[mask2] - p2, axis=1)

    return dist


# R函数：将两个SDF场进行混合拼接
def r_function(sdf_a, sdf_b, blend):
    p = 2
    denominator = np.sqrt((sdf_a ** p + sdf_b ** p) + 1e-6)
    return sdf_a + sdf_b - denominator


# 将SDF归一化到[0, 1]范围
def normalize_sdf(sdf):
    max_val = np.max(sdf)
    min_val = np.min(sdf)
    return (sdf - min_val) / (max_val - min_val)


# 多个SDF场混合拼接
def r_function_n(sdf_fields, blend):
    p = 2
    result = sdf_fields[0]
    for sdf in sdf_fields[1:]:
        denominator = np.sqrt((result ** p + sdf ** p) + 1e-6)
        result = result + sdf - denominator
    return result


# 多个SDF场混合拼接并归一化
def combined_r_function_n(sdf_fields, p=2):
    normalized_sdf_fields = [normalize_sdf(sdf) for sdf in sdf_fields]

    result = normalized_sdf_fields[0]
    for sdf in normalized_sdf_fields[1:]:
        denominator = np.sqrt((result ** p + sdf ** p) + 1e-6)
        result = result + sdf - denominator
    return result


# 生成SDF场和坐标网格
def generate_sdf(p1, p2, p3, p4, resolution=500):
    x = np.linspace(-2, 3, resolution)
    y = np.linspace(-2, 3, resolution)
    X, Y = np.meshgrid(x, y)
    points = np.stack((X.flatten(), Y.flatten()), axis=-1)

    sdf_line_segment_1 = sdf_line_segment(p1, p2, points)
    sdf_line_segment_2 = sdf_line_segment(p3, p4, points)

    sdf_line_segment_1 = np.reshape(sdf_line_segment_1, X.shape)
    sdf_line_segment_2 = np.reshape(sdf_line_segment_2, X.shape)

    result_sdf = r_function(sdf_line_segment_1, sdf_line_segment_2, 0.5)

    sdf_line_segment_1 = normalize_sdf(sdf_line_segment_1)
    sdf_line_segment_2 = normalize_sdf(sdf_line_segment_2)
    result_sdf = normalize_sdf(result_sdf)

    result_sdf = np.reshape(result_sdf, X.shape)
    return X, Y, sdf_line_segment_1, sdf_line_segment_2, result_sdf


# 绘制SDF场
def plot_sdf(integrated_sdf):
    x = np.linspace(-200, 200, 100)
    y = np.linspace(-200, 200, 100)
    X, Y = np.meshgrid(x, y)
    plt.subplot(2, 1, 2)
    cs = plt.contour(X, Y, integrated_sdf, levels=50, cmap='coolwarm')
    plt.colorbar()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.title('Integrated SDF Field')
    plt.clabel(cs, fmt='%1.1f')

    # plt.tight_layout()
    # plt.show()


# # # 设置参数
# p1 = np.array([0.0, 0.0])
# p2 = np.array([1.0, 1.0])
# p3 = np.array([1.0, 1.0])
# p4 = np.array([2.0, 0.0])
#
# # 生成SDF场和坐标网格
# X, Y, sdf1, sdf2, integrated_sdf = generate_sdf(p1, p2, p3, p4)
#
# # 绘制SDF场
# plot_sdf( integrated_sdf)
