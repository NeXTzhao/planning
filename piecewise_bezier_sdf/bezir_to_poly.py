import numpy as np


# import matplotlib.pyplot as plt


def bezier_to_poly(control_points):
    P0, P1, P2, P3 = control_points
    t = np.linspace(0, 1, 100)

    def basis_functions(t):
        B0 = (1 - t) ** 3
        B1 = 3 * (1 - t) ** 2 * t
        B2 = 3 * (1 - t) * t ** 2
        B3 = t ** 3
        return B0, B1, B2, B3

    B0, B1, B2, B3 = basis_functions(t)

    # 将三次贝塞尔曲线转化为三次多项式曲线
    poly_x = B0 * P0[0] + B1 * P1[0] + B2 * P2[0] + B3 * P3[0]
    poly_y = B0 * P0[1] + B1 * P1[1] + B2 * P2[1] + B3 * P3[1]

    # 绘制三次贝塞尔曲线
    # plt.plot(poly_x, poly_y, label='Bezier Curve')

    # 计算多项式曲线的系数
    poly_coeffs_x = np.polyfit(t, poly_x, 3)
    poly_coeffs_y = np.polyfit(t, poly_y, 3)

    # 构造多项式曲线的函数
    poly_func_x = np.poly1d(poly_coeffs_x)
    poly_func_y = np.poly1d(poly_coeffs_y)

    # 参数 t 取值范围为 [0, 1]
    t_poly = np.linspace(0, 1, 100)

    # 计算多项式曲线的值
    poly_x = poly_func_x(t_poly)
    poly_y = poly_func_y(t_poly)
    print('coff_x = ', poly_coeffs_x)
    print('coff_y = ', poly_coeffs_y)

    return poly_coeffs_x, poly_coeffs_y, poly_x, poly_y

# 三次贝塞尔曲线的控制点
# control_point = [
#     [1, 1],
#     [2, 3],
#     [4, 2],
#     [5, 4]
# ]

# x, y = bezier_to_poly(control_point)
# plt.plot(x, y, label='Polynomial Curve')
# con_x, con_y = zip(*control_point)
# plt.scatter(con_x, con_y, color='red', label='Control Points')
#
# # 绘制控制点
#
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('Bezier Curve and Polynomial Curve')
# plt.legend()
# plt.grid(True)
# plt.show()
