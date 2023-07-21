import numpy as np
import matplotlib.pyplot as plt


class implicit_fun:
    def __init__(self, px, py):
        self.px = px
        self.py = py
        x0 = np.poly1d(px)(0)
        y0 = np.poly1d(py)(0)
        dx = self.gradx(x0, y0)
        dy = self.grady(x0, y0)
        self.scale = np.hypot(dx, dy)

    def eval(self, x, y):
        [a3, a2, a1, a0] = self.px
        [b3, b2, b1, b0] = self.py
        out = (b0 - y) * (pow(a3, 3) * pow(b0 - y, 2) + b3 * (
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
        return out / self.scale

    def gradx(self, x, y):
        [a3, a2, a1, a0] = self.px
        [b3, b2, b1, b0] = self.py
        out = pow(a3, 2) * (pow(b1, 3) + 3 * b3 * pow(b0 - y, 2) + 3 * b1 * b2 * (-b0 + y)) + b3 * (
                2 * a2 * (pow(b2, 2) - 2 * b1 * b3) * (a0 - x) + b3 * (
                pow(a1, 2) * b1 + 3 * b3 * pow(a0 - x, 2) + 2 * a1 * b2 * (-a0 + x)) - a1 * a2 * (
                        b1 * b2 - 3 * b0 * b3 + 3 * b3 * y) + pow(a2, 2) * (
                        pow(b1, 2) + 2 * b2 * (-b0 + y))) + a3 * (
                      -2 * (a0 - x) * (pow(b2, 3) - 3 * b1 * b2 * b3 + 3 * pow(b3, 2) * (b0 - y)) - a2 * (
                      pow(b1, 2) * b2 + 2 * pow(b2, 2) * (-b0 + y) + b1 * b3 * (-b0 + y)) + a1 * (
                              b1 * pow(b2, 2) - 2 * pow(b1, 2) * b3 + b2 * b3 * (-b0 + y)))
        return out

    def grady(self, x, y):
        [a3, a2, a1, a0] = self.px
        [b3, b2, b1, b0] = self.py
        out = b3 * (-(pow(a1, 3) * b3) - pow(a2, 2) * (a1 * b1 + 2 * b2 * (a0 - x)) + a1 * a2 * (
                a1 * b2 + 3 * b3 * (a0 - x)) + 2 * pow(a2, 3) * (b0 - y)) - 3 * pow(a3, 3) * pow(b0 - y, 2) + pow(
            a3, 2) * (2 * a2 * b1 * (b0 - y) - a1 * (pow(b1, 2) - 4 * b0 * b2 + 4 * b2 * y) - 3 * (a0 - x) * (
                b1 * b2 - 2 * b0 * b3 + 2 * b3 * y)) + a3 * (
                      -(pow(a1, 2) * (pow(b2, 2) - 2 * b1 * b3)) + a2 * (2 * pow(b2, 2) + b1 * b3) * (a0 - x) - 3 * pow(
                  b3, 2) * pow(a0 - x, 2) + a1 * b2 * b3 * (-a0 + x) + 2 * pow(a2, 2) * b2 * (-b0 + y) + a1 * a2 * (
                              b1 * b2 + 6 * b3 * (-b0 + y)))
        return out


# # 创建空列表来存储x和y坐标数据
# xdata = []
# ydata = []
#
# # 添加半径为50的U型转弯数据点（x坐标为50*cos(t)，y坐标为50*sin(t)，t从0到π）
#
#
# tdata1 = np.linspace(-40, 0, 30)
# # for t1 in tdata1:
# #     xdata.append(50)
# #     ydata.append(t1)
# tdata = np.linspace(0, np.pi, 100)
# for t in tdata:
#     xdata.append(100 * np.cos(t))
#     ydata.append(100 * np.sin(t))
#
# # for t1 in tdata1:
# #     xdata.append(-50)
# #     ydata.append(t1)
#
# # 计算弧长数据
# sdata = []
# for i in range(len(xdata)):
#     if i == 0:
#         sdata.append(0)
#     else:
#         iprev = i - 1
#         dx = xdata[i] - xdata[iprev]
#         dy = ydata[i] - ydata[iprev]
#         sprev = sdata[i - 1]
#         s = sprev + np.hypot(dx, dy)
#         sdata.append(s)
#
# print('x', xdata)
# print('y', ydata)
# # print('s', sdata)
#
# # 使用三次多项式拟合弧长和x、y坐标之间的关系
# px = np.polyfit(sdata, xdata, 3)
# py = np.polyfit(sdata, ydata, 3)
# fx = np.poly1d(px)  # 生成拟合的x坐标函数
# fy = np.poly1d(py)  # 生成拟合的y坐标函数
#
# # print('px : ', px)
# # print('py : ', py)
#
# # 在拟合函数上生成等间隔的弧长数据点
# sdata_fit = np.linspace(0, sdata[-1], 100)
# xdata_fit = fx(sdata_fit)
# ydata_fit = fy(sdata_fit)
#
# # 创建包含两个子图的图表对象
# fig, ax = plt.subplots(1, 2)
#
# # 在第一个子图上绘制原始数据点和拟合曲线
# ax[0].plot(xdata, ydata, 'r.')  # 绘制原始数据点
# ax[0].plot(xdata_fit, ydata_fit)  # 绘制拟合曲线
# ax[0].axis('equal')  # 设置坐标轴比例相等，保证图形不会被拉伸或压缩
#
# # 在第二个子图上绘制隐式函数的等高线图
# x = np.linspace(-60, 60, 10)
# y = np.linspace(-60, 60, 10)
# X, Y = np.meshgrid(x, y)
# evalxy = implicit_fun(px, py)  # 创建隐式函数对象
# F = evalxy.eval(X, Y)  # 计算隐式函数在网格点上的值
# # print('X = ',X)
# # print('Y = ',Y)
# # print('F : ', F)
# contour = ax[1].contourf(X, Y, F)  # 绘制等高线图
# ax[1].axis('equal')  # 设置坐标轴比例相等，保证图形不会被拉伸或压缩
# plt.colorbar(contour)
#
# # 保存图形为SVG格式的文件
# plt.savefig('u_turn.svg')
#
# # 显示图形
# plt.show()
