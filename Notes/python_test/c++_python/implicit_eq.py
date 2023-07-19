import numpy as np
import matplotlib.pyplot as plt


# 定义隐式方程
def f(x, y):
    return x ** 2 + y ** 2 - x * y - 8


# 生成网格点
x = np.linspace(-10, 10, 100)
y = np.linspace(-10, 10, 100)
X, Y = np.meshgrid(x, y)

# 计算隐式方程值
Z = f(X, Y)

# 绘制等高线曲线
plt.contour(X, Y, Z, levels=[-5, 0, 5, 12, 25], colors='black')

# 设置图形参数
plt.axis('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Level Curves of an Ellipse')

# 显示图形
plt.show()
