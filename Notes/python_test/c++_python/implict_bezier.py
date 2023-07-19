import numpy as np
import matplotlib.pyplot as plt

# 定义控制点
P0x, P0y = -2, 0
P1x, P1y = 0, 4
P2x, P2y = 2, 0


# 定义隐式方程
def implicit_eq(x, y):
    return x ** 2 + y ** 2 - (P0x + 2 * (P1x - P0x) * 0.5 + (P0x - 2 * P1x + P2x) * 0.5 ** 2) ** 2 \
        - (P0y + 2 * (P1y - P0y) * 0.5 + (P0y - 2 * P1y + P2y) * 0.5 ** 2) ** 2


# 生成网格点
x = np.linspace(-5, 5, 200)
y = np.linspace(-5, 5, 200)
X, Y = np.meshgrid(x, y)

# 计算隐式方程值
Z = implicit_eq(X, Y)
print(Z)
# 绘制等值线
plt.contour(X, Y, Z, levels=[0], colors='b')
plt.scatter([P0x, P1x, P2x], [P0y, P1y, P2y], color='r')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Implicit Equation of Quadratic Bezier Curve')
plt.grid(True)
plt.axis('equal')
plt.show()
