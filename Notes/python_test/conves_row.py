import random

import matplotlib.pyplot as plt


class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Line2D:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2


def check_turn_right(p1, p2, p3):
    vec1 = (p2.x - p1.x, p2.y - p1.y)
    vec2 = (p3.x - p2.x, p3.y - p2.y)
    return vec1[0] * vec2[1] < vec2[0] * vec1[1]


def slow_convex_hull(p):
    e = []
    for i in range(len(p)):
        for j in range(len(p)):
            if i != j:
                valid = True
                for k in range(len(p)):
                    if k != i and k != j:
                        if check_turn_right(p[i], p[j], p[k]):
                            valid = False
                            break
                if valid:
                    e.append(Line2D(p[i], p[j]))
    l = []
    for edge in e:
        l.append(edge.p1)
    l.append(e[-1].p2)
    return l

# 设置随机种子
random.seed(42)

# 生成随机点集
num_points = 30
points = [Point2D(random.uniform(0, 10), random.uniform(0, 10)) for _ in range(num_points)]

points[1].x = 0.26535969683863625+10
points[1].y = 1.988376506866485 + 2.0

points[8].x = 0.26535969683863625+5
points[8].y = 1.988376506866485 + 4.0

# 计算凸包
hull = slow_convex_hull(points)

# 提取点集的坐标
x = [point.x for point in points]
y = [point.y for point in points]

# 提取凸包的坐标
hull_x = [point.x for point in hull]
hull_y = [point.y for point in hull]

# 绘制点集和凸包
plt.scatter(x, y, color='blue', label='Points')
plt.plot(hull_x, hull_y, color='red', label='Convex Hull')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Points and Convex Hull')
plt.legend()
plt.show()
