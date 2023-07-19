import random

import matplotlib.pyplot as plt


class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def check_turn_right(p1, p2, p3):
    vec1 = (p2.x - p1.x, p2.y - p1.y)
    vec2 = (p3.x - p2.x, p3.y - p2.y)
    return vec1[0] * vec2[1] < vec2[0] * vec1[1]


def cal_upper_hull(inp_points):
    cmp = lambda p: (p.x, p.y)
    inp_points = sorted(inp_points, key=cmp)
    res = [inp_points[0], inp_points[1]]
    i = 2
    while i < len(inp_points):
        res.append(inp_points[i])
        cur_end_index = len(res) - 1
        while len(res) > 2 and not check_turn_right(res[cur_end_index - 2], res[cur_end_index - 1], res[cur_end_index]):
            res.pop(cur_end_index - 1)
            cur_end_index = len(res) - 1
        i += 1
    return res


def cal_lower_hull(inp_points):
    cmp = lambda p: (p.x, p.y)
    inp_points = sorted(inp_points, key=cmp, reverse=True)
    res = [inp_points[0], inp_points[1]]
    i = 2
    while i < len(inp_points):
        res.append(inp_points[i])
        cur_end_index = len(res) - 1
        while len(res) > 2 and not check_turn_right(res[cur_end_index - 2], res[cur_end_index - 1], res[cur_end_index]):
            res.pop(cur_end_index - 1)
            cur_end_index = len(res) - 1
        i += 1
    return res


def plot_points_and_hull(points, upper_hull, lower_hull):
    x = [point.x for point in points]
    y = [point.y for point in points]

    upper_hull_x = [point.x for point in upper_hull]
    upper_hull_y = [point.y for point in upper_hull]

    lower_hull_x = [point.x for point in lower_hull]
    lower_hull_y = [point.y for point in lower_hull]

    plt.scatter(x, y, color='blue', label='Points')
    plt.plot(upper_hull_x, upper_hull_y, color='red', label='Upper Convex Hull')
    plt.plot(lower_hull_x, lower_hull_y, color='green', label='Lower Convex Hull')

    # for i, point in enumerate(points):
    #     plt.annotate(f'{i}', (point.x, point.y), textcoords="offset points", xytext=(0,10), ha='center')

    for i, point in enumerate(upper_hull):
        plt.annotate(f'{i}', (point.x, point.y), textcoords="offset points", xytext=(0,10), ha='center')

    for i, point in enumerate(lower_hull):
        plt.annotate(f'{i}', (point.x, point.y), textcoords="offset points", xytext=(0,10), ha='center')

    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Points and Convex Hull')
    plt.legend()
    plt.axis('equal')
    plt.show()



# 设置随机种子
random.seed(42)

# 生成随机点集
num_points = 30
points = [Point2D(random.uniform(0, 10), random.uniform(0, 10)) for _ in range(num_points)]

points[1].x = 0.26535969683863625-1e-5
points[1].y = 1.988376506866485+2.0

points[8].x = 0.26535969683863625
points[8].y = 1.988376506866485+4.0

x = [point.x for point in points]
y = [point.y for point in points]

# print('x : ', x)
# print('y : ', y)
upper_hull = cal_upper_hull(points)
lower_hull = cal_lower_hull(points)

plot_points_and_hull(points, upper_hull, lower_hull)


