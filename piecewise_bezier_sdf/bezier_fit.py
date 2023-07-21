import math
import numpy as np
import matplotlib.pyplot as plt


def distance(p1, p2):
    return math.sqrt(((p1 - p2) ** 2).sum())


class bezier:
    def __init__(self, points, n=2):
        """
        Args:
            points: np.array, shape=[n, 2], points是曲线经过的点
                （self.P是贝塞尔曲线中的顶点，顶点数为阶数+1，
                  self.points是曲线上的点，其个数要不小于顶点数）
            n: 阶数, 这里为2或3(这里我只支持二阶和三阶，其实也可以扩展为更一般化，添加不同的T即可)
        """
        assert len(points) >= n + 1
        self.n = n
        if n == 2:
            self.T = bezier.T_2
            self.t = bezier.t_2
        elif n == 3:
            self.T = bezier.T_3
            self.t = bezier.t_3
        self.__points = points
        self.solve()

    def __call__(self, t):
        t = self.t(t).reshape(1, -1)
        return bezier.calc(self.P, t)

    @property
    def points(self):
        return self.__points

    @points.setter
    def points(self, points):
        self.__points = points
        self.solve()

    def solve(self):
        ts = [0]
        ts += [distance(p1, p2) for p1, p2 in zip(self.__points[0:-1], self.__points[1:])]
        ts_sum = sum(ts)

        for i in range(1, len(ts)):
            ts[i] = ts[i - 1] + ts[i]
        ts = [t / ts_sum for t in ts]
        T = self.T(ts)
        assert T.shape[0] >= self.n + 1
        if T.shape[0] == self.n + 1:
            self.P = np.dot(np.linalg.inv(T), self.__points)
        else:
            TtT_inv = np.linalg.inv(np.dot(T.T, T))
            TtB = np.dot(T.T, self.__points)
            self.P = np.dot(TtT_inv, TtB)

    @staticmethod
    def calc(P, t):
        """
        P: 顶点, shape=[d, 2]
        T: shape=[1, d]
        """
        assert P.shape[0] == t.shape[1]
        return np.dot(t, P)

    @staticmethod
    def t_2(t):
        return np.asarray([(1 - t) ** 2, 2 * t * (1 - t), t ** 2])

    @staticmethod
    def T_2(ts):
        assert len(ts) >= 3
        T = [bezier.t_2(t) for t in ts]
        return np.asarray(T)

    @staticmethod
    def t_3(t):
        return np.asarray([(1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t ** 2 * (1 - t), t ** 3])

    @staticmethod
    def T_3(ts):
        assert len(ts) >= 4
        T = [bezier.t_3(t) for t in ts]
        return np.asarray(T)


if __name__ == '__main__':
    x = []
    y = []

    tdata = np.linspace(0, np.pi, 30)
    for t in tdata:
        x.append(100 * np.cos(t))
        y.append(100 * np.sin(t))

    data = []
    for x_val, y_val in zip(x, y):
        data.append([x_val, y_val])

    points = np.array(data)

    # points = np.asarray([[100, 315], [200, 380], [300, 495], [400, 660], [500, 875]])
    curve = bezier(points, 3)
    plt.scatter(points[:, 0], points[:, 1], c='red', label='Original Points')

    x = []
    y = []
    tn = np.linspace(0, 1, 50)
    for t in tn:
        x_values, y_values = curve(t).reshape(-1)
        x.append(x_values)
        y.append(y_values)

    # 绘制贝塞尔曲线
    # plt.scatter(x, y, c='blue', label='Bezier Curve')
    plt.plot(x, y, c='blue', label='Bezier Curve')

    plt.legend()
    plt.show()
