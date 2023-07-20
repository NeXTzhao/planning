from numpy import *
from scipy.optimize import minimize
import bezier


def fitCurve(points, maxError):
    leftTangent = normalize(points[1] - points[0])
    rightTangent = normalize(points[-2] - points[-1])
    return fitCubic(points, leftTangent, rightTangent, maxError)


def fitCubic(points, leftTangent, rightTangent, error):
    bezier_num = []
    u = chordLengthParameterize(points)
    bezCurve = generateBezier(points, u, leftTangent, rightTangent)
    maxError, splitPoint_error = computeMaxError(points, bezCurve, u, error)
    print("maxError:", maxError, "splitPoint_error:", splitPoint_error)
    curve = bezier_curve(bezCurve, len(points) - 1)
    splitPoint_kappa = find_index_exceed_kappa(curve, 0.015)
    if splitPoint_kappa > splitPoint_error:
        splitPoint = splitPoint_kappa
        print('kappa_index = ', splitPoint)
    else:
        splitPoint = splitPoint_error
        print('error_index = ', splitPoint)
    # splitPoint = splitPoint_kappa
    remainPoint = points[splitPoint:]
    print('rem = ', len(remainPoint))
    print('points_len = ', len(points)-1)
    if splitPoint == len(points):
        remainPoint = points

    if len(remainPoint) > 2:
        bezier_num.append(curve[:splitPoint])
        # leftTangent = normalize(curve[splitPoint + 1] - curve[splitPoint - 1])
        leftTangent = normalize(remainPoint[1] - remainPoint[0])
        bezier_num += fitCubic(remainPoint, leftTangent, rightTangent, error)
    else:
        bezier_num.append(curve)

    return bezier_num


def compute_curvature(point1, point2, point3):
    # 计算三个点形成的曲率
    x1, y1 = point1
    x2, y2 = point2
    x3, y3 = point3

    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x3 - x2
    dy2 = y3 - y2

    numerator = dx1 * dy2 - dx2 * dy1
    denominator = (dx1 ** 2 + dy1 ** 2) ** 1.5
    curvature = numerator / denominator

    return curvature


def compute_curve_curvature(curve):
    num_points = len(curve)
    curvatures = []

    for i in range(num_points):
        if i == 0:
            curvature = compute_curvature(curve[i], curve[i + 1], curve[i + 2])
        elif i == num_points - 1:
            curvature = compute_curvature(curve[i - 2], curve[i - 1], curve[i])
        else:
            curvature = compute_curvature(curve[i - 1], curve[i], curve[i + 1])

        curvatures.append(curvature)

    return curvatures


def find_index_exceed_kappa(curve, threshold):
    curvatures = compute_curve_curvature(curve)

    for i, kappa in enumerate(curvatures):
        # print('kappa = ', kappa)
        if kappa > threshold:
            return i
    return -1


def calculate_max_error(point, curve_points):
    max_error = 0.0
    max_error_index = None
    for i, original_point in enumerate(point):
        error_x = curve_points[i][0] - point[i][0]
        error_y = curve_points[i][1] - point[i][1]
        error = error_x ** 2 + error_y ** 2
        if error > max_error:
            max_error = error
            max_error_index = i

    return max_error, max_error_index


def find_curvature_threshold_index(curve, threshold):
    print('len()curve :', len(curve))
    for i in range(1, len(curve) - 1):
        # print('curve :', curve[i - 1])
        curvature = compute_curvature(curve[i - 1], curve[i], curve[i + 1])
        # print('kappa = ', curvature)
        if curvature > 0.1:
            print('i = ', i)
            return i

    # 如果没有满足阈值条件的点，则返回 -1 表示未找到
    return -1


def bezier_curve(control_points, num_points):
    n = len(control_points) - 1
    t = linspace(0, 1, num_points)
    curve_points = zeros((num_points, 2))

    for i in range(num_points):
        for j in range(n + 1):
            curve_points[i] += control_points[j] * binomial_coefficient(n, j) * (1 - t[i]) ** (n - j) * t[i] ** j

    return curve_points


def binomial_coefficient(n, k):
    return math.factorial(n) / (math.factorial(k) * math.factorial(n - k))


# Use least-squares method to find Bezier control points for region.
def generateBezier(points, parameters, leftTangent, rightTangent):
    bezCurve = [points[0], None, None, points[-1]]

    A = zeros((len(parameters), 2, 2))
    for i, u in enumerate(parameters):
        A[i][0] = leftTangent * 3 * (1 - u) ** 2 * u
        A[i][1] = rightTangent * 3 * (1 - u) * u ** 2

    C = zeros((2, 2))
    X = zeros(2)

    for i, (point, u) in enumerate(zip(points, parameters)):
        C[0][0] += dot(A[i][0], A[i][0])
        C[0][1] += dot(A[i][0], A[i][1])
        C[1][0] += dot(A[i][0], A[i][1])
        C[1][1] += dot(A[i][1], A[i][1])

        tmp = point - bezier.q([points[0], points[0], points[-1], points[-1]], u)

        X[0] += dot(A[i][0], tmp)
        X[1] += dot(A[i][1], tmp)

    # Compute the determinants of C and X
    det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1]
    det_C0_X = C[0][0] * X[1] - C[1][0] * X[0]
    det_X_C1 = X[0] * C[1][1] - X[1] * C[0][1]

    # Finally, derive alpha values
    alpha_l = 0.0 if det_C0_C1 == 0 else det_X_C1 / det_C0_C1
    alpha_r = 0.0 if det_C0_C1 == 0 else det_C0_X / det_C0_C1

    segLength = linalg.norm(points[0] - points[-1])
    epsilon = 1.0e-6 * segLength
    if alpha_l < epsilon or alpha_r < epsilon:
        # fall back on standard (probably inaccurate) formula, and subdivide further if needed.
        bezCurve[1] = bezCurve[0] + leftTangent * (segLength / 3.0)
        bezCurve[2] = bezCurve[3] + rightTangent * (segLength / 3.0)

    else:
        bezCurve[1] = bezCurve[0] + leftTangent * alpha_l
        bezCurve[2] = bezCurve[3] + rightTangent * alpha_r

    return bezCurve


def generateBezier1(points, parameters, leftTangent, rightTangent):
    bezCurve = [points[0], None, points[-1]]

    segLength = linalg.norm(points[0] - points[-1])
    epsilon = 1.0e-6 * segLength

    alpha_l = linalg.norm(leftTangent) / segLength
    alpha_r = linalg.norm(rightTangent) / segLength

    if alpha_l < epsilon or alpha_r < epsilon:
        bezCurve[1] = bezCurve[0] + leftTangent * (segLength / 3.0)
    else:
        bezCurve[1] = bezCurve[0] + leftTangent * alpha_l

    return bezCurve


def reparameterize(bezier, points, parameters):
    return [newtonRaphsonRootFind(bezier, point, u) for point, u in zip(points, parameters)]


def newtonRaphsonRootFind(bez, point, u):
    d = bezier.q(bez, u) - point

    numerator = (d * bezier.qprime(bez, u)).sum()
    denominator = (bezier.qprime(bez, u) ** 2 + d * bezier.qprimeprime(bez, u)).sum()

    if denominator == 0.0:
        return u
    else:
        return u - numerator / denominator


def chordLengthParameterize(points):
    # 初始化参数列表，第一个点的参数值为0.0
    u = [0.0]

    # 计算相邻点之间的弦长并累积得到每个点的参数值
    for i in range(1, len(points)):
        # linalg.norm(points[i] - points[i - 1]) 计算两个点之间的欧几里得距离，即弦长
        u.append(u[i - 1] + linalg.norm(points[i] - points[i - 1]))

    # 将参数值归一化到[0, 1]的范围内
    for i, _ in enumerate(u):
        u[i] = u[i] / u[-1]

    return u


def computeMaxError1(points, bez, parameters):
    maxDist = 0.0
    splitPoint = len(points) / 2

    # 对于给定的点和参数列表，遍历每个点及其对应的参数。
    for i, (point, u) in enumerate(zip(points, parameters)):
        # 使用贝塞尔曲线的控制点bez和参数u，在贝塞尔曲线上计算点P。
        # bezier.q(bez, u)函数贝塞尔曲线上求点的函数，

        # 计算贝塞尔曲线上的点P与实际点point之间的距离（平方距离）。
        dist = linalg.norm(bezier.q(bez, u) - point) ** 2

        # 如果当前距离dist大于之前的最大误差maxDist，则更新maxDist和splitPoint。
        if dist > maxDist:
            maxDist = dist
            splitPoint = i

    # 返回最大误差maxDist和对应的分割点splitPoint。
    return maxDist, splitPoint


def normalize(v):
    return v / linalg.norm(v)


def distance_squared(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def computeMaxError(points, bez, u, error):
    # 定义用于最小化的距离函数
    def distance_to_bezier(t, Q_point):
        B = bezier.q(bez, t)
        return distance_squared(B, Q_point)

    max_distance = 0
    max_distance_index = -1
    for i, point in enumerate(points):
        result = minimize(distance_to_bezier, x0=u[0], bounds=[(u[0], u[-1])], args=(point,))
        t_min_distance = result.x[0]
        B_min_distance = bezier.q(bez, t_min_distance, )
        dist = distance_squared(B_min_distance, point)

        # 如果当前点的距离更大，则更新最大距离和索引
        print('dist', i, '=', dist, ',max_distance', i, '=', max_distance, ',max_distance_index', i, '=',
              max_distance_index)

        if dist < error:
            if dist >= max_distance:
                max_distance = dist
                max_distance_index = i
        else:
            return max_distance, max_distance_index

        if max_distance < error:
            max_distance_index = i
    return max_distance, max_distance_index
