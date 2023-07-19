""" Python implementation of
    Algorithm for Automatically Fitting Digitized Curves
    by Philip J. Schneider
    "Graphics Gems", Academic Press, 1990
"""
from numpy import *

import bezier


# Fit one (ore more) Bezier curves to a set of points
def fitCurve(points, maxError):
    leftTangent = normalize(points[1] - points[0])
    rightTangent = normalize(points[-2] - points[-1])
    return fitCubic(points, leftTangent, rightTangent, maxError)


def find_curvature_threshold_index(curve, threshold):
    print('len()curve :', len(curve))
    for i in range(1, len(curve) - 1):
        # print('curve :', curve[i - 1])
        curvature = compute_curvature(curve[i - 1], curve[i], curve[i + 1])
        print('kappa = ', curvature)
        if curvature > 0.1:
            print('i = ', i)
            return i

    # 如果没有满足阈值条件的点，则返回 -1 表示未找到
    return -1


def fitCubic(points, leftTangent, rightTangent, error):
    bezier_num = []

    # Use heuristic if region only has two points in it
    if len(points) == 2:
        dist = linalg.norm(points[0] - points[1]) / 3.0
        bezCurve = [points[0], points[0] + leftTangent * dist, points[1] + rightTangent * dist, points[1]]
        bezier_num.append(bezier_curve(bezCurve, len(points)))
        return bezier_num

    # Parameterize points, and attempt to fit curve
    u = chordLengthParameterize(points)
    bezCurve = generateBezier(points, u, leftTangent, rightTangent)
    curve = bezier_curve(bezCurve, len(points))
    splitPoint = find_curvature_threshold_index(curve, 0.15)
    if splitPoint != -1:
        bezier_num.append(curve[:splitPoint + 1])
        remainPoint = points[splitPoint:]
    else:
        bezier_num.append(curve)
        return bezier_num
    # Compute left tangent outside the loop
    leftTangent1 = normalize(remainPoint[1] - remainPoint[0])

    while len(remainPoint) >= 2:
        u1 = chordLengthParameterize(remainPoint)
        bezCurve1 = generateBezier(remainPoint, u1, leftTangent1, rightTangent)
        curve1 = bezier_curve(bezCurve1, len(remainPoint))
        splitPoint1 = find_curvature_threshold_index(curve1, 0.15)
        if splitPoint1 != -1:
            bezier_num.append(curve1[:splitPoint1 + 1])
            remainPoint = remainPoint[splitPoint1:]
        else:
            bezier_num.append(curve1)
            return bezier_num

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


def bezier_curve(control_points, num_points):
    n = len(control_points) - 1
    t = linspace(0, 1, num_points)
    curve_points = []

    for i in range(num_points):
        point = zeros(2)
        for j in range(n + 1):
            point += control_points[j] * binomial_coefficient(n, j) * (1 - t[i]) ** (n - j) * t[i] ** j
        curve_points.append(point)

    return curve_points


def binomial_coefficient(n, k):
    return math.factorial(n) / (math.factorial(k) * math.factorial(n - k))


def calculate_max_error(control_points, num_points):
    curve_points = bezier_curve(control_points, num_points)

    max_error = 0.0
    max_error_index = None
    for i, original_point in enumerate(control_points):
        fitted_point = curve_points[i]
        error = linalg.norm(original_point - fitted_point)
        if error > max_error:
            max_error = error
            max_error_index = i

    last_curve_point = curve_points[-1]

    return max_error, max_error_index, last_curve_point


def generateBezier(points, parameters, leftTangent, rightTangent):
    bezCurve = [points[0], None, None, points[-1]]

    # compute the A's
    A = zeros((len(parameters), 2, 2))
    for i, u in enumerate(parameters):
        A[i][0] = leftTangent * 3 * (1 - u) ** 2 * u
        A[i][1] = rightTangent * 3 * (1 - u) * u ** 2

    # Create the C and X matrices
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

    # If alpha negative, use the Wu/Barsky heuristic (see text) */
    # (if alpha is 0, you get coincident control points that lead to
    # divide by zero in any subsequent NewtonRaphsonRootFind() call. */
    segLength = linalg.norm(points[0] - points[-1])
    epsilon = 1.0e-6 * segLength
    if alpha_l < epsilon or alpha_r < epsilon:
        # fall back on standard (probably inaccurate) formula, and subdivide further if needed.
        bezCurve[1] = bezCurve[0] + leftTangent * (segLength / 3.0)
        bezCurve[2] = bezCurve[3] + rightTangent * (segLength / 3.0)

    else:
        # First and last control points of the Bezier curve are
        # positioned exactly at the first and last data points
        # Control points 1 and 2 are positioned an alpha distance out
        # on the tangent vectors, left and right, respectively
        bezCurve[1] = bezCurve[0] + leftTangent * alpha_l
        bezCurve[2] = bezCurve[3] + rightTangent * alpha_r

    return bezCurve


def reparameterize(bezier, points, parameters):
    return [newtonRaphsonRootFind(bezier, point, u) for point, u in zip(points, parameters)]


def newtonRaphsonRootFind(bez, point, u):
    """
       Newton's root finding algorithm calculates f(x)=0 by reiterating
       x_n+1 = x_n - f(x_n)/f'(x_n)

       We are trying to find curve parameter u for some point p that minimizes
       the distance from that point to the curve. Distance point to curve is d=q(u)-p.
       At minimum distance the point is perpendicular to the curve.
       We are solving
       f = q(u)-p * q'(u) = 0
       with
       f' = q'(u) * q'(u) + q(u)-p * q''(u)

       gives
       u_n+1 = u_n - |q(u_n)-p * q'(u_n)| / |q'(u_n)**2 + q(u_n)-p * q''(u_n)|
    """
    d = bezier.q(bez, u) - point
    numerator = (d * bezier.qprime(bez, u)).sum()
    denominator = (bezier.qprime(bez, u) ** 2 + d * bezier.qprimeprime(bez, u)).sum()

    if denominator == 0.0:
        return u
    else:
        return u - numerator / denominator


def chordLengthParameterize(points):
    u = [0.0]
    for i in range(1, len(points)):
        u.append(u[i - 1] + linalg.norm(points[i] - points[i - 1]))

    for i, _ in enumerate(u):
        u[i] = u[i] / u[-1]

    return u


def computeMaxError(points, bez, parameters):
    maxDist = 0.0
    splitPoint = len(points) / 2
    for i, (point, u) in enumerate(zip(points, parameters)):
        dist = linalg.norm(bezier.q(bez, u) - point) ** 2
        if dist > maxDist:
            maxDist = dist
            splitPoint = i

    return maxDist, splitPoint


def normalize(v):
    return v / linalg.norm(v)
