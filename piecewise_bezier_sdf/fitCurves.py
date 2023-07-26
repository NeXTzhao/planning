from numpy import *


def q(ctrlPoly, t):
    return (1.0 - t) ** 3 * ctrlPoly[0] + 3 * (1.0 - t) ** 2 * t * ctrlPoly[1] + 3 * (1.0 - t) * t ** 2 * ctrlPoly[
        2] + t ** 3 * ctrlPoly[3]


# evaluates cubic bezier first derivative at t, return point
def qprime(ctrlPoly, t):
    return 3 * (1.0 - t) ** 2 * (ctrlPoly[1] - ctrlPoly[0]) + 6 * (1.0 - t) * t * (
            ctrlPoly[2] - ctrlPoly[1]) + 3 * t ** 2 * (ctrlPoly[3] - ctrlPoly[2])


# evaluates cubic bezier second derivative at t, return point
def qprimeprime(ctrlPoly, t):
    return 6 * (1.0 - t) * (ctrlPoly[2] - 2 * ctrlPoly[1] + ctrlPoly[0]) + 6 * (t) * (
            ctrlPoly[3] - 2 * ctrlPoly[2] + ctrlPoly[1])


def fitCurve(points, maxError):
    leftTangent = normalize(points[1] - points[0])
    rightTangent = normalize(points[-2] - points[-1])
    control_point = fitCubic(points, leftTangent, rightTangent, maxError)
    curves = get_fit_curves(control_point, points)
    return control_point, curves


def get_fit_curves(control_point, points):
    curve = []
    for con in control_point:
        curve.append(bezier_curve(con, len(points) - 1))
    return curve


def fitCubic(points, leftTangent, rightTangent, error):
    if len(points) == 2:
        dist = linalg.norm(points[0] - points[1]) / 3.0
        bezCurve = [points[0], points[0] + leftTangent * dist, points[1] + rightTangent * dist, points[1]]
        return [bezCurve]

    u = chordLengthParameterize(points)
    bezCurve = generateBezier(points, u, leftTangent, rightTangent)
    maxError, splitPoint = computeMaxError(points, bezCurve, u)

    if maxError < error:
        return [bezCurve]

    if maxError < error ** 2:
        for i in range(20):
            uPrime = reparameterize(bezCurve, points, u)
            bezCurve = generateBezier(points, uPrime, leftTangent, rightTangent)
            maxError, splitPoint = computeMaxError(points, bezCurve, uPrime)
            if maxError < error:
                return [bezCurve]
            u = uPrime

    beziers = []
    centerTangent = normalize(points[splitPoint - 1] - points[splitPoint + 1])

    beziers += fitCubic(points[:splitPoint + 1], leftTangent, centerTangent, error)
    beziers += fitCubic(points[splitPoint:], -centerTangent, rightTangent, error)

    return beziers


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

        tmp = point - q([points[0], points[0], points[-1], points[-1]], u)

        X[0] += dot(A[i][0], tmp)
        X[1] += dot(A[i][1], tmp)

    det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1]
    det_C0_X = C[0][0] * X[1] - C[1][0] * X[0]
    det_X_C1 = X[0] * C[1][1] - X[1] * C[0][1]

    alpha_l = 0.0 if det_C0_C1 == 0 else det_X_C1 / det_C0_C1
    alpha_r = 0.0 if det_C0_C1 == 0 else det_C0_X / det_C0_C1

    segLength = linalg.norm(points[0] - points[-1])
    epsilon = 1.0e-6 * segLength
    if alpha_l < epsilon or alpha_r < epsilon:
        bezCurve[1] = bezCurve[0] + leftTangent * (segLength / 3.0)
        bezCurve[2] = bezCurve[3] + rightTangent * (segLength / 3.0)

    else:
        bezCurve[1] = bezCurve[0] + leftTangent * alpha_l
        bezCurve[2] = bezCurve[3] + rightTangent * alpha_r

    return bezCurve


def reparameterize(bezier, points, parameters):
    return [newtonRaphsonRootFind(bezier, point, u) for point, u in zip(points, parameters)]


def newtonRaphsonRootFind(bez, point, u):
    d = q(bez, u) - point

    numerator = (d * qprime(bez, u)).sum()
    denominator = (qprime(bez, u) ** 2 + d * qprimeprime(bez, u)).sum()

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
        dist = linalg.norm(q(bez, u) - point) ** 2
        if dist > maxDist:
            maxDist = dist
            splitPoint = i
    return maxDist, splitPoint


def normalize(v):
    return v / linalg.norm(v)


def distance_squared(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
