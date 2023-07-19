import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splrep, splev


def fit_bspline_curve(points, degree, num_control_points, num_samples):
    # Generate a parameterization for the points
    t = np.linspace(0, 1, len(points))

    # Fit the B-spline curve to the points
    tck = splrep(t, points[:, 0], k=degree, s=0)

    # Evaluate the B-spline curve at the specified number of samples
    t_eval = np.linspace(0, 1, num_samples)
    curve_points_x = splev(t_eval, tck)

    tck = splrep(t, points[:, 1], k=degree, s=0)

    # Evaluate the B-spline curve at the specified number of samples
    curve_points_y = splev(t_eval, tck)

    curve_points = np.column_stack((curve_points_x, curve_points_y))

    return curve_points


# Generate some random discrete points
np.random.seed(0)
num_points = 10
points = np.random.rand(num_points, 2) * 10

# Fit a B-spline curve to the points
degree = 3  # Degree of the B-spline curve
num_control_points = 6  # Number of control points to fit
num_samples = 100  # Number of samples to evaluate the B-spline curve

curve_points = fit_bspline_curve(points, degree, num_control_points, num_samples)

# Plot the original points and the fitted B-spline curve
plt.figure(figsize=(8, 6))
plt.scatter(points[:, 0], points[:, 1], color='red', label='Original Points')
plt.plot(curve_points[:, 0], curve_points[:, 1], 'b-', label='Fitted B-spline Curve')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Fitted B-spline Curve')
plt.legend()
plt.grid(True)
plt.show()
