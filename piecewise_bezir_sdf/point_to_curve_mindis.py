import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


def bezier(t, p0, p1, p2, p3):
    x = (1 - t) ** 3 * p0[0] + 3 * t * (1 - t) ** 2 * p1[0] + 3 * t ** 2 * (1 - t) * p2[0] + t ** 3 * p3[0]
    y = (1 - t) ** 3 * p0[1] + 3 * t * (1 - t) ** 2 * p1[1] + 3 * t ** 2 * (1 - t) * p2[1] + t ** 3 * p3[1]
    return x, y


def distance_squared(p1, p2):
    return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2


def find_min_distance_to_bezier(Q, P0, P1, P2, P3):
    # Define the distance function to be minimized
    def distance_to_bezier(t):
        B = bezier(t, P0, P1, P2, P3)
        return distance_squared(B, Q)

    # Perform the optimization to find t that minimizes the distance
    result = minimize(distance_to_bezier, x0=0.5, bounds=[(0, 1)])
    t_min_distance = result.x[0]
    B_min_distance = bezier(t_min_distance, P0, P1, P2, P3)
    return B_min_distance, distance_squared(B_min_distance, Q)


# Example usage
Q = (1, 5)  # Point for which we want to find the minimum distance to the Bezier curve
P0 = (1, 1)
P1 = (2, 8)
P2 = (6, 1)
P3 = (8, 5)

B, min_distance_squared = find_min_distance_to_bezier(Q, P0, P1, P2, P3)
min_distance = min_distance_squared ** 0.5  # Square root to get the actual distance

# Plotting the Bezier curve, the given point, and the nearest point
t_values = np.linspace(0, 1, 100)
curve_points = [bezier(t, P0, P1, P2, P3) for t in t_values]
curve_x, curve_y = zip(*curve_points)

plt.figure(figsize=(8, 6))
plt.plot(curve_x, curve_y, label="Bezier Curve", color="blue")
plt.scatter([Q[0]], [Q[1]], color="red", label="Given Point Q")
plt.scatter([B[0]], [B[1]], color="green", label="Nearest Point on Bezier Curve")
plt.plot([Q[0], B[0]], [Q[1], B[1]], "--", color="gray", label="Minimum Distance")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Visualization of Three-Order Bezier Curve and Nearest Point")
plt.legend()
plt.grid(True)
plt.show()
