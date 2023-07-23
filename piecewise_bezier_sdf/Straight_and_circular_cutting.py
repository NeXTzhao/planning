import math

import matplotlib.pyplot as plt

from quadrilateral_sdf import *


# Function to calculate f value
def f_fun(x, y, x1, y1, x2, y2):
    return ((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


# Function to calculate t value
def t_fun(x, y, x1, y1, x2, y2):
    d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    xc = (x1 + x2) / 2
    yc = (y1 + y2) / 2
    return 1 / d * (d ** 2 / 4 - ((x - xc) ** 2 + (y - yc) ** 2))


# Function to calculate t, f, and Z values
def h(x, y, x1, y1, x2, y2):
    P0 = np.array([5, 3])
    P1 = np.array([3, 5])
    P2 = np.array([1, 3])
    P3 = np.array([4, 2])
    t = create_rectangle_sdf(-50, 50, 100, P0, P1, P2, P3)
    # t = t_fun(x, y, x1, y1, x2, y2)
    f = f_fun(x, y, x1, y1, x2, y2)
    return t, f, np.sqrt(f ** 2 + (np.sqrt(t ** 2 + f ** 4) - t) ** 2 * 0.25)
    # return t, f, f + t - np.sqrt(f ** 2 + t ** 2)
    # return t, f, np.sqrt(f ** 2 + (np.abs(t) - t) ** 2 * 0.25)


# Set the line segment endpoints
x1, y1, x2, y2 = 0.0, 0.0, 1.0, 1.0

# Generate coordinate grid
x = np.linspace(-150, 150, 100)
y = np.linspace(-150, 150, 100)
X, Y = np.meshgrid(x, y)

# Calculate t, f, and Z values
t, f, Z = h(X, Y, x1, y1, x2, y2)

# Set a common color map for all subplots
cmap = 'coolwarm'

# Create the figure and axes
fig, ax = plt.subplots(1, 3, figsize=(15, 5))

# Plot the contour plots and store the contour object for adding color bars
contour1 = ax[0].contourf(X, Y, t, cmap=cmap)
contour2 = ax[1].contourf(X, Y, f, cmap=cmap)
contour3 = ax[2].contourf(X, Y, Z, cmap=cmap)

# Add color bars
cbar1 = fig.colorbar(contour1, ax=ax[0])
cbar2 = fig.colorbar(contour2, ax=ax[1])
cbar3 = fig.colorbar(contour3, ax=ax[2])

# Set titles and axis labels
ax[0].set_title('Trim Function')
ax[0].set_xlabel('X')
ax[0].set_ylabel('Y')

ax[1].set_title('Line Function')
ax[1].set_xlabel('X')
ax[1].set_ylabel('Y')

ax[2].set_title('Trimming Operation')
ax[2].set_xlabel('X')
ax[2].set_ylabel('Y')

# Adjust the layout and spacing between subplots
plt.tight_layout()

# Show the plot
plt.show()
