import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

x = [0, 1, 2, 3, 5, 6, 8,  11]
y = [4, 3, 4, 6, 7, 5, 10, 1]
x_h = [0, 1, 1, 2, 3, 5, 5, 6, 6,  8, 8, 11, 11]
y_h = [4, 3, 0, 4, 6, 7, 0, 5, 0, 10, 0, 1, 0]
x_new = np.linspace(0, 11, 100)

f_linear = interpolate.interp1d(x, y, kind = 'linear')
f_quadratic = interpolate.interp1d(x, y, kind = 'quadratic')
f_cubic = interpolate.interp1d(x, y, kind = 'cubic')
f_hermite = interpolate.KroghInterpolator(x, y)
f_pchip = interpolate.PchipInterpolator(x, y)

plt.plot(x, y, "o", label="data")
plt.plot(x_new, f_linear(x_new), label="linear")
plt.plot(x_new, f_quadratic(x_new), label="quadratic")
plt.plot(x_new, f_cubic(x_new), label="cubic")
plt.plot(x_new, f_hermite(x_new), label="hermite")
plt.plot(x_new, f_pchip(x_new), label="pchip")
plt.ylabel("y")
plt.xlabel("x")
plt.legend()
plt.show()