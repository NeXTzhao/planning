import numpy as np
from math import *
import matplotlib.pyplot as plt

def frenet_to_cartesian1D(rs, rx, ry, rtheta, s_condition, d_condition):
    if fabs(rs - s_condition[0])>= 1.0e-6:
        print("The reference point s and s_condition[0] don't match")
        
    cos_theta_r = cos(rtheta)
    sin_theta_r = sin(rtheta)
    
    x = rx - sin_theta_r * d_condition[0]
    y = ry + cos_theta_r * d_condition[0]    
    return x, y

theta = np.linspace(0, np.pi, 10)
x = 5.0*np.cos(theta)
y = 5.0*np.sin(theta)
s = theta*5.0

fx = np.poly1d(np.polyfit(s,x,5))
dfx = fx.deriv()

fy = np.poly1d(np.polyfit(s,y,5))
dfy = fy.deriv()

snew = np.linspace(0, 5*np.pi, 1000)

newx = fx(snew)
newy = fy(snew)

left_bound = []
right_bound = []

for i in range(1000):
    rs = snew[i]
    rx = fx(rs)
    ry = fy(rs)
    
    drx = dfx(rs)
    dry = dfy(rs)
    
    rtheta = atan2(dry, drx)
    
    l_s_condition = np.array([rs])
    l_d_condition = np.array([1.5])
    lx, ly = frenet_to_cartesian1D(rs, rx, ry, rtheta, l_s_condition, l_d_condition)
    left_bound.append(np.array([lx, ly]))
    
    r_s_condition = np.array([rs])
    r_d_condition = np.array([-1.5])
    rx, ry = frenet_to_cartesian1D(rs, rx, ry, rtheta, r_s_condition, r_d_condition)   
    right_bound.append(np.array([rx, ry]))
left_bound = np.array(left_bound)
right_bound = np.array(right_bound)

plt.plot(newx, newy, 'y')
plt.plot(left_bound[:,0],left_bound[:,1], 'b')
plt.plot(right_bound[:,0],right_bound[:,1], 'b')
plt.show()
