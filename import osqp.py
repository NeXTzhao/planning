# import numpy as np
# from scipy import sparse

# import osqp

# # Define problem data
# P = sparse.csc_matrix([[4, 1], [1, 2]])
# q = np.array([1, 1])
# A = sparse.csc_matrix([[1, 1], [1, 0], [0, 1]])
# l = np.array([1, 0, 0])
# u = np.array([1, 0.7, 0.7])

# # Create an OSQP object
# prob = osqp.OSQP()

# # Setup workspace and change alpha parameter
# prob.setup(P, q, A, l, u, alpha=1.0)

# # Solve problem
# res = prob.solve()
# print('x:\n',res.x[:] )
# print('y:\n',res.y[:] )



# import osqp
# import numpy as np
# import time
# from scipy.interpolate import *
# from scipy import sparse
# import matplotlib.pyplot as plt


# class PathOptimizer:
#     def __init__(self, points, ts, acc, d='2d'):
#         self.ctrl_points = points
#         self.ctrl_ts = ts
#         self.acc_max = acc
#         self.dimension = d

#         # 时间分配
#         self.seg_length = self.acc_max * self.ctrl_ts * self.ctrl_ts
#         self.total_length = 0
#         for i in range(self.ctrl_points.shape[0] - 1):
#             self.total_length += np.linalg.norm(self.ctrl_points[i + 1, :] - self.ctrl_points[i, :])
#         self.total_time = self.total_length/self.seg_length * self.ctrl_ts * 1.2

#         # 生成控制点B样条函数
#         time_list = np.linspace(0, self.total_time, self.ctrl_points.shape[0])
#         self.px_spline = InterpolatedUnivariateSpline(time_list, self.ctrl_points[:, 0])
#         self.py_spline = InterpolatedUnivariateSpline(time_list, self.ctrl_points[:, 1])

#         # 对B样条优化参考轨迹进行均匀采样
#         self.seg_time_list = np.arange(0, self.total_time, self.ctrl_ts)
#         self.px = self.px_spline(self.seg_time_list)
#         self.py = self.py_spline(self.seg_time_list)

#         # 优化问题权重参数
#         self.weight_pos = 5
#         self.weight_vel = 1
#         self.weight_acc = 1
#         self.weight_jerk = 0.1

#         start_time = time.perf_counter()
#         self.smooth_path = self.osqpSmooth()
#         print('OSQP in ' + self.dimension + ' Cost time: ' + str(time.perf_counter() - start_time))
#         pass

#     def osqpSmooth(self):
#         px0, py0 = self.px, self.py
#         vx0 = vy0 = vz0 = np.zeros(self.px.shape)
#         ax0 = ay0 = az0 = np.zeros(self.px.shape)

#         # x(0), x(1), ..., x(n - 1), x'(0), x'(1), ..., x'(n - 1), x"(0), x"(1), ..., x"(n - 1)
#         x0_total = np.hstack((px0, py0, vx0, vy0, ax0, ay0))


#         n = int(x0_total.shape[0] / 3)   # 每种级别状态有几个变量,即x共n个，x'共n个，x"也是n个
#         Q_x = self.weight_pos * np.eye(n)
#         Q_dx = self.weight_vel * np.eye(n)
#         Q_zeros = np.zeros((n, n))
#         w_ddl = self.weight_acc
#         w_dddl = self.weight_jerk

#         if n % 2 != 0:
#             print('x0 input error.')
#             return
#         n_part = int(n/2)   # 每部分有n_part个数据,即x_x(0), x_x(1), ..., x_x(n-1), x_y(0), x_y(1), ..., x_y(n-1), ...

#         Q_ddx_part = (w_ddl + 2 * w_dddl / self.ctrl_ts / self.ctrl_ts) * np.eye(n_part) \
#                      - 2 * w_dddl / self.ctrl_ts / self.ctrl_ts * np.eye(n_part, k=-1)
#         Q_ddx_part[0][0] = w_ddl + w_dddl / self.ctrl_ts / self.ctrl_ts
#         Q_ddx_part[n_part-1][n_part-1] = w_ddl + w_dddl / self.ctrl_ts / self.ctrl_ts

#         np_zeros = np.zeros(Q_ddx_part.shape)
#         Q_ddx_l = np.vstack((Q_ddx_part, np_zeros))
#         Q_ddx_r = np.vstack((np_zeros, Q_ddx_part))
#         Q_ddx = np.hstack((Q_ddx_l, Q_ddx_r))


#         Q_total = sparse.csc_matrix(np.block([[Q_x, Q_zeros, Q_zeros],
#                                               [Q_zeros, Q_dx, Q_zeros],
#                                               [Q_zeros, Q_zeros, Q_ddx]
#                                               ]))

#         p_total = - x0_total
#         p_total[:n] = self.weight_pos * p_total[:n]

#         # 动力学模型，所以是等式约束
#         # x(i+1) = x(i) + x'(i)△t
#         # x'(i+1) = x'(i) + x"(i)△t
#         AI_part = np.eye(n_part-1, n_part) - np.eye(n_part-1, n_part, k=1)  # 计算同阶变量之间插值 (n-1) x n维
#         AT_part = self.ctrl_ts * np.eye(n_part-1, n_part)   # 时间矩阵 (n-1) x n维
#         AZ_part = np.zeros([n_part-1, n_part])  # 全0矩阵 (n-1) x n维

#         # 起点为第一个点
#         A_init = np.zeros([4, x0_total.shape[0]])
#         A_init[0, 0] = A_init[1, n_part] = 1
#         A_init[2, n_part-1] = A_init[3, 2*n_part-1] = 1

#         A_l_init = A_u_init = np.array([x0_total[0], x0_total[n_part],
#                                         x0_total[n_part-1], x0_total[2*n_part-1]])

#         A_eq = sparse.csc_matrix(np.block([
#             [A_init],
#             [AI_part, AZ_part, AT_part, AZ_part, AZ_part, AZ_part],
#             [AZ_part, AI_part, AZ_part, AT_part, AZ_part, AZ_part],
#             [AZ_part, AZ_part, AI_part, AZ_part, AT_part, AZ_part],
#             [AZ_part, AZ_part, AZ_part, AI_part, AZ_part, AT_part]
#         ]))
#         A_leq = A_ueq = np.zeros(A_eq.shape[0])
#         A_leq[:4] = A_ueq[:4] = A_l_init


#         # Create an OSQP object
#         prob = osqp.OSQP()
#         prob.setup(Q_total, p_total, A_eq, A_leq, A_ueq, alpha=1.0)
#         res = prob.solve()
#         print('x:\n',res.x[:] )
#         print('y:\n',res.y[:] )
#         return res.x[:]


# class PathGenerator:
#     def __init__(self):
#         self.ctrl_points = np.array([[0.5, 0.5],
#                                      [0.5, 1],
#                                      [1.5, 1.],
#                                      [2., 2.],
#                                      [2.5, 2.5]])

#         self.ctrl_ts = 0.1  # 控制时间s
#         self.acc_max = 2    # 米/秒

#         self.ref_path = PathOptimizer(self.ctrl_points, self.ctrl_ts, self.acc_max)


# if __name__ == '__main__':
#     pg = PathGenerator()

#     fig = plt.figure()

#     ax = plt.axes()
#     ax.scatter(pg.ctrl_points[:, 0], pg.ctrl_points[:, 1], color='red')
#     ax.plot(pg.ref_path.px, pg.ref_path.py, color='gray')

#     seg_num = pg.ref_path.px.shape[0]
#     ax.plot(pg.ref_path.smooth_path[:seg_num], pg.ref_path.smooth_path[seg_num:2*seg_num], color='green')

#     plt.show()

import random

import matplotlib.pyplot as plt
import numpy as np
from scipy import sparse

import osqp

# 障碍物设置
obs = [[5,10,2,3],[15,20,-2,-0.5],[25,30,0,1]]#start_s,end_s,l_low,l_up

# obs = [[0,10,2,3],[15,20,-2,-0.5],[25,30,0,1]]#start_s,end_s,l_low,l_up
s_len = 50
delta_s = 0.1
n = int(s_len / delta_s)
x = np.linspace(0,s_len,n)
up_bound = [0]*(5*n + 3)
low_bound = [0]*(5*n + 3)
s_ref = [0]*3*n

dddl_bound = 0.01

####################边界提取################
l_bound = 5
for i in range(n):
    for j in range(len(obs)):
        if x[i] >= obs[j][0] and x[i] <= obs[j][1]:            
            low_ = obs[j][2]
            up_ = obs[j][3]         
            break
        else:
            up_ = l_bound
            low_ = -l_bound
    up_bound[i] = up_
    low_bound[i] = low_   
    s_ref[i] = 0.5*(up_ + low_)

for i in range(3*n,4*n):
    up_bound[i] = dddl_bound * delta_s *delta_s *delta_s/6
    low_bound[i] = -dddl_bound * delta_s *delta_s *delta_s/6  
for i in range(4*n,5*n):
    up_bound[i] = dddl_bound * delta_s *delta_s / 2
    low_bound[i] = -dddl_bound * delta_s *delta_s / 2



####################构造P和Q################
w_l = 0.005
w_dl = 1
w_ddl = 1
w_dddl = 0.1
eye_n = np.identity(n)
zero_n = np.zeros((n, n))

P_zeros = zero_n
P_l = w_l * eye_n
P_dl = w_dl * eye_n
P_ddl = (w_ddl + 2*w_dddl/delta_s/delta_s) * eye_n -2 * w_dddl / delta_s/delta_s* np.eye(n,k = -1)
P_ddl[0][0] = w_ddl + w_dddl/delta_s/delta_s
P_ddl[n-1][n-1] = w_ddl + w_dddl/delta_s/delta_s

P = sparse.csc_matrix(np.block([
    [P_l,P_zeros,P_zeros],
    [P_zeros,P_dl,P_zeros],
    [P_zeros,P_zeros,P_ddl]
    ]))
q = np.array([-w_l*s_ for s_ in s_ref])

####################构造A和LU################

#构造：l(i+1) = l(i) + l'(i) * delta_s + 1/2 * l''(i) * delta_s^2 + 1/6 * l'''(i) * delta_s^3
A_ll = -eye_n + np.eye(n,k = 1)
A_ldl = -delta_s * eye_n
A_lddl = -0.5 * delta_s * delta_s * eye_n
A_l = (np.block([
    [A_ll,A_ldl,A_lddl]
    ]))

# 构造：l'(i+1) = l'(i) + l''(i) * delta_s + 1/2 * l'''(i) * delta_s^2
A_dll = zero_n
A_dldl = -eye_n + np.eye(n,k = 1)
A_dlddl =  -delta_s * eye_n
A_dl = np.block([
    [A_dll,A_dldl,A_dlddl]
    ])

A_ul = np.block([
    [eye_n,zero_n,zero_n],
    [zero_n,zero_n,zero_n],
    [zero_n,zero_n,zero_n]
    ])#3n*3n
# 初始化设置
A_init = np.zeros((3, 3*n))
A_init[0][0] = 1

A = sparse.csc_matrix(np.row_stack((A_ul,A_l,A_dl,A_init)))

low_bound[5*n] = 1
up_bound[5*n] = 1
l = np.array(low_bound)
u = np.array(up_bound)

# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace and change alpha parameter
prob.setup(P, q, A, l, u, alpha=1.0)

# Solve problem
res = prob.solve()

plt.plot(u[:n],'.',color = 'blue')
plt.plot(l[:n],'.',color = 'black')
# plt.plot(s_ref[:n],'.')
plt.plot(res.x[:n],'.',color = 'red')
plt.show()
