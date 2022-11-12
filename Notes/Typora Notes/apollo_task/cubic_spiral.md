# 三次螺旋曲线

## Hermite_Curves

三阶曲线方程的标准形式
$$
x(t) = a_xt^3 + b_xt^2 + c_xt +d_x \\
y(t) = a_yt^3 + b_yt^2 + c_yt +d_y \\
z(t) = a_zt^3 + b_zt^2 + c_zt +d_z \\
$$
单独就x变量来考虑
$$
P(t)_x = a_xt^3 + b_xt^2 + c_xt +d_x\\
\frac {d}{dt}P(t)_x = 3a_xt^2 + 2 b_xt + c_x
$$
起始点 $(P(0)_X,\frac{d}{dt}P(0)_X)$，终点$(P(1)_X,\frac{d}{dt} P(1)_X)$ 

带入上述方程可得到


$$
\begin{vmatrix}
P(0)_X \\ 
P(1)_X \\
\frac{d}{dt}P(0)_X\\
\frac{d}{dt} P(1)_X
\end{vmatrix}
=
\begin{vmatrix}
0&0&0&1\\
1&1&1&1\\
0&0&1&0\\
3&2&1&0\\
\end{vmatrix} 
\begin{vmatrix}
a_x\\
b_x\\
c_x\\
d_x
\end{vmatrix} =>
\begin{vmatrix}
a_x\\
b_x\\
c_x\\
d_x
\end{vmatrix}
=
\begin{vmatrix}
2&-2&1&1\\
-3&3&-2&-1\\
0&0&1&0\\
1&0&0&0\\
\end{vmatrix} 
\begin{vmatrix}
P(0)_X \\ 
P(1)_X \\
\frac{d}{dt}P(0)_X\\
\frac{d}{dt} P(1)_X
\end{vmatrix}
$$
故
$$
\\ P(t)_x = [t^3,t^2,t,1]
\begin{bmatrix}
2&-2&1&1\\
-3&3&-2&-1\\
0&0&1&0\\
1&0&0&0\\
\end{bmatrix}
\begin{bmatrix}
P(0)_X \\ 
P(1)_X \\
\frac{d}{dt}P(0)_X\\
\frac{d}{dt} P(1)_X
\end{bmatrix}\\
$$
推广到三维，可得到
$$
Q(t)_x = [t^3,t^2,t,1]
\begin{bmatrix}
2&-2&1&1\\
-3&3&-2&-1\\
0&0&1&0\\
1&0&0&0\\
\end{bmatrix}
\begin{bmatrix}
x_0&y_0&z_0 \\ 
x_1&y_1&z_1 \\
dx_0&dy_0&dz_0\\
dx_1&dy_1&dz_1
\end{bmatrix}\\
$$
Example

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Screenshot from 2022-06-12 22-02-15.png" alt="Screenshot from 2022-06-12 22-02-15" style="zoom: 80%;" />

## 构建三次螺旋曲线

### 三次螺旋曲线表达式推导

$$
\theta(s) = as^3+ bs^2 + cs+d\\
\kappa(s) = 3as^2+ 2bs + c
$$
在单位区间$(0,1)$，给定**起点** $(\theta(0),kappa(0))$，**终点** $(\theta(1),kappa(1))$

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Screenshot from 2022-06-13 00-16-24.png" alt="Screenshot from 2022-06-13 00-16-24"  />

得到系数矩阵为：
$$
\begin{vmatrix}
a\\
b\\
c\\
d
\end{vmatrix}
=
\begin{vmatrix}
2&-2&1&1\\
-3&3&-2&-1\\
0&0&1&0\\
1&0&0&0\\
\end{vmatrix} 
\begin{vmatrix}
\theta(0) \\
\theta(1)\\
kappa(0) \\
kappa(1)
\end{vmatrix}
$$

推广到任意区间 $(s_i,s_{i+1})$，将后者变量映射到 $(0,1)$，则得到三次螺旋曲线表达式为：
$$
\theta(s)  = [ t^3, t^2, t,1]
\begin{bmatrix}
2&-2&1&1\\
-3&3&-2&-1\\
0&0&1&0\\
1&0&0&0\\
\end{bmatrix}
\begin{bmatrix}
\theta_i \\
\theta_{i+1}\\
(s_{i+1}-s_i)kappa_i \\
(s_{i+1}-s_i)kappa_{i+1}
\end{bmatrix},& t = \frac {s-s_i}{s_{i+1}-s_i} ,s \in[s_i,s_{i+1}]
$$

### 建立数学模型

#### 优化变量

$$
\vec{q} =[ \vec{\theta},\vec{\dot{\theta} },\vec{x},\vec{y} ]
$$

#### 建立约束方程

$$
cost function = \frac{1}{2}\|(x_{i+1} - x_i-\int_{0}^{ s}\cos(\theta(s))ds) + (y_{i+1} - y_i-\int_{0}^{ s}\sin(\theta(s))ds)\|^2
$$
![两点之间的弧长](https://www.shuxuele.com/calculus/images/arc-length-1.gif)

#### 设置边界条件

$$
\begin{cases} x_{ref_i} -r_i \leq x_i \leq x_{ref_i} +r_i \\
     y_{ref_i} -r_i \leq y_i \leq y_{ref_i} +r_i \end{cases}
$$

#### 求解约束方程中的积分问题

$$
x_{i+1}=x_i+\int_{0}^{ s}\cos(\theta(s))ds 	\approx x_i + \sum_{i=1}^{7} w_i \cdot cos(\theta(\Delta s_i)) \\
    y_{i+1}=y_i+\int_{0}^{ s}\sin(\theta(s))ds\approx y_i + \sum_{i=1}^{7} w_i \cdot sin(\theta(\Delta s_i))
$$
将区间$s \in[s_i,s_{i+1}]$等分为7份，通过高斯勒让得求积来获得在笛卡尔坐标系下的位置

| $\Delta s_i$(采样点) | $w_i$(权重)  |
| :------------------: | :----------: |
|     0.025446$s$      | 0.0647425$s$ |
|     0.129234$s$      | 0.139853$s$  |
|     0.297077$s$      | 0.190915$s$  |
|        0.5$s$        |  0.20898$s$  |
|     0.702923$s$      | 0.190915$s$  |
|     0.870766$s$      | 0.139853$s$  |
|     0.974554$s$      | 0.0647425$s$ |

**最终该问题可表述为:**
$$
\min_x \ \ \sum_{i}\frac{1}{2}\|(x_{i+1} - x_i-\int_{0}^{ s}\cos(\theta(s))ds) + (y_{i+1} - y_i-\int_{0}^{ s}\sin(\theta(s))ds)\|^2 \\
s.t.\begin{cases}  x_{ref_i} -r_i \leq x_i \leq x_{ref_i} +r_i \\
     y_{ref_i} -r_i \leq y_i \leq y_{ref_i} +r_i\\\end{cases}
$$
利用ceres库求解上述问题

```c++
自动微分耗时：
Time for solve time = 320.6 msec.
手动微分耗时：
Time for solve time = 0.773201 msec.
```

初始点不平滑的拟合结果:

![new_xy](/home/next/new_xy.png)

初始点平滑的拟合结果:

![hermit](/home/next/hermit.png)

#### 整体逻辑

 ```mermaid
 graph LR;
     n个离散点x,y-->n-1个分段区间-->三次螺旋曲线构建分段区间表达式
 ```

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Screenshot from 2022-05-27 09-42-56.png" alt="Screenshot from 2022-05-27 09-42-56" style="zoom: 67%;" />

![matchpoints](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/matchpoints.png)
