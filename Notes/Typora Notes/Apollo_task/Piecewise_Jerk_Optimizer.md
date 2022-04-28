# Piecewise Jerk Path Optimizer

该task的代码位于`/modules/planning/tasks/optimizers/piecewise_jerk_path` ,主要靠调用`modules/planning/math/piecewise_jerk` 下的类方法来帮助其完成二次规划问题的构造及求解工作。

```c++
/*
 * @brief:
 * FEM stands for finite element method.
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */
class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
	...
};
```

```bash
/modules/planning/tasks/optimizers/piecewise_jerk_path
.
├── BUILD
├── piecewise_jerk_path_ipopt_solver.cc
├── piecewise_jerk_path_ipopt_solver.h
├── piecewise_jerk_path_optimizer.cc
└── piecewise_jerk_path_optimizer.h

modules/planning/math/piecewise_jerk
.
├── BUILD
├── piecewise_jerk_path_problem.cc
├── piecewise_jerk_path_problem.h #派生类
├── piecewise_jerk_problem.cc
├── piecewise_jerk_problem.h #基类
├── piecewise_jerk_speed_problem.cc
└── piecewise_jerk_speed_problem.h #派生类

#path优化和speed优化的约束条件是一致的，都是在基类中实现的那个约束条件构造函数
```

## 横向轨迹优化

### 1	整体流程

1. **Process**

2. **OptimizePath** (task)

3. **set (Points,weight,DP_ref)**(目标点个数及坐标，各项优化目标权重，DP规划出来的path)

4. **Optimize**(优化函数的入口，设置默认迭代次数4000)

5. **FormulateProblem**( 用于构造二次优化问题的具体矩阵，也就是将规划问题的求解条件转化为OSQP可求解形式的接口)

   1. **CalculateKernel()**
   2. **CalculateAffineConstraint()**
   3. **CalculateOffset()**

6. **SolverDefaultSettings**(默认配置的参数接口)

7. **osqp setup**(osqp库接口)

8. **osqp solve**(osqp求解接口)

9. **FreeData**(删除数据，释放内存)

   ```c++
   //osqp求解步骤
     OSQPData* data = FormulateProblem();
     OSQPSettings* settings = SolverDefaultSettings();
     settings->max_iter = max_iter;
     OSQPWorkspace* osqp_work = osqp_setup(data, settings);
     osqp_solve(osqp_work);
   ```

### 2	FormulateProblem()

> 二次规划问题中P、A是稀疏矩阵

1. **CalculateKernel**

   构造二次项系数矩阵p的压缩矩阵

2. **CalculateAffineConstraint**

   构造A矩阵以及上下边界lower_bounds和upper_bounds的压缩矩阵

3. **CalculateOffset**

   构造一次项系数矩阵q的压缩矩阵

4. **csc matrix**

将上述转换得到的矩阵压入OSQPData中

```c++
  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A =csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
```

## 如何构造一个最优化问题？

以四个点(p1、p2、p3、p4)为例构造最优化问题

### **二次规划的一般形式**

$$
minimize \frac{1}{2} \cdot x^T \cdot P \cdot x + 	Q \cdot x \\
s.t. LB \leq A\cdot x \leq UB
$$

### CalculateKernel()构造目标函数矩阵

#### 1.`x`矩阵

- x矩阵即为需要优化的变量

$$
x^T =\begin{vmatrix}
 l_1\ l_2\ l_3\  l_4\ l_1'\ l_2'\ l_3'\ l_4'\ l_1''\ l_2''\ l_3''\ l_4''
\end{vmatrix}
$$

#### 2.`p、q`矩阵

通过构造函数来构造`p、q`矩阵，其中代价函数分为三部分

1. 曲线平滑`l,l',l'',l'''`
2. 与参考线的偏差`(l - lref)`
3. 终点位置的软约束

##### 总体代价函数公式

$$
cost\ \ function= 
w_l\cdot \sum_{i=0}^{n-1} l_i^2 + w_{{l}'}\cdot \sum_{i=0}^{n-1} {l_i}'^2 + w_{{l}''}\cdot \sum_{i=0}^{n-1} {l_i}''^2 + w_{{l}'''}\cdot \sum_{i=0}^{n-2}(\frac{{l_{i+1}}'' - {l_i}''}{\Delta s})^2 +\\ 
w_{end_l}\cdot (l_{n-1} - l_{endref})^2 + w_{end_{dl}}\cdot ({l}'_{n-1}-{l_{endref}}')^2 + w_{end_{ddl}}\cdot ({l}''_{n-1} - {l_{endref}}'')^2+\\
w_{ref}\cdot \sum_{i=0}^{n-1}(l_i-l_{ref})^2 +\\
w_{end_l}\cdot (l_{n-1} - l_{endref})^2 + w_{end_{dl}}\cdot ({l}'_{n-1}-{l_{endref}}')^2 + w_{end_{ddl}}\cdot ({l}''_{n-1} - {l_{endref}}'')^2
$$


##### ①曲线平滑的`cost`

$$
w_l\cdot \sum_{i=0}^{n-1} l_i^2 + w_{{l}'}\cdot \sum_{i=0}^{n-1} {l_i}'^2 + w_{{l}''}\cdot \sum_{i=0}^{n-1} {l_i}''^2 + w_{{l}'''}\cdot \sum_{i=0}^{n-2}(\frac{{l_{i+1}}'' - {l_i}''}{\Delta s})^2 \\
$$

- 转化为p矩阵(12*12)，记为`p1`

$$
p1=\begin{vmatrix}
w_l&0&0&0&0&0&0&0&0&0&0&0\\
0&w_l&0&0&0&0&0&0&0&0&0&0\\
0&0&w_l&0&0&0&0&0&0&0&0&0\\
0&0&0&w_l&0&0&0&0&0&0&0&0\\
0&0&0&0&w_{{l}'}&0&0&0&0&0&0&0\\
0&0&0&0&0&w_{{l}'}&0&0&0&0&0&0\\
0&0&0&0&0&0&w_{{l}'}&0&0&0&0&0\\
0&0&0&0&0&0&0&w_{{l}'}&0&0&0&0\\
0&0&0&0&0&0&0&0&w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2}&0&0&0\\
0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+2\frac{w_{{l}'''}}{\Delta s^2}&0&0\\
0&0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+2\frac{w_{{l}'''}}{\Delta s^2}&0\\
0&0&0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2}\\
\end{vmatrix}
$$
##### ②与参考线偏差

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210312221759419.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L0lIVFlfTlVJ,size_16,color_FFFFFF,t_70#pic_center)
$$
w_{ref}\cdot \sum_{i=0}^{n-1}(l_i-l_{ref})^2 \\
$$
- 二次项转化为p矩阵(4*12)，记为`p2`

$$
p2=\begin{vmatrix}
w_{ref_1}&0&0&0&0&0&0&0&0&0&0&0\\
0&w_{ref_2}&0&0&0&0&0&0&0&0&0&0\\
0&0&w_{ref_3}&0&0&0&0&0&0&0&0&0\\
0&0&0&w_{ref_4}&0&0&0&0&0&0&0&0\\
\end{vmatrix}
$$
- 一次项转化为q矩阵(4*1)，记为`q1`


​		**注：去掉上述约束方程的常量项**
$$
q1=\begin{vmatrix}
-2w_{ref_1}\cdot l_{ref_1}\\
-2w_{ref_2}\cdot l_{ref_2}\\
-2w_{ref_3}\cdot l_{ref_3}\\
-2w_{ref_4}\cdot l_{ref_4}\\
\end{vmatrix}
$$

##### ③终点

$$
w_{end_l}\cdot (l_{n-1} - l_{endref})^2 + w_{end_{dl}}\cdot ({l}'_{n-1}-{l'_{endref}})^2 + w_{end_{ddl}}\cdot ({l}''_{n-1} - {l''_{endref}})^2
$$

- ​	二次项转化为p矩阵(12*12)，记为`p3`

$$
p3=\begin{vmatrix}
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&w_{end_l}&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&w_{end_{dl}}&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&0&0&0&w_{end_{ddl}}
\end{vmatrix}
$$
- ​	一次项转化为q矩阵(12*1)，记为`q2`

$$
q2=\begin{vmatrix}
0\\
0\\
0\\
-2w_{end_l}\cdot endl\\
0\\
0\\
0\\
-2w_{end_dl}\cdot enddl\\
0\\
0\\
0\\
-2w_{end_ddl}\cdot endddl\\
\end{vmatrix}
$$

#### 3.构造优化目标函数

综合`x，p，q`可得到`cost function`:
$$
minimize\ f(l(s)) = {x}^T_{(1*12)}(p_{1}^T p_{1} +  p_{2}^T p_{2} + p_{3}^T p_{3})_{(12*12)}{x}_{(12*1)} + (q_{1}^T+q_{2}^T)_{(1*12)}x_{(12*1)}
$$
记：
$$
{P}_{(12*12)} = p_{1}^T p_{1} +  p_{2}^T p_{2} + p_{3}^T p_{3}\\
{Q}_{(1*12)} = q_{1}^T+q_{2}^T
$$
得到最终目标函数的表达式:
$$
minimize\ f(l(s)) = {x}^TP{x} + Qx
$$

其中：
$$
x^T_{(1*12)} =\begin{vmatrix}
 l_1\ l_2\ l_3\  l_4\ l_1'\ l_2'\ l_3'\ l_4'\ l_1''\ l_2''\ l_3''\ l_4''
\end{vmatrix}
$$

$$
P_{(12*12)}=\begin{vmatrix}
w_l + w_{ref_1}&0&0&0&0&0&0&0&0&0&0&0\\
0&w_l+ w_{ref_1}&0&0&0&0&0&0&0&0&0&0\\
0&0&w_l+ w_{ref_1}&0&0&0&0&0&0&0&0&0\\
0&0&0&w_l+ w_{ref_1}+w_{end_l}&0&0&0&0&0&0&0&0\\
0&0&0&0&w_{{l}'}&0&0&0&0&0&0&0\\
0&0&0&0&0&w_{{l}'}&0&0&0&0&0&0\\
0&0&0&0&0&0&w_{{l}'}&0&0&0&0&0\\
0&0&0&0&0&0&0&w_{{l}'}+w_{end_{dl}}&0&0&0&0\\
0&0&0&0&0&0&0&0&w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2}&0&0&0\\
0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+2\frac{w_{{l}'''}}{\Delta s^2}&0&0\\
0&0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+2\frac{w_{{l}'''}}{\Delta s^2}&0\\
0&0&0&0&0&0&0&0&0&0&-2\frac{w_{{l}'''}}{\Delta s^2}&w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2}++w_{end_{dl}}\\
\end{vmatrix}\\
$$

$$
Q_{(12*1)}=\begin{equation}
	\begin{bmatrix}
-2w_{ref_1}\cdot l_{ref_1}\\
-2w_{ref_2}\cdot l_{ref_2}\\
-2w_{ref_3}\cdot l_{ref_3}\\
-2w_{ref_4}\cdot l_{ref_4}-2w_{end_l}\cdot endl\\
0\\
0\\
0\\
-2w_{end_dl}\cdot enddl\\
0\\
0\\
0\\
-2w_{end_ddl}\cdot endddl\\
 \end{bmatrix}
\end{equation}
$$

#### 扩展到n个点？

假设约束维度任然是二阶(l、l'、l'')，那么上述的P矩阵为(3n* 3n)，Q矩阵为(3n* * 1)
$$
x^T =\begin{vmatrix}
 l_1\ \dots  l_n\ l_1'\ \dots l_n'\ l_1''\dots\ l_n''
\end{vmatrix}_{3n \times 1}
$$

$$
P=
\begin{equation}
\begin{bmatrix}

\begin{bmatrix}
w_l + w_{ref_1} & \dots & 0\\ \vdots
  & \ddots & \vdots\\
0 &  \dots& w_l+ w_{ref_1}+w_{end_l}
\end{bmatrix}_{n \times n}	\\
&   & &  \\& 

\begin{bmatrix}
w_{{l}'} & \dots & 0\\ \vdots
  & \ddots & \vdots\\
0 &  \dots& w_{{l}'}+w_{end_{dl}}
\end{bmatrix}_{n \times n} 
&  & \\&   & 

\begin{bmatrix}
w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2} & 0 &\dots &\dots\\ 
-2\frac{w_{{l}'''}}{\Delta s^2} &w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2} & 0&\dots\\
0&-2\frac{w_{{l}'''}}{\Delta s^2}&\ddots&\dots \\
\vdots&\ddots &\ddots& \\
0 &\dots& -2\frac{w_{{l}'''}}{\Delta s^2} & w_{{l}''}+\frac{w_{{l}'''}}{\Delta s^2}+w_{end_{dl}}
\end{bmatrix}_{n \times n}&\\

\end{bmatrix}_{3n \times 3n}
\end{equation}
$$

$$
Q=
\begin{equation}
	\begin{bmatrix}
		\begin{bmatrix}
-2w_{ref_1}\cdot l_{ref_1}\\
\vdots\\
-2w_{ref_4}\cdot l_{ref_4}-2w_{end_l}\cdot endl\\

	\end{bmatrix}_{n \times 1}\\
\begin{bmatrix}
	0\\
	\vdots\\
	-2w_{end_dl}\cdot enddl\\
\end{bmatrix}_{n \times 1}\\
\begin{bmatrix}

0\\
\vdots\\
	\vdots\\
	-2w_{end_ddl}\cdot endddl\\
\end{bmatrix}_{n \times 1}\\
 \end{bmatrix}
\end{equation}_{3n \times 1}
$$



### CalculateAffineConstraint()构造约束矩阵

#### 1.对I的约束

车辆行驶位置，即对道路边界的约束![img](https://www.asam.net/index.php?eID=dumpFile&t=p&p=48183&token=13641da5e3ca621e2a28b920f5e33596adcd4a9e)

```c++
DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
              "The upper bound of longitudinal jerk.");
```

构造矩阵(4*12)：
$$
\begin{bmatrix}
lb_{s1} \\ 
lb_{s2} \\ 
lb_{s3} \\
lb_{s4} 
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
1&0&0&0&0&0&0&0&0&0&0&0&\\
0&1&0&0&0&0&0&0&0&0&0&0&\\
0&0&1&0&0&0&0&0&0&0&0&0&\\
0&0&0&1&0&0&0&0&0&0&0&0&\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
ub_{s1} \\ 
ub_{s2} \\ 
ub_{s3} \\
ub_{s4} 
 \end{bmatrix}
$$

#### 2.对l‘的约束

轨迹的一阶导为heading，可以近似理解为横向运动的“速度”，希望不要横向走的太快

```c++
DEFINE_double(lateral_derivative_bound_default, 2.0,
              "the default value for lateral derivative bound.");
```

构造矩阵(4*12)：
$$
\begin{bmatrix}
-2.0 \\ 
-2.0 \\ 
-2.0 \\
-2.0 
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&1&0&0&0&0&0&0\\
0&0&0&0&0&0&1&0&0&0&0&0\\
0&0&0&0&0&0&0&1&0&0&0&0\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
2.0 \\ 
2.0 \\ 
2.0 \\
2.0 
 \end{bmatrix}
$$

#### 3.对l‘‘的约束

轨迹的二阶导可以近似理解为横向运动的“加速度”，希望方向盘不要打得太猛

因为车辆存在最小的转弯半径,即存在最大行驶曲率，所以要对车辆运动学进行限制。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190116203624681.jpg)



![img](https://pic4.zhimg.com/80/v2-1ac642bc93c3a89a67c31925dba6cc47_720w.jpg)
$$
K_{max}=\frac{1}{R}=\frac{tan(\delta_{max})}{L} = \frac{tan(\frac{maxSteerAngle}{steerRadio})}{L}\\
其中：R为车辆的最大转弯半径，L为车辆轴距\\
\delta_{max}为前轮最大转角，steerRadio为车辆转向传动比
$$



##### 问题(bug)

代码中的意思是直接把`l''`等价为当前轨迹的曲率k，而实际的k值计算如下文所示

- 在百度发表的论文中对车辆曲率的约束如下：[Optimal Vehicle Path Planning Using Quadratic Optimization for Baidu Apollo Open Platform](https://ieeexplore.ieee.org/document/9304787)

![image-20220412125502258](/home/next/.config/Typora/typora-user-images/image-20220412125502258.png)

![image-20220412125529603](/home/next/.config/Typora/typora-user-images/image-20220412125529603.png)
$$
\kappa_rκ 和κ r ˙ \dot{\kappa_r} 是参考线在 p_r处的曲率和曲率变化率，\Delta \theta是车辆和参考线点p_r处切线方向的角度差。\\
 简化：假设车辆几乎在沿着道路方向行驶，因此\Delta \theta = 0；
''横向加速度''l'' 是很小的，数量级在10^{-2}，因此l'' =0
$$

- 百度公开课讲解的曲率约束内容如下

<img src="/home/next/.config/Typora/typora-user-images/image-20220412124915719.png" alt="image-20220412124915719" style="zoom: 67%;" />

- 但是在代码中A矩阵的赋值根据代码直接设为1，也就是说`l''`直接等于了当前车辆的行驶曲率k

  - 给A矩阵赋值

    ```c++
      int constraint_index = 0;
      // set x, x', x'' bounds
      for (int i = 0; i < num_of_variables; ++i) {
        if (i < n) {
          variables[i].emplace_back(constraint_index, 1.0);
          lower_bounds->at(constraint_index) =
              x_bounds_[i].first * scale_factor_[0];
          upper_bounds->at(constraint_index) =
              x_bounds_[i].second * scale_factor_[0];
        } else if (i < 2 * n) {
          variables[i].emplace_back(constraint_index, 1.0);
    
          lower_bounds->at(constraint_index) =
              dx_bounds_[i - n].first * scale_factor_[1];
          upper_bounds->at(constraint_index) =
              dx_bounds_[i - n].second * scale_factor_[1];
        } else {
          variables[i].emplace_back(constraint_index, 1.0);
          lower_bounds->at(constraint_index) =
              ddx_bounds_[i - 2 * n].first * scale_factor_[2];
          upper_bounds->at(constraint_index) =
              ddx_bounds_[i - 2 * n].second * scale_factor_[2];
        }
        ++constraint_index;
      }
      
      //给A矩阵赋值
      int ind_p = 0;
      for (int i = 0; i < num_of_variables; ++i) {
        A_indptr->push_back(ind_p);
        for (const auto& variable_nz : variables[i]) {
          // coefficient
          A_data->push_back(variable_nz.second);
    
          // constraint index
          A_indices->push_back(variable_nz.first);
          ++ind_p;
        }
    ```

  - 上下边界赋值

    ```c++
    //车辆运动学约束，由车轮最大转角推导行驶过程中的最大曲率
    const double lat_acc_bound =
            std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
            veh_param.wheel_base();
    
    //要考虑道路的曲率,所以要减去道路的kappa值
    double kappa = reference_line.GetNearestReferencePoint(s).kappa();
          ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    ```

根据代码构造出矩阵如下(4*12)：
$$
\begin{bmatrix}
-lat\_acc\_bound - kappa_{s1} \\ 
-lat\_acc\_bound - kappa_{s2} \\ 
-lat\_acc\_bound - kappa_{s3} \\
-lat\_acc\_bound - kappa_{s4} 
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
0&0&0&0&0&0&0&0&1&0&0&0\\
0&0&0&0&0&0&0&0&0&1&0&0\\
0&0&0&0&0&0&0&0&0&0&1&0\\
0&0&0&0&0&0&0&0&0&0&0&1\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
lat\_acc\_bound - kappa_{s1} \\ 
lat\_acc\_bound - kappa_{s2} \\ 
lat\_acc\_bound - kappa_{s3} \\
lat\_acc\_bound - kappa_{s4} 
 \end{bmatrix}
$$

上述矩阵相当于直接把了**l‘’等价为曲率**,而实际k的约束如论文中所示，两个地方约束计算并不相同。
$$
tan(δ_{max})×k_{r}×l − tan(δ_{max})+k_{r}×L≤0
$$


#### 4.对I‘‘’的约束

由差分求导可得到轨迹的三阶导数，可以理解为人打方向盘的加速度，此时是对`jerk`的约束，`delta_s_ = 1.0;`
$$
l''' = \frac{{l}_{i+1}'' - {l}_{i}''}{\Delta{s}}
$$
横摆角速度：
$$
yaw\_rate = \frac{({{w}_1-{w}_2})\cdot{R}_r}{A}
$$


```c++
double PiecewiseJerkPathOptimizer::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) const {
  return max_yaw_rate / axis_distance / vehicle_speed;
}
```

$$
\begin{bmatrix}
-jerk_1 * \Delta s \\ 
-jerk_2 * \Delta s \\ 
-jerk_3 * \Delta s 
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
0&0&0&0&0&0&0&0&-1&1&0&0\\
0&0&0&0&0&0&0&0&0&-1&1&0\\
0&0&0&0&0&0&0&0&0&0&-1&1\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
jerk_1 * \Delta s \\ 
jerk_2 * \Delta s \\ 
jerk_3 * \Delta s 
 \end{bmatrix}
$$

#### 5.对起点p1的约束

起点必须在初始点的位置
$$
\begin{bmatrix}
ego_l \\ 
ego_{dl} \\ 
ego_{ddl} \\
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
1&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&1&0&0&0\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
ego_l \\ 
ego_{dl} \\ 
ego_{ddl} \\
 \end{bmatrix}
$$

#### 6.路径连续性约束

```c++
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
		...
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x(i+1) - x(i) - delta_s * x(i)'
  // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
		..._{(12*1)}

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }
```

对上述代码中的公式推导：

> 将零阶状态用一二阶状态进行线性表示，使其更为合理地表示**各界状态的关联关系**，确保**路径路径的连续性**。轨迹的三阶导数会随着二阶导数的变化而变化，但两点的三阶导保持相等

![img](https://pic1.zhimg.com/80/v2-bdec651f742853355a7748bdaaac095c_720w.jpg)

![img](https://pic3.zhimg.com/80/v2-1afc411996ef4df7d8a47f6540fcc13a_720w.jpg)

​									通过上述(1-6)(1-8)可得到：
$$
l_{i+1}' - l_i' -\frac{1}{2}\Delta s *l_i'' - \frac{1}{2}\Delta s *l_{i+1}'' = 0\\
l_{i+1} - l_i - \Delta s \cdot l_i' - \frac{1}{3} \Delta s ^2 \cdot l_i'' - \frac{1}{6} \Delta s^2 \cdot l_{i+1}'' = 0
$$

$$
\begin{bmatrix}
0 \\ 
0 \\ 
0 
\end{bmatrix} 
 \leq 
 \begin{bmatrix} 
0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&0&0\\
0&0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&0\\
0&0&0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
0 \\ 
0 \\ 
0 
 \end{bmatrix}
$$

$$
\begin{bmatrix}
0 \\ 
0 \\ 
0 
\end{bmatrix} 
 \leq 
  \begin{bmatrix} 
-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&0&0\\
0&-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&0\\
0&0&-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2\\
 \end{bmatrix} 
 x 
 \leq  
 \begin{bmatrix}
0 \\ 
0 \\ 
0 
 \end{bmatrix}
$$

#### 7.构造约束条件

$$
x^T=\begin{vmatrix}
 l_1\ l_2\ l_3\  l_4\ l_1'\ l_2'\ l_3'\ l_4'\ l_1''\ l_2''\ l_3''\ l_4''
\end{vmatrix}_{1 \times 12}
$$


$$
A=
\begin{bmatrix} 
1&0&0&0&0&0&0&0&0&0&0&0&\\
0&1&0&0&0&0&0&0&0&0&0&0&\\
0&0&1&0&0&0&0&0&0&0&0&0&\\
0&0&0&1&0&0&0&0&0&0&0&0&\\
0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&1&0&0&0&0&0&0\\
0&0&0&0&0&0&1&0&0&0&0&0\\
0&0&0&0&0&0&0&1&0&0&0&0\\
0&0&0&0&0&0&0&0&1&0&0&0\\
0&0&0&0&0&0&0&0&0&1&0&0\\
0&0&0&0&0&0&0&0&0&0&1&0\\
0&0&0&0&0&0&0&0&0&0&0&1\\
0&0&0&0&0&0&0&0&-1&1&0&0\\
0&0&0&0&0&0&0&0&0&-1&1&0\\
0&0&0&0&0&0&0&0&0&0&-1&1\\
1&0&0&0&0&0&0&0&0&0&0&0\\
0&0&0&0&1&0&0&0&0&0&0&0\\
0&0&0&0&0&0&0&0&1&0&0&0\\
0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&0&0\\
0&0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&0\\
0&0&0&0&0&0&-1&1&0&0&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s\\
-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&0&0\\
0&-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&0\\
0&0&-1&1&0&0&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2\\
 \end{bmatrix}_{24 \times 12}
$$

$$
LB=
\begin{bmatrix} 
lb_{s1} \\ 
lb_{s2} \\ 
lb_{s3} \\
lb_{s4} \\
-2.0 \\ 
-2.0 \\ 
-2.0 \\
-2.0 \\
-lat\_acc\_bound - kappa_{s1} \\ 
-lat\_acc\_bound - kappa_{s2} \\ 
-lat\_acc\_bound - kappa_{s3} \\
-lat\_acc\_bound - kappa_{s4} \\
-jerk_1 * \Delta s \\ 
-jerk_2 * \Delta s \\ 
-jerk_3 * \Delta s \\
ego_l \\ 
ego_{dl} \\ 
ego_{ddl} \\
0 \\ 
0 \\ 
0 \\
0 \\ 
0 \\ 
0 
 \end{bmatrix}_{24 \times 1}
 =
 \begin{bmatrix} 
Lbsi_{(4*1)} \\ 
Heading1_{(4*1)} \\ 
Kappa1_{(4*1)}\\
Jerk1_{(3*1)} \\
Ego1_{(3*1)}\\
Continuous1_{(6*1)}
 \end{bmatrix}\       ;    \
 UB=
\begin{bmatrix} 
ub_{s1} \\ 
ub_{s2} \\ 
ub_{s3} \\
ub_{s4} \\
2.0 \\ 
2.0 \\ 
2.0 \\
2.0 \\
lat\_acc\_bound - kappa_{s1} \\ 
lat\_acc\_bound - kappa_{s2} \\ 
lat\_acc\_bound - kappa_{s3} \\
lat\_acc\_bound - kappa_{s4} \\
jerk_1 * \Delta s \\ 
jerk_2 * \Delta s \\ 
jerk_3 * \Delta s \\
ego_l \\ 
ego_{dl} \\ 
ego_{ddl} \\
0 \\ 
0 \\ 
0 \\
0 \\ 
0 \\ 
0 
 \end{bmatrix}_{24 \times 1}
 =
 \begin{bmatrix} 
Ubsi_{(4*1)} \\ 
Heading2_{(4*1)} \\ 
Kappa2_{(4*1)}\\
Jerk2_{(3*1)} \\
Ego2_{(3*1)}\\
Continuous2_{(6*1)}
 \end{bmatrix}
$$

综上得到约束条件：
$$
LB_{(12*1)}\le A_{(24*12)}x_{(12*1)}\le UB_{(12*1)}
$$



#### 扩展到n个点？

$$
A=
\begin{equation}
	\begin{bmatrix}
	
		
1 & \dots &\dots& 0\\ 
\vdots & \ddots && \vdots\\
\vdots && \ddots &\vdots\\
0 &\dots  &\dots& 1\\

&&&&1 & \dots &\dots& 0\\ 
&&&&\vdots & \ddots && \vdots\\
&&&&\vdots && \ddots &\vdots\\
&&&&0 &\dots  &\dots& 1\\

&&&&&&&&1&\dots &\dots& 0\\ 
&&&&&&&&\vdots & \ddots && \vdots\\
&&&&&&&&\vdots && \ddots &\vdots\\
&&&&&&&&0 &\dots  &\dots& 1\\

&&&&&&&&-1&1 &\dots& 0\\ 
&&&&&&&&\vdots & \ddots&\ddots & \vdots\\
&&&&&&&&\vdots && \ddots &\ddots \\
&&&&&&&&0 &\dots  &-1& 1\\\\


1&\dots&\dots&0\\
0&\dots&&\dots&1&\dots&\dots&0\\
0&\dots&&&&&&\dots&1&\dots&\dots&0 \\\\\

0&\dots&\dots&0&-1&1&\dots&\dots&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&\dots&\dots\\
0&\dots&&\dots&0&-1&1&\dots&\dots&-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s&\dots\\
\vdots&\dots&&&\dots&\ddots&\ddots&\ddots&\ddots&\ddots&\ddots&\ddots\\
0&\dots&&&&\dots&0&-1&1&\dots&\dots-\frac{1}{2}\cdot\Delta s&-\frac{1}{2}\cdot\Delta s\\\\

-1&1&\dots&\dots&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&\dots&\dots\\
0&-1&1&\dots&\dots&-\Delta s&\dots&\dots&\dots&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2&\dots\\
\vdots&\dots&\ddots&\ddots&\dots&\dots&\ddots&\ddots&\ddots&\ddots&\ddots&\vdots\\
0&\dots&-1&1&\dots&\dots&-\Delta s&0&0&0&-\frac{1}{3}\Delta s^2&-\frac{1}{6}\Delta s^2\\
	 \end{bmatrix}
\end{equation}_{6n \times 3n}
$$

$$
LB=
\begin{bmatrix} 
Lbsi_{(n*1)} \\ 
Heading1_{(n*1)} \\ 
Kappa1_{(n*1)}\\
Jerk1_{((n-1)*1)} \\
Ego1_{(3*1)}\\
Continuous1_{((2n-2)*1)}
 \end{bmatrix}_{6n \times 1}&
UB=
 \begin{bmatrix} 
Ubsi_{(n*1)} \\ 
Heading2_{(n*1)} \\ 
Kappa2_{(n*1)}\\
Jerk2_{((n-1)*1)} \\
Ego2_{(3*1)}\\
Continuous2_{((2n-2)*1)}
 \end{bmatrix}_{6n \times 1}
$$

### 总结

假设有n个点，优化维度为三维(l、l'、l'')，通过构造`P,Q,A,LB,UB`矩阵方程，将此问题转化为二次规划问题
$$
minimize\ f(l(s)) = {x}^TP{x} + Qx\\
s.t.\  \   LB\le Ax\le UB\\
$$
其中：
$$
x^T =\begin{vmatrix}
 l_1\ \dots  l_n\ l_1'\ \dots l_n'\ l_1''\dots\ l_n''
\end{vmatrix}_{3n \times 1}\\

{P}= [...]_{3n \times 3n} \\
{Q}= [...]_{3n \times 1} \\
{A}= [...]_{6n \times 3n} \\

LB=[...]_{6n \times 1}\\
UB=[...]_{6n \times 1}
$$





# Picewise Jerk Speed Optimizer

## 纵向速度轨迹优化

SL规划保证车辆的横向偏移足够平滑，ST规划保证车辆的前进方向速度变化足够平滑.

### 1.`x`矩阵

x矩阵即为需要优化的变量
$$
x^T =\begin{vmatrix}
 s_1\ \dots  s_{n}\ s_1'\ \dots  s'_{n}\ s_1''\dots  s_{n}''
\end{vmatrix}
$$

### 2.`p、q`矩阵

跟path optimizer区别在于p矩阵，speed多了对参考线偏差的一阶偏差约束

#### ①曲线平滑

$$
w_{s}\cdot \sum_{i=0}^{n-1} s_{i}^2 + 
w_{ds}\cdot \sum_{i=0}^{n-1} (s'_{i})^2 +w_{dds}\cdot \sum_{i=0}^{n-2} (\frac {s''_{i+1}-s''_{i}}{\Delta s^2} )^2\\
$$

#### ②与参考线偏差

$$
w_{xref}\cdot \sum_{i=0}^{n-1}(s_{i}-s_{ref})^2 + 
w_{dxref}\cdot \sum_{i=0}^{n-1}(s'_{i}-s'_{ref})^2\\
$$

#### ③终点

$$
w_{end_l}\cdot (s_{n-1} - s_{endref})^2 + w_{end_{dl}}\cdot ({s}'_{n-1}-{s'_{endref}})^2 + w_{end_{ddl}}\cdot ({s}''_{n-1} - {s''_{endref}})^2
$$

### 3.**约束条件**

约束分为六个部分(6n*1)
$$
对变量的约束(3n)：LowerBounds < s,s',s'',s'''<upperBounds\\
对Jerk的约束(n-1)：LowerBounds < s'''<upperBounds
对起点的约束(3)：ego_{1} \leq s_{1},s'_{1},s''_{1} \leq ego_{1}\\
连续性约束(2n-2)：
s_{i+1} - s_i - \Delta s \cdot s_i' - \frac{1}{3} \Delta s ^2 \cdot s_i'' - \frac{1}{6} \Delta s^2 \cdot s_{i+1}'' = 0\\
s_{i+1}' - s_i' -\frac{1}{2}\Delta s *s_i'' - \frac{1}{2}\Delta s *s_{i+1}'' = 0\\
$$


# 思考

## 1	为什么需要正定矩阵？

- 如果P是半正定矩阵，那么f(x)是一个[凸函数](https://zh.wikipedia.org/wiki/凸函数)。相应的二次规划为凸二次规划问题；此时若约束条件定义的可行域不为空，且目标函数在此可行域有下界，则该问题有全局最小值。
- 如果P是正定矩阵，则该问题有唯一的全局最小值。
- 若P为非正定矩阵，则目标函数是有多个平稳点]和局部极小点的[NP难问题](https://zh.wikipedia.org/wiki/NP问题)。
- 如果P=0，二次规划问题就变成线性规划问题。

正定是对二次函数有效的一个定义，对方程无效。

![img](https://pic1.zhimg.com/80/v2-ebdfb7c7510ac85a582d6e79c1ef37b3_720w.jpg?source=1940ef5c)
$$
f(x)>0,x!=0,则x为正定二次型，P为正定矩阵
$$
![img](https://pica.zhimg.com/80/v2-99b76aece0306301d4b983bf4a30f048_720w.jpg?source=1940ef5c)
$$
f(x)>=0,x!=0,则f为半正定二次型，P为半正定矩阵
$$


![img](https://pic1.zhimg.com/80/v2-374e0eef3c259af60bef57c22beb937e_720w.jpg?source=1940ef5c)
$$
不定
$$

## 2.压缩矩阵有哪几种方法？

参考[英伟达](https://www.bu.edu/pasi/files/2011/01/NathanBell1-10-1000.pdf)的介绍



------



