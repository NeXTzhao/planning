# 知识铺垫

## 范数

- **什么是范数？**

​		我们知道距离的定义是一个宽泛的概念，只要满足非负、自反、三角不等式就可以称之为距离。范数是一种强化了的距离概念，它在定义上比距离多了一条数乘的运算法则。有时候为了便于理解，我们可以把范数当作距离来理解。

​		在数学上，范数包括向量范数和矩阵范数，向量范数表征向量空间中向量的大小，矩阵范数表征矩阵引起变化的大小。一种非严密的解释就是，对应向量范数，向量空间中的向量都是有大小的，这个大小如何度量，就是用范数来度量的，不同的范数都可以来度量这个大小，就好比米和尺都可以来度量远近一样；对于矩阵范数，在线性代数中，我们知道，通过运算`AX=B`，可以将向量X变化为B，矩阵范数就是来度量这个变化大小的。

- **几种范数的定义和含义**

  LP范数：

  ​	![在这里插入图片描述](https://img-blog.csdnimg.cn/20190221143044448.png)

  根据P 的变化，范数也有着不同的变化，一个经典的有关P范数的变化图![img](https://pic4.zhimg.com/80/v2-715ee105f46adf614ab33700f06cca23_720w.jpg)

## 向量范数

### L0 范数

当P=0时，也就是 `L0` 范数，由上面可知，`L0`范数并不是一个真正的范数，它主要被用来度量向量中非零元素的个数。用上面的L-P定义可以得到的L-0的定义为：

![img](https://pic1.zhimg.com/80/v2-6ad7268f2b88f804ed27d327b6cbde98_720w.jpg)

在实际应用中，由于`L0`范数本身不容易有一个好的数学表示形式，给出上面问题的形式化表示是一个很难的问题，故被人认为是一个`NP`难问题。所以在实际情况中，`L0`的最优问题会被放宽到`L1`或`L2`下的最优化。

### L1范数

`L1`范数是我们经常见到的一种范数，它的定义如下：

![img](https://pic3.zhimg.com/80/v2-a4b18d3de67b6b628e60464cb7571ff2_720w.jpg)

表示向量x中非零元素的绝对值之和。

`L1`范数有很多的名字，例如我们熟悉的曼哈顿距离、最小绝对误差等。使用L1范数可以度量两个向量间的差异，如绝对误差和（Sum of Absolute Difference）：

![img](https://pic2.zhimg.com/80/v2-2fe034248733e3fff0b92e42acc65971_720w.jpg)

对于`L1`范数，它的优化问题如下：

![img](https://pic1.zhimg.com/80/v2-c6e86f5a00d1c83ba9d5e004d2b34d1c_720w.jpg)

![img](https://pic3.zhimg.com/80/v2-9aad23df9132265c83f189e9ac5d95aa_720w.jpg)

由于`L1`范数的天然性质，对`L1`优化的解是一个稀疏解，因此`L1`范数也被叫做稀疏规则算子。通过`L1`可以实现特征的稀疏，去掉一些没有信息的特征，例如在对用户的电影爱好做分类的时候，用户有100个特征，可能只有十几个特征是对分类有用的，大部分特征如身高体重等可能都是无用的，利用`L1`范数就可以过滤掉。

- 从几何角度来解释`L1`正则化的稀疏作用

  <img src="/home/next/.config/Typora/typora-user-images/image-20220422212736014.png" alt="image-20220422212736014" style="zoom:50%;" />

`L1` 正则化，带来的结果是，他在某一个坐标轴上的取值为零，而只在某一个轴上有数值，相当于把特征之间的关系去耦合了，通过此性质我可就可以对特征进行分类处理，把为零的特征过滤掉只保留想要的特征

### L2 范数

`L2`范数是我们最常见最常用的范数了，我们用的最多的度量距离欧氏距离就是一种`L2`范数，它的定义如下：

![img](https://pic4.zhimg.com/80/v2-8c7d73ca210d0d678ffc9c2112c632ab_720w.jpg)

表示向量元素的平方和再开平方。

像`L1`范数一样，`L2`也可以度量两个向量间的差异，如平方差和（Sum of Squared Difference）:

![img](https://pic4.zhimg.com/80/v2-49a25088faed0a811ac7902d4144a197_720w.jpg)

对于`L2`范数，它的优化问题如下：

![img](https://pic4.zhimg.com/80/v2-858a9ba0e32627308707a10210c8c9d7_720w.jpg)

![img](https://pic3.zhimg.com/80/v2-9aad23df9132265c83f189e9ac5d95aa_720w.jpg)

`L2`范数通常会被用来做优化目标函数的正则化项，防止模型为了迎合训练集而过于复杂造成过拟合的情况，从而提高模型的泛化能力。

- 从几何角度来解释 `L2` 正则化的防止过拟合的作用

  ![image-20220422213739444](/home/next/.config/Typora/typora-user-images/image-20220422213739444.png)

二维平面下`L2`正则化的函数图形是个圆（绝对值的平方和，是个圆），与方形相比，被磨去了棱角。因为不太可能出现多数w都为0的情况，这就是为什么 `L2` 正则化不具有稀疏性的原因。

拟合过程中通常都倾向于让权值尽可能小，最后构造一个所有参数都比较小的模型。因为一般认为参数值小的模型比较简单，能适应不同的数据集，也在一定程度上避免了过拟合现象。可以设想一下对于一个线性回归方程，若参数很大，那么只要数据偏移一点点，就会对结果造成很大的影响；但如果参数足够小，数据偏移得多一点也不会对结果造成什么影响，专业一点的说法是**抗扰动能力强**，而 `L2` 正则化正好可以获得值很小的参数。

### L-∞ 范数

当![img](https://img-blog.csdn.net/20180604172938724?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2E0OTM4MjM4ODI=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)时，也就是![img](https://img-blog.csdn.net/20180604172949953?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2E0OTM4MjM4ODI=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)范数，它主要被用来度量向量元素的最大值，与`L0`一样，通常情况下表示为 

![img](https://img-blog.csdn.net/20180604172742607?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2E0OTM4MjM4ODI=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)



### 举例

一范数二范数也是用来度量一个整体，比如两个个班的人比较高度，你可以用班里面最高的人（无穷范数）去比较，也可以用班里所有人的身高总和比较（一范数），也可以求平均（类似二范数）。

### 总结

**L-0范数**：用来统计向量中非零元素的个数。
**L-1范数**：向量中所有元素的绝对值之和。可用于优化中去除没有取值的信息，又称稀疏规则算子。
**L-2范数**：典型应用——欧式距离。可用于优化正则化项，避免过拟合。
**L-∞范数**：计算向量中的最大值。

------

## 矩阵范数

```matlab
A=[2, 3, -5, -7;
   4, 6, 8, -4;
   6,-11,-3,16];
```

1. 矩阵的1-范数（列模）,矩阵的每一列上的元素绝对值先求和，再从中取个最大的，（列和最大）；即矩阵A的1-范数为：27

2. 矩阵的2-范数（谱模）,其中   为的特征值；矩阵的最大特征值开平方根。

3. 矩阵的无穷范数（行模）,矩阵的每一行上的元素绝对值先求和，再从中取个最大的，（行和最大）

### 总结

1. 矩阵的核范数：矩阵的奇异值（将矩阵`svd`分解）之和，这个范数可以用来低秩表示（因为最小化核范数，相当于最小化矩阵的秩——低秩）；

2. 矩阵的`L0`范数：矩阵的非0元素的个数，通常用它来表示稀疏，`L0`范数越小0元素越多，也就越稀疏。

3. 矩阵的`L1`范数：矩阵中的每个元素绝对值之和，它是`L0`范数的最优凸近似，因此它也可以近似表示稀疏；

4. 矩阵的`F`范数：矩阵的各个元素平方之和再开平方根，它通常也叫做矩阵的`L2`范数，它有点在它是一个凸函数，可以求导求解，易于计算；

5.  矩阵的`L21`范数：矩阵先以每一列为单位，求每一列的F范数（也可认为是向量的2范数），然后再将得到的结果求`L1`范数（也可认为是向量的1范数），很容易看出它是介于`L1`和`L2`之间的一种范数



------

# 增广拉格朗日法

### 拉格朗日函数

- 对于优化问题
   ![[公式]](https://www.zhihu.com/equation?tex=%5Cmin+%5C%3B+f%28x%29)
   ![[公式]](https://www.zhihu.com/equation?tex=s.t.+%5C%3BAx%3Db)
- 把约束乘以一个系数放在目标函数上，得到拉格朗日函数为
   ![[公式]](https://www.zhihu.com/equation?tex=L%28x%2Cv%29%3Df%28x%29%2Bv%5ET%28Ax-b%29)
   

### **增广拉格朗日函数**

- Augmented Lagrangian Method进一步将约束的二范数平方放在了目标函数上，得到的增广拉格朗日函数为如下形式
   ![[公式]](https://www.zhihu.com/equation?tex=L_c%28x%2Cv%29%3Df%28x%29%2Bv%5ET%28Ax-b%29%2B%5Cfrac%7Bc%7D%7B2%7D%5C%7CAx-b%5C%7C_2%5E2)
- 以上![[公式]](https://www.zhihu.com/equation?tex=L_c)也是如下问题的拉格朗日函数
   ![[公式]](https://www.zhihu.com/equation?tex=%5Cmin%5C%3Bf%28x%29%2B%5Cfrac%7Bc%7D%7B2%7D%5C%7CAx-b%5C%7C_2%5E2)
   ![[公式]](https://www.zhihu.com/equation?tex=s.t.%5C%3B+Ax%3Db)

​     这个问题与原问题有着相同的最优解，增加2范数不会影响解的结果

### **增广拉格朗日法**

- Augmented Lagrangian Method, `ALM` 通过交替优化原变量与对偶变量求解问题
   ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7Bk%2B1%7D%3D%5Carg%5Cmin%5Climits_x+L_c%28x%2Cv%5Ek%29)
   ![[公式]](https://www.zhihu.com/equation?tex=v%5E%7Bk%2B1%7D%3Dv%5Ek%2Bc%28Ax%5E%7Bk%2B1%7D-b%29)
   其中![[公式]](https://www.zhihu.com/equation?tex=c%3D1)或是递增的序列
- **性质**：1）当![[公式]](https://www.zhihu.com/equation?tex=v%3Dv%5E%5Cstar)，则![[公式]](https://www.zhihu.com/equation?tex=%5Cforall+c%3E0)，![[公式]](https://www.zhihu.com/equation?tex=x%5E%5Cstar%3D%5Carg%5Cmin%5Climits_x+L_c%28x%2Cv%5E%5Cstar%29)
   ​         2）当![[公式]](https://www.zhihu.com/equation?tex=c%5Crightarrow+%5Cinfty)，则![[公式]](https://www.zhihu.com/equation?tex=%5Cforall+v)，![[公式]](https://www.zhihu.com/equation?tex=x%5E%5Cstar%3D%5Carg%5Cmin%5Climits_x+L_c%28x%2Cv+%29)

## **交替方向乘子法**--ADMM

### **方法的引出**

- 使用上面讲的增广拉格朗日法求解
   ![[公式]](https://www.zhihu.com/equation?tex=%5Cmin+f%28x%29%2Bg%28x%29)
   这里![[公式]](https://www.zhihu.com/equation?tex=f%28x%29%2Cg%28x%29)都是凸函数，此时目标函数有两个，整体优化可能不太方便，现在想要交替优化两项，考虑将这个优化问题写为
   ![[公式]](https://www.zhihu.com/equation?tex=%5Cmin%5C%3B+f%28x%29%2Bg%28z%29)
   ![[公式]](https://www.zhihu.com/equation?tex=s.t.%5C%3B+x%3Dz)

- 以上问题的增广拉格朗日函数为
   ![[公式]](https://www.zhihu.com/equation?tex=L_c%28x%2Cz%2Cv%29%3Df%28x%29%2Bg%28z%29%2Bv%5ET%28x-z%29%2B0.5c%5C%7Cx-z%5C%7C_2%5E2)
   
- 使用增广拉格朗日函数交替优化原变量![[公式]](https://www.zhihu.com/equation?tex=x%2Cz)，和对偶变量![[公式]](https://www.zhihu.com/equation?tex=v)，交替优化步骤如下
   1）![[公式]](https://www.zhihu.com/equation?tex=%5C%7Bx%5E%7Bk%2B1%7D%2Cz%5E%7Bk%2B1%7D%5C%7D%3D%5Carg%5Cmin%5Climits_%7Bx%2Cz%7Df%28x%29%2Bg%28z%29%2Bv%5ET%28x-z%29%2B0.5c%5C%7Cx-z%5C%7C_2%5E2)

   2）![[公式]](https://www.zhihu.com/equation?tex=v%5E%7Bk%2B1%7D%3Dv%5Ek%2Bc%28x%5E%7Bk%2B1%7D-z%5E%7Bk%2B1%7D%29)

- 在上面步骤1）中，优化了两个变量，现在把这一步运用分块坐标轮换法，进一步地交替优化![[公式]](https://www.zhihu.com/equation?tex=x%2Cz)，拆解为两步，并且将增广拉格朗日函数后两项配方法合并
  ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7Bk%2B1%7Ct%2B1%7D%3D%5Carg%5Cmin%5Climits_x+f%28x%29%2B%5Cfrac%7Bc%7D%7B2%7D%5C%7Cx-z%5E%7Bk%2B1%7Ct%7D%2B%5Cfrac%7Bv%5Ek%7D%7Bc%7D%5C%7C_2%5E2)
   ![[公式]](https://www.zhihu.com/equation?tex=z%5E%7Bk%2B1%7Ct%2B1%7D%3D%5Carg%5Cmin%5Climits_z+g%28z%29%2B%5Cfrac%7Bc%7D%7B2%7D%5C%7Cz-x%5E%7Bk%2B1%7Ct%2B1%7D-%5Cfrac%7Bv%5Ek%7D%7Bc%7D%5C%7C_2%5E2)

​    这两步多次交替迭代，然后再迭代一次对偶变量![[公式]](https://www.zhihu.com/equation?tex=v)，这种方法即是大名鼎鼎的**交替方向乘子法**(Alternating Direction Method of Multipliers, `ADMM`)。

### **ADMM举例**

ADMM算法一般用于解决如下的凸优化问题:

![[公式]](https://www.zhihu.com/equation?tex=min+f%28x%29%2Bg%28y%29+) 

![[公式]](https://www.zhihu.com/equation?tex=s.t.+Ax%2BBy%3Dc) 


其中， ![[公式]](https://www.zhihu.com/equation?tex=x%5Cin+R%5E%7Bn%7D) 为目标函数 ![[公式]](https://www.zhihu.com/equation?tex=f%28x%29) 的优化变量， ![[公式]](https://www.zhihu.com/equation?tex=y%5Cin+R%5E%7Bm%7D) 为目标函数 ![[公式]](https://www.zhihu.com/equation?tex=g%28y%29) 的优化变量， ![[公式]](https://www.zhihu.com/equation?tex=A%5Cin+R%5E%7Bp%5Ctimes+n%7D) ， ![[公式]](https://www.zhihu.com/equation?tex=B%5Cin+R%5E%7Bp%5Ctimes+m%7D) ， ![[公式]](https://www.zhihu.com/equation?tex=c%5Cin+R%5Ep) 。 函数 ![[公式]](https://www.zhihu.com/equation?tex=f) ， ![[公式]](https://www.zhihu.com/equation?tex=g) 是凸函数。


它的增广拉格朗日函数如下:
![[公式]](https://www.zhihu.com/equation?tex=L_%5Crho%28x%2Cy%2C%5Clambda%29%3Df%28x%29%2Bg%28y%29%2B%5Clambda%5ET%28Ax%2BBy-c%29%2B%28%5Crho%2F2%29%7C%7CAx%2BBy-c%7C%7C%5E2_2%2C%5Cquad+%5Crho%3E0) 

其中， ![[公式]](https://www.zhihu.com/equation?tex=%5Clambda) 称为拉格朗日乘子， ![[公式]](https://www.zhihu.com/equation?tex=%5Crho) 是惩罚参数且 ![[公式]](https://www.zhihu.com/equation?tex=%5Crho%3E0) 。此时，用`ADMM`算法进行求解，则过程如下：

![[公式]](https://www.zhihu.com/equation?tex=x%5E%7Bk%2B1%7D%3A%3DargminL_%5Crho%28x%2Cy%2C%5Clambda%29%5C%5C++++++y%5E%7Bk%2B1%7D%3A%3DargminL_%5Crho%28x%2Cy%2C%5Clambda%29%5C%5C+++++%5Clambda%3A%3D%5Clambda%5Ek%2B%5Crho%28Ax%5E%7Bk%2B1%7D%2BBy%5E%7Bk%2B1%7D-c%29) 

第一步简化：

通过公式 ![[公式]](https://www.zhihu.com/equation?tex=2a%5ETb%2B%7C%7Cb%7C%7C%5E2_2%3D%7C%7Ca%2Bb%7C%7C%5E2_2-%7C%7Ca%7C%7C%5E2_2) 替换增广拉格朗日函数中的线性项 ![[公式]](https://www.zhihu.com/equation?tex=%5Clambda%5ET%28Ax%2BBy-c%29) 和二次项 ![[公式]](https://www.zhihu.com/equation?tex=%5Crho%7C%7CAx%2BBy-c%7C%7C%5E2_2)

![[公式]](https://www.zhihu.com/equation?tex=%5Clambda%5ET%28Ax%2BBy-c%29+%2B+%5Crho%7C%7CAx%2BBy-c%7C%7C%5E2_2%3D%5Crho%2F2%7C%7CAX%2Bby-c%2B%5Crho%2F%5Clambda%7C%7C%5E2_2-%5Crho%2F2%7C%7C%5Clambda%2F%5Crho%7C%7C%5E2_2) 

于是`ADMM`求解过程可以简化为如下形式：

![[公式]](https://www.zhihu.com/equation?tex=x%5E%7Bk%2B1%7D%3A%3Dargmin%28f%28x%29%2B%5Crho%2F2%7C%7CAx%2BBy%5Ek-c%2B%5Clambda%5Ek%2F%5Crho%7C%7C%5E2_2%29%5C%5C+y%5E%7Bk%2B1%7D%3A%3Dargmin%28g%28y%29%2B%5Crho%2F2%7C%7CAx%5E%7Bk%2B1%7D%2BBy-c%2B%5Clambda%5Ek%2F%5Crho%7C%7C%5E2_2%29%5C%5C++%5Clambda%3A%3D%5Clambda%5Ek%2B%5Crho%28Ax%5E%7Bk%2B1%7D%2BBy%5E%7Bk%2B1%7D-c%29) 

第二步简化：

令缩放对偶变量为 ![[公式]](https://www.zhihu.com/equation?tex=u%3D%281%2F%5Crho%29%5Clambda) ，于是`ADMM`求解过程再次简化为如下形式：

 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7Bk%2B1%7D%3A%3Dargmin%28f%28x%29%2B%5Crho%2F2%7C%7CAx%2BBy%5Ek-c%2Bu%5Ek%7C%7C%5E2_2%29%5C%5C+y%5E%7Bk%2B1%7D%3A%3Dargmin%28g%28y%29%2B%5Crho%2F2%7C%7CAx%5E%7Bk%2B1%7D%2BBy-c%2Bu%5Ek%7C%7C%5E2_2%29%5C%5C+u%5E%7Bk%2B1%7D%3A%3Du%5Ek%2BAx%5E%7Bk%2B1%7D%2BBy%5E%7Bk%2B1%7D-c)

- **分布式计算：**从以上三步迭代可以看到：更新![[公式]](https://www.zhihu.com/equation?tex=x_i%2Cv_i)时，与![[公式]](https://www.zhihu.com/equation?tex=x_%7B-i%7D%2Cv_%7B-i%7D%2Cf_%7B-i%7D)无关，即![[公式]](https://www.zhihu.com/equation?tex=n)个下标索引的计算可以分配到![[公式]](https://www.zhihu.com/equation?tex=n)台计算机/线程/进程分别进行计算，只在更新![[公式]](https://www.zhihu.com/equation?tex=z)时，需要汇总各个计算节点的数据，更新![[公式]](https://www.zhihu.com/equation?tex=z)后，再将结果分发到各个计算节点。这也是`ADMM`的一大优点。
- 总结来说就是，构造增广拉格朗日函数，然后用交替求解计算(固定某些变量，再求解目标变量)，当达到某个阈值时，求解结束



------

# 内点法(Interior Method)

## 简介

内点法是一种处理带约束优化问题的方法，其在线性规划，二次规划，非线性规划等问题上都有着很好的表现。内点法或障碍法是解决线性和非线性凸优化问题的一类算法。通过在目标函数中增加一个障碍项来防止违反不等式约束，该障碍项导致最佳无约束值位于可行空间中。

- 线性规划单纯形法：通过一系列迭代达到最优解，迭代点沿着可行多面体的边界从一个顶点到另一个顶点，直到得到最优解。一般而言单纯形法每次迭代的开销相对内点法来说较小，但所需迭代次数较多。
- 线性规划内点法：同样是通过一系列迭代达到最优解，但其是从多面体内部逐渐收敛到最优解。一般而言内点法每次迭代的开销相对单纯形法来说较大，但所需迭代次数较少。

内点法并不仅仅用于线性规划的求解，值得一提的是内点法的很多思想有着更广泛的应用，例如障碍函数法的思想。线性规划问题的一般形式为

![[公式]](https://www.zhihu.com/equation?tex=%5Ctext%7Bmin+%7D+c%5E%7BT%7Dx+%5C%5C+%5Ctext%7Bs.t.+%7D+Ax+%5Cpreceq+b)

这里，目标函数为线性函数 ![[公式]](https://www.zhihu.com/equation?tex=c%5E%7BT%7Dx%3A+%5Cmathbb%7BR%7D%5E%7Bn%7D+%5Crightarrow+%5Cmathbb%7BR%7D) ，约束条件为 ![[公式]](https://www.zhihu.com/equation?tex=A_%7Bij%7Dx_%7Bj%7D+%5Cle+b_%7Bi%7D%2C+i+%3D+1%2C+2%2C+...%2C+m.+) 矩阵 ![[公式]](https://www.zhihu.com/equation?tex=A) 为 ![[公式]](https://www.zhihu.com/equation?tex=m%5Ctimes+n) 的满秩矩阵，其中 ![[公式]](https://www.zhihu.com/equation?tex=m) 为约束条件的个数， ![[公式]](https://www.zhihu.com/equation?tex=n) 为变量的个数。通常约束条件的个数大于变量的个数，所以有 ![[公式]](https://www.zhihu.com/equation?tex=m+%3E+n) .

## 障碍法(Barrier Method)

### **1.线性规划问题的等价(近似)表述**

这个线性规划问题可以重新表述为计算 ![[公式]](https://www.zhihu.com/equation?tex=%5Ctext%7Bmin+%7D+f%28x%29) ，其中

![[公式]](https://www.zhihu.com/equation?tex=f%28x%29+%3D+c%5E%7BT%7Dx+%2B+%5Csum_%7Bi+%3D+1%7D%5E%7Bm%7DI%28A_%7Bij%7Dx_%7Bj%7D+-+b_%7Bi%7D%29)

这里，我们使用了一个indicator函数，定义为

![[公式]](https://www.zhihu.com/equation?tex=%5C%5B+I%28u%29+%3D+%5Cbegin%7Bcases%7D+0+%26+%5Ctext%7Bif+%7D+x+%5Cle+0+%5C%5C+%5Cinfty+%26+%5Ctext%7Bif+%7D+x+%3E+0+%5Cend%7Bcases%7D+%5C%5D)

![image-20220425103451965](/home/next/.config/Typora/typora-user-images/image-20220425103451965.png)

引入这个函数的意义在于可以将约束条件直接写入到目标函数里面，这样我们直接求新的函数的极小值就可以了，而不必借助于未知乘子。 但是这里有一个问题，那就是indicator函数存在不可求导的点，因此在求函数极小值的时候我们没法通过普通的微分法来确定函数的极小值。为了规避这个问题，我们可以用一个光滑的函数来近似这个indicator函数。一个不错的选择是用 ![[公式]](https://www.zhihu.com/equation?tex=I_%7Bt%7D%28u%29+%3D+-%5Cfrac%7B1%7D%7Bt%7D%5Clog+%28-u%29) 来代替indicator函数。 ![[公式]](https://www.zhihu.com/equation?tex=I_%7Bt%7D%28u%29) 只有在 ![[公式]](https://www.zhihu.com/equation?tex=u+%3C+0) 的时候有定义，我们规定当 ![[公式]](https://www.zhihu.com/equation?tex=u+%3E+0) 的时候 ![[公式]](https://www.zhihu.com/equation?tex=I_%7Bt%7D%28u%29+%3D+%5Cinfty) . 而且参数 ![[公式]](https://www.zhihu.com/equation?tex=t+%3E+0) 越大，函数 ![[公式]](https://www.zhihu.com/equation?tex=I_%7Bt%7D%28u%29) 就越接近于 ![[公式]](https://www.zhihu.com/equation?tex=I%28u%29) . 所以我们可以通过调节 ![[公式]](https://www.zhihu.com/equation?tex=t) 的值来调节这个函数的近似程度。使用这个近似的indicator函数，我们新的的目标函数可以写作

![[公式]](https://www.zhihu.com/equation?tex=f%28x%29+%3Dt+c%5E%7BT%7Dx+-+%5Csum_%7Bi+%3D+1%7D%5E%7Bm%7D+%5Clog%28-A_%7Bij%7D+x_j+%2B+b_i%29) .

因为线性函数是凸函数，并且 ![[公式]](https://www.zhihu.com/equation?tex=I_%7Bt%7D%28u%29) 也是凸函数，所以 ![[公式]](https://www.zhihu.com/equation?tex=f%28x%29) 是凸函数，因此我们可以很容易用凸优化的经典方法得到该函数的极小值。

### **2.计算函数的梯度和 Hessian矩阵**

为了求函数的极小值，根据微积分的经典结果，只需令函数的梯度等于零，然后计算梯度为零时对应的解 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7B%5Cstar%7D) .

函数的梯度为

![[公式]](https://www.zhihu.com/equation?tex=%5Cfrac%7B%5Cpartial+f%7D%7B%5Cpartial+x_%7Bk%7D%7D+%3D+t+c_%7Bk%7D+%2B+%5Csum_%7Bi+%3D+1%7D%5E%7Bm%7D%5Cfrac%7B-A_%7Bik%7D%7D%7BA_%7Bij%7Dx_%7Bj%7D+-+b_%7Bi%7D%7D)

**Hessian**矩阵为

![[公式]](https://www.zhihu.com/equation?tex=%5Cfrac%7B%5Cpartial%5E2+f%7D%7B%5Cpartial+x_%7Bk%7D%5Cpartial+x_%7Bl%7D%7D+%3D+%5Csum_%7Bi+%3D+1%7D%5E%7Bm%7D%5Cfrac%7BA_%7Bik%7D+A_%7Bil%7D%7D%7B%5CBig%28A_%7Bij%7D+x_%7Bj%7D+-+b_%7Bi%7D+%5CBig%29%5E2%7D)

定义对角型矩阵为

![[公式]](https://www.zhihu.com/equation?tex=D_%7Bij%7D+%3D+%5Cdelta_%7Bij%7D+%5Cfrac%7B1%7D%7B%5CBig%28A_%7Bik%7Dx_%7Bk%7D+-+b_%7Bi%7D%5CBig%29%5E2%7D)

于是**Hessian**矩阵可以写作

![[公式]](https://www.zhihu.com/equation?tex=H_%7Bf%7D+%3D+A%5E%7BT%7D+D+A)

因为 ![[公式]](https://www.zhihu.com/equation?tex=D) 为正定矩阵，所以 `Hessian` 矩阵至少为半正定矩阵。所以函数 ![[公式]](https://www.zhihu.com/equation?tex=f%28x%29) 是一个凸函数。而且矩阵 ![[公式]](https://www.zhihu.com/equation?tex=D) 为可逆矩阵，矩阵 ![[公式]](https://www.zhihu.com/equation?tex=A) 满秩，所以 `Hessian` 矩阵为可逆矩阵。于是函数 ![[公式]](https://www.zhihu.com/equation?tex=f%28x%29) 为强凸函数。所以，要计算 ![[公式]](https://www.zhihu.com/equation?tex=%5Cnabla+f+%3D+0) 的根，我们可以用高效的**牛顿迭代法**。

### **3.牛顿迭代法**

> Newton Method求解近似问题的算法框架
>
> 重复进行：
>
> 1. 中心点步骤：
>    从初始值 ![[公式]](https://www.zhihu.com/equation?tex=x) 开始，采用Newton Method在 ![[公式]](https://www.zhihu.com/equation?tex=Ax%3Db) 的约束下极小化 ![[公式]](https://www.zhihu.com/equation?tex=tf_0+%2B+%5Cphi) ，最终确定 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%2A%28t%29) 
>
> 2. 改进： ![[公式]](https://www.zhihu.com/equation?tex=x%3A%3D+x%5E%2A%28t%29) 
>
> 3. 停止准则。 如果 ![[公式]](https://www.zhihu.com/equation?tex=m%2Ft+%3C+%5Cepsilon) 则退出
>
> 4. 增加 ![[公式]](https://www.zhihu.com/equation?tex=t) ： ![[公式]](https://www.zhihu.com/equation?tex=t%3A%3D+%5Cmu+t%2C+%5Cquad+%5Cmu%3E1) 
>
> 根据在中心路径小节中的分析，近似问题的最优解与原始问题的最优解的误差不超过 ![[公式]](https://www.zhihu.com/equation?tex=m%2Ft)，所以在求解时，我们从较小的 ![[公式]](https://www.zhihu.com/equation?tex=t) 开始，不断增大 ![[公式]](https://www.zhihu.com/equation?tex=t) 直至收敛。此外，我们把每次求出的最优解 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%2A%28t%29) 当作下一个 ![[公式]](https://www.zhihu.com/equation?tex=t%3A%3D+%5Cmu+t) 的初值，这样Newton Method会收敛更快，即Step 1.中心点步骤的运行时间可以当作常数对待，而不是一个耗时的迭代循环。

我们现在的目标是计算 ![[公式]](https://www.zhihu.com/equation?tex=%5Cnabla+f+%3D+0) 的根。因为Hessian矩阵可逆，所以我们可以用牛顿迭代法求解。牛顿迭代法为

![[公式]](https://www.zhihu.com/equation?tex=x%5E%7B%28n%2B1%29%7D+%3D+x%5E%7B%28n%29%7D+-+H%5E%7B-1%7D%5Cnabla+f)

在这个迭代过程中，参数 ![[公式]](https://www.zhihu.com/equation?tex=t) 为固定的。每对应一个 ![[公式]](https://www.zhihu.com/equation?tex=t) ，我们都可以得到一个解 ![[公式]](https://www.zhihu.com/equation?tex=x_%7Bt%7D%5E%7B%5Cstar%7D) . 如果我们扫描参数 ![[公式]](https://www.zhihu.com/equation?tex=t) ，我们就可以得到一系列的解。 其中最大的 ![[公式]](https://www.zhihu.com/equation?tex=t) 的对应的解应该最精确。

一个更好的算法是选取一个比较小的初始参数 ![[公式]](https://www.zhihu.com/equation?tex=t_%7B0%7D) ，求出这个参数对应的解 ![[公式]](https://www.zhihu.com/equation?tex=x_%7Bt_0%7D%5E%7B%5Cstar%7D) . 然后增加 ![[公式]](https://www.zhihu.com/equation?tex=t) ，用之前得到的解 ![[公式]](https://www.zhihu.com/equation?tex=x_%7Bt_0%7D%5E%7B%5Cstar%7D) 来初始化当前 ![[公式]](https://www.zhihu.com/equation?tex=t) 所对应的牛顿迭代法的试解。这样算出来的解应该比直接计算 ![[公式]](https://www.zhihu.com/equation?tex=t) 所对应的解更加精确。这样逐步迭代从小 ![[公式]](https://www.zhihu.com/equation?tex=t_%7B0%7D) 到大 ![[公式]](https://www.zhihu.com/equation?tex=t) 可以得到一系列的解，最后得到的解 ![[公式]](https://www.zhihu.com/equation?tex=x_%7Bt%7D%5E%7B%5Cstar%7D) 称作是淬火解。这个解应该可以满足我们的精度需求。

#### **牛顿法的补充**

**一维变量情况**

<img src="/home/next/.config/Typora/typora-user-images/image-20220423001930057.png" alt="image-20220423001930057" style="zoom: 80%;" />

**二维变量情况**

<img src="/home/next/.config/Typora/typora-user-images/image-20220423001959084.png" alt="image-20220423001959084" style="zoom: 105%;" />



## 原始-对偶法(Primal-Dual Method)

### 原问题

$$
\min c^Tx\\	
\quad \text{subject to } Ax=b\\ x\ge0 \tag9
$$

### 对偶问题

$$
\max b^Ty \\ 
\quad \text{subject to } A^Ty +s=c \\
s\ge0 \tag{10}
$$

### KKT条件

$$
\begin{align} 
A^Ty+s&=c 
\\ Ax&=b 
\\ x_i s_i &= 0,  i=1,...,n 
\\(x,s) &\ge0 
\end{align}
$$

其中
$$
X=\text{diag} (x_1,x_2,...,x_n),\quad S=\text{diag} (s_1,s_2,...,s_n), \quad e = (1,1,...1)^T
$$

- 上述`xs`已经是一个非线性系统，一种实用的方法是采用牛顿法
- 这里需要定义一个量来检验当前的迭代点与最优点(x,s)的差距。在Barrier Method中，使用 duality gap 的上界  `m/t`  来检验的，在 Primal Dual Method 中，定义一个新的 duality measure 来进行某种衡量

$$
u= \frac {1}{n} \cdot \sum_{i=1}^{n}x_{i}s_{i} = \frac {x^T}{n}
$$

​		上述 `u` 也被成为`对偶间隙`，当`u->0`时，`(x,s)`将接近可行域的边界，假设给定当前的可行点`(x,y,s)` ,寻找下一个点
$$
(x',y',s') = (x,y,s)+(\Delta x,\Delta y,\Delta s)
$$
​		使得如下条件成立
$$
\begin{align} 
A^Ty'+s'&=c,&s'>0 \\ 
Ax'&=b,	&x'>0 \\ 
x_i s_i &= 0,  &i=1,...,n \\
x'_{i}s'_{i}=\sigma u, &&i=1,...,n ;0 < σ < 1
\end{align}
$$
​		上述问题也被成为**扰动`kkt`条件**，可进一步简化转化为矩阵形式为
$$
\begin{bmatrix} 
 A & 0 & 0 \\ 0 &A^T &I \\ S & 0 &X \end{bmatrix}\begin{bmatrix}  \Delta x\\ \Delta y \\ \Delta s\end{bmatrix}=\begin{bmatrix}  -r_b \\ -r_c \\ -XSe+\sigma\mu e\end{bmatrix}
$$
​		其中
$$
r_b=Ax-b,\quad r_c=A^Ty+s-c
$$
​		可以看出，Barrier Method 中控制 t 使得 duality gap 为 `m/t`，而在Primal Dual 内点法中控制 `σμ` ，二者是一致的。

-  当 `σμ` 小于一定阈值，则计算结束

  

## Ipopt 求解思路

### 原问题

![image-20220423130324306](/home/next/.config/Typora/typora-user-images/image-20220423130324306.png)

#### 1.处理不等式约束

作为内点法会将不等式问题装换为等式问题, 会构造了一个 `ln(x)` 的障碍函数，来代替`x>=0`

![image-20220423152221182](/home/next/.config/Typora/typora-user-images/image-20220423152221182.png)

当 `xi` 小于零时，目标函数值为无穷大，因此，原问题的最优解任然会在 `x>=0` 的定义域内。 障碍函数的近似度项取决于障碍参数 μ 的大小，即在 `u -> 0` 时，新构造的问题最优解会收敛到原问题的最优解为 。因此，求解新问题的思路就是，通过不同的取值`u`，解决一系列不同的障碍函数构成的问题(**由u的取值不同决定**)以及用户指定的起始点，不断迭代u,直到满足`KKT`条件，那么问题得到解决。

#### 2.处理等式约束

需要满足如下`KKT`条件
$$
增广拉格朗日函数：L(x,y,z)  = f(x) +c(x)y+u \cdot\sum_{i=0}^{n-1}ln(x)
$$
![image-20220423152152095](/home/next/.config/Typora/typora-user-images/image-20220423152152095.png)

其中 `令Z = u / X` , `e`为单位列向量，所以`z`也需要被纳入优化变量中，由(`5c`)可推断出这是个非线性系统，应为是x，z两个变量的乘积，一般采用牛顿法来求解。

##### 2.1 将`KKT`条件转换为矩阵形式求解

Newton迭代的步长 `step (∆xk, ∆yk, ∆zk)`

![image-20220423132103751](/home/next/.config/Typora/typora-user-images/image-20220423132103751.png)

 其中![image-20220423133033864](/home/next/.config/Typora/typora-user-images/image-20220423133033864.png)![image-20220423133945961](/home/next/.config/Typora/typora-user-images/image-20220423133945961.png)

#### 3.评估结果

​	**3.1 首先判断走过步长后的值，是否增加了一个足够大的值**

![image-20220423132132205](/home/next/.config/Typora/typora-user-images/image-20220423132132205.png)

​	**3.2 如何满足条件，则更新(α的确定由过滤器类来确定)**

![image-20220423132151328](/home/next/.config/Typora/typora-user-images/image-20220423132151328.png)

#### 4.判断收敛准则

当容差满足 `kkt` 条件的时收敛

<img src="/home/next/.config/Typora/typora-user-images/image-20220423145609428.png" alt="image-20220423145609428" style="zoom:60%;" />

#### 计算结果输出

| 参数                | 意义                                |
| ------------------- | ----------------------------------- |
| iter                | 迭代次数                            |
| objective           | 目标函数的当前值 (max-norm)         |
| inf_pr              | 当前原始不可行性（最大范数）        |
| inf_du              | 当前双重不可行性（最大范数）        |
| lg(mu)              | `log10` 的障碍函数的参数 u          |
| \|\|d\|\|           | 原始搜索方向的无穷范数              |
| lg(rg)              | log10 的 Hessian 扰动 δx            |
| alpha_du , 双步长 α | 对偶步长αk                          |
| alpha_pr            | 原始步长αx                          |
| ls                  | 回溯步数(α的大小要依赖于之前的数据) |

![image-20220423152056135](/home/next/.config/Typora/typora-user-images/image-20220423152056135.png)

### `warm_start`

在求解之前我们往往对最优点处的有效约束了解很少,其实在一些应用中，我们需要求解一系列类似的 `QP` 命题，这个时候我们往往对最优点处的有效约束有一个初始猜测，因此通过这种方式可以实现算法的热启动（Warm Start），从而加速算法的收敛

## example

这个例子可以用简单的几何方法求解出来。用数值方法可以得到同样的结果（在一定的精度范围之内）。之所以要用数值方法求解这个简单的问题，是因为数值方法对更复杂的问题同样有效，而简单的几何方法对复杂问题却已经不适用了。

### 例 1

 ![[公式]](https://www.zhihu.com/equation?tex=min+%5Cfrac%7B1%7D%7B12%7D%28x_%7B1%7D%2B1%29%5E3%2Bx_%7B2%7D)

![[公式]](https://www.zhihu.com/equation?tex=s.t+) ![[公式]](https://www.zhihu.com/equation?tex=x_%7B1%7D-1+%5Cgeq0);![[公式]](https://www.zhihu.com/equation?tex=x_%7B2%7D%5Cgeq0)

令 ![[公式]](https://www.zhihu.com/equation?tex=G%28x%2Cr_%7Bk%7D%29%3D%5Cfrac%7B1%7D%7B12%7D%28x_%7B1%7D%2B1%29%5E3%2Bx_%7B2%7D%2Br_%7Bk%7D%28%5Cfrac%7B1%7D%7Bx_%7B1%7D-1%7D%2B%5Cfrac%7B1%7D%7Bx_%7B2%7D%7D%29) ![[公式]](https://www.zhihu.com/equation?tex=%281%29)

用解析法求解：（其实就是高数中的求偏导并令其等于零）

![[公式]](https://www.zhihu.com/equation?tex=%5Cfrac%7B%E2%88%82G%28x%29%7D%7B%E2%88%82x_1%7D%3D%5Cfrac%7B1%7D%7B4%7D%28x_%7B1%7D%2B1%29%5E2-%5Cfrac%7Br_%7Bk%7D%7D%7Bx_%7B1%7D-1%7D%3D0) ![[公式]](https://www.zhihu.com/equation?tex=%282%29)

![[公式]](https://www.zhihu.com/equation?tex=%5Cfrac%7B%E2%88%82G%28x%29%7D%7B%E2%88%82x_2%7D%3D1-%5Cfrac%7Br_%7Bk%7D%7D%7Bx%5E2_%7B2%7D%7D+%3D+0) ![[公式]](https://www.zhihu.com/equation?tex=%283%29)

由 ![[公式]](https://www.zhihu.com/equation?tex=%281%29) 式可得， ![[公式]](https://www.zhihu.com/equation?tex=x_1+%3D+%5Csqrt%7B1%2B2%5Csqrt%7Br_k%7D%7D) ; 由 ![[公式]](https://www.zhihu.com/equation?tex=%282%29) 式可得， ![[公式]](https://www.zhihu.com/equation?tex=x_2+%3D+%5Csqrt%7Br_k%7D) ,故：

![[公式]](https://www.zhihu.com/equation?tex=%5Cbar%7Bx_%7Br_%7Bk%7D%7D%7D+%3D+%28x_%7B1%7D%2Cx_%7B2%7D%29+%3D+%28%5Csqrt%7B1%2B2%5Csqrt%7Br_k%7D%7D%2C%5Csqrt%7Br_k%7D%29)

当 ![[公式]](https://www.zhihu.com/equation?tex=r_k%5Crightarrow0) 时， ![[公式]](https://www.zhihu.com/equation?tex=%5Cbar%7Bx_%7Br_k%7D%7D%5Crightarrow%5Cbar%7Bx%7D%3D%281%2C0%29) , ![[公式]](https://www.zhihu.com/equation?tex=%5Cbar%7Bx%7D) 就是原问题的最优解。



### 例 2

![[公式]](https://www.zhihu.com/equation?tex=%5Ctext%7Bmin+%7D+x+%2B+y+%5C%5C+%5Ctext%7Bs.t.+%7D%5C%5C+x+%2B+2y+%5Cle+1+%5C%5C+2x+%2B+y+%5Cle+1+%5C%5C+x+%5Cge+0+%5C%5C+y+%5Cge+0)

也就是

![[公式]](https://www.zhihu.com/equation?tex=c+%3D+%5Cbegin%7Bpmatrix%7D+1+%5C%5C+1+%5Cend%7Bpmatrix%7D) , ![[公式]](https://www.zhihu.com/equation?tex=A+%3D+%5Cbegin%7Bpmatrix%7D+1+%26+2+%5C%5C+2+%26+1+%5C%5C+-1+%26+0+%5C%5C+0+%26+-1+%5Cend%7Bpmatrix%7D) , ![[公式]](https://www.zhihu.com/equation?tex=b+%3D+%5Cbegin%7Bpmatrix%7D+1+%5C%5C+1+%5C%5C+0+%5C%5C+0+%5Cend%7Bpmatrix%7D) .

梯度为

![[公式]](https://www.zhihu.com/equation?tex=%5Cnabla+f%28x%29+%3D+tc+-+A%5E%7BT%7D+%5Cbegin%7Bpmatrix%7D+%5Cfrac%7B1%7D%7Bx+%2B+2y+-1%7D%5C%5C+%5Cfrac%7B1%7D%7B2x+%2B+y+-1%7D+%5C%5C+%5Cfrac%7B1%7D%7B-x%7D+%5C%5C+%5Cfrac%7B1%7D%7B-y%7D+%5Cend%7Bpmatrix%7D)

Hessian矩阵为

![[公式]](https://www.zhihu.com/equation?tex=H_%7Bf%7D%28x%29+%3D+A%5E%7BT%7DDA)

其中，对角型矩阵 ![[公式]](https://www.zhihu.com/equation?tex=D) 为

![[公式]](https://www.zhihu.com/equation?tex=D+%3D+%5Cbegin%7Bpmatrix%7D+%5Cfrac%7B1%7D%7B%28x+%2B+2y+-+1%29%5E2+%7D+%26+0+%26+0+%26+0+%5C%5C+0+%26+%5Cfrac%7B1%7D%7B%282x+%2B+y+-+1%29%5E2+%7D+%26+0+%26+0+%5C%5C+0%26+0+%26+%5Cfrac%7B1%7D%7Bx%5E2%7D+%26+0+%5C%5C+0+%26+0+%26+0+%26+%5Cfrac%7B1%7D%7By%5E2%7D+%5Cend%7Bpmatrix%7D)

对于一个固定的参数 ![[公式]](https://www.zhihu.com/equation?tex=t) ，选择一个恰当的初始解 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7B%280%29%7D) ，代入牛顿迭代公式

![[公式]](https://www.zhihu.com/equation?tex=x%5E%7B%28n%2B1%29%7D+%3D+x%5E%7B%28n%29%7D+-+H_%7Bf%7D%28x%5E%7B%28n%29%7D%29%5E%7B-1%7D%5Cnabla+f%28x%5E%7B%28n%29%7D%29%2C+n+%5Cge+0)

可以得到一个依赖于参数 ![[公式]](https://www.zhihu.com/equation?tex=t) 的解 ![[公式]](https://www.zhihu.com/equation?tex=x%5E%7B%5Cstar%7D_%7Bt%7D) .

用几何方法很容易求得这个例子的解为 ![[公式]](https://www.zhihu.com/equation?tex=%28x%5E%7B%5Cstar%7D%2C+y%5E%7B%5Cstar%7D+%29+%3D+%280%2C+0%29) . 所以我们期待，当 ![[公式]](https://www.zhihu.com/equation?tex=%5Clim_%7Bt+%5Crightarrow+%5Cinfty%7D+%28x_%7Bt%7D%5E%7B%5Cstar%7D%2C+y_%7Bt%7D%5E%7B%5Cstar%7D%29+%3D+%280%2C+0%29) .

# 总结

- **对于最简单的无约束凸优化问题**，可以采用Gradient Descent或者Newton Method求解
- **对于稍复杂的等式约束凸优化问题**，可以对目标函数进行二阶Taylor近似，然后采用Newton Method求解`KKT`最优条件；
- **对于最复杂的带不等式约束问题**，则时引入对数障碍函数，转化为带等式约束的凸优化问题，然后采用Newton Method方法迭代求解。



# 速度优化

## DP搜索

通过DP搜索出一系列点集合，相当于一条粗略的轨迹

![image-20220417222104431](/home/next/.config/Typora/typora-user-images/image-20220417222104431.png)

针对Dp搜索出来的轨迹进行平滑，但是由于st图在构建时，在距离自车较近的范围内对s的采样间隔为0.1,而离自车较远的距离的采样间隔为1.0

## piecewise_jerk_speed_nonlinear_optimizer

### 流程

- **OptimizeByQP**

首先对DP采集到的点集合进行平滑(按0.1采样)(QP求解) ，为下面的非线性优化提供位置，速度，加速度的迭代初始值
$$
w_{speed}\cdot \sum_{i=0}^{n-1} s_i^2 + w'_{speed}\cdot \sum_{i=0}^{n-1} {s'_{i}}^2 + w''_{speed}\cdot \sum_{i=0}^{n-1} {s''_{i}}^2 + w'''_{speed}\cdot \sum_{i=0}^{n-2}(\frac{{s_{i+1}}'' - {s_i}''}{\Delta s})^2 \\
$$

- **CheckSpeedLimitFeasibility**
  限速检查，会检查规划周期的第一个点是否满足当前道路的的限速要求

  - 曲率曲线平滑`SmoothPathCurvature`(`QP`求解)
    $$
    w_{k}\cdot \sum_{i=0}^{n-1} x_i^2 + w'_{k}\cdot \sum_{i=0}^{n-1} {x'_{i}}^2 + w''_{k}\cdot \sum_{i=0}^{n-1} {x''_{i}}^2 + w'''_{k}\cdot \sum_{i=0}^{n-2}(\frac{{x_{i+1}}'' - {x_i}''}{\Delta s})^2 \\
    $$

  - 限速曲线平滑`SmoothSpeedLimit`(`QP`求解)
    $$
    w_{sl}\cdot \sum_{i=0}^{n-1} x_i^2 + w'_{sl}\cdot \sum_{i=0}^{n-1} {x'_{i}}^2 + w''_{sl}\cdot \sum_{i=0}^{n-1} {x''_{i}}^2 + w'''_{sl}\cdot \sum_{i=0}^{n-2}(\frac{{x_{i+1}}'' - {x_i}''}{\Delta s})^2 \\
    $$

#### 	为什么要对曲率和限速曲线进行平滑？

​		曲率曲线和限速曲线都是对纵向位移s的函数，而优化的变量正是s，所以在二次规划之前通过动态规划的粗轨迹提前计算出每个时间t时的曲率，得到每个时间t的曲率惩罚权重p。

- OptimizeByNLP 非线性优化入口

  ```c++
  ptr_interface->set_safety_bounds(s_bounds_);
  
  // Set weights and reference values
  const auto& config = config_.piecewise_jerk_nonlinear_speed_config();
  
  ptr_interface->set_curvature_curve(smoothed_path_curvature_);
  
  // TODO(Hongyi): add debug_info for speed_limit fitting curve
  ptr_interface->set_speed_limit_curve(smoothed_speed_limit_);
  ```

  - 传入st图中的可行上下边界到对象中

  - 传入上一步 QP 平滑后的曲率曲线和速度曲线

  - 设置优化变量 s 的参考值，如果没有使用dp优化的参考线，就直接把优化变量个数和路径总长传入，开st空间内直接搜索

    ```c++
    if (FLAGS_use_smoothed_dp_guide_line) {
        ptr_interface->set_reference_spatial_distance(*distance);
        // TODO(Jinyun): move to confs
        ptr_interface->set_w_reference_spatial_distance(10.0);
      } else {
        std::vector<double> spatial_potantial(num_of_knots_, total_length_);
        ptr_interface->set_reference_spatial_distance(spatial_potantial);
        ptr_interface->set_w_reference_spatial_distance(
            config.s_potential_weight());
      }
    ```

  - 设置**wart_start**加入求解

    ```c++
    void PiecewiseJerkSpeedNonlinearIpoptInterface::set_warm_start(
        const std::vector<std::vector<double>> &speed_profile) {
      x_warm_start_ = speed_profile;
    }
    ```

  - 设置优化变量的权重，包括 `s_bound`、加速度、加加速度、曲率、参考速度

- Ipopt 求解

  

### 数学建模

#### 优化变量

变量包含了每个固定间隔时间点的位置、速度、加速度。此外非线性规划中如果打开了软约束`FLAGS_use_soft_bound_in_nonlinear_speed_opt`，还会有几个关于s软约束的松弛变量
$$
s_0,\cdots s_{n-1},s'_0 \cdots s'_{n-1},s''_0,\cdots, s''_{n-1},s_{lowerSlack(0)}, \cdots s_{lowerSlack(n-1)},,s_{upperSlack(0)}, \cdots s_{upperSlack(n-1)},
$$


#### 代价函数

$$
cost\ \ function= 
w_{ref}\cdot \sum_{i=0}^{n-1} (s_i - s_{refi})^2 + w'_{ref}\cdot \sum_{i=0}^{n-1} {(s'_i - s'_{refi})}^2 + w''_{ref}\cdot \sum_{i=0}^{n-1} {s''_{i}}^2 + w'''_{ref}\cdot \sum_{i=0}^{n-2}(\frac{{s''_{i+1}} - {s''_i}}{\Delta t})^2 + \\

w_{centrAcc}\cdot \sum_{i=0}^{n-1} ({s'_{i}}^2 \cdot kappa(s_i))^2 + \\

w_{end} \cdot \sum_{i=0}^{n-1} ({s_{end}- s_{n-1}})^2 + w'_{end} \cdot \sum_{i=0}^{n-1} ({s'_{end}-s'_{n-1}})^2 + w''_{end} \cdot \sum_{i=0}^{n-1} ({s''_{end}-s''_{n-1}})^2 + \\w_{s} \sum_{i=0}^{n-1} {s_{lowerSlack}} +w_{s} \sum_{i=0}^{n-1} {s_{upperSlack}}
$$

> 原因一是引入松弛变量后的问题与原问题等价，最优解不改变，不等式![[公式]](https://www.zhihu.com/equation?tex=x_1%2Bx_2%5Cle+0)通过引入松弛变量![[公式]](https://www.zhihu.com/equation?tex=s_1)后变为![[公式]](https://www.zhihu.com/equation?tex=x_1%2Bx_2%2Bs_1+%3D+0)且![[公式]](https://www.zhihu.com/equation?tex=s_1%5Cge+0)，引入前后完全等价。原因二是通过引入松弛变量可以将原问题可行域扩展到更大的空间。。

#### 约束

- 对单调性的约束(车只允许向前开)

$$
0\leq s_{i+1}-s_{i} \leq s'_{i} \cdot \Delta t
$$

- 对`Jerk`的约束

$$
-s'''_{i_{max}}\leq \frac {s''_{i}-s''_{i}}{\Delta t}  \leq s'''_{i_{max}}
$$

- 连续性约束(位置相等 & 速度相等)

$$
s_{i+1}' - s_i' -\frac{1}{2}\Delta t *s_i'' - \frac{1}{2}\Delta t *s_{i+1}'' = 0\\
s_{i+1} - s_i - \Delta t \cdot s_i' - \frac{1}{3} \Delta t ^2 \cdot s_i'' - \frac{1}{6} \Delta t^2 \cdot s_{i+1}'' = 0
$$

- 对`speed_limit`的约束(非线性)

$$
s'_{i}-vboundfunc(s_{i}) \leq 0.0
$$

- 设置安全边界(加入松弛因子,合并到代价函数中)

$$
soft_lower_{i} - lowerslack_{i}\leq s_{i} \leq softupper_{i} +lowerslack_{i}
$$

#### 目标函数的梯度

原始
$$
\frac {\delta costfunc}{\delta s_i} =2\cdot w_{ref}\cdot(s_i - s_{refi})+2\cdot w_{centrAcc}\cdot s^5_{i}\cdot kappa(s_i)\cdot kappa'(s_i)
$$
一阶
$$
\frac {\delta costfunc}{\delta s'_i} =2\cdot w'_{ref}\cdot (s'_{i}-s'_{refi})+4\cdot w_{centrAcc}\cdot s^3_{i}\cdot kappa^2(s_i)
$$
二阶
$$
\frac {\delta costfunc}{\delta s''_i} =2 \cdot w''_{ref}\cdot s''_{i} -2\cdot w''_{rfe}\cdot \frac {s''_{i+1}- s'_{i}}{\Delta t^2}
$$
松弛变量
$$
\frac {\delta costfunc}{\delta s_{lowerslack}} =s_{lowerSlack} \\ 
\frac {\delta costfunc}{ds_{upperslack}} = s_{upperSlack}
$$


#### 约束条件的雅克比矩阵

对于速度约束：

![[公式]](https://www.zhihu.com/equation?tex=g_1%3D%5Cdot+s_i+-+speed%5C_limit%28s_i%29%5C%5C+%5Cfrac%7B%5Cdelta+g_1%7D%7Bd%5Cdot+s_i%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_1%7D%7Bd+s_i%7D%3D-speed%5C_limit%28s_i%29++)

对于位置软约束：

![[公式]](https://www.zhihu.com/equation?tex=g_2%3Ds_i-s%5C_lower_i%2Bslack%5C_lower_i+%5C%5C+%5Cfrac%7B%5Cdelta+g_2%7D%7Bd+s_i%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_2%7D%7Bd+slack%5C_lower_i%7D%3D1+)

![[公式]](https://www.zhihu.com/equation?tex=g_3%3Ds_i-s%5C_upper_i%2Bslack%5C_upper_i+%5C%5C+%5Cfrac%7B%5Cdelta+g_2%7D%7Bd+s_i%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_2%7D%7Bd+slack%5C_upper_i%7D%3D-1+)

对于jerk约束：

![[公式]](https://www.zhihu.com/equation?tex=g_3%3D%5Cfrac%7B%5Cddot+s_%7Bi%2B1%7D-%5Cddot+s_i%7D%7B%5CDelta+t%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_3%7D%7Bd+%5Cddot+s_i%7D%3D%5Cfrac%7B-1%7D%7B%5CDelta+t%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_3%7D%7Bd+%5Cddot+s_%7Bi%2B1%7D%7D%3D%5Cfrac%7B1%7D%7B%5CDelta+t%7D%5C%5C)

微分关系的等式约束

![[公式]](https://www.zhihu.com/equation?tex=g_4%3D+s_%7Bi%2B1%7D-s_i-%5CDelta+t+%5Cdot+s_i-%5Cfrac%7B%5CDelta+t%5E2%5Cddot+s_i%7D%7B3%7D-%5Cfrac%7B%5CDelta+t%5E3%5Cddot+s_%7Bi%2B1%7D%7D%7B6%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_4%7D%7Bd++s_i%7D%3D-1%5C%5C+%5Cfrac%7B%5Cdelta+g_4%7D%7Bd++s_%7Bi%2B1%7D%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_4%7D%7Bd++dot+s_%7Bi%7D%7D%3D-%5CDelta+t%5C%5C+%5Cfrac%7B%5Cdelta+g_4%7D%7Bd+%5Cddot+s_%7Bi%7D%7D%3D-%5Cfrac%7B%5CDelta+t%5E2%7D%7B3%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_4%7D%7Bd+%5Cddot+s_%7Bi%2B1%7D%7D%3D-%5Cfrac%7B%5CDelta+t%5E3%7D%7B6%7D%5C%5C)

![[公式]](https://www.zhihu.com/equation?tex=g_5%3D+%5Cdot+s_%7Bi%2B1%7D-%5Cdot+s_i-%5CDelta+t+%5Cddot+s_%7Bi%7D-%5Cfrac%7B%5CDelta+t+%28%5Cddot+s_%7Bi%2B1%7D-%5Cddot+s_%7Bi%7D%29%7D%7B2%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_5%7D%7Bd++%5Cdot+s_i%7D%3D-1%5C%5C+%5Cfrac%7B%5Cdelta+g_5%7D%7Bd++%5Cdot+s_%7Bi%2B1%7D%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_5%7D%7Bd+%5Cddot+s_%7Bi%7D%7D%3D-+%5Cfrac%7B%5CDelta+t%7D%7B2%7D%5C%5C+%5Cfrac%7B%5Cdelta+g_5%7D%7Bd+%5Cddot+s_%7Bi%2B1%7D%7D%3D-%5Cfrac%7B%5CDelta+t%7D%7B2%7D%5C%5C)

对于位置约束

![[公式]](https://www.zhihu.com/equation?tex=g_6%3Ds_%7Bi%2B1%7D-s_i%5C%5C+%5Cfrac%7B%5Cdelta+g_6%7D%7Bds_%7Bi%2B1%7D%7D%3D1%5C%5C+%5Cfrac%7B%5Cdelta+g_6%7D%7Bds_i%7D%3D-1)



## Ipopt求解最优化问题

### 原始-对偶法(Primal-Dual Method)

#### 原问题

$$
\min c^Tx\\	
\quad \text{subject to } Ax=b\\ x\ge0 \tag9
$$

#### 对偶问题

$$
\max b^Ty \\ 
\quad \text{subject to } A^Ty +s=c \\
s\ge0 \tag{10}
$$

![image-20220425101710813](/home/next/.config/Typora/typora-user-images/image-20220425101710813.png)

kkt条件
$$
\begin{align} 
A^Ty+s&=c 
\\ Ax&=b 
\\ x_i s_i &= 0,  i=1,...,n 
\\(x,s) &\ge0 
\end{align}
$$

其中
$$
X=\text{diag} (x_1,x_2,...,x_n),\quad S=\text{diag} (s_1,s_2,...,s_n), \quad e = (1,1,...1)^T
$$

- 上述`xs`已经是一个非线性系统，一种实用的方法是采用牛顿法
- 这里需要定义一个量来检验当前的迭代点与最优点(x,s)的差距。在Barrier Method中，使用 duality gap 的上界  `m/t`  来检验的，在 Primal Dual Method 中，定义一个新的 duality measure 来进行某种衡量

$$
u= \frac {1}{n} \cdot \sum_{i=1}^{n}x_{i}s_{i} = \frac {x^T}{n}
$$

​		上述 `u` 也被成为 `对偶间隙`，当`u->0`时，`(x,s)`将接近可行域的边界，假设给定当前的可行点`(x,y,s)` ,寻找下一个点
$$
(x',y',s') = (x,y,s)+(\Delta x,\Delta y,\Delta s)
$$
​		使得如下条件成立
$$
\begin{align} 
A^Ty'+s'&=c,&s'>0 \\ 
Ax'&=b,	&x'>0 \\ 
x_i s_i &= 0,  &i=1,...,n \\
x'_{i}s'_{i}=\sigma u, &&i=1,...,n ;0 < σ < 1
\end{align}
$$
​		上述问题也被成为**扰动`kkt`条件**，可进一步简化转化为矩阵形式为
$$
\begin{bmatrix} 
 A & 0 & 0 \\ 0 &A^T &I \\ S & 0 &X \end{bmatrix}\begin{bmatrix}  \Delta x\\ \Delta y \\ \Delta s\end{bmatrix}=\begin{bmatrix}  -r_b \\ -r_c \\ -XSe+\sigma\mu e\end{bmatrix}
$$
​		其中
$$
r_b=Ax-b,\quad r_c=A^Ty+s-c
$$
​		可以看出，Barrier Method 中控制 t 使得 duality gap 为 `m/t`，而在Primal Dual 内点法中控制 `σμ` ，二者是一致的。

- 当 `σμ` 小于一定阈值，则计算结束

  ![image-20220425101917410](/home/next/.config/Typora/typora-user-images/image-20220425101917410.png)

#### 牛顿迭代法

上述矩阵形式不是线性方程组，需要用到牛顿迭代法求解

**一维变量情况**

<img src="/home/next/.config/Typora/typora-user-images/image-20220423001930057.png" alt="image-20220423001930057" style="zoom: 70%;" />

**二维变量情况**

<img src="/home/next/.config/Typora/typora-user-images/image-20220423001959084.png" alt="image-20220423001959084" style="zoom: 90%;" />





## Ipopt 接口

### **get_nlp_info** 

对该函数对优化问题的维度进行设置

- n=4，变量x个数

- m=2，约束条件个数

- nnz_jac_g=8，Jacobian非零个数

- Nnz_h_lag = 10，Hessian非零个数


### **get_bounds_info** 

设置优化变量的上下限范围以及约束条件的上下限值

- x_l[i]设置xi的下界值

- x_u[i]设置xi的上界值

- g_l[i]设置约束i的下界值

- g_u[i]设置约束i的上界值


### get_start_point 

设置优化变量的初始点

- x[i]设置第i个变量的初始迭代值


### eval_f 

设置代价函数

- object_value 设置目标函数计算方式


### eval_grad_f

设置目标函数的梯度

-  grad_f[i]设置目标函数对第i个变量的偏导


### eval_g

设置约束条件

- g[i]约束条件i


### eval_jac_g

设置约束条件的雅克比矩阵

- `iRow`和`jCol`设置非零行列的坐标
- Values设置矩阵迭代值，如果values==NULL，即尚未初始化时，需要设置雅克比矩阵哪些下标位置非零


### eval_h

设置二阶梯度海森矩阵

- `iRow`和`jCol`设置非零行列的坐标
- obj_factor为目标函数系数

- lambda[i]为第i个约束的拉格朗日乘子

- values设置矩阵的迭代求值


### finalize_solution

求解

- status为返回的求解状态
- obj_value最优值

- x:最优解变量取值

- z_l 拉格朗日乘子上界

- z_u 拉格朗日乘子下届

- lambda 最优解拉格朗日乘子取值
