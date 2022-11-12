# 1 目录概述

***routing_planning/Astart改进***

  针对A*算法做出优化：加入靠近路沿的启发函数，并对生成的轨迹点做了均值滤波处理，使轨迹更加平滑。


## 2 算法介绍

### ***2.1 Astart改进***

```shell
 编译：g++ -std=c++11  xxx.cpp -o xx $(pkg-config --cflags --libs opencv) （需要安装opencv）
```

**实现思路：**

先用opencv将图片做灰度处理，再做二值化，将像素保存到vector二维数组作为地图，设置起点和终点，调用AStart算法(改进版：加入路沿代价函数)找到一条路径，由于算法会导致路径出现锯齿状，故用均值滤波对路径点做平滑处理。

**算法流程：**

1. 原始地图:

<img src="https://user-images.githubusercontent.com/68492981/132976491-de0eb792-02cf-4d98-a0cc-24c78338121e.jpg" alt="mapload4" style="zoom:80%;" />

2. A*算法生成的路径不平滑且贴近路沿，故增加道路膨胀层并加入靠近路沿的启发函数:

<img src="https://user-images.githubusercontent.com/68492981/133076047-7c432bd4-a349-4288-8f30-e6b61ddbc2e9.jpg" alt="loadToMap1" style="zoom: 300%;" />         <img src="https://user-images.githubusercontent.com/68492981/132976596-99eee2ee-7b96-464c-9700-36805340588b.jpg" alt="loadToMap4" style="zoom: 80%;" />

3. 利用均值滤波对路径做平滑处理并加大膨胀半径:

<img src="https://user-images.githubusercontent.com/68492981/132976579-f1298c8a-17c5-4eeb-8fc4-a1b2bfde91ae.jpg" alt="loadToMap7" style="zoom:80%;" />