g++编译:g++ -std=c++11  xxx.cpp -o xx $(pkg-config --cflags --libs opencv) （需要安装opencv）

原始地图:

![mapload4](https://user-images.githubusercontent.com/68492981/132976491-de0eb792-02cf-4d98-a0cc-24c78338121e.jpg)

A*算法生成的路径不平滑且太靠近路沿:

![loadToMap4](https://user-images.githubusercontent.com/68492981/132976596-99eee2ee-7b96-464c-9700-36805340588b.jpg)

运用均值滤波对路径做平滑处理且加入靠近路沿的启发函数:

![loadToMap7](https://user-images.githubusercontent.com/68492981/132976579-f1298c8a-17c5-4eeb-8fc4-a1b2bfde91ae.jpg)


