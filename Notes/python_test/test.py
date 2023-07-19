import matplotlib.pyplot as plt

i = 0
x = []
y = []
while i < 50:
    plt.clf()  # 清除上一幅图像
    x.append(i)
    y.append(i ** 2)
    plt.plot(x, y)
    i = i + 1
    plt.pause(0.01)  # 暂停0.01秒
    plt.ioff()  # 关闭画图的窗口
