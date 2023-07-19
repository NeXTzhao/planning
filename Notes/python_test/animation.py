import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

# 设置plt为不阻塞模式
plt.ion()

fig, ax = plt.subplots()

plt.plot([1, 2], [10, 20], 'b-')

# 画线
xdata = []
ydata = []
line, = plt.plot(xdata, ydata, 'r-')

xdata1 = []
ydata1 = []
line1, = plt.plot(xdata1, ydata1, 'g-')

# 显示文字
# 0.05和0.9 表示线框显示的位置
x_template = 'x = %.2f'
x_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


def init():
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 30)
    return line, line1


def update(frame, n):
    print(frame)

    xdata.append(frame)
    ydata.append(frame ** n)
    line.set_data(xdata, ydata)

    xdata1.append(frame)
    ydata1.append(frame ** (n + 1))
    line1.set_data(xdata1, ydata1)

    # 显示文字
    x_text.set_text(x_template % frame)
    return line, line1, x_text


ani = animation.FuncAnimation(
    fig=fig,
    func=update,
    frames=np.linspace(0., 5.0, 100),
    init_func=init,
    interval=100,
    repeat=False,
    blit=True,
    fargs=(3,)  # 将参数传给func
)

# 阻塞模式
plt.show(block=False)

# 不阻塞
# plt.pause(2)
print('不阻塞')