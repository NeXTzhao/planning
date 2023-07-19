import numpy as np
import matplotlib.pyplot as plt


def structured_sampling_planning(start, goal, road_map, grid_size, num_samples):
    # 将道路地图划分为网格
    num_rows = int(road_map.shape[0] / grid_size)
    num_cols = int(road_map.shape[1] / grid_size)

    # 生成每个网格的采样点
    samples = []
    for i in range(num_rows):
        for j in range(num_cols):
            if road_map[i * grid_size, j * grid_size] == 0:  # 检查网格是否为道路
                x = np.random.uniform(j * grid_size, (j + 1) * grid_size)
                y = np.random.uniform(i * grid_size, (i + 1) * grid_size)
                samples.append((x, y))

    # 将起点和终点加入采样点列表
    samples.append(start)
    samples.append(goal)

    # 随机选择采样点
    samples = np.random.choice(len(samples), num_samples, replace=False)
    valid_samples = [samples[i] for i in samples]

    # 可视化路径规划结果
    visualize_path(start, goal, road_map, grid_size, valid_samples)


def visualize_path(start, goal, road_map, grid_size, samples):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # 绘制道路地图
    ax.imshow(road_map, cmap='gray')

    grid_num_rows = road_map.shape[0] // grid_size
    grid_num_cols = road_map.shape[1] // grid_size

    # 绘制采样点
    for sample in samples:
        x = sample[1] * grid_size + grid_size / 2
        y = sample[0] * grid_size + grid_size / 2
        ax.plot(x, y, 'bo')

    # 绘制起点和终点
    ax.plot(start[1], start[0], 'go')
    ax.plot(goal[1], goal[0], 'go')

    # 设置坐标轴范围
    ax.set_xlim([0, road_map.shape[1]])
    ax.set_ylim([0, road_map.shape[0]])

    plt.grid(True)
    plt.show()


# 示例使用
start = (10, 10)
goal = (190, 190)
grid_size = 20
road_map = np.zeros((200, 200))
road_map[50:150, 50:150] = 1  # 模拟一个道路区域，100x100的矩形
num_samples = 100
structured_sampling_planning(start, goal, road_map, grid_size, num_samples)
