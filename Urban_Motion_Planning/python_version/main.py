import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches


class IDMModel:
    def __init__(self, v0, T, a, b, s0, d, dt=0.1):
        self.v0 = v0  # 自由流速度
        self.T = T  # 舒适时间头way
        self.a = a  # 最大加速度
        self.b = b  # 加速度耦合系数
        self.s0 = s0  # 最小安全跟随距离
        self.d = d  # 车辆长度
        self.dt = dt  # 时间步长

    def calculate_next_velocity(self, v, s, v_front):
        # 计算期望速度
        v_desired = self.v0 * (math.tanh((s - self.s0) / self.T) + math.tanh((s - self.s0 - self.d) / self.T))

        # 计算期望加速度
        a_desired = self.a * (1.0 - (v / self.v0) ** 4 - (self.s_desired(v, v_front) / s) ** 2)

        # 计算下一时刻的速度
        v_next = v + a_desired * self.dt

        return v_next

    def s_desired(self, v, v_front):
        return self.s0 + max(0.0, v * self.T + 0.5 * v * (v - v_front) / math.sqrt(self.a * self.b))


class MOBILModel:
    def __init__(self, idm_model, threshold):
        self.idm_model = idm_model
        self.threshold = threshold

    def should_lane_change(self, v_self, s_self, v_front, s_front, v_rear):
        v_self_desired = self.idm_model.calculate_next_velocity(v_self, s_self, v_front)  # 修正此处的方法名
        v_self_current = min(v_self, v_self_desired)
        v_rear_current = v_rear if v_rear is not None else 0.0

        if v_self_current < v_front:
            return False  # 不进行车道变更，因为速度较慢的车辆在前方

        if v_self_current - v_front >= self.threshold and v_self_current - v_rear_current >= self.threshold:
            return True  # 进行车道变更，因为有足够的加速度来安全变道

        return False


def simulate_traffic(num_steps, initial_velocity, initial_distance, lead_vehicle_velocity):
    idm = IDMModel(25.0, 1.5, 1.0, 2.0, 2.0, 4.0)  # 添加车辆长度参数
    mobil = MOBILModel(idm, 1.0)  # 设置 MOBIL 模型的阈值
    velocities = [initial_velocity]
    distances = [initial_distance]

    lane_change = False  # 是否执行车道变换
    lane_change_start_step = None  # 车道变换开始的步数
    lane_change_target_lane = None  # 目标车道（0表示左车道，1表示右车道）

    for step in range(num_steps):
        v = velocities[-1]
        s = distances[-1]
        v_front = lead_vehicle_velocity
        v_rear = velocities[step - 1] if step > 0 else None  # 前一步的速度

        # 如果当前没有进行车道变换，而且 MOBIL 模型建议变换
        if not lane_change and mobil.should_lane_change(v, s, v_front, 0, v_rear):
            lane_change = True
            lane_change_start_step = step
            lane_change_target_lane = 1  # 这里示例选择向右变道，实际应用中可根据需要修改

        # 如果当前正在进行车道变换
        if lane_change:
            # 在车道变换开始后的一定步数后，切换到目标车道
            if step - lane_change_start_step >= 10:  # 假设变道需要10步完成
                lane_change_target_lane = None  # 变道完成，重置目标车道
                lane_change = False

        # 计算下一步速度和位置
        v_next = idm.calculate_next_velocity(v, s, v_front)
        s_next = s + v_next * idm.dt

        velocities.append(v_next)
        distances.append(s_next)

    return velocities, distances


if __name__ == "__main__":
    num_steps = 100
    initial_velocity = 10.0
    initial_distance = 5.0
    lead_vehicle_velocity = 15.0

    velocities, distances = simulate_traffic(num_steps, initial_velocity, initial_distance, lead_vehicle_velocity)

    # 初始化自车位置和尺寸
    car_length = 4.0  # 车辆长度
    car_width = 1.8  # 车辆宽度
    car_x = [0]  # 自车x坐标列表
    car_y = [0]  # 自车y坐标列表

    # 创建车道边界线坐标
    lane_width = 3.5  # 车道宽度
    left_lane_boundary_x = np.array([0, max(distances) + 10])
    left_lane_boundary_y = np.array([-lane_width / 2, -lane_width / 2])
    right_lane_boundary_x = np.array([0, max(distances) + 10])
    right_lane_boundary_y = np.array([lane_width / 2, lane_width / 2])
    # 创建车道中心线坐标
    lane_center_x = np.array([0, max(distances) + 10])
    lane_center_y = np.array([0, 0])
    # 创建可视化窗口
    fig, ax = plt.subplots()

    for t, v in enumerate(velocities):
        s = distances[t]

        # 更新自车位置
        car_x.append(s)
        car_y.append(0)

        # 清空图形
        ax.clear()

        # 绘制车道边界线
        ax.plot(left_lane_boundary_x, left_lane_boundary_y, 'k-', linewidth=2, label='Left Lane Boundary')
        ax.plot(right_lane_boundary_x, right_lane_boundary_y, 'k-', linewidth=2, label='Right Lane Boundary')

        # 绘制车道中心线
        ax.plot(lane_center_x, lane_center_y, 'g--', linewidth=2, label='Lane Centerline')

        # 绘制自车矩形
        car_rect = patches.Rectangle((car_x[t], car_y[t] - car_width / 2), car_length, car_width, linewidth=1,
                                     edgecolor='r', facecolor='r')
        ax.add_patch(car_rect)

        # 绘制前车
        lead_car_x = s + 10  # 前车距离自车10米
        lead_car_rect = patches.Rectangle((lead_car_x, car_y[t] - car_width / 2), car_length, car_width, linewidth=1,
                                          edgecolor='b', facecolor='b')
        ax.add_patch(lead_car_rect)

        # 设置坐标轴范围
        ax.set_xlim(-5, max(s + 10, 50))
        ax.set_ylim(-lane_width / 2 - 1, lane_width / 2 + 1)

        # 绘制图例
        ax.legend(loc='upper right')

        plt.xlabel("Distance (m)")
        plt.ylabel("Width (m)")
        plt.axis("equal")
        plt.title("Self-Vehicle Position and Speed Over Time")
        plt.grid()

        # 显示图形
        plt.pause(0.1)

    plt.show()
