import random
import math

import numpy as np

from utils import config


def get_random_start_point_3d(sim_seed):
    start_position = []
    for i in range(config.NUMBER_OF_DRONES):
        random.seed(sim_seed + i)
        position_x = random.uniform(1, config.MAP_LENGTH - 1)
        position_y = random.uniform(1, config.MAP_WIDTH - 1)
        position_z = random.uniform(1, config.MAP_HEIGHT - 1)

        start_position.append(tuple([position_x, position_y, position_z]))

    return start_position


def generate_connected_drones(min_distance=10):
    """
    在立方体内生成全连通无人机位置

    参数:
    num_drones: 无人机数量
    cube_size: 立方体边长(米)
    comm_range: 通信范围(米)
    min_distance: 无人机间最小距离(米)

    返回:
    positions: 无人机位置数组 (num_drones x 3)
    graph: 网络图对象
    """
    # 初始化位置数组和图
    num_drones = config.NUMBER_OF_DRONES
    cube_size = config.MAP_WIDTH
    comm_range = config.BROADCAST_RANGE
    positions = np.zeros((num_drones, 3))

    # 1. 放置第一个节点在立方体中心
    center = cube_size / 2
    positions[0] = [center, center, center]

    # 2. 放置剩余无人机
    for i in range(1, num_drones):
        # 随机选择一个已放置的参考节点
        ref_idx = random.randint(0, i - 1)
        ref_pos = positions[ref_idx]

        # 尝试找到合适的位置
        placed = False
        attempts = 0
        max_attempts = 100  # 避免无限循环

        while not placed and attempts < max_attempts:
            attempts += 1

            # 在参考节点通信范围内随机生成位置
            # 随机方向向量
            theta = random.uniform(0, 2 * math.pi)  # 方位角
            phi = random.uniform(0, math.pi)  # 俯仰角

            # 随机距离 (在最小距离和通信范围之间)
            dist = random.uniform(min_distance, comm_range)

            # 计算新位置
            dx = dist * math.sin(phi) * math.cos(theta)
            dy = dist * math.sin(phi) * math.sin(theta)
            dz = dist * math.cos(phi)

            new_pos = ref_pos + np.array([dx, dy, dz])

            # 确保新位置在立方体内
            new_pos = np.clip(new_pos, 0, cube_size)

            # 检查是否与其他无人机太近
            too_close = False
            for j in range(i):
                if np.linalg.norm(new_pos - positions[j]) < min_distance:
                    too_close = True
                    break

            if too_close:
                continue

            # 位置有效，添加到数组
            positions[i] = new_pos
            placed = True

        if not placed:
            # 回退策略：在中心附近放置
            positions[i] = [
                center + random.uniform(-50, 50),
                center + random.uniform(-50, 50),
                center + random.uniform(-50, 50)
            ]

    return positions