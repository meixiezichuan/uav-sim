import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import config
from utils.util_function import euclidean_distance_3d
import numpy as np
from phy.large_scale_fading import maximum_communication_range


def scatter_plot(simulator, init):
    fig = plt.figure(figsize=(10, 8))
    #ax = fig.add_subplot(111, projection='3d')  # 更标准的3D坐标创建方式
    ax = fig.add_axes(Axes3D(fig))

    for drone in simulator.drones:
        print("drone ", drone.identifier, "coods: ", drone.coords)
    # 1. 收集所有无人机坐标
    all_coords = np.round(np.array([drone.coords for drone in simulator.drones]), 2)

    # 2. 一次性绘制所有节点（避免重复）
    ax.scatter(
        all_coords[:, 0],
        all_coords[:, 1],
        all_coords[:, 2],
        c='red', s=50, alpha=0.8, label='Drones'
    )

    #for xi, yi, zi in all_coords:
    #    label = f'({xi}, {yi}, {zi})'
    #    ax.text(xi, yi, zi, label, fontsize=10, color='black')

    # 3. 绘制连接线（避免重复和无效连接）
    connected_pairs = set()  # 跟踪已绘制的连接对

    for i, drone1 in enumerate(simulator.drones):
        for j, drone2 in enumerate(simulator.drones):
            if i >= j:  # 避免重复检查同一对无人机
                continue
            distance = euclidean_distance_3d(drone1.coords, drone2.coords)
            print("drone ", i, j, "distance: ", distance)
            if distance <= config.BROADCAST_RANGE:
                pair_id = tuple(sorted((drone1.identifier, drone2.identifier)))
                if pair_id not in connected_pairs:
                    connected_pairs.add(pair_id)

                    # 绘制连接线
                    x = [drone1.coords[0], drone2.coords[0]]
                    y = [drone1.coords[1], drone2.coords[1]]
                    z = [drone1.coords[2], drone2.coords[2]]

                    ax.plot(
                        x, y, z,
                        color='blue',  # 更明显的颜色
                        linestyle='-',  # 实线更清晰
                        linewidth=1.5,
                        alpha=0.5
                    )

    # 4. 添加坐标轴标签和范围
    ax.set_xlim(0, simulator.map_length)
    ax.set_ylim(0, simulator.map_width)
    ax.set_zlim(0, simulator.map_height)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)

    ax.set_title(f'Drone Network at Time: {simulator.env.now:.1f}s', fontsize=14)

    # 5. 添加图例和网格
    ax.legend(loc='best')
    ax.grid(True, linestyle='--', alpha=0.3)

    # 6. 设置视角
    #ax.view_init(elev=30, azim=45)  # 更好的默认视角

    # 7. 保存和显示
    filename = "init.png" if init else "map.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')

    if init:
        plt.show()

    plt.close(fig)  # 关闭图形释放内存

# draw a static scatter plot, includes communication edges
def scatter_plot_1(simulator, init):
    fig = plt.figure()
    ax = fig.add_axes(Axes3D(fig))

    for drone1 in simulator.drones:
        for drone2 in simulator.drones:
            if drone1.identifier != drone2.identifier:
                ax.scatter(drone1.coords[0], drone1.coords[1], drone1.coords[2], c='red', s=30)
                distance = euclidean_distance_3d(drone1.coords, drone2.coords)
                #if distance <= maximum_communication_range():
                if distance <= config.BROADCAST_RANGE:
                    x = [drone1.coords[0], drone2.coords[0]]
                    y = [drone1.coords[1], drone2.coords[1]]
                    z = [drone1.coords[2], drone2.coords[2]]
                    ax.plot(x, y, z, color='black', linestyle='dashed', linewidth=1)

    ax.set_xlim(0, simulator.map_length)
    ax.set_ylim(0, simulator.map_width)
    ax.set_zlim(0, simulator.map_height)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if init:
        plt.savefig("init.png")
        plt.show()
    else:
        plt.savefig("map.png")

    #plt.show()
