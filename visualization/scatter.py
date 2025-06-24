import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import config
from utils.util_function import euclidean_distance_3d
from phy.large_scale_fading import maximum_communication_range


# draw a static scatter plot, includes communication edges
def scatter_plot(simulator, init):
    fig = plt.figure()
    ax = fig.add_axes(Axes3D(fig))

    for drone1 in simulator.drones:
        for drone2 in simulator.drones:
            if drone1.identifier != drone2.identifier:
                ax.scatter(drone1.coords[0], drone1.coords[1], drone1.coords[2], c='red', s=30)
                distance = euclidean_distance_3d(drone1.coords, drone2.coords)
                if distance <= maximum_communication_range():
                    x = [drone1.coords[0], drone2.coords[0]]
                    y = [drone1.coords[1], drone2.coords[1]]
                    z = [drone1.coords[2], drone2.coords[2]]
                    ax.plot(x, y, z, color='black', linestyle='dashed', linewidth=1)

    ax.set_xlim(0, config.MAP_LENGTH)
    ax.set_ylim(0, config.MAP_WIDTH)
    ax.set_zlim(0, config.MAP_HEIGHT)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if init:
        plt.savefig("init.png")
    else:
        plt.savefig("map.png")

    #plt.show()
