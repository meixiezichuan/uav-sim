import random
import numpy as np
from utils import config
import matplotlib.pyplot as plt
from utils.util_function import euclidean_distance_3d
from mpl_toolkits.mplot3d import Axes3D


class RandomWaypoint3D:
    """
    3-D Random Waypoint Mobility Model

    In this mobility model, the waypoint of drone will be generated in advance. Then drone will visit these waypoints
    in order. When the drone reached the waypoint, it will pause for a while, and then start heading down to the next
    waypoint. Normally, we will set up multiple waypoints as many as possible to prevent the drone visiting all the
    waypoints before the simulation is finished.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/19
    Updated at: 2025/1/7
    """

    def __init__(self, drone):
        self.model_identifier = 'RandomWaypoint'
        self.my_drone = drone
        self.rng_mobility = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 1)

        self.position_update_interval = 1 * 1e5  # 0.1s
        self.pause_time = 5 * 1e6  # in second

        self.min_x = 0
        self.max_x = config.MAP_LENGTH

        self.min_y = 0
        self.max_y = config.MAP_WIDTH

        self.min_z = 0
        self.max_z = config.MAP_HEIGHT

        # generate random waypoint
        self.waypoint_num = 5
        self.waypoint_spacing_x = 50
        self.waypoint_spacing_y = 50
        self.waypoint_spacing_z = 50
        self.waypoint_coords = []
        self.waypoint_generator(self.my_drone.coords)

        # used to determine if the waypoint has been visited
        self.waypoint_visited = [0 for _ in range(self.waypoint_num)]

        self.my_drone.simulator.env.process(self.mobility_update(self.my_drone))
        self.trajectory = []
        self.my_drone.simulator.env.process(self.show_trajectory())

    def waypoint_generator(self, start_coords):
        for i in range(self.waypoint_num):
            if i == 0:
                last_waypoint = start_coords
            else:
                last_waypoint = self.waypoint_coords[i-1]

            ranges_x = []
            if last_waypoint[0] - self.waypoint_spacing_x > self.min_x:
                ranges_x.append([0, last_waypoint[0] - self.waypoint_spacing_x])
            if last_waypoint[0] + self.waypoint_spacing_x < self.max_x:
                ranges_x.append([last_waypoint[0] + self.waypoint_spacing_x, self.max_x])

            which_range_x = self.rng_mobility.choice(ranges_x)
            waypoint_x = self.rng_mobility.uniform(which_range_x[0], which_range_x[1])

            ranges_y = []
            if last_waypoint[1] - self.waypoint_spacing_y > self.min_y:
                ranges_y.append([0, last_waypoint[1] - self.waypoint_spacing_y])
            if last_waypoint[1] + self.waypoint_spacing_y < self.max_y:
                ranges_y.append([last_waypoint[1] + self.waypoint_spacing_y, self.max_y])

            which_range_y = self.rng_mobility.choice(ranges_y)
            waypoint_y = self.rng_mobility.uniform(which_range_y[0], which_range_y[1])

            ranges_z = []
            if last_waypoint[2] - self.waypoint_spacing_z > self.min_z:
                ranges_z.append([0, last_waypoint[2] - self.waypoint_spacing_z])
            if last_waypoint[2] + self.waypoint_spacing_z < self.max_z:
                ranges_z.append([last_waypoint[2] + self.waypoint_spacing_z, self.max_z])

            which_range_z = self.rng_mobility.choice(ranges_z)
            waypoint_z = self.rng_mobility.uniform(which_range_z[0], which_range_z[1])

            next_waypoint = [waypoint_x, waypoint_y, waypoint_z]
            self.waypoint_coords.append(next_waypoint)

    def get_first_unvisited_waypoint(self):
        if 0 in self.waypoint_visited:
            waypoint_idx = self.waypoint_visited.index(0)
            waypoint_coords = self.waypoint_coords[waypoint_idx]
            return waypoint_coords, waypoint_idx
        else:
            return self.waypoint_coords[-1], -1

    def mobility_update(self, drone):
        while True:
            env = drone.simulator.env
            drone_id = drone.identifier
            drone_speed = drone.speed
            cur_position = drone.coords
            target_waypoint, target_waypoint_idx = self.get_first_unvisited_waypoint()
            drone.velocity = calculate_velocity(cur_position, target_waypoint, drone_speed)

            # update the position of next time step
            if config.STATIC_CASE == 0:
                next_position_x = cur_position[0] + drone.velocity[0] * self.position_update_interval / 1e6
                next_position_y = cur_position[1] + drone.velocity[1] * self.position_update_interval / 1e6
                next_position_z = cur_position[2] + drone.velocity[2] * self.position_update_interval / 1e6
            else:
                next_position_x = cur_position[0]
                next_position_y = cur_position[1]
                next_position_z = cur_position[2]

            next_position = [next_position_x, next_position_y, next_position_z]

            if drone_id == 1:
                self.trajectory.append(next_position)

            # judge if the drone has reach the target waypoint
            if euclidean_distance_3d(next_position, target_waypoint) < 20:
                self.waypoint_visited[target_waypoint_idx] = 1
                yield env.timeout(self.pause_time)

            drone.coords = next_position
            yield env.timeout(self.position_update_interval)
            energy_consumption = (self.position_update_interval / 1e6) * drone.energy_model.power_consumption(drone.speed)
            drone.residual_energy -= energy_consumption

    def show_trajectory(self):
        print(self.waypoint_coords)
        x = []
        y = []
        z = []
        yield self.my_drone.simulator.env.timeout(config.SIM_TIME-1)
        if self.my_drone.identifier == 1:
            for i in range(len(self.trajectory)):
                x.append(self.trajectory[i][0])
                y.append(self.trajectory[i][1])
                z.append(self.trajectory[i][2])

            plt.figure()
            ax = plt.axes(projection='3d')
            ax.set_xlim(self.min_x, self.max_x)
            ax.set_ylim(self.min_y, self.max_y)
            ax.set_zlim(self.min_z, self.max_z)

            waypoint_x = [point[0] for point in self.waypoint_coords]
            waypoint_y = [point[1] for point in self.waypoint_coords]
            waypoint_z = [point[2] for point in self.waypoint_coords]
            ax.scatter(waypoint_x, waypoint_y, waypoint_z, c='r')

            x = np.array(x)
            y = np.array(y)
            z = np.array(z)

            ax.plot(x, y, z)
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            #plt.show()
            plt.savefig('random_waypoint.png')


def calculate_velocity(current_pos, target_pos, moving_speed):
    distance = euclidean_distance_3d(current_pos, target_pos)
    normalized_vector = [(target_pos[0] - current_pos[0]) / distance,
                         (target_pos[1] - current_pos[1]) / distance,
                         (target_pos[2] - current_pos[2]) / distance]
    moving_speed = [moving_speed] * 3
    velocity = [d * v for d, v in zip(normalized_vector, moving_speed)]
    return velocity
