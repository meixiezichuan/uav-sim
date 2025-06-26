
import math
import numpy as np
from phy.channel import Channel
from entities.drone import Drone
from simulator.metrics import Metrics
from mobility import start_coords
from utils import config
from visualization.scatter import scatter_plot


class Simulator:
    """
    Description: simulation environment

    Attributes:
        env: simpy environment
        total_simulation_time: discrete time steps, in nanosecond
        n_drones: number of the drones
        channel_states: a dictionary, used to describe the channel usage
        channel: wireless channel
        metrics: Metrics class, used to record the network performance
        drones: a list, contains all drone instances

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/8/16
    """

    def __init__(self,
                 seed,
                 env,
                 channel_states,
                 n_drones,
                 total_simulation_time=config.SIM_TIME):

        self.env = env
        self.seed = seed
        self.total_simulation_time = total_simulation_time  # total simulation time (ns)

        self.n_drones = n_drones  # total number of drones in the simulation
        self.channel_states = channel_states
        self.channel = Channel(self.env)

        self.map_width = config.MAP_WIDTH
        self.map_height = config.MAP_HEIGHT
        self.map_length = config.MAP_LENGTH

        self.drone_range = list()

        self.metrics = Metrics(self)  # use to record the network performance

        #start_position = start_coords.get_random_start_point_3d(seed)
        start_position = start_coords.generate_connected_drones()

        self.drones = []
        #for i in range(n_drones):
        #    if config.HETEROGENEOUS:
        #        speed = random.randint(5, 60)
        #    else:
        #       speed = config.BROADCAST_RANGE / 20

        #    print('UAV: ', i, ' initial location is at: ', start_position[i], ' speed is: ', speed)
        #    drone = Drone(env=env, node_id=i, coords=start_position[i], speed=speed,
        #                  inbox=self.channel.create_inbox_for_receiver(i), simulator=self)
        #    self.drones.append(drone)

        self.create_drones()
        self.current_turn = 0  # 表示“当前轮到哪台无人机”
        self.token_event = [env.event() for _ in range(config.NUMBER_OF_DRONES)]

        scatter_plot(self, True)

        #self.env.process(self.show_performance())
        self.env.process(self.show_time())

    def show_time(self):
        while True:
            print('At time: ', self.env.now / 1e6, ' s.')
            yield self.env.timeout(0.5*1e6)  # the simulation process is displayed every 0.5s

    def show_performance(self):
        #yield self.env.timeout(self.total_simulation_time)

        scatter_plot(self, False)

        self.metrics.print_metrics()

    def create_backbone_drones(self, hop):
        posX = list()
        for i in range(1, hop+1):
            posX.append(i * config.BROADCAST_RANGE)

        maxX = posX[-1] + config.BROADCAST_RANGE

        self.map_height = config.BROADCAST_RANGE * 2
        self.map_width = config.BROADCAST_RANGE * 2
        self.map_length = maxX

        centerY = config.BROADCAST_RANGE
        centerZ = config.BROADCAST_RANGE
        delta = (math.sqrt(3) / 2) * config.BROADCAST_RANGE
        cd = math.ceil(delta)
        fd = cd - 1
        x_range = (cd, maxX - cd - 1)
        y_range = (centerY - fd, centerY + fd)
        z_range = (centerZ - fd, centerZ + fd)

        self.drone_range.append(x_range)
        self.drone_range.append(y_range)
        self.drone_range.append(z_range)

        for i in range(hop):
            speed = 0
            pos = (posX[i], config.BROADCAST_RANGE, config.BROADCAST_RANGE)
            print('UAV: ', i, ' initial location is at: ', pos, ' speed is: ', speed)
            drone = Drone(env=self.env, node_id=i, coords=pos, speed=speed,
                          inbox=self.channel.create_inbox_for_receiver(i), simulator=self)
            self.drones.append(drone)

    def create_normal_drones(self, hop):
            drone_count = config.NUMBER_OF_DRONES
            speed = config.BROADCAST_RANGE / 20
            for i in range(hop, drone_count):
                pos = start_coords.get_random_start_point_3d_for_normal(self.seed+i, self.drone_range)
                print('UAV: ', i, ' initial location is at: ', pos, ' speed is: ', speed)
                drone = Drone(env=self.env, node_id=i, coords=pos, speed=speed,
                              inbox=self.channel.create_inbox_for_receiver(i), simulator=self)
                self.drones.append(drone)

    def create_drones(self):
        hop = config.MAX_HOP
        self.create_backbone_drones(hop)
        self.create_normal_drones(hop)
