import logging
import math
from utils.util_function import euclidean_distance_3d
from collections import defaultdict


class GreedyNeighborTable:
    """
    Neighbor table of Greedy Forwarding

    type of the neighbor table: dictionary
    the structure of the neighbor table is: {drone1: [coords1, updated time1], drone2: [coords2, updated time2],...}
    each item in the neighbor table has its lifetime, if the hello packet from a drone has not been received for more
    than a certain time, it can be considered that this drone has flown out of my communication range. Therefore, the
    item associated with this drone is removed from my neighbor table

    Attributes:
        env: simulation environment
        neighbor_table: dictionary in python, core member
        entry_life_time: lifetime of each item in the neighbor table
        have_void_area: used to indicate if encounters void area

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/11/20
    """

    def __init__(self, env, my_drone):
        self.env = env
        self.my_drone = my_drone
        self.neighbor_table = defaultdict(list)
        self.entry_life_time = 1 * 1e6  # unit: us (1s)
        self.have_void_area = 1

    # determine if the neighbor table is empty
    def is_empty(self):
        return not bool(self.neighbor_table)

    # get the updated time of certain item
    def get_updated_time(self, drone_id):
        if drone_id not in self.neighbor_table.keys():
            raise RuntimeError('This item is not in the neighbor table')
        else:
            return self.neighbor_table[drone_id][1]

    def add_neighbor(self, hello_packet, cur_time):
        """
        Update the neighbor table according to the hello packet
        :param hello_packet: the received hello packet
        :param cur_time: the moment when the packet is received
        :return: None
        """

        drone_id = hello_packet.src_drone.identifier
        position = hello_packet.cur_position
        self.neighbor_table[drone_id] = [position, cur_time]

    # delete the specified item
    def remove_neighbor(self, drone_id):
        del self.neighbor_table[drone_id]

    # determine whether a certain drone is one's neighbor
    def is_neighbor(self, certain_drone):
        drone_id = certain_drone.identifier
        return drone_id in self.neighbor_table.keys()

    # get the position of a neighbor node
    def get_neighbor_position(self, certain_drone):
        if self.is_neighbor(certain_drone):
            drone_id = certain_drone.identifier
            return self.neighbor_table[drone_id][0]  # return the position list
        else:
            raise RuntimeError('This drone is not my neighbor!')

    # remove the expired item
    def purge(self):
        if not bool(self.neighbor_table):
            # it means that the neighbor table is empty
            return

        for key in list(self.neighbor_table):
            updated_time = self.get_updated_time(key)
            if updated_time + self.entry_life_time < self.env.now:  # expired
                self.remove_neighbor(key)

    # print neighbor table
    def print_neighbor(self, my_drone):
        logging.info('|----------Neighbor Table of: %s ----------|', my_drone.identifier)
        for key in self.neighbor_table:
            logging.info('Neighbor: %s, position is: %s, updated time is: %s, ',
                         key, self.neighbor_table[key][0], self.neighbor_table[key][1])
        logging.info('|-----------------------------------------------------------------|')

    # clear neighbor table
    def clear(self):
        self.neighbor_table.clear()

    def best_neighbor(self, my_drone, dst_drone):
        """
        Choose the best next hop according to the neighbor table
        :param my_drone: the drone that installed the GPSR
        :param dst_drone: the destination of the data packet
        :return: none
        """

        best_distance = euclidean_distance_3d(my_drone.coords, dst_drone.coords)
        best_id = my_drone.identifier

        for key in self.neighbor_table.keys():
            next_hop_position = self.neighbor_table[key][0]
            temp_distance = euclidean_distance_3d(next_hop_position, dst_drone.coords)
            if temp_distance < best_distance:
                best_distance = temp_distance
                best_id = key
                self.have_void_area = 0

        return best_id
