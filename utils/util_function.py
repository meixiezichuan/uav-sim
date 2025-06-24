import logging

from utils import config


def euclidean_distance_3d(p1, p2):
    """
    Calculate the 3-D Euclidean distance between two nodes
    :param p1: the first point
    :param p2: the second point
    :return: Euclidean distance between p1 and p2
    """

    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5
    return dist


def euclidean_distance_2d(p1, p2):
    """
    Calculate the 2-D Euclidean distance between two nodes
    :param p1: the first point
    :param p2: the second point
    :return: 2-D Euclidean distance between p1 and p2
    """

    dist = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    return dist

def has_intersection(interval1, interval2):
    if interval2[0] <= interval1[0] <= interval2[1]:
        return True
    if interval2[0] <= interval1[1] <= interval2[1]:
        return True
    if interval1[0] <= interval2[0] <= interval1[1]:
        return True
    if interval1[0] <= interval2[1] <= interval1[1]:
        return True

    return False


def check_channel_availability(channel_states, sender_drone, drones):
    """
    Check if the channel is busy or idle
    :param channel_states: a dictionary, indicates the use of the channel by different drones
    :param sender_drone: the drone that is about to send packet
    :param drones: a list, which contains all the drones in the simulation
    :return: if the channel is busy, return "False", else, return "True"
    """

    for node_id in channel_states.keys():
        if len(channel_states[node_id].users) != 0:
            if node_id != sender_drone.identifier:
                d = euclidean_distance_3d(sender_drone.coords, drones[node_id].coords)
                if d < config.SENSING_RANGE:
                    return False

    return True
