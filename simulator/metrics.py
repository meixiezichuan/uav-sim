import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict


class Metrics:
    """
    Tools for statistics of network performance

    1. Packet Delivery Ratio (PDR): is the ratio of number of packets received at the destinations to the number
       of packets sent from the sources
    2. Average end-to-end (E2E) delay: is the time a packet takes to route from a source to its destination through
       the network. It is the time the data packet reaches the destination minus the time the data packet was generated
       in the source node
    3. Routing Load: is calculated as the ratio between the numbers of control Packets transmitted
       to the number of packets actually received. NRL can reflect the average number of control packets required to
       successfully transmit a data packet and reflect the efficiency of the routing protocol
    4. Throughput: it can be defined as a measure of how fast the data is sent from its source to its intended
       destination without loss. In our simulation, each time the destination receives a data packet, the throughput is
       calculated and finally averaged
    5. Hop count: used to record the number of router output ports through which the packet should pass.

    References:
        [1] Rani. N, Sharma. P, Sharma. P., "Performance Comparison of Various Routing Protocols in Different Mobility
            Models," in arXiv preprint arXiv:1209.5507, 2012.
        [2] Gulati M K, Kumar K. "Performance Comparison of Mobile Ad Hoc Network Routing Protocols," International
            Journal of Computer Networks & Communications. vol. 6, no. 2, pp. 127, 2014.

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
    Updated at: 2024/8/31
    """

    def __init__(self, simulator):
        self.simulator = simulator

        self.control_packet_num = 0

        self.datapacket_generated = set()  # all data packets generated
        self.datapacket_arrived = set()  # all data packets that arrives the destination
        self.datapacket_generated_num = 0


        self.delivery_time = []
        self.deliver_time_dict = defaultdict()

        self.throughput = []
        self.throughput_dict = defaultdict()

        self.hop_cnt = []
        self.hop_cnt_dict = defaultdict()

        self.mac_delay = []

        self.collision_num = 0

        # metricc for broadcast
        self.b_datapacket_generated = defaultdict(set)  # all data packets generated
        self.b_datapacket_arrived = defaultdict(set)
        self.b_datapacket_sent = 0

        self.drone_num = 0
        self.sum_arrived = 0

    def print_metrics(self):
        # calculate the average end-to-end delay
        for key in self.deliver_time_dict.keys():
            self.delivery_time.append(self.deliver_time_dict[key])

        for key2 in self.throughput_dict.keys():
            self.throughput.append(self.throughput_dict[key2])

        for key3 in self.hop_cnt_dict.keys():
            self.hop_cnt.append(self.hop_cnt_dict[key3])

        e2e_delay = np.mean(self.delivery_time) / 1e3  # unit: ms

        #pdr = len(self.datapacket_arrived) / self.datapacket_generated_num * 100  # in %
        for k, v in self.b_datapacket_arrived.items():
            self.sum_arrived += len(v)

        num_drones = len(self.b_datapacket_arrived.items())
        self.drone_num = num_drones

        pdr = self.sum_arrived / self.datapacket_generated_num / num_drones * 100  # in %

        #rl = self.control_packet_num / len(self.datapacket_arrived)

        throughput = np.mean(self.throughput) / 1e3  # in Kbps

        hop_cnt = np.mean(self.hop_cnt)

        #average_mac_delay = np.mean(self.mac_delay)

        print("Len of hop_cnt: ", len(self.hop_cnt))

        print('Totally generated: ', self.datapacket_generated_num, ' data packets')
        print('Totally sent: ', self.b_datapacket_sent, ' data packets')
        print('Totally receive: ', self.sum_arrived, ' data packets')
        print('Packet delivery ratio is: ', pdr, '%')
        print('Average end-to-end delay is: ', e2e_delay, 'ms')
        #print('Routing load is: ', rl)
        print('Average throughput is: ', throughput, 'Kbps')
        print('Average hop count is: ', hop_cnt)
        print('Collision num is: ', self.collision_num)
        #print('Average mac delay is: ', average_mac_delay, 'ms')

        with open("simulation_result.txt", "w") as f:
            f.write(f"Totally generated: {self.datapacket_generated_num} data packets\n")
            f.write(f"Totally sent: {self.b_datapacket_sent} data packets\n")
            f.write(f"Totally receive: {self.sum_arrived} data packets\n")
            f.write(f"Totally drone num: {self.drone_num}\n")
            f.write(f"Packet delivery ratio is: {pdr} %\n")
            f.write(f"Average end-to-end delay is: {e2e_delay} ms\n")
            #f.write(f"Routing load is: {rl}\n")
            f.write(f"Average throughput is: {throughput} Kbps\n")
            f.write(f"Average hop count is: {hop_cnt}\n")
            f.write(f"Collision num is: {self.collision_num}\n")
            #f.write(f"Average mac delay is: {average_mac_delay} ms\n")

