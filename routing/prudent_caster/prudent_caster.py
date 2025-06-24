from collections import defaultdict
import copy
import random
import logging
import os
from entities.packet import DataPacket, AckPacket
from topology.virtual_force.vf_packet import VfPacket
from routing.prudent_caster.graph import Graph
from routing.prudent_caster.prudent_packet import PrudentHelloPacket, PrudentDronePacket, PrudentDataPacket
from utils import config
from utils.util_function import euclidean_distance_3d

GLOBAL_PRUDENT_DATA_PACKET_ID = 0

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class PrudentCaster:
    """
    Main procedure of PrudentCaster



    Attributes:
        simulator: the simulation platform that contains everything
        my_drone: the drone that installed the greedy routing
        rng_routing: a Random class based on which we can call the function that generates the random number
        hello_interval: interval of sending hello packet
        local_graph: local network graph

    Author: Yunna
    """

    def __init__(self, simulator, my_drone):
        self.simulator = simulator
        self.my_drone = my_drone
        self.rng_routing = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 10)
        self.hello_interval = 0.5 * 1e6  # broadcast hello packet every 0.5s
        self.data_interval = 1 * 1e6
        self.check_interval = 0.6 * 1e6
        self.local_graph = Graph(self.simulator.env)
        self.drone_path = defaultdict(list)
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.broadcast_data_packet_periodically())

    def broadcast_hello_packet(self):
        config.GL_ID_HELLO_PACKET += 1
        self.local_graph.update()
        neighbors = self.local_graph.find_neighbor(self.my_drone.identifier)
        hello_pkd = PrudentHelloPacket(src_drone=self.my_drone.identifier,
                                      neighbors=neighbors,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_HELLO_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator)
        hello_pkd.transmission_mode = 1

        logging.info('At time: %s, UAV: %s has hello packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)

        self.simulator.metrics.control_packet_num += 1
        #self.my_drone.transmitting_queue.put(hello_pkd)
        self.send_broadcast(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        while True:
            self.broadcast_hello_packet()
            jitter = self.rng_routing.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval + jitter)

    def send_broadcast(self, packet):
        # TODO, 判断通信范围，进行广播
        for drone in self.simulator.drones:
            if drone.identifier != self.my_drone.identifier :
                d = euclidean_distance_3d(self.my_drone.coords, drone.coords)
                if d < config.BROADCAST_RANGE:
                    self.my_drone.mac_protocol.phy.unicast(packet, drone.identifier)

    def broadcast_data_packet_periodically(self):
        while True:
            self.broadcast_data_packet()
            jitter = self.rng_routing.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.data_interval + jitter)
    def generate_broadcast_data_packet(self):
        global GLOBAL_PRUDENT_DATA_PACKET_ID
        self.local_graph.update()
        GLOBAL_PRUDENT_DATA_PACKET_ID += 1
        data_packet = PrudentDataPacket(src_drone=self.my_drone.identifier,
                                        drone_packets=list(),
                                        creation_time=self.simulator.env.now,
                                        data_packet_id=GLOBAL_PRUDENT_DATA_PACKET_ID,
                                        data_packet_length=config.DATA_PACKET_LENGTH,
                                        simulator=self.simulator)
        self.simulator.metrics.datapacket_generated_num += 1

        for drone, paths in self.drone_path.items():
            #print("drone: ", drone, "paths: ", paths)
            for path in paths:
                src_drone = path[-1]
                sub_graph = self.local_graph.get_subgraph_within_hops(src_drone, 2)
                new_path = path + [self.my_drone.identifier]
                mlst, _ = sub_graph.MLST(src_drone)
                #print("drone: ", drone, "new_path: ", new_path)
                if not mlst.is_leaf(self.my_drone.identifier) and mlst.path_exists(new_path):
                    drone_packet = PrudentDronePacket(prev_drone=src_drone,
                                 drone_id=drone[0],
                                 creation_time=drone[1],
                                 data_packet_id=drone[2],
                                 data_packet_length=config.DATA_PACKET_LENGTH,
                                 simulator=self.simulator)
                    data_packet.drone_packets.append(drone_packet)
                    break

        self.drone_path.clear()

        return data_packet

    def broadcast_data_packet(self):
        data_packet = self.generate_broadcast_data_packet()
        logging.info('At time: %s, UAV: %s has data packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)
        self.send_broadcast(data_packet)

    def packet_reception(self, packet):
        """
        Packet reception at network layer

        since different routing protocols have their own corresponding packets, it is necessary to add this packet
        reception function in the network layer
        :param packet: the received packet
        :param src_drone_id: previous hop
        :return: None
        """

        current_time = self.simulator.env.now
        if isinstance(packet, PrudentHelloPacket):
            neighbor = packet.src_drone
            self.local_graph.add_edge(self.my_drone.identifier, neighbor)  # update local_graph
            self.local_graph.add_node(neighbor, current_time)

            for prev_neigh in packet.neighbors:
                self.local_graph.add_edge(neighbor, prev_neigh)

        elif isinstance(packet, PrudentDataPacket):
            packet_copy = copy.copy(packet)

            logging.info('~~~Packet: %s is received by UAV: %s at: %s',
                         packet_copy.packet_id, self.my_drone.identifier, self.simulator.env.now)

            if packet_copy.src_drone != self.my_drone.identifier:
                neighbor = packet_copy.src_drone
                self.local_graph.add_edge(self.my_drone.identifier, neighbor)  # update local_graph
                self.local_graph.add_node(neighbor, current_time)
                key = (neighbor, packet_copy.creation_time, packet_copy.packet_id)
                self.drone_path[key].append([neighbor])
                self.write_msg(key)
                self.calc_metrics(packet_copy)


                drone_packets = packet_copy.drone_packets
                for p in drone_packets:

                    key = (p.drone_id, p.creation_time, p.packet_id)
                    self.drone_path[key].append([p.prev_drone, neighbor])
                    self.write_msg(key)
                    self.calc_metrics(p)

        elif isinstance(packet, VfPacket):
            logging.info('At time %s, UAV: %s receives the vf hello msg from UAV: %s, pkd id is: %s',
                         self.simulator.env.now, self.my_drone.identifier, src_drone_id, packet.packet_id)

            # update the neighbor table
            self.my_drone.motion_controller.neighbor_table.add_neighbor(packet, current_time)

            if packet.msg_type == 'hello':
                config.GL_ID_VF_PACKET += 1
                ack_packet = VfPacket(src_drone=self.my_drone,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_VF_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator)
                ack_packet.msg_type = 'ack'

                self.my_drone.transmitting_queue.put(ack_packet)
    def calc_metrics(self, packet_copy):
        latency = self.simulator.env.now - packet_copy.creation_time  # in us
        self.simulator.metrics.deliver_time_dict[packet_copy.packet_id] = latency
        self.simulator.metrics.throughput_dict[packet_copy.packet_id] = config.DATA_PACKET_LENGTH / (latency / 1e6)
        self.simulator.metrics.hop_cnt_dict[packet_copy.packet_id] = packet_copy.get_current_ttl()
        self.simulator.metrics.datapacket_arrived.add(packet_copy.packet_id)
        logging.info('Packet: %s is received by destination UAV: %s',
                     packet_copy.packet_id, self.my_drone.identifier)
    def write_msg(self, packet):
        """
        将消息写入日志文件
        :param packet: PrudentDronePacket对象
        """

        # 获取日志路径（环境变量或默认当前目录）
        log_path = config.LOG_PATH
        #print("write_msg: log_path: ", log_path)

        # 确保日志目录存在
        os.makedirs(log_path, exist_ok=True)

        # 构建日志文件名
        filename = os.path.join(log_path, str(self.my_drone.identifier))
        now = self.simulator.env.now

        drone_id = packet[0]
        creation_time = packet[1]
        packet_id = packet[2]

        try:
            # 计算延迟
            latency = now - creation_time

            # 准备写入的内容
            log_line = f"{drone_id}_{packet_id} {latency} \n"

            # 写入文件（追加模式）
            with open(filename, "a", encoding="utf-8") as file:
                file.write(log_line)

        except OSError as e:
            print(f"Error writing to log file: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
