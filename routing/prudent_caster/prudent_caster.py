from collections import defaultdict, deque
import copy
import random
import logging
import os
from topology.virtual_force.vf_packet import VfPacket
from routing.prudent_caster.graph import Graph
from routing.prudent_caster.safe_dict import SafeDict
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
        self.data_packet_sent = 0
        self.local_graph = Graph(self.simulator.env)
        self.drone_path = SafeDict()
        self.packets = SafeDict()
        self.simulator.env.process(self.broadcast_hello_packet_periodically())
        self.simulator.env.process(self.broadcast_data_packet_periodically())
        self.hello_count = 0

    def broadcast_hello_packet(self):
        config.GL_ID_HELLO_PACKET += 1
        self.hello_count += 1
        self.update_local_graph()
        adjs = defaultdict(list)
        neighbors = self.local_graph.find_neighbor(self.my_drone.identifier)
        for n in neighbors:
            adjs[n] = self.local_graph.find_neighbor(n)
        hello_pkd = PrudentHelloPacket(src_drone=self.my_drone.identifier,
                                      neighbors=adjs,
                                      creation_time=self.simulator.env.now,
                                      id_hello_packet=config.GL_ID_HELLO_PACKET,
                                      hello_packet_length=config.HELLO_PACKET_LENGTH,
                                      simulator=self.simulator)
        hello_pkd.transmission_mode = 1

        #logging.info('At time: %s, UAV: %s has hello packet to broadcast',
        #             self.simulator.env.now, self.my_drone.identifier)

        self.simulator.metrics.control_packet_num += 1
        #self.my_drone.transmitting_queue.put(hello_pkd)
        self.send_hello_broadcast(hello_pkd)

    def broadcast_hello_packet_periodically(self):
        yield self.simulator.env.timeout(self.my_drone.identifier*10)
        while True:
            self.broadcast_hello_packet()
            #jitter = self.rng_routing.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.hello_interval)

    def send_hello_broadcast(self, packet):
        # TODO, 判断通信范围，进行广播
        for drone in self.simulator.drones:
            if drone.identifier != self.my_drone.identifier:
                d = euclidean_distance_3d(self.my_drone.coords, drone.coords)
                #print("Hello my pos: ", self.my_drone.coords, "other pos:", drone.coords, "d: ", d)
                if d <= config.BROADCAST_RANGE:
                    self.my_drone.mac_protocol.phy.unicast(packet, drone.identifier)


    def send_data_broadcast(self, packet):
        # TODO, 判断通信范围，进行广播
        print("--------begin send_data_broadcast UAV", self.my_drone.identifier)
        uavs = []
        for drone in self.simulator.drones:
            if drone.identifier != self.my_drone.identifier :
                d = euclidean_distance_3d(self.my_drone.coords, drone.coords)
                print("Data my pos: ", self.my_drone.coords, "other pos:", drone.coords, "d: ", d)
                logging.info('UAV %s, send to UAV %s, d: %d',
                             self.my_drone.identifier, drone.identifier, d)
                if d <= config.BROADCAST_RANGE:
                    self.my_drone.mac_protocol.phy.unicast(packet, drone.identifier)
                    uavs.append(drone)

        packet_ids = [packet.packet_id]
        for p in packet.drone_packets:
            packet_ids.append(p.packet_id)
        logging.info('Broadcast Data, UAV %s, send to UAV %s, packets %s ',
                     self.my_drone.identifier, drone.identifier, packet_ids)
        print("--------send_data_broadcast UAV ", self.my_drone.identifier, "packet: ", packet_ids)
        print("--------end send_data_broadcast UAV ", self.my_drone.identifier)

    def broadcast_data_packet_periodically(self):
        while config.GL_ID_HELLO_PACKET - config.GL_ID_HELLO_PACKET_START < config.NUMBER_OF_DRONES + 1:
            yield self.simulator.env.timeout(self.hello_interval)  #
        while True:
            if self.simulator.env.now >= config.SIM_TIME:
                break
            self.broadcast_data_packet()
            #jitter = self.rng_routing.randint(1000, 2000)  # delay jitter
            yield self.simulator.env.timeout(self.data_interval)


    def generate_broadcast_data_packet(self):
        print("--------begin UAV", self.my_drone.identifier, "generate_broadcast_data_packet \n")
        global GLOBAL_PRUDENT_DATA_PACKET_ID
        self.update_local_graph()
       # print("--------UAV", self.my_drone.identifier, "local_graph: \n")
        #self.local_graph.display()
       # print("-------------------\n")

        GLOBAL_PRUDENT_DATA_PACKET_ID += 1
        data_packet = PrudentDataPacket(src_drone=self.my_drone.identifier,
                                        drone_packets=[],
                                        creation_time=self.simulator.env.now,
                                        data_packet_id=GLOBAL_PRUDENT_DATA_PACKET_ID,
                                        data_packet_length=config.DATA_PACKET_LENGTH,
                                        simulator=self.simulator)
        self.simulator.metrics.datapacket_generated_num += 1

        for key, paths in self.drone_path.items():
            print("packet_id: ", key, "paths: ", paths)
            for path in paths:
                src_drone = path[-1]
                sub_graph = self.local_graph.get_subgraph_within_hops(src_drone, 2)
                #print("--------sub_graph: \n")
                #sub_graph.display()
                #print("-------------------\n")
                prev_drone = path[0]
                mlst, _ = sub_graph.get_mlst(prev_drone)
                new_path = path + [self.my_drone.identifier]
                #print("--------mlst: \n")
                #mlst.display()
                #print("-------------------\n")
                #print("new_path: ", new_path)
                if not mlst.is_leaf(self.my_drone.identifier) and mlst.path_exists(new_path):
                    #print("Path exists: ", new_path)
                    packet = copy.copy(self.packets.get(key))
                    #print("add packet, key:", key, "packet_id: ", packet.packet_id)
                    packet.prev_drone = src_drone
                    packet.increase_ttl()
                    data_packet.drone_packets.append(packet)
                    break
            self.drone_path.remove(key)
            self.packets.remove(key)


        packet_ids = [data_packet.packet_id]
        for p in data_packet.drone_packets:
            packet_ids.append(p.packet_id)
        #print("data_packet:", packet_ids)
        #print("--------end UAV", self.my_drone.identifier, "generate_broadcast_data_packet \n")
        return data_packet

    def broadcast_data_packet(self):
        data_packet = self.generate_broadcast_data_packet()
        #self.data_packet_sent += (len(data_packet.drone_packets) + 1)

        logging.info('At time: %s, UAV: %s has data packet to broadcast',
                     self.simulator.env.now, self.my_drone.identifier)
        self.send_data_broadcast(data_packet)
        self.simulator.metrics.b_datapacket_sent += (len(data_packet.drone_packets) + 1)
        self.simulator.metrics.b_datapacket_arrived[self.my_drone.identifier].add(data_packet.packet_id)
        self.write_msg((data_packet.packet_id, data_packet.creation_time))

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

            for n, neigh_neighs in packet.neighbors.items():
                self.local_graph.add_edge(neighbor, n)
                for pn in neigh_neighs:
                    self.local_graph.add_edge(n, pn)

        elif isinstance(packet, PrudentDataPacket):
            #print("-------packet_reception UAV ", self.my_drone.identifier)
            packet_copy = packet
            if packet_copy.src_drone != self.my_drone.identifier:
                logging.info('~~~Packet: UAV %s received %s ',
                             self.my_drone.identifier, packet_copy.packet_id)
                neighbor = packet_copy.src_drone
                self.local_graph.add_edge(self.my_drone.identifier, neighbor)  # update local_graph
                self.local_graph.add_node(neighbor, current_time)
                key = packet_copy.packet_id

                new_drone_packet = PrudentDronePacket(drone_id=neighbor,
                                            prev_drone=neighbor,
                                            creation_time=packet_copy.creation_time,
                                            data_packet_id=packet_copy.packet_id,
                                            data_packet_length = packet_copy.packet_length,
                                            simulator = packet_copy.simulator)
                new_drone_packet.set_ttl(packet_copy.get_current_ttl())
                self.calc_metrics(new_drone_packet)
                self.update_packets(new_drone_packet, [neighbor])

                drone_packets = packet_copy.drone_packets
                for p in drone_packets:
                    self.calc_metrics(p)
                    key = p.packet_id
                    if p.packet_id != self.my_drone.identifier:
                        logging.info('~~~Packet: UAV %s received %s ',
                                     self.my_drone.identifier, p.packet_id)
                        new_packet = copy.copy(p)
                        path = [p.prev_drone, neighbor]
                        self.update_packets(new_packet, path)
                        #print("----received packet, p_id:", key, "packet_id: ", new_packet.packet_id)

        elif isinstance(packet, VfPacket):
            #logging.info('At time %s, UAV: %s receives the vf hello msg, pkd id is: %s',
             #            self.simulator.env.now, self.my_drone.identifier, packet.packet_id)

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

    def update_packets(self, packet, path):
        key = (packet.drone_id, packet.packet_id)
        queue = self.drone_path.get(key, deque())
        queue.append(path)
        self.drone_path.add(key, queue)
        self.write_msg((packet.packet_id, packet.creation_time))
        self.packets.add(key, packet)

    def calc_metrics(self, packet_copy):
            my = self.my_drone.identifier
            arrived_set = self.simulator.metrics.b_datapacket_arrived[my]
            if packet_copy.packet_id not in arrived_set:
                latency = self.simulator.env.now - packet_copy.creation_time  # in us
                self.simulator.metrics.deliver_time_dict[(my,packet_copy.packet_id)] = latency
                self.simulator.metrics.throughput_dict[(my,packet_copy.packet_id)] = config.DATA_PACKET_LENGTH / (latency / 1e6)
                self.simulator.metrics.hop_cnt_dict[(my,packet_copy.packet_id)] = packet_copy.get_current_ttl()
                self.simulator.metrics.b_datapacket_arrived[my].add(packet_copy.packet_id)

    def update_local_graph(self):
            for node, time in self.local_graph.nodeTime.items():
                if self.simulator.env.now - time > 5 * self.data_interval :
                    self.local_graph.remove_edge(self.my_drone.identifier, node)

    def write_msg(self, packet):
        """
        将消息写入日志文件
        :param packet: PrudentDronePacket对象
        """

        # 获取日志路径（环境变量或默认当前目录）
        log_path = config.LOG_PATH

        # 确保日志目录存在
        os.makedirs(log_path, exist_ok=True)

        # 构建日志文件名
        filename = os.path.join(log_path, str(self.my_drone.identifier))
        now = self.simulator.env.now

        packet_id = packet[0]
        creation_time = packet[1]
        try:
            # 计算延迟
            latency = now - creation_time

            # 准备写入的内容
            log_line = f"{packet_id} {latency} \n"

            # 写入文件（追加模式）
            with open(filename, "a", encoding="utf-8") as file:
                file.write(log_line)

        except OSError as e:
            print(f"Error writing to log file: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
