from entities.packet import Packet, DataPacket

class PrudentHelloPacket(Packet):
    def __init__(self,
                 src_drone,
                 neighbors,
                 creation_time,
                 id_hello_packet,
                 hello_packet_length,
                 simulator):
        super().__init__(id_hello_packet, hello_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.neighbors = neighbors

class PrudentDronePacket(Packet):
    def __init__(self,
                 drone_id,
                 prev_drone,
                 creation_time,
                 data_packet_id,
                 data_packet_length,
                 simulator):
        super().__init__(data_packet_id, data_packet_length, creation_time, simulator)

        self.drone_id = drone_id
        self.prev_drone = prev_drone
class PrudentDataPacket(Packet):
    def __init__(self,
                 src_drone,
                 drone_packets,
                 creation_time,
                 data_packet_id,
                 data_packet_length,
                 simulator):
        super().__init__(data_packet_id, data_packet_length, creation_time, simulator)

        self.src_drone = src_drone
        self.drone_packets = drone_packets





