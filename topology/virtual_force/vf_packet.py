from entities.packet import Packet


class VfPacket(Packet):
    def __init__(self,
                 src_drone,
                 creation_time,
                 id_hello_packet,
                 hello_packet_length,
                 simulator):
        super().__init__(id_hello_packet, hello_packet_length, creation_time, simulator)

        self.msg_type = 'hello'
        self.src_drone = src_drone
        self.cur_position = src_drone.coords
