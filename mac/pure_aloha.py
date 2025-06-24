import simpy
import logging
import random
from phy.phy import Phy
from utils import config

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class PureAloha:
    """
    Pure ALOHA protocol

    This protocol allows devices to transmit packet at any time, without a set schedule. After transmitting a packet,
    the drone should wait for the ACK packet. If it fails to receive the corresponding ACK packet after a period of time,
    the drone will simply wait a random amount of time before attempting to transmit again.

    The basic flow of the Pure ALOHA is as follows:
        1) when a node has a packet to send, it just sends it, without listening to the channel and random backoff
        2) after sending the packet, the node starts to wait for the ACK
        3) if it receives ACK, the mac_send process will finish
        4) if not, the node will wait a random amount of time, according to the number of re-transmissions attempts

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/4/22
    Updated at: 2025/1/22
    """

    def __init__(self, drone):
        self.my_drone = drone
        self.simulator = drone.simulator
        self.rng_mac = random.Random(self.my_drone.identifier + self.my_drone.simulator.seed + 5)
        self.env = drone.env
        self.phy = Phy(self)
        self.channel_states = self.simulator.channel_states
        self.enable_ack = True

        self.wait_ack_process_dict = dict()
        self.wait_ack_process_finish = dict()
        self.wait_ack_process_count = 0
        self.wait_ack_process = None

    def mac_send(self, pkd):
        yield self.env.timeout(0.01)

        if pkd.number_retransmission_attempt[self.my_drone.identifier] == 1:
            """
            NOTE: because the service time of the packet is given by the interval between the time when this packet
            starts transmission attempt and the time when it is acknowledged and removed from the queue, so the 
            "first_attempt_time" should be recorded only when the drone transmits this packet for the first time.
            """
            pkd.first_attempt_time = self.env.now

        key = 'mac_send' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
        self.my_drone.mac_process_finish[key] = 1  # mark the process as "finished"

        logging.info('UAV: %s can send packet at: %s', self.my_drone.identifier, self.env.now)

        transmission_mode = pkd.transmission_mode

        if transmission_mode == 0:  # for unicast
            # only unicast data packets need to wait for ACK
            logging.info('UAV: %s start to wait ACK for packet: %s at time: %s',
                         self.my_drone.identifier, pkd.packet_id, self.env.now)

            next_hop_id = pkd.next_hop_id

            pkd.increase_ttl()
            self.phy.unicast(pkd, next_hop_id)  # note: unicast function should be executed first!
            yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)  # transmission delay

            if self.enable_ack:
                # used to identify the process of waiting ack
                key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
                self.wait_ack_process = self.env.process(self.wait_ack(pkd))
                self.wait_ack_process_dict[key2] = self.wait_ack_process
                self.wait_ack_process_finish[key2] = 0  # indicate that this process hasn't finished

                # continue to occupy the channel to prevent the ACK from being interfered
                yield self.env.timeout(config.SIFS_DURATION + config.ACK_PACKET_LENGTH / config.BIT_RATE * 1e6)

        elif transmission_mode == 1:
            pkd.increase_ttl()
            self.phy.broadcast(pkd)
            yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)

    def wait_ack(self, pkd):
        """
        If ACK is received within the specified time, the transmission is successful, otherwise,
        a re-transmission will be originated
        :param pkd: the data packet that waits for ACK
        :return: none
        """

        try:
            yield self.env.timeout(config.ACK_TIMEOUT)

            logging.info('ACK timeout of packet: %s at: %s', pkd.packet_id, self.env.now)

            if pkd.number_retransmission_attempt[self.my_drone.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                # random wait
                transmission_attempt = pkd.number_retransmission_attempt[self.my_drone.identifier]
                r = self.rng_mac.randint(0, 2 ** transmission_attempt)
                waiting_time = r * 500

                yield self.env.timeout(waiting_time)
                yield self.env.process(self.my_drone.packet_coming(pkd))  # resend
            else:
                self.simulator.metrics.mac_delay.append((self.simulator.env.now - pkd.first_attempt_time) / 1e3)

                key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
                self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1

                logging.info('Packet: %s is dropped!', pkd.packet_id)

        except simpy.Interrupt:
            # receive ACK in time
            logging.info('UAV: %s receives the ACK for data packet: %s, at: %s',
                         self.my_drone.identifier, pkd.packet_id, self.env.now)
