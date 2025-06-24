import simpy
import logging
import random
from phy.phy import Phy
from utils import config
from utils.util_function import check_channel_availability

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )


class CsmaCa:
    """
    Medium access control protocol: CSMA/CA (Carrier Sense Multiple Access With Collision Avoidance) without RTS/CTS

    The basic flow of the basic CSMA/CA (without RTS/CTS) is as follows:
        1) when a node has a packet to send, it first needs to wait until the channel is idle
        2) when the channel is idle, the node starts a timer and waits for "DIFS+backoff" periods of time, where the
           length of backoff is related to the number of re-transmissions
        3) if the entire decrement of the timer to 0 is not interrupted, then the node can occupy the channel and start
           sending the data packet and waiting for the corresponding ACK
        4) if the countdown is interrupted, it means that the node loses the game. The node should freeze the timer and
           wait for channel idle again before re-starting its timer

    Main attributes:
        my_drone: the drone that installed the CSMA/CA protocol
        simulator: the simulation platform that contains everything
        rng_mac: a Random class based on which we can call the function that generates the random number
        env: simulation environment created by simpy
        phy: the installed physical layer
        channel_states: used to determine if the channel is idle
        enable_ack: use ack or not

    References:
        [1] J. Li, et al., "Packet Delay in UAV Wireless Networks Under Non-saturated Traffic and Channel Fading
            Conditions," Wireless Personal Communications, vol. 72, no. 2, pp. 1105-1123, 2013.
        [2] M. A. Siddique and J. Kamruzzaman, "Performance Analysis of M-Retry BEB Based DCF Under Unsaturated Traffic
            Condition," in 2010 IEEE Wireless Communication and Networking Conference, 2010, pp. 1-6.
        [3] F. Daneshgaran, M. Laddomada, F. Mesiti and M. Mondin, "Unsaturated Throughput Analysis of IEEE 802.11 in
            Presence of Non-ideal Transmission Channel and Capture Effects," IEEE transactions on Wireless Communications
            vol. 7, no. 4, pp. 1276-1286, 2008.
        [4] Park, Ki hong, "Wireless Lecture Notes". Purdue. Retrieved 28 September 2012, link:
            https://www.cs.purdue.edu/homes/park/cs536-wireless-3.pdf

    Author: Zihao Zhou, eezihaozhou@gmail.com
    Created at: 2024/1/11
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
        """
        Control when drone can send packet
        :param pkd: the packet that needs to send
        :return: none
        """

        transmission_attempt = pkd.number_retransmission_attempt[self.my_drone.identifier]
        contention_window = (config.CW_MIN + 1) * (2 ** (transmission_attempt-1)) - 1

        backoff = self.rng_mac.randint(0, contention_window - 1) * config.SLOT_DURATION  # random backoff, in us
        to_wait = config.DIFS_DURATION + backoff

        logging.info('UAV: %s back-off is: %s', self.my_drone.identifier, backoff)

        while to_wait:
            # wait until the channel becomes idle
            yield self.env.process(self.wait_idle_channel(self.my_drone, self.simulator.drones))

            if pkd.number_retransmission_attempt[self.my_drone.identifier] == 1:
                """
                NOTE: because the service time of the packet is given by the interval between the time when this packet
                starts backoff and the time when it is acknowledged and removed from the queue, so the "backoff_start_
                time" should be recorded only when the drone transmits this packet for the first time.
                """
                pkd.first_attempt_time = self.env.now

            # start listen the channel at backoff stage
            self.env.process(self.listen(self.channel_states, self.simulator.drones, pkd))

            logging.info('UAV: %s should wait from: %s, and wait for %s',
                         self.my_drone.identifier, self.env.now, to_wait)
            start_time = self.env.now  # start to wait

            try:
                yield self.env.timeout(to_wait)
                to_wait = 0  # to break the while loop

                key = 'mac_send' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
                self.my_drone.mac_process_finish[key] = 1  # mark the process as "finished"

                # occupy the channel to send packet
                with self.channel_states[self.my_drone.identifier].request() as req:
                    yield req

                    logging.info('UAV: %s can send packet (pkd id: %s) at: %s ',
                                 self.my_drone.identifier, pkd.packet_id, self.env.now)

                    pkd.transmitting_start_time = self.env.now
                    transmission_mode = pkd.transmission_mode

                    if transmission_mode == 0:  # for unicast
                        next_hop_id = pkd.next_hop_id

                        pkd.increase_ttl()
                        self.phy.unicast(pkd, next_hop_id)  # note: unicast function should be executed first!
                        yield self.env.timeout(pkd.packet_length / config.BIT_RATE * 1e6)  # transmission delay

                        # only unicast data packets need to wait for ACK
                        logging.info('UAV: %s start to wait ACK for packet: %s at time: %s',
                                     self.my_drone.identifier, pkd.packet_id, self.env.now)

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

            except simpy.Interrupt:
                already_wait = self.env.now - start_time
                logging.info('UAV: %s was interrupted at: %s, already waits for: %s, original to_wait is: %s',
                             self.my_drone.identifier, self.env.now, already_wait, to_wait)

                to_wait -= already_wait  # the remaining waiting time

                if to_wait > backoff:
                    # interrupted in the process of DIFS
                    to_wait = config.DIFS_DURATION + backoff
                else:
                    # interrupted in the process of backoff, freeze the backoff
                    backoff = to_wait  # remaining backoff time
                    to_wait = config.DIFS_DURATION + backoff

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
                yield self.env.process(self.my_drone.packet_coming(pkd))
            else:
                self.simulator.metrics.mac_delay.append((self.simulator.env.now - pkd.first_attempt_time) / 1e3)

                key2 = 'wait_ack' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
                self.my_drone.mac_protocol.wait_ack_process_finish[key2] = 1

                logging.info('Packet: %s is dropped!', pkd.packet_id)

        except simpy.Interrupt:
            # receive ACK in time
            logging.info('UAV: %s receives the ACK for data packet: %s, at: %s',
                         self.my_drone.identifier, pkd.packet_id, self.env.now)

    def wait_idle_channel(self, sender_drone, drones):
        """
        Wait until the channel becomes idle
        :param sender_drone: the drone that is about to send packet
        :param drones: a list, which contains all the drones in the simulation
        :return: none
        """

        while not check_channel_availability(self.channel_states, sender_drone, drones):
            yield self.env.timeout(config.SLOT_DURATION)

    def listen(self, channel_states, drones, pkd):
        """
        When the drone waits until the channel is idle, it starts its own timer to count down, in this time, the drone
        needs to detect the state of the channel during this period, and if the channel is found to be busy again, the
        countdown process should be interrupted
        :param channel_states: a dictionary, indicates the use of the channel by different drones
        :param drones: a list, contains all drones in the simulation
        :param pkd: listen to the channel for which packet
        :return: none
        """

        logging.info('At time: %s, UAV: %s starts to listen the channel and perform backoff',
                     self.env.now, self.my_drone.identifier)

        key = 'mac_send' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)

        while self.my_drone.mac_process_finish[key] == 0:  # interrupt only if the process is not complete
            if check_channel_availability(channel_states, self.my_drone, drones) is False:
                # found channel be occupied, start interrupt

                key = 'mac_send' + str(self.my_drone.identifier) + '_' + str(pkd.packet_id)
                if not self.my_drone.mac_process_dict[key].triggered:
                    self.my_drone.mac_process_dict[key].interrupt()
                    break
            else:
                pass

            yield self.env.timeout(1)
