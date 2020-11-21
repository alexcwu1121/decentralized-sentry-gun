""" Alex Wu, Rensselaer Polytechnic Institute
    Message queue manager
    # TODO substitute pickle with protobuf
"""
import zmq
from queue import *
import time
import threading
import pickle
import logging

LOG = logging.getLogger("comms")
LOG.setLevel(level=logging.INFO)

class Comms:
    def __init__(self):
        self.subscriber_ports = {}
        self.publisher_ports = {}
        self.subscriber_queues = {}

    def add_publisher_port(self, ip, port, topic):
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://{}:{}".format(ip, port))
        self.publisher_ports[topic] = socket

    def receive(self, topic):
        while True:
            message = self.subscriber_ports[topic].recv()
            self.subscriber_queues[topic].put(message)
            time.sleep(.00000001)

    def add_subscriber_port(self, ip, port, topic):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        socket.connect("tcp://{}:{}".format(ip, port))
        self.subscriber_ports[topic] = socket
        self.subscriber_queues[topic] = Queue()
        t = threading.Thread(target=self.receive, args=(topic,))
        t.start()

    def get(self, topic):
        # Non blocking queue get because system is fully asynchronous
        encoded = self.subscriber_queues[topic].get(False)
        return pickle.loads(encoded)

    def send(self, topic, message):
        encoded = pickle.dumps(message)
        self.publisher_ports[topic].send(encoded)

    def define_message(self, origin, topic, payload):
        msg = Message(origin, topic, payload)
        return msg

    def define_and_send(self, origin, topic, payload):
        msg = Message(origin, topic, payload)
        encoded = pickle.dumps(msg)
        self.publisher_ports[topic].send(encoded)

"""
Potential message payloads:
    1. 2x1 np array holding configuration details (q1, q2)
    2. 2xn np array holding path matrices
    3. 3x1 np array holding a position vector
"""
class Message:
    def __init__(self, origin, topic, payload):
        self.origin = origin
        self.topic = topic
        # Parsed controller readout
        self.payload = payload

    def __str__(self):
        print(">"*20)
        print("Origin process: {}".format(self.origin))
        print("Origin topic: {}".format(self.topic))
        print("Payload: \n{}".format(self.payload))
        print(">" * 20)
