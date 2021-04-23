import json
import socket
import struct

from time import sleep

import threading

class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.

    source: https://refactoring.guru/design-patterns/singleton/python/example
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]

class DataSender(metaclass=SingletonMeta):
    def __init__(self):
        super(DataSender, self).__init__()
        self.nodes = {}
        # talvez arquivo de congf?
        self.multicast_group = '224.1.2.3'
        self.multicast_port = 5007
        self.multicast_ttl = 2

        self.sock = None
        
    def append_datanode(self, datanode):
        self.nodes[datanode.node_name] = datanode
    

    def get_node(self, name):
        if not self.nodes.get(name):
            print(f'Data node [{name}] not registered! registering now!')

            new_node = DataNode(name)
            self.append_datanode(new_node)

        return self.nodes.get(name)
    
    def start(self):
        print("Starting Data Sender...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, self.multicast_ttl)
        print(f"Data Sender started at {self.multicast_group}:{self.multicast_port}!")

        # while True:
        #     self.send_data()

    def send_data(self):
        data_pack = []

        if not self.nodes.keys():
            return

        for node in self.nodes.values():
            d_ = node.retrieve()
            if d_:
                data_pack.append({'name': node.node_name, 'data': d_})

        data_bytes = bytes(json.dumps(data_pack), 'utf-8')
        
        self.sock.sendto(data_bytes, (self.multicast_group, self.multicast_port))


class DataNode(object):
    def __init__(self, node_name):
        self.node_name = node_name
        self._buffer_data = None

    def capture(self, **data):
        self._buffer_data = data
    
    def retrieve(self):
        b_data = self._buffer_data
        self._buffer_data = None
        return b_data

if __name__ == '__main__':
    import random
    s1 = DataSender()

    dn = DataNode('PID_ROBOT_1')
    dn2 = DataNode('PID_ROBOT_2')

    s1.start()
    sleep(1)
    
    s1.append_datanode(dn)
    s1.append_datanode(dn2)
    while True:
        sleep(1/1000)
        s1.get_node('PID_ROBOT_1').capture(error=random.random())
        s1.get_node('PID_ROBOT_2').capture(error=random.random())