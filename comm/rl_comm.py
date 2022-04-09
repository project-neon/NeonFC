import os
import json
import struct
import serial
import socket
import threading

from random import randint

import json

def get_config(config_file=None):
    if config_file:
        config = json.loads(open(config_file, 'r').read())
    else:
        config = json.loads(open('config_real_life.json', 'r').read())

    return config

class RLComm(object):
    def __init__(self):
        super(RLComm, self).__init__()
        self.config = get_config()
        self.commands = []
        self.command_port = os.environ.get('COMM_PORT', self.config['comm']['comm_port'])
        self.baud_rate = int(os.environ.get('BAUD_RATE', self.config['comm']['baud_rate']))

    def start(self):
        print("Starting communication...")
        self.comm = serial.Serial(self.command_port, self.baud_rate)
        print(f"Communication port created on {self.command_port}!")
    
    def send(self, robot_commands = []):
        '''
        Send commands to FIRASim

        robot_commands follows:
        [
            {
                robot_id: int,
                color: 'yellow|blue',
                wheel_left: float,
                wheel_right: float,
            }
        ]
        '''
        message = ""
        for rb in robot_commands:
            message += f"{rb['robot_id']} {rb['wheel_right']} {rb['wheel_left']},"

        message = message[:-1]

        self.comm.write(message.encode())

        battery = self.comm.readline().decode('ascii')

        print('mensagem', message)
        print('bateria: ', battery)


    def _get_robot_color(self, robot):
        return True if robot['color'] == 'yellow' else False

if __name__ == "__main__":
    import time
    c = RLComm()

    c.start()

    while True:
        time.sleep(1)
        c.send(
            [
                {
                    'robot_id': 0,
                    'wheel_left': 20,
                    'wheel_right': -20,
                    'color': 'blue'
                },
                # {
                #     'robot_id': 3,
                #     'wheel_left': 40,
                #     'wheel_right': -40,
                #     'color': 'blue'
                # },
                # {
                #     'robot_id': 9,
                #     'wheel_left': 60,
                #     'wheel_right': -60,
                #     'color': 'blue'
                # }
            ]
        )