import os
import serial
import json

def get_config(config_file=None):
    if config_file:
        config = json.loads(open(config_file, 'r').read())
    else:
        config = json.loads(open('config_real_life.json', 'r').read())

    return config

def find_esp():
    print('trying to find esp32')
    options: list[string] = []


    for file in os.listdir("/dev/"):
        if file.startswith("ttyACM") or file.startswith("ttyUSB"):
            print("found potential file: %s",file)
            options.append("/dev/"+file)
        pass

    if len(options) == 0:
        print("could not find ESP32 pseudofile")
        return None
    elif len(options) == 1:
        print("found file %s as potential ESP32 file",options[0])
        return options[0]
    else:
        print("found multiple options for ESP32 pseudofile, please select one:")
        i = 1
        for potential_file in options:
            print("[%d]: \"%s\"".format(i,potential_file))
            i+=1
        location = int(input())
        final_file = options[location-1]
        print("using %s as ESP32 file".format(final_file))
        return final_file

class RLComm(object):
    def __init__(self):
        super(RLComm, self).__init__()
        self.config = get_config() 
        # FIXME puxa isso do ambiente e não do arquivo (pode ser que seja um parametro)
        # acho que ele faz isso aqui embaixo, mas dá uma olhada
        self.commands = []
        # comm port é o pseudoarquivo que precisa estar nossa ESP32
        self.command_port = os.environ.get('COMM_PORT', self.config['comm']['comm_port'])
        self.baud_rate = int(os.environ.get('BAUD_RATE', self.config['comm']['baud_rate']))

        if self.command_port == 'auto': self.command_port = find_esp()


    def start(self):
        print("Starting communication...")
        self.comm = serial.Serial(self.command_port, self.baud_rate)
        print(f"Communication port created on {self.command_port}!")
    
    def send(self, robot_commands = []):
        '''
        Send commands to ESP-32

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
        message = "<"
        robot_commands = sorted(robot_commands, key = lambda i: i['robot_id'])
        for rb in robot_commands:
            message += f"{rb['robot_id']},{round(rb['wheel_left'], 4)},{round(rb['wheel_right'], 4)},"

        message = message[:-1] + '>'

        self.comm.write(message.encode())

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
                }
            ]
        )
