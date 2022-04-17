import serial
from random import randint

# No Linux, a porta não será "COMX" e sim algo parecido com a linha abaixo
esp32 = serial.Serial('/dev/ttyUSB0', 115200)
# esp32 = serial.Serial("COM6", 115200)

i = 0
while i < 200000:
    # n1 = float(input())
    # n2 = float(input())
    id1 = str(0)
    v_r1 = str(10)
    v_l1 = str(340)

    id2 = str(3)
    v_r2 = str(64.2)
    v_l2 = str(43)

    id3 = str(9)
    v_r3 = str(80)
    v_l3 = str(27)

    vel = (f"<{id1},{v_r1},{v_l1},{id2},{v_r2},{v_l2},{id3},{v_r3},{v_l3}>")
    esp32.write(vel.encode())
    print(vel)

esp32.close()