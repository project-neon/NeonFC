import sys
import json

import socket
import struct

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import _thread as thread

MCAST_GRP = '224.1.2.3'
MCAST_PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

sock.bind((MCAST_GRP, MCAST_PORT))

mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

def socketparse():
    data = sock.recv(1024)
    data = json.loads(data.decode('utf8').replace("'", '"'))
    if not data:
        return
    
    data_point = data[0]['data']

    return data_point


desired_linear = []
now_linear = []
new_linear = []
desired_angular = []
now_angular = []
new_angular = []

for x in range(50):
    while True:
        data_point = socketparse()
        if data_point:
            break

    desired_linear.append(data_point['desired_linear'])
    now_linear.append(data_point['now_linear'])
    new_linear.append(data_point['new_linear'])

    desired_angular.append(data_point['desired_angular'])
    now_angular.append(data_point['now_angular'])
    new_angular.append(data_point['new_angular'])

win = pg.GraphicsWindow()
win.setWindowTitle('PID fine-tuning data - Live plot - Angular')
p1 = win.addPlot()

curve1 = p1.plot(desired_angular, pen=pg.mkPen('r', width=3))
curve2 = p1.plot(now_angular, pen=pg.mkPen('g', width=3))
# curve3 = p1.plot(new_linear, pen=pg.mkPen('b', width=3))

ptr1 = 0

def updatelist():
    global desired_linear, now_linear, new_linear, curve1, curve2,  ptr1
    while True:
        data_point = socketparse()
        if data_point:
            break
    

    temp = data_point['desired_angular']

    #temperature plot
    desired_angular[:-1] = desired_angular[1:]
    desired_angular[-1] = temp
    curve1.setData(desired_angular)

    temp = data_point['now_angular']

    #temperature plot
    now_angular[:-1] = now_angular[1:]
    now_angular[-1] = temp
    curve2.setData(now_angular)

    # temp = data_point['new_linear']

    # #temperature plot
    # new_linear[:-1] = new_linear[1:]
    # new_linear[-1] = temp
    # curve3.setData(new_linear)

    ptr1 += 1

def updateplot():
    updatelist()

timer = pg.QtCore.QTimer()
timer.timeout.connect(updateplot)
timer.start(50)

if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    
