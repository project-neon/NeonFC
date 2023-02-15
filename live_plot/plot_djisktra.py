import sys
import json

import socket
import struct

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import _thread as thread

MCAST_GRP = '224.1.2.3'
MCAST_PORT = 5007

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

sock.bind((MCAST_GRP, MCAST_PORT))

mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)

sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

class RectItem(pg.GraphicsObject):
    def __init__(self, rect, parent=None):
        super().__init__(parent)
        self._rect = rect
        self.picture = QtGui.QPicture()
        self._generate_picture()

    @property
    def rect(self):
        return self._rect

    def _generate_picture(self):
        painter = QtGui.QPainter(self.picture)
        painter.setPen(pg.mkPen("w"))
        painter.setBrush(pg.mkBrush("g"))
        painter.drawRect(self.rect)
        painter.end()

    def paint(self, painter, option, widget=None):
        painter.drawPicture(0, 0, self.picture)

    def boundingRect(self):
        return QtCore.QRectF(self.picture.boundingRect())

def update_points():
    data = socketparse()
    node_spots = [{'pos': [n['x'], n['y']], 'data': 1, 'brush': pg.intColor(100 if n['name'] == 'BALL' else 300)} for n in data['nodes']]
    obs_spots = [{'pos': [n['x'], n['y']], 'data': 1, 'brush': pg.intColor(800)} for n in data['obstacles']]
    rect_item = RectItem(QtCore.QRectF(0, 0, 1.5, 1.3))
    pf.addItem(rect_item)
    pf.addItem(scatter)

    scatter.clear()
    scatter.addPoints(node_spots)
    scatter.addPoints(obs_spots)

    pf.clear()
    lines = [pg.LineSegmentROI(a, pen=(1,1)) for a in data['edges']]
    for line in lines:
        pf.addItem(line)


def socketparse():
    data = sock.recv(40023)
    data = json.loads(data.decode('utf8').replace("'", '"'))
    if not data:
        return
    
    data_point = data[0]['data']

    return data_point

points = []

win = pg.GraphicsWindow()
win.setWindowTitle('Waypoint pathfinding - live plot')
pf = win.addPlot()


data = socketparse()

node_spots = [{'pos': [n['x'], n['y']], 'data': 1, 'brush': pg.intColor(300)} for n in data['nodes']]
obs_spots = [{'pos': [n['x'], n['y']], 'data': 1, 'brush': pg.intColor(100)} for n in data['obstacles']]

edges = []


scatter = pg.ScatterPlotItem(size=10)
scatter.addPoints(node_spots)
scatter.addPoints(obs_spots)


def updateplot():
    update_points()

timer = pg.QtCore.QTimer()
timer.timeout.connect(updateplot)
timer.start( (1000/60) * 5)

if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
