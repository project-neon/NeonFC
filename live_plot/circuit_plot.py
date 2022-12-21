from bokeh.plotting import curdoc, figure
import json
import socket

UDP_IP_ADDRESS = 'localhost'
UDP_PORT_NO = 43215

serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))

def update():
    raw_data, addr = serverSock.recvfrom(4096)
    data = json.loads(raw_data)
    x = data['robot']['x']
    y = data['robot']['y']
    print(data)
    circuit_x = [p[0] for p in data['circuit']]
    circuit_y = [p[1] for p in data['circuit']]

    obstacles_x = [p['x'] for p in data['obstacles']]
    obstacles_y = [p['y'] for p in data['obstacles']]

    r.data_source.stream({'x': [x], 'y': [y]})
    
    c.data_source.stream({'x': circuit_x, 'y': circuit_y})
    o.data_source.stream({'x': obstacles_x, 'y': obstacles_y})
    
    

p = figure()
r = p.circle([], [])
c = p.circle([], [], color="olive")
o = p.circle([], [], color="red")

curdoc().add_root(p)
curdoc().add_root(c)

curdoc().add_periodic_callback(update, 1)