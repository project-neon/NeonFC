import sys
import math

class Vertex:
    def __init__(self, node, x, y):
        self.id = node

        self.x = x
        self.y = y

        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = 999999
        # Mark all nodes unvisited        
        self.visited = False  
        # Predecessor
        self.previous = None
    
    def __lt__(self, other):
        return self.distance < other.distance

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()  

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

        self.edges = []

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node, x, y):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node, x, y)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = None):
        if frm.get_id() not in self.vert_dict:
            self.add_vertex(frm)
        if to.get_id() not in self.vert_dict:
            self.add_vertex(to)
        
        self.edges.append([[frm.x, frm.y], [to.x, to.y]])

        if cost == None:
            cost = math.sqrt( (frm.x - to.x)**2 + (frm.y - to.y)**2 )
        


        frm.add_neighbor(to, cost)
        to.add_neighbor(frm, cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous

def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

import heapq

def shortest(v, path):
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return

def dijkstra(aGraph, start_name, target_name):
    # print("Dijkstra's shortest path")
    # Set the distance for the start node to zero
    # print(start_name, target_name, aGraph.vert_dict.keys())
    start = aGraph.vert_dict[start_name]
    target = aGraph.vert_dict[target_name]

    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(), v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance 
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)
            
            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
            else:
                pass

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)

