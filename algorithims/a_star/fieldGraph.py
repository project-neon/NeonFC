class Node:
    def __init__(self, position):
        self.previous = None
        self.position = position
        self.g = 0 # distance so far
        self.h = 0 # heuristic/distance to target
        self.f = 0 # g plus h
        self.neighbours = []

    def __lt__(self, other):
        if (self.f == other.f):
            return self.h < other.h
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


# creates nodes on the field and add them to the graph that astar is going to use
# currently only being used for testing the astar algorithm
class FieldGraph():
    def __init__(self, start_position):
        self.start = Node(start_position)

    def update_neighbours(self, edges):
        for edge in edges:
            edge[0].neighbours.append(edge[1])
            edge[1].neighbours.append(edge[0])

    def create_graph(self):
        A = Node([3, 10])
        B = Node([2, 6])
        C = Node([4, 2])
        D = Node([8, 4])
        E = Node([11, 8])
        F = Node([6, 8])
        edges = [[A, B], [B, C], [D, B], [D, C], [C, F], [D, F], [D, E], [E, F]]
        self.update_neighbours(edges)
        self.start = F
        return self.start
