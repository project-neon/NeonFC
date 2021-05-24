class Node:
    def __init__(self, position):
        self.previous = None
        self.position = position
        self.g = 0 # distance so far
        self.h = 0 # heuristic/distance to target
        self.f = 0 # g plus h
        self.neighbours = []


# creates nodes on the field to add to the graph that astar is going to use
class FieldGraph():
    def __init__(self, start_position):
        self.start = Node(start_position)
        # self.nodes = list with the nodes on the field
        self.nodes = []

    # method to create graph
