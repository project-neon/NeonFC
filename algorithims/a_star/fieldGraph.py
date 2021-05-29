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


# creates nodes on the field to add to the graph that astar is going to use
class FieldGraph():
    def __init__(self, start_position):
        self.start = Node(start_position)
        # graph tests here
    
    # obs - remember that start node's neighbours shouldn't have the start node as their neighbor in order not to have loop to the start node
