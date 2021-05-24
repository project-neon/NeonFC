import numpy

from algorithims.a_star.fieldGraph import Node, FieldGraph


def distance(pos1, pos2):
    # Considering pos = [x, y] or pos = (x, y)
    x1 = pos1[0]
    x2 = pos2[0]
    y1 = pos1[1]
    y2 = pos2[1]
    d = numpy.sqrt(((x2-x1)**2 + (y2-y1)**2))
    return d


class AStar():
    def __init__(self, start_position, target_position):
        self.start = FieldGraph().start
        self.target_position = target_position
        self.open_list = []
        # self.open_set.update(FieldGraph().graph/nodes)
        self.closed_list = []
    
    def create_path(self, current):
        backtracking = []
        while current != None:
            backtracking.append(current.position)
            current = current.previous
        return backtracking[::-1]

    def calculate(self):
        # heapq?
        self.open_list = [self.start]
        while len(self.open_list) > 0:
            # open_list nodes are ordered by lowest f value to highest
            # in case of a tie, lowest h is prioritized
            # current is the node with the smallest f value and should be the first on the open_list
            current = self.open_list[0]
            if current.pos == self.target_position:
                path = self.create_path(current)
                return path
            for neighbor in current.neighbours:
                # If neighbor is NOT in closed_list
                dist_from_previous = current.g + distance(neighbor.position, current.position)
                if neighbor.h == 0:
                    neighbor.h = distance(neighbor.position, self.target_position)
                if neighbor.g == 0 or dist_from_previous < neighbor.g:
                    neighbor.g = dist_from_previous
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.previous = current
                # self.open_list.remove(neighbor)?
                # self.open_list push neighbor?
            # self.closed_list.append(current)
            # self.open_list.remove(current)
