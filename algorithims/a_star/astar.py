import numpy
import heapq


def distance(pos1, pos2):
    # Considering pos = [x, y] or pos = (x, y)
    x1 = pos1[0]
    x2 = pos2[0]
    y1 = pos1[1]
    y2 = pos2[1]
    d = numpy.sqrt(((x2-x1)**2 + (y2-y1)**2))
    return d


class Node:
    def __init__(self, position):
        self.previous = None
        self.position = position
        self.g = 0 # distance so far
        self.h = 0 # heuristic/distance to target
        self.f = 0 # g plus h
        self.neighbours = []

    def set_position(self, position = []):
        self.position = position

    def __lt__(self, other):
        if (self.f == other.f):
            return self.h < other.h
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


class AStar():
    def __init__(self, initial_node, target_position):
        self.start = initial_node
        self.target_position = target_position
        self.open_list = []
        self.closed_list = []
    
    def create_path(self, current):
        backtracking = []
        while current != None:
            backtracking.append(current.position)
            current = current.previous
        return backtracking[::-1]

    def calculate(self):
        self.open_list = [self.start]
        heapq.heapify(self.open_list)
        while len(self.open_list) > 0:
            # open_list nodes are ordered by lowest f value to highest
            # in case of a tie, lowest h is prioritized
            # current is the node with the smallest f value and is the first in the open_list
            current = self.open_list[0]
            if current.position == self.target_position:
                path_to_target = self.create_path(current)
                return path_to_target
            self.closed_list.append(heapq.heappop(self.open_list))
            for neighbor in current.neighbours:
                if neighbor not in self.closed_list:
                    if neighbor not in self.open_list:
                        neighbor.h = distance(neighbor.position, self.target_position)
                        neighbor.g = distance(neighbor.position, current.position)
                        neighbor.f = neighbor.g + neighbor.h
                        neighbor.previous = current
                    else:
                        dist_from_previous = current.g + distance(neighbor.position, current.position)
                        if dist_from_previous < neighbor.g:
                            neighbor.g = dist_from_previous
                            neighbor.f = neighbor.g + neighbor.h
                            neighbor.previous = current
                            self.open_list.remove(neighbor)
                    heapq.heappush(self.open_list, neighbor)
        return []
