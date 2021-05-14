from algorithims.discretizedField import DiscreteField
import heapq, time, math
from multiprocessing import Process
from threading import Thread
import api

class Node:

    def __init__(self, parent=None, pos=None):
        self.parent = parent
        self.position = pos
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __gt__(self, other):
        return self.f > other.f


class AStar():
    def __init__(self):
        super(AStar, self).__init__()
        self.maze = DiscreteField()
        self.last_calculation = 0
        self.path = []

        self.start_pos_last_calc = None
        self.target_pos_last_calc = None

    def reset_calculation_timespan(self):
        self.last_calculation = 0

    def reset(self):
        self.start_pos_last_calc = None
        self.target_pos_last_calc = None
        self.last_calculation = 0
    
    def update(self, start, target):
        self.start_pos_last_calc = start
        self.target_pos_last_calc = target

    def update_field(self, obstacles):
        self.maze.update(avoiances=obstacles)

    def calculate_when(self, start, target, timespan=1):
        if (not start) or (not target):
            return

        if time.time() - self.last_calculation > timespan:
            x = time.time()
            self.calculate(start, target)

    def recalculate(self):
        while True:
            self.calculate_when(
                self.start_pos_last_calc,
                self.target_pos_last_calc,
                timespan=0.25
            )
    
    def start(self):
        
        self.__calc_process = Thread(target=self.recalculate, )
        self.__calc_process.start()

    def return_path(self, current_node):
        path = []
        current = current_node
        while current is not None:
            path.append(current.position)
            current = current.parent

        return path[::-1]

    def calculate(self, start, target):
        self.last_calculation = time.time()
        start = (self.maze._fm)(*start)
        target = (self.maze._fm)(*target)
        start_node = Node(None, start)
        end_node = Node(None, target)
        open_list = []
        closed_list = []
        heapq.heapify(open_list)
        heapq.heappush(open_list, start_node)
        outer_iterations = 0
        max_iterations = len(self.maze.matrix[0]) * len(self.maze.matrix) // 2
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1),
                            (1, 1))
        while len(open_list) > 0:
            outer_iterations += 1
            if outer_iterations > max_iterations:
                self.path = self.return_path(current_node)
                return
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)
            if current_node == end_node:
                self.path = self.return_path(current_node)
                return
            children = []
            for new_position in adjacent_squares:
                node_position = (
                 current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                if not node_position[0] > len(self.maze.matrix) - 1:
                    if node_position[0] < 0 or node_position[1] > len(self.maze.matrix[(len(self.maze.matrix) - 1)]) - 1 or node_position[1] < 0:
                        continue
                    if self.maze.matrix[node_position[0]][node_position[1]] != 0:
                        continue
                    new_node = Node(current_node, node_position)
                    children.append(new_node)

            for child in children:
                if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                    continue
                child.g = current_node.g + 1
                child.h = (child.position[0] - end_node.position[0]) ** 2 + (child.position[1] - end_node.position[1]) ** 2
                child.f = child.g + child.h
                if len([open_node for open_node in open_list if child.position == open_node.position if child.g > open_node.g]) > 0:
                    continue
                heapq.heappush(open_list, child)

    def next_node(self, x, y):
        if len(self.path):
            if self.maze._fm(x, y) == self.path[0]:
                coords = self.path.pop(0)
                return (coords[0] * self.maze.resolution / 1000, coords[1] * self.maze.resolution / 1000)
            return (self.path[0][0] * self.maze.resolution / 1000, self.path[0][1] * self.maze.resolution / 1000)
        print('NO ROUTE!')
        return (-1, -1)