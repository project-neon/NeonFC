from algorithms.astar.astar import Node

class FieldGraph():
    def __init__(self):
        self.start = None
        self.nodes = None

    def set_start(self, initial_node):
        self.start = initial_node

    def set_nodes(self, nodes = []):
        self.nodes = nodes

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, n1):
        for n2 in self.nodes:
            self.remove_edge([n1, n2])

        self.nodes.remove(n1)

    def update_neighbours(self, edges):
        for edge in edges:
            self.add_edge(edge)

    def add_edge(self, edge = []):
        edge[0].neighbours.append(edge[1])
        edge[1].neighbours.append(edge[0])

    def remove_edge(self, edge = []):
        edge[0].neighbours.remove(edge[1])
        edge[1].neighbours.remove(edge[0])

    def generate_graph(self, nodes = [], edges = []):
        """
        generate_graph is an abstract method that can be used by robots to create
        the nodes on the field and their edges.
        """
        return self.start
