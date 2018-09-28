import sys
from _ast import List
from abc import abstractmethod
import math

DIRECTIONS_START = -1
DIRECTIONS_END = 2


class Vertex:  # Defined as a tile,, pre calculate the cost of these

    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.parent = None
        self.neighbors = []
        self.edgeCost = 0

    def name(self):
        return str(self.x) + " " + str(self.y)

    def reset(self):
        self.edgeCost = 0
        self.parent = None


class Graph:  # We are going for a grid based approach, so we just need a max height and width as our params

    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.vertices = [[Vertex(x, y) for y in range(height)] for x in range(width)]  # type List[List[Vertex]
        for x in range(width):
            for y in range(height):
                self.populateNeighbors(self.vertices[x][y])

    def populateNeighbors(self, vertex):
        for x in range(DIRECTIONS_START, DIRECTIONS_END):
            for y in range(DIRECTIONS_START, DIRECTIONS_END):
                if x == 0 and y == 0:
                    continue

                translated_x = vertex.x + x
                translated_y = vertex.y + y
                if translated_x >= self.width or translated_x < 0 or translated_y >= self.height or translated_y < 0:
                    continue
                vertex.neighbors.append(self.vertices[translated_x][translated_y])


class Path:

    @abstractmethod
    def findPath(self, start, goal): raise NotImplementedError

    @abstractmethod
    def updateVertex(self, vertex, neighbor, goal): raise NotImplementedError

    def h(self, vertex, goal):  # the estimated path cost from the node we're at to the goal node
        return Path.c(vertex, goal)

    @staticmethod
    def c(from_vertex, to_vertex):  # the straight line distance between the s node and e node.
        dx = from_vertex.x - to_vertex.x
        dy = from_vertex.y - to_vertex.y
        return math.sqrt(dx * dx + dy * dy)

    def f(self, vertex):
        return vertex.edgeCost + self.h(vertex)


class APath(Path):

    def __init__(self):
        self.fringe = dict()

    def findPath(self, start, goal):
        closed = []  # type: List[Vertex]

        start.reset()
        goal.reset()

        self.fringe.clear()
        self.fringe.update({start: start.edgeCost + self.h(start, goal)})

        while len(self.fringe) > 0:
            vertex = pop(self.fringe)
            if vertex == goal:
                return pathFromGoal(vertex)

            closed.append(vertex)
            for neighbor in vertex.neighbors:
                if neighbor not in closed:
                    if neighbor not in self.fringe:
                        neighbor.edgeCost = sys.maxint
                        neighbor.parent = None
                    self.updateVertex(vertex, neighbor, goal)
        return None

    def updateVertex(self, vertex, neighbor, goal):
        if vertex.edgeCost + Path.c(vertex, neighbor) >= neighbor.edgeCost:
            pass
        neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
        neighbor.parent = vertex
        if neighbor in self.fringe:
            self.fringe.pop(neighbor)
        self.fringe.update({neighbor: neighbor.edgeCost + self.h(neighbor, goal)})


class TracePath(APath):

    def h(self, vertex, goal):
        min_offset = min(abs(vertex.x - goal.x), abs(vertex.y - goal.y))
        return math.sqrt(2) * min_offset + max(abs(vertex.x - goal.x), abs(vertex.y - goal.y)) - min_offset


class FDAPath(APath):

    def updateVertex(self, vertex, neighbor, goal):
        if self.lineOfSight(vertex.parent, vertex):
            if vertex.parent.edgeCost + Path.c(vertex.parent, neighbor) < neighbor.edgeCost:
                neighbor.edgeCost = vertex.parent.edgeCost + Path.c(vertex.parent, neighbor)
                neighbor.parent = vertex.parent
                if neighbor in self.fringe:
                    self.fringe.pop(neighbor)
                self.fringe.update({neighbor: neighbor.edgeCost + self.h(neighbor, goal)})
        else:
            if vertex.edgeCost + Path.c(vertex, neighbor) < neighbor.edgeCost:
                neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
                neighbor.parent = vertex.parent
                if neighbor in self.fringe:
                    self.fringe.update({neighbor: neighbor.edgeCost + self.h(neighbor, goal)})

    def lineOfSight(self, parent, vertex):
        pass


class TraceFDAPath(FDAPath):

    def h(self, vertex, goal):
        min_offset = min(abs(vertex.x - goal.x), abs(vertex.y - goal.y))
        return math.sqrt(2) * min_offset + max(abs(vertex.x - goal.x), abs(vertex.y - goal.y)) - min_offset


def pathFromGoal(vertex):
    path = []
    while vertex is not None:
        path.append(vertex)
        vertex = vertex.parent
    return path[::-1]


def pop(dictionary):
    key = min(dictionary.keys(), key=(lambda k: dictionary[k]))
    dictionary.pop(key)
    return key


graph = Graph(5, 3)
astar = APath()
trace = TracePath()

p1 = astar.findPath(graph.vertices[4][2], graph.vertices[0][0])
p2 = trace.findPath(graph.vertices[4][2], graph.vertices[0][0])

for v in p1:
    print v.name()
print "\n\n"

for v in p2:
    print v.name()
