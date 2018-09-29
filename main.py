import sys
from _ast import List
from abc import abstractmethod
import math
import random

DIRECTIONS_START = -1
DIRECTIONS_END = 2


class Vertex:  # Defined as a tile,, pre calculate the cost of these

    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.parent = self
        self.filled = False

        self.neighbors = []
        self.edgeCost = 0

    def name(self):
        return str(self.x) + " " + str(self.y)

    def reset(self):
        self.edgeCost = 0
        self.parent = self


class Graph:  # We are going for a graph.vertex based approach, so we just need a max height and width as our params

    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.vertices = [[Vertex(x, y) for y in range(height)] for x in range(width)]  # type List[List[Vertex]

        self.vertices[1][2].filled = True  # Todo remove this when loading real fill data
        self.vertices[1][1].filled = True
        self.vertices[2][2].filled = True
        self.vertices[2][1].filled = True

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

                neighbor = self.vertices[translated_x][translated_y]
                if ((x == 1 and y == 1) or (x == -1 and y == -1) or (x == -1 and y == 1) or (x == 1 and y == -1)) \
                        and neighbor.filled is True and vertex.filled is True:  # block off filled diagonal neighbors
                    continue

                vertex.neighbors.append(neighbor)


class Path:

    @abstractmethod
    def findPath(self, start, goal): raise NotImplementedError

    @abstractmethod
    def updateVertex(self, vertex, neighbor, goal): raise NotImplementedError

    def h(self, vertex, goal):  # the estimated path cost from the node we're at to the goal node

        # return               max(abs(vertex.x - goal.x), abs(vertex.y - goal.y))

        return Path.c(vertex, goal)

    @staticmethod
    def c(from_vertex, to_vertex):  # the straight line distance between the s node and e node.
        return math.sqrt((from_vertex.x - to_vertex.x) ** 2 + (from_vertex.y - to_vertex.y) ** 2)

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
        return (math.sqrt(2) * min_offset) + max(abs(vertex.x - goal.x), abs(vertex.y - goal.y)) - min_offset


class FDAPath(APath):

    def updateVertex(self, vertex, neighbor, goal):
        if self.lineOfSight(vertex.parent, neighbor):
            if vertex.parent.edgeCost + Path.c(vertex.parent, neighbor) < neighbor.edgeCost:
                neighbor.edgeCost = vertex.parent.edgeCost + Path.c(vertex.parent, neighbor)
                neighbor.parent = vertex.parent
                if neighbor in self.fringe:
                    self.fringe.pop(neighbor)
                self.fringe.update({neighbor: neighbor.edgeCost + self.h(neighbor, goal)})
        else:
            if vertex.edgeCost + Path.c(vertex, neighbor) < neighbor.edgeCost:
                neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
                neighbor.parent = vertex
                if neighbor in self.fringe:
                    self.fringe.pop(neighbor)
                self.fringe.update({neighbor: neighbor.edgeCost + self.h(neighbor, goal)})

    def lineOfSight(self, fromVertex, toVertex):
        x0 = fromVertex.x
        y0 = fromVertex.y

        x1 = toVertex.x
        y1 = toVertex.y

        f = 0

        dy = y1 - y0
        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1

        dx = x1 - x0
        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1

        if dx >= dy:
            while x0 != x1:
                f = f + dy
                if f >= dx:
                    if graph.vertices[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)].filled:
                        return False
                    y0 = y0 + sy
                    f = f - dx
                if f != 0 and graph.vertices[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)].filled:
                    return False
                if dy == 0 and graph.vertices[x0 + ((sx - 1) / 2)][y0].filled \
                        and graph.vertices[x0 + ((sx - 1) / 2)][y0 - 1].filled:
                    return False
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx
                if f >= dy:
                    if graph.vertices[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)].filled:
                        return False
                    x0 = x0 + sx
                    f = f - dy
                if f != 0 and graph.vertices[x0 + ((sx - 1) / 2)][y0 + ((sy - 1) / 2)].filled:
                    return False
                if dx == 0 and graph.vertices[x0][y0 + ((sy - 1) / 2)].filled \
                        and graph.vertices[x0 - 1][y0 + ((sy - 1) / 2)].filled:
                    return False
                y0 = y0 + sy
        return True


class TraceFDAPath(FDAPath):

    def h(self, vertex, goal):
        min_offset = min(abs(vertex.x - goal.x), abs(vertex.y - goal.y))
        return math.sqrt(2) * min_offset + max(abs(vertex.x - goal.x), abs(vertex.y - goal.y)) - min_offset


def pathFromGoal(vertex):
    path = []
    while True:
        path.append(vertex)
        if vertex.parent is vertex:
            break
        vertex = vertex.parent
    return path[::-1]


def pop(dictionary):  # need to sort this better
    key = min(dictionary.keys(), key=(lambda k: (dictionary[k], dictionary[k] - sys.maxunicode * k.edgeCost)))
    dictionary.pop(key)
    return key


graph = Graph(5, 3)
astar = APath()
trace = TracePath()
fda = FDAPath()

p3 = fda.findPath(graph.vertices[0][0], graph.vertices[4][2])

for v in p3:
    print v.name()
