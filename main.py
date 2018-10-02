import sys
from _ast import List
from abc import abstractmethod
import math
import heapq
import hashlib

DIRECTIONS_START = -1
DIRECTIONS_END = 2

GRID_EXPANSION = 10
TURTLE_RADIUS = 3

startAndGoals = []

__metaclass__ = type


class Vertex:  # Defined as a tile,, pre calculate the cost of these

    def __init__(self, x, y):
        self.x = x
        self.y = y

        self.parent = self
        self.filled = False
        self.filtered = False
        self.populated = False

        self.neighbors = []
        self.edgeCost = 0
        self.f = 0

    def name(self):
        x = (self.x - graph.lowXCorrection) * 1.0 / GRID_EXPANSION
        y = (self.y - graph.lowYCorrection) * 1.0 / GRID_EXPANSION

        x_str = str(x)
        if ".8" in x_str:
            x_str = x_str.replace(".8", ".75").replace(".3", ".25")

        y_str = str(y)
        if ".8" in y_str:
            y_str = y_str.replace(".8", ".75").replace(".3", ".25")

        return x_str + " " + y_str

    def reset(self):
        self.edgeCost = 0
        self.parent = self
        self.f = 0

    def __hash__(self):
        return hash((self.x, self.y))

    def __cmp__(self, other):
        return cmp((self.f, self.f - sys.maxunicode * self.edgeCost),
                   (other.f, other.f - sys.maxunicode * other.edgeCost))


class Graph:  # We are going for a graph.vertex based approach, so we just need a max height and width as our params

    def __init__(self, file_directory):
        self.lowXCorrection = 0
        self.lowYCorrection = 0

        state = 0
        with open(file_directory, "r") as mapFile:
            for line in mapFile.readlines():
                if "---" in line:
                    state += 1
                else:
                    split = line.strip().replace("(", "").replace(")", "").split(" ")
                    points = map(lambda (x, y): (
                        int(x * GRID_EXPANSION + self.lowXCorrection),
                        int(y * GRID_EXPANSION + self.lowYCorrection)),
                                 [tuple(map(float, pair.split(','))) for pair in split])

                    if state == 0:
                        points.sort()

                        lowX = points[0][0]
                        lowY = points[0][1]

                        pointsLen = len(points)

                        highX = points[pointsLen - 1][0]
                        highY = points[pointsLen - 1][1]

                        if lowX < 0:
                            self.lowXCorrection = -lowX
                        if lowY < 0:
                            self.lowYCorrection = -lowY

                        self.width = self.lowXCorrection + highX
                        self.height = self.lowYCorrection + highY

                        self.vertices = [[Vertex(x, y) for y in range(self.height)] for x in range(self.width)]
                    if state == 1:
                        self.__addPolygonalObstacle(points)
                    if state == 2:
                        startAndGoals.append(points)

    def filterNeighbors(self, parent):
        if parent.filtered:
            return

        for neighbor in parent.neighbors[:]:
            if self.__filterNeighborLoop(parent, neighbor):
                continue
        parent.filtered = True

    def __addPolygonalObstacle(self, points):
        points.sort()

        for x in range(self.width):
            for y in range(self.height):
                vertex = self.vertices[x][y]
                if not point_inside_polygon(vertex, points):
                    continue
                vertex.filled = True

    def populateNeighbors(self, vertex):
        if vertex.populated:
            return
        vertex.populated = True

        for x in range(DIRECTIONS_START, DIRECTIONS_END):
            for y in range(DIRECTIONS_START, DIRECTIONS_END):
                if x == 0 and y == 0:
                    continue

                translated_x = vertex.x + x
                translated_y = vertex.y + y
                if translated_x >= self.width or translated_x < 0 or translated_y >= self.height or translated_y < 0:
                    continue

                neighbor = self.vertices[translated_x][translated_y]
                # if ((x == 1 and y == 1) or (x == -1 and y == -1) or (x == -1 and y == 1) or (x == 1 and y == -1)) \
                #       and neighbor.filled is True and vertex.filled is True:  # block off filled diagonal neighbors
                #   continue

                if neighbor.filled:
                    continue

                vertex.neighbors.append(neighbor)

    def __filterNeighborLoop(self, parent, neighbor):
        r2 = TURTLE_RADIUS * TURTLE_RADIUS
        y = neighbor.y - TURTLE_RADIUS
        while True:
            if y > neighbor.y + TURTLE_RADIUS:
                break

            x = neighbor.x
            while True:

                if (x - neighbor.x) * (x - neighbor.x) + (y - neighbor.y) * (y - neighbor.y) > r2:
                    break
                if self.__filterNeighborCheck(parent, neighbor, x, y):
                    return True
                x -= 1

            x = neighbor.x + 1
            while True:
                if (x - neighbor.x) * (x - neighbor.x) + (y - neighbor.y) * (y - neighbor.y) > r2:
                    break
                if self.__filterNeighborCheck(parent, neighbor, x, y):
                    return True
                x += 1
            y += 1

        return False

    def __filterNeighborCheck(self, parent, neighbor, x, y):
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False

        vertex = self.vertices[x][y]
        if (not vertex.filled) and TURTLE_RADIUS <= y <= self.height - TURTLE_RADIUS \
                and TURTLE_RADIUS <= x <= self.width - TURTLE_RADIUS:
            return False
        parent.neighbors.remove(neighbor)
        return True


class Path:

    def __init__(self):
        pass

    @abstractmethod
    def findPath(self, start, goal): raise NotImplementedError

    @abstractmethod
    def updateVertex(self, vertex, neighbor, goal): raise NotImplementedError

    def h(self, vertex, goal):  # the estimated path cost from the node we're at to the goal node

        # return               max(abs(vertex.x - goal.x), abs(vertex.y - goal.y))

        return Path.c(goal, vertex) * 1.222222

    @staticmethod
    def c(from_vertex, to_vertex):  # the straight line distance between the s node and e node.
        return math.hypot(from_vertex.x - to_vertex.x, from_vertex.y - to_vertex.y)

    def f(self, vertex, goal):
        vertex.f = vertex.edgeCost + self.h(vertex, goal)


class APath(Path):

    def __init__(self):
        Path.__init__(self)
        self.heap = []
        self.openSet = set()

    def findPath(self, start, goal):
        closed = set()  # type: List[Vertex]

        start.reset()
        goal.reset()

        self.openSet = set()
        self.heap = []
        heapq.heapify(self.heap)

        heapq.heappush(self.heap, start)
        self.openSet.add(start)
        start.f = start.edgeCost + self.h(start, goal)

        while len(self.heap) > 0:
            vertex = heapq.heappop(self.heap)

            if vertex == goal:
                return pathFromGoal(vertex, start)

            closed.add(vertex)

            graph.populateNeighbors(vertex)
            graph.filterNeighbors(vertex)

            for neighbor in vertex.neighbors[:]:
                if neighbor not in closed:
                    if neighbor not in self.openSet:
                        neighbor.edgeCost = sys.maxint
                        neighbor.parent = None
                    self.updateVertex(vertex, neighbor, goal)
        return []

    def updateVertex(self, vertex, neighbor, goal):
        if vertex.edgeCost + Path.c(vertex, neighbor) < neighbor.edgeCost:
            neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
            neighbor.parent = vertex
            if neighbor in self.openSet:
                self.remove(neighbor)
            self.f(neighbor, goal)
            self.add(neighbor)

    def remove(self, vector):
        self.openSet.remove(vector)
        try:
            self.heap.remove(vector)
            heapq.heapify(self.heap)
        except ValueError:
            pass

    def add(self, vector):
        heapq.heappush(self.heap, vector)
        self.openSet.add(vector)
        pass


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
                if neighbor in self.openSet:
                    self.remove(neighbor)
                self.f(neighbor, goal)
                self.add(neighbor)
        else:
            super(FDAPath, self).updateVertex(vertex, neighbor, goal)

    def lineOfSight(self, from_vertex, to_vertex):
        x0 = from_vertex.x
        y0 = from_vertex.y

        x1 = to_vertex.x
        y1 = to_vertex.y

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
                    if self.isRadiusFilled(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                        return False
                    y0 = y0 + sy
                    f = f - dx
                if f != 0 and self.isRadiusFilled(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                    return False
                if dy == 0 and self.isRadiusFilled(x0 + ((sx - 1) / 2), y0) \
                        and self.isRadiusFilled(x0 + ((sx - 1) / 2), y0 - 1):
                    return False
                x0 = x0 + sx
        else:
            while y0 != y1:
                f = f + dx
                if f >= dy:
                    if self.isRadiusFilled(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                        return False
                    x0 = x0 + sx
                    f = f - dy
                if f != 0 and self.isRadiusFilled(x0 + ((sx - 1) / 2), y0 + ((sy - 1) / 2)):
                    return False
                if dx == 0 and self.isRadiusFilled(x0, y0 + ((sy - 1) / 2)) \
                        and self.isRadiusFilled(x0 - 1, y0 + ((sy - 1) / 2)):
                    return False
                y0 = y0 + sy
        return True

    @staticmethod
    def isRadiusFilled(centerX, centerY):
        r2 = 2 * 2
        y = centerY - 2
        while True:
            if y > centerY + 2:
                break

            x = centerX
            while True:
                if (x - centerX) * (x - centerX) + (y - centerY) * (y - centerY) > r2:
                    break
                if graph.vertices[x][y].filled:
                    return True
                x -= 1

            x = centerX + 1
            while True:
                if (x - centerX) * (x - centerX) + (y - centerY) * (y - centerY) > r2:
                    break
                if graph.vertices[x][y].filled:
                    return True
                x += 1
            y += 1
        return False


class TraceFDAPath(FDAPath):

    def h(self, vertex, goal):
        min_offset = min(abs(vertex.x - goal.x), abs(vertex.y - goal.y))
        return math.sqrt(2) * min_offset + max(abs(vertex.x - goal.x), abs(vertex.y - goal.y)) - min_offset


def point_inside_polygon(vertex, poly):
    x = vertex.x
    y = vertex.y

    n = len(poly)
    inside = False
    p2x = 0.0
    p2y = 0.0
    xints = 0.0
    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def pathFromGoal(vertex, start):
    path = []
    while True:
        path.append(vertex)
        if vertex == start:
            break
        vertex = vertex.parent
    return path[::-1]


def print_board(board):
    board.reverse()
    for row in board:
        print " ".join(row)
    print "\n"


graph = Graph("map5.txt")

astar = APath()
trace = TracePath()
fda = FDAPath()
fdaTrace = TraceFDAPath()

for startAndGoal in startAndGoals:
    start = graph.vertices[startAndGoal[0][0]][startAndGoal[0][1]]
    goal = graph.vertices[startAndGoal[1][0]][startAndGoal[1][1]]

    pathVerticies = fda.findPath(start, goal)
    path = map(lambda vertex: (vertex.x, vertex.y), pathVerticies)
    board = []

    print "Start:\t" + start.name()
    print "Goal:\t" + goal.name()

    for row in range(graph.height):
        board.append([])
        for column in range(graph.width):
            vertex = graph.vertices[column][row]
            if vertex.filled:
                board[row].append('x')
            else:
                if start.name() == vertex.name():
                    board[row].append("S")
                else:
                    if goal.name() == vertex.name():
                        board[row].append("E")
                    else:
                        if (vertex.x, vertex.y) in path:
                            board[row].append('/')
                        else:
                            board[row].append('-')

    print_board(board)
