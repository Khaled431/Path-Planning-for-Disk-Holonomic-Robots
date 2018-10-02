import sys
from _ast import List
from abc import abstractmethod
import math
import bisect
import threading

DIRECTIONS_START = -1
DIRECTIONS_END = 2

GRID_EXPANSION = 10
TURTLE_RADIUS = int(0.5 * GRID_EXPANSION)

startAndGoals = []


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

    def __cmp__(self, other):
        return cmp((other.f, other.f - sys.maxunicode * other.edgeCost),
                   (self.f, self.f - sys.maxunicode * self.edgeCost))


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
            pass

        for neighbor in parent.neighbors[:]:
            if self.__filterNeighborLoop(parent, neighbor):
                continue
        parent.filtered = True
        pass

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

                if (x - neighbor.x) * (x - neighbor.x) + (y - neighbor.y) * (y - neighbor.y) >= r2:
                    break
                if self.__filterNeighborCheck(parent, neighbor, x, y):
                    return True
                x -= 1

            x = neighbor.x + 1
            while True:
                if (x - neighbor.x) * (x - neighbor.x) + (y - neighbor.y) * (y - neighbor.y) >= r2:
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

        return Path.c(vertex, goal)

    @staticmethod
    def c(from_vertex, to_vertex):  # the straight line distance between the s node and e node.
        return math.sqrt((from_vertex.x - to_vertex.x) ** 2 + (from_vertex.y - to_vertex.y) ** 2)

    def f(self, vertex, goal):
        vertex.f = vertex.edgeCost + self.h(vertex, goal)


class APath(Path):

    def __init__(self):
        Path.__init__(self)
        self.heap = []

    def findPath(self, start, goal):
        closed = []  # type: List[Vertex]

        start.reset()
        goal.reset()

        self.heap = []
        self.heap.append(start)
        start.f = start.edgeCost + self.h(start, goal)

        while len(self.heap) > 0:
            vertex = self.heap.pop()
            if vertex == goal:
                return pathFromGoal(vertex)

            closed.append(vertex)

            graph.populateNeighbors(vertex)

            thread1 = threading.Thread(target=graph.filterNeighbors, args=(vertex,))
            thread1.start()

            thread1.join()

            for neighbor in vertex.neighbors[:]:
                if neighbor not in closed:
                    if neighbor not in self.heap:
                        neighbor.edgeCost = sys.maxint
                        neighbor.parent = None
                    self.updateVertex(vertex, neighbor, goal)
        return []

    def updateVertex(self, vertex, neighbor, goal):
        if vertex.edgeCost + Path.c(vertex, neighbor) >= neighbor.edgeCost:
            pass
        neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
        neighbor.parent = vertex
        if neighbor in self.heap:
            self.heap.remove(neighbor)
        self.f(neighbor, goal)
        self.add(bisect.bisect_left(self.heap, neighbor), neighbor)

    def add(self, index, vector):
        self.heap.insert(index, vector)
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
                if neighbor in self.heap:
                    self.heap.remove(neighbor)
                self.f(neighbor, goal)
                self.add(bisect.bisect_left(self.heap, neighbor), neighbor)
        else:
            if vertex.edgeCost + Path.c(vertex, neighbor) < neighbor.edgeCost:
                neighbor.edgeCost = vertex.edgeCost + Path.c(vertex, neighbor)
                neighbor.parent = vertex
                if neighbor in self.heap:
                    self.heap.remove(neighbor)
                self.f(neighbor, goal)
                self.add(bisect.bisect_left(self.heap, neighbor), neighbor)

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


def point_inside_polygon(vertex, poly, include_edges=True):
    x = vertex.x
    y = vertex.y

    n = len(poly)
    inside = False

    p1x, p1y = poly[0]
    for i in range(1, n + 1):
        p2x, p2y = poly[i % n]
        if p1y == p2y:
            if y == p1y:
                if min(p1x, p2x) <= x <= max(p1x, p2x):
                    # point is on horisontal edge
                    inside = include_edges
                    break
                elif x < min(p1x, p2x):  # point is to the left from current edge
                    inside = not inside
        else:  # p1y!= p2y
            if min(p1y, p2y) <= y <= max(p1y, p2y):
                xinters = (y - p1y) * (p2x - p1x) / float(p2y - p1y) + p1x

                if x == xinters:  # point is right on the edge
                    inside = include_edges
                    break

                if x < xinters:  # point is to the left from current edge
                    inside = not inside

        p1x, p1y = p2x, p2y

    return inside


def pathFromGoal(vertex):
    path = []
    while True:
        path.append(vertex)
        if vertex.parent is vertex:
            break
        vertex = vertex.parent
    return path[::-1]


def print_board(board):
    board.reverse()
    for row in board:
        print " ".join(row)


graph = Graph("map1.txt")

astar = APath()
trace = TracePath()
fda = FDAPath()
fdaTrace = TraceFDAPath()

for startAndGoal in startAndGoals:
    start = graph.vertices[startAndGoal[0][0]][startAndGoal[0][1]]
    goal = graph.vertices[startAndGoal[1][0]][startAndGoal[1][1]]

    path = astar.findPath(start, goal)
    board = []

    for row in range(graph.height):
        board.append([])
        for column in range(graph.width):
            vertex = graph.vertices[column][row]
            if vertex.filled:
                board[row].append('x')
            else:
                if start.name() == vertex.name():
                    board[row].append("s")
                else:
                    if goal.name() == vertex.name():
                        board[row].append("e")
                    else:
                        if vertex in path:
                            board[row].append('.')
                        else:
                            board[row].append('-')

    print_board(board)
