import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green
from math import sqrt
from queue import PriorityQueue

pq = PriorityQueue()

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def _markFoundPath(path, edges, edge_id): # green
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = green
    graphUI.updateUI()

def _markVisitedPathNode(graph, path, edges, edge_id): # white
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = white

    for i in path:
        _markQueuedNode(graph, i)

    graphUI.updateUI()

def _markCurrentNode(graph, node): # yellow
    graph[node][3] = yellow
    graphUI.updateUI()

def _markStartNode(graph, node): # orange
    graph[node][3] = orange

def _markGoalNode(graph, node): # purple
    graph[node][3] = purple

def _markVisitedNode(graph, node): # red
    graph[node][3] = red
    graphUI.updateUI()

def _markQueuedNode(graph, node): # blue
    graph[node][3] = blue
    graphUI.updateUI()


def trace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    parent = {}
    visited = [False] * len(graph)
    queue = [start]
    visited[start] = True;

    while queue:
        node = queue.pop(0)
        if node == goal:
            path = trace(parent, start, goal)
            _markStartNode(graph, start)
            _markGoalNode(graph, path[-1])
            _markFoundPath(path, edges, edge_id)
            break

        for adjacent in graph[node][1]:
            if visited[adjacent] == False:
                parent[adjacent] = node
                visited[adjacent] = True
                queue.append(adjacent)

                edges[edge_id(node, adjacent)][1] = white
                graph[node][3] = yellow
                graph[adjacent][3] = red
                graphUI.updateUI()

        if node in parent.values():
            graph[node][3] = blue
            graphUI.updateUI()
            

def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    parent = {}
    stack = [start]
    visited = [False] * len(graph)
    visited[start] = True

    while stack:
        print(parent)

        node = stack.pop()
        if node == goal:
            path = trace(parent, start, goal)
            _markStartNode(graph, start)
            _markGoalNode(graph, path[-1])
            _markFoundPath(path, edges, edge_id)
            break

        for adjacent in graph[node][1]:
            if visited[adjacent] == False:
                parent[adjacent] = node
                visited[adjacent] = True
                stack.append(adjacent)

                edges[edge_id(node, adjacent)][1] = white
                graph[node][3] = yellow
                graph[adjacent][3] = red
                graphUI.updateUI()

        if node in parent.values():
            graph[node][3] = blue
            graphUI.updateUI()
        

def cost(nodeA, nodeB):
    a = nodeA[0] - nodeB[0]
    b = nodeA[1] - nodeB[1]
    return sqrt(a * a + b * b)


def getMinCost(pq):
    minIndex = 0
    for i in range(len(pq)):
        if pq[i][1] < pq[minIndex][1]:
            minIndex = i
    return minIndex


def nodeInPQ(node):
    for i in range(len(pq.queue)):
        if node == pq.queue[i][1]:
            return i
    return -1


def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    parent = {}
    pq.put((0, start))
    visited = [False] * len(graph)
    
    while pq:
        node = pq.get()
        node_key = node[1]
        node_cost = node[0]
        if node_key == goal:
            print(trace(parent, start, goal))
            break

        for adjacent in graph[node_key][1]:
            pq_index = nodeInPQ(adjacent)
            _cost = cost(graph[node_key][0], graph[adjacent][0])

            if visited[adjacent] == False and pq_index == -1:
                parent[adjacent] = node_key
                visited[adjacent] = True
                pq.put((node_cost + _cost, adjacent))
            elif pq_index != -1:
                if _cost < (pq.queue[pq_index][0]):
                    pq.queue.pop(pq_index)
                    pq.put((node_cost + _cost, adjacent))

def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    print("Implement A* algorithm.")
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()

