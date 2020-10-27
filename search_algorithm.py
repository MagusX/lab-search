import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green
from math import sqrt
from queue import PriorityQueue

INF = 100000
pq = PriorityQueue()

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def _markResult(graph, path, edges, edge_id, start, goal): # green
    graph[start][3] = orange
    graph[goal][3] = purple

    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = green
    graphUI.updateUI()


def _markVisited(graph, edges, edge_id, curNode, adjacent):
    edges[edge_id(curNode, adjacent)][1] = white
    graph[curNode][3] = yellow
    graph[adjacent][3] = red
    graphUI.updateUI()


def _markParent(graph, parent, node):
    if node in parent.values():
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
            _markResult(graph, path, edges, edge_id, start, goal)
            break

        for adjacent in graph[node][1]:
            if visited[adjacent] == False:
                parent[adjacent] = node
                visited[adjacent] = True
                queue.append(adjacent)
                _markVisited(graph, edges, edge_id, node, adjacent)

        _markParent(graph, parent, node)
            

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
        node = stack.pop()
        if node == goal:
            path = trace(parent, start, goal)
            _markResult(graph, path, edges, edge_id, start, goal)
            break

        for adjacent in graph[node][1]:
            if visited[adjacent] == False:
                parent[adjacent] = node
                visited[adjacent] = True
                stack.append(adjacent)
                _markVisited(graph, edges, edge_id, node, adjacent)

        _markParent(graph, parent, node)
        

def cost(nodeA, nodeB):
    a = nodeA[0] - nodeB[0]
    b = nodeA[1] - nodeB[1]
    return sqrt(a * a + b * b)


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
            path = trace(parent, start, goal)
            _markResult(graph, path, edges, edge_id, start, goal)
            break

        for adjacent in graph[node_key][1]:
            pq_index = nodeInPQ(adjacent)
            _cost = cost(graph[node_key][0], graph[adjacent][0])

            if visited[adjacent] == False and pq_index == -1:
                parent[adjacent] = node_key
                visited[adjacent] = True
                pq.put((node_cost + _cost, adjacent))
                _markVisited(graph, edges, edge_id, node_key, adjacent)
            elif pq_index != -1:
                if _cost < pq.queue[pq_index][0]:
                    pq.queue.pop(pq_index)
                    pq.put((node_cost + _cost, adjacent))

        _markParent(graph, parent, node_key)


def a_star_h(graph, cur, goal):
    return cost(graph[cur][0], graph[goal][0])


def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    parent = {}
    gScore = [INF] * len(graph)
    fScore = [INF] * len(graph)

    gScore[start] = 0
    fScore[start] = a_star_h(graph, start, goal)
    pq.put((fScore[start], start))
    
    while pq:
        node = pq.get()
        node_key = node[1]
        node_cost = node[0]
        if node_key == goal:
            path = trace(parent, start, goal)
            _markResult(graph, path, edges, edge_id, start, goal)
            break

        for adjacent in graph[node_key][1]:
            tentative_gScore = gScore[node_key] + cost(graph[node_key][0], graph[adjacent][0])
            if tentative_gScore < gScore[adjacent]:
                parent[adjacent] = node_key
                gScore[adjacent] = tentative_gScore
                fScore[adjacent] = gScore[adjacent] + a_star_h(graph, adjacent, goal)
                _markVisited(graph, edges, edge_id, node_key, adjacent)
                if nodeInPQ(adjacent) == -1:
                    pq.put((fScore[adjacent], adjacent))

        _markParent(graph, parent, node_key)


def DijkstraSearch(graph, edges, edge_id, start, goal):
    """
    Dijkstra search
    """
    # TODO: your code
    parent = {}
    fScore = [INF] * len(graph)
    fScore[start] = 0
    pq.put((fScore[start], start))
    
    while pq:
        node = pq.get()
        node_key = node[1]
        node_cost = node[0]
        if node_key == goal:
            path = trace(parent, start, goal)
            _markResult(graph, path, edges, edge_id, start, goal)
            break

        for adjacent in graph[node_key][1]:
            tentative = fScore[node_key] + cost(graph[node_key][0], graph[adjacent][0])
            if tentative < fScore[adjacent]:
                parent[adjacent] = node_key
                fScore[adjacent] = tentative
                _markVisited(graph, edges, edge_id, node_key, adjacent)
                if nodeInPQ(adjacent) == -1:
                    pq.put((fScore[adjacent], adjacent))

        _markParent(graph, parent, node_key)


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

