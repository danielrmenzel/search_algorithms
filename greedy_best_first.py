
import heapq
from graphviz import Digraph

class Edge:
    def __init__(self, node_from, node_to, cost):
        self.node_from = node_from
        self.node_to = node_to
        self.cost = cost


heuristics = {
    "A": 8,
    "B": 6,
    "C": 5,
    "D": 4,
    "E": 2,
    "F": 3,
    "G": 0,
    "S": 6,
}

# Define edges and heuristics
edges = [
    Edge("S", "A", 6),
    Edge("S", "B", 2),
    Edge("S", "C", 1),
    Edge("A", "D", 3),
    Edge("A", "G", 20),
    Edge("B", "D", 2),
    Edge("B", "E", 6),
    Edge("C", "B", 3),
    Edge("C", "E", 6),
    Edge("C", "F", 4),
    Edge("D", "F", 5),
    Edge("E", "G", 2),
    Edge("F", "G", 3),
]

start = "S"
goal = "G"


def a_star_search(start, goal, edges, heuristics):
    graph = {}
    for edge in edges:
        if edge.node_from not in graph:
            graph[edge.node_from] = []
        graph[edge.node_from].append((edge.node_to, edge.cost))
    
    frontier = [(heuristics[start], 0, start, [start])]  # (Estimated total cost, Cost so far, Current node, Path)
    visited = {}

    while frontier:
        estimated_total_cost, cost_so_far, current_node, path = heapq.heappop(frontier)

        if current_node in visited and visited[current_node] <= cost_so_far:
            continue

        if current_node == goal:
            return path

        visited[current_node] = cost_so_far

        for neighbor, step_cost in graph.get(current_node, []):
            new_cost = cost_so_far + step_cost
            if neighbor not in visited or new_cost < visited[neighbor]:
                estimated_total_cost = new_cost + heuristics[neighbor]
                heapq.heappush(frontier, (estimated_total_cost, new_cost, neighbor, path + [neighbor]))

    return None

def greedy_best_first_search_strict(start, goal, edges, heuristics):
    graph = {}
    for edge in edges:
        if edge.node_from not in graph:
            graph[edge.node_from] = []
        graph[edge.node_from].append((edge.node_to, edge.cost))
    
    frontier = [(heuristics[start], start, [start])]
    visited = set()

    while frontier:
        _, current_node, path = heapq.heappop(frontier)

        if current_node == goal:
            return path

        visited.add(current_node)

        for neighbor, _ in graph.get(current_node, []):
            if neighbor not in visited:
                heapq.heappush(frontier, (heuristics[neighbor], neighbor, path + [neighbor]))

    return None

def draw_graph_with_edges_and_heuristics(edges, heuristics, start, goal):
    dot = Digraph()

    for node, heuristic in heuristics.items():
        label = f'{node}\n(H: {heuristic})'
        if node == start:
            dot.node(node, label, color='green', style='filled', fillcolor='lightgreen')
        elif node == goal:
            dot.node(node, label, color='red', style='filled', fillcolor='lightcoral')
        else:
            dot.node(node, label)

    for edge in edges:
        dot.edge(edge.node_from, edge.node_to, label=str(edge.cost))

    return dot

def draw_graph_with_path(edges, heuristics, path):
    dot = Digraph()
    path_edges = set(zip(path, path[1:]))

    for node, heuristic in heuristics.items():
        label = f'{node}\n(H: {heuristic})'
        if node in path:
            dot.node(node, label, color='blue', style='filled', fillcolor='lightblue')
        else:
            dot.node(node, label)

    for edge in edges:
        if (edge.node_from, edge.node_to) in path_edges:
            dot.edge(edge.node_from, edge.node_to, label=str(edge.cost), color='blue', penwidth='2.0')
        else:
            dot.edge(edge.node_from, edge.node_to, label=str(edge.cost))

    return dot



# Drawing and saving the graph before the search
dot_before = draw_graph_with_edges_and_heuristics(edges, heuristics, start, goal)
dot_before.render('graph_before_greedy_bfs', format='pdf', cleanup=True)

# Running the search algorithm
path = greedy_best_first_search_strict(start, goal, edges, heuristics)

# Drawing and saving the graph after the search, highlighting the path found
dot_after = draw_graph_with_path(edges, heuristics, path)
dot_after.render('graph_after_greedy_bfs', format='pdf', cleanup=True)



# For A* Search
path_a_star = a_star_search(start, goal, edges, heuristics)

# Drawing and saving the graph after the A* search, highlighting the path found
dot_after_a_star = draw_graph_with_path(edges, heuristics, path_a_star)
dot_after_a_star.render('graph_after_a_star', format='pdf', cleanup=True)


