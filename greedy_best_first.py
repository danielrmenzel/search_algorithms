
import heapq
from graphviz import Digraph

class Edge:
    def __init__(self, node_from, node_to, cost):
        self.node_from = node_from
        self.node_to = node_to
        self.cost = cost



heuristics = {
    "A": 0,
    "B": 6,
    "C": 4,
    "D": 1,
    "E": 10,
    "G": 0,
    "S": 6,
    

}

# Define edges and heuristics
edges = [
    Edge("S", "A", 2),
    Edge("S", "B", 1),
    Edge("S", "G", 9),
    Edge("A", "C", 2),
    Edge("A", "D", 3),
    Edge("B", "D", 2),
    Edge("B", "E", 4),
    Edge("C", "G", 4),
    Edge("D", "G", 4),
]

start = "S"
goal = "G"



# heuristics = {
#     "A": 8,
#     "B": 6,
#     "C": 5,
#     "D": 4,
#     "E": 2,
#     "F": 3,
#     "G": 0,
#     "S": 6,
# }

# # Define edges and heuristics
# edges = [
#     Edge("S", "A", 6),
#     Edge("S", "B", 2),
#     Edge("S", "C", 1),
#     Edge("A", "D", 3),
#     Edge("A", "G", 20),
#     Edge("B", "D", 2),
#     Edge("B", "E", 6),
#     Edge("C", "B", 3),
#     Edge("C", "E", 6),
#     Edge("C", "F", 4),
#     Edge("D", "F", 5),
#     Edge("E", "G", 2),
#     Edge("F", "G", 3),
# ]

# start = "S"
# goal = "G"

def a_star_search_track_expanded(start, goal, edges, heuristics):
    graph = {}
    for edge in edges:
        if edge.node_from not in graph:
            graph[edge.node_from] = []
        graph[edge.node_from].append((edge.node_to, edge.cost))
    
    frontier = [(heuristics[start], 0, start, [start])]
    visited = {}
    expanded = set()
    children = {}

    while frontier:
        estimated_total_cost, cost_so_far, current_node, path = heapq.heappop(frontier)

        if current_node == goal:
            print(f"Goal reached: {current_node} with total path cost: {cost_so_far}")
            return path, cost_so_far, expanded, children

        visited[current_node] = cost_so_far
        expanded.add(current_node)

        for neighbor, step_cost in graph.get(current_node, []):
            new_cost = cost_so_far + step_cost
            if neighbor not in visited or new_cost < visited[neighbor]:
                estimated_total_cost = new_cost + heuristics[neighbor]
                heapq.heappush(frontier, (estimated_total_cost, new_cost, neighbor, path + [neighbor]))
                if current_node not in children:
                    children[current_node] = []
                children[current_node].append(neighbor)

                # Print the costs for each edge considered
                print(f"Considering edge {current_node} -> {neighbor}:")
                print(f"  g({current_node}) = {cost_so_far}")
                print(f"  Cost({current_node}, {neighbor}) = {step_cost}")
                print(f"  g({neighbor}) = g({current_node}) + Cost({current_node}, {neighbor}) = {new_cost}")
                print(f"  h({neighbor}) = {heuristics[neighbor]}")
                print(f"  f({neighbor}) = g({neighbor}) + h({neighbor}) = {estimated_total_cost}\n")

    return None, 0, expanded, children


def calculate_gn_for_all(edges):
    # Initialize g(n) values dictionary with infinite cost except for the start node
    gn_all = {node: float('inf') for edge in edges for node in [edge.node_from, edge.node_to]}
    gn_all[start] = 0  # Start node g(n) is 0

    # Create a graph representation
    graph = {}
    for edge in edges:
        if edge.node_from not in graph:
            graph[edge.node_from] = []
        graph[edge.node_from].append((edge.node_to, edge.cost))
    
    # Use Dijkstra's algorithm to calculate g(n) for all nodes
    visited = set()
    frontier = [(0, start)]  # Heap queue of (g(n), node)
    
    while frontier:
        current_gn, current_node = heapq.heappop(frontier)
        if current_node in visited:
            continue
        visited.add(current_node)
        
        for neighbor, cost in graph.get(current_node, []):
            new_gn = current_gn + cost
            if new_gn < gn_all[neighbor]:
                gn_all[neighbor] = new_gn
                heapq.heappush(frontier, (new_gn, neighbor))
    
    return gn_all

# Adjust calculate_a_star_costs to use the corrected g(n) values
def calculate_a_star_costs(children, edges, heuristics, gn_all):
    costs = {}
    for parent, child_list in children.items():
        for child in child_list:
            if (parent, child) in [(edge.node_from, edge.node_to) for edge in edges]:
                edge_cost = next((edge.cost for edge in edges if edge.node_from == parent and edge.node_to == child), None)
                if edge_cost is not None:
                    g_child = gn_all[parent] + edge_cost
                    h_child = heuristics[child]
                    f_child = g_child + h_child
                    costs[(parent, child)] = {"g(n)": g_child, "h(n)": h_child, "f(n)": f_child}
    return costs




def greedy_best_first_search_strict(start, goal, edges, heuristics):
    graph = {}
    for edge in edges:
        if edge.node_from not in graph:
            graph[edge.node_from] = []
        graph[edge.node_from].append((edge.node_to, edge.cost))
    
    frontier = [(heuristics[start], start, [start])]
    visited = set()
    expanded = set()  # Track expanded nodes
    siblings = set()  # Track siblings of expanded nodes

    while frontier:
        _, current_node, path = heapq.heappop(frontier)
        expanded.add(current_node)

        if current_node == goal:
            return path, expanded, siblings

        visited.add(current_node)

        for neighbor, _ in graph.get(current_node, []):
            if neighbor not in visited:
                siblings.update([n for n, _ in graph.get(current_node, []) if n != neighbor])
                heapq.heappush(frontier, (heuristics[neighbor], neighbor, path + [neighbor]))

    return None, expanded, siblings






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



def draw_graph_with_path(edges, heuristics, path, expanded_nodes, siblings):
    dot = Digraph()
    path_edges = set(zip(path, path[1:]))

    # Define a function to decide if an edge should be drawn
    def should_draw_edge(edge):
        # Draw the edge if both nodes are either expanded or siblings, with the source being expanded
        return edge.node_from in expanded_nodes and (edge.node_to in expanded_nodes or edge.node_to in siblings)

    for node, heuristic in heuristics.items():
        if node in expanded_nodes or node in siblings:
            label = f'{node}\n(H: {heuristic})'
            color = 'lightblue' if node in path else 'white'
            dot.node(node, label, color='blue', style='filled', fillcolor=color)

    for edge in edges:
        if should_draw_edge(edge):
            color = 'blue' if (edge.node_from, edge.node_to) in path_edges else 'black'
            penwidth = '2.0' if (edge.node_from, edge.node_to) in path_edges else '1.0'
            dot.edge(edge.node_from, edge.node_to, label=str(edge.cost), color=color, penwidth=penwidth)

    return dot





def draw_graph_for_a_star_adjusted(edges, heuristics, path, expanded, children, start, goal):
    dot = Digraph()
    path_edges = set(zip(path, path[1:]))

    # Function to calculate g(n) for each node along the path
    def calculate_gn(path, edges):
        gn = {path[0]: 0}  # Cost from start to itself is 0
        for i in range(1, len(path)):
            for edge in edges:
                if edge.node_from == path[i-1] and edge.node_to == path[i]:
                    gn[path[i]] = gn.get(path[i-1], 0) + edge.cost
                    break
        return gn

    gn = calculate_gn(path, edges)  # Calculate g(n) values for nodes in the path

    for node, heuristic in heuristics.items():
        if node in expanded or node in set().union(*children.values()):  # Show node if it's expanded or a child of an expanded node
            label = f'{node}\n(H: {heuristic})'
            color, fillcolor = 'black', 'white'
            if node == start:
                color, fillcolor = 'green', 'lightgreen'
            elif node == goal:
                color, fillcolor = 'red', 'lightcoral'
            elif node in path:
                color, fillcolor = 'blue', 'lightblue'
            dot.node(node, label, color=color, style='filled', fillcolor=fillcolor)

    for edge in edges:
        if edge.node_from in expanded or edge.node_from in set().union(*children.values()):
            label = str(edge.cost)
            color, penwidth = 'black', '1.0'
            if (edge.node_from, edge.node_to) in path_edges or edge.node_to in children.get(edge.node_from, []):
                g_value = gn.get(edge.node_from, 0)
                h_value = heuristics[edge.node_to]
                total_cost = g_value + edge.cost + h_value
                label = f'{edge.cost} + {h_value} = {total_cost}'
                color, penwidth = 'red', '2.0'
            if edge.node_to in expanded or edge.node_to in set().union(*children.values()):  # Ensure edge leads to a relevant node
                dot.edge(edge.node_from, edge.node_to, label=label, color=color, penwidth=penwidth)

    return dot







# Drawing and saving the graph before the search
dot_before = draw_graph_with_edges_and_heuristics(edges, heuristics, start, goal)
dot_before.render('graph_before_greedy_bfs', view=True)





# Running the search algorithm and capturing expanded nodes and siblings
path, expanded_nodes, siblings = greedy_best_first_search_strict(start, goal, edges, heuristics)

# Drawing and saving the graph after the search, highlighting the path found and showing only expanded nodes, their connections, and siblings
dot_after = draw_graph_with_path(edges, heuristics, path, expanded_nodes, siblings)
dot_after.render('graph_after_greedy_bfs_expanded_and_siblings', view=True)


# Running the A* search
path_a_star, cost_a_star, expanded, children = a_star_search_track_expanded(start, goal, edges, heuristics)

gn_all = calculate_gn_for_all(edges)
a_star_costs = calculate_a_star_costs(children, edges, heuristics, gn_all)
print(a_star_costs)

print(f"Path found by A*: {path_a_star}")
print(f"Total cost to reach the goal: {cost_a_star}")
print(f"Nodes expanded during the search: {expanded}")
print(f"Children of expanded nodes: {children}")

dot = draw_graph_for_a_star_adjusted(edges, heuristics, path_a_star, expanded, children, start, goal)


dot.render('graph_after_a_star_adjustedtree.gv', view=True)
