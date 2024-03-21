from graphviz import Digraph

def alpha_beta_value(node, tree_structure, values, alpha, beta, visited, depth=0):
    # Markiere den Knoten als besucht
    visited[node] = True

    # Basisfall: Wenn der Knoten ein Blattknoten ist, gib seinen Wert zurück.
    if node not in tree_structure or not tree_structure[node]:
        return int(values[node])

    if depth % 2 == 0:  # Max-Ebene
        value = float('-inf')
        for child in tree_structure[node]:
            value = max(value, alpha_beta_value(child, tree_structure, values, alpha, beta, visited, depth + 1))
            alpha = max(alpha, value)
            if beta <= alpha:
                break  # Beta-Cut
        return value
    else:  # Min-Ebene
        value = float('inf')
        for child in tree_structure[node]:
            value = min(value, alpha_beta_value(child, tree_structure, values, alpha, beta, visited, depth + 1))
            beta = min(beta, value)
            if beta <= alpha:
                break  # Alpha-Cut
        return value

def visualize_tree_with_alpha_beta(tree_structure, values, visited):
    dot = Digraph(comment='Alpha-Beta Pruning Tree')

    for node, value in values.items():
        if node in visited:
            dot.node(node, f'{node}\n{value}')
        else:
            dot.node(node, f'{node}\n{value}', color='red', style='filled', fillcolor='lightpink')

    for node, edges in tree_structure.items():
        for edge in edges:
            dot.edge(node, edge)

    dot.render('output_alpha_beta_tree.gv', view=True)

# Definition der Baumstruktur
tree_structure = {
    #Wurzel
    'a': ['b', 'c'],

    #Tiefe1
    'b': ['b8', 'b6', 'b7', 'b5'],
    'c': ['d', 'e', 'f', 'g'],
    #Tiefe2
    'd': ['d9', 'd2'],
    'e': ['e8', 'e10', 'e2'],
    'f': ['f3', 'f2', 'f4'],
    'g': ['g0', 'g5', 'g6'],
    
    
}
# Definition der Knotenwerte
values = {
    'b8': '8', 'b6': '6', 'b7': '7', 'b5': '5', 'd9': '9'
    ,'d2': '2', 'e8': '8', 'e10': '10', 'e2': '2', 'f3': '3','f2': '2','f4': '4','g0': '0','g5': '5','g6': '6'    
}

# Alpha- und Beta-Werte initialisieren
alpha = float('-inf')
beta = float('inf')

# Besuchte Knoten tracken
visited = {}

# Alpha-Beta Pruning durchführen
alpha_beta_value('a', tree_structure, values, alpha, beta, visited)

# Visualisierung mit markierten nicht besuchten Knoten
visualize_tree_with_alpha_beta(tree_structure, values, visited)

# Ergebnis ausgeben
print("Besuchte Knoten:", visited)