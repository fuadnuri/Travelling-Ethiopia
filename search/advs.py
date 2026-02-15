class CoffeeAdversarialSearch:
    def __init__(self, graph):
        """
        graph: A dictionary representing the state tree.
        Format: { 'Parent': ['Child1', 'Child2'], ... }
        terminal_values: A dictionary of leaf nodes and their coffee quality scores.
        """
        self.graph = graph
        # Coffee Quality Scores (Utilities) for leaf nodes based on Figure 4
        self.coffee_quality = {
            "Gondar": 12,
            "Metema": 8,
            "Dessie": 10,
            "Samara": 3,
            "Jimma": 15,
            "Bonga": 14,
            "Batu": 5,
            "Assella": 2
        }

    def minimax(self, node, depth, is_maximizing):
        # Base case: if node is a terminal city (leaf)
        if node in self.coffee_quality:
            return self.coffee_quality[node], [node]

        if is_maximizing:
            best_value = float('-inf')
            best_path = []
            for child in self.graph.get(node, []):
                value, path = self.minimax(child, depth + 1, False)
                if value > best_value:
                    best_value = value
                    best_path = [node] + path
            return best_value, best_path

        else:
            best_value = float('inf')
            best_path = []
            for child in self.graph.get(node, []):
                value, path = self.minimax(child, depth + 1, True)
                if value < best_value:
                    best_value = value
                    best_path = [node] + path
            return best_value, best_path

# --- Defining the Search Space from Figure 4 ---
# Level 0: Addis Ababa (MAX)
# Level 1: Ambo, Debre Birhan, Adama (MIN)
# Level 2: Cities leading to Coffee scores (MAX/Terminal)

