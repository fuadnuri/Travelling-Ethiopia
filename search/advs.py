class CoffeeAdversarialSearch:
    def __init__(self, graph, coffee_quality):
        """
        graph:          dict { 'Parent': ['Child1', 'Child2'], ... }
        coffee_quality: dict { 'CityName': score, ... } for leaf/terminal nodes
        """
        self.graph = graph
        self.coffee_quality = coffee_quality

    def minimax(self, node, depth, is_maximizing):
        # Base case: leaf node with a known coffee quality score
        if node in self.coffee_quality:
            return self.coffee_quality[node], [node]

        children = self.graph.get(node, [])

        # Guard: non-terminal node with no children (broken tree)
        if not children:
            raise ValueError(
                f"Node '{node}' has no children and is not a terminal node. "
                f"Check your graph definition."
            )

        if is_maximizing:
            best_value = float('-inf')
            best_path  = []
            for child in children:
                value, path = self.minimax(child, depth + 1, False)
                if value > best_value:
                    best_value = value
                    best_path  = path
            return best_value, [node] + best_path

        else:
            best_value = float('inf')
            best_path  = []
            for child in children:
                value, path = self.minimax(child, depth + 1, True)
                if value < best_value:
                    best_value = value
                    best_path  = path
            return best_value, [node] + best_path
