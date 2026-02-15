import heapq


class UCS:
    def __init__(self, graph):
        self.graph = graph

    def search(self, start: str, goal: str):
        """
        Standard Uniform Cost Search (single goal)
        """
        pq = [(0, start, [start])]
        visited = set()

        while pq:
            cost, node, path = heapq.heappop(pq)

            if node == goal:
                return cost, path

            if node not in visited:
                visited.add(node)
                for neighbor, edge_cost in self.graph.get(node, []):
                    heapq.heappush(
                        pq,
                        (cost + edge_cost, neighbor, path + [neighbor])
                    )

        return float("inf"), []

    def multi_goal_search(self, start:str, goals:str):
        """
        Customized UCS:
        - Always go to the nearest unvisited goal
        - Preserves local optimality
        """
        current:str = start
        remaining_goals = set(goals)

        total_cost = 0
        full_path = [start]

        while remaining_goals:
            best_goal = ""
            best_cost = float("inf")
            best_path = []

            # Run UCS to each remaining goal
            for goal in remaining_goals:
                cost, path = self.search(current, goal)
                if cost < best_cost:
                    best_cost = cost
                    best_goal = goal
                    best_path = path

            # Move to the nearest goal
            total_cost += best_cost
            full_path.extend(best_path[1:])  # avoid duplicate node
            current = best_goal
            remaining_goals.remove(best_goal)

        return total_cost, full_path
