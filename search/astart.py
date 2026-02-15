import heapq
from typing import Dict, List

class AStarSearch:
    def __init__(self, graph:Dict[str, List[tuple[str, int]]], heuristics):
        self.graph = graph
        self.heuristics = heuristics

    def search(self, start, goal):
        # Priority queue stores: (f_cost, current_node, path, g_cost)
        # f(n) = g(n) + h(n)
        frontier = []
        heapq.heappush(frontier, (self.heuristics[start], start, [start], 0))
        
        visited = {} # Stores the lowest g_cost to reach a city

        while frontier:
            f_cost, current_node, path, g_cost = heapq.heappop(frontier)

            if current_node == goal:
                return path, g_cost

            if current_node in visited and visited[current_node] <= g_cost:
                continue
            
            visited[current_node] = g_cost

            for neighbor, weight in self.graph.get(current_node, []):
                new_g_cost = g_cost + weight
                # Calculate f(n) = g(n) + h(n)
                new_f_cost = new_g_cost + self.heuristics.get(neighbor, float('inf'))
                
                new_path = list(path)
                new_path.append(neighbor)
                
                heapq.heappush(frontier, (new_f_cost, neighbor, new_path, new_g_cost))

        return None, float('inf')
    