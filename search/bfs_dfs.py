from typing import Dict
from collections import deque
class Search:
    def __init__(self, graph:Dict):
        self.graph = graph

    def bfs(self, start:str, end:str) -> bool:
        visited = set()
        queue = deque([start])

        while queue:
            current_node = queue.popleft()

            if current_node == end:
                return True

            if current_node not in visited:
                visited.add(current_node)
                neighbors = self.graph.get(current_node, [])
                queue.extend(neighbors)

        return False
    def dfs(self, start:str, end:str) -> bool:
        visited = set()
        stack = [start]

        while stack:
            current_node = stack.pop()

            if current_node == end:
                return True

            if current_node not in visited:
                visited.add(current_node)
                neighbors = self.graph.get(current_node, [])
                stack.extend(neighbors)

        return False
    