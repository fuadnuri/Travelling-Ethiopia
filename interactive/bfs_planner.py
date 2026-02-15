from collections import deque


class BFSPlanner:
    def __init__(self, graph):
        self.graph = graph

    def search(self, start, goal):
        queue = deque([(start, [start])])
        visited = {start}

        while queue:
            node, path = queue.popleft()
            if node == goal:
                return path

            for neighbor in self.graph.get(node, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return None
