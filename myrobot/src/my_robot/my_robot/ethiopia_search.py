#!/usr/bin/env python3
"""
Travelling Ethiopia — Uninformed Search (BFS) ROS 2 Node
=========================================================
Task 5.3: Uses Breadth-First Search (BFS) to find a path from any
initial state to a given goal state based on the relaxed state space
graph in Figure 5.

Usage (after building and sourcing):
  ros2 run my_robot ethiopia_search --ros-args \
    -p start:="Addis Ababa" -p goal:="Moyale"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque


# ─────────────────────────────────────────────────────────────
# Figure 5: Relaxed State Space Graph — Travelling Ethiopia
# ─────────────────────────────────────────────────────────────
ADJACENCY = {
    "Addis Ababa":   ["Ambo", "Adama", "Debre Birhan"],
    "Ambo":          ["Addis Ababa", "Nekemte", "Wolkite"],
    "Adama":         ["Addis Ababa", "Batu", "Assella", "Matahara"],
    "Debre Birhan":  ["Addis Ababa"],
    "Nekemte":       ["Ambo", "Gimbi", "Bedelle"],
    "Gimbi":         ["Nekemte", "Dembi Dollo"],
    "Dembi Dollo":   ["Gimbi", "Gambella"],
    "Gambella":      ["Dembi Dollo", "Gore"],
    "Gore":          ["Gambella", "Tepi", "Bedelle"],
    "Bedelle":       ["Gore", "Nekemte", "Jimma"],
    "Jimma":         ["Bedelle", "Wolkite", "Bonga"],
    "Wolkite":       ["Ambo", "Jimma", "Worabe"],
    "Buta Jirra":    ["Batu", "Worabe"],
    "Batu":          ["Adama", "Buta Jirra", "Shashemene"],
    "Assella":       ["Adama", "Assasa"],
    "Matahara":      ["Adama", "Awash"],
    "Awash":         ["Matahara", "Chiro"],
    "Chiro":         ["Awash", "Dire Dawa"],
    "Dire Dawa":     ["Chiro", "Harar"],
    "Harar":         ["Dire Dawa", "Babile"],
    "Babile":        ["Harar", "Jigjiga"],
    "Jigjiga":       ["Babile", "Dega Habur"],
    "Dega Habur":    ["Jigjiga", "Kebri Dehar"],
    "Kebri Dehar":   ["Dega Habur", "Gode", "Werder"],
    "Werder":        ["Kebri Dehar"],
    "Gode":          ["Kebri Dehar"],
    "Tepi":          ["Gore", "Mezan Teferi", "Bonga"],
    "Bonga":         ["Tepi", "Jimma", "Dawro"],
    "Mezan Teferi":  ["Tepi"],
    "Dawro":         ["Bonga", "Wolaita Sodo"],
    "Wolaita Sodo":  ["Dawro", "Hossana", "Arba Minch"],
    "Hossana":       ["Wolaita Sodo", "Worabe"],
    "Worabe":        ["Hossana", "Wolkite", "Buta Jirra"],
    "Arba Minch":    ["Wolaita Sodo", "Konso"],
    "Konso":         ["Arba Minch", "Yabello"],
    "Yabello":       ["Konso", "Moyale", "Bule Hora"],
    "Bule Hora":     ["Yabello", "Dilla"],
    "Dilla":         ["Bule Hora", "Hawassa"],
    "Hawassa":       ["Dilla", "Shashemene"],
    "Shashemene":    ["Hawassa", "Batu", "Dodolla"],
    "Dodolla":       ["Shashemene", "Assasa", "Bale"],
    "Assasa":        ["Dodolla", "Assella"],
    "Bale":          ["Dodolla", "Goba", "Sof Oumer", "Liben"],
    "Goba":          ["Bale", "Sof Oumer", "Dega Habur"],
    "Sof Oumer":     ["Goba", "Bale", "Gode"],
    "Liben":         ["Bale"],
    "Moyale":        ["Yabello"],
}

# Cartesian coordinates (X, Y) from Figure 5
COORDINATES = {
    "Addis Ababa":  (0,    0),
    "Adama":        (1,   -1),
    "Ambo":         (-1,   0),
    "Debre Birhan": (0.5,  1),
    "Nekemte":      (-2,   0.5),
    "Jimma":        (-1.5,-1.5),
    "Wolkite":      (-0.8,-0.8),
    "Buta Jirra":   (0.2, -1.2),
    "Batu":         (0.8, -1.5),
    "Shashemene":   (1,   -2.5),
    "Hawassa":      (1,   -3.5),
    "Dilla":        (1,   -4.5),
    "Moyale":       (1,   -7),
}


# ─────────────────────────────────────────────────────────────
# BFS Implementation
# ─────────────────────────────────────────────────────────────
def bfs(graph: dict, start: str, goal: str):
    """
    Breadth-First Search — guarantees shortest path (fewest edges).

    Returns:
        path  (list of city names) if found, else None
        nodes_explored (int)
    """
    if start not in graph:
        return None, 0
    if goal not in graph:
        return None, 0
    if start == goal:
        return [start], 1

    visited = {start}
    queue = deque([[start]])          # each item is a path
    nodes_explored = 1

    while queue:
        path = queue.popleft()
        current = path[-1]

        for neighbour in graph.get(current, []):
            if neighbour == goal:
                return path + [neighbour], nodes_explored
            if neighbour not in visited:
                visited.add(neighbour)
                nodes_explored += 1
                queue.append(path + [neighbour])

    return None, nodes_explored       # no path exists


# ─────────────────────────────────────────────────────────────
# ROS 2 Node
# ─────────────────────────────────────────────────────────────
class EthiopiaSearchNode(Node):
    """
    ROS 2 node that performs BFS on the Ethiopia state-space graph.

    Parameters
    ----------
    start : str  — initial city  (default: "Addis Ababa")
    goal  : str  — goal city     (default: "Moyale")

    Publishes
    ---------
    /search_path   (std_msgs/String)  — the found path as a string
    /search_status (std_msgs/String)  — SUCCESS or FAILURE
    """

    def __init__(self):
        super().__init__('ethiopia_search_node')

        # ── Parameters ──
        self.declare_parameter('start', 'Addis Ababa')
        self.declare_parameter('goal',  'Moyale')

        self.start = self.get_parameter('start').get_parameter_value().string_value
        self.goal  = self.get_parameter('goal').get_parameter_value().string_value

        # ── Publishers ──
        self.path_pub   = self.create_publisher(String, 'search_path',   10)
        self.status_pub = self.create_publisher(String, 'search_status', 10)

        # ── Run once on startup ──
        self.timer = self.create_timer(0.5, self.run_search)
        self._done = False

        self.get_logger().info(
            f'Ethiopia Search Node started | '
            f'Strategy: BFS | Start: {self.start} | Goal: {self.goal}'
        )

    def run_search(self):
        if self._done:
            return
        self._done = True

        self.get_logger().info('─' * 55)
        self.get_logger().info(f'  Searching: {self.start}  →  {self.goal}')
        self.get_logger().info('─' * 55)

        path, explored = bfs(ADJACENCY, self.start, self.goal)

        status_msg = String()
        path_msg   = String()

        if path:
            # Format path with step numbers
            steps = '\n'.join(
                f'  Step {i:>2}: {city}' +
                (f'  (x={COORDINATES[city][0]}, y={COORDINATES[city][1]})'
                 if city in COORDINATES else '')
                for i, city in enumerate(path)
            )
            summary = (
                f'\n✅  Path FOUND\n'
                f'  Nodes explored : {explored}\n'
                f'  Path length    : {len(path) - 1} steps\n'
                f'\n{steps}\n'
                f'\n  Path string: {" → ".join(path)}'
            )
            self.get_logger().info(summary)

            path_msg.data   = ' -> '.join(path)
            status_msg.data = f'SUCCESS | {len(path)-1} steps | {" -> ".join(path)}'
        else:
            msg = f'❌  No path found from "{self.start}" to "{self.goal}"'
            self.get_logger().warn(msg)
            path_msg.data   = ''
            status_msg.data = f'FAILURE | No path from {self.start} to {self.goal}'

        self.path_pub.publish(path_msg)
        self.status_pub.publish(status_msg)
        self.get_logger().info('─' * 55)


# ─────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = EthiopiaSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
