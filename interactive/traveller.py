from interactive.bfs_planner import BFSPlanner
from interactive.robot_controller import RobotController


class Traveler:
    def __init__(self, graph, robot_id):
        self.planner = BFSPlanner(graph)
        self.controller = RobotController(robot_id)

    def navigate(self, start, goal):
        path = self.planner.search(start, goal)
        print("Path:", path)

        for city in path:
            self.controller.drive_to_city(city)
