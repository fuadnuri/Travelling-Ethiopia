import pybullet as p
import numpy as np
import time
from data.graph5 import city_coordinates


class RobotController:
    def __init__(self, robot_id):
        self.robot_id = robot_id

    def get_gyro(self):
        _, ang_vel = p.getBaseVelocity(self.robot_id)
        return ang_vel

    def drive_to_city(self, city):
        target = city_coordinates[city]

        while p.isConnected():
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            dx, dy = target[0] - pos[0], target[1] - pos[1]
            dist = np.sqrt(dx * dx + dy * dy)

            if dist < 0.15:
                break

            p.resetBasePositionAndOrientation(
                self.robot_id,
                [pos[0] + dx * 0.02, pos[1] + dy * 0.02, pos[2]],
                [0, 0, 0, 1]
            )

            p.stepSimulation()
            time.sleep(1./240.)
