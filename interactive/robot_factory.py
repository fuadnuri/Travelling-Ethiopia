import pybullet as p
from .robot_config import RobotConfig


class RobotFactory:
    @staticmethod
    def create():
        base_col = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=RobotConfig.BASE_DIMENSIONS
        )
        base_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=RobotConfig.BASE_DIMENSIONS,
            rgbaColor=RobotConfig.BASE_COLOR
        )

        robot_id = p.createMultiBody(
            baseMass=RobotConfig.BASE_MASS,
            baseCollisionShapeIndex=base_col,
            baseVisualShapeIndex=base_vis,
            basePosition=[0, 0, RobotConfig.ROBOT_HEIGHT]
        )

        return robot_id
