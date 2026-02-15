import pybullet as p
import pybullet_data
from data.graph5 import city_coordinates
from .robot_config import RobotConfig


class WorldManager:
    @staticmethod
    def setup():
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

    @staticmethod
    def visualize_cities():
        for city, (x, y) in city_coordinates.items():
            visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=RobotConfig.CITY_SPHERE_RADIUS,
                rgbaColor=RobotConfig.CITY_SPHERE_COLOR
            )
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual,
                basePosition=[x, y, RobotConfig.CITY_HEIGHT]
            )

            p.addUserDebugText(
                city,
                [x, y, RobotConfig.CITY_HEIGHT + 0.2],
                textColorRGB=RobotConfig.TEXT_COLOR,
                textSize=RobotConfig.TEXT_SIZE
            )
