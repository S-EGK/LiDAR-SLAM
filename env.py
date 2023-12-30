import math
import pygame

class BuildEnv:
    def __init__(self, map_dims):
        pygame.init()
        self.point_clouds = []
        self.external_map = pygame.image.load("map.png")
        self.maph, self.mapw = map_dims
        self.map_window_name = "Indoor ENV"
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.external_map, (0, 0))
        # Colors
        self.black = (0, 0, 0)
        self.grey = (128, 128, 128)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.white = (255, 255, 255)

    def ad2pos(self, distance, angle, robot_position):
        x = robot_position[0] + distance * math.cos(angle)
        y = robot_position[1] - distance * math.sin(angle)
        return (int(x), int(y))
    
    def data_storage(self, data):
        # print(len(self.point_clouds))
        for element in data:
            point = self.ad2pos(element[0], element[1], element[2])
            if point not in self.point_clouds:
                self.point_clouds.append(point)

    def show_sensor_data(self):
        self.info_map = self.map.copy()
        for point in self.point_clouds:
            self.info_map.set_at((int(point[0]), int(point[1])), self.red)