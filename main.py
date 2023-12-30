import pygame
import random

import env
import sensors
import features

def random_color():
    levels = range(32,256,32)
    return tuple(random.choice(levels) for _ in range(3))

feature_map = features.FeaturesDetection()
env = env.BuildEnv((600, 1200))
original_map = env.map.copy()
laser = sensors.LaserSensor(200, original_map, (0.5,0.01))
env.map.fill((env.white))
env.info_map = env.map.copy()
original_map = env.map.copy()
running = True
feature_detection = True
break_point_ind = 0

while running:
    env.info_map = original_map.copy()
    feature_detection = True
    break_point_ind = 0
    end_points = [0,0]
    sensor_on = False
    predicyed_points_to_draw = []
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if pygame.mouse.get_focused():
        sensor_on = True
    elif not pygame.mouse.get_focused():
        sensor_on = False
    if sensor_on:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        feature_map.laser_points_set(sensor_data)
        while break_point_ind < (feature_map.np - feature_map.pmin):
            seed_seg = feature_map.seed_segment_detection(laser.position, break_point_ind)
            if seed_seg == False:
                break
            else:
                seed_segment = seed_seg[0]
                predicyed_points_to_draw = seed_seg[1]
                indices = seed_seg[2]
                results = feature_map.seed_segment_growing(indices, break_point_ind)
                if results == False:
                    break_point_ind = indices[1]
                    continue
                else:
                    line_eq = results[1]
                    m, c = results[5]
                    line_seg = results[0]
                    outer_most = results[2]
                    break_point_ind = results[3]

                    end_points[0] = feature_map.projection_point2line(outer_most[0], m , c)
                    end_points[1] = feature_map.projection_point2line(outer_most[1], m , c)
                    feature_map.features.append([[m,c], end_points])
                    pygame.draw.line(env.info_map, (0,255,0), end_points[0], end_points[1], 1)
                    env.data_storage(sensor_data)

                    feature_map.features = feature_map.linefeats2point()
                    features.lankmark_association(feature_map.features)
        for landmark in features.Landmarks:
            pygame.draw.line(env.info_map, (0,0,255), landmark[1][0], landmark[1][1], 1)

    env.map.blit(env.info_map, (0, 0))
    pygame.display.update()

"""
                    color = random_color()
                    for point in line_seg:
                        env.info_map.set_at((int(point[0][0]), int(point[0][1])), (0,255,0))
                        pygame.draw.circle(env.info_map, color, (int(point[0][0]), int(point[0][1])), 2, 0)
                    pygame.draw.line(env.info_map, (255,0,0), end_points[0], end_points[1], 2)

                    env.data_storage(sensor_data)
    env.map.blit(env.info_map, (0, 0))
    pygame.display.update()
"""




"""
env = env.BuildEnv((600, 1200))
env.original_map = env.map.copy()
laser = sensors.LaserSensor(200, env.original_map, (0.5,0.01))
env.map.fill((env.black))
env.info_map = env.map.copy()

running = True

while running:
    sensor_on = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if pygame.mouse.get_focused():
            sensor_on = True
        elif not pygame.mouse.get_focused():
            sensor_on = False
    if sensor_on:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        env.data_storage(sensor_data)
        env.show_sensor_data()
    env.map.blit(env.info_map, (0, 0))
    pygame.display.update()
"""