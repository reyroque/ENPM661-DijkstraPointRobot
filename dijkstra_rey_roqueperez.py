# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Written by: Rey Roque-Pérez
# UID: 120498626

import heapq as hq
import time

import numpy as np
import matplotlib.pyplot as plt
import cv2

def move_right(coordinate):
    new_coordinate = [coordinate[0]+1, coordinate[1]]
    return new_coordinate

def move_left(coordinate):
    new_coordinate = [coordinate[0]-1, coordinate[1]]
    return new_coordinate

def move_up(coordinate):
    new_coordinate = [coordinate[0], coordinate[1]+1]
    return new_coordinate

def move_down(coordinate):
    new_coordinate = [coordinate[0], coordinate[1]-1]
    return new_coordinate

def move_up_right(coordinate):
    new_coordinate = [coordinate[0]+1, coordinate[1]+1]
    return new_coordinate

def move_up_left(coordinate):
    new_coordinate = [coordinate[0]-1, coordinate[1]+1]
    return new_coordinate

def move_down_right(coordinate):
    new_coordinate = [coordinate[0]+1, coordinate[1]-1]
    return new_coordinate

def move_down_left(coordinate):
    new_coordinate = [coordinate[0]-1, coordinate[1]-1]
    return new_coordinate

def get_moves(coordinate):
    actions = [move_right(coordinate),move_left(coordinate),move_up(coordinate),move_down(coordinate),move_up_right(coordinate),move_up_left(coordinate),move_down_right(coordinate),move_down_left(coordinate)]
    return actions

def in_rectangle(object_coordinate, obstruction, padding):
    return_bool = False
    x1 = obstruction[0][0] - padding
    x2= obstruction[1][0] + padding
    y1 = obstruction[0][1] - padding
    y2 = obstruction[1][1] + padding
    xobj = object_coordinate[0]
    yobj = object_coordinate[1]

    #print("x1:", x1," x2:", x2, " y1:", y1, " y2:", y2)

    if ((xobj >= x1) and (xobj <= x2) and (yobj >= y1) and (yobj <= y2)):
        return_bool = True

    return return_bool

# Will cause an error if x2 - x1 = 0
def get_slope(coordinate_1, coordinate_2):
    x1, y1 = coordinate_1
    x2, y2 = coordinate_2
    slope = (y2-y1)/(x2-x1)
    return slope

# Returns f(x,y) for a half plane created by two points
def half_plane(coordinate_1, coordinate_2,object_coordinate):
    slope = get_slope(coordinate_1, coordinate_2)
    x1, y1 = coordinate_1
    x, y = object_coordinate
    equation = slope*(x-x1)-y+y1
    return equation

def in_hexagon(object_coordinate, padding):
    hexagon = get_hexagon_coordinates((650,250),150,90,padding)
    H1 = False
    H2 = False
    H3 = False
    H4 = False
    H5 = False
    H6 = False

    # Variables H are booleans for the half-planes
    if half_plane(hexagon[0],hexagon[1],object_coordinate) >= 0:
        H1 = True
    # H2 half plane is vertical
    if object_coordinate[0] >= hexagon[1][0]:
        H2 = True
    if half_plane(hexagon[2],hexagon[3],object_coordinate) <= 0:
        H3 = True
    if half_plane(hexagon[3],hexagon[4],object_coordinate) <= 0:
        H4 = True
    # H5 half plane is vertical    
    if object_coordinate[0] <= hexagon[5][0]:
        H5 = True
    if half_plane(hexagon[5],hexagon[0],object_coordinate) >= 0:
        H6 = True

    return (H1 & H2 & H3 & H4 & H5 & H6)    

def in_obstacle(obstacles):
    return_bool = False
    for obstacle in obstacles:
        if len(obstacle) == 2:
            if in_rectangle(start_coordinate, obstacle, 0):
                return_bool = True
        elif len(obstacle) == 6:
            if in_hexagon(start_coordinate, obstacle, 5):
                return_bool = True
    return return_bool

def get_hexagon_coordinates(center_coordinate, radius, angle, padding):
    # Define angles for each point of the hexagon
    angles = np.deg2rad(np.arange(0, 360, 60) + angle)
    
    # Calculate x and y coordinates for each angle
    x_coordinates = np.round((radius + padding) * np.cos(angles) + center_coordinate[0])
    y_coordinates = np.round((radius + padding) * np.sin(angles) + center_coordinate[1])
    
    # Combine x and y coordinates into tuples
    coordinates = [(int(x), int(y)) for x, y in zip(x_coordinates, y_coordinates)]
    return coordinates

def draw_obstacles(canvas, obstacles):
    for obstacle in obstacles:
        if len(obstacle) == 2:
            start_x = obstacle[0][0]
            start_y = 500 - obstacle[0][1]  # Invert y-value
            end_x = obstacle[1][0]
            end_y = 500 - obstacle[1][1]  # Invert y-value
            start_coordinate = (start_x, start_y)
            end_coordinate = (end_x, end_y)
            cv2.rectangle(canvas, pt1=start_coordinate, pt2=end_coordinate, color=(0, 0, 0), thickness=-1)
        elif len(obstacle) == 6:
            polygon = np.array(obstacle)
            polygon = polygon.reshape(-1, 1, 2)
            cv2.fillPoly(canvas, [polygon], color=(0, 0, 0))

def obstacle_space():
    obstacle_list = []

    obstacle_list.append(((100,100),(175,500)))
    obstacle_list.append(((275,0),(350,400)))
    obstacle_list.append(((900,50),(1100,125)))
    obstacle_list.append(((900,375),(1100,450)))
    obstacle_list.append(((1020,125),(1100,375)))
    
    polygon = (get_hexagon_coordinates((650,250),150,90,0))
    obstacle_list.append(polygon)

    return obstacle_list

def dijkstra_algorithm():
    
    return

# Cost to come, index, parent node index, and coordinate values (x,y)
goal_coordinate = (1200,250)
start_coordinate = (450,250)
open_queue = [(0, 0, 0, start_coordinate)]
hq.heapify(open_queue)

# Key: coordinates (x,y) , Values: Cost to come, index, parent node index 
open_dic={}
open_dic[start_coordinate]=(0,0,0)

# Key: Parent index, Values: Cost to come, index, and coordinate values (x,y) 
closed_dict = {}

# Get obstacles
obstacles = obstacle_space()

# Initialize canvas
canvas = np.ones((500, 1200, 3), dtype=np.uint8) * 255  # White canvas

# Draw obstacles on canvas
draw_obstacles(canvas, obstacles)

# Display canvas
plt.imshow(canvas)
plt.show()

print(in_hexagon(start_coordinate,5))
