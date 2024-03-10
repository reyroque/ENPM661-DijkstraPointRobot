# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Written by: Rey Roque-Pérez
# UID: 120498626

import heapq as hq
from queue import PriorityQueue
import time

import numpy as np
import matplotlib.pyplot as plt
import cv2

def move_right(node):
    new_node = (node[0]+1, node[1])
    return new_node

def move_left(node):
    new_node = (node[0]-1, node[1])
    return new_node

def move_up(node):
    new_node = (node[0], node[1]+1)
    return new_node

def move_down(node):
    new_node = (node[0], node[1]-1)
    return new_node

def move_up_right(node):
    new_node = (node[0]+1, node[1]+1)
    return new_node

def move_up_left(node):
    new_node = (node[0]-1, node[1]+1)
    return new_node

def move_down_right(node):
    new_node = (node[0]+1, node[1]-1)
    return new_node

def move_down_left(node):
    new_node = (node[0]-1, node[1]-1)
    return new_node

def get_straight_moves(node):
    actions = (move_right(node),move_left(node),move_up(node),move_down(node))
    return actions

def get_diagonal_moves(node):
    actions = (move_up_right(node),move_up_left(node),move_down_right(node),move_down_left(node))
    return actions

def in_rectangle(node, obstruction, padding):
    return_bool = False
    x1 = obstruction[0][0] - padding
    x2= obstruction[1][0] + padding
    y1 = obstruction[0][1] - padding
    y2 = obstruction[1][1] + padding
    xobj = node[0]
    yobj = node[1]

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

def outside_workspace(object_coordinate, obstacle, padding):
    return_bool = False
    x1 = obstacle[0][0] + padding
    x2= obstacle[1][0] - padding
    y1 = obstacle[0][1] + padding
    y2 = obstacle[1][1] - padding
    xobj = object_coordinate[0]
    yobj = object_coordinate[1]
    if ((xobj <= x1) or (xobj >= x2) or (yobj <= y1) or (yobj >= y2)):
        return_bool = True    
    return return_bool

def in_obstacle(object_coordinate, obstacles, padding):
    return_bool = False
    for obstacle in obstacles:
        if len(obstacle) == 2 and obstacle == ((0,0),(1200,500)):
            if outside_workspace(object_coordinate, obstacle, padding):
                    return_bool = True
        elif len(obstacle) == 2 and obstacle != ((0,0),(1200,500)):
            if in_rectangle(object_coordinate, obstacle, 5):
                return_bool = True
        elif len(obstacle) == 6:
            if in_hexagon(object_coordinate, 5):
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
        if len(obstacle) == 2 and obstacle != ((0,0),(1200,500)):
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
    return

def draw_explored(canvas, points, start_node, goal_node):
    count = 0
    for point in points.keys():
        # Draw the dot for the current point
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(0, 255, 0), thickness=-1)
        count += 1
        if count % 100 == 0:
            count = 0
            cv2.imshow('Dijkstra', canvas)
            cv2.waitKey(1)  # Delay between each dot drawing, adjust as needed  
    #cv2.destroyAllWindows()     
    return

def draw_path(canvas, path):
    for point in path:
        # Draw the dot for the current point
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(255, 255, 0), thickness=-1)

        # Display the updated image
        cv2.imshow('Dijkstra', canvas)
        cv2.waitKey(1)  # Delay between each dot drawing, adjust as needed   
    #cv2.destroyAllWindows()  
    return

def draw_animation(obstacles, explored, path, start_node, goal_node):
    # Initialize canvas
    canvas = np.ones((500, 1200, 3), dtype=np.uint8) * 255  # White canvas
    cv2.rectangle(canvas, (start_node[0] - 2, 500 - start_node[1] - 2), (start_node[0] + 3, 500 - start_node[1] + 3), color=(0, 0, 255), thickness=cv2.FILLED)
    cv2.rectangle(canvas, (goal_node[0] - 2, 500 - goal_node[1] - 2), (goal_node[0] + 3, 500 - goal_node[1] + 3), color=(255, 0, 0), thickness=cv2.FILLED)

    # Draw obstacles on canvas
    draw_obstacles(canvas, obstacles)
    cv2.waitKey(3000)
    draw_explored(canvas, explored, start_node, goal_node)
    draw_path(canvas,path)
    cv2.waitKey(3000)
    return

def obstacle_space():
    obstacle_list = []

    obstacle_list.append(((0,0),(1200,500)))
    obstacle_list.append(((100,100),(175,500)))
    obstacle_list.append(((275,0),(350,400)))
    obstacle_list.append(((900,50),(1100,125)))
    obstacle_list.append(((900,375),(1100,450)))
    obstacle_list.append(((1020,125),(1100,375)))
    
    polygon = (get_hexagon_coordinates((650,250),150,90,0))
    obstacle_list.append(polygon)

    return obstacle_list

def dijkstra_algorithm(start_node, goal_node, obstacles):
    # Create node_grid and initialize cost to come for start_node
    node_grid = [[float('inf')] * 500 for _ in range(1200)]
    node_grid[start_node[0]][start_node[1]] = 0

    # Cost to come, coordinate values (x,y), parent
    # Priority queue to store open nodes
    open_queue = PriorityQueue()
    open_queue.put((0, start_node))  # (priority, node)

    # Dictionary to store the parent nodes
    parent_dict = {}

    # Key: nodes (x,y) , Values: parent 
    open_dict = {}
    open_dict  = {start_node: None}

    while not open_queue.empty():
        _, node = open_queue.get()
        parent = open_dict.pop(node)
        parent_dict[node] = parent

        if node == goal_node:
            print("Path Found")
            return parent_dict

        # Get neighboring nodes
        straight_moves = get_straight_moves(node)
        diagonal_moves = get_diagonal_moves(node)

        for move in straight_moves:
            if not in_obstacle(move, obstacles, 5):
                # Calculate new cost to come
                new_cost = node_grid[node[0]][node[1]] + 1
                if move not in parent_dict:                
                    # Update if new cost is lower
                    if new_cost < node_grid[move[0]][move[1]]:
                        if move not in open_dict:
                            node_grid[move[0]][move[1]] = new_cost
                            open_queue.put((new_cost, move))
                            open_dict[move] = node
                else:
                    if new_cost < node_grid[move[0]][move[1]]:
                        node_grid[move[0]][move[1]] = new_cost
                        parent_dict[move] = node   

        for move in diagonal_moves:
            if not in_obstacle(move, obstacles, 5):
                # Calculate new cost to come
                new_cost = node_grid[node[0]][node[1]] + 1.4
                if move not in parent_dict:                
                    # Update if new cost is lower
                    if new_cost < node_grid[move[0]][move[1]]:
                        if move not in open_dict:
                            node_grid[move[0]][move[1]] = new_cost
                            open_queue.put((new_cost, move))
                            open_dict[move] = node
                else:
                    if new_cost < node_grid[move[0]][move[1]]:
                        node_grid[move[0]][move[1]] = new_cost
                        parent_dict[move] = node  

    return print("Failed to find goal")

def find_path(visited_dict, start, goal):
    current_node = goal
    path = [current_node]
    while start != current_node:
        current_node = visited_dict[current_node]
        path.insert(0, current_node)
    return path

goal_node = (75,75)
start_node = (15, 15)

#xs = int(input('Enter x coordinate value for start location: '))
#ys = int(input('Enter y coordinate value for start location: '))
#xg = int(input('Enter x coordinate value for goal location: '))
#yg = int(input('Enter y coordinate value for goal location: '))

#start_node = tuple((xs, ys))
#goal_node = tuple((xg,yg))

# Get obstacles
obstacles = obstacle_space()

ti = time.time()

print('Exploring nodes')
explored_dict = dijkstra_algorithm(start_node, goal_node, obstacles)

print('Generating path')
path = find_path(explored_dict, start_node, goal_node)

tf = time.time()
print('Path found in: ', tf-ti)

draw_animation(obstacles, explored_dict, path, start_node, goal_node)