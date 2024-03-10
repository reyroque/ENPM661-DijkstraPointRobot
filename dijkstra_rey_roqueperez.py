# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Github Repository: https://github.com/reyroque/ENPM661-DijkstraPointRobot
# Written by: Rey Roque-Pérez
# UID: 120498626

from queue import PriorityQueue
import time

import numpy as np
import cv2

###### Move functions ######
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

###### End Move functions ######

# Checks if a given point is inside a rectangle
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
def half_plane(coordinate_1, coordinate_2,node):
    slope = get_slope(coordinate_1, coordinate_2)
    x1, y1 = coordinate_1
    x, y = node
    equation = slope*(x-x1)-y+y1
    return equation

# Checks if a given point is inside a hexagon
def in_hexagon(node, padding):
    hexagon = get_hexagon_coordinates((650,250),150,90,padding)
    H1 = False
    H2 = False
    H3 = False
    H4 = False
    H5 = False
    H6 = False

    # Variables H are booleans for the half-planes
    if half_plane(hexagon[0],hexagon[1],node) >= 0:
        H1 = True
    # H2 half plane is vertical
    if node[0] >= hexagon[1][0]:
        H2 = True
    if half_plane(hexagon[2],hexagon[3],node) <= 0:
        H3 = True
    if half_plane(hexagon[3],hexagon[4],node) <= 0:
        H4 = True
    # H5 half plane is vertical    
    if node[0] <= hexagon[5][0]:
        H5 = True
    if half_plane(hexagon[5],hexagon[0],node) >= 0:
        H6 = True

    return (H1 & H2 & H3 & H4 & H5 & H6)    

# Checks if the point is outside the map
def outside_workspace(node, obstacle, padding):
    return_bool = False
    x1 = obstacle[0][0] + padding
    x2= obstacle[1][0] - padding
    y1 = obstacle[0][1] + padding
    y2 = obstacle[1][1] - padding
    xobj = node[0]
    yobj = node[1]
    if ((xobj <= x1) or (xobj >= x2) or (yobj <= y1) or (yobj >= y2)):
        return_bool = True    
    return return_bool

# Checks if node is ouside the map or inside an obstacle
def in_obstacle(node, obstacles, padding):
    return_bool = False
    for obstacle in obstacles:
        if len(obstacle) == 2 and obstacle == ((0,0),(1200,500)):
            if outside_workspace(node, obstacle, padding):
                    return_bool = True
        elif len(obstacle) == 2 and obstacle != ((0,0),(1200,500)):
            if in_rectangle(node, obstacle, 5):
                return_bool = True
        elif len(obstacle) == 6:
            if in_hexagon(node, 5):
                return_bool = True
    return return_bool

# Determines the edges of a hexagon of r radius at a given point and angle
def get_hexagon_coordinates(center_coordinate, radius, angle, padding):
    # Define angles for each point of the hexagon
    angles = np.deg2rad(np.arange(0, 360, 60) + angle)
    
    # Calculate x and y coordinates for each angle
    x_coordinates = np.round((radius + padding) * np.cos(angles) + center_coordinate[0])
    y_coordinates = np.round((radius + padding) * np.sin(angles) + center_coordinate[1])
    
    # Combine x and y coordinates into tuples
    coordinates = [(int(x), int(y)) for x, y in zip(x_coordinates, y_coordinates)]
    return coordinates

# Draws start node and end node
def draw_nodes(canvas, start_node, goal_node):
    cv2.rectangle(canvas, (start_node[0] - 4, 500 - start_node[1] - 4), (start_node[0] + 6, 500 - start_node[1] + 6), color=(0, 250, 0), thickness=cv2.FILLED)
    cv2.rectangle(canvas, (goal_node[0] - 4, 500 - goal_node[1] - 4), (goal_node[0] + 6, 500 - goal_node[1] + 6), color=(0, 0, 255), thickness=cv2.FILLED)

# Populates the canvas with the obstacles
def draw_obstacles(canvas, obstacles, video_output):
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
    cv2.imshow('Dijkstra', canvas)
    video_output.write(canvas)
    cv2.waitKey(2000)
    return

# Populates and updates the canvas with explored nodes
def draw_explored(canvas, points, video_output):
    count = 0
    for point in points.keys():
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(200, 0, 0), thickness=-1)
        count += 1
        if count % 700 == 0:
            count = 0
            cv2.imshow('Dijkstra', canvas)
            cv2.waitKey(int(1000 / 60)) 
            video_output.write(canvas)   
    return

# Populates and updates the canvas with path nodes
def draw_path(canvas, path, video_output):
    count = 0
    for point in path:
        cv2.rectangle(canvas, (point[0], 500-point[1]), (point[0] + 1, 500-point[1] + 1), color=(0, 0, 250), thickness=2)
        count += 1
        if count % 5 == 0:
            count = 0
            cv2.imshow('Dijkstra', canvas)
            video_output.write(canvas)
            cv2.waitKey(int(1000 / 60))  
    return

# Adds seconds at end of video
def add_blank_frames(canvas, video_output, fps, seconds):
    blank_frames = fps * seconds
    for _ in range(blank_frames):
        video_output.write(canvas)
    return

# Calls the functions for populating the canvas
def record_animation(obstacles, explored, path, start_node, goal_node):
    # Initialize VideoWriter
    v_writer = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 60
    video_output = cv2.VideoWriter('dijkstra_output.mp4', v_writer, fps, (1200, 500))

    canvas = np.ones((500, 1200, 3), dtype=np.uint8) * 255  # White canvas
    
    draw_nodes(canvas, start_node, goal_node)
    draw_obstacles(canvas, obstacles, video_output)
    add_blank_frames(canvas, video_output, fps, 2)    
    draw_explored(canvas, explored, video_output)
    draw_nodes(canvas, start_node, goal_node)
    draw_path(canvas,path, video_output)
    cv2.waitKey(3000)
    add_blank_frames(canvas, video_output, fps, 2)
    video_output.release()
    return

# Creates a list of the obstacles in the workspace
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

    # Priority queue to store open nodes
    # Cost to come, coordinate values (x,y), parent
    open_queue = PriorityQueue()
    open_queue.put((0, start_node))  # (priority, node)

    # Visited/Parent dictionary
    # Key: nodes (x,y) , Values: parent
    parent_dict = {}

    # Key: nodes (x,y) , Values: parent
    open_dict = {}
    open_dict  = {start_node: None}

    while not open_queue.empty():
        _, node = open_queue.get()
        parent = open_dict.pop(node)
        parent_dict[node] = parent

        if node == goal_node:
            return parent_dict

        # Get neighboring nodes
        straight_moves = get_straight_moves(node)
        diagonal_moves = get_diagonal_moves(node)

        for move in straight_moves:
            if move not in parent_dict and move not in open_dict and not in_obstacle(move, obstacles, 5):
                new_cost = node_grid[node[0]][node[1]] + 1
                if new_cost < node_grid[move[0]][move[1]]:
                    node_grid[move[0]][move[1]] = new_cost
                    open_queue.put((new_cost, move))
                    open_dict[move] = node

        for move in diagonal_moves:
            if move not in parent_dict and move not in open_dict and not in_obstacle(move, obstacles, 5):
                new_cost = node_grid[node[0]][node[1]] + 1.4
                if new_cost < node_grid[move[0]][move[1]]:
                    node_grid[move[0]][move[1]] = new_cost
                    open_queue.put((new_cost, move))
                    open_dict[move] = node  

    return print("Failed to find goal")

# Backtracking using path list created from visited/path dictionary
def find_path(visited_dict, start, goal):
    current_node = goal
    path = [current_node]
    while start != current_node:
        current_node = visited_dict[current_node]
        path.insert(0, current_node)
    return path


#### Main ###
# Get obstacles
obstacles = obstacle_space()
padding = 5

# Get and verify input coordinates
xs = int(input('Enter x coordinate value for start coordinate: '))
ys = int(input('Enter y coordinate value for start coordinate: '))
start_node = tuple((xs, ys))
while in_obstacle(start_node, obstacles, padding):
    print('Node outside workspace or in obstacle. Choose new start location')
    xs = int(input('Enter x coordinate value for start location: '))
    ys = int(input('Enter y coordinate value for start location: '))    
    start_node = tuple((xs, ys))

# Get and verify input coordinates
xg = int(input('Enter x coordinate value for goal coordinate: '))
yg = int(input('Enter y coordinate value for goal coordinate: '))
goal_node = tuple((xg,yg))
while in_obstacle(goal_node, obstacles, padding):
    print('Node outside workspace or in obstacle. Choose new goal location')
    xg = int(input('Enter x coordinate value for goal location: '))
    yg = int(input('Enter y coordinate value for goal location: '))
    goal_node = tuple((xg,yg))

# Start timer
ti = time.time()

print('Exploring nodes...')
explored_dict = dijkstra_algorithm(start_node, goal_node, obstacles)

print('Generating path...')
path = find_path(explored_dict, start_node, goal_node)

# Get time taken to find path
tf = time.time()
print('Path found in: ', tf-ti)

record_animation(obstacles, explored_dict, path, start_node, goal_node)