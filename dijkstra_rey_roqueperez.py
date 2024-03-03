# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Written by: Rey Roque-Pérez
# UID: 120498626

import heapq as hq

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



def dijkstra_algorithm():
    
    return

# Cost to come, index, parent node index, and coordinate values (x,y)
goal_coordinate = (1200,500)
start_coordinate = (0,0)
open_queue = [(0, 0, 0, start_coordinate)]
hq.heapify(open_queue)

# Key: coordinates (x,y) , Values: Cost to come, index, parent node index 
open_dic={}
open_dic[start_coordinate]=(0,0,0)

# Key: coordinates (x,y) , Values: Cost to come, index, parent node index 
closed_dict = {}

print(get_moves(start_coordinate))