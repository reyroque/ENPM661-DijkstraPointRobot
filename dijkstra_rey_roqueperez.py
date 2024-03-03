# Project 2: Implementation of the Dijkstra Algorithm for a Point Robot
# Written by: Rey Roque-Pérez
# UID: 120498626

import heapq as hq

# Cost to come, index, parent node index, and coordinate values (x,y)
start_coordinate = (0,0)
open_queue = [(0, 0, 0, start_coordinate)]
hq.heapify(open_queue)

# Key: coordinates (x,y) , Values: Cost to come, index, parent node index 
open_dic={}
open_dic[start_coordinate]=(0,0,0)

# Cost to come, index, parent node index, and coordinate values (x,y)
closed_list = []