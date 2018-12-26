#!/usr/bin/env python3

#The initial state is the start city given by user
#The goal city is the end city given by user
#The succesors of a place are the connecting places from that place
#All the other functions are explained above their definitions

#The astar search finds the optimal solution in the fastest manner
#But since the data set was bad, it doesn't give the best output due to the heuristic becoming inconsistent for missing datapoints

import sys
import math
from collections import defaultdict
from queue import PriorityQueue

arguments = sys.argv
start_city = arguments[1]
end_city = arguments[2]
routing_algo = arguments[3]
cost_func = arguments[4]

#Creating a dictionary that stores all the next cities from a city. Both the cities dictionary value is appended with new segments

file = open("road-segments.txt", "r")
seg_dict = defaultdict(list)

for line in file:
        parts = line.split(" ")
        speed = parts[3].strip()
        if speed == "0" or speed == "":
            speed = 30
        time = round(float(parts[2].strip())/float(speed),2)
        seg_dict[parts[0].strip()].append([parts[1].strip(), int(parts[2].strip()), time, parts[4].strip()])
        seg_dict[parts[1].strip()].append([parts[0].strip(), int(parts[2].strip()), time, parts[4].strip()])
file.close()

#Creating a dictionary that stores the coordinates of all the cities present in file

file = open("city-gps.txt", "r")
city_dict = defaultdict(list)

for line in file:
    parts = line.split(" ")
    city_dict[parts[0].strip()].append(float(parts[1].strip()))
    city_dict[parts[0].strip()].append(float(parts[2].strip()))
file.close()

#Going through the segments and city dictionary, I found over 1000 places in segments that are not there in the city dictionary
#To overcome this, I tried calculating the approximate coordinates of the missing cities using barycentric weighted centres
#The segment dictionary is sorted in descending based on number of connections, then stored in a list
#Using this list, if node is not there in city dictionary, its barycentric coordinates ae calculated and it is added to the city dictionary
#But experimenting on the dataset yileded poorer performance and thus I have commented it out

'''
sorted_by_value = sorted(seg_dict.items(), key=lambda kv: len(kv[1]), reverse=True)
print((seg_dict['St-Simeon,_Quebec']))#[0][0])
for seg in sorted_by_value:
    place = seg[0]
    if place not in city_dict:
        connections = seg_dict[place]
        num_connections = len(connections)
        total_mass = 0
        x_mass = 0
        y_mass = 0

        for i in range(num_connections):
            city = connections[i][0]
            dist = connections[i][1]
            mass = float(1/dist)
            if city in city_dict:
                [x, y] = city_dict[city]
                total_mass += mass
                x_mass += mass * x
                y_mass += mass * y
        if total_mass != 0:
            coord_x = x_mass/total_mass
            coord_y = y_mass/total_mass
            city_dict[place] = [coord_x, coord_y]

'''


visit = set()

#All functions use the same successor function
#This function finds the city in segment dictionary and returns all the connections that aren't in visit set

def successors(city):
    states = seg_dict[city]
    cities = list()
    for state in states:
        current_city = state[0]
        if current_city not in visit:
            cities.append(state)
    return cities

#Fringe is the same for all the functions
#Fringe has [city, route till that city, distance till that city, time till that city and all highways taken to reach that city from start]
#Whenever new city is appended, it uses the information of its parent and just adds its own information that it gets from the successors function
#If no route is found all the functions return zeros

#In BFS fringe is popped from zero place and insertions are done at the end of the fringe

def bfs():
	fringe = [[start_city, start_city + "", 0, 0, ""]]
	while len(fringe) > 0:
		[state, route_so_far, total_dist, total_time, h_taken] = fringe.pop(0)
		if state not in visit:
			for succ in successors(state):
				city_name = succ[0]
				dist = succ[1]
				time = succ[2]
				highway = succ[3]
				if city_name == end_city:
					return (route_so_far + " " + city_name), (total_dist + dist), (total_time + time), (
					h_taken + " " + highway)
				fringe.append([(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							   (h_taken + " " + highway)])
		visit.add(state)
	return 0,0,0,0

#In DFS fringe is popped from zero place and insertions are also done from the zero place

def dfs():
	fringe = [[start_city, start_city + "", 0, 0, ""]]
	while len(fringe) > 0:
		[state, route_so_far, total_dist, total_time, h_taken] = fringe.pop(0)
		visit.add(state)
		for succ in successors(state):
			city_name = succ[0]
			dist = succ[1]
			time = succ[2]
			highway = succ[3]
			if city_name == end_city:
				return (route_so_far + " " + city_name), (total_dist + dist), (total_time + time), (
				h_taken + " " + highway)
			fringe.insert(0, [(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							  (h_taken + " " + highway)])
	return 0,0,0,0

#The IDS function iteratively calls a modified dfs (ids_dfs) function. I calculated the number of highways taken to the city popped from fringe which would give us the depth
#If this is more than the depth till which we are currently searching then it doesn't calculate successors for that city and pops another city
#If solution isn't found it returns zeros and ids increases depth and calls again if the depth doesn't exceed the length of the city dictionary

def ids():
	global cond
	cond = True
	depth = 1
	while cond == True and depth <=len(city_dict):
		global visit
		visit = set()
		route, total_dist, total_time, highways = ids_dfs(depth)
		if total_dist == 0:
			depth = depth + 1
		else:
			cond = False
			return route, total_dist, total_time, highways

	return 0,0,0,0

def ids_dfs(depth):
	fringe = [[start_city, start_city + "", 0, 0, ""]]
	while len(fringe) > 0:
		[state, route_so_far, total_dist, total_time, h_taken] = fringe.pop(0)
		visit.add(state)
		if (len(route_so_far.split(" ")) < depth):
			for succ in successors(state):
				city_name = succ[0]
				dist = succ[1]
				time = succ[2]
				highway = succ[3]
				if city_name == end_city:
					return (route_so_far + " " + city_name), (total_dist + dist), (total_time + time), (
						h_taken + " " + highway)
				fringe.insert(0, [(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							  (h_taken + " " + highway)])
	return 0,0,0,0


#For uniform search, the fringe is a priority queue with the cost function at the start and the rest of the information same as before as a list after the priority
#While putting cities in the fringe the order is automatically maintained
#Only cities with least cost are popped
#Depending on the cost function, seperate things are added to the priority
#Uniform with segments cost function gives the same answer as BFS but takes a lot more time


def uniform():
	fringe = PriorityQueue()
	fringe.put((0, [start_city, start_city + "", 0, 0, ""]))
	while fringe.qsize() > 0:
		(priority, [state, route_so_far, total_dist, total_time, h_taken]) = fringe.get()
		visit.add(state)
		for succ in successors(state):
			city_name = succ[0]
			dist = succ[1]
			time = succ[2]
			highway = succ[3]
			if city_name == end_city:
				return (route_so_far + " " + city_name), (total_dist + dist), (total_time + time), (
				h_taken + " " + highway)
			if cost_func == "distance":
				fringe.put((priority + dist, [(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							  (h_taken + " " + highway)]))
			elif cost_func == "time":
				fringe.put((priority + time, [(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							  (h_taken + " " + highway)]))
			else:
				fringe.put((priority + 1, [(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							  (h_taken + " " + highway)]))
	return 0,0,0,0

#Heuristic function for distnace is the haversine distance between the two cities raised to 1/5
#The more power we raised to the better performance we got as the heuristic approached zero and became almost a uniform search
#The haversine distance is admissible and consistent as it is the least distance between any two coordinates on the globe. The highways due to their circuitous nature will
#always take more distance
#Heuristic function for time is the haversine distance divided by the maximum speed (which was found to be 65)
#It is consistent and admissible as the time to reach will always be more than the haversine distance(least possible distance) divided by maximum speed
#Heuristic for segments is the haversine distance divided by the maximum distance between any city (which was found to be 923)
#It is consistent and admissible as the number of segments will always be greater than the haversine distance(least distance) divided by distance of max possible segment
#The Fringe is similar to uniform search with a modification of adding the heuristic also to the priority


def haversine(lat1, lon1, lat2, lon2):
    R = 3959.87433
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    a = math.sin(dLat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon / 2) ** 2
    c = 2 * math.asin(math.sqrt(a))
    return (R * c)**(1./5)

def dist_heuristic(succ):
	if succ in city_dict:
		return abs(haversine(city_dict[succ][0], city_dict[succ][1], city_dict[end_city][0], city_dict[end_city][1]))
	else:
		return 0

def time_heuristic(succ):
	temp_dist = dist_heuristic(succ)
	return float(temp_dist/65)

def seg_heuristic(succ):
	temp_dist = dist_heuristic(succ)
	return float(temp_dist/923)

def astar():
	fringe = PriorityQueue()
	fringe.put((0, [start_city, start_city + "", 0, 0, ""]))
	while fringe.qsize() > 0:
		(priority, [state, route_so_far, total_dist, total_time, h_taken]) = fringe.get()
		visit.add(state)
		for succ in successors(state):
			city_name = succ[0]
			dist = succ[1]
			time = succ[2]
			highway = succ[3]
			if city_name == end_city:
				return (route_so_far + " " + city_name), (total_dist + dist), (total_time + time), (
					h_taken + " " + highway)
			if cost_func == "distance":
				h_dist = dist_heuristic(city_name)
				fringe.put((priority + dist + h_dist,
							[(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							 (h_taken + " " + highway)]))

			elif cost_func == "time":
				h_time = time_heuristic(city_name)
				fringe.put((priority + time + h_time,
							[(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							 (h_taken + " " + highway)]))
			else:
				h_seg = seg_heuristic(city_name)
				fringe.put((priority + 1 + h_seg,
							[(city_name), (route_so_far + " " + city_name), (total_dist + dist), (total_time + time),
							 (h_taken + " " + highway)]))
	return 0,0,0,0



optimal = ""

if routing_algo == 'bfs':
	route, total_dist, total_time, highways = bfs()
	if cost_func == "segments":
		optimal = "yes"
	else:
		optimal = "no"

elif routing_algo == 'dfs':
	route, total_dist, total_time, highways = dfs()
	optimal = "no"

elif routing_algo == 'ids':
	route, total_dist, total_time, highways = bfs()
	if cost_func == "segments":
		optimal = "yes"
	else:
		optimal = "no"

elif routing_algo == 'uniform':
	route, total_dist, total_time, highways = uniform()
	optimal = "yes"

elif routing_algo == 'astar':
	route, total_dist, total_time, highways = astar()
	optimal = "yes"

if route == 0:
	print("No route found")

else:
	city_names = route.split(" ")
	highway_names = highways.split(" ")
	count = len(city_names)

	for i in range(count-1):
		print("From " + city_names[i] + " take the " + highway_names[i+1] + " to " + city_names[i+1])

if route != 0:
	print(optimal, total_dist, round(total_time, 2), route)

