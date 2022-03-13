from turtle import distance
import pygame
import sys

from sqlalchemy import true
from arena import Arena
from heapq import heappush, heappop
from queue import PriorityQueue
import math
import time
import numpy as np
import math

class AStar:

    def __init__(self):
        self.stepsize = 3
        self.angles = np.array([0, math.pi/6, 2*math.pi/6, 3*math.pi/6, 4*math.pi/6, 5*math.pi/6, 6*math.pi/6])
        # self.angles = np.linspace(0, 360, 360//30)
        print("Angles: ",self.angles)
        self.directions  = np.column_stack( (np.cos(self.angles)*self.stepsize, np.sin(self.angles)*self.stepsize, self.angles) ) 
        # self.stepsize = int(input("Input step size(0 <= L <= 10): "))
        # self.directions = [ [-1, -1],[-1, 0], [-1, 1],
        #             [0, -1], [0, 0], [0, 1],
        #             [1, -1], [1, 0], [1, 1]] 
        print(self.directions)
        self.recently_closed=[]    

    def distance(self, start, goal):
        return math.sqrt(math.pow(start.x-goal.x,2)+math.pow(start.y-goal.y,2))    
        # return (goal.x-start.x)+(goal.y-start.y)

    def search(self, arena):
        solution_found = False

        #Loop through all the open nodes
        if arena.front:
            min_cost_node = heappop(arena.front)

            
            total_cost, cost, current_node, previous = min_cost_node
            if arena.nodes.get((current_node.x,current_node.y, current_node.theta)):
                return solution_found, arena
            
            arena.nodes[(current_node.x,current_node.y, current_node.theta)]=current_node
            # arena.cameFrom[(current_node.x,current_node.y)] = previous

            if current_node == arena.goal_location:
                # arena.goal_location=current_node
                solution_found = True
            
            #TODO: ROBOT can only turn 4 angles from it's current heading
            theta_indx = np.argmin(abs(self.angles-current_node.theta))
            directions_centered = np.roll(self.directions, -2*theta_indx)
            directions = directions_centered[:4]

            #TODO: Implement above condition and remove this
            # directions = self.directions

            # Loop through all the possible actions
            for direction in directions:
                x_ = int(current_node.x + direction[0])
                y_ = int(current_node.y + direction[1])
                theta_ = (current_node.theta + direction[2]) % 2*math.pi

                # Run action which gives the nerest distance: 
                node = Arena.Node(x_, y_, theta_)
                node.parent=current_node

                # Check if the node is created inside the arena
                if(not arena.isValid(node)):
                    continue

                # Check if the newly created node lies inside any obstacles or already created nodes
                if (arena.nodes.get((node.x,node.y, node.theta)) or arena.obstacle_nodes.get((node.x,node.y))):
                    continue

                deltacost = self.distance(current_node, node)
                heappush(arena.front, (deltacost+self.distance(node, arena.goal_location), cost+deltacost, node, current_node))
            
            if arena.goal_location == current_node: 
                arena.goal_node = current_node
                solution_found=true
            
        return solution_found, arena

if __name__ == "__main__":
    arena = Arena()
    dijkstra = AStar()
    solution_found = False
    
    # Paint the obstacles node white
    for i in range(arena.WIDTH):
        for j in range(arena.HEIGHT):
            node = Arena.Node(i, j,0)
            if (arena.isCollision(i,j)):
                obstacle_node = arena.obstacle_nodes.get((node.x,node.y))
                if not obstacle_node:
                    arena.obstacle_nodes[(i, j)] = node
                continue
    
    arena.start_time = time.time()
    while(not solution_found): # your main loop
        # get all events
        arena.updateEvents()
        
        #Search Dijsktra
        solution_found, arena = dijkstra.search(arena)

        # Update MAP - Pygame display
        arena.drawAll()
    arena.drawAll()
    input("Finished algorithm")
    # arena.displayResults()
