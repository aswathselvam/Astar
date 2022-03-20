# from turtle import distance
import pygame
import sys

# from sqlalchemy import true
from arena import Arena
from heapq import heappush, heappop
# from queue import PriorityQueue
import math
import time
import numpy as np
import math

class AStar:

    def __init__(self):
        self.stepsize = 15
        self.angles = [-math.pi/3, math.pi/3, -math.pi/6, math.pi/6, 0]
        self.recently_closed=[]    

    def distance(self, start, goal):
        return math.sqrt(math.pow(start.x-goal.x,2)+math.pow(start.y-goal.y,2))    
        # return (goal.x-start.x)+(goal.y-start.y)

    def cycleTheta(self,theta,delta_theta):
        theta_ = theta + delta_theta
        if theta_ > 0:
            theta_ = theta_ % math.pi
        elif theta_ < 0:
            theta_ = theta_ + 2*math.pi

        return theta_

    def search(self, arena):
        solution_found = False

        #Loop through all the open nodes
        if arena.front:
            # pick mincost node from heap
            min_cost_node = heappop(arena.front)
            total_cost, cost, current_node, previous = min_cost_node
            
            # update the latest cache in Arena.
            arena.latestnodepop.append(min_cost_node)            

            if arena.nodes.get((current_node.x,current_node.y, current_node.theta)):
                return solution_found, arena
            
            # mark node visited in hashmap
            arena.nodes[(current_node.x,current_node.y, current_node.theta)]=current_node

            if current_node == arena.goal_location:
                arena.goal_node = current_node
                solution_found = True
            
            # Loop through all the possible actions            
                        
            angles=[]
            for count, deltaTheta in enumerate(self.angles):
                angles.append(self.cycleTheta(current_node.theta,deltaTheta))
            for theta_ in angles:

                # Create a new neighbor node.
                x_ = int(current_node.x + self.stepsize*np.cos(theta_))
                y_ = int(current_node.y + self.stepsize*np.sin(theta_))
                node = Arena.Node(x_, y_, theta_)
                node.parent=current_node

                # Check if the node is created inside the arena
                if(not arena.isValid(node)):
                    continue

                # Check if the newly created node lies inside any obstacles or already created nodes
                if (arena.nodes.get((node.x,node.y, node.theta)) or \
                        arena.obstacle_nodes.get((node.x,node.y)) or \
                        arena.obstacles_clearance.get((node.x,node.y))):
                    continue
                startcost = self.distance(arena.start_location, current_node)
                deltacost = self.distance(current_node, node) # distance between current node and new neighbor
                goalcost = self.distance(node, arena.goal_location)
                heappush(arena.front, (startcost+deltacost+goalcost, startcost+deltacost, node, current_node))
            
        return solution_found, arena

if __name__ == "__main__":
    arena = Arena()
    planner = AStar()
    solution_found = False
    
    # Paint the obstacles node white
    for i in range(arena.WIDTH):
        for j in range(arena.HEIGHT):
            node = Arena.Node(i, j,0)
            if (arena.isCollision(i,j,clearance=False)):
                    arena.obstacle_nodes[(i, j)] = node
                    continue

            if (arena.isCollision(i,j,clearance=True)):
                arena.obstacles_clearance[(i, j)] = node
    
    arena.start_time = time.time()
    while(not solution_found): # your main loop
        # get all events
        arena.updateEvents()
        
        #Search A star
        solution_found, arena = planner.search(arena)

        # Update MAP - Pygame display
        arena.drawAll()

    arena.drawAll()
    input("Finished algorithm")
    arena.displayResults()
