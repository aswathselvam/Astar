import pygame
import sys
from arena import Arena
import heapq
from queue import PriorityQueue
import math
import time
import numpy as np
import math

class AStar:

    def __init__(self):
        self.stepsize = 1 
        self.angles = [0, math.pi/6, 2*math.pi/6]
        # self.angles = np.linspace(0, 360, 360//30)
        print("Angles: ",self.angles)
        self.directions  = np.column_stack( (np.cos(self.angles)*self.stepsize, np.sin(self.angles)*self.stepsize, self.angles) ) 
        # self.stepsize = int(input("Input step size(0 <= L <= 10): "))
        print(self.directions)
        self.recently_closed=[]

    def search(self, arena):
        solution_found = False
        open_nodes=arena.open_nodes.copy()
        
        #Loop through all the open nodes
        for _,current_node in open_nodes.items():
            if current_node == arena.goal_location:
                arena.goal_location=current_node
                solution_found = True

            theta_indx = np.argmin(abs(self.directions-current_node.theta))
            directions_centered = np.roll(self.directions, -2*theta_indx)
            directions = directions_centered[:4]
            # directions = self.directions
            # Loop through all the possible actions
            for direction in directions:
                x_ = current_node.x + direction[0]
                y_ = current_node.y + direction[1]
                theta_ = current_node.theta + direction[2]
                node = Arena.Node(x_, y_,theta_)

                # Check of the node is created inside the arena
                if(not arena.isValid(node)):
                    continue

                # Check if the newly created node lies inside any obstacles
                if (arena.isCollision(x_,y_)):
                    obstacle_node = arena.obstacle_nodes.get((node.x,node.y))
                    if not obstacle_node:
                        arena.obstacle_nodes[(x_, y_)] = node
                    continue

                
                # Skip evaluating "cost to come" of the newly created node if 
                # any open nodes are already present in the opennode list :
                open_node_visited = arena.open_nodes.get((node.x,node.y))
                if open_node_visited:
                    # Skip evaluating open nodes' parents:
                    open_node_parent_visited = arena.open_nodes.get((open_node_visited.parent.x,open_node_visited.parent.y))
                    if open_node_parent_visited:
                        continue
                    continue

                # Skip evaluating "cost to come" if new node's location 
                # is already a visited nodes:
                node_visited = arena.nodes.get((node.x,node.y))
                if node_visited:
                    continue
                
                # Evaluate "cost to come"
                costToCome = current_node.costToCome + math.sqrt(math.pow(x_-current_node.x,2)+math.pow(y_-current_node.y,2))
                if node.costToCome > costToCome:
                    node.parent=current_node
                    node.costToCome=costToCome
                arena.open_nodes[(node.x,node.y)]=node
            arena.nodes[(current_node.x, current_node.y)] = current_node

            # Delete the node from open list, as it has been visited now:
            del arena.open_nodes[(current_node.x, current_node.y)]

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
