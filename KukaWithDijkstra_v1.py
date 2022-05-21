#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 02:04:33 2020

@author: nithila
NAME: Nithilasaravanan Kuppan
ID: 260905444
Email: nithilasaravana.kuppan@mail.mcgill.ca

ECSE 683 ASSIGNMENT 1
(Final Version)
"""

import pybullet as p
import time
import math
import pybullet_data
from DijsktraForKukaEE import Dijkstra
import matplotlib.pyplot as plt

def main():
    
    show_animation = True
    
    print("\n Please choose the start, end and obstacle position's X and Y between -1 to 1 IN THE CODE \n -> Z is fixed at 0.5 \n")
    
    StartPoint_x = -0.7
    StartPoint_y = -0.7
    EndPoint_x = 1.1
    EndPoint_y = 1.1
    Obstacle_x = 0.2
    Obstacle_y = 0.2
    """
    Boundary Selection: Please set it to 1 of SAFE boundary of 0.2 units around the obstacle that will
    majorly prevent issues of the arm (other than the end effector) to go through the object
    """
    BoundarySelection = 1 # 1 for SAFE (0.2 units) and 0 for NOT SO SAFE (0.1 units) around the obstacle
    
    print("The starting point for Kuka's end effector has been chosen as:", StartPoint_x,", ", StartPoint_y,"\n")
    print("The ending point for Kuka's end effector has been chosen as:", EndPoint_x,", ", EndPoint_y,"\n")
    print("The Obstacle is at", Obstacle_x,", ", Obstacle_y,"\n")
    print("The simulation will begin after Dijkstra's algorithm finds out the best path \n \n")
    
    
    # start and goal position (reversing to get a proper order of points for Kuka to simulate)
    sx = EndPoint_x
    sy = EndPoint_y  
    gx = StartPoint_x
    gy = StartPoint_y
    grid_size = 0.1  #Fixed this for Dijkstra's to search in steps of 0.1
    robot_radius = 0.2  #Fixed this to maintain safe distance from the boundary from crashing into the obstacle

    # Initializing obstacle variables
    ox, oy = [], []
    
    
    #Setting a boundary for the the algorithm (Box grid around the plane)
    #Fixing boundary axis 1
    for i in range(-2,2):
        ox.append(i)
        oy.append(-2)
    for i in range(-2,2):
        ox.append(i)
        oy.append(2)
    #Fixing boundary axis 2
    for i in range(-2,2):
        oy.append(i)
        ox.append(-2)
    for i in range(-2,2):
        oy.append(i)
        ox.append(2)
    for i in range(1):
        oy.append(i+2)
        ox.append(i+2)
        
    obs_s_x = int((Obstacle_x -0.2)*10)
    obs_e_x = int((Obstacle_x +0.2)*10)
         
    
    #For the Obstacle (Creating a SAFE boundary of 0.2 units )
    if BoundarySelection ==1:
        for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y - 0.2)
        for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y - 0.1)
        for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y - 0.0)
        for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y + 0.1)
        for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y + 0.2)
    #Creating a NOT SO SAFE boundary of 0.1 units - MIGHT LEAD TO ARM GOING THORUGH THE OBSTACLE
    else:
         for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y - 0.1)
         for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y - 0.0)
         for i in range(obs_s_x,obs_e_x):
            ox.append(i/10)
            oy.append(Obstacle_y + 0.1)
        
    FindMyPath = Dijkstra
    FinalPath = FindMyPath(ox, oy, grid_size, robot_radius)
    rx, ry = FinalPath.planning(sx, sy, gx, gy)
    
    #To show the final plot that was found out by the algorith along with the path and obstacles
    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

   

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(1/120)
        plt.show()
    
    
    """
    Code for simulating the above calculated path begins: Kuka End Effector 
    
    """

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    #Loading all required models
    p.loadURDF("plane.urdf", [0, 0, -0.3])
    kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
    
    p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
    kukaEndEffectorIndex = 6
    numJoints = p.getNumJoints(kukaId)
    
    #restposes for null space
    rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]


    for i in range(numJoints):
        p.resetJointState(kukaId, i, rp[i])
     
        
    group = 0#other objects don't collide with me
    mask=0 # don't collide with any other object
       
    #Loading sphere_small to act like an obstacle
    ObstacleObj = p.loadURDF("sphere_small.urdf", [Obstacle_x, Obstacle_y, 0.5], useFixedBase = True)
    
    #To avoid stalling the simulation when links go near or through the obstacle
    p.setCollisionFilterGroupMask(kukaId, 0, group, mask) 
    p.setCollisionFilterGroupMask(kukaId, 1, group, mask)
    p.setCollisionFilterGroupMask(kukaId, 2, group, mask)
    p.setCollisionFilterGroupMask(kukaId, 3, group, mask)
    p.setCollisionFilterGroupMask(kukaId, 4, group, mask)
    p.setCollisionFilterGroupMask(kukaId, 5, group, mask)
    p.setCollisionFilterGroupMask(kukaId, 6, group, mask)    

    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(0)
    #trailDuration = 15
    z=0.5
    
    
    
    for i in range(len(rx)): 

        pos = [rx[i], ry[i], z]

        jointPoses = p.calculateInverseKinematics(kukaId,kukaEndEffectorIndex, pos, solver = 0)

        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=kukaId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=100,
                                    positionGain=0.03,
                                    velocityGain=1)
            
        for i in range(50):
            p.stepSimulation()
            time.sleep(1/50) 

  
    p.disconnect()
    return(rx, ry)

if __name__ == '__main__':
   FinalPathX, FinalPathY = main()
