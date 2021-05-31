#!/usr/bin/env python
#import libraries
import rospy 
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
import math
import os
from threading import Timer

import time
import threading

import pyaudio
import pygame



closestObstacle = 0
currentPosition = 0
maximum = 0
currentmaximum = 0
frequency = 0 
direction = 0
valueOfBeep = 0
Forward = 1

def _del_(self):
	print('Destroyed')

def callback1(msg):
	#Getting the x and y position og the robot from the odometry node
    positionx =  msg.pose.pose.position.x
    positiony =  msg.pose.pose.position.y
    
        

def callback(dt): 
    distances = list(range(0, 90)) 
    Forward = rospy.get_param("/drivingDirection")
    isDriving = rospy.get_param("/ps3LinVel")
    sumOfObstacles = 0 
    global closestObstacle 
    global maximum  
    global frequency
    global direction
    global valueOfBeep
    #Assigning the values - 90 degrees front or back to te array
    if (Forward == 1 or isDriving == 0.0):
        for i in range(0,45):       
            distances[i] = dt.ranges[315 + i]


        for i in range(46,90):        
            distances[i] = dt.ranges[i - 45]

    if (Forward == -1 and isDriving != 0.0):
        for i in range(135, 224):  
            indexBack = abs(135 - i) 
            distances[indexBack] = dt.ranges[i]

    #Assigning the minimam values
    minimum = min(distances)
	frequency = 3.5
    frequency =  minimum
    index = distances.index(frequency)
    #Looking weather the robot is driving forward or backward
    if(Forward == 1 or isDriving == 0.0):
        #Looking if the robot is in the left or the right 45 degrees while driving forward
        if(45- index > 0):
            valueOfBeep = index * 0.02
            direction = 1
        if (45 - index <= 0):
            direction = 2
            valueOfBeep = abs(index - 90) * 0.02
        if(index > 30 and index < 60):
            valueOfBeep = 0.5

    if(Forward == -1 and isDriving != 0.0):
    	#Looking if the robot is in the left or the right 45 degrees while driving backward
        if(45 - index > 0):
            valueOfBeep = index * 0.02
            direction = 2
        if (45 - index <= 0):
            direction = 1
            valueOfBeep = abs(index - 90) * 0.02
        if(index > 30 and index < 60):
            valueOfBeep = 0.5	
   
    pub.publish(move)
    beep()

#Function which will play the sound
def beep():
    currentDirection = 0
    #Checking if the robot is wihtin a certain distance to the obstacle
    if (frequency > 0.6):
        Timer(0.001, beep).start()
    if(frequency <= 0.6):
    	#Calculating the new Frequency - which will be used to define the freqeuncy of the beep
        newFrequency = 3.5 - frequency
        newFrequency =  2 * math.exp(newFrequency)/100
        newFrequency = 0.65 - newFrequency
        currentDirection = direction
        curentBeepingValue = valueOfBeep
        pygame.init()
        #assigning the sound which will be played
        sound0 = pygame.mixer.Sound('/home/harumanager/catkin_ws/src/sounds/beep-07.wav')
        #Checking if the sound should be played from the left or right speakers
        if(currentDirection == 1):
            channel0 = pygame.mixer.Channel(0)
            channel0.play(sound0)
            channel0.set_volume(curentBeepingValue, 1 - curentBeepingValue)
            #print curentBeepingValue
        if(currentDirection == 2):
            channel0 = pygame.mixer.Channel(0)
            channel0.play(sound0)
            channel0.set_volume(1 - curentBeepingValue, curentBeepingValue)           
        #Using the sleep function - to delay the execuion of the function
        time.sleep(float(newFrequency))


if __name__ == '__main__':
    move = Twist() 
    rospy.init_node('obstacle_avoidance_node') 
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)                                  				
    sub = rospy.Subscriber("/scan", LaserScan, callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, callback1) 
    rospy.spin() 
