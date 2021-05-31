#!/usr/bin/env python
#import libraries
import rospy
from PIL import Image
import cv2 
import numpy as np 
from matplotlib import pyplot as pp
import math
from nav_msgs.msg import Odometry
from threading import Timer
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot
import threading
from math import cos, sin, pi
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from turtlebot3_master.msg import ScanData

positionx = None
positiony = None
zq = 0
wq= 0
xq = 0
yq = 0
alreadyClicked = 0
start = False
J11 = 0
J22 = 0
closestObstacle = 0
currentPosition = 0
closestObtacleAngle = 0
arrayOfObstacles = list(range(0, 360))
assumedx = list(range(0, 360))
assumedy = list(range(0, 360))

def callback1(dt):   

   
    global closestObstacle
    global closestObtacleAngle
    global arrayOfObstacles
    global assumedx
    global assumedy
    global start
    global alreadyClicked    
    closestObstacle = min(dt.ranges)
  	#Looking for the smallest distance 
    closestObtacleAngle = dt.ranges.index(min(dt.ranges))
    t0 = +2.0 * (wq * xq + yq * zq)
    t1 = +1.0 - 2.0 * (xq * xq + yq* yq)
    roll_x = math.atan2(t0, t1)
    #Transformation from Quaternion to Euler angles 
    t2 = +2.0 * (wq * yq - zq * xq)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (wq * zq + xq * yq)
    t4 = +1.0 - 2.0 * (yq * yq + zq * zq)
    yaw_z = math.atan2(t3, t4)
    changedToAngles = (180 * yaw_z)/(math.pi)    
    x = (407/2) + -(1) * ((407 - 0) / (10 - (-10)))

    for distance in range(len(dt.ranges)):
    	#We iterate through every element from the ranges array and transform values to find the assumed x and y position for all of the obstacles
    	arrayOfObstacles[distance] = dt.ranges[distance]
    	#Calculating the angle under which the obstacles lies , taking in consideration both - rotation and scanner angle   	
    	angle = (math.radians(360 - distance - changedToAngles))  	
    	radius = dt.ranges[distance] * 20
    	#calculating the assumed x and y for the obstacles  	
    	x = J11 + (radius * cos(angle))
    	y = J22 + (radius * sin(angle))      	
    	assumedx[distance] = x
    	assumedy[distance] = y
    #Cleaning the arrays    	
    if(len(arrayOfObstacles) > 360):
    	arrayOfObstacles = []
    	assumedx =[]
    	assumedy =[]

def callback(msg):	
	global positionx
	global positiony
	global zq
	global wq
	global xq
	global yq
	positionx =  msg.pose.pose.position.x
	positiony =  msg.pose.pose.position.y
	global J11
	global J22
	#Calculating the current position of the robot
	J11 = (407/2) + -(positionx) * ((407 - 0) / (10 - (-10)))
	J11 = 407 - J11 
	J22 = (407/2) + -(positiony) * ((407 -  0 )/ (10 - (-10)))
	#Getting the current orientation of the robot 
	zq =  msg.pose.pose.orientation.z
	wq =  msg.pose.pose.orientation.w
	yq =  msg.pose.pose.orientation.y
	xq =  msg.pose.pose.orientation.x	
	return J11 , J22 

def cutCirle():
	#Opening the map
	matrix = cv2.imread("/home/harumanager/catkin_ws/src/maps/supermap1.pgm", cv2.IMREAD_COLOR)
	#Cutting the picture
	matrix = matrix[200:607, 0:407]
	#Displaying the robot cooridnates
	newCoorinatex = int(J22)
	newCoorinatey = int(J11)
	matrix[newCoorinatex,newCoorinatey] = [254 , 50 ,50]
	#Applying the mask
	mask = sector_mask(matrix.shape,(J22,J11,),71.225,(0,360))
	matrix[~mask] = 125
	n = np.array(matrix)
	#Getting the number of white and black pixels
	number_of_white_pix = np.sum(matrix== 254)
	number_of_black_pix = np.sum(matrix == [0,0,0])
	obstacleType = None	
	disntance_array = []
	angle_array = []
	#Assigning the coordinates of the black pixels to the array
	if(number_of_black_pix > 0):		
		ycoords, xcoords = np.where((n[:, :, 0:3] == [0,0,0]).all(2))		
		sizeArray = len(xcoords)		
		obstacleTypeArray = []
		#Iterating through all 360 values from scanner reading 	
		for values in range(360):
			obstacleType = None			
			for xy in range(len(xcoords)):
				#If value is infinitive - there are no obstacles
				if(math.isinf(arrayOfObstacles[values]) == True):
					obstacleType = 2
					obstacleTypeArray.append(obstacleType)
					break	

				#Checking if the assumed value lies within 3% error margin as the black pixel values from the cutted piece of map
				if (assumedx[values] - (assumedx[values] * 0.03) <= xcoords[xy] <= assumedx[values] + (assumedx[values] * 0.03)):
					indexxy = xy
					#Checking the same condition for y coorrdinate		
					if (assumedy[values] - (assumedy[values] * 0.03)<= ycoords[indexxy] <= assumedy[values] + (assumedy[values] * 0.03)):
						#Obstacle exists both on the map and simulation 
						obstacleType = 1
						obstacleTypeArray.append(obstacleType)
						break

					#Obstacle doesnt exist on the map
					else:
						if(xy == len(xcoords)-1):					
							
							obstacleType = 0 
							obstacleTypeArray.append(obstacleType)
																  
							  
					
									
				else:
					if(xy == len(xcoords)-1):					
						obstacleType = 0 
						obstacleTypeArray.append(obstacleType)
						

					
	#There are no obstacles on the map	
	else:
		if (math.isinf(closestObstacle) == True ):			
			obstacleType = 2

		elif (math.isinf(closestObstacle) == False):			
			obstacleType = 0 


	AnglesObstacles = list(range(1, 360))	
	rate = rospy.Rate(10)	
	#Publishing the message
	scanDataMsg = ScanData()
	scanDataMsg.angles = AnglesObstacles
	scanDataMsg.ranges = arrayOfObstacles
	scanDataMsg.type = obstacleTypeArray	
	pub.publish(scanDataMsg)	
	Timer(0.01, cutCirle).start()

def point_on_circle():   
    
    #center of circle, angle in degree and radius of circle
    center = [J22, J11]
    angle = (math.radians(closestObtacleAngle - 90))
    radius = closestObstacle * 20.35
    x = J11 + (radius * cos(angle))
    y = J22 + (radius * sin(angle))
   

    return x,y

def sector_mask(shape,centre,radius,angle_range):
    
	#Cutting the piece of map - which resambles the scanner range 
    x,y = np.ogrid[:shape[0],:shape[1]]
    cx,cy = centre
    tmin,tmax = np.deg2rad(angle_range)    
    if tmax < tmin:
            tmax += 2*np.pi    
    r2 = (x-cx)*(x-cx) + (y-cy)*(y-cy)
    theta = np.arctan2(x-cx,y-cy) - tmin   
    theta %= (2*np.pi)   
    circmask = r2 <= radius*radius   
    anglemask = theta <= (tmax-tmin)
    return circmask*anglemask

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
	#start = False
	rospy.init_node('ObstacleType')
	pub = rospy.Publisher("scan_eval", ScanData, queue_size=10)
	odom_sub = rospy.Subscriber('/odom', Odometry, callback)
	sub = rospy.Subscriber("/scan", LaserScan, callback1)		
	startingx = 280
	startingy = 290	
	cutCirle()	
	point_on_circle()	
	rospy.spin()
	
