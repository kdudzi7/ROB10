#!/usr/bin/env python

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
from math import cos, sin, pi
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from turtlebot3_master.msg import ScanData

positionx = None
positiony = None
rotationz = 0
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

    #print '-------------------------------------------'
    #print 'Closest obstacle:   {}'.format(min(dt.ranges))
    #print 'Range data at 15 deg:  {}'.format(dt.ranges[15])
    #print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    #print '-------------------------------------------'
    global closestObstacle
    global closestObtacleAngle
    global arrayOfObstacles
    global assumedx
    global assumedy
    global start
    global alreadyClicked  
    #alreadyClicked = 0
    #print start, alreadyClicked
    #print(dt.ranges , len(dt.ranges))
    closestObstacle = min(dt.ranges)
    #print("90 odleglosc",dt.ranges[90])
   # #print ("90 odleglooosc",dt.ranges[dt.ranges[90]] * 20)
    closestObtacleAngle = dt.ranges.index(min(dt.ranges))
    x = (407/2) + -(1) * ((407 - 0) / (10 - (-10)))
    for distance in range(len(dt.ranges)):
    	#print (distance)
    	arrayOfObstacles[distance] = dt.ranges[distance]
    	#print rotationz
    	if (rotationz >  0.98 or rotationz < -0.98):    		
    		if(start == False and alreadyClicked == 0):
    			start = True
    			alreadyClicked = 1

    		if(start == True and alreadyClicked == 0):
    			start = False
    			alreadyClicked = 1
    		   		

    	if start == False:
    		angle = (math.radians(360 - distance + (150 * rotationz)))
    	if start == True: 
    		angle = (math.radians(360 - distance - (150 * rotationz)))
    	 	
    	#print ("kat",angle)
    	radius = dt.ranges[distance] * 20
    	#print start
    	#print (distance,dt.ranges[70])
    	#print (radius)
    	x = J11 + (radius * cos(angle))
    	y = J22 + (radius * sin(angle))      	
    	assumedx[distance] = x
    	assumedy[distance] = y
    	#print start
    	#print("odleglosc", radius)
    	#arrayOfObstacles[distance] = dt.ranges[distance]
    #print(closestObstacle)
    #closestObstacle = (383.5/2) + -(closestObstacle) * ((383.5 - 0) / (10 - (-10)))
    #print(closestObstacle , closestObtacleAngle)
    #print(arrayOfObstacles[4] , dt.ranges[4])

    #print("dlugosc",len(assumedx), assumedx , arrayOfObstacles)
    #assumedx = []
    #assumedy = []
    #arrayOfObstacles = []
    if(len(arrayOfObstacles) > 360):
    	arrayOfObstacles = []
    	assumedx =[]
    	assumedy =[]

def changeTozero():
	global alreadyClicked
	print alreadyClicked
	alreadyClicked = 0
	Timer(1, changeTozero).start()


def callback(msg):
	#print(msg.pose.pose.position.x)
	#print(msg.pose.pose.position.y)	
	global positionx
	global positiony
	global rotationz
	positionx =  msg.pose.pose.position.x
	positiony =  msg.pose.pose.position.y
   

    

	global J11
	global J22


	J11 = (407/2) + -(positionx) * ((407 - 0) / (10 - (-10)))
	J11 = 407 - J11 

	J22 = (407/2) + -(positiony) * ((407 -  0 )/ (10 - (-10)))
	rotationz =  msg.pose.pose.orientation.z
	#print msg.pose.pose.orientation.z
	#print(J11 , J22)
	return J11 , J22 

def cutCirle():
	#matrix = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)
	matrix = cv2.imread("/home/harumanager/catkin_ws/src/ROB10/maps/supermap1.pgm", cv2.IMREAD_COLOR)
	print("zaczynam")	
	#pp.imshow(matrix)
	#pp.show()
	matrix = matrix[200:607, 0:407]
	#pp.imshow(matrix)
	#pp.show()
	#pp.close("all")
	mask = sector_mask(matrix.shape,(J22,J11,),71.225,(0,360))
	matrix[~mask] = 125
	n = np.array(matrix)
	number_of_white_pix = np.sum(matrix== 254)
	number_of_black_pix = np.sum(matrix == [0,0,0])
	obstacleType = None
	obstacleDistance =  closestObstacle
	obstacleAngle = closestObtacleAngle

	print('Number of white pixels:', number_of_white_pix)
	print('Number of black pixels:', number_of_black_pix)
	newCoorinatex = int(J22)
	newCoorinatey = int(J11)
	matrix[newCoorinatex,newCoorinatey] = [254 , 50 ,50]
	matrix[199,151] = [254 , 50 ,50]
	#pp.imshow(matrix)
	#pp.show()
	#pp.close("all")
	disntance_array = []
	angle_array = []
	if(number_of_black_pix > 0):
		print('There is an obstacle')
		ycoords, xcoords = np.where((n[:, :, 0:3] == [0,0,0]).all(2))
		#xcoords = (383.5/2) + -(xcoords) * ((383.5 - 0) / (10 - (-10)))
		#xcoords = 383.5 - xcoords
		#J22 = (383.5/2) + -(ycoords) * ((383.5 -  0 )/ (10 - (-10)))
		#print(xcoords)
		#print(ycoords)
		#print (xcoords[3])
		sizeArray = len(xcoords)
		for x in range(sizeArray):
			#for y in ycoords:
				#print (x)
					#print(x , y)
					#matrix[xcoords,ycoords] = [254,0,0]
					#matrix[startingx+up,startingy] = [60,60,60]
					#number_of_green_pix = np.sum(matrix == [60,60,60])
					#print('Number of green pixels:', number_of_green_pix)
					#254, 0, 0 = im.getpixel(134,179) 
				
				#print("J11",J11)
				#x = (10/2) + -(x) * ((10 - (-10)) / (383.5))
				#x = 383.5 - x
				#y = (383.5/2) + -(y) * ((383.5 -  0 )/ (10 - (-10)))
				#print("x" , xcoords[x])
				#print("J22",J22)

			#print("Y" , ycoords[x])
			#print( "wspolrzedne" , J11 , J22)

			
			

			distance =  math.sqrt(((J22 - ycoords[x])**2) + ((J11 - xcoords[x])**2))
			distance = distance / 20.35			
			dx =   J11
			dy =   J22
			p1 = (J11 , J22)
			p2 = (xcoords[x], ycoords[x])
			#myradians = math.atan2(J11,J22)
			#myradians2 = math.atan2(xcoords[x],ycoords[x])
			#myradians = math.degrees(myradians)
			#myradians = math.degrees(myradians2)
			#mydegrees = myradians2 - myradians2
			ang1 = np.arctan2(*p1[::-1])
			ang2 = np.arctan2(*p2[::-1])
			mydegrees = np.rad2deg((ang1 - ang2) % (2 * np.pi))
			mydegrees %= 360 

			#distance = (20/2) + -(distance) * ((10 - (-10)) / (383.5))	
			disntance_array.append(distance)
			#myradians = math.atan2(J22 - ycoords[x], J11 - xcoords[x])
			#mydegrees = math.degrees(myradians)
			#angle = math.atan2(dy, dx) * 180 / 3.14
			#mydegrees %= 360 


			angle_array.append(mydegrees)
							
			#distance = (20/2) + -(distance) * ((10 - (-10)) / (383.5))
			#print( " new x" ,x)
		#print("odleglosci" , disntance_array)
		#print("katy" , angle_array)

		przewidywana_odleglosc =  math.atan2(12 - 14 ,0 - 0 )
		przewidywana_odleglosc = math.degrees(przewidywana_odleglosc)
		#print("Moze" ,przewidywana_odleglosc)

		smallest_distance = min(disntance_array)
		mallest_angle_index = disntance_array.index(min(disntance_array))
		angle_of_the_smallest_distance = angle_array[mallest_angle_index]		
		print("distance" ,smallest_distance , mallest_angle_index , len(disntance_array) , len(angle_array), xcoords[mallest_angle_index] , ycoords[mallest_angle_index] , angle_of_the_smallest_distance)
		x_on_map = xcoords[mallest_angle_index] 
		y_on_map = ycoords[mallest_angle_index]

		del disntance_array
		del angle_array
		#assumedx,assumedy  = point_on_circle2()
		#print("nasz assumed x", assumedx)
		obstacleTypeArray = []		
		for values in range(360):
			obstacleType = None			
			for xy in range(len(xcoords)):
				if(math.isinf(arrayOfObstacles[values]) == True):
					obstacleType = 2
					obstacleTypeArray.append(obstacleType)
					break
				#print(xy , len(xcoords))

				

			#print('value' , x)
			##x_on_map - (x_on_map * 0.05) <= x <=  x_on_map + (x_on_map * 0.05)
				if (assumedx[values] - (assumedx[values] * 0.04) <= xcoords[xy] <= assumedx[values] + (assumedx[values] * 0.04)):
					indexxy = xy
					#print(indexxy , xcoords[xy] , x )				
					if (assumedy[values] - (assumedy[values] * 0.04)<= ycoords[indexxy] <= assumedy[values] + (assumedy[values] * 0.04)):
						#print("obstacle in both map and simulation", values ,xcoords[xy] ,assumedx[values] , ycoords[xy] , assumedy[values] )
						obstacleType = 1
						obstacleTypeArray.append(obstacleType)
						break

						#obstacleType = 1
					else:
						if(xy == len(xcoords)-1):
						
							#print ("changing to 0.1")
							obstacleType = 0 
							obstacleTypeArray.append(obstacleType)
																  
							  
					
									
				else:
					if(xy == len(xcoords)-1):
					
						#print ("changing to 0")
						obstacleType = 0 
						obstacleTypeArray.append(obstacleType)
						

					
		#if(x_on_map + x_on_map * 0.005) >= x or x_on_map - (x_on_map* 0.005) <= x and y_on_map + (y_on_map * 0.005) >= y or y_on_map - (y_on_map * 0.005) <= y :
		#if x_on_map - (x_on_map * 0.05) <= x <=  x_on_map + (x_on_map * 0.05) and y_on_map - (y_on_map * 0.05) <= y <=  y_on_map + (y_on_map * 0.05) :
			#print("within a margin" ,x_on_map , x )
			#obstacleType = 1
		#else:
			#print("not within margin")
			#obstacleType = 0
	else:
		if (math.isinf(closestObstacle) == True ):
			print("There is no obstacles")
			obstacleType = 2

		elif (math.isinf(closestObstacle) == False):
			print ("There is obstacle only in simulator")
			obstacleType = 0 


	#rrayOfObstacles = arrayOfObstacles[:len(arrayOfObstacles)-360]

	#print("wektor typow", len(obstacleTypeArray), obstacleTypeArray)
	AnglesObstacles = list(range(1, 360))
	#print("obstacle",arrayOfObstacles[5])
	print("dlugosc" , len(assumedx))

	rate = rospy.Rate(10)
	obstacleType1 = [5,5,5]
	#array = [1, 1, 1]
	#print(obstacleTypeArray[1],arrayOfObstacles[1])
	#print("x",assumedx[1])
	#print("y",assumedy[1])
	#assx = int(assumedx[1])
	#assy = int(assumedy[1])

	#matrix[assx,assy] = [60 , 254 ,50]
	#matrix[assy,assx] = [60 , 254 ,50]
	#pp.imshow(matrix)
	#pp.show()
	array = [1, 1 , 1]
	#my_array_for_publishing = Int32MultiArray(data=array)	
	scanDataMsg = ScanData()
	scanDataMsg.angles = AnglesObstacles
	scanDataMsg.ranges = arrayOfObstacles
	scanDataMsg.type = obstacleTypeArray
	#hello_str = obstacleType
	rospy.loginfo(scanDataMsg)
	pub.publish(scanDataMsg)
	print("HAAAAAAAAAAAAAAALLLLLLLLLLLOOOOOOOOOOOOOOOOOO")

	#del arrayOfObstacles
	#arrayOfObstacles = []



	Timer(1, cutCirle).start()

	#return obstacleType
	#pp.close()


def point_on_circle():
    '''
        Finding the x,y coordinates on circle, based on given angle
    '''
    
    #center of circle, angle in degree and radius of circle
    center = [J22, J11]

    angle = (math.radians(closestObtacleAngle - 90))
    radius = closestObstacle * 20.35
    print("radius" ,radius)
    x = J11 + (radius * cos(angle))
    y = J22 + (radius * sin(angle))
    print("czy sie zgadzaja ",x,y)
    print(arrayOfObstacles[4])

    return x,y

def point_on_circle2():
	assumedx = []
	assumedy = []
	center = [J22, J11]	

	for xy in range(360):
		print(xy)
		angle = (math.radians(xy - 90))		
    	radius = arrayOfObstacles[xy] * 20.35
    	print("new radius " , arrayOfObstacles[xy])    	
    	x = J11 + (radius * cos(angle))
    	print("nowe x" , x)
    	y = J22 + (radius * sin(angle))    	
    	assumedx.append(J11 + (arrayOfObstacles[xy]*20.35 * cos(angle)))
    	assumedy.append(J22 + (arrayOfObstacles[xy]*20.35 * sin(angle)))

	print("nasz assumed x dlufi" ,assumedx ,assumedx[5])
	return assumedx , assumedy

def sector_mask(shape,centre,radius,angle_range):
    

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

def NewCoordinates(newx, newy):
	J11 = (383.5/2) + -(newx) * ((383.5 - 0) / (10 - (-10)))
	J11 = 383.5 - J11
	J22 = (383.5/2) + -(newy) * ((383.5 -  0 )/ (10 - (-10)))


	return J11 , J22



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
	#J11 = (383/2) + -(4) * ((383 - 0) / (10 - (-10)))
	#J11 = 383 - J11
	#J22 = (383/2) + -(-2) * ((383 -  0 )/ (10 - (-10)))
	#print(J11 , J22)
	#X = [[J11, 0], 
    #[0, J22]]
	#print("A =", X[1][1])
	#Y = [[-2], [0.4]]
	#print(Y)
	#result = [[0], [0]]
	#for i in range(len(X)):   	
		#for j in range(len(Y[0])):
			#for k in range(len(Y)):
				#result[i][j] += X[i][k] * Y[k][j]

	#for r in result:
		#print(r)

	#matrix = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)
	#matrix1 = cv2.imread("/home/harumanager/map.pgm", cv2.IMREAD_COLOR)
	print(positionx , positiony)
	#newx, newy = NewCoordinates(positionx, positiony)
	#print("NewCoordinates" , newx, newy)
	print(J11)





	#pp.imshow(matrix)
	#pp.show()
	#mask = sector_mask(matrix.shape,(0,0,),40,(0,360))
	#matrix[~mask] = 125
	#pp.imshow(mask)
	#pp.show()
	#number_of_white_pix = np.sum(matrix== 254)
	#number_of_black_pix = np.sum(matrix == [0,0,0])
	#newx, newy = NewCoordinates(positionx, positiony)
	#print("NewCoordinates" , newx, newy)
	#mask = sector_mask(matrix.shape,(newy,newx,),40,(0,360))
	#matrix[~mask] = 125
	#pp.imshow(mask)
	#pp.show()


	print("positionx", positionx)
	cutCirle()
	changeTozero()
	#a = IntList()
	#a.data = [typeOfObstacle,distance,angle]
	point_on_circle()
	print("type",obstacleType)
	del assumedx
	del assumedy
	del arrayOfObstacles
	
	
	#pub = rospy.Publisher('obstacleType', distance , angle , typeOfObstacle)
	#rospy.init_node('talker_obstacle')
	#ros::NodeHandle nh
	#ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("obstacleType/cmd_vel", 100)
	rospy.spin()
	#pp.imshow(matrix)	
	#pp.show()
	#rospy.spin() 
