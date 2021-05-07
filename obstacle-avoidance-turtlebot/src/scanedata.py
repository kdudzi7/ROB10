#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry
import math
import os
from threading import Timer
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
    positionx =  msg.pose.pose.position.x
    positiony =  msg.pose.pose.position.y
    #print(positionx , positiony)
        

def callback(dt): 
    distances = list(range(0, 90)) 
    Forward = rospy.get_param("/drivingDirection")
    isDriving = rospy.get_param("/ps3LinVel")
    sumOfObstacles = 0 
    print(isDriving)
    #print '-------------------------------------------'
    #print 'Closest obstacle:   {}'.format(min(dt.ranges))
    #print 'Range data at 15 deg:  {}'.format(dt.ranges[180])
    #print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    #print '-------------------------------------------'
    global closestObstacle 
    global maximum  
    global frequency
    global direction
    global valueOfBeep
    """Yield successive n-sized chunks from lst."""
    if (Forward == 1 or isDriving == 0.0):
        for i in range(0,45):       
            distances[i] = dt.ranges[315 + i]


        for i in range(46,90):        
            distances[i] = dt.ranges[i - 45]

    if (Forward == -1 and isDriving != 0.0):
        for i in range(135, 224):  
            indexBack = abs(135 - i) 
            distances[indexBack] = dt.ranges[i]


    minimum = min(distances)
    #print minimum , distances , dt.ranges[316]
    frequency = 3.5
    frequency =  minimum
    print(frequency)
    index = distances.index(frequency)
    print index
    if(Forward == 1 or isDriving == 0.0):
        print("ForwardDriving")
        if(45- index > 0):
            print ("right",index)
            valueOfBeep = index * 0.02
            direction = 1
        if (45 - index <= 0):
            print ("left",index)
            direction = 2
            valueOfBeep = abs(index - 90) * 0.02
    if(Forward == -1 and isDriving != 0.0):
        print("BackwardDriving")
        if(45 - index > 0):
            print "right"
            valueOfBeep = index * 0.02
            direction = 2
        if (45 - index <= 0):
            print "left"
            direction = 1
            valueOfBeep = abs(index - 90) * 0.02
    start = 0
    startRight = 10
    endRight = 55
    startLeft = 35
    endLeft = 80
    k=0
    j=0
    l=0
    end = 45
    totalSumCurrent = 0
    totalSumTurnLeft=0
    totalSumTurnRight=0
    PointsForObstavles = []
    PointsForObstavlesRight =[]
    PointsForObstavlesLeft = []
    for j in range(0,8):
        for i in range(start, end):
            #print i,k , start , end
            if (math.isinf(dt.ranges[i]) == False ):
                if(j == 0 or j == 8):
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 4.5 * (math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
                #print sumOfObstacles
                else:
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 3*(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles

            else:
                sumOfObstacles += 0
                
        
        PointsForObstavles.append(sumOfObstacles)
        totalSumCurrent += sumOfObstacles
        sumOfObstacles = 0
        k+=1
        start = k * 45
        end = start + 44
        maximum = frequency
        
    #print totalSumCurrent

    for j in range(0,7):
        for i in range(startRight, endRight):
            #print startRight , endRight
            #print i,k , start , end
            if (math.isinf(dt.ranges[i]) == False ):
                if(j == 0):
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
                #print sumOfObstacles
                else:
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 3*(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
               
            else:
                sumOfObstacles += 0
                
        
        PointsForObstavlesRight.append(sumOfObstacles)
        totalSumTurnRight += sumOfObstacles
        sumOfObstacles = 0
        j+=1
        startRight = 10 + (j * 45)
        endRight = startRight + 44

    for i in range(0,9):
        sumOfObstacles = 3.5 - dt.ranges[i]
        sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
        sumOfObstacles += sumOfObstacles
    for i in range (325,359):
        sumOfObstacles = 3.5 - dt.ranges[i]
        sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
        sumOfObstacles += sumOfObstacles

    PointsForObstavlesRight.append(sumOfObstacles)
    totalSumTurnRight += sumOfObstacles



    for j in range(0,7):
        for i in range(startLeft, endLeft):
            #print startRight , endRight
            #print i,k , start , end
            if (math.isinf(dt.ranges[i]) == False ):
                if(j == 0):
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 5 *(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
                #print sumOfObstacles
                else:
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
               
            else:
                sumOfObstacles += 0
                
        
        PointsForObstavlesLeft.append(sumOfObstacles)
        totalSumTurnLeft += sumOfObstacles
        sumOfObstacles = 0
        j+=1
        startRight = 35 + (j * 45)
        endRight = startRight + 44

    for i in range(0,34):
        sumOfObstacles = 3.5 - dt.ranges[i]
        sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
        sumOfObstacles += sumOfObstacles
    for i in range (349,359):
        sumOfObstacles = 3.5 - dt.ranges[i]
        sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
        sumOfObstacles += sumOfObstacles

    PointsForObstavlesLeft.append(sumOfObstacles)
    totalSumTurnRight += sumOfObstacles


    #print  totalSumCurrent, PointsForObstavles, maximum
    #beeping()     
    #print(dt.ranges , len(dt.ranges))   
    #print(closestObstacle)
    #closestObstacle = (383.5/2) + -(closestObstacle) * ((383.5 - 0) / (10 - (-10)))
    #print(closestObstacle , closestObtacleAngle)
    

    
    #print (J11)

    thr1 = 0.8 # Laser scan range threshold
    thr2 = 0.8
    if totalSumTurnRight > totalSumTurnLeft : # Checks if there are obstacles in front and
                                                                         # 15 degrees left and right (Try changing the
									 # the angle values as well as the thresholds)
        move.linear.x = 0.0 # go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)
    else:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.0 # rotate counter-clockwise
        if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
            move.linear.x = 0.0
            move.angular.z = 0.0
    pub.publish(move) # publish the move object


def beeping():
    global currentfrequency
    currentfrequency = frequency
    if(frequency < 0.6):
        
        #frequency = 0.1 # Hertz
        #duration  = 2000 # milliseconds
        #threading.Timer(5.0, printit).start()
        #print "Hello, World!"
        beep()
    Timer(0.1, beeping).start()
def beep():
    currentDirection = 0
    if (frequency > 0.6):
        Timer(0.001, beep).start()
    if(frequency <= 0.6):
        #frequency = 0.1 # Hertz
        #duration  = 2000 # milliseconds
        #threading.Timer(5.0, printit).start()
        #print "Hello, World!"
        #print "\a"
        newFrequency = 3.5 - frequency
        #print newFrequency
        newFrequency =  2 * math.exp(newFrequency)/100
        newFrequency = 0.65 - newFrequency
        #print direction
        currentDirection = direction
        curentBeepingValue = valueOfBeep
        pygame.init()
        sound0 = pygame.mixer.Sound('/home/harumanager/catkin_ws/src/sounds/beep-07.wav')
        if(currentDirection == 1):
            channel0 = pygame.mixer.Channel(0)
            channel0.play(sound0)
            channel0.set_volume(curentBeepingValue, 1 - curentBeepingValue)
            #print curentBeepingValue
        if(currentDirection == 2):
            channel0 = pygame.mixer.Channel(0)
            channel0.play(sound0)
            channel0.set_volume(1 - curentBeepingValue, curentBeepingValue)
            #print curentBeepingValue
        #print newFrequency
        #print frequency
        Timer(newFrequency, beep).start()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        move = Twist() # Creates a Twist message type object
        rospy.init_node('obstacle_avoidance_node') # Initializes a node
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                             # outgoing message queue used for asynchronous publishing

        sub = rospy.Subscriber("/scan", LaserScan, callback)


        odom_sub = rospy.Subscriber('/odom', Odometry, callback1)  # Subscriber object which will listen "LaserScan" type messages
        try:
            beep()                                                     # from the "/scan" Topic and call the "callback" function
        except rospy.ROSInterruptException:
            pass						      # each time it reads something from the Topic
	

        rospy.spin() # Loops infinitely until someone stops the program execution

