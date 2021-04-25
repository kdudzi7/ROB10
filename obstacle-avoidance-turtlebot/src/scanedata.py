#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry
import math

closestObstacle = 0
currentPosition = 0

def callback1(msg):
    positionx =  msg.pose.pose.position.x
    positiony =  msg.pose.pose.position.y
    #print(positionx , positiony)
        

def callback(dt):  

    sumOfObstacles = 0 

    print '-------------------------------------------'
    #print 'Closest obstacle:   {}'.format(min(dt.ranges))
    print 'Range data at 15 deg:  {}'.format(dt.ranges[180])
    #print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    print '-------------------------------------------'
    global closestObstacle   
    """Yield successive n-sized chunks from lst."""
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
                    sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
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
                    sumOfObstacles = 4.5*(math.exp(sumOfObstacles))
                    sumOfObstacles += sumOfObstacles
                #print sumOfObstacles
                else:
                    sumOfObstacles = 3.5 - dt.ranges[i]
                    sumOfObstacles = 3*(math.exp(sumOfObstacles))
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


    print  totalSumCurrent, totalSumTurnRight , totalSumTurnLeft

    
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
        move.linear.x = 0.1 # go forward (linear velocity)
        move.angular.z = -0.2 # do not rotate (angular velocity)
    else:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.0 # rotate counter-clockwise
        if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2:
            move.linear.x = 0.0
            move.angular.z = 0.0
    pub.publish(move) # publish the move object


move = Twist() # Creates a Twist message type object
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing

sub = rospy.Subscriber("/scan", LaserScan, callback)


odom_sub = rospy.Subscriber('/odom', Odometry, callback1)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

rospy.spin() # Loops infinitely until someone stops the program execution

