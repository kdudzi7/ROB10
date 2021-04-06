#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from nav_msgs.msg import Odometry

closestObstacle = 0
currentPosition = 0

def callback1(msg):
    positionx =  msg.pose.pose.position.x
    positiony =  msg.pose.pose.position.y
    #print(positionx , positiony)
        

def callback(dt):   

    print '-------------------------------------------'
    print 'Closest obstacle:   {}'.format(min(dt.ranges))
    #print 'Range data at 15 deg:  {}'.format(dt.ranges[15])
    #print 'Range data at 345 deg: {}'.format(dt.ranges[345])
    print '-------------------------------------------'
    global closestObstacle
    print(dt.ranges , len(dt.ranges))
    closestObstacle = min(dt.ranges)
    closestObtacleAngle = dt.ranges.index(min(dt.ranges))
    x = (383.5/2) + -(1) * ((383.5 - 0) / (10 - (-10)))
    print(closestObstacle)
    #closestObstacle = (383.5/2) + -(closestObstacle) * ((383.5 - 0) / (10 - (-10)))
    print(closestObstacle , closestObtacleAngle)
    

    
    #print (J11)

    thr1 = 0.8 # Laser scan range threshold
    thr2 = 0.8
    if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: # Checks if there are obstacles in front and
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
