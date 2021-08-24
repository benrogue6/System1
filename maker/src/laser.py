#!/usr/bin/env python
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from geometry_msgs.msg import Twist
import numpy as np

class Laser(): #main class
   
    def __init__(self): #main function
       #global circle
        global move
        move = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message

    def callback(self, msg): #function for obstacle avoidance
        print '-------RECEIVING LIDAR SENSOR DATA-------'
        print 'Front: {}'.format(msg.ranges[0]) #lidar data for front side
        print 'Left:  {}'.format(msg.ranges[90]) #lidar data for left side
        print 'Right: {}'.format(msg.ranges[270]) #lidar data for right side
        print 'Back: {}'.format(msg.ranges[180]) #lidar data for back side
      
      	#Obstacle Avoidance
      # self.distance = 0.7
        self.distance = 0.3
        if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance: 
        #when no any obstacle near detected
            move.linear.x = 0.5 # go (linear velocity)
           #move.linear.x = 0.5 # go (linear velocity)

          # move.angular.z = 0.1 # rotate (angular velocity)
          # move.angular.z = 0.0 # rotate (angular velocity)

           #rospy.loginfo("Circling") #state situation constantly
        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            move.linear.x = 0.0 # stop
            move.angular.z = 0.5 # rotate counter-clockwise
          # move.angular.z =-0.5 # rotate clockwise

            if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance and msg.ranges[45] > self.distance and msg.ranges[180] > self.distance:
                #when no any obstacle near detected after rotation
                move.linear.x = 0.5 #go
            #   move.angular.z = 0.1 #rotate
             #  move.angular.z = 0.5 # rotate counter-clockwise

        self.pub.publish(move) # publish the move object

    def odometry(self, msg): #function for odometry
        print msg.pose.pose #print position and orientation of turtlebot

if __name__ == '__main__':
    rospy.init_node('laser_scan') #initialize node
    Laser() #run class
    rospy.spin() #loop it
