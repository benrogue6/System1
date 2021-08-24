#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class lane_track: # camera class

  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # inserting the range for the black color in hsv format
    lower_black = numpy.array([ 0, 0, 0])
    upper_black = numpy.array([180, 255, 30])

    # inserting the range for the blue color in hsv format
    lower_blue = numpy.array([110, 50, 50])
    upper_blue = numpy.array([130, 255, 255]) 

    mask1 = cv2.inRange(hsv, lower_black, upper_black)
    mask2 = cv2.inRange(hsv, lower_blue, upper_blue)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
 
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
   
    if M1['m00'] > 0:#black-lane-mask
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err1 = cx1 - w/2
      
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err1) / 100
      self.cmd_vel_pub.publish(self.twist)
    
    if M2['m00'] > 0:#blue-cube-mask
      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])
      err2 = cx2 - w/2
      track = -1
      print "blue"
      print cy2
      print cx2
      print err2
      
      if cy2 > 360:
        
        if err2 < 0:
          track = 1

        self.twist.linear.x = 0.4
        self.twist.angular.z = -float(track) * 0.5
        self.cmd_vel_pub.publish(self.twist)
    
    #cv2.imshow("black_lane_mask",mask1)
    #cv2.imshow("blue_cube_mask",mask2)
    #cv2.imshow("camera_output", image)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_track') #initilize node

    lane_track = lane_track() #lane_track object

    rospy.spin() #loop it#!/usr/bin/env python


