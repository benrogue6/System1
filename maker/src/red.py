#!/usr/bin/env python
from __future__ import print_function


import sys
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

class image_converter:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)
    self.twist = Twist()

   
  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
     
      #converting bgr to hsv in order to identify the green color
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
     # crop =cv2.resize(cv_image,(500,250))
    except CvBridgeError as e:
      print(e)

    lower_red = np.array([0, 55, 55])
    upper_red = np.array([10, 255, 255]) 
    mask = cv2.inRange(hsv, lower_red, upper_red)
  
    h, w, d = image.shape
    
    M = cv2.moments(mask)


    if M['m00'] > 0:
     cx = int(M['m10']/M['m00'])
     cy = int(M['m01']/M['m00'])
   #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      
   #err = cx - w/2
   

     if cy > 320:
      self.twist.linear.x = 0.0
      self.twist.angular.z = 0.0
      self.cmd_vel_pub.publish(self.twist)
     
    

    cv2.imshow("Red_detection", mask)
   
    cv2.waitKey(3)

 
     
   
def main(args):
  ic = image_converter()
  rospy.init_node('red', anonymous = True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
    
