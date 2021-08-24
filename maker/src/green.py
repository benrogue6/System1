#!/usr/bin/env python
from __future__ import print_function


import sys
import numpy as np

import rospy, cv2, cv_bridge, numpy
from std_msgs.msg import String
#from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CameraInfo
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
   
    except CvBridgeError as e:
      print(e)

  
    lower_green = np.array([45, 100, 50])
    upper_green = np.array([70, 255, 255]) 

    mask = cv2.inRange(hsv, lower_green, upper_green)
    M = cv2.moments(mask)

    h, w, d = image.shape
    if M['m00'] > 0:#green mask
     cx = int(M['m10']/M['m00'])
     cy = int(M['m01']/M['m00'])
   #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      
     err = cx - w/2
     #  print "green"
     #   print cy1
      
     if cy == 237:
    #  for x in range[5]:
      self.twist.linear.x = 0.4
      self.twist.angular.z = 0.0
      self.cmd_vel_pub.publish(self.twist)
      
  
   
     
    cv2.imshow("green_mask", mask)
  #cv2.imshow("output", image)
    cv2.waitKey(3)

    
 # cv2.imshow("Green_detection", mask)

 
def main(args):
  ic = image_converter()

  rospy.init_node('traffic_green', anonymous = True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()




if __name__ == '__main__':
    main(sys.argv)
    
