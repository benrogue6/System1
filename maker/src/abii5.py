#!/usr/bin/env python
#from __future__ import print_function

import sys, rospy, cv2, cv_bridge, numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

class detect_cube:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)
    self.twist = Twist()

  def callback(self,data):
    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #converting bgr to hsv in order to identify the green color
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # inserting the range for the black color in hsv format
    lower_black = numpy.array([ 0, 0, 0])
    upper_black = numpy.array([180, 255, 30])
 
    # inserting the range for the blue color in hsv format
    lower_blue = numpy.array([110, 50, 50])
    upper_blue = numpy.array([130, 255, 255]) 

    mask1 = cv2.inRange(hsv, lower_black, upper_black)
    mask2 = cv2.inRange(hsv, lower_blue, upper_blue)

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
   
    #cv2.imshow("blue-cube-mask", mask2)
    #cv2.imshow("black-road-mask", mask1)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0
   
    
    if M1['m00'] > 0:#black-road-mask
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err1 = cx1 - w/2
      
      self.twist.linear.x = 0.4
      self.twist.angular.z = -float(err1) / 100
      self.cmd_vel_pub.publish(self.twist)

    if M2['m00'] > 0:#blue-cube-mask
      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])
      err2 = cx2 - w/2
      print "blue"
      print cy2
      
      if cy2 < 226:
        self.twist.linear.x = -0.2
        self.twist.angular.z = -float(err2) / 100
        self.cmd_vel_pub.publish(self.twist)
    
    """
      if cy < 180 and cy > 152:
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
	self.cmd_vel_pub.publish(self.twist)
      elif cy < 152: 
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)
      else:
        self.twist.linear.x = 0.4
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)
    """
    cv2.waitKey(3)

def main(args):
  detect_cube()
  rospy.init_node('traffic_node', anonymous = True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
