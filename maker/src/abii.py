#!/usr/bin/env python
#from __future__ import print_function

import sys, rospy, cv2, cv_bridge, numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

class detect_lane:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.lane_callback)
    self.twist = Twist()

  def lane_callback(self, msg):
    image_a = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #converting bgr to hsv in order to identify the black color
    hsv_a = cv2.cvtColor(image_a, cv2.COLOR_BGR2HSV)

    # inserting the range for the black color in hsv format
    lower_black = numpy.array([ 0, 0, 0])
    upper_black = numpy.array([180, 255, 30])

    mask4 = cv2.inRange(hsv_a, lower_black, upper_black)
    M4 = cv2.moments(mask4)

    h, w, d = image_a.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask4[0:search_top, 0:w] = 0
    mask4[search_bot:h, 0:w] = 0
    
    if M4['m00'] > 0:
      cx4 = int(M4['m10']/M4['m00'])
      cy4 = int(M4['m01']/M4['m00'])
      err = cx4 - w/2
      self.twist.linear.x = 0.3
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)

class detect_lights(object):

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)
    self.twist = Twist()
    self.lane_follow = detect_lane()


  def callback(self,msg):
    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #converting bgr to hsv in order to identify the color
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # inserting the range for the green color in hsv format
    lower_green = numpy.array([45, 100, 50])
    upper_green = numpy.array([70, 255, 255]) 

    # inserting the range for the red color in hsv format
    lower_red = numpy.array([0, 150, 230])
    upper_red = numpy.array([10, 255, 255]) 

    # inserting the range for the yellow color in hsv format
    lower_yellow = numpy.array([30,150,200])
    upper_yellow = numpy.array([50,255,255])

    mask1 = cv2.inRange(hsv, lower_green, upper_green)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    mask3 = cv2.inRange(hsv, lower_yellow, upper_yellow)

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    M3 = cv2.moments(mask3)

    #cv2.imshow("green light mask", mask1)
    #cv2.imshow("red light mask", mask2)
    #cv2.imshow("yellow light mask", mask3)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0
    mask3[0:search_top, 0:w] = 0
    mask3[search_bot:h, 0:w] = 0
   

    if M1['m00'] > 0:#green mask
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err = cx1 - w/2
      print "green"
      print cy1
      
      if cy1 == 237:
        for x in range[5]:
          self.twist.linear.x = 0.8
          self.twist.angular.z = 0.0
          self.cmd_vel_pub.publish(self.twist)
      
    elif M2['m00'] > 0:#red mask
      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])
      err = cx2 - w/2
      print "red"
      print cy2

      if cy2 < 194:
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    elif M3['m00'] > 0:#yellow mask
      cx3 = int(M3['m10']/M3['m00'])
      cy3 = int(M3['m01']/M3['m00'])
      err = cx3 - w/2
      print "yellow"
      print cy3
      
      if cy3 < 215:
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
   
    else:
       self.lane_follow.lane_callback(msg)

    # Display masking on a window entitled traffic Image status window
    # cv2.imshow("traffic Image", image)
    # cv2.imshow("traffic Image status", crop)
    cv2.waitKey(3)

def main(args):

  detect_lights()
  rospy.init_node('traf_node', anonymous = True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
