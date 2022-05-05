#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
#from Robot_State import Bot_State
#from WaypointManager import WaypointManager
#from OdomSubscriber import OdomSubscriber
import numpy as np
from Object_Tracking import SampleClass
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image


class obj_follower:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub =rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.velocity_msg = Twist()
    
  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.control_loop()      
      
    except CvBridgeError as e:
      print(e)

  def control_loop(self):
    sc = SampleClass()
    result=sc.fun(self.cv_image)
    x_length=result[0].shape[0]
    if result[2][0]>(x_length/2+20):
      self.velocity_msg.linear.x = 1
      self.velocity_msg.linear.y = 0
      self.velocity_msg.linear.z = 0
      self.velocity_msg.angular.x = 0
      self.velocity_msg.y = 0
      self.velocity_msg.angular.z = -1
      self.pub.publish(self.velocity_msg)
    elif result[2][0]<(x_length/2-20):
      self.velocity_msg.linear.x = 1
      self.velocity_msg.linear.y = 0
      self.velocity_msg.linear.z = 0
      self.velocity_msg.angular.x = 0
      self.velocity_msg.y = 0
      self.velocity_msg.angular.z = 1
      self.pub.publish(self.velocity_msg)
    else:
      self.velocity_msg.linear.x = 1
      self.velocity_msg.linear.y = 0
      self.velocity_msg.linear.z = 0
      self.velocity_msg.angular.x = 0
      self.velocity_msg.y = 0
      self.velocity_msg.angular.z = 0
      self.pub.publish(self.velocity_msg)


    #print("control+loop")
    #self.move(0.5,0)

  def move(self, linear, angular):
    print("move")
    self.velocity_msg.linear.x = linear
    self.velocity_msg.angular.z = angular
    self.pub.publish(self.velocity_msg)
    








def main():
  rospy.init_node("Obj_follower",anonymous=True)
  of= obj_follower()
  try:
    rospy.spin()
    
  except:
    print("error")


  cv2.destroyAllWindows()


main()

	