#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
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
    self.velocity_msg.linear.y = 0
    self.velocity_msg.linear.z = 0
    self.velocity_msg.angular.x = 0
    self.velocity_msg.angular.y = 0
    self.radius_threshold=130
    self.buffer=20
  
  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.control_loop()      
      
    except CvBridgeError as e:
      print(e)
      self.move(0,0)
      
  def control_loop(self):
    sc = SampleClass()
    result=sc.fun(self.cv_image)
    x_length=result[0].shape[0]

    y=result[0].shape[0]
    x =int(x_length/2)
    cv2.line(result[1],(x,0),(x,800),(255,0,0),2)
    if(result[3]== None and result[2]==None):
      self.move(0,1)
    else: 
      if(result[3]<=self.radius_threshold):
        if result[2][0]>(x_length/2+self.buffer):
          self.move(1,-1)
          cv2.putText(result[0],"Right",(x-60,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
          cv2.putText(result[0],"Go Forward",(x-100,750),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
  
        elif result[2][0]<(x_length/2-self.buffer):
         self.move(1,1) 
         cv2.putText(result[0],"Left",(x-50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
         cv2.putText(result[0],"Go Forward",(x-100,750),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
        
        else:
          self.move(1,0)
          cv2.putText(result[0],"Center",(x-75,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
          cv2.putText(result[0],"Go Forward",(x-100,750),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
  
      else:
        if result[2][0]>(x_length/2+self.buffer):
          self.move(0,-1)
          cv2.putText(result[0],"Right",(x-60,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
          
  
        elif result[2][0]<(x_length/2-self.buffer):
          self.move(0,1) 
          cv2.putText(result[0],"Left",(x-50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA) 
              
        else:
         self.move(0,0)
         cv2.putText(result[0],"Stop",(x-50,750),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
    
    cv2.imshow("Frame",result[0])
    cv2.imshow("Mask",result[1])
    cv2.waitKey(1)
    

  def move(self, linear, angular):
    self.velocity_msg.linear.x = linear
    self.velocity_msg.angular.z = angular
    self.pub.publish(self.velocity_msg)
    

def main():
  rospy.init_node("Obj_follower",anonymous=True)
  of=obj_follower()
  try:
    rospy.spin()
    
  except:
    print("error")


  cv2.destroyAllWindows()


main()

	
