#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

vid = cv2.VideoCapture(0)

class opencv_publisher:
    def __init__(self):
         
        self.pub = rospy.Publisher('/rrbot/camera1/image_raw', Image,queue_size=10)
        self.bridge = CvBridge()

    def publish(self):
        
  
        while(True):
            ret, frame = vid.read()
            
            
            if ret :
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                self.pub.publish(image_message)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
   



def main():
  rospy.init_node("Opencv_Publisher",anonymous=True)
  
  op=opencv_publisher()
  op.publish()
    
  try:
    rospy.spin()
    
  except:
    print("error")


  cv2.destroyAllWindows()


main()