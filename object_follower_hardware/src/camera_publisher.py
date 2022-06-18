#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        rospy.init_node('Camera_Publisher')

        self.pub = rospy.Publisher('video_frames', Image, queue_size=10)
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.rate = rospy.Rate(10)
        self.publish_message()
    
    def publish_message(self):
        while not rospy.is_shutdown():
   
            ret, frame = self.cap.read()

            if ret == True:
              #rospy.loginfo('publishing video frame')
              self.pub.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

            self.rate.sleep()


if __name__ == '__main__':
    Camera = CameraPublisher()
