#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


vel= Twist()

def vel_data(data):
	global vel	
	rospy.loginfo("linear_velocity:%f",data.twist.twist.linear.x)
	vel.linear.x = data.twist.twist.linear.x
	vel.linear.y = data.twist.twist.linear.y
	vel.linear.z = data.twist.twist.linear.z
	vel.angular.x = data.twist.twist.linear.x
	vel.angular.y = data.twist.twist.linear.y
	vel.angular.z = data.twist.twist.linear.z
	
rospy.init_node("Vel_subs",anonymous=True)
vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)

while True:
	rospy.Subscriber("/odom_data_quat",Odometry,callback=vel_data)
	vel_pub.publish(vel)

