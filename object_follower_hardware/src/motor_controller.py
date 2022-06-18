#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy

class Robot_Controller:
    #initialised values
    def __init__(self):

        rospy.init_node('controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.Joy_callback)
        
        self.ya_axis = 0
        self.yb_axis = 0
        self.velocity_msg = Twist()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(19, GPIO.OUT)
        self.lapwm = GPIO.PWM(12, 1000)
        self.lbpwm = GPIO.PWM(13, 1000)
        self.rapwm = GPIO.PWM(18, 1000)
        self.rbpwm = GPIO.PWM(19, 1000)

        self.lapwm.start(0)
        self.lbpwm.start(0)
        self.rapwm.start(0)
        self.rbpwm.start(0)

    def provide_pwm(self):
        if self.ya_axis >= 0:
            self.lapwm.ChangeDutyCycle(abs(self.ya_axis) * 100)
            self.lbpwm.ChangeDutyCycle(0)
        else:
            self.lapwm.ChangeDutyCycle(0)
            self.lbpwm.ChangeDutyCycle(abs(self.ya_axis) * 100)

        if self.yb_axis >= 0:
            self.rapwm.ChangeDutyCycle(abs(self.ya_axis) * 100)
            self.rbpwm.ChangeDutyCycle(0)
        else:
            self.rapwm.ChangeDutyCycle(0)
            self.rbpwm.ChangeDutyCycle(abs(self.ya_axis) * 100)
            
    
    def Joy_callback(self, data):
        axes = data.axes
        self.ya_axis = axes[1]
        self.yb_axis = axes[4]
        print("a = " + str(self.ya_axis) + "  " + "b = " + str(self.yb_axis))
        self.provide_pwm()



if __name__ == "__main__":
    Robot = Robot_Controller()
    rospy.spin()