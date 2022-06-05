#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
import playsound
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import sys, select, os
from std_msgs.msg import Int32,String
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
stop = 0
x = 0.0
y = 0.0 
theta = 0.0
def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

class contorl_move:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.phone = rospy.Subscriber("/commandar_line", String, self.control)
    def control(self,data):
        global stop
        print(type(data))
        speed = Twist()      
        if data == 'Go up' :
            speed.linear.x += 0.01
        if data == 'Go left' :
            speed.angular.z += 0.01
        if data == 'Stop' :
            speed.linear.x = 0
            speed.angular.z = 0
        if data == 'Go down' :
            speed.angular.z += -0.01
        if data == 'Go right' :
            speed.linear.x += -0.01
        if data == 'Play' :
            speed.linear.x = 0
            speed.angular.z = 0.26
        if data == 'i' :
            speed.linear.x = 0.2
            speed.angular.z = 0.26
        if data == 'q' :
            stop = 1
            speed.linear.x = 0.26
            speed.angular.z = 0.26
        if data == 'e' :
            stop = 1
            speed.linear.x = 0.26
            speed.angular.z = -0.26
        if data == 'z' :
            stop = 1
            speed.linear.x = -0.26
            speed.angular.z = 0.26
        if data == 'c' :
            stop = 1
            speed.linear.x = -0.26
            speed.angular.z = 0.26
        if data == 's' :
            stop = 1
            speed.linear.x = -0.26
            speed.angular.z = 0.26    
        if data == 'Call':
            playsound.playsound('sample.wav')
        if stop >= 1:
            stop += 1
            if(stop > 50):
                stop = 0
                speed.linear.x = 0
                speed.angular.z = 0
        print("current",speed.linear.x,"angle",speed.angular.z)
        rospy.sleep(5)
        self.pub.publish(speed)
def run():
    rospy.init_node("speed_controller")
    move_control = contorl_move()
    rospy.spin()
if __name__ == '__main__':
    run()