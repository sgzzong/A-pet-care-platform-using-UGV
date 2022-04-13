#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
import playsound
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import sys, select, os
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
rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
speed = Twist()
while not rospy.is_shutdown():
    key = getKey()
    if key == 'w' :
        speed.linear.x += 0.01
    if key == 'a' :
        speed.angular.z += 0.01
    if key == 's' :
        speed.linear.x = 0
        speed.angular.z = 0
    if key == 'd' :
        speed.angular.z += -0.01
    if key == 'x' :
        speed.linear.x += -0.01
    if key == 'u' :
        speed.linear.x = 0
        speed.angular.z = 0.26
    if key == 'i' :
        speed.linear.x = 0.2
        speed.angular.z = 0.26
    if key == 'q' :
        stop = 1
        speed.linear.x = 0.26
        speed.angular.z = 0.26
    if key == 'e' :
        stop = 1
        speed.linear.x = 0.26
        speed.angular.z = -0.26
    if key == 'z' :
        stop = 1
        speed.linear.x = -0.26
        speed.angular.z = 0.26
    if key == 'c' :
        stop = 1
        speed.linear.x = -0.26
        speed.angular.z = 0.26
    if key == 'p':
        playsound.playsound('sample.wav')
    else:
        if (key == '\x03'):
            break
    if stop >= 1:
        stop += 1
        if(stop > 50):
            stop = 0
            speed.linear.x = 0
            speed.angular.z = 0
    print("current",speed.linear.x,"angle",speed.angular.z)
    pub.publish(speed)