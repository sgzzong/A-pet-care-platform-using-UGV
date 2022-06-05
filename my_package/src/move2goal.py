#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
import playsound
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import sys, select, os
from actionlib_msgs.msg import GoalStatusArray,GoalID

from std_msgs.msg import Int32,String
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
stop = 0
x = 0.0
y = 0.0 
theta = 0.0
change = 0
app = ""
stack = 0
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
def we(speed):
    speed.linear.x = 0.2
    speed.angular.z = -0.4
    for i in range(3):
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
        pub.publish(speed)  
        if(i < 2):
            rospy.sleep(1)  
def wq(speed):
    speed.linear.x = 0.2
    speed.angular.z = 0.4
    for i in range(3):
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
        pub.publish(speed)
        if(i < 2):
            rospy.sleep(1)
    
def sc(speed):
    speed.linear.x = -0.2
    speed.angular.z = 0.4
    for i in range(3):
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
        pub.publish(speed)
        if(i < 2):
            rospy.sleep(1)
def sz(speed):
    speed.linear.x = -0.2
    speed.angular.z = -0.4
    for i in range(3):
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
        pub.publish(speed)
        if(i < 2):
            rospy.sleep(1)
def spin(speed):
    speed.linear.x = 0
    speed.angular.z = 4
    for i in range(3):
        rospy.sleep(1.3)
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
        pub.publish(speed)
def spin_r(speed):
    speed.linear.x = 0
    speed.angular.z = -4
    pub.publish(speed)
    for i in range(3):
        rospy.sleep(1.3)
        if(i == 2):
            speed.linear.x = 0
            speed.angular.z = 0
            pub.publish(speed)
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    turtlebot3_model = rospy.get_param("model", "waffle")
    speed = Twist()
    
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w' :
            speed.linear.x += 0.1
        if key == 'a' :
            speed.angular.z += 0.1
        if key == 's' :
            speed.linear.x = 0
            speed.angular.z = 0
        if key == 'd' :
            speed.angular.z += -0.1
        if key == 'x' :
            speed.linear.x += -0.1
        if key == 'u' :
            speed.linear.x = 0
            speed.angular.z = 3
        if key == 'i' :
            stack += 1
            if stack == 1: spin(speed)
            if stack == 2: wq(speed); sz(speed); we(speed); sc(speed)
            if stack == 3: we(speed); wq(speed); wq(speed); we(speed); spin(speed)
            if stack == 4: sz(speed); sc(speed); sc(speed); sz(speed); spin(speed)
            if stack == 5: wq(speed); sc(speed); wq(speed); sc(speed); wq(speed)
            if stack == 6: spin_r(speed); spin(speed)
            if stack == 7: stack = 0

        if key == 'q' :
            wq(speed)
        if key == 'e' :
            we(speed)
        if key == 'z' :
            sz(speed)
        if key == 'c' :
            sc(speed)
        if key == 'b': #walts
            change = 1
            stop = 1
            speed.linear.x = 0.26
            speed.angular.z = 0.26
        if key == 'g': #walts
            spin(speed)
            
            # we(speed)
        # if key == 'c' :
        #     cancel_msg = GoalID()
        #     cancel_pub.publish(cancel_msg)
        if key == 'p':
            playsound.playsound('sample.wav')
        else:
            if (key == '\x03'):
                break
        if stop >= 1:
            stop += 1
            if(stop > 15):
                stop = 0
                bx = 0
                bz = 0
                if(change == 1):
                    bx = - speed.linear.x
                    bz = - speed.angular.z
                    change = 0
                speed.linear.x = bx
                speed.angular.z = bz
        print("current",speed.linear.x,"angle",speed.angular.z)
        pub.publish(speed)
    rospy.sleep(1)
