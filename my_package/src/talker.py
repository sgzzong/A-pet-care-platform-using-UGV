#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int32
position = [[4.0,-6.5,1.0],[4.0,6.5,1.0]]
once = 0
i = 1
k = 1
stoping = 4
class move_point:
    def __init__(self):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/move_base/status', GoalStatusArray , self.talker)
        self.result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.turn)
        self.stop = rospy.Subscriber("/warning",Int32, self.stoped)
        self.go = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    def stoped(self,data):
        global stoping
        stoping = data.data
    def turn(self,data):
        global once,i
        print("open",data.status.text)
        if data.status.text == "Goal reached.":
            if i == 1:
                i -= 1
            if i == 0:     
                i += 1
    def talker(self,data):
        global position, once, i, k, stoping
        self.rate = rospy.Rate(10)
        if stoping != 1:
            goal = PoseStamped()
            goal.header.frame_id="map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.y = position[i][0]
            goal.pose.position.x = position[i][1]
            goal.pose.orientation.w = position[i][2]
            self.pub.publish(goal)
            print(i,once)
            rospy.sleep(1)
        elif stoping == 1:
            move = Twist()
            move.linear.x=0.0
            move.angular.z=0.0
            print("stop",stoping)
            self.go.publish(move)
def run():
    rospy.init_node('talker',anonymous=True)
    move_turtlebot3 = move_point()
    rospy.spin()
if __name__ == '__main__':
    run()
        