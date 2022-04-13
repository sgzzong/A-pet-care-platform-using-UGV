#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32
from actionlib_msgs.msg import GoalStatusArray,GoalID
from tf.transformations import quaternion_from_euler
import math
# from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
position = [[1.0,-1.0,1.0],[4.0,-1.0,-1.0],[4.0,-4.0,-1.0],[1.0,-4.0,-1.0]]
once = 0
i = 0
k = 1
p1 = -1
p2 = 1
stoping = 4
euler = 1.57 #first angle setting
class move_point:
    def __init__(self):
        global euler
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/move_base/status', GoalStatusArray , self.talker)
        self.result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.turn)
        self.stop = rospy.Subscriber("/warning",Int32, self.stoped)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    def stoped(self,data):
        global stoping
        stoping = data.data
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)
    def turn(self,data):
        global once,i
        # print("open",data.status.text)
        if data.status.text == "Goal reached.":
            once = 0
            if(i < 4):
                i += 1
            if(i == 4):
                i = 0
            self.angle(position)
            print(i)
    def talker(self,data):
        global position, once, i, k, stoping, euler
        if once == 0:
            goal = PoseStamped()
            goal.header.frame_id="map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.y = position[i][0]
            goal.pose.position.x = position[i][1]
            goal.pose.position.z = 0   
            q = quaternion_from_euler(0, 0, euler) # 90 -> 1.57
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            self.pub.publish(goal)
            k += 1
            if k == 5:
                once = 1
                k = 0     
        rospy.sleep(1)
    def angle(self,position):
        global p1,p2,euler
        x1 = abs(position[p1][0] - position[p2][0])
        x2 = abs(position[p1][1] - position[p2][1])
        angle = math.atan2(x1,x2)/(2*math.pi)*360*0.01744
        if position[p1][0] <= position[p2][0] and position[p1][1] <= position[p2][1]:
            angle = 3.14 - angle  
        if position[p1][0] >= position[p2][0] and position[p1][1] <= position[p2][1]:
            pass
        if position[p1][0] >= position[p2][0] and position[p1][1] >= position[p2][1]:
            angle = 6.28 - angle
        if position[p1][0] <= position[p2][0] and position[p1][1] >= position[p2][1]:
            angle = 3.14 + angle
        p1 += 1
        p2 += 1
        if p1 > 2:
            p1 = -1
        if p2 > 3:
            p2 = 0
        euler = angle
def run():
    rospy.init_node('talker',anonymous=True)
    move_turtlebot3 = move_point()
    rospy.spin()
if __name__ == '__main__':
    run()
        