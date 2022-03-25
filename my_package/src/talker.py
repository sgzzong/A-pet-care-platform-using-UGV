#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32
from actionlib_msgs.msg import GoalStatusArray,GoalID
from tf.transformations import quaternion_from_euler
# from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
position = [[1.0,-1.0,1.0],[4.0,-1.0,-1.0],[4.0,-4.0,-1.0],[1.0,-4.0,-1.0]]
orient = [[0, 0, 0, 1],[0, 0, 0.707, 0.707],[0.707, 0, 0, 0.707],[0, -0.707, 0, 0.707]]
once = 0
i = 0
k = 1
stoping = 4
class move_point:
    def __init__(self):
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
            print(i)
    def talker(self,data):
        global position, once, i, k, stoping
        if once == 0:
            goal = PoseStamped()
            goal.header.frame_id="map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.y = position[i][0]
            goal.pose.position.x = position[i][1]
            goal.pose.position.z = 0
            # q = quaternion_from_euler(0, 0, goal.yaw)
            # goal.pose.orientation.w = position[i][2]
            goal.pose.orientation.x = orient[0][0]
            goal.pose.orientation.y = orient[0][1]
            goal.pose.orientation.z = orient[0][2]
            goal.pose.orientation.w = orient[0][3]
            self.pub.publish(goal)
            k += 1
            if k == 5:
                once = 1
                k = 0     
        rospy.sleep(1)
def run():
    rospy.init_node('talker',anonymous=True)
    move_turtlebot3 = move_point()
    rospy.spin()
if __name__ == '__main__':
    run()
        