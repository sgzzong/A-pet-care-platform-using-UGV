#!/usr/bin/env python
# -*- coding: utf-8- -*-
import rospy
import roslib
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Int32,String
from actionlib_msgs.msg import GoalStatusArray,GoalID
from tf.transformations import quaternion_from_euler
from darknet_ros_msgs.msg import BoundingBoxes
from sound_play.libsoundplay import SoundClient
import math
from playsound import playsound
from std_srvs.srv import Empty, EmptyRequest
import time
# from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
# position = [[-2.3,2.5,1.0],[-4.3,2.5,1.0],[-4.3,0.7,0],[-2.3,0.3,1.0]]
# position = [[-0.5,-0.5,1],[0.5,-0.5,1],[0.5,0.5,1],[-0.5,0.5,1]]
position = [[4.5,-1,1],[4.7,-3,1],[3.3,-4.1,1],[2.9,-2.5,1]]
play = [['spin','spin_r'],
['left_forward','left_back','right_forward','right_back'],
['right_forward','left_forward','left_forward','right_forward','spin'],
['left_back','right_back','right_back','left_back','spin'],
['left_forward','right_back','left_forward','right_back','left_forward']]
angle = [4.8,4.5,2.7,0.9]
once = 1
i = 0
k = 0
p1 = -1
p2 = 1
stoping = 0
playon = 0
stop = 0
euler = 4.5 #first angle setting
app = ""
stack = 0
class move_point:
    def __init__(self):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/move_base/status', GoalStatusArray , self.talker)
        self.result = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.turn)
        self.stop = rospy.Subscriber("/warning",Int32, self.stoped)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.stoped)        
        self.Twisted = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.phone = rospy.Subscriber("/commandar_line", String, self.control)
        self.speed = Twist()
    def control(self,data):
        global stop, stack, once, play, playon,k , stoping
        print(data.data)
        if data.data != '' and once == 1:
            cancel_msg = GoalID()
            self.cancel_pub.publish(cancel_msg)
        if data.data == 'Go up' :
            print("go")
            self.speed.linear.x = 0.26
            self.Twisted.publish(self.speed)
        if data.data == 'Go left' :
            self.speed.angular.z = 0.26
            self.Twisted.publish(self.speed)
        if data.data == "Stop" :
            self.speed.linear.x = 0
            self.speed.angular.z = 0
            self.Twisted.publish(self.speed)
        if data.data == 'Go right' :
            self.speed.angular.z = -0.26
            self.Twisted.publish(self.speed)
        if data.data == 'Go down' :
            self.speed.linear.x = -0.26
            self.Twisted.publish(self.speed)
        if data.data == 'Play':
            playon = 1
        if playon == 1:
            if play[stack][k] == 'spin':
                self.speed.linear.x = 0
                self.speed.angular.z = 4
                self.Twisted.publish(self.speed)
                rospy.sleep(4.1)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("스핀 끝",k,stack)
            elif play[stack][k] == 'spin_r':
                self.speed.linear.x = 0
                self.speed.angular.z = -4
                self.Twisted.publish(self.speed)
                rospy.sleep(4.1)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("스핀 끝",k,stack)
                        
            elif play[stack][k] == 'right_forward':
                self.speed.linear.x = 0.2
                self.speed.angular.z = -0.4
                self.Twisted.publish(self.speed)
                rospy.sleep(2.5)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("우 전방",k,stack)
            elif play[stack][k] == 'left_forward':
                self.speed.linear.x = 0.2
                self.speed.angular.z = 0.4
                self.Twisted.publish(self.speed)
                rospy.sleep(2.5)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("좌 전방",k,stack)

            elif play[stack][k] == 'left_back':
                self.speed.linear.x = -0.2
                self.speed.angular.z = -0.4
                self.Twisted.publish(self.speed)
                rospy.sleep(2.5)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("좌 후방",k,stack)
            elif play[stack][k] == 'right_back':
                self.speed.linear.x = -0.2
                self.speed.angular.z = 0.4
                self.Twisted.publish(self.speed)
                rospy.sleep(2.5)
                k += 1
                if k == len(play[stack]):
                    k = 0
                    stack += 1
                    playon = 0
                self.speed.linear.x = 0
                self.speed.angular.z = 0
                self.Twisted.publish(self.speed)
                print("우 후방",k,stack)
            if stack == len(play):
                stack = 0
        if data.data == 'Feed' :
            rospy.wait_for_service('/motor_deg_a')
            service_client = rospy.ServiceProxy("/motor_deg_a", Empty)
            request_srv = EmptyRequest()
            result = service_client(request_srv)
            print(result)
            # rospy.sleep(3)
            # self.speed.linear.x = -0.15
            # self.Twisted.publish(self.speed)
            # rospy.sleep(1.5)
            # self.speed.linear.x = 0
            # self.Twisted.publish(self.speed)
            pass
        if data.data == 'Call':
            sound_client = SoundClient()
            rospy.sleep(1)
            sound = sound_client.waveSound('/home/gilljong/catkin_ws/src/my_package/src/다올아.mp4')
            sound.play()
        if data.data == 'Wait' :
            sound_client = SoundClient()
            rospy.sleep(1)
            sound = sound_client.waveSound('/home/gilljong/catkin_ws/src/my_package/src/기다려.mp4')
            sound.play()
        if data.data == 'Sit' :
            sound_client = SoundClient()
            rospy.sleep(1)
            sound = sound_client.waveSound('/home/gilljong/catkin_ws/src/my_package/src/앉아.mp4')
            sound.play()
        if data.data == 'Find Pet' :
            once = 0
            stoping = 0
    def stoped(self,data):
        global stoping
        for box in data.bounding_boxes:
            if box.Class == "dog" or box.Class == "teddy bear": 
                stoping += 1
                if(stoping == 4):
                    cancel_msg = GoalID()
                    self.cancel_pub.publish(cancel_msg)
                    print("강아지 발견")
                    # if box.xmin > 600:
                    #     self.speed.angular.z = -0.26
                    #     self.Twisted.publish(self.speed)
                    #     if box.xmin > 600 and box.xmin < 800:
                    #         rospy.sleep(0.5)
                    #     if box.xmin > 800:
                    #         rospy.sleep(1)
                    #     self.speed.angular.z = 0
                    #     self.Twisted.publish(self.speed)
                    # elif box.xmax < 600:
                    #     self.speed.angular.z = 0.26
                    #     self.Twisted.publish(self.speed)
                    #     if box.xmax < 600 and box.xmax > 400:
                    #         rospy.sleep(0.5)
                    #     if box.xmax < 400:
                    #         rospy.sleep(1)
                    #     self.speed.angular.z = 0
                    #     self.Twisted.publish(self.speed)
    def turn(self,data):
        global once,i
        if data.status.text == "Goal reached.":
            once = 0
            if(i < 4):
                i += 1
            if(i == 4):
                i = 0
            print(i)
    def talker(self,data):
        global position, once, i, k, stoping, euler, angle
        if once == 0:
            print("GOGO")
            goal = PoseStamped()
            goal.header.frame_id="map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = position[i][0]
            goal.pose.position.y = position[i][1]
            goal.pose.position.z = 0   
            q = quaternion_from_euler(0, 0, angle[i]) # 90 -> 1.57
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            self.pub.publish(goal)
            k += 1
            if k == 2:
                once = 1
                k = 0
        rospy.sleep(0.5)
    def angle(self,position):
        global p1,p2,euler
        x1 = abs(position[p1][0] - position[p2][0])
        x2 = abs(position[p1][1] - position[p2][1])
        angle = math.atan2(x1,x2)/(2*math.pi)*360*0.01744
        if position[p1][0] <= position[p2][0] and position[p1][1] <= position[p2][1]:#좌측 하단
            pass
            print("세번째 포인트")
        if position[p1][0] >= position[p2][0] and position[p1][1] <= position[p2][1]:#좌측 상단
            angle = 3.14 - angle
            print("네번째 포인트") 
        if position[p1][0] >= position[p2][0] and position[p1][1] >= position[p2][1]:#우측 상단
            angle = 3.14 + angle
            print("첫번째 포인트")
            pass
        if position[p1][0] <= position[p2][0] and position[p1][1] >= position[p2][1]:#우측 하단
            angle = 6.28 - angle
            print("두번째 포인트")
        p1 += 1
        p2 += 1
        if p1 > 2:
            p1 = -1
        if p2 > 3:
            p2 = 0
        euler = angle
def run():
    rospy.init_node('talker',anonymous=True)
    print("나 실행됐따 ㅎ!")
    move_turtlebot3 = move_point()
    rospy.spin()
if __name__ == '__main__':
    run()
