#!/usr/bin/env python3
#-*-coding:utf-8-*-
import rospy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray,GoalID
import cv2 #rostopic list 카메라의 데이터 타입을 확인 
from cv_bridge import CvBridge #opencv와 로스 이미지 사이에는 차이가 있으므로 opencv 이미지 파일형태로 변환시켜주는 모듈
from geometry_msgs.msg import Twist

class receiver:
    def __init__(self):
        #self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.callback)
        self.stop_pub = rospy.Publisher("/warning",Int32, queue_size = 5)
        #self.Image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.img)
        #self.odom_sub = rospy.Subscriber("/odom", Odometry, self.state)
        self.cvbridge = CvBridge()
        self.Twisted = rospy.Publisher("/cmd_vel",Twist, queue_size = 5)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.speed = Twist()
    def state(self,data):
        print(f"현재 위치 : {data.pose.pose.position.x} {data.pose.pose.position.y} {data.pose.pose.position.z}") #pose(위치)
        print(f"현재 위치 : {data.twist.twist.linear.x} {data.twist.twist.linear.y} {data.twist.twist.linear.z}") #twist(각속도)

    def img(self,data):
        frame = self.cvbridge.imgmsg_to_cv2(data,"bgr8")
        resized_frame = cv2.resize(frame, (640, 480))
        cv2.imshow("frame",resized_frame)
        cv2.waitKey(1) #이미지 창을 띄우고 기다리는것 (1ms) imshow와 세트임

    def callback(self,data): #rosmsg show sensor_msgs/LaserScan에 나온 변수들을 사용가능
        print(data.ranges[300])
        if data.ranges[300] < 1.0:
            print("warning")
            #self.stop_pub.publish(1)
        else:
            print("safe")
            #self.stop_pub.publish(0)
    def go(self):
        # cancel_msg = GoalID()
        # self.cancel_pub.publish(cancel_msg)
        while(1):
            print("go")
            self.speed.linear.x = 0.26
            self.Twisted.publish(self.speed)
        # if data.data == 'Go left' :
        #     self.speed.angular.z = 0.26
        #     self.Twisted.publish(self.speed)
        # if data.data == "Stop" :
        #     self.speed.linear.x = 0
        #     self.speed.angular.z = 0
        #     self.Twisted.publish(self.speed)
        # if data.data == 'Go right' :
        #     self.speed.angular.z = -0.26
        #     self.Twisted.publish(self.speed)
        # if data.data == 'Go down' :
        #     self.speed.linear.x = -0.26
        #     self.Twisted.publish(self.speed)

        # self.stop_pub.publish(1)
def run():
    rospy.init_node("lidar_camera", anonymous=True) 
    receiver_a = receiver()
    receiver_a.go()
    rospy.spin()
    
if __name__ == "__main__":
    run()