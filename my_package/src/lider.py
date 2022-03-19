#!/usr/bin/env python 
#-*-coding:utf-8-*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Header
class lidar_receiver:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.callback)
        self.stop_pub = rospy.Publisher("/warning",Int32, queue_size = 5)
    def callback(self,data): #rosmsg show sensor_msgs/LaserScan에 나온 변수들을 사용가능
        # for i in range(len(data.ranges)):
        #     angle = data.angle_min + i * data.angle_increment #각도를 구하기 라디안 기준 
        #     if angle > -0.01 and angle < 0.01:
        #         print("{}th index range = {}".format(i,data.ranges[i]))
        # if data.ranges[360] == 0.2: #m단위이므로 2cm
        #     print("warning")
        #     self.stop_pub.publish(1)
        # else:
        #     print("safe")
        #     self.stop_pub.publish(0)
        # for box in data.bounding_boxes:
        #     if box.Class == "person":
        #         rospy.loginfo(
        #             "정확성 : {} 객체: {}".format(
        #                 box.probability, box.Class
        #             )
        #         )
        print(data.ranges[300])
        if data.ranges[300] < 1.0:
            print("warning")
            self.stop_pub.publish(1)
        else:
            print("safe")
            self.stop_pub.publish(0)
def run():
    rospy.init_node("lidar_sub", anonymous=True) # 3개를 동시에 킬때 _pub 앞에 부분의 이름을 수정해야댐
    lidar_receiver_a = lidar_receiver()
    rospy.spin()
if __name__ == "__main__":
    run()