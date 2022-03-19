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
    def callback(self,data):
        for i in range(0,360):
            if data.ranges[i] < 0.5:
                print("%d 각도", i)
                self.stop_pub.publish(1)
def run():
    rospy.init_node("lidar_sub", anonymous=True) # 3개를 동시에 킬때 _pub 앞에 부분의 이름을 수정해야댐
    lidar_receiver_a = lidar_receiver()
    rospy.spin()
if __name__ == "__main__":
    run()