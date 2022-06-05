#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
name = "?"
xmin,ymin,xmax,ymax = 0,0,0,0
class lidar_receiver:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan, self.callback)
        self.stop_pub = rospy.Publisher("/warning",Int32, queue_size = 5)
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.call)        
    def callback(self,data): 
        print(data.ranges[300])
        if data.ranges[300] < 1.0:
            print("warning")
            self.stop_pub.publish(1)
        else:
            print("safe")
            self.stop_pub.publish(0) #0이 발행되고 있을 때 cancel 노드가 실행되지 않는건가 ?
        print("Class : ",name,xmin,xmax,ymin,ymax)
    def call(self,data):
        for box in data.bounding_boxes:
            if box.Class == "person":
                global name,xmin,ymin,xmax,ymax
                name = box.Class
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                print(xmin, xmax)
def run():
    rospy.init_node("lidar_sub", anonymous=True) # 3개를 동시에 킬때 _pub 앞에 부분의 이름을 수정해야댐
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == '__main__':
    try :
        run()
    except rospy.ROSInterruptException:
        pass
            #     rospy.loginfo(
            #         "정확성 : {} 객체: {}".format(
            #             box.probability, box.Class
            #         )
            #     )
# def callback(data):
#     for box in data.bounding_boxes:
#         rospy.loginfo(
#             "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}, Class: {}".format(
#                 box.xmin, box.xmax, box.ymin, box.ymax, box.Class
#             )
#         )
# def main():
#     while not rospy.is_shutdown():
#         rospy.init_node('listener', anonymous=True)
#         rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.call)
#         rospy.spin()
#         print("obj = ",name)