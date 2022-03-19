#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(
        "현재상태 : {}".format(
            data.status.text
        )
    )
def main():
    while not rospy.is_shutdown():
        rospy.init_node('talkresult', anonymous=True)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult , callback)
        rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
