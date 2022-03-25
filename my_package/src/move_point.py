#!/usr/bin/env python
# license removed for brevity

from re import I, X
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus, GoalID
from std_msgs.msg import Int32
position = [[-1.0,1.0,1.0],[-4.0,1.0,1.0],[-4.0,4.0,1.0],[-1.0,4.0,1.0]]
i = 0
stoping = 0
def stoped(data):
    global stoping
    stoping = data.data   
def movebase_client(x,y):
    global i
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    client.wait_for_server()
    if stoping == 1:
        print(" Fail")
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
        return False
    else:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        print("start")
        client.send_goal(goal)
        success = client.wait_for_result()
        state = client.get_state()
        print("end")
        if success and state == GoalStatus.SUCCEEDED:
            print(" Complete")
            i += 1
            if(i > 3):
                i = 0
    
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    #     return client.get_result()
def main():
    rospy.Subscriber("/warning",Int32,stoped)
    while not rospy.is_shutdown():
            rospy.init_node('movebase_client_py')
            movebase_client(position[i][0],position[i][1])
            rospy.sleep(1)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
