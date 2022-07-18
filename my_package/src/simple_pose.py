#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)                                                      
        group_names = robot.get_group_names()
        self.move_group = move_group
    def go_to_pose_goal(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = 0.5
        pose_goal.position.y = 0
        pose_goal.position.z = 0.2 #static

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
def main():
    tutorial = MoveGroupPythonIntefaceTutorial()
    tutorial.go_to_pose_goal()
if __name__ == '__main__':
  main()