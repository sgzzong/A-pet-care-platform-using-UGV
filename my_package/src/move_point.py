#!/usr/bin/env python

# Author: Jhensen Ray Agni
# RBE500 Project
# Controller

# DESCRIPTION:
# The controller node utilizes the turtlebot3's odometry and lidar data in order to move through the perimeter of an indoor course. The controller allows for
# obstacle avoidance. When coupled with the turtlebot3_slam turtlebot3_slam.launch command, the turtlebot3 will map and localize using Rviz for visualization.
# The motion algorithm follows a modified Bug Algorithm 0. 

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import message_filters
import tf
import math

TEST_FLAG = 5

class Controller():

    def __init__(self):
        # define constants
        self.maxLinearVelocity = .2
        self.maxAngularVelocity = 1.2
        self.Kp_linear = 0.22
        self.Kp_angular = .5
        self.twist = Twist()
        self.distTolerance = 0.01 
        self.distSafety = .2
        self.previousAngularError = 0
        self.previousLinearError = 0
        self.modeFlag = 0
        self.entryObstacle=[]
        self.currentGoal = []
        self.frontScan = []
        self.prevPoses = []
        self.iteration = 0
        # mode 0 -> search for nearest obstacle
        # mode 1 -> move towards  nearest obstacle
        # mode 2 -> follow obstacle until able to return to goal
        # mode 3 -> Stop
        rospy.loginfo('STARTING...')
        ## Initialize nodes 
        
        self.posePub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.lidarSub = message_filters.Subscriber('/scan',LaserScan)
        self.poseSub = message_filters.Subscriber('/odom',Odometry)
        self.rate = rospy.Rate(10)
        rospy.loginfo('NODES initialized....')

        # Algorithm Start 
        while not rospy.is_shutdown():
            self.lidarSub.registerCallback(self.lidarCallback)
            self.poseSub.registerCallback(self.poseCallback)
            rospy.sleep(.5)
     
            print('Current Mode: ',self.modeFlag)
            if self.modeFlag == 0:
                # find nearest object 
                self.currentGoal = [self.goalX,self.goalY]
                self.modeFlag = 1
                
            elif self.modeFlag == 1:
                # move towards nearest object 
                self.point2point([self.goalX,self.goalY])
                self.entryObstacle=[self.goalX,self.goalY]
                print('moving... distance to goal: {}'.format(self.dist2Goal))
                if self.dist2Goal <= self.distTolerance+self.distSafety:
                    self.modeFlag = 2

            elif self.modeFlag == 2:
                # circumnavigate the perimeter of the obstacle
               self.followWall()
               if math.sqrt( (self.entryObstacle[0]-self.X)**2 + (self.entryObstacle[1]-self.Y)**2) < 0.2 and self.iteration>10:
                   self.modeFlag = 3
            elif self.modeFlag ==3:
                # when repeat, exit 
                self.stop()
                rospy.loginfo('Task Complete... ')
            
            self.rate.sleep()
        
    # Callbacks 

    def lidarCallback(self,msg):
        self.scans = msg.ranges
        if self.modeFlag == 0:
            self.findNearestObstacle()   
        self.frontScan = list(self.scans[338:359])
        self.frontScan.append(self.scans[0:23])
        self.frontLeftScan = list(self.scans[24:44])
        self.leftScan = list(self.scans[45:89])
        self.rightScan = list(self.scans[180:314])
        self.frontRightScan = list(self.scans[315:337])


    def poseCallback(self,msg):
        self.X = msg.pose.pose.position.x
        self.Y = msg.pose.pose.position.y
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (r,p,yaw)= tf.transformations.euler_from_quaternion(quat)
        self.theta = yaw

    # Action methods

    def findNearestObstacle(self):

        self.nearestObstacleDist = 100000  # set to an large number
        for angle in range(len(self.scans)):
            if(self.scans[angle] < self.nearestObstacleDist):
                self.nearestObstacleDist = self.scans[angle]
                self.nearestObstacleHeading = angle*math.pi/180
        # if nearest obstacle is 100000, set closest angle at 0 degrees 
        if self.nearestObstacleDist > 4:
            self.nearestObstacleDist = 3.5
            self.nearestObstacleHeading = 0

        # convert from body frame to world frame
        self.goalX = self.nearestObstacleDist * math.cos(self.nearestObstacleHeading)
        self.goalY = self.nearestObstacleDist * math.sin(self.nearestObstacleHeading)

    def followWall(self):
        # gather sensor data 
        self.frontRight = min(self.frontRightScan)
        self.frontLeft = min(self.frontLeftScan)
        self.front = min(self.frontScan)
        print(self.frontRight,self.front,self.frontLeft)
        # store positions
        self.prevPoses.append([self.X,self.Y])
        # set distance tolerance
        d = .7
        self.iteration += 1
        if self.front > d and self.frontLeft > d and self.frontRight>d:
            # if not within range, find wall
            self.twist.linear.x = 0.2
            self.twist.angular.z = -.3
            print('Current action: finding wall case1')
        elif self.front < d and self.frontLeft > d and self.frontRight > d:
            # if object is directly ahead, turn left
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0

            print('Current action: turning Left case2')

        elif self.front > d and self.frontLeft > d and self.frontRight < d:
            # if object is to the right, follow wall
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            print('Current action: following wall case3')
        elif self.front > d and self.frontLeft < d and self.frontRight > d:
            self.twist.linear.x = 0.2
            self.twist.angular.z = -.5
            print('Current action: finding wall case4')
        elif self.front < d and self.frontLeft > d and self.frontRight < d:
            self.twist.angular.z = .5
            self.twist.linear.x = 0
            print('Current action: turning Left case5')
 
        elif self.front < d and self.frontLeft < d and self.frontRight > d:
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0 
            print('Current action: turning Left case6')

        elif self.front < d and self.frontLeft < d and self.frontRight < d: 
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0
            print('Current action: turning Left case7')

        elif self.front > d and self.frontLeft < d and self.frontRight < d:
            self.twist.linear.x = 0.1
            self.twist.angular.z = -.5
            print('Current action: finding wall case8')

        else:
            rospy.logerr('ERROR: sensor reading failure')

        self.posePub.publish(self.twist)


    ## movement 
            

    def point2point(self,goal):
        # MOVES TURTLEBOT3 from current position to goal

        # Linear Controller
        self.xError = goal[0] - self.X
        self.yError = goal[1] - self.Y
        self.dist2Goal = math.sqrt(self.xError**2 + self.yError**2) - self.distSafety
        self.vel = min(self.Kp_linear * self.dist2Goal,self.maxLinearVelocity)
    
        # Angular Controller
        # determine heading
        self.desiredHeading = math.atan2(goal[1]-self.Y, goal[0]-self.X)
        self.headingError = self.desiredHeading - self.theta
        self.headingError = math.atan2(math.sin(self.headingError),math.cos(self.headingError))
        # angular velocity
        self.angularVel =self.Kp_angular*self.headingError

        # if moving towards obstacle, have safty tolerance
        if self.modeFlag == 1:
            if self.dist2Goal >=  self.distTolerance:
                self.twist.linear.x =  self.vel
                self.twist.angular.z = self.angularVel
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0

        self.posePub.publish(self.twist)
        self.rate.sleep()

    def stop(self):
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        self.posePub.publish(self.twist)
        self.rate.sleep()

    def turnLeft90deg(self):
        self.twist.angular.z = math.pi / 4
        self.posePub.publish(self.twist)
        
        rospy.sleep(2)
        self.stop()
        self.rate.sleep()
        



    def distance(self,x,y):
        dist = math.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2 )

        return dist

        
if __name__ == '__main__':
    
    rospy.init_node('turtebot3_controller',anonymous=False)
    m = Controller()