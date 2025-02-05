#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

import json
import os
import math


class NaviBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def calcTwist(self,th):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def go_goalpoint(self,r):
        goal_point = [[-0.87,0,math.radians(0)], \
                      [0,0.88,math.radians(270)], \
                      [0.87,0,math.radians(180)], \
                      [0,-0.88,math.radians(90)]]

        out_way = [
            [-0.71, 0.71, math.radians(45)], #[0] home <-> left out
            [0.71, 0.71, math.radians(315)], #[1] left <-> enemy out
            [0.71, -0.71, math.radians(225)], #[2] enemy <-> right out
            [-0.71, -0.71, math.radians(135)], #[3] right <-> home out
        ]
        for G,W in zip(goal_point,out_way):
            self.setGoal(G[0],G[1],G[2])
            theta = [-2,2]
            for th,n in zip(theta,range(1,3)):
                for i in range(0,4*n):
                    twist = self.calcTwist(th)
                    self.vel_pub.publish(twist)
                    r.sleep()
            twist = self.calcTwist(th=0)
            self.vel_pub.publish(twist)

            self.setGoal(W[0], W[1], W[2])

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        while not rospy.is_shutdown():
            self.go_goalpoint(r)



if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
