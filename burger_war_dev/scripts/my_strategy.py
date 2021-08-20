#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf

import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

#import rosre

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

import json
import random
import re
import os
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import String

class NaviBot():
    def __init__(self):
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.target_id_sub = rospy.Subscriber('war_state', String, self.get_war_state)

    path_gws = os.path.dirname(__file__) + '/get_war_state.json'
    path_ts = os.path.dirname(__file__) + '/target_state.json'
    blue_pos = {"n":14,"num":7}
    red_pos = {"n":17,"num":1}
    distance = 4

    def get_war_state(self, data):
        delword = ['\\n','\\',' ','\n']
        joinword = ['\n','\n','\n','']

        str_data = str(data)  
        for (dw,jw) in zip(delword, joinword):
            str_data = str_data.split(dw)
            str_data = jw.join(str_data)
        json_data = str_data[6:len(str_data)-1]

        with open(self.path_gws, mode='w') as f:
            f.write(json_data)
        
        self.update_target_player()
        self.search_enemy()

    def update_target_player(self):
        coordinate = [[-2,3],[-2,2],[2,3],[2,2],[-2,-2],[-2,-3],[2,-2],[2,-3],[0,1],[1,0],[-1,0],[0,-1]]
        course = [6,5,8,9,3,2,11,12,7,10,4,1]

        with open(self.path_gws, mode='r') as f:
            json_load = json.load(f)

        with open(self.path_ts, mode='r') as f:
            json_log = json.load(f)


        status_dict = {}
        status_dict_in = {}
        target_state = []
        for n in range(18):
            target_state_in = {}
            if n > 5:
                target_state_in["num"] = course[n-6]
                target_state_in["coordinate"] = coordinate[n-6]
            target_state_in["name"] = json_load["targets"][n]["name"]
            target_state_in["player"] = json_load["targets"][n]["player"]
            target_state.append(target_state_in)
        status_dict_in["targets"] = target_state
        status_dict_in["time"] = json_load["time"]

        status_dict["now"] = status_dict_in
        status_dict["log"] = json_log["now"]

        with open(self.path_ts, mode='w') as f:
            json.dump(status_dict, f, indent=2)

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
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

    def search_enemy(self):
        with open(self.path_ts, mode='r') as f:
            json_load = json.load(f)
        
        for n in range(18):
            if json_load["now"]["targets"][n]["player"] == "r":
                if (json_load["log"]["targets"][n]["player"] == "n") or (json_load["log"]["targets"][n]["player"] == "b"):
                    self.red_pos["num"] = json_load["now"]["targets"][n]["num"]
                    self.red_pos["n"] = n
                    print("red got:",self.red_pos["num"],json_load["now"]["targets"][n]["name"])

            if json_load["now"]["targets"][n]["player"] == "b":
                if (json_load["log"]["targets"][n]["player"] == "n") or (json_load["log"]["targets"][n]["player"] == "r"):
                    self.blue_pos["num"] = json_load["now"]["targets"][n]["num"]
                    self.blue_pos["n"] = n
                    print("blue got:",self.blue_pos["num"],json_load["now"]["targets"][n]["name"])
            if n > 16:        
                distance_x = (json_load["now"]["targets"][self.blue_pos["n"]]["coordinate"][0] - json_load["now"]["targets"][self.red_pos["n"]]["coordinate"][0])
                distance_y = (json_load["now"]["targets"][self.blue_pos["n"]]["coordinate"][1] - json_load["now"]["targets"][self.red_pos["n"]]["coordinate"][1])
                self.distance = pow(pow(distance_x, 2), 0.5) + pow(pow(distance_y, 2), 0.5)
                print(distance_x,distance_y,self.distance)
        

    def updatePoint(self, direction):
        if direction > 0:
            invert_ang = 0
            cw_ang = 1
            ccw_ang = -1
        else:
            invert_ang = 180
            cw_ang = -1
            ccw_ang = 1
        
        home_position = [[-0.8, 0, math.radians(0)],[-0.4, 0, math.radians(0)]]

        # goal_point = [
        #     [[-0.4, 0, math.radians(0)],[-1.0, 0, math.radians(0)],[-0.85, 0.5 + ccw_ang * 0.07, math.radians(cw_ang * 20)]], #[0] home goal point,
        #     [[0.15, 0.65, math.radians(invert_ang + 335)],[0.15, 0.45, math.radians(invert_ang + 225)],[-0.15, 0.45, math.radians(invert_ang + 135)],[-0.15, 0.65, math.radians(invert_ang + 25)]], #[1] left goal point
        #     [[0.85, 0.5 + cw_ang * 0.07, math.radians(ccw_ang * 160)],[0.45, 0, math.radians(180)],[0.85, -0.5 - ccw_ang * 0.07, math.radians(cw_ang * 200)]], #[2] enemy goal point
        #     [[-0.15, -0.65, math.radians(invert_ang + 155)],[-0.15, -0.45, math.radians(invert_ang + 45)],[0.15, -0.45, math.radians(invert_ang + 335)],[0.15, -0.65, math.radians(invert_ang + 225)]], #[3] right goal point
        #     [[-0.85, -0.5 - cw_ang * 0.07, math.radians(ccw_ang * 340)]],
        #             ]

        goal_point = [
            [[-0.4, 0, math.radians(0)],[-1.0, 0, math.radians(0)],[-0.85, 0.4, math.radians(cw_ang * 20)]], #[0] home goal point,
            [[0.15, 0.65, math.radians(invert_ang + 335)],[0.15, 0.45, math.radians(invert_ang + 225)],[-0.15, 0.45, math.radians(invert_ang + 135)],[-0.15, 0.65, math.radians(invert_ang + 25)]], #[1] left goal point
            [[0.85, 0.4, math.radians(ccw_ang * 200)],[0.45, 0, math.radians(180)],[0.85, -0.4, math.radians(cw_ang * 200)]], #[2] enemy goal point
            [[-0.15, -0.65, math.radians(invert_ang + 155)],[-0.15, -0.45, math.radians(invert_ang + 45)],[0.15, -0.45, math.radians(invert_ang + 335)],[0.15, -0.65, math.radians(invert_ang + 225)]], #[3] right goal point
            [[-0.85, -0.4, math.radians(ccw_ang * 20)]],
                    ]

        in_way = [
            [-0.29, 0.29, math.radians((direction + 45))], #[0] home <-> left in
            [0.29, 0.29, math.radians(direction + 315)], #[1] left <-> enemy in
            [0.29, -0.29, math.radians(direction + 225)], #[2] enemy <-> right in
            [-0.29, -0.29, math.radians(direction + 135)], #[3] right <-> home in
        ]

        out_way = [
            [-0.73, 0.73, math.radians(invert_ang + 45)], #[0] home <-> left out
            [0.73, 0.73, math.radians(invert_ang + 315)], #[1] left <-> enemy out
            [0.73, -0.73, math.radians(invert_ang + 225)], #[2] enemy <-> right out
            [-0.73, -0.73, math.radians(invert_ang + 135)], #[3] right <-> home out
        ]
        return home_position, goal_point, in_way, out_way 

    def uw(self, direction):
        roop = 1
        (home_position, goal_point, in_way, out_way) = self.updatePoint(direction)
        for num in range(5):
            (home_position, goal_point, in_way, out_way) = self.updatePoint(direction)
            for n in range(len(goal_point[num])):
                    self.setGoal(goal_point[num][n][0], goal_point[num][n][1], goal_point[num][n][2])
            if self.distance < 5:
                if num < 4:
                    self.setGoal(out_way[num][0], out_way[num][1], out_way[num][2])
            else:
                if num < 4:
                    self.setGoal(in_way[num][0], in_way[num][1], in_way[num][2])
        for n in range(2):
            self.setGoal(home_position[n][0], home_position[n][1], home_position[n][2])

    def cw(self, direction):
        roop = 1
        for num in range(4):
            (home_position, goal_point, in_way, out_way) = self.updatePoint(direction)
            
            #self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
            if roop == -1:
                calc_num = 2 * num
                self.setGoal(out_way[calc_num][0], out_way[calc_num][1], out_way[calc_num][2])
            elif self.distance > 5:
                self.setGoal(in_way[num][0], in_way[num][1], in_way[2])
            else:
                if num % 2 != 0:
                    for n in range(4):
                        self.setGoal(goal_point[num][n][0], goal_point[num][n][1], goal_point[num][n][2])
                    for n in range(2):    
                        self.setGoal(out_way[num][n][0], out_way[num][n][1], out_way[num][n][2])
                else:
                    self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
                    for n in range(3):
                        self.setGoal(out_way[num][n][0], out_way[num][n][1], out_way[num][n][2])
        for n in range(2):
            self.setGoal(home_position[n][0], home_position[n][1], home_position[n][2])
            

    def ccw(self, direction):
        roop = 1
        (home_position, goal_point, in_way ,out_way) = self.updatePoint(direction)
        for n in range(2):
            self.setGoal(home_position[n][0], home_position[n][1], home_position[n][2])
        for num in reversed(range(4)):
            (home_position, goal_point, in_way ,out_way) = self.updatePoint(direction)
            if roop == -1:
                calc_num = 2 * num
                self.setGoal(out_way[calc_num][0], out_way[calc_num][1], out_way[calc_num][2])
            else:
                if num % 2 != 0:
                    for n in reversed(range(2)):    
                        self.setGoal(out_way[num][n][0], out_way[num][n][1], out_way[num][n][2])
                    for n in reversed(range(4)):
                        self.setGoal(goal_point[num][n][0], goal_point[num][n][1], goal_point[num][n][2])
                else:
                    for n in reversed(range(3)):
                        self.setGoal(out_way[num][n][0], out_way[num][n][1], out_way[num][n][2]) 
                    self.setGoal(goal_point[num][0], goal_point[num][1], goal_point[num][2])
     

    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        
        roop = 0
        direction = 1
        while True:
            if direction > 0:
                self.uw(direction)
            else:
                self.uw(direction)            
            roop = roop + 1
            # direction = direction * -1

if __name__ == '__main__':
    rospy.init_node('test_run')
    bot = NaviBot()
    bot.strategy()
