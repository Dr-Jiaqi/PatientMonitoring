# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 11:41:12 2024

@author: Jiaqi Ye
"""

import socket

import time

import math

import URBasic

import keyboard


# IP of the robot
ROBOT_IP = "192.168.1.102"

PORT = 30002

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)


robot.reset_error()
print("robot initialised")
time.sleep(1)

waypoint1 = (math.radians(-114),
                    math.radians(-67),
                    math.radians(-119),
                    math.radians(-80),
                    math.radians(84),
                    math.radians(61))

waypoint2 = (math.radians(-136),
                    math.radians(-51),
                    math.radians(-126),
                    math.radians(-86),
                    math.radians(86),
                    math.radians(39))
try:
    while True:
        if keyboard.is_pressed('enter'):  # Check if Enter key is pressed
            print('Stopped ......')
            break  # Break out of the loop if Enter key is pressed
            
        robot.movej(q=waypoint1, a=0.2, v=0.2)
        if keyboard.is_pressed('enter'):  # Check if Enter key is pressed
            print('Stopped ......')
            break  # Break out of the loop if Enter key is pressed

        robot.movej(q=waypoint2, a=0.2, v=0.2)
        if keyboard.is_pressed('enter'):  # Check if Enter key is pressed
            print('Stopped ......')
            break  # Break out of the loop if Enter key is pressed

finally:
    robot.close()