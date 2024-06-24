# -*- coding: utf-8 -*-
"""
Created on Thu Apr 28 16:15:16 2022

@author: ye
"""
import socket

import time

import math

import URBasic


# IP of the robot
ROBOT_IP = "192.168.1.102"

PORT = 30002

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)


robot.reset_error()
print("robot initialised")
time.sleep(1)

waypoint1 = list((math.radians(-2),
                    math.radians(-108),
                    math.radians(107),
                    math.radians(-88),
                    math.radians(-91),
                    math.radians(18)))

waypoint2 = list((math.radians(-30),
                    math.radians(-108),
                    math.radians(107),
                    math.radians(-88),
                    math.radians(-91),
                    math.radians(18)))


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.connect((ROBOT_IP, PORT))
time.sleep(1)

s.send(("set_digital_out(0,True)"+"\n").encode('utf8'))
# time.sleep(1)
# s.send(("set_digital_out(0,False)"+"\n").encode('utf8'))
time.sleep(1)
s.send(("set_digital_out(7,True)"+"\n").encode('utf8'))
time.sleep(1)
s.send(("set_digital_out(7,False)"+"\n").encode('utf8'))

time.sleep(1)

for i in range(5):
# robot.movej(q=robot_startposition, a= ACCELERATION, v= VELOCITY )
    s.send((f"movej({waypoint1},a = 0.2, v = 0.2)"+"\n").encode('utf8'))
    time.sleep(3)        
    
    s.send((f"movej({waypoint2},a = 0.2, v = 0.2)"+"\n").encode('utf8'))
    time.sleep(3)   
# s.send(("movej([135,430,488,\
#         2.217,-2.222,0.035],\
#         a = 0.2, v = 0.2)"+"\n").encode('utf8'))

# time.sleep(1)
# s.send(("set_digital_out(1,True)"+"\n").encode('utf8'))
# s.send(("set_digital_out(1,False)"+"\n").encode('utf8'))

data = s.recv(1024)
s.close()
print('Received',repr(data))

#%% Modern_Robotics course
import modern_robotics as mr
import numpy as np
R = np.array([[0, 0, 1],
              [1, 0, 0],
              [0, 1, 0]])
invR = mr.RotInv(R)
print(invR)
help(mr.RotInv)