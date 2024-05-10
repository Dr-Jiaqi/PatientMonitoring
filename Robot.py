# -*- coding: utf-8 -*-
"""
Created on Fri Mar  8 16:31:12 2024

@author: Jiaqi Ye
"""

import math
import URBasic
import math3d as m3d
import time

class MoveUR():
    '''
     Connect to the UR through IP
     
     Move to the centre of the object using basic transformations
     
     Stop the robot when needed
    
    '''    
    def __init__(self, robot_ip = '192.168.1.102'):
        self.robot_ip = robot_ip
        self.m_per_pixel = 0.000014  # JQ: This needs to be checked
        # The robot will at most move this distance in each direction
        self.max_x = 0.05 #JQ: 0.5 for the red box
        self.max_y = 0.05 #JQ: 0.5 for the red box
        self.max_z = 0.2
        self.pre_z = 0
        self.current_z = 0
        # Maximum Rotation of the robot at the edge of the view window
        self.hor_rot_max = math.radians(50) #JQ: 5 for the red box
        self.vert_rot_max = math.radians(25) #JQ: 5 for the red box
        # self.z_rot_max = math.radians(5)
        # The Joint position the robot starts at
        self.robot_startposition = (math.radians(-87),
                            math.radians(-95),
                            math.radians(95),
                            math.radians(-172),
                            math.radians(-91),
                            math.radians(0))
        
        robotModel = URBasic.robotModel.RobotModel()
        self.robot = URBasic.urScriptExt.UrScriptExt(host=self.robot_ip,robotModel=robotModel)
        self.robot.reset_error()
        print("robot initialised")
        time.sleep(1)
        
        # Move Robot to the midpoint of the lookplane
        self.robot.movej(q=self.robot_startposition, a= 0.9, v= 0.8) #JQ: 0.3 and 0.2 for the red box
        
        
        # Creates a new coordinate system at the current robot tcp position.
        # This coordinate system is the basis of the face following.
        # It describes the midpoint of the plane in which the robot follows faces.
        position = self.robot.get_actual_tcp_pose()
        self.origin  = m3d.Transform(position)
        
        self.robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
        time.sleep(1) # just a short wait to make sure everything is initialised
        
        
    def check_max_xyz(self, xyz_coord):
        """
        Checks if the face is outside of the predefined maximum values on the lookaraound plane
     
        Inputs:
            xy_coord: list of 2 values: x and y value of the face in the lookaround plane.
                These values will be evaluated against max_x and max_y
     
        Return Value:
            x_y: new x and y values
                if the values were within the maximum values (max_x and max_y) these are the same as the input.
                if one or both of the input values were over the maximum, the maximum will be returned instead
        """
     
        x_y_z = [0, 0]
        #print("xy before conversion: ", xy_coord)
     
        if -self.max_x <= xyz_coord[0] <= self.max_x:
            # checks if the resulting position would be outside of max_x
            x_y_z[0] = xyz_coord[0]
        elif -self.max_x > xyz_coord[0]:
            x_y_z[0] = -self.max_x
        elif self.max_x < xyz_coord[0]:
            x_y_z[0] = self.max_x
        else:
            raise Exception(" x is wrong somehow:", xyz_coord[0], -self.max_x, self.max_x)
     
        if -self.max_y <= xyz_coord[1] <= self.max_y:
            # checks if the resulting position would be outside of max_y
            x_y_z[1] = xyz_coord[1]
        elif -self.max_y > xyz_coord[1]:
            x_y_z[1] = -self.max_y
        elif self.max_y < xyz_coord[1]:
            x_y_z[1] = self.max_y
        else:
            raise Exception(" y is wrong somehow", xyz_coord[1], self.max_y)
        
        #print("xy after conversion: ", x_y)
        # if -self.max_z <= xyz_coord[2] <= self.max_z:
        #     # checks if the resulting position would be outside of max_z
        #     
        # elif -self.max_z > xyz_coord[2]:
        #     x_y_z[2] = -self.max_z
        # elif self.max_z < xyz_coord[2]:
        #     x_y_z[2] = self.max_z
        # else:
        #     raise Exception(" x is wrong somehow:", xyz_coord[0], -self.max_x, self.max_x)
        
     
        return x_y_z
    
    def set_lookorigin(self):
        """
        Creates a new coordinate system at the current robot tcp position.
        This coordinate system is the basis of the face following.
        It describes the midpoint of the plane in which the robot follows faces.
    
        Return Value:
            orig: math3D Transform Object
                characterises location and rotation of the new coordinate system in reference to the base coordinate system
    
        """
        position = self.robot.get_actual_tcp_pose()
        orig = m3d.Transform(position)
        return orig
    
    
    def move_to_obj(self, list_of_facepos, robot_pos, depth):
        """
        Function that moves the robot to the position of the face

        Inputs:
            list_of_facepos: a list of face positions captured by the camera, only the first face will be used
            robot_pos: position of the robot in 2D - coordinates

        Return Value:
            prev_robot_pos: 2D robot position the robot will move to. The basis for the next call to this funtion as robot_pos
        """


        face_from_center = list(list_of_facepos[-1])  # TODO: find way of making the selected face persistent

        prev_robot_pos = robot_pos
        scaled_face_pos = [c * self.m_per_pixel for c in face_from_center]
        
        # print('Here2: ', scaled_face_pos)
        
        # scaled_face_pos.append(depth - 0.26)
        
        # print('Here3: ', scaled_face_pos)
        
       #JQ: Update the robot endeffector(tcp) position to the centre of the object
       # This assumes that the tcp position always align with the centre of the camera frame?
        robot_target_xyz = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]   # These are still in the camera frame 
        # print("Here4： ", robot_target_xyz)

        robot_target_xyz = self.check_max_xyz(robot_target_xyz)
        prev_robot_pos = robot_target_xyz
        # print("Here5： ", robot_target_xyz)

        x = robot_target_xyz[0]
        y = robot_target_xyz[1]
        z = (0.55 - depth)*1000 # For the red box
        # z = 0 
        
        # print('Current Z offset (mm): ', z)
        # if  z < -10:
        #     self.current_z = self.pre_z-0.01
        #     print('Down')
        # if z > 10 :
        #     self.current_z = self.pre_z+0.01
        #     print('Up')
        # elif abs(z) < 10:
        #     self.current_z = self.pre_z
        #     print('Still')
        
    
        
        # print('Robot Pre_Z: ', self.current_z)
        # print('Robot Pre_X_Y: ', x, y)
        # print('tcp_pose', self.robot.get_actual_tcp_pose())
        xyz_coords = m3d.Vector(x, y, 0)
        print('xyz_coords: ', xyz_coords)
        

        x_pos_perc = x / self.max_x
        y_pos_perc = y / self.max_y
        # z_pos_perc = z / self.max_z

        x_rot = x_pos_perc * self.hor_rot_max
        y_rot = y_pos_perc * self.vert_rot_max * -1
        # z_rot = z_pos_perc * self.z_rot_max

        tcp_rotation_rpy = [y_rot, x_rot, 0]
        # tcp_rotation_rvec = convert_rpy(tcp_rotation_rpy)
        tcp_orient = m3d.Orientation.new_euler(tcp_rotation_rpy, encoding='xyz')
        position_vec_coords = m3d.Transform(tcp_orient, xyz_coords)

        oriented_xyz = self.origin * position_vec_coords
        oriented_xyz_coord = oriented_xyz.get_pose_vector()

        coordinates = oriented_xyz_coord # coordinates are [x, y, z, Rx, Ry, Rz] of the end effector
        
        current_coordinates = self.robot.get_actual_tcp_pose()
        # print('tcp_pose', self.robot.get_actual_tcp_pose())
        print('coordinates: ', coordinates)
        
                
        print('Current Z offset (mm): ', z)
        if  z < -10:
            coordinates[1] = current_coordinates[1]+0.01
            # coordinates[3] = coordinates[3] + 0.075
            print('Forward')
        if z > 10 :
            coordinates[1] = current_coordinates[1]-0.01
            # coordinates[3] = coordinates[3] - 0.05
            print('Backward')
        elif abs(z) < 10:
            coordinates[1] = current_coordinates[1]
            print('Still')
        
 
        # qnear = self.robot.get_actual_joint_positions()
        next_pose = coordinates
        self.robot.set_realtime_pose(next_pose)
        
        return prev_robot_pos
    
    def move_to_org (self):
        self.robot.movej(q=self.robot_startposition, a= 0.3, v= 0.2) #JQ: 0.3 and 0.2 for the red box

    def stop_robot(self):
        self.robot.close()
        print('Robot disconnected ...')




    # robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
    # time.sleep(1) # just a short wait to make sure everything is initialised

    
        
        