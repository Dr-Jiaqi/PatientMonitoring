# -*- coding: utf-8 -*-
"""
Created on Wed Mar  6 10:59:57 2024

@author: Jiaqi Ye
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
import threading
from threading import Thread
import mediapipe as mp
import math
import open3d as o3d
# from mediapipe_detector import Mediapipe_BodyDetector
from yolov8_detector import YoloV8_BodyDetector
# from measurer import QRDetector
from Detector import hsvDetector
from Robot import MoveUR

class RealSense():
    
    '''
    The Realsense object manage the dataflow from the realsense camera including 
    the color and depth images, and the accel and gyro data.
    
    '''
    def __init__(self, detector = ''):
        # Create a pipeline
        self.pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        # different resolutions of color and depth streams
        self.config = rs.config()
        
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # self.config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)

        # if device_product_line == 'L500':
        #     self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        # else:
        #     self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)  # Depth stream configuration
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # Color stream configuration
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # IMU acceleration stream
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # IMU gyroscope stream
        
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        self.qr = False
        self.resolution_x = 0
        self.resolution_y = 0
        self.greencolor = (0,255,0)
        self.count = 0
        
        self.isReceiving = False
        self.timer = []
        self.accel_x_list = []
        self.gyro_x_list = []
        
        self.accel_y_list = []
        self.gyro_y_list = []
        
        self.accel_z_list = []
        self.gyro_z_list = []
        # Initialize the legend handles and labels
        self.legend_handles = []
        self.legend_labels = []
        
        # Call and intitialise the detector
        self.mediapipe = False
        self.yolo = False
        # self.detector = hsvDetector()
        self.is_obj = False
        
        self.robot_pos = [0,0]
        self.surface_depth = np.zeros((10,10))
        
        # if detector == 'mediapipe':
        #     self.detector = Mediapipe_BodyDetector()
        #     self.mediapipe = True
        if detector == 'yolo':
            self.detector = YoloV8_BodyDetector()
            self.yolo = True
        elif detector == 'box':
            self.detector = hsvDetector()
            
        # self.qr_detector = QRDetector()
        
        # Call and intitialise the robot
        self.my_robot = MoveUR()
    
        
        
    def start(self,lock):
        lock.acquire()
        Thread(target=self.get).start()
        lock.release()
        # Block till we start receiving values
        while self.isReceiving != True:
            time.sleep(0.15)
        return self
    
    def get(self):
        # Start the RealSense pipeline
        profile = self.pipeline.start(self.config)
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        # We will be removing the background of objects more than
        # clipping_distance_in_meters meters away
        clipping_distance_in_meters = 1.5 #1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        
        try:
            while True:
                # Wait for the next set of frames from the RealSense camera
                self.frames = self.pipeline.wait_for_frames()
                
                
                # Align the depth frame to color frame
                self.aligned_frames = self.align.process(self.frames)

                # Extract the depth frame, color frame, and IMU data
                depth_frame = self.aligned_frames.get_depth_frame()
                color_frame = self.aligned_frames.get_color_frame()
                
                
                
                accel_frame = self.frames.first_or_default(rs.stream.accel)
                gyro_frame = self.frames.first_or_default(rs.stream.gyro)


                if not depth_frame or not color_frame:
                    continue
                
                # Convert the color frame to a numpy array
                color_image = np.asanyarray(color_frame.get_data())
                color_image = cv2.flip(color_image, -1)
                # Detect the QR code and calculate the pixel sizes
                # self.qr, color_image = self.qr_detector.findqr(color_image)
                
                # if self.qr and self.count == 0:
                #     self.resolution_x = self.qr_detector.pixel_size()
                    
                #     string = self.resolution_x * abs(self.detector.shoulder[0] - self.detector.shoulder[1])
                    
                #     self.count += 1
                
                # if self.count == 1:
                    
                #     cv2.putText(color_image, str(string), (100,100), cv2.FONT_HERSHEY_SIMPLEX,0.8,self.greencolor, 2)    

                # Convert the depth frame to a numpy array
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_image = cv2.flip(depth_image, -1)
                
                
                # depth_frame = cv2.flip(depth_frame, 1) 
                # Greyscale to colormap
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_HOT)
                # Remove background - Set pixels further than clipping_distance to balck
                depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
                depth_bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), 0, depth_colormap)
                # color_frame = cv2.flip(color_frame, 1) 
                
                
                self.is_obj, obj_centre, obj_pos, color_image = self.detector.findPose(color_image)
 
                # mask = self.detector.create_mask(color_image)
                # self.is_obj, obj_centre, obj_pos, color_image = self.detector.plot_box(mask, color_image)
                if len(obj_pos) > 0 and self.is_obj:
                    displacement = math.sqrt((obj_pos[-1][0]-640)**2 + (obj_pos[-1][1]-360)**2)
                    print('----',displacement)
                    if 10 < displacement < 1200:
                        # print(obj_centre[0],obj_centre[1])
                        # print(np.shape(depth_image))
                        # Need to check if depth is from the object
                        self.surface_depth = depth_image[obj_centre[1]-5:obj_centre[1]+5,obj_centre[0]-5:obj_centre[0]+5]
                        # print(self.surface_depth)
                        depth = np.mean(self.surface_depth)
                        # if depth:
                        depth = depth * depth_scale
                        print ('Object depth: ', depth)
                        print('Sending commends to the robot')
                        self.robot_pos = self.my_robot.move_to_obj(obj_pos, self.robot_pos, depth)
                        self.is_obj = False
                    # else:
                        # self.robot_pos = self.my_robot.move_to_org()
                        # self.is_obj = False
                   
                
                # if self.mediapipe:
                #     color_image = self.detector.findPose(color_image)
                #     lmList, bboxInfo = self.detector.findBodyPosition(color_image)
                # elif self.yolo:
                #     color_image = self.detector.findPose(color_image)
                
                
                
                # Access and display IMU data
                if accel_frame and gyro_frame:
                    self.accel_data = accel_frame.as_motion_frame().get_motion_data()
                    self.gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                    
                    # print("IMU Acceleration (m/s^2):", self.accel_data.x, self.accel_data.y, self.accel_data.z)
                    # print("IMU Gyroscope (rad/s):", self.gyro_data.x, self.gyro_data.y, self.gyro_data.z)

                # Set the transparency level (0.0 to 1.0, where 0.0 is fully transparent and 1.0 is fully opaque)
                alpha = 0.5
                color_back = color_image.copy()
                # Create the overlaid image using cv2.addWeighted
                cv2.addWeighted(depth_bg_removed, alpha, color_back, 1, 0, color_back)
                # Display the overlaid image
                cv2.imshow('Overlay', color_back)
                # Display the depth and color frames (you can also save or process them as needed)
                # cv2.imshow('Depth Stream', depth_colormap)
                # cv2.imshow('Color Stream', color_image)

                # Break the loop when the 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.my_robot.stop_robot()
                    break
                    
                self.isReceiving = True
                
            


        finally:
            # Release the RealSense pipeline
            self.pipeline.stop()

            # Close all OpenCV windows
            cv2.destroyAllWindows()
            
    
    def imu_plot(self, frames):
        current_time = time.perf_counter()
        self.timer.append(current_time)
        self.accel_x_list.append(round(self.accel_data.x,1))
        self.gyro_x_list.append(round(self.gyro_data.x,1))
        
        self.accel_y_list.append(round(self.accel_data.y,1))
        self.gyro_y_list.append(round(self.gyro_data.y,1))
        
        self.accel_z_list.append(round(self.accel_data.z,1))
        self.gyro_z_list.append(round(self.gyro_data.z,1))
        
        
        plt.cla()
        
        # Plot IMU Gyroscope
        plt.subplot(211)
        line1, = plt.plot(self.timer,self.gyro_x_list, color = 'red', label = 'gyro_x')
        line2, = plt.plot(self.timer,self.gyro_y_list, color = 'green', label = 'gyro_y')
        line3, = plt.plot(self.timer,self.gyro_z_list, color = 'blue', label = 'gyro_z')
        plt.title('IMU Gyroscope (m/s^2)')
        plt.xlim([max(self.timer)-20,max(self.timer)])
        
        
        # Update the legend handles and labels
        self.legend_handles = [line1, line2, line3]
        self.legend_labels = ['gyro_x','gyro_y','gyro_z']
        
        #  Set the legend for subplot(211)
        plt.legend(self.legend_handles, self.legend_labels, loc='upper left')
        
        # Plot IMU Acceleration
        plt.subplot(212)
        line4, = plt.plot(self.timer,self.accel_x_list, color = 'red', label = 'accel_x')
        line5, = plt.plot(self.timer,self.accel_y_list, color = 'green', label = 'accel_y')
        line6, = plt.plot(self.timer,self.accel_z_list, color = 'blue', label = 'accel_z')
        plt.title('IMU Acceleration (rad/s)')
        plt.xlim([max(self.timer)-20,max(self.timer)])
        
        # Update the legend handles and labels
        self.legend_handles = [line4, line5, line6]
        self.legend_labels = ['accel_x', 'accel_y', 'accel_z']

        #  Set the legend for subplot(212)
        plt.legend(self.legend_handles, self.legend_labels, loc='upper left')

        # plt.suptitle('IMU Data')
        plt.tight_layout()
        
    