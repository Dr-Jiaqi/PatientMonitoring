# -*- coding: utf-8 -*-
"""
Created on Wed Mar  6 11:38:08 2024

@author: Jiaqi Ye
"""

import cv2
import numpy as np

class hsvDetector():
    def __init__(self):
        # self.image = image
        self.lower_range = np.array([0, 183, 77])
        self.upper_range = np.array([179, 255, 255])
        # self.mask = self.create_mask()
        self.dis2_centre = []
        self.obj_centre = [0,0]
        self.is_object = False
        
    def create_mask(self, image):
        
        # image = cv2.flip(image, 1) 
        
        # Create the sharpening kernel
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]) # I  added for sperm tracking
          
        # Sharpen the image
        image = cv2.filter2D(image, -1, kernel) # I  added for sperm tracking
        # Convert the BGR image to HSV image.
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)
        mask = cv2.erode(mask, None, iterations=1)  # I  added for sperm tracking
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask', mask)
        return mask
    
    def plot_box(self, mask, image):
        
        self.is_object = False
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
          
        for contour in contours:
            # Ignore small contours to eliminate noise
            if cv2.contourArea(contour) > 300:
                # Get bounding box coordinates
                x, y, w, h = cv2.boundingRect(contour)
        
                # Draw bounding box on the original image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
                # Calculate and print the center coordinates
                center_x = x + w // 2
                center_y = y + h // 2
                self.obj_centre[0] = center_x
                self.obj_centre[1] = center_y
                # print(f"Center coordinates: ({center_x}, {center_y})")
                
                position_from_center = (-(center_x - 640), -(center_y - 360))
                
                self.dis2_centre.append(position_from_center)
                tmp = self.dis2_centre[-1]
                print(f"Distance to the centre: {tmp}")
                # Draw a cross at the center of the bounding box
                cv2.line(image, (640,360), (center_x,center_y), (0, 200, 0), 5)
                cv2.drawMarker(image, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                self.is_object = True
                
        return self.is_object, self.obj_centre, self.dis2_centre, image
                