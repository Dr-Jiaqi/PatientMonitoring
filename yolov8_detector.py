# -*- coding: utf-8 -*-
"""
Created on Mon Feb  5 14:14:31 2024

@author: Jiaqi Ye
"""

import cv2
from ultralytics import YOLO


class YoloV8_BodyDetector():
    
    def __init__(self):
        self.model1 = YOLO('yolov8n-pose.pt')
        # self.model2 = YOLO('yolov8n-seg.pt')
        # self.classes = self.model2.names
        self.shoulder = [0,0]
        self.obj_centre = [0,0]
        self.dis2_centre = []
        self.body = False
        
    def findPose(self, img):
        # print(self.classes)
        results1  = self.model1(img, stream=True, conf = 0.5, max_det = 1)
        # results2  = self.model2(img, stream=True, conf = 0.5)
        results1 = list(results1)
        # results2 = list(results2)
        self.body = False
        # annotated_frame = img
        if results1:
            for r in results1:
                mypoints = r.keypoints  # Keypoints object for pose outputs
                # print('Here:',mypoints.xy[0][0:3])
                
                cords = mypoints.xy[0][0:1]
                
                for i, cord in enumerate(cords):
                    x, y = cord[0].tolist(), cord[1].tolist()
                    self.obj_centre[0] = int(x)
                    self.obj_centre[1] = int(y)
                    # self.shoulder[i] = x
                    print('X:', x, 'Y:', y)
                    # print(self.shoulder)
                    self.body = True
                    
                    position_from_center = ((x - 640), (y - 360))

                    self.dis2_centre.append(position_from_center)
                    tmp = self.dis2_centre[-1]
                    print(f"Distance to the centre: {tmp}")
                    cv2.circle(img, (int(x),int(y)), 20, (153, 51, 255), 2)  
                    cv2.line(img, (640,360), (int(x),int(y)), (153, 51, 255), 5)
        else:
            self.body = False         
        
        # for result in results2:
        #     labels = result.boxes.cls
        #     for i, cls_ in enumerate(labels):
        #         if self.classes[int(cls_)] == 'person':
        #             if results2:
        #                 annotated_frame = results2[0].plot() 
        annotated_frame = results1[0].plot() 
        
       
        return self.body, self.obj_centre, self.dis2_centre, annotated_frame