# -*- coding: utf-8 -*-
"""
Created on Wed Mar  6 11:29:15 2024

@author: Jiaqi Ye

This is a project of using ur5e to track an object in the 3D sapce based on visual feedback.

main module: controlling the threadings and the anumation of imu_plot

Realsense module: reading image and imu data from the realsense camera, iteratively.

Detector module: a cutomised object detector applied to the image frames in the realsense 
          module, which can be changed to any other detectors, e.g. yolo-bsaed
          
Robot module: reading the (x, y, z) coordinate of the object from the camera, 
              calculating the robot pose through reverse kinematics,
              and sending the updated robot pose to the robot continuously. 

"""
from Realsense import RealSense
import threading
from threading import Thread
import matplotlib.animation as animation
import matplotlib.pyplot as plt



if __name__ =='__main__':
    # The camera threading
    r = RealSense(detector='yolo')
    lock = threading.Lock()
    t1 = threading.Thread(target=r.start,args=(lock,))
    t1.start()
    t1.join()
    
    # Consideirng to add a robot threading
    
    # The imu_plot threading
    ani = animation.FuncAnimation(plt.gcf(), r.imu_plot, fargs=None, interval=15)
    #
    plt.show()