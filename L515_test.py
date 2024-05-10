# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 12:58:56 2024

@author: Jiaqi Ye
"""

import pyrealsense2 as rs
import open3d as o3d
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)

# Start streaming with the configured settings
pipeline.start(config)

# Create Open3D visualizer
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()

try:
    while True:
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()
        
        # Get the depth frame
        depth_frame = frames.get_depth_frame()
        
        # Convert the depth frame to a point cloud
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        
        # Convert point cloud to numpy array
        vtx = points.get_vertices()
        vertices = np.asarray(vtx).view(np.float32).reshape(-1, 3)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vertices)
        
        # Visualize the point cloud
        visualizer.clear_geometries()
        visualizer.add_geometry(pcd)
        visualizer.poll_events()
        visualizer.update_renderer()

finally:
    pipeline.stop()
    visualizer.destroy_window()

