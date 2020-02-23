# Grasp_Planner

Grasp planning and execution 

1. sim_grasp

Main file: img_listener.py: 
- Subscribe RGB-D image, perform segmentation
- Perform prediction
- Find the point with minimum depth inside the predicted rectangle 
- Publish predicted rectangle with the minimum-depth point

2. Grasp_plan

Main file: grasp_planner.cpp
 -Subcribe Point cloud data, RGB-Image with Grasp points
 - Surface normal estimation
 - Form 6D poses of grasping points in Sensor Coordinates
 - Transforms Sensor Coordinates data to Robot final pose coordinates using Eye-Hand Transformation
 - Publishes final UR-Robot Poses to grasp requested object
 
3. UR_Communication
 
 
