# Grasp_Planner

Grasp planning and execution 

1. sim_grasp
 Main file: img_listener.py: 
- Subscribe RGB-D image, perform segmentation
- Perform prediction
- Find the point with minimum depth inside the predicted rectangle 
- Publish predicted rectangle with the minimum-depth point
2. Grasp_plan
Main file: 
