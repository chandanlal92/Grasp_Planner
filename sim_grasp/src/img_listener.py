#!/usr/bin/env python
#! -*- coding: utf-8 -*-
import argparse
import logging
import IPython
import os
import signal
import message_filters
from std_msgs.msg import String, Float32MultiArray, Float32, MultiArrayDimension
from sensor_msgs.msg import Image 
import rospy
import cv2
import sys, time
from cv_bridge import CvBridge, CvBridgeError
import roslib
import numpy as np
from rospy.numpy_msg import numpy_msg
from visualization_msgs.msg import Marker 
import geometry_msgs.msg 
from predict import *
import PIL
import PIL.ImageDraw  
from shapely.geometry import Polygon, Point
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
import math
from pcl_ros import* 
import pcl_ros as pc
#from perception import RgbdImage, RgbdSensorFactory
from autolab_core import RigidTransform, YamlConfig
from perception import CameraIntrinsics, CameraSensor, ColorImage, DepthImage, RgbdImage, RgbdSensorFactory,Kinect2BridgedQuality
from perception import Image as Im
from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
from gqcnn import Visualizer as vis

#from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
#from gqcnn import Visualizer as vis

count = 0
count_3 = 0
c = 0
model = create_model('/home/chandan_main/Documents/Trinh/Grap_Redmon/More_neurons/backup/1024/grasp_redmon-95120')
#model = create_model('/home/chandan_main/Downloads/model_zoo/GQ-Adv/model')
#model = create_model('/home/chandan_main/Documents/tncong/source_code/CNN/grasp_redmon_3.tfl')
bg = cv2.imread('/home/chandan_main/catkin_ws/src/sim_grasp/src/background.png') # background image
#bg = cv2.imread('/home/chandan_main/catkin_ws/src/sim_grasp/src/back_grounds/image.png') # background image
im = PIL.Image.fromarray(bg)
im_depth = PIL.Image.fromarray(bg)
poses=[]
point_pub=[]
grasp_rect_points=[]
grasp_rect_pub={}
grasp_angle_points=[]
grasp_angle_pub={}
#point_pub  = [geometry_msgs.msg.Point(),geometry_msgs.msg.Point(),geometry_msgs.msg.Point()]
#image_merge=[][][]
grasp = {}
pos=[]
pub={}
grasp_point=geometry_msgs.msg.Point()
grasp_rect_array=[geometry_msgs.msg.Point()]
#rate = rospy.Rate(5)
#pub = rospy.Publisher('/sim_grasp/point', geometry_msgs.msg.Point, queue_size=10)  
#get coordinate of points inside a polygon
def get_points_inside_polygon(poly):
    (minx, miny, maxx, maxy) = poly.bounds
    minx = int(minx)
    miny = int(miny)
    maxx = int(maxx)
    maxy = int(maxy)
    #print("poly.bounds:", poly.bounds)
    points_in_polygon = []
    for x in range(minx, maxx+1):
        for y in range(miny, maxy+1):
            p = Point(x, y)
            if poly.contains(p):
                points_in_polygon.append([x, y])
    return points_in_polygon



#def normalestimation

old_x = 0.
old_y = 0.

class img_listener:

    def __init__(self):
        
        global model, bg, count, point_pub, old_x, old_y, im
        self.rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)
        self.depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
        self.model_names = rospy.get_param("/simtrack/model_names") #gö,,et names of all objects in demo_object.yaml
        #self.points_sub = message_filters.Subscriber("/simtrack/"+self.model_names+"_bb", Float32MultiArray)
        self.bb_subscribers_ = {}
        self.bridge = CvBridge()
        #self.min_depth = 100000.
        #self.min_dis   = 1000000.
        #self.pub={}
        #for multiple objects at a time
        count_1=0
        #while(1):
        for name in self.model_names:
	 self.bb_subscribers_[name] = message_filters.Subscriber( "/simtrack/" + name + "_bb", Float32MultiArray)
	 
         ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.bb_subscribers_[name]],30, 0.1, allow_headerless=True)
         ts.registerCallback(self.callback)     
         pub[name]=rospy.Publisher("/sim_grasp/points_"+name,geometry_msgs.msg.Point,queue_size=10)
         point_pub.append(geometry_msgs.msg.Point())
         grasp_rect_pub[name]=rospy.Publisher("/sim_grasp/grasp_rect_points_"+name,Float32MultiArray,queue_size=10)
         grasp_rect_points.append(Float32MultiArray())
         grasp_angle_pub[name]=rospy.Publisher("/sim_grasp/grasp_angle_points_"+name,Float32,queue_size=1)
         grasp_angle_points.append(Float32)
         #rospy.sleep(1.0)
         #count_1=count_1+1
         #if(count_1==3):
          #count_1=0 
       # print pub
        '''      
        #uncomment to Take background image
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1, allow_headerless=False)
        ts.registerCallback(self.take_bground_callback)  
        
        # uncomment to for one object
        
        self.pub = rospy.Publisher('/sim_grasp/point', geometry_msgs.msg.Point, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub,  self.points_sub], 20, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        print ("Finish Init!")
        '''
    def take_bground_callback(self, rgb_img, depth_img):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")                               
            #cv2.imwrite('/home/chandan_main/img/rgb.png', rgb_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")
            
        except CvBridgeError as e:
            print(e)
        depth_array = np.array(depth_image, dtype=np.float32) #convert depth img to numpy array

        depth_copy = np.asarray(depth_image)
        frame = cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX) #normalize the depth to range of 0-255
        bgr_image[:,:,0] = frame #replace blue by depth
        #cv2.imwrite('/home/chandan_main/catkin_ws/src/sim_grasp/src/background.png', bgr_image)
        #bgr_image.show()
    def callback(self, rgb_img, depth_img, points):

        global count, point_pub, c, grasp, old_x, old_y, im,count_3
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")
            #cv2.imwrite('/home/chandan_main/img/original.png', rgb_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")
            #depth_image.show()
            #print depth_img
            p = np.asarray(points.data)
            #print p
        except CvBridgeError as e:
            print(e)
        rgb_pil = PIL.Image.fromarray(rgb_image[:, :, ::-1].copy())
        im_depth = PIL.Image.fromarray(depth_image)
        #depth_pil.show()
        depth_array = np.array(depth_image, dtype=np.float32) #convert depth img to numpy array
        #pointcloud_Image=array_to_pointcloud2(depth_array,stamp=None, frame_id=None)
        #depth_copy = np.asarray(depth_image)
        frame = cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX) #normalize the depth to range of 0-255
        rgb_image[:,:,0] = frame #replace blue by depth
        #cv2.imwrite('/home/tncong/img/bluetodepth.png', rgb_image)
        mask = np.zeros(rgb_image.shape, dtype=np.uint8)
        #roi_corners = np.array([(p[8],p[9]), (p[10],p[11]), (p[14],p[15]), (p[12],p[13]) ], dtype=np.int32) #bounding box vertices, last 4 points, upper of the bounding cube
        #points in bounding cube
        roi_corners = np.array([[(p[0],p[1]), (p[2],p[3]), (p[6],p[7]), (p[4],p[5]), (p[8],p[9]), (p[10],p[11]), (p[14],p[15]), (p[12],p[13]) ]], dtype=np.int32) #bounding box vertices, first 4 points  
        hull = cv2.convexHull(roi_corners) #automatically get the convex hull from the points
        channel_count = rgb_image.shape[2] #Shape contains [row,column,numberOfChannel] => shape[2] = 3 (i.e. RGB)
        ignore_mask_color = (255,)*channel_count
        
        cv2.fillConvexPoly(mask, hull, ignore_mask_color)
        masked_image1 = cv2.bitwise_and(rgb_image, mask)
      
        mask2 = cv2.bitwise_not(mask)
        masked_image2 = cv2.bitwise_and(bg, mask2)
        
        image_final = masked_image1 + masked_image2 #object + background

        
        #image_merge = image_final+ im_depth
       
        # origin of the bounding box
        origin_x = (p[0]+p[2]+p[4]+p[6])/4 
        origin_y = (p[1]+p[3]+p[5]+p[7])/4 
        #print(origin_x,origin_y)
        #print(img_crop.size)
        offset = 17; #update every 17 pixel = 5mm
        #print(count)
        #poses=[]
        #predict all object
        if (count < 4):
           #print(count)
           
           img_pil = PIL.Image.fromarray(image_final)
           #img_pil.show()
           img_crop = img_pil.crop((origin_x-113, origin_y-113, origin_x+114, origin_y+114)) #crop img to 227x227 using PIL
           size=32,32
           img_crop_dexnet=img_crop.resize(size,PIL.Image.ANTIALIAS) 
           #img_crop_dexnet = img_pil.crop((origin_x-15, origin_y-15, origin_x+16, origin_y+16)) #crop img to 32x32 using PIL 
           #img_crop_dexnet.show()
           #print img_crop
           #img_crop.show()
           img_crop_opencv = np.array(img_crop) # convert back to opencv type
           #print img_crop_opencv
           #point_cloud=array_to_pointcloud2(img_crop_opencv)
           #point_cloud.show()
           img_crop_opencv  = img_crop_opencv[:, :, ::-1].copy()# Convert RGB to BGR 
           img_crop_opencv = PIL.Image.fromarray(img_crop_opencv)
         
           #img_crop_opencv.show()
           #cv2.imwrite('/home/chandan_main/img/test'+str(c)+'.png', img_crop_opencv)
           #c = c+1 
           grasp_rect, grasp_angle, rect_center = predict_grasp(model, img_crop_opencv)
           
           
         
















           #draw rectangle
           #translate back to original image
           grasp_rect[:,0] = grasp_rect[:,0] + origin_x-113
           grasp_rect[:,1] = grasp_rect[:,1] + origin_y-113
           grasp_angle_degree=(grasp_angle/np.pi)*180
           #find point with minimum depth inside the rectangle
           poly = Polygon(grasp_rect)
           min_dis   = 10000000.
           min_depth = 1000000.
           grasp_rect_points[count].layout.dim.append(MultiArrayDimension())
           grasp_rect_points[count].layout.dim.append(MultiArrayDimension())
           grasp_rect_points[count].layout.dim[0].label = "height"
           grasp_rect_points[count].layout.dim[1].label = "width"
           grasp_rect_points[count].layout.dim[0].size = 4
           grasp_rect_points[count].layout.dim[1].size = 2
           grasp_rect_points[count].layout.dim[0].stride = 4*2
           grasp_rect_points[count].layout.dim[1].stride = 2
           grasp_rect_points[count].layout.data_offset = 0
           grasp_rect_points[count].data = [0]*8
           dstride0 = grasp_rect_points[count].layout.dim[0].stride
           dstride1 = grasp_rect_points[count].layout.dim[1].stride
           offset = grasp_rect_points[count].layout.data_offset
           grasp_rect_points[count].data[0]=grasp_rect[0][0]
           grasp_rect_points[count].data[1]=grasp_rect[0][1]
           grasp_rect_points[count].data[2]=grasp_rect[1][0]
           grasp_rect_points[count].data[3]=grasp_rect[1][1]
           grasp_rect_points[count].data[4]=grasp_rect[2][0]
           grasp_rect_points[count].data[5]=grasp_rect[2][1]
           grasp_rect_points[count].data[6]=grasp_rect[3][0]
           grasp_rect_points[count].data[7]=grasp_rect[3][1]
           #grasp_rect_points[count].data[6]=grasp_rect[i][j]
           #for i in range(4):
            #for j in range(2):
              #grasp_rect_points[count].data[0 + i +dstride1*j]=grasp_rect[i][j]
           #print grasp_rect_points[count].data
           grasp_angle_points[count]=grasp_angle
           #self.min_depth = 100000.
           #self.min_dis   = 1000000.
           for pt in get_points_inside_polygon(poly):
			#print(depth_image[pt[0],pt[1]])
                        pt_shapely = Point(pt[0],pt[1])
                        #print poly.centroid
                        #print pt
                        #cross=np.cross(pt,np.array(poly.centroid))
			dis =  np.linalg.norm(pt - np.array(poly.centroid))
                        #self.min_depth =depth_image[pt[0],pt[1]]
                        #print(depth_image[pt[0],pt[1]])
                        #print (depth_image[pt[0],pt[1]].size())
			if ((depth_image[pt[0],pt[1]] <= min_depth) & (dis < min_dis)) & (min_depth>0):
		                self.min_depth =depth_image[pt[0],pt[1]]
				#print(self.min_depth)
				point_pub[count].x = pt[0]
				point_pub[count].y = pt[1]
                                point_pub[count].z= self.min_depth
                                min_dis = dis
                                #print self.min_depth
                              
                                #zcross=np.cross(point_pub.x,point_pub.y)
				#zvec= np.linalg.norm(zcross)
                                #znorm=zcross/zvec
                                #znorm1=(grasp_rect[1]-grasp_rect[0])               
                                #znorm2=(grasp_rect[2]-grasp_rect[0])
                                #print znorm1
                                #print znorm2
                                #zcross=np.cross(znorm1,znorm2)
                                #print zcross
                               
                                #zvec=cross/znorm        frame = cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX) #normalize the depth to range of 0-255
                                #print zvec
           #print count
           print point_pub
           #grasp_rect_points[count]=grasp_rect
           #grasp_rect_pub.publish(grasp_rect_points[count])
           #pos.append(point_pub)  

           if (count==0):
            im = rgb_pil

           
           count = count+1
           
           #print grasp_rect_points                         
           draw = PIL.ImageDraw.Draw(im)
           #draw2 = PIL.ImageDraw.Draw(im_depth)
           draw_box(grasp_rect, draw, blue, red)
           #draw_box(grasp_rect, draw2, blue, red)
           r=1
           draw.ellipse((point_pub[count-1].x-r, point_pub[count-1].y-r, point_pub[count-1].x+r, point_pub[count-1].y+r), fill=red)
           #draw2.ellipse((point_pub[count-1].x-r, point_pub[count-1].y-r, point_pub[count-1].x+r, point_pub[count-1].y+r), fill=red)
           #pub.publish(point_pub)
           print('postion of' ,self.model_names[count-1])
           print self.model_names[count-1]
           print self.min_depth
           print point_pub[count-1]  
         #poses.append(pos[count-1])   
          # print poses
           
           #print pos
           
           
           #pub[self.model_names[count-1]].publish(point_pub[count-1])
           print rect_center
           print grasp_angle
           print grasp_rect
           #point_pub=[geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Point(0,0,0)]
           #rospy.loginfo(point_pub) 
           #pub.publish(point_pub)
          # print zcross
          # print zvec
           #print znorm
           #print point_pub
           if (count==4):
               
              #count = 0
              #point_pub=[geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Point(0,0,0)]
              #print point_pub
              #count_3=count_3+1
              #if(count_3==1):
              #print im
              #print im_depth
              im.show()
              #im = rgb_pil
               #count_3=0 
              #cv2.imwrite('/home/chandan_main/img/final.png', image_final)
              #im_depth.show()
              #cv2.imwrite('/home/chandan_main/img/final_Depth.png', depth_image)
              #print poses
              #return poses  
              #print point_pub
              
          
        '''
        #predict one object
        img_pil = PIL.Image.fromarray(image_final)
        img_crop = img_pil.crop((origin_x-113, origin_y-113, origin_x+114, origin_y+114)) #crop img to 227x227 using PIL
        img_crop_opencv = np.array(img_crop) # convert back to opencv type
        img_crop_opencv  = img_crop_opencv[:, :, ::-1].copy()# Convert RGB to BGR 
        grasp_rect, grasp_angle, rect_center = predict_grasp(model, img_crop_opencv)l
        grasp_rect[:,0] = grasp_rect[:,0] + origin_x-113
        grasp_rect[:,1] = grasp_rect[:,1] + origin_y-113
 
        
           
        #find point with minimum depth inside the rectangle
        poly = Polygon(grasp_rect)
        for pt in get_points_inside_polygon(poly):
			#print(depth_image[pt[0],pt[1]])
			pt_shapely = Point(pt[0],pt[1])
			dis = np.linalg.norm(pt - np.array(poly.centroid))
			if ((depth_image[pt[0],pt[1]] <= self.min_depth) &  (dis < self.min_dis)):
				self.min_depth = depth_image[pt[0],pt[1]]
				#print(self.min_depth)
				point_pub.x = pt[0]
				point_pub.y = pt[1]
				point_pub.z = self.min_depth
				self.min_dis = dis
        curr_x = np.mean(roi_corners[:,0])
        curr_y = np.mean(roi_corners[:,1])
        if (curr_x > old_x+17 or curr_x < old_x-17 or curr_y> old_y +17 or curr_y < old_y-17):
           img_pil.show()
           old_x = curr_x
           old_y = curr_y
           #img = PIL.Image.fromarray(img_crop_opencv)
           #img = PIL.Image.open('/home/tncong/Desktop/o17.png')
           img = rgb_pil;
           #print(grasp_rect, point_pub)
           draw = PIL.ImageDraw.Draw(img)
           draw_box(grasp_rect, draw, blue, red)
           r=1
           #draw.ellipse((point_pub.x-r, point_pub.y-r, point_pub.x+r, point_pub.y+r), fill=red)
           #draw.point((point_pub.x,point_pub.y),fill=red)
           img.show()
           #count = 1
       '''

     
             


def main(args):
   #while(1):
    global point_pub,grasp_rect_points,grasp_angle_points, im, count
   
    rospy.init_node('img_listener', anonymous=True)
    #pub_point=rospy.Publisher('/gqcnn_grasp/point',geometry_msgs.msg.Point,queue_size=10)
    
    #pub = rospy.Publisher('/sim_grasp/point', geometry_msgs.msg.Point, queue_size=10)  
    rate = rospy.Rate(5)
    il = img_listener()
    #gqcnn_policy()
    #il_1=[]
    #il_1=il.callback(img_listener())
    #print il_1
    n=0
    #print pos
    #print pub
   #test()
    #print grasp_rect_points
    while not rospy.is_shutdown():
     count_2=0
 
    # print pos
     for name in il.model_names:  
      #rospy.sleep(0.5)
      pub[name].publish(point_pub[count_2])
      grasp_rect_pub[name].publish(grasp_rect_points[count_2])
      grasp_angle_pub[name].publish(grasp_angle_points[count_2])
      count_2=count_2+1
     
      if(count_2==4):
       count_2=0
    
       #pub[name].publish(point_pub[name])
      #for n in range(0,4): 
      
            #pub[n].publish(point_pub)
     #print pos 
     #rate.sleep()

  
    #rospy.init_node('my_pcl',anonymous=True)
   
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
