#!/usr/bin/env python
import message_filters
from std_msgs.msg import String, Float32MultiArray
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

count = 0
c = 0
model = create_model('/home/chandan_main/Documents/Trinh/Grap_Redmon/More_neurons/backup/1024/grasp_redmon-95120')
#model = create_model('/home/tncong/Documents/Trinh/source_code/CNN/grasp_redmon_3.tfl')
bg = cv2.imread('/home/chandan/catkin_ws/src/sim_grasp/src/background.png') # background image
im = PIL.Image.fromarray(bg)
point_pub  = geometry_msgs.msg.Point()
grasp = {}
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

old_x = 0.
old_y = 0.

class img_listener:

    def __init__(self):
        global model, bg, count, point_pub, old_x, old_y, im
        self.rgb_sub = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)
        self.depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect", Image)
        self.points_sub = message_filters.Subscriber("/simtrack/model_names_bb", Float32MultiArray)       
        self.model_names = rospy.get_param("/simtrack/model_names") #get names of all objects in demo_object.yaml
        self.bb_subscribers_ = {}
        self.bridge = CvBridge()
        
        #for multiple objects at a time
        for name in self.model_names:
			self.bb_subscribers_[name] = message_filters.Subscriber( "/simtrack/" + name + "_bb", Float32MultiArray)
			ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.bb_subscribers_[name]], 10, 0.1, allow_headerless=True)
			ts.registerCallback(self.callback)      
        '''
        # uncomment to Take background image
        #ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1, allow_headerless=False)
        #ts.registerCallback(self.take_bground_callback)
        
        # uncomment to for one object
        
        self.pub = rospy.Publisher('/sim_grasp/point', geometry_msgs.msg.Point, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub,  self.points_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        print ("Finish Init!")
        '''
    def take_bground_callback(self, rgb_img, depth_img):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")
            #cv2.imwrite('/home/chandan/img/rgb.png', rgb_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")
        except CvBridgeError as e:
            print(e)
        depth_array = np.array(depth_image, dtype=np.float32) #convert depth img to numpy array
        depth_copy = np.asarray(depth_image)
        frame = cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX) #normalize the depth to range of 0-255
        bgr_image[:,:,0] = frame #replace blue by depth
        cv2.imwrite('/home/tncong/img/background.png', bgr_image)
        #bgr_image.show()
         
    def callback(self, rgb_img, depth_img, points):
        global count, point_pub, c, grasp, old_x, old_y, im
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_img, "bgr8")
            #cv2.imwrite('/home/chandan/img/original.png', rgb_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_img, "passthrough")
            p = np.asarray(points.data)
        except CvBridgeError as e:
            print(e)
        rgb_pil = PIL.Image.fromarray(rgb_image[:, :, ::-1].copy())
        depth_array = np.array(depth_image, dtype=np.float32) #convert depth img to numpy array
        #depth_copy = np.asarray(depth_image)
        frame = cv2.normalize(depth_array, depth_array, 0, 255, cv2.NORM_MINMAX) #normalize the depth to range of 0-255
        rgb_image[:,:,0] = frame #replace blue by depth
        #cv2.imwrite('/home/chandan/img/bluetodepth.png', rgb_image)
       #print(rgb_image)
        mask = np.zeros(rgb_image.shape, dtype=np.uint8)
        #roi_corners = np.array([(p[8],p[9]), (p[10],p[11]), (p[14],p[15]), (p[12],p[13]) ], dtype=np.int32) #bounding box vertices, last 4 points, upper of the bounding cube
        #points in bounding cube
        roi_corners = np.array([[(p[0],p[1]), (p[2],p[3]), (p[6],p[7]), (p[4],p[5]), (p[8],p[9]), (p[10],p[11]), (p[14],p[15]), (p[12],p[13]) ]], dtype=np.int32) #bounding box vertices, first 4 points  
        hull = cv2.convexHull(roi_corners) #automatically get the convex hull from the points
        channel_count = rgb_image.shape[2] #Shape contains [row,column,numberOfChannel] => shape[2] = 3 (i.e. RGB)
        ignore_mask_color = (255,)*channel_count
        
        cv2.fillConvexPoly(mask, hull, ignore_mask_color)
        masked_image1 = cv2.bitwise_and(rgb_image, mask)
        #masked_image_11=PIL.Image.fromarray(masked_image1)
        #masked_image_11.show()
        mask2 = cv2.bitwise_not(mask)
        masked_image2 = cv2.bitwise_and(bg, mask2)
   
        
        image_final = masked_image1 + masked_image2 #object + background

        # origin of the bounding box
        origin_x = (p[0]+p[2]+p[4]+p[6])/4 
        origin_y = (p[1]+p[3]+p[5]+p[7])/4 
        #print(origin_x,origin_y)
        #print(img_crop.size)
        offset = 17; #update every 17 pixel = 5mm
        #print(count)
        
        #predict all object
        if (count < 3):
           print(count)
           img_pil = PIL.Image.fromarray(image_final)
           #img_pil.show()
           img_crop = img_pil.crop((origin_x-113, origin_y-113, origin_x+114, origin_y+114)) #crop img to 227x227 using PIL
           #print(img_crop)
           #img_crop.show()
           img_crop_opencv = np.asarray(img_crop) # convert back to opencv type
           #r,g,b=img_crop.split()
           #img_crop_opencv=PIL.merge("BGR",(b, g, r))
           #print(img_crop_opencv)
           #cv2.imwrite('/home/chandan/img/img_crop_opencv.png',img_crop_opencv)
           #print(img_crop_opencv)
           #img_crop_opencv = cv2.imread('/home/chandan/img/img_crop_opencv.png')
           #img_crop_opencv = cv2.cvtColor(img_crop_opencv, cv2.COLOR_RGB2BGR)
           
           img_crop_opencv  = img_crop_opencv[:,:,::-1].copy() # Convert RGB to BGR 
           img_crop_opencv = PIL.Image.fromarray(img_crop_opencv)
           #print(img_crop_opencv)
           img_crop_opencv.show()
           #cv2.imwrite('/home/chandan/img/test'+str(c)+'.png', img_crop_opencv)
           #c = c+1 
           grasp_rect, grasp_angle, rect_center = predict_grasp(model,img_crop_opencv)
           
           #draw rectangle
           #translate back to original image
           grasp_rect[:,0] = grasp_rect[:,0] + origin_x-113
           grasp_rect[:,1] = grasp_rect[:,1] + origin_y-113
           
           #find point with minimum depth inside the rectangle
           poly = Polygon(grasp_rect)
           min_dis   = 1000000.
           min_depth = 100000.
           for pt in get_points_inside_polygon(poly):
			#print(depth_image[pt[0],pt[1]])
			pt_shapely = Point(pt[0],pt[1])
			dis = np.linalg.norm(pt - np.array(poly.centroid))
			if ((depth_image[pt[0],pt[1]] <= min_depth) &  (dis < min_dis)):
				self.min_depth = depth_image[pt[0],pt[1]]
				#print(self.min_depth)
				point_pub.x = pt[0]
				point_pub.y = pt[1]
				point_pub.z = self.min_depth
				min_dis = dis
				
           if (count==0):
			   im = rgb_pil
           draw = PIL.ImageDraw.Draw(im)
           draw_box(grasp_rect, draw, blue, red)
           r=1
           draw.ellipse((point_pub.x-r, point_pub.y-r, point_pub.x+r, point_pub.y+r), fill=red)
           count = count+1
           print point_pub
           if (count==3):
              im.show()
           #print(count)

        '''
        #predict one object
        img_pil = PIL.Image.fromarray(image_final)
        img_crop = img_pil.crop((origin_x-113, origin_y-113, origin_x+114, origin_y+114)) #crop img to 227x227 using PIL
        img_crop_opencv = np.array(img_crop) # convert back to opencv type
        img_crop_opencv  = img_crop_opencv[:, :, ::-1].copy()# Convert RGB to BGR 
        grasp_rect, grasp_angle, rect_center = predict_grasp(model, img_crop_opencv)
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
    global point_pub, im, count
    rospy.init_node('img_listener', anonymous=True)
   # pub = rospy.Publisher('/sim_grasp/point', geometry_msgs.msg.Point, queue_size=10)  
    rate = rospy.Rate(5)
    il = img_listener()
    while not rospy.is_shutdown():
        #pub.publish(point_pub)
        rate.sleep()
    '''
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()'''

if __name__ == '__main__':
    main(sys.argv)
