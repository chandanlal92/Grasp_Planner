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
from geometry_msgs.msg import Pose as pose
import math
import urx



#rob = urx.Robot("192.168.0.100")
final_grasp_pose__to_robot=[0,0,0,0,0,0]
home_pose=[0.84415,-0.5416,0.93217,2.44,-2.27,0.7759]
count=0



def move_to_grasp_pose(pose):
 print 'robot'
 rob = urx.Robot("192.168.12.69")
 rob.set_tcp((0,0,0.165,0,0,-0.3925))
 final_grasp_pose__to_robot[0]=pose.position.x
 final_grasp_pose__to_robot[1]=pose.position.y
 final_grasp_pose__to_robot[2]=pose.position.z
 final_grasp_pose__to_robot[3]=pose.orientation.x
 final_grasp_pose__to_robot[4]=pose.orientation.y
 final_grasp_pose__to_robot[5]=pose.orientation.z
 rob.set_payload(2,(0,0,0.1))
 time.sleep(0.2)
 rob.movel_tool(final_grasp_pose__to_robot,0.15,0.2)
 time.sleep(0.2)
 rob.movep(home_pose,0.15,0.2)
 rob.stopl(0.25)
#Move to Grasping postion of Bat_box
def pose_batbox_callback(pose):
 global count 
 final_pose_to_robot=pose
 print pose
 count=count+1
 print count
 if(count==1):
  move_to_grasp_pose(final_pose_to_robot)


 





def main(args):
   #while(1):
    global point_pub, im
    
    rospy.init_node('UR_Messages', anonymous=True)
    rospy.Subscriber('pose_object',pose,pose_batbox_callback,queue_size=1)  
    print 'Hello'
    
    #rosy.subscribe('pose_point_bombe',pose,pose_bombe_callback,queue_size=1)
    #rosy.subscribe('pose_point_staple',pose,pose_staple_callback,queue_size=1)
    #rosy.subscribe('pose_point_test',pose,pose_test_callback,queue_size=1)
    rate = rospy.Rate(5)
    #il = img_listener()
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
