#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose as pose
def callback(pose):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', pose.x)
    print pose
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('UR_Messages', anonymous=True)

    rospy.Subscriber('pose_object', pose, callback)
    print 'Hello'
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
   
if __name__ == '__main__':
    listener()
