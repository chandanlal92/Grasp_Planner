import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from roslib import message

def listen():
    #rospy.init_node('listen', anonymous=True)
    #depth_data=rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback_kinect)
    #rospy.Subscriber("/kinect2/qhd/points",PointCloud2,callback_kinect)
    #return depth_data
    #rospy.spin()
def callback_kinect(pc2) :
    print "IN callback"
    rospy.loginfo("hello the height and width is ",pc2.data)
    # pick a height
    height =  int (pc2.height / 2)
    print height
    # pick x coords near front and center 
    middle_x = int (data.width / 2)
    # examine point
    rospy.loginfo("hello the height and width is ",height,middle_x)
    middle = read_depth (middle_x, height, data)
    # do stuff with middle
    print print2
    print middle
    return height
    return middle_x
def read_depth(width, height, data) :
    #print "hi"
    # read function
    if (height >= data.height) or (width >= data.width) :
        return -1
    print"hello"
    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
    int_data = next(data_out)
    rospy.loginfo("int_data " + str(int_data))

 

if __name__ == '__main__':
    try:
       
     listen()   
     rospy.init_node('listen', anonymous=True)
     depth_data=rospy.Subscriber("/kinect2/qhd/points", PointCloud2, callback_kinect)
     rospy.Subscriber("kinect2/qhd/points",PointCloud2,callback_kinect)
    ## print depth_data
     rospy.spin()
    except rospy.ROSInterruptException:
        pass
