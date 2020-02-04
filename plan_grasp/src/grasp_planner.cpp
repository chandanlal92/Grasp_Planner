#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include "geometry_msgs/Point.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/median_filter.h>
#include <vector>
#include "std_msgs/Float32MultiArray.h"

using namespace message_filters;

ros::Publisher pub;
ros::Publisher pub_gqcnn_pose;
geometry_msgs::Point gqcnn_point;
geometry_msgs::Quaternion gqcnn_orientation;
geometry_msgs::Pose gqcnn_pose;

sensor_msgs::PointCloud2 output;
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PCLPointCloud2* cloud_p2 = new pcl::PCLPointCloud2; 
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_p2);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointXYZ object_grasp_point;
pcl::PointXYZ object_width_point1;
pcl::PointXYZ object_width_point2;
Eigen::Vector3f Robot_to_sensor_pos;
Eigen::Quaternionf Robot_to_Sensor_rot;
Eigen::Matrix4f Homgenous_Matrix_Robot;
Eigen::Matrix4f Homgenous_Matrix_Robot_Inverse;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object;
int count=0;


//~ double cx = 9.6762715580208510e+02;
//~ double cy = 5.5317956136661428e+02;
//~ double fx = 1.0628881742777685e+03;
//~ double fy = 1.0620718706590644e+03;
//~ double cx = 2.5360556750641319e+02;
//~ double cy =2.0586703189590986e+02;
//~ double fx =  3.6393331026421930e+02;
//~ double fy =  3.6405684158586399e+02;
boost::mutex m;
double cx = 256.89471435546875;
double cy =205.3076934814453;
double fx =  364.4661865234375;
double fy =  364.4661865234375;


/*
 * Method to get the Surface Normal of Grasping points
 * */
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::PointXYZ center, Eigen::Matrix3f matrix_1)
{
// --------------------------------------------------------
// -----Open 3D viewer and add point cloud and normals-----
// --------------------------------------------------------
m.lock();
//~ pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;

/*
 * Defining Point Cloud Viewer Parameters
 * 
 * */
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

//~ viewer->spinOnce (10);

//~ cout<< ("PointCloud with %d data points (%s)  and frame %s.",
//~ cloud->width * cloud->height,
//~ pcl::getFieldsList (*cloud).c_str (),
//~ cloud->header.frame_id.c_str ());   
//~ color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(cloud, "rgb" ));            
viewer->setBackgroundColor (0, 0, 0);
viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");
viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//~ viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.1, "normals");
//~ viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "normals");
//~ viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "normals"); 



pcl::PointXYZ center_0 (center.x,center.y,center.z);

/*
 * Defining X,Y,Z Axis in the From the Vector Form 
 * 
 * */


pcl::PointXYZ x_axis (matrix_1(0,0), matrix_1(0,1), matrix_1(0,2));
pcl::PointXYZ y_axis (matrix_1(1,0), matrix_1(1,1), matrix_1(1,2));
pcl::PointXYZ z_axis (matrix_1(2,0), matrix_1(2,1), matrix_1(2,2));
 
viewer->addLine (center_0, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"+1);
viewer->addLine (center_0, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector3"+4);
viewer->addLine (center_0, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector6"+7);
viewer->addCoordinateSystem (0.5);
//~ viewer->initCameraParameters ();
while(!viewer->wasStopped())
{
viewer->spinOnce (100);
boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
//~ return(viewer);
m.unlock();
}
void map_to_3d_pointcloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,pcl::PointXYZ &search_Point)
{
	
	
	int index;
	index=search_Point.x+search_Point.y*cloud->width;
	search_Point.x=(double)cloud->points[index].x;
	search_Point.y=(double)cloud->points[index].y;
	search_Point.z=(double)cloud->points[index].z;
	//~ search_Point.x=(double)(((search_Point.x- cx)/1000 )* object_grasp_point.z / (fx/1000));
    //~ search_Point.y=(double)(((search_Point.y - cy)/1000) * object_grasp_point.z / (fy/1000));
	//~ search_Point.z=object_grasp_point.z;
	//~ cout<<"\n"<<search_Point;
	
}

void Convert_to_Homgenous_Matrix(Eigen::Matrix4f &Homogenous_Matrix,Eigen::Matrix3f Rotation_vector,Eigen::Vector3f Position_vector)
{
	Homogenous_Matrix.setIdentity();
	Homogenous_Matrix.block<3,3>(0,0)=Rotation_vector;
	Homogenous_Matrix.block<3,1>(0,3)=Position_vector;
	

}
void transformation_to_Tool_Center_Point(Eigen::Matrix4f &Object_Transformation)
{
	
Robot_to_sensor_pos[0]=-0.0259082758806;
Robot_to_sensor_pos[1]= -0.208224435277;
Robot_to_sensor_pos[2]=0.110147810307;
Robot_to_Sensor_rot.x()= -0.274335101982;
Robot_to_Sensor_rot.y()=-0.039277782493;
Robot_to_Sensor_rot.z()= -0.116505880263;
Robot_to_Sensor_rot.w()= 0.953742044521;



//~ translation: 
  //~ x: -0.0259082758806
  //~ y: -0.208224435277
  //~ z: 0.110147810307
//~ rotation: 
  //~ x: -0.274335101982
  //~ y: 0.039277782493
  //~ z: -0.116505880263
  //~ w: 0.953742044521


//IR FRAME CALIBRATION TO BE USED FOR THIS GRASPING ALGORITHM
//~ translation: 
  //~ x: -0.0362414926042
  //~ y: -0.161618238669
  //~ z: 0.0755769717094
//~ rotation: 
  //~ x: -0.244878514509
  //~ y: -0.0109775775901
  //~ z: -0.173427527022
  //~ w: 0.953853709325


 Eigen::Matrix4f Rotate_Yaxis;
 Eigen::Matrix4f Rotate_Xaxis;
 Eigen::Matrix4f Rotate_Zaxis;
  Eigen::Matrix4f Rotate_Yaxis_Pitch;
 Eigen::Matrix4f Rotate_Xaxis_Roll;
 Eigen::Matrix4f Rotate_Zaxis_Yaw;
 Eigen::Matrix4f Tranform_EE_Tool;


//Defining Tool Center Point Length and Rotation
 float theta,Length_X,theta_Yaw,theta_Roll,theta_Pitch,Length_Y,Length_Z;
 theta=-22.5;
 //~ theta_Yaw=-theta_Yaw*(3.14/180);
 //~ theta_Roll=-theta_Roll*(3.14/180);
 //~ theta_Pitch=-theta_Pitch*(3.14/180);

 theta=-theta*(3.14/180);

 Length_X=0;
 Length_Y=0.00;
 Length_Z=0.165;
 //~ Rotate_Yaxis << -1,0,0,0,
                  //~ 0,1,0,0,
                  //~ 0,0,-1,0,
                  //~ 0,0,0,1;
 //~ Rotate_Xaxis <<  1,0,0,0,
                  //~ 0,-1,0,0,
                  //~ 0,0,-1,0,
                  //~ 0,0,0,1;
 //~ Rotate_Zaxis << -1,0,0,0,
                  //~ 0,-1,0,0,
                  //~ 0,0,1,0,
                  //~ 0,0,0,1;
 //~ Rotate_Yaxis_Pitch << cos(theta_Pitch),0,sin(theta_Pitch),0,
                            //~ 0,1,0,0,
                       //~ -sin(theta_Pitch),0,cos(theta_Pitch),0,
                       //~ 0,0,0,1;
 //~ Rotate_Xaxis_Roll <<  1,0,0,0,
                       //~ 0,cos(theta_Roll),-sin(theta_Roll),0,
                       //~ 0,sin(theta_Roll),cos(theta_Roll),0,
                       //~ 0,0,0,1;
 //~ Rotate_Zaxis_Yaw << cos(theta_Yaw),-sin(theta_Yaw),0,0,
                      //~ sin(theta_Yaw),cos(theta_Yaw),0,0,
                      //~ 0,0,1,0,
                      //~ 0,0,0,1;     
                  
 //Tool is rotated with respect to z-axis thats why only following Matrix is used
 Tranform_EE_Tool << cos(theta),-sin(theta),0,0,
                      sin(theta),cos(theta),0,Length_Y,
                      0,0,1,Length_Z,
                      0,0,0,1;     

 





//~ /*Forming Homogenous Matrix Sensor to Robot flange*/
Eigen::Matrix3f Rotation_robot_to_sensor=Robot_to_Sensor_rot.normalized().toRotationMatrix();
std::cout << "Rotation_robot_to_sensor=" << std::endl << Rotation_robot_to_sensor << std::endl;
Eigen::Matrix4f Aruco_transform;
Homgenous_Matrix_Robot.setIdentity();
Homgenous_Matrix_Robot.block<3,3>(0,0)=Rotation_robot_to_sensor;
Homgenous_Matrix_Robot.block<3,1>(0,3)=Robot_to_sensor_pos;
std::cout << "Homogenous_Transformation=" << std::endl << Homgenous_Matrix_Robot << std::endl;
//~ Homgenous_Matrix_Robot_Inverse=Homgenous_Matrix_Robot.inverse();
//~ std::cout << "Homogenous_Transformation Inverse=" << std::endl << Homgenous_Matrix_Robot_Inverse << std::endl;
//~ Object_Transformation=Object_Transformation*Rotate_Yaxis;
//Transformation of Object To Tool Coordinate System Calculation
Object_Transformation=Homgenous_Matrix_Robot*Object_Transformation;	
Eigen::Matrix4f Object_Transformation_EE=Tranform_EE_Tool.inverse()*Object_Transformation;
std::cout << "Object_Transformation_EE=" << std::endl << Object_Transformation_EE << std::endl;






}

void form_3_point_Rotation_Matrix(pcl::PointXYZ Point_1,pcl::PointXYZ Point_2,pcl::PointXYZ Point_3,Eigen::Vector3f &X_Vector,Eigen::Vector3f &Y_Vector,Eigen::Vector3f Z_Vector,Eigen::Matrix3f &Rotation_matrix)
{
X_Vector[0]=Point_2.x-Point_1.x;
X_Vector[1]=Point_2.y-Point_1.y;
//~ X_Vector[2]=Point_2.z-Point_1.z;
X_Vector[2]=0;
X_Vector.normalize();
Y_Vector=Z_Vector.cross(X_Vector);	
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Y_Vector)	;
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Z_Vector)	;	
cout<<"Orthonormality check....."<<endl<<Z_Vector.dot(Y_Vector)	;	

Rotation_matrix.block<3,1>(0,0)=X_Vector;
Rotation_matrix.block<3,1>(0,1)=Y_Vector;
Rotation_matrix.block<3,1>(0,2)=Z_Vector;
std::cout << "Rotation_matrix="<<std::endl<<Rotation_matrix<<std::endl;

}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;
//  const std::string & cloudname = "cloud"; 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(*cloud_msg, *cloud_p2);
pcl::fromPCLPointCloud2(*cloud_p2, *cloud_pt);
std::vector<int> indices;
pcl::removeNaNFromPointCloud(*cloud_pt, *cloud_out, indices);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//~ octree.setInputCloud (cloud.makeShared());
kdtree.setInputCloud(cloud_pt);
float radius = 1;//search in sphere of 1 cm           
std::vector<int> radiusIdx;
std::vector<float> radiusSQDist;

Eigen::Vector4f plane_parameters; 
float curvature; 
int index;         
//~ index=object_grasp_point.x+object_grasp_point.y*cloud_pt->width;
//~ index=object_grasp_point.x+object_grasp_point.y*cloud_pt->width;
//~ object_grasp_point.z=(double)(cloud_pt->points[index].z); 
//~ object_grasp_point.x=(double)(cloud_pt->points[index].x); 
//~ object_grasp_point.y=(double)(cloud_pt->points[index].y); 


map_to_3d_pointcloud(cloud_pt,object_grasp_point);
map_to_3d_pointcloud(cloud_pt,object_width_point1);
map_to_3d_pointcloud(cloud_pt,object_width_point2);
cout<<"\n"<<object_grasp_point;
cout<<"\n"<<object_width_point1;
cout<<"\n"<<object_width_point2;


gqcnn_point.x=object_grasp_point.x;
gqcnn_point.y=object_grasp_point.y;
gqcnn_point.z=object_grasp_point.z;

if (kdtree.radiusSearch (object_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{            
cout<<"YES!";
}                                     

computePointNormal(*cloud_pt,radiusIdx,plane_parameters,curvature); 
//~ flipNormalTowardsViewpoint (object_grasp_point, 0, 0, 0, plane_parameters);
cout<<"param: "<<plane_parameters<<endl;
cout<<"curvature: "<<curvature<<endl;
//~ gqcnn_orientation.x=(double)plane_parameters[0];
//~ gqcnn_orientation.y=(double)plane_parameters[1];
//~ gqcnn_orientation.z=(double)plane_parameters[2];
//~ gqcnn_orientation.w=(double)plane_parameters[3];



Eigen::Quaternionf quat(plane_parameters);
Eigen::Vector3f position(object_grasp_point.x,object_grasp_point.y,object_grasp_point.z);
Eigen::Vector3f vector_z;
Eigen::Vector3f vector_x;
Eigen::Vector3f vector_y;
Eigen::Matrix3f Rotation_matrix_object;
vector_z(0)=(double)plane_parameters[0];
vector_z(1)=(double)plane_parameters[1];
vector_z(2)=(double)plane_parameters[2];
form_3_point_Rotation_Matrix(object_grasp_point,object_width_point1,object_width_point2,vector_x,vector_y,vector_z,Rotation_matrix_object);
cout<<"Rotation_matrix_object =="<<endl<<Rotation_matrix_object<<endl;

Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object,Rotation_matrix_object,position);
cout<<"homogenous_matrix_object_in_sensor_coordinates =="<<endl<<Homgenous_Matrix_Sensor_Object<<endl;

Eigen::Matrix4f Transformed_object_to_Robot=Homgenous_Matrix_Sensor_Object;
transformation_to_Tool_Center_Point(Transformed_object_to_Robot);
cout<<"homogenous_matrix_object_in_Robot_Tool_coordinates =="<<endl<<Transformed_object_to_Robot<<endl;
Eigen::Matrix3f Orientation_object_Robot_Coordinates=Transformed_object_to_Robot.block<3,3>(0,0);
Eigen::Vector3f position_object_in_Robot_coordinates=Transformed_object_to_Robot.block<3,1>(0,3);
cout<<"position_object_in_Robot_coordinates =="<<endl<<position_object_in_Robot_coordinates<<endl;
cout<<"Orientation_object_Robot_Coordinates =="<<endl<<Orientation_object_Robot_Coordinates<<endl;
Eigen::Vector3f Orientation_object_Robot_Coordinates_in_rpy=Orientation_object_Robot_Coordinates.eulerAngles(0,1,2);
cout<<"Orientation_object_Robot_Coordinates_in_RPY_Radians =="<<endl<<Orientation_object_Robot_Coordinates_in_rpy<<endl;
Eigen::Vector3f Orientation_object_Robot_Coordinates_in_rpy_angels;
Orientation_object_Robot_Coordinates_in_rpy_angels(0)=(Orientation_object_Robot_Coordinates_in_rpy(0)*180)/3.14;
Orientation_object_Robot_Coordinates_in_rpy_angels(1)=(Orientation_object_Robot_Coordinates_in_rpy(1)*180)/3.14;
Orientation_object_Robot_Coordinates_in_rpy_angels(2)=(Orientation_object_Robot_Coordinates_in_rpy(2)*180)/3.14;
cout<<"Orientation_object_Robot_Coordinates_in_RPY_Angles =="<<endl<<Orientation_object_Robot_Coordinates_in_rpy_angels<<endl;

gqcnn_pose.position.x=position_object_in_Robot_coordinates(0);
gqcnn_pose.position.y=position_object_in_Robot_coordinates(1);
gqcnn_pose.position.z=position_object_in_Robot_coordinates(2);
gqcnn_pose.orientation.x=Orientation_object_Robot_Coordinates_in_rpy(0);
gqcnn_pose.orientation.y=Orientation_object_Robot_Coordinates_in_rpy(1);
gqcnn_pose.orientation.z=Orientation_object_Robot_Coordinates_in_rpy(2);

//calculate final pose by multiplying with the transformation matrix according to the Robot tool and publish it#
pub_gqcnn_pose.publish(gqcnn_pose);
pcl::toROSMsg(*cloud_out, output);
pub.publish(output);


//Finding normal surface normals of the point cloud
pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud(cloud_pt);
ne.setSearchSurface(cloud_pt);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
ne.setSearchMethod (tree);
ne.setRadiusSearch (0.01);
ne.setKSearch(0);
//~ ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
//~ ne.setMaxDepthChangeFactor(0.02f);
//~ ne.setNormalSmoothingSize(10.0f);
ne.compute(*normals);
cout<<"\n"<<normals->points.size();
//~ pcl::toROSMsg(*normals,output2);
//~ pcl::PCLPointCloud2 cloud_filtered;
normalsVis(cloud_pt,normals,object_grasp_point,Rotation_matrix_object);
}

/*
 * This method is to get Grasping Points of Object 
 * @parameter[IN] coordinate point(X,Y,Z) of object 
 * */
void  normal_estimate_cb(const geometry_msgs::Point::ConstPtr& msg)
{
//~ object_grasp_point.z = (double)((msg->z));
//~ object_grasp_point.x =(double)((-(msg->x) +cx) /fx);
//~ object_grasp_point.y =(double)(((msg->y)-cy ) /fy);
object_grasp_point.z = (msg->z);
object_grasp_point.x =(msg->x);
object_grasp_point.y =(msg->y);
cout<<"\n"<<object_grasp_point;
}
void  bombe_bb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
//~ for(int i=0;i<16;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }

}
void  grasp_width_points(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
//~ for(int i=0;i<4;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }
object_width_point1.x=msg->data[0];
object_width_point1.y=msg->data[1];
object_width_point2.x=msg->data[2];
object_width_point2.y=msg->data[3];

}


int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl");
ros::NodeHandle nh;
geometry_msgs::Point search;
pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
pub_gqcnn_pose=nh.advertise<geometry_msgs::Pose>("/gqcnn_pose", 1);
// Create a ROS subscriber for the input point cloud
ros::Subscriber pt_4 = nh.subscribe<geometry_msgs::Point> ("/gqcnn_grasp/point", 1, normal_estimate_cb);
ros::Subscriber pt_5 = nh.subscribe<std_msgs::Float32MultiArray> ("/simtrack/bombe_bb", 1, bombe_bb);
ros::Subscriber pt_6 = nh.subscribe<std_msgs::Float32MultiArray> ("/gqcnn_grasp/Width_points", 1, grasp_width_points);

ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/sd/points", 1, cloud_cb);
ros::spin();
}

