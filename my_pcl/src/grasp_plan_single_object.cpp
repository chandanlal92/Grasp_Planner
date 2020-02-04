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
#include <sensor_msgs/CameraInfo.h>
#include "std_msgs/Float32MultiArray.h"
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
#include <yaml.h>
#include <Eigen/Dense>


using namespace message_filters;
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher pub6;
//~ ros::Publisher pub_batbox_points;
//~ ros::Publisher pub_staple_points;
//~ ros::Publisher pub_test_points;
//~ ros::Publisher pub_bombe_points;
//~ ros::Publisher pub_batbox_pose;
//~ ros::Publisher pub_staple_pose;
//~ ros::Publisher pub_test_pose;
//~ ros::Publisher pub_bombe_pose;
ros::Publisher object_points;
ros::Publisher object_pose;

sensor_msgs::PointCloud2 output;
sensor_msgs::PointCloud2 output2;
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PCLPointCloud2* cloud_p2 = new pcl::PCLPointCloud2; 
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_p2);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZ>);


geometry_msgs::Point Point_bat_box;
pcl::PointXYZ object1_grasp_point;
pcl::PointXYZ object1_rect_coordinates_1;
pcl::PointXYZ object1_rect_coordinates_2;
pcl::PointXYZ object1_rect_coordinates_3;
pcl::PointXYZ object1_rect_coordinates_4;

pcl::PointXYZ Depth_Point;
geometry_msgs::Quaternion Orientation_batbox;
geometry_msgs::Pose pose_bat_box;
geometry_msgs::Pose Sensor;
geometry_msgs::Pose Aruco_pose;


/*Provide The Home Pose of the Robot with Respect to Base*/

const float Robot_Home_Position_x=0.84414;
const float Robot_Home_Position_y=-0.5417;
const float Robot_Home_Position_z=0.88664;
const float Robot_Home_Orientation_x=-2.622;//RPY=-150.15
const float Robot_Home_Orientation_y=-0.134;//RPY=-7.53
const float Robot_Home_Orientation_z=-1.534;//RPY=-87.97
Eigen::Vector3f Robot_Home_Position;
Eigen::Vector3f Robot_Home_Orientation;
Eigen::Matrix3f Robot_Home_Rotation_Matrix;
Eigen::Matrix4f Homogenous_Robot_Home_Pose;
Eigen::Vector3f Object_Rotation_Base_vector;
int count=0;

//~ double cx = 9.6762715580208510e+02;
//~ double cy = 5.5317956136661428e+02;
//~ double fx = 1.0628881742777685e+03;
//~ double fy = 1.0620718706590644e+03;
double cx = 2.5360556750641319e+02;
double cy =2.0586703189590986e+02;
double fx =  3.6393331026421930e+02;
double fy =  3.6405684158586399e+02;

Eigen::Vector3f Robot_to_sensor_pos;
Eigen::Quaternionf Robot_to_Sensor_rot;
Eigen::Vector3f Aruco_pos;
Eigen::Vector3f Aruco_pos_Tool;
Eigen::Quaternionf Aruco_rot;
Eigen::Matrix4f Homgenous_Matrix_Robot;
Eigen::Matrix4f Homgenous_Matrix_Robot_Inverse;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object1;
Eigen::Matrix4f Homgenous_Matrix_Aruco;
//~ double px=-5.2052476112081990e-02;
//~ double py=-4.6313865353939110e-04;
//~ double pz=8.8806735554907584e-04;
boost::mutex m;

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



//~ search_Point.x=(double)(((search_Point.x- cx))* object1_grasp_point.z / fx);
//~ search_Point.y=(double)(((search_Point.y - cy)) * object1_grasp_point.z / fy);
//~ search_Point.z=object1_grasp_point.z;
//~ search_Point.x=(double)(((search_Point.x- cx)/1000 )* object1_grasp_point.z / (fx/1000));
//~ search_Point.y=(double)(((search_Point.y - cy)/1000) * object1_grasp_point.z / (fy/1000));
//~ search_Point.z=object1_grasp_point.z/1000;

//~ Depth_Point.z=object1_grasp_point.z;
//~ search_Point.z=Depth_Point.z;
//~ cout<<"\n"<<search_Point;

}
void Convert_to_Homgenous_Matrix(Eigen::Matrix4f &Homogenous_Matrix,Eigen::Matrix3f Rotation_vector,Eigen::Vector3f Position_vector)
{
Homogenous_Matrix.setIdentity();
Homogenous_Matrix.block<3,3>(0,0)=Rotation_vector;
Homogenous_Matrix.block<3,1>(0,3)=Position_vector;


}
void Rotate_Y_axis(float theta_Pitch,Eigen::Matrix3f &Rotate_Yaxis_Pitch)
{

Rotate_Yaxis_Pitch << cos(theta_Pitch),0,sin(theta_Pitch),
	0,1,0,
-sin(theta_Pitch),0,cos(theta_Pitch);

}
void Rotate_X_axis(float theta_Roll,Eigen::Matrix3f &Rotate_Xaxis_Roll)
{

Rotate_Xaxis_Roll<<1,0,0,
0,cos(theta_Roll),-sin(theta_Roll),
0,sin(theta_Roll),cos(theta_Roll);

}
void Rotate_Z_axis(float theta_Yaw,Eigen::Matrix3f &Rotate_Zaxis_Yaw)
{
Rotate_Zaxis_Yaw<<cos(theta_Yaw),-sin(theta_Yaw),0,
sin(theta_Yaw),cos(theta_Yaw),0,
0,0,1;


}
/*
* This Method is Used To Form Rotation Matrix Using Three Point Method
* 
* */
void form_3_point_Rotation_Matrix(pcl::PointXYZ Point_1,pcl::PointXYZ Point_2,pcl::PointXYZ Point_3,Eigen::Vector3f &X_Vector,Eigen::Vector3f &Y_Vector,Eigen::Vector3f &Z_Vector,Eigen::Matrix3f &Rotation_matrix)
{
	
	
	
X_Vector[0]=Point_2.x-Point_1.x;
X_Vector[1]=Point_2.y-Point_1.y;
X_Vector[2]=Point_2.z-Point_1.z;
//~ X_Vector[2]=0;

//~ X_Vector=X_Vector.normalized();

Y_Vector[0]=Point_3.x-Point_1.x;
Y_Vector[1]=Point_3.y-Point_1.y;
Y_Vector[2]=Point_3.z-Point_1.z;
//~ Y_Vector[2]=0;
//~ Y_Vector=Y_Vector.normalized();

Z_Vector=X_Vector.cross(Y_Vector);
//~ Y_Vector=Y_Vector.normalized();
Z_Vector.normalize();
X_Vector.normalize();
Y_Vector[0]=0;
Y_Vector[1]=0;
Y_Vector[2]=0;
Y_Vector=Z_Vector.cross(X_Vector);	 
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Y_Vector)	;
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Z_Vector)	;
cout<<"Orthonormality check....."<<endl<<Y_Vector.dot(Z_Vector)	;	

Rotation_matrix.block<3,1>(0,0)=X_Vector;
Rotation_matrix.block<3,1>(0,1)=Y_Vector;
Rotation_matrix.block<3,1>(0,2)=Z_Vector;
std::cout << "Rotation_matrix="<<std::endl<<Rotation_matrix<<std::endl;
//~ Rotation_matrix=Rotation_matrix.transpose();

}

void transformation_to_Tool_Center_Point(Eigen::Matrix4f &Object_Transformation)
{


/*Calibrated Transformation of Kinect Sensor to Robot Flange*/	

Robot_to_sensor_pos[0]= 0.00698862397997;
Robot_to_sensor_pos[1]= -0.191616229867;
Robot_to_sensor_pos[2]= 0.0996425526147;
Robot_to_Sensor_rot.x()= -0.245258233196;
Robot_to_Sensor_rot.y()= -0.000215451617785;
Robot_to_Sensor_rot.z()=-0.207058543993;
Robot_to_Sensor_rot.w()=0.947087700263;


/*Very very Good Calibration Result*/
//~ x: 0.00698862397997
//~ y: -0.191616229867
//~ z: 0.0996425526147
//~ rotation: 
//~ x: -0.245258233196
//~ y: -0.000215451617785
//~ z: -0.207058543993
//~ w: 0.947087700263




//~ Eigen::Vector4f Aruco_check;
Eigen::Matrix4f Aruco_check1;
Eigen::Matrix4f Rotate_Yaxis;
Eigen::Matrix4f Rotate_Xaxis;
Eigen::Matrix4f Rotate_Zaxis;
Eigen::Matrix4f Rotate_Yaxis_Pitch;
Eigen::Matrix4f Rotate_Xaxis_Roll;
Eigen::Matrix4f Rotate_Zaxis_Yaw;
Eigen::Matrix4f Tranform_EE_Tool;
Eigen::Matrix3f Rotation_Flange_Base;
Eigen::Matrix3f Rotation_Tool_Base;
Eigen::Vector3f Position_Tool_Flange;
Eigen::Matrix3f Rotation_Tool_Flange;
Eigen::Matrix3f Rotation_Matrix_Xaxis_Roll;
Eigen::Matrix3f Rotation_Matrix_Yaxis_Pitch;
Eigen::Matrix3f Rotation_Matrix_Zaxis_Yaw;
Eigen::Vector3f Rotation_Flange_Base_Angles;
Eigen::Vector3f X_Vector_Tool_Base;
Eigen::Vector3f Y_Vector_Tool_Base;
Eigen::Vector3f Z_Vector_Tool_Base;
//~ Position_Tool_Flange[0]=0.00701;
//~ Position_Tool_Flange[1]=0.0033;
//~ Position_Tool_Flange[2]=0.165;
Position_Tool_Flange[0]=0.0;
Position_Tool_Flange[1]=0.0;
Position_Tool_Flange[2]=0.170;

float theta,Length_X,theta_Yaw,theta_Roll,theta_Pitch,Length_Y,Length_Z;

theta=-22.5;
/*Initializing Robot Home Postion*/
Robot_Home_Position(0)=Robot_Home_Position_x;
Robot_Home_Position(1)=Robot_Home_Position_y;
Robot_Home_Position(2)=Robot_Home_Position_z;
Robot_Home_Orientation(0)=Robot_Home_Orientation_x;
Robot_Home_Orientation(1)=Robot_Home_Orientation_y;
Robot_Home_Orientation(2)=Robot_Home_Orientation_z;

//~ theta=0;
//~ theta_Roll=178.86;
//~ theta_Pitch=1.95;
//~ theta_Yaw=-137.4;
/*
* Flange to Tool Center Point Calculation
*Three Point of Tool To from Home Position is Given Below 
* 
* */
pcl::PointXYZ P,Q,R;
//Points On the Tool
//~ P.x=754.18;
//~ P.y=36.48;
//~ P.z=1051.78;
//~ Q.x=747.52;
//~ Q.y=-60.54;
//~ Q.z=1050.07;
//~ R.x=736.86;
//~ R.y=-31.28;
//~ R.z=1041.55;
//~ P.x=1000;
//~ P.y=0;
//~ P.z=550;
//~ Q.x=990.38;
//~ Q.y=-23.05;
//~ Q.z=499.97;
//~ R.x=981.53;
//~ R.y=7.78;
//~ R.z=499.90;

/*
* Euler Angles of The Robot Flange
* 
* */
//~ theta_Roll=180;
//~ theta_Pitch=0;
//~ theta_Yaw=-112.5;
//~ theta_Roll=-149.5;
//~ theta_Pitch=-3.85;
//~ theta_Yaw=-130.6;
//~ theta_Roll=-149.5;
//~ theta_Pitch=-3.85;
//~ theta_Yaw=-130.6;
//~ theta_Roll=180;
//~ theta_Pitch=0;
//~ theta_Yaw=-112.5;

//~ theta_Roll=3.126;
//~ theta_Pitch=-0.006;
//~ theta_Yaw=-1.968;
//Values at my working Range
//~ theta_Roll=-151.59;
//~ theta_Pitch=5.38;
//~ theta_Yaw=-106.95;


//~ theta_Roll=Robot_Home_Orientation(0);
//~ theta_Pitch=Robot_Home_Orientation(1);
//~ theta_Yaw=Robot_Home_Orientation(2);

//Forming Rotation Matrix of Tool Center Point With Respect To Base of the Robot
//~ form_3_point_Rotation_Matrix(P,Q,R,X_Vector_Tool_Base,Y_Vector_Tool_Base,Z_Vector_Tool_Base,Rotation_Tool_Base);
//~ theta_Yaw=theta_Yaw*(3.14/180);
//~ theta_Roll=theta_Roll*(3.14/180);
//~ theta_Pitch=theta_Pitch*(3.14/180);

theta=theta*(3.14/180);


Length_X=0;
Length_Y=0;
Length_Z=0.170;

Rotate_Yaxis << -1,0,0,0,
0,1,0,0,
0,0,-1,0,
0,0,0,1;
Rotate_Xaxis <<  1,0,0,0,
0,-1,0,0,
0,0,-1,0,
0,0,0,1;
Rotate_Zaxis << -1,0,0,0,
0,-1,0,0,
0,0,1,0,
0,0,0,1;
Rotate_Yaxis_Pitch << cos(theta_Pitch),0,sin(theta_Pitch),0,
	0,1,0,0,
-sin(theta_Pitch),0,cos(theta_Pitch),0,
0,0,0,1;
Rotate_Xaxis_Roll <<  1,0,0,0,
0,cos(theta_Roll),-sin(theta_Roll),0,
0,sin(theta_Roll),cos(theta_Roll),0,
0,0,0,1;
Rotate_Zaxis_Yaw << cos(theta_Yaw),-sin(theta_Yaw),0,0,
sin(theta_Yaw),cos(theta_Yaw),0,0,
0,0,1,0,
0,0,0,1;   


//~ Rotation_Matrix_Xaxis_Roll=Rotate_Xaxis_Roll.block<3,3>(0,0);
//~ Rotation_Matrix_Yaxis_Pitch=Rotate_Yaxis_Pitch.block<3,3>(0,0);
//~ Rotation_Matrix_Zaxis_Yaw=Rotate_Zaxis_Yaw.block<3,3>(0,0);








//Roll Pitch Yaw to Rotation Matrix

//~ Rotation_Flange_Base=Eigen::AngleAxisf(theta_Roll,Eigen::Vector3f::UnitX()) 
//~ *Eigen::AngleAxisf(theta_Pitch,Eigen::Vector3f::UnitY())
//~ *Eigen::AngleAxisf(theta_Yaw,Eigen::Vector3f::UnitZ());

//Yaw Pitch Roll to Rotation Matrix

//~ Rotation_Flange_Base=Eigen::AngleAxisf(theta_Yaw,Eigen::Vector3f::UnitZ())
//~ *Eigen::AngleAxisf(theta_Pitch,Eigen::Vector3f::UnitY())
//~ *Eigen::AngleAxisf(theta_Roll,Eigen::Vector3f::UnitX()); 

Tranform_EE_Tool << cos(theta),-sin(theta),0,Length_X,
sin(theta),cos(theta),0,Length_Y,
0,0,1,Length_Z,
0,0,0,1;     
//~ Rotation_Flange_Base=Rotation_Matrix_Zaxis_Yaw*Rotation_Matrix_Yaxis_Pitch*Rotation_Matrix_Xaxis_Roll;
//~ Rotation_Flange_Base=Rotation_Matrix_Xaxis_Roll*Rotation_Matrix_Yaxis_Pitch*Rotation_Matrix_Zaxis_Yaw;
std::cout << "Rotation_Flange_Base=" << std::endl << Rotation_Flange_Base << std::endl;

/*Calculated Tool_to_Base Rotation Matrix Using Three Point Method*/

//~ Rotation_Tool_Base<<-3.86627936e-01, -9.22235437e-01, -7.98818049e-04,
//~ -0.9222325 ,  0.38662873, -0.00233239,
//~ 2.45985877e-03, -1.65071186e-04, -9.99996961e-01;
//~ Rotation_Tool_Base<<-0.26615242,-0.9615056 ,-0.06833643,
//~ -0.82975553,0.2646089,-0.49141417,
//~ 0.4905799,-0.07408854,-0.86824089;
	 //~ Rotation_Tool_Base<<9.99643673e-01,-2.66929499e-02,1.12296802e-04,
			//~ 0.02669313 ,0.99962451,-0.00618923,
			//~ 5.29542924e-05,6.19002676e-03,9.99980840e-01;
//~ Rotation_Tool_Base<<-0.23882053,-0.92424047,-0.29789983,
		 //~ -0.55433749 ,0.38163756,-0.73963688,
		 //~ 0.7972921 ,-0.01150343,-0.60348403;
//~ Rotation_Tool_Base <<-0.28907262,-0.95294283,-0.09130707,
		 //~ -0.82997162,0.29700979,-0.47215707,
		  //~ 0.47705779,-0.0607054 ,-0.8767729;
		 
Rotation_Flange_Base_Angles=Rotation_Tool_Base.eulerAngles(0,1,2);
Rotation_Flange_Base_Angles(0)=(Rotation_Tool_Base(0)*180)/3.14;
Rotation_Flange_Base_Angles(1)=(Rotation_Tool_Base(1)*180)/3.14;
Rotation_Flange_Base_Angles(2)=(Rotation_Tool_Base(2)*180)/3.14;
std::cout << "Rotation_Tool_Base=" << std::endl << Rotation_Flange_Base_Angles << std::endl;
Rotation_Flange_Base_Angles=Rotation_Tool_Base.eulerAngles(2,1,0);
Rotation_Flange_Base_Angles(0)=(Rotation_Tool_Base(0)*180)/3.14;
Rotation_Flange_Base_Angles(1)=(Rotation_Tool_Base(1)*180)/3.14;
Rotation_Flange_Base_Angles(2)=(Rotation_Tool_Base(2)*180)/3.14;
std::cout << "Rotation_Tool_Base=" << std::endl << Rotation_Flange_Base_Angles << std::endl;

Rotation_Tool_Flange=Rotation_Flange_Base.transpose()*Rotation_Tool_Base.transpose();

//~ Convert_to_Homgenous_Matrix(Tranform_EE_Tool,Rotation_Tool_Flange,Position_Tool_Flange);

std::cout << "Tool With Respect To Flange=" << std::endl << Tranform_EE_Tool << std::endl;
Eigen::Vector3f Tool_To_Flange_Angles= Rotation_Tool_Flange.eulerAngles(2,1,0) ;  
std::cout << "Tool With Respect To Flange=" << std::endl << Tool_To_Flange_Angles << std::endl;




/*Calculating Homegenous Transformation Matrix of Tool Center Point with Respect to Base of The Robot at Home Position*/


Rotate_X_axis(Robot_Home_Orientation_x,Rotation_Matrix_Xaxis_Roll);
Rotate_Y_axis(Robot_Home_Orientation_y,Rotation_Matrix_Yaxis_Pitch);
Rotate_Z_axis(Robot_Home_Orientation_z,Rotation_Matrix_Zaxis_Yaw);

Robot_Home_Rotation_Matrix=Rotation_Matrix_Xaxis_Roll*Rotation_Matrix_Yaxis_Pitch*Rotation_Matrix_Zaxis_Yaw;

Convert_to_Homgenous_Matrix(Homogenous_Robot_Home_Pose,Robot_Home_Rotation_Matrix,Robot_Home_Position);                                                                               










/*Provide Current Aruco Marker Pose in the Following Lines of Code to check The Calibration.*/
//~ Aruco_pos[0]= 0.0459329262376;
//~ Aruco_pos[1]= -0.00543490564451;
//~ Aruco_pos[2]=  0.639806032181;
//~ Aruco_rot.x()=-0.0386844292024;
//~ Aruco_rot.y()=0.998593059763;
//~ Aruco_rot.z()=-0.00833455726622;
//~ Aruco_rot.w()=0.0352980323133;


/*Use the Following Variables if Want to Aruco Marker Poses Via ROS Messages*/
//~ Aruco_pos[0]=   Aruco_pose.position.x;
//~ Aruco_pos[1]=    Aruco_pose.position.y;
//~ Aruco_pos[2]= Aruco_pose.position.z;
//~ Aruco_rot.x()=Aruco_pose.orientation.x;
//~ Aruco_rot.y()=Aruco_pose.orientation.y;
//~ Aruco_rot.z()=Aruco_pose.orientation.z;




//~ /*Forming Homogenous Matrix Sensor to Robot flange*/
Eigen::Matrix3f Rotation_robot_to_sensor=Robot_to_Sensor_rot.normalized().toRotationMatrix();
std::cout << "Rotation_robot_to_sensor=" << std::endl << Rotation_robot_to_sensor << std::endl;
Homgenous_Matrix_Robot.setIdentity();
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Robot,Rotation_robot_to_sensor,Robot_to_sensor_pos);
//~ Homgenous_Matrix_Robot.block<3,3>(0,0)=Rotation_robot_to_sensor;
//~ Homgenous_Matrix_Robot.block<3,1>(0,3)=Robot_to_sensor_pos;
std::cout << "Homogenous_Transformation=" << std::endl << Homgenous_Matrix_Robot << std::endl;




//Transformation of Object To Tool Coordinate System Calculation
//~ Object_Transformation.block<3,3>(0,0)=Object_Transformation.block<3,3>(0,0)*Rotate_Yaxis.block<3,3>(0,0);
//~ Object_Transformation=Object_Transformation*Rotate_Y_axis;
float Rotation_Theta;
Eigen::Matrix3f Rotate_Coordinate_System;
Eigen::Matrix4f Homogenous_Rotate_Coordinate_System;
Eigen::Vector3f position;
position[0]=0;
position[1]=0;
position[2]=0;
//~ //if object is straight
Rotation_Theta=-90;
//~ //If object is rotated 90
//~ Rotation_Theta=-180;
Rotate_X_axis(Rotation_Theta,Rotate_Coordinate_System);
Convert_to_Homgenous_Matrix(Homogenous_Rotate_Coordinate_System,Rotate_Coordinate_System,position);
Object_Transformation.block<3,3>(0,0)=Object_Transformation.block<3,3>(0,0)*Rotate_Coordinate_System;
Object_Transformation=Object_Transformation*Homogenous_Rotate_Coordinate_System;
Object_Transformation=Homgenous_Matrix_Robot*Object_Transformation;

//Transformation of Object with Respect to TCP i.e, Final Pose of Robot with Respect to TCP
Eigen::Matrix4f Object_Transformation_Base;
Eigen::Matrix3f Object_Rotation_Base;
Eigen::Vector3f Object_Position_Base;

Object_Transformation=Tranform_EE_Tool.inverse()*Object_Transformation;
std::cout << "Object_Transformation_Respect_To_TCP=" << std::endl << Object_Transformation<< std::endl;






/*Uncomment This Part To Validate The Calibration Using Aruco Marker
* 
* The Following Lines of Code Provides the Poses of Aruco Marker With Respect To Tool Center Point
* */



//Aruco Pose to Transformation Matrix
//~ Eigen::Matrix4f Aruco_transform;
//~ Eigen::Matrix3f Rotation_Aruco=Aruco_rot.normalized().toRotationMatrix();
//~ std::cout << "Rotation_Aruco=" << std::endl << Rotation_robot_to_sensor << std::endl;
//~ Eigen::Vector3f Rotation_Aruco_RPY=Rotation_Aruco.eulerAngles(0,1,2);
//~ std::cout << "Rotation_Aruco_RPY=" << std::endl << Rotation_Aruco_RPY << std::endl;
//~ Rotation_Aruco_RPY(0)=(Rotation_Aruco_RPY(0)*180)/3.14;
//~ Rotation_Aruco_RPY(1)=(Rotation_Aruco_RPY(1)*180)/3.14;
//~ Rotation_Aruco_RPY(2)=(Rotation_Aruco_RPY(2)*180)/3.14;
//~ std::cout << "Rotation_Aruco_RPY=" << std::endl << Rotation_Aruco_RPY << std::endl;
//~ Homgenous_Matrix_Aruco.setIdentity();
//~ Homgenous_Matrix_Aruco.block<3,3>(0,0)=Rotation_Aruco;
//~ Rotation_Aruco=Rotation_Aruco;
//~ Aruco_pos=Aruco_pos-Position_Tool_Flange;
//~ Homgenous_Matrix_Aruco.block<3,1>(0,3)=Aruco_pos;
//~ Rotation_Aruco=Rotation_Aruco*Rotate_Yaxis.block<3,3>(0,0);
//~ Convert_to_Homgenous_Matrix(Homgenous_Matrix_Aruco,Rotation_Aruco,Aruco_pos);
//~ Homgenous_Matrix_Aruco=Homgenous_Matrix_Aruco*Rotate_Yaxis;
//~ std::cout << "Homogenous_Transformation_Aruco=" << std::endl << Homgenous_Matrix_Aruco << std::endl;


//~ //Aruco Transform with Respect to End Effector/Flange
//~ Aruco_transform=Homgenous_Matrix_Robot*Homgenous_Matrix_Aruco;
//~ Aruco_transform=Homgenous_Matrix_Aruco*Homgenous_Matrix_Robot;

//~ std::cout << "Aruco_check=" << std::endl << Aruco_transform<< std::endl;
//~ Eigen::Matrix3f Transformed_Aruco_Rotation= Aruco_transform.block<3,3>(0,0);
//~ Eigen::Vector3f Aruco_Transformed_RPY=Transformed_Aruco_Rotation.eulerAngles(0,1,2);
//~ Eigen::Vector3f Aruco_Transformed_Pos=Aruco_transform.block<3,1>(0,3);
//~ Aruco_pos_Tool=Aruco_Transformed_Pos-Position_Tool_Flange;
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;
//~ Aruco_Transformed_RPY(0)=(Aruco_Transformed_RPY(0)*180)/3.14;
//~ Aruco_Transformed_RPY(1)=(Aruco_Transformed_RPY(1)*180)/3.14;
//~ Aruco_Transformed_RPY(2)=(Aruco_Transformed_RPY(2)*180)/3.14;
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;
//~ //Aruco Transform with Respect to Gripping Tool

//~ Eigen::Matrix4f Aruco_transform_EE;
//~ Eigen::Matrix4f Aruco_transform_EE= Tranform_EE_Tool.inverse()*Aruco_transform;
//~ std::cout << "Aruco_check_EE_Transformation_Matrix=" << std::endl << Aruco_transform_EE<< std::endl;
//~ Eigen::Matrix3f Transformed_Aruco_Rotation_EE= Aruco_transform_EE.block<3,3>(0,0);
//~ Eigen::Vector3f Aruco_Transformed_RPY_EE=Transformed_Aruco_Rotation_EE.eulerAngles(0,1,2);
//~ std::cout << "Aruco_Transformed_RPY_EE=" << std::endl << Aruco_Transformed_RPY_EE << std::endl;
//~ std::cout << "Aruco_Transformed_RPY_EE=" << std::endl << Transformed_Aruco_Rotation_Tool << std::endl;
//~ Aruco_Transformed_RPY_EE(0)=(Aruco_Transformed_RPY_EE(0)*180)/3.14;
//~ Aruco_Transformed_RPY_EE(1)=(Aruco_Transformed_RPY_EE(1)*180)/3.14;
//~ Aruco_Transformed_RPY_EE(2)=(Aruco_Transformed_RPY_EE(2)*180)/3.14;
//~ std::cout << "Aruco_Transformed_RPY_EE=" << std::endl << Aruco_Transformed_RPY_EE << std::endl;


}


/*
* This Method is used to Calculate 3DRotation Matrix using Point Normal Vector and Rectangle Coordinate Points of the object 
* */
void form_Rotation_Matrix_Normal(pcl::PointXYZ Point_1,pcl::PointXYZ Point_2,pcl::PointXYZ Point_3,Eigen::Vector3f &X_Vector,Eigen::Vector3f &Y_Vector,Eigen::Vector3f Z_Vector,Eigen::Matrix3f &Rotation_matrix)
{
X_Vector[0]=Point_2.x-Point_1.x;
X_Vector[1]=Point_2.y-Point_1.y;
//~ X_Vector[2]=Point_2.z-Point_1.z;
X_Vector[2]=0;
Z_Vector.normalize();
X_Vector.normalize();
Y_Vector=X_Vector.cross(Z_Vector);	

//~ Y_Vector=Z_Vector.cross(X_Vector);

cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Y_Vector)	;
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Z_Vector)	;
cout<<"Orthonormality check....."<<endl<<Y_Vector.dot(Z_Vector)	;	

Rotation_matrix.block<3,1>(0,0)=X_Vector;
Rotation_matrix.block<3,1>(0,1)=Y_Vector;
Rotation_matrix.block<3,1>(0,2)=Z_Vector;
std::cout << "Rotation_matrix="<<std::endl<<Rotation_matrix<<std::endl;

}

/*This Method is to find The pose using Normal Vector and Orthogonal Vectors.*/
void form_Rotation_Matrix_Ortho_Normal(pcl::PointXYZ Point_1,pcl::PointXYZ Point_2,pcl::PointXYZ Point_3,Eigen::Vector3f &X_Vector,Eigen::Vector3f &Y_Vector,Eigen::Vector3f Z_Vector,Eigen::Matrix3f &Rotation_matrix)
{
X_Vector[0]=0;
X_Vector[1]=1;
X_Vector[2]=0;
//~ X_Vector[2]=0;
//~ Eigen::Vector3f translation_vector;
//~ translation_vector[0]=Point_1.x;
//~ translation_vector[1]=Point_1.y;
//~ translation_vector[2]=Point_1.z;

//~ X_Vector=X_Vector.normalized();
//~ X_Vector=Z_Vector.cross(translation_vector);
//~ Y_Vector[0]=Point_3.x-Point_1.x;
//~ Y_Vector[1]=Point_3.y-Point_1.y;
//~ Y_Vector[2]=Point_3.z-Point_1.z;
//~ Y_Vector=Y_Vector.normalized();
//~ Z_Vector=X_Vector.cross(Y_Vector);
X_Vector=X_Vector.cross(Z_Vector);
Z_Vector.normalize();
X_Vector.normalize();
Y_Vector=Z_Vector.cross(X_Vector);	



//~ Y_Vector=Z_Vector.cross(X_Vector);

cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Y_Vector)	;
cout<<"Orthonormality check....."<<endl<<X_Vector.dot(Z_Vector)	;
cout<<"Orthonormality check....."<<endl<<Y_Vector.dot(Z_Vector)	;	



Rotation_matrix.block<3,1>(0,0)=X_Vector;
Rotation_matrix.block<3,1>(0,1)=Y_Vector;
Rotation_matrix.block<3,1>(0,2)=Z_Vector;
std::cout << "Rotation_matrix="<<std::endl<<Rotation_matrix<<std::endl;

}
/*
* This Method Is used to all the Operations of Point Cloud Data
* */

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{

//~ m.lock();

pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZ>);



//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(*cloud_msg, *cloud_p2);
//cout<<*cloud_p2;

pcl::fromPCLPointCloud2(*cloud_p2, *cloud_pt);
std::vector<int> indices;
pcl::removeNaNFromPointCloud(*cloud_pt, *cloud_out, indices);


pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree3;

//~ octree.setInputCloud (cloud.makeShared());
kdtree.setInputCloud(cloud_pt);
kdtree1.setInputCloud(cloud_pt);
kdtree2.setInputCloud(cloud_pt);
kdtree3.setInputCloud(cloud_pt);
//kdtree.addPointsFromInputCloud ();

//pcl::PointXYZ center_point;
//center_point.x = 0 ;
//center_point.y = 0.4;
//center_point.z = -1.4;

float radius = 0.01;//search in sphere of 1 cm           
std::vector<int> radiusIdx;
std::vector<float> radiusSQDist;

Eigen::Vector4f plane_parameters; 
float curvature; 

Eigen::Vector4f plane_parameters1; 
float curvature1; 

Eigen::Vector4f plane_parameters2; 
float curvature2; 
Eigen::Vector4f plane_parameters3; 
float curvature3;          
map_to_3d_pointcloud(cloud_pt,object1_grasp_point);

map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_1);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_2);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_3);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_4);

cout<<"\n"<<object1_grasp_point;
cout<<"\n"<<object1_rect_coordinates_1;
cout<<"\n"<<object1_rect_coordinates_2;
cout<<"\n"<<object1_rect_coordinates_3;


std_msgs::Float32MultiArray array;
Point_bat_box.x=object1_grasp_point.x;
Point_bat_box.y=object1_grasp_point.y;
Point_bat_box.z=object1_grasp_point.z;


if (kdtree.radiusSearch (object1_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{            
cout<<"YES!";
//~ for(size_t i=0;i<radiusIdx.size();++i)
//~ {
//~ cout<<" "<< cloud_pt->points[radiusIdx[i]].x << "  " <<	cloud_pt->points[radiusIdx[i]].y << "  " << cloud_pt->points[radiusIdx[i]].z <<  "squared distance: " << radiusIdx[i] << ")"<<endl;
//~ }
}                                     

computePointNormal(*cloud_pt,radiusIdx,plane_parameters,curvature); 
//~ flipNormalTowardsViewpoint (object1_grasp_point, 0, 0, 0, plane_parameters);

cout<<"param: "<<plane_parameters<<endl;
cout<<"curvature: "<<curvature<<endl;



Orientation_batbox.x=(double)plane_parameters[0];
Orientation_batbox.y=(double)plane_parameters[1];
Orientation_batbox.z=(double)plane_parameters[2];
Orientation_batbox.w=(double)plane_parameters[3];

Eigen::Quaternionf quat(plane_parameters);
Eigen::Vector3f position(object1_grasp_point.x,object1_grasp_point.y,object1_grasp_point.z);
Eigen::Vector3f vector;
Eigen::Vector3f x_vector_object1;
Eigen::Vector3f y_vector_object1;
Eigen::Vector3f z_vector_object1;

Eigen::Matrix3f Rotation_object1;
vector(0)=(double)plane_parameters[0];
vector(1)=(double)plane_parameters[1];
vector(2)=(double)plane_parameters[2];
//~ Rotation_object1=quat.normalized().toRotationMatrix();
form_3_point_Rotation_Matrix(object1_grasp_point,object1_rect_coordinates_3,object1_rect_coordinates_4,x_vector_object1,y_vector_object1,z_vector_object1,Rotation_object1);
//~ form_Rotation_Matrix_Normal(object1_grasp_point,object1_rect_coordinates_3,object1_rect_coordinates_4,x_vector_object1,y_vector_object1,vector,Rotation_object1);
//~ form_Rotation_Matrix_Ortho_Normal(object1_grasp_point,object1_rect_coordinates_3,object1_rect_coordinates_4,x_vector_object1,y_vector_object1,vector,Rotation_object1);


std::cout << "Rotation_object1="<<std::endl<<Rotation_object1<<std::endl;
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object1,Rotation_object1,position);
cout<<"\n"<<"Homgenous_Matrix_Sensor_Object1="<<Homgenous_Matrix_Sensor_Object1;
Eigen::Matrix4f Transformed_Object_to_Robot=Homgenous_Matrix_Sensor_Object1;
transformation_to_Tool_Center_Point(Transformed_Object_to_Robot);
cout<<"\n"<<Transformed_Object_to_Robot;
Eigen::Matrix3f Transformed_Rotation_object1=Transformed_Object_to_Robot.block<3,3>(0,0);
Eigen::Vector3f Transformed_Position_object1=Transformed_Object_to_Robot.block<3,1>(0,3);
Eigen::Vector3f orientation_in_rpy=Transformed_Rotation_object1.eulerAngles(0,1,2);
//~ Eigen::Vector3f Object_Rotation_Base_vector;
//~ Transformed_Rotation_object1=Transformed_Rotation_object1.transpose();
Eigen::Matrix3f Roll;
Eigen::Matrix3f Pitch;
Eigen::Matrix3f Yaw;

float object1_rx=orientation_in_rpy(0);
float object1_ry=orientation_in_rpy(1);
float object1_rz=orientation_in_rpy(2);

Rotate_X_axis(object1_rx,Roll);
Rotate_Y_axis(object1_ry,Pitch);
Rotate_Z_axis(object1_rz,Yaw);

Eigen::Matrix3f Transformed_Rotation_object1_rotation_rpy;
Transformed_Rotation_object1_rotation_rpy=Yaw * Pitch *Roll ;
//~ Transformed_Rotation_object1_rotation_rpy=Roll*Pitch*Yaw;


/*Converting RPY To rotation Vector */

//~ float object1_rx;
//~ float object1_ry;
//~ float object1_rz;
cout<<"\n"<<"Rotation_object1_angle_axisrx"<<"\n"<<Transformed_Rotation_object1_rotation_rpy<<endl;

float multiaxis;
float Object_Rotation_theta;
Object_Rotation_theta=acos((Transformed_Rotation_object1_rotation_rpy(0,0)+Transformed_Rotation_object1_rotation_rpy(1,1)+Transformed_Rotation_object1_rotation_rpy(2,2)-1)/2);
multiaxis=1/(2*sin(Object_Rotation_theta));

object1_rx=multiaxis*(Transformed_Rotation_object1_rotation_rpy(2,1)-Transformed_Rotation_object1_rotation_rpy(1,2))*Object_Rotation_theta;
object1_ry=multiaxis*(Transformed_Rotation_object1_rotation_rpy(0,2)-Transformed_Rotation_object1_rotation_rpy(2,0))*Object_Rotation_theta;
object1_rz=multiaxis*(Transformed_Rotation_object1_rotation_rpy(1,0)-Transformed_Rotation_object1_rotation_rpy(0,1))*Object_Rotation_theta;




cout<<"\n"<<"Rotation_object1_angle_axisrx"<<"\n"<<object1_rx<<endl;
cout<<"\n"<<"Rotation_object1_angle_axisry"<<"\n"<<object1_ry<<endl;
cout<<"\n"<<"Rotation_object1_angle_axisrz"<<"\n"<<object1_rz<<endl;
cout<<"\n"<<"Rotation_object1_in radians"<<"\n"<<orientation_in_rpy<<endl;


orientation_in_rpy(0)=(orientation_in_rpy(0)*180)/3.14;
orientation_in_rpy(1)=(orientation_in_rpy(1)*180)/3.14;
orientation_in_rpy(2)=(orientation_in_rpy(2)*180)/3.14;





cout<<"\n"<<"Rotation_object1"<<"\n"<<orientation_in_rpy<<endl;
cout<<"\n"<<"Position_object1"<<Transformed_Position_object1<<endl;
pose_bat_box.position.x=Transformed_Position_object1[0];
pose_bat_box.position.y=Transformed_Position_object1[1];
pose_bat_box.position.z=Transformed_Position_object1[2];
pose_bat_box.orientation.x=object1_rx;
pose_bat_box.orientation.y=object1_ry;
pose_bat_box.orientation.z=object1_rz;




//Publising Final_pose obtained after Transformation
object_pose.publish(pose_bat_box);



pcl::toROSMsg(*cloud_out, output);
pub.publish(output);

//  m.lock();


//~ m.unlock();

// Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
pcl::fromPCLPointCloud2(*cloud_p2,*temp_cloud);
std::vector<int> indices2 (floor (cloud_pt->points.size () / 10));
for (size_t i = 0; i < indices2.size (); ++i) indices2[i] = i;

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
pcl::toROSMsg(*normals,output2);
//~ pub3.publish(output2);  

//}
// normalsVis(cloud_out,cloud_normals1);
pcl::PCLPointCloud2 cloud_filtered;
// Perform the actual filtering
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud (cloudPtr);
//~ sor.setSearchSurface(cloud_pt);
sor.setLeafSize (0.03, 0.03, 0.03);
sor.filter (cloud_filtered);
pcl::fromPCLPointCloud2(cloud_filtered,*cloud_filter);
// Convert to ROS data type
sensor_msgs::PointCloud2 output5;
pcl_conversions::fromPCL(cloud_filtered, output5);
//Get normals of filtered output
pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;

ne2.setInputCloud(cloud_filter);
ne2.setSearchSurface(cloud_pt);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ()); 
ne2.setSearchMethod (tree2);
ne2.setRadiusSearch (0.03);
ne2.setKSearch(0);
ne2.compute(*normals2);
cout<<"\n"<<normals2->points.size();
// Publish the data
pub5.publish (output5);
normalsVis(cloud_pt,normals,object1_grasp_point,Rotation_object1);
}

/*
* This method is to get Grasping Points of Object 1
* @parameter[IN] coordinate point(X,Y,Z) of object 1
* */
void  normal_estimate_cb(const geometry_msgs::Point::ConstPtr& msg)
{

object1_grasp_point.x =(msg->x) ;
object1_grasp_point.y =(msg->y);
object1_grasp_point.z = (msg->z);
Depth_Point.z=(msg->z);
//~ object1_grasp_point.x =(double)(msg->x) ;
//~ object1_grasp_point.y =(double)(msg->y);
//~ object1_grasp_point.z = (msg->z);
//~ Depth_Point.z=(msg->z);
//~ std::cout<<"Object1_Point="<<object1_grasp_point;
}
/*
* This Method to Recieving Grasping Rectangle Coordinate Points of the Object
* From the Recieved Rectangle Coordinates Midpoint of Vetices is Found which Later used to define the Pose of the object
* @parameter[IN] Object Rectangle Cordinates Point 1 & Point2
* */

void  grasp_rectange_1(const std_msgs::Float32MultiArray::ConstPtr & msg)
{


//~ for(int i=0;i<8;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }

object1_rect_coordinates_1.x=msg->data[0];
object1_rect_coordinates_1.y=msg->data[1];
object1_rect_coordinates_2.x=msg->data[2];
object1_rect_coordinates_2.y=msg->data[3];

object1_rect_coordinates_3.x=(msg->data[0]+msg->data[2])/2;
object1_rect_coordinates_3.y=(msg->data[1]+msg->data[3])/2;
object1_rect_coordinates_4.x=(msg->data[2]+msg->data[4])/2;
object1_rect_coordinates_4.y=(msg->data[3]+msg->data[5])/2;
//~ object1_rect_coordinates_3.x=((msg->data[0]+msg->data[2])/2)/1000;
//~ object1_rect_coordinates_3.y=((msg->data[1]+msg->data[3])/2)/1000;
//~ object1_rect_coordinates_4.x=((msg->data[2]+msg->data[4])/2)/1000;
//~ object1_rect_coordinates_4.y=((msg->data[3]+msg->data[5])/2)/1000;
}


/*
* This Method Recieves Intrinsic Camera Calibration Results
* It can be to Map the Pixel Point to the Point Cloud
* @parameter[IN] Sensor_msgs::CameraInfo
* */

void camera_info(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
fx=msg->K[0];
cx=msg->K[2];
fy=msg->K[4];
cy=msg->K[5];

}




//~ void Aruco_pose_cb (const geometry_msgs::Pose::ConstPtr & msg)
//~ {
//~ Aruco_pose.position=msg->position;
//~ Aruco_pose.orientation=msg->orientation;
//~ cout<<"Aruco Pose:"<<endl<<Aruco_pose;
//~ }




/*
* 
* Main Function Where all the Subcribers and Publishers are Defined
* */
int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl");
ros::NodeHandle nh;
geometry_msgs::Point search;

pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
pub2 = nh.advertise<geometry_msgs::Point> ("output2", 1);
object_points = nh.advertise<geometry_msgs::Point> ("Cameraspace_point_batbox", 1);
object_pose=nh.advertise<geometry_msgs::Pose>("pose_object", 1);
pub3 = nh.advertise< sensor_msgs::PointCloud2> ("output3", 1);
pub5 = nh.advertise< sensor_msgs::PointCloud2> ("output5", 1);
pub4 = nh.advertise<geometry_msgs::Quaternion> ("output4", 1);
pub6 = nh.advertise<geometry_msgs::Point> ("output6", 1);

// Create a ROS subscriber for the input point cloud
ros::Subscriber camera = nh.subscribe<sensor_msgs::CameraInfo> ("/kinect2/qhd/camera_info", 1, camera_info);
ros::Subscriber pt = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_test", 1, normal_estimate_cb);
//~ ros::Subscriber aruco_sub=nh.subscribe<geometry_msgs::Pose> ("/aruco_single/pose",1,Aruco_pose_cb);
ros::Subscriber pt_4 = nh.subscribe<std_msgs::Float32MultiArray> ("/sim_grasp/grasp_rect_points_test", 1, grasp_rectange_1);
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1, cloud_cb);

ros::spin();
}
