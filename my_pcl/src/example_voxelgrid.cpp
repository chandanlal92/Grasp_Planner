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
ros::Publisher pub_batbox_points;
ros::Publisher pub_staple_points;
ros::Publisher pub_test_points;
ros::Publisher pub_bombe_points;
ros::Publisher pub_batbox_pose;
ros::Publisher pub_staple_pose;
ros::Publisher pub_test_pose;
ros::Publisher pub_bombe_pose;
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
geometry_msgs::Point Point_staple;
geometry_msgs::Point Point_test;
geometry_msgs::Point Point_bombe;
pcl::PointXYZ object1_grasp_point;
pcl::PointXYZ object2_grasp_point;
pcl::PointXYZ object3_grasp_point;
pcl::PointXYZ object4_grasp_point;
pcl::PointXYZ object1_rect_coordinates_1;
pcl::PointXYZ object1_rect_coordinates_2;
pcl::PointXYZ object1_rect_coordinates_3;
pcl::PointXYZ object2_rect_coordinates_1;
pcl::PointXYZ object2_rect_coordinates_2;
pcl::PointXYZ object2_rect_coordinates_3;
pcl::PointXYZ object3_rect_coordinates_1;
pcl::PointXYZ object3_rect_coordinates_2;
pcl::PointXYZ object3_rect_coordinates_3;
pcl::PointXYZ object4_rect_coordinates_1;
pcl::PointXYZ object4_rect_coordinates_2;
pcl::PointXYZ object4_rect_coordinates_3;

geometry_msgs::Quaternion Orientation_batbox;
geometry_msgs::Quaternion Orientation_staple;
geometry_msgs::Quaternion Orientation_test;
geometry_msgs::Quaternion Orientation_bombe;
geometry_msgs::Pose pose_bat_box;
geometry_msgs::Pose pose_staple;
geometry_msgs::Pose pose_test;
geometry_msgs::Pose pose_bombe; 
geometry_msgs::Pose Sensor;



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
Eigen::Quaternionf Aruco_rot;
Eigen::Matrix4f Homgenous_Matrix_Robot;
Eigen::Matrix4f Homgenous_Matrix_Robot_Inverse;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object1;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object2;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object3;
Eigen::Matrix4f Homgenous_Matrix_Sensor_Object4;
Eigen::Matrix4f Homgenous_Matrix_Aruco;





//~ double cx=0;
//~ double cy=0;
//~ double fx=1;
//~ double fy=1;
//~ double cx=9.7516594281244113e+02;
//~ double cy=5.4984180054452986e+02;
//~ double fx= 1.0648053153404119e+03;
//~ double fy=1.0637996673435098e+03;
//~ double depthshift= -3.6049205941883486e+01;

//~ double cx=2.5534440760366272e+02;
//~ double cy=2.0274614274703237e+02;
//~ double fx=3.6275749422389919e+02;
//~ double fy=3.6263845077543624e+02;
//~ double fx=5.2921508098293293e+02;
//~ double fy=5.2556393630057437e+02;
//~ double cx=.2894272028759258e+02;
//~ double cy=2.6748068171871557e+02;
//~ double cx= 2.6189690256164300e+02;
//~ double cy= 2.1108418348118809e+02;
//~ double fx= 3.6346922199813889e+02;
//~ double fy= 3.6300739824951370e+02;
//~ double cx = 254.878;
//~ double cy = 205.395;
//~ double fx = 365.456;
//~ double fy = 365.456;

//~ static float cx = 254.878f;
//~ static float cy = 205.395f;
//~ static float fx = 365.456f;
//~ static float fy = 365.456f;
//~ static float k1 = 0.0905474;
//~ static float k2 = -0.26819;
//~ static float k3 = 0.0950862;
double px=-5.2052476112081990e-02;
double py=-4.6313865353939110e-04;
double pz=8.8806735554907584e-04;
boost::mutex m;

/*
 * Method to get the Surface Normal of Grasping points
 * */
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::PointXYZ center,pcl::PointXYZ center1,pcl::PointXYZ center2,pcl::PointXYZ center3, Eigen::Matrix3f matrix_1,Eigen::Matrix3f matrix_2,Eigen::Matrix3f matrix_3,Eigen::Matrix3f matrix_4)
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
pcl::PointXYZ center_1 (center1.x,center1.y,center1.z);
pcl::PointXYZ center_2 (center2.x,center2.y,center2.z);
pcl::PointXYZ center_3 (center3.x,center3.y,center3.z);
/*
 * Defining X,Y,Z Axis in the From the Vector Form 
 * 
 * */


 //~ pcl::PointXYZ x_axis (matrix_1(0,0), matrix_1(0,1), matrix_1(0,2));
 //~ pcl::PointXYZ x_axis1(matrix_2(0,0), matrix_2(0,1), matrix_2(0,2));
 //~ pcl::PointXYZ x_axis2(matrix_3(0,0), matrix_3(0,1), matrix_3(0,2));
 pcl::PointXYZ x_axis3(matrix_4(0,0), matrix_4(0,1), matrix_4(0,2));

 //~ pcl::PointXYZ y_axis (matrix_1(1,0), matrix_1(1,1), matrix_1(1,2));
 //~ pcl::PointXYZ y_axis1(matrix_2(1,0), matrix_2(1,1), matrix_2(1,2));
 //~ pcl::PointXYZ y_axis2(matrix_3(1,0), matrix_3(1,1), matrix_3(1,2));
 pcl::PointXYZ y_axis3(matrix_4(1,0), matrix_4(1,1), matrix_4(1,2));
 //~ pcl::PointXYZ z_axis (matrix_1(2,0), matrix_1(2,1), matrix_1(2,2));
 //~ pcl::PointXYZ z_axis1(matrix_2(2,0), matrix_2(2,1), matrix_2(2,2));
 //~ pcl::PointXYZ z_axis2(matrix_3(2,0), matrix_3(2,1), matrix_3(2,2));
 pcl::PointXYZ z_axis3(matrix_4(2,0), matrix_4(2,1), matrix_4(2,2));
 

//~ viewer->addLine (center_0, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"+1);
//~ viewer->addLine (center_1, x_axis1, 1.0f, 0.0f, 0.0f, "major eigen vector1"+2);
//~ viewer->addLine (center_2, x_axis2, 1.0f, 0.0f, 0.0f, "major eigen vector2"+3);
viewer->addLine (center_3, x_axis3, 1.0f, 0.0f, 0.0f, "major eigen vector9"+3);
//~ viewer->addLine (center_0, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector3"+4);
//~ viewer->addLine (center_1, y_axis1, 0.0f, 1.0f, 0.0f, "middle eigen vector4"+5);
//~ viewer->addLine (center_2, y_axis2, 0.0f, 1.0f, 0.0f, "middle eigen vector5"+6);
viewer->addLine (center_3, y_axis3, 0.0f, 1.0f, 0.0f, "middle eigen vector10"+6);
//~ viewer->addLine (center_0, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector6"+7);
//~ viewer->addLine (center_1, z_axis1, 0.0f, 0.0f, 1.0f, "minor eigen vector7"+8);
//~ viewer->addLine (center2, z_axis2, 0.0f, 0.0f, 1.0f, "minor eigen vector8"+9);
viewer->addLine (center3, z_axis3, 0.0f, 0.0f, 1.0f, "minor eigen vector11"+9);
viewer->addCoordinateSystem (0.5);
//~ viewer->initCameraParameters ();
while(!viewer->wasStopped())
{
viewer->spinOnce (100);
boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
return(viewer);
m.unlock();
}

void map_to_3d_pointcloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,pcl::PointXYZ &search_Point)
{
	
	
	int index;
	index=search_Point.x+search_Point.y*cloud->width;
	search_Point.x=(double)cloud->points[index].x;
	search_Point.y=(double)cloud->points[index].y;
	search_Point.z=(double)cloud->points[index].z;
	cout<<"\n"<<search_Point;
	
}
void Convert_to_Homgenous_Matrix(Eigen::Matrix4f &Homogenous_Matrix,Eigen::Matrix3f Rotation_vector,Eigen::Vector3f Position_vector)
{
	Homogenous_Matrix.setIdentity();
	Homogenous_Matrix.block<3,3>(0,0)=Rotation_vector;
	Homogenous_Matrix.block<3,1>(0,3)=Position_vector;
	

}




void transformation_to_Tool_Center_Point(Eigen::Matrix4f &Object_Transformation)
{
	
Robot_to_sensor_pos[0]=-0.00675417640812;
Robot_to_sensor_pos[1]=-0.197975798488;
Robot_to_sensor_pos[2]=0.104342621318;
Robot_to_Sensor_rot.x()= -0.222353848288;
Robot_to_Sensor_rot.y()=-0.0181815745267;
Robot_to_Sensor_rot.z()=-0.241667191857;
Robot_to_Sensor_rot.w()= 0.944364953225;

//-----------------------------Latest=id:26 size:0.1 Correct Aruco:Orientation
//~ translation: 
  //~ x: -0.00675417640812
  //~ y: -0.197975798488
  //~ z: 0.104342621318
//~ rotation: 
  //~ x: -0.222353848288
  //~ y: -0.0181815745267
  //~ z: -0.241667191857
  //~ w: 0.944364953225

//~ -----------------Not good
//~ translation: 
  //~ x: 0.0334492309467
  //~ y: -0.170330213031
  //~ z: 0.024185969944
//~ rotation: 
  //~ x: -0.198004088257
  //~ y: -0.0463932139165
  //~ z: -0.126236018744
  //~ w: 0.970930748461
//-------------------------------Not tested
//~ translation: 
  //~ x: 0.0417714175598
  //~ y: -0.169849683207
  //~ z: 0.159927540802
//~ rotation: 
  //~ x: -0.222767633884
  //~ y: -0.0355118312671
  //~ z: -0.111964722801
  //~ w: 0.967769286546

//---------------------------Good result
//~ translation: 
  //~ x: 0.0374645684294
  //~ y: -0.188843173616
  //~ z: 0.102134225027
//~ rotation: 
  //~ x: -0.232834879332
  //~ y: -0.0559402158099
  //~ z: -0.136531486598
  //~ w: 0.961258427473
 // --------------------------------
//End-effector to sensor
//~ x: 0.0342034887874
  //~ y: -0.183062885495
  //~ z: 0.116670529522
//~ rotation: 
  //~ x: -0.287611364106
  //~ y: -0.0344720060903
  //~ z: -0.123531256316
  //~ w: 0.94912138989
//----------------------------------------------


//~ translation: 
  //~ x: 0.0197420169644
  //~ y: -0.158139913703
  //~ z: -0.0937878798637
//~ rotation: 
  //~ x: -0.225707486058
  //~ y: -0.0108939311425
  //~ z: -0.120640878944
  //~ w: 0.966635004192
  //translation: 
  //~ x: 0.0192393426514
  //~ y: -0.201144198921
  //~ z: -0.0551939155751
//~ rotation: 
  //~ x: -0.272196229128
  //~ y: -0.0179160979211
  //~ z: -0.134158512861
  //~ w: 0.952675033635
//~ Eigen::Vector4f Aruco_check;
 Eigen::Matrix4f Aruco_check1;
 Eigen::Matrix4f Rotate_Yaxis;
 Eigen::Matrix4f Rotate_Xaxis;
 Eigen::Matrix4f Rotate_Zaxis;
  Eigen::Matrix4f Rotate_Yaxis_Pitch;
 Eigen::Matrix4f Rotate_Xaxis_Roll;
 Eigen::Matrix4f Rotate_Zaxis_Yaw;
 Eigen::Matrix4f Tranform_EE_Tool;

 float theta,Length_X,theta_Yaw,theta_Roll,theta_Pitch,Length_Y,Length_Z;
 theta=-10;
 theta_Yaw=-theta_Yaw*(3.14/180);
 theta_Roll=-theta_Roll*(3.14/180);
 theta_Pitch=-theta_Pitch*(3.14/180);

 theta=-theta*(3.14/180);

 Length_X=0;
 Length_Y=0.020;
 Length_Z=0.175;
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
                  
 
 Tranform_EE_Tool << cos(theta),-sin(theta),0,0,
                      sin(theta),cos(theta),0,Length_Y,
                      0,0,1,Length_Z,
                      0,0,0,1;     
                  
Aruco_check1 << 1,0,0,0.0512193329632,
                0,1,0,0.0348199866712,
                0,0,1, 0.345557838678,
                0,0,0,1; 
                Aruco_pos[0]=  -0.187098443508;
                Aruco_pos[1]=  -0.0904397368431;
                Aruco_pos[2]=   0.535137951374;
                Aruco_rot.x()= 0.0176874145224;
                Aruco_rot.y()=0.996359313308;
                Aruco_rot.z()=  0.0833750635366;
                Aruco_rot.w()=-0.00196797657761;
     //~ position: 
    //~ x: -0.187098443508
    //~ y: -0.0904397368431
    //~ z: 0.535137951374
  //~ orientation: 
    //~ x: 0.0176874145224
    //~ y: 0.996359313308
    //~ z: 0.0833750635366
    //~ w: -0.00196797657761
//~ ---





//~ Aruco_check(0)=51.2193329632;
//~ Aruco_check(1)=34.8199866712;
//~ Aruco_check(2)=345.557838678;
//~ Aruco_check(3)=1;
//~ Aruco_check1=(51.2193329632,34.8199866712,345.557838678,1);

Eigen::Matrix3f Rotation_robot_to_sensor=Robot_to_Sensor_rot.normalized().toRotationMatrix();
std::cout << "Rotation_robot_to_sensor=" << std::endl << Rotation_robot_to_sensor << std::endl;
Eigen::Matrix4f Aruco_transform;
Homgenous_Matrix_Robot.setIdentity();
Homgenous_Matrix_Robot.block<3,3>(0,0)=Rotation_robot_to_sensor;
Homgenous_Matrix_Robot.block<3,1>(0,3)=Robot_to_sensor_pos;
std::cout << "Homogenous_Transformation=" << std::endl << Homgenous_Matrix_Robot << std::endl;
//~ Homgenous_Matrix_Robot_Inverse=Homgenous_Matrix_Robot.inverse();
//~ std::cout << "Homogenous_Transformation Inverse=" << std::endl << Homgenous_Matrix_Robot_Inverse << std::endl;
Object_Transformation=Object_Transformation*Rotate_Yaxis;
Object_Transformation=Homgenous_Matrix_Robot*Object_Transformation;	
Eigen::Matrix4f Object_Transformation_EE=Tranform_EE_Tool.inverse()*Object_Transformation;
std::cout << "Object_Transformation_EE=" << std::endl << Object_Transformation_EE << std::endl;
 //~ Eigen::Matrix3f Transformed_Aruco_Rotation= Aruco_transform.block<3,3>(0,0);
//~ Eigen::Vector3f Aruco_Transformed_RPY=Transformed_Aruco_Rotation.eulerAngles(0,1,2);
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;
//~ Aruco_Transformed_RPY(0)=(Aruco_Transformed_RPY(0)*180)/3.14;
//~ Aruco_Transformed_RPY(1)=(Aruco_Transformed_RPY(1)*180)/3.14;
//~ Aruco_Transformed_RPY(2)=(Aruco_Transformed_RPY(2)*180)/3.14;
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;

//

//~ Eigen::Matrix3f Rotation_Aruco=Aruco_rot.normalized().toRotationMatrix();
//~ std::cout << "Rotation_Aruco=" << std::endl << Rotation_robot_to_sensor << std::endl;
//~ Eigen::Vector3f Rotatoion_Aruco_RPY=Rotation_Aruco.eulerAngles(0,1,2);
//~ std::cout << "Rotatoion_Aruco_RPY=" << std::endl << Rotatoion_Aruco_RPY << std::endl;
//~ Rotatoion_Aruco_RPY(0)=(Rotatoion_Aruco_RPY(0)*180)/3.14;
//~ Rotatoion_Aruco_RPY(1)=(Rotatoion_Aruco_RPY(1)*180)/3.14;
//~ Rotatoion_Aruco_RPY(2)=(Rotatoion_Aruco_RPY(2)*180)/3.14;
//~ std::cout << "Rotatoion_Aruco_RPY=" << std::endl << Rotatoion_Aruco_RPY << std::endl;

//~ Homgenous_Matrix_Aruco.setIdentity();
//~ Homgenous_Matrix_Aruco.block<3,3>(0,0)=Rotation_Aruco;
//~ Homgenous_Matrix_Aruco.block<3,1>(0,3)=Aruco_pos;
//~ std::cout << "Homogenous_Transformation_Aruco=" << std::endl << Homgenous_Matrix_Aruco << std::endl;
//~ Homgenous_Matrix_Aruco=Homgenous_Matrix_Aruco*Rotate_Yaxis;
//~ std::cout << "Rotated_Y_axis_Homogenous_Transformation_Aruco=" << std::endl << Homgenous_Matrix_Aruco << std::endl;
//~ Aruco_transform=Homgenous_Matrix_Robot*Homgenous_Matrix_Aruco;
//~ std::cout << "Aruco_check=" << std::endl << Aruco_transform<< std::endl;
//~ Eigen::Matrix3f Transformed_Aruco_Rotation= Aruco_transform.block<3,3>(0,0);
//~ Eigen::Vector3f Aruco_Transformed_RPY=Transformed_Aruco_Rotation.eulerAngles(0,1,2);
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;
//~ Aruco_Transformed_RPY(0)=(Aruco_Transformed_RPY(0)*180)/3.14;
//~ Aruco_Transformed_RPY(1)=(Aruco_Transformed_RPY(1)*180)/3.14;
//~ Aruco_Transformed_RPY(2)=(Aruco_Transformed_RPY(2)*180)/3.14;
//~ std::cout << "Aruco_Transformed_RPY=" << std::endl << Aruco_Transformed_RPY << std::endl;

//~ Eigen::Matrix4f Aruco_transform_EE= Tranform_EE_Tool.inverse()*Aruco_transform;
//~ std::cout << "Aruco_check_EE=" << std::endl << Aruco_transform_EE<< std::endl;
//~ Eigen::Matrix3f Transformed_Aruco_Rotation_EE= Aruco_transform_EE.block<3,3>(0,0);
//~ Eigen::Vector3f Aruco_Transformed_RPY_EE=Transformed_Aruco_Rotation_EE.eulerAngles(0,1,2);
//~ std::cout << "Aruco_Transformed_RPY_EE=" << std::endl << Aruco_Transformed_RPY_EE << std::endl;
//~ Aruco_Transformed_RPY_EE(0)=(Aruco_Transformed_RPY_EE(0)*180)/3.14;
//~ Aruco_Transformed_RPY_EE(1)=(Aruco_Transformed_RPY_EE(1)*180)/3.14;
//~ Aruco_Transformed_RPY_EE(2)=(Aruco_Transformed_RPY_EE(2)*180)/3.14;
//~ std::cout << "Aruco_Transformed_RPY_EE=" << std::endl << Aruco_Transformed_RPY_EE << std::endl;


}


void form_3_point_Rotation_Matrix(pcl::PointXYZ Point_1,pcl::PointXYZ Point_2,pcl::PointXYZ Point_3,Eigen::Vector3f &X_Vector,Eigen::Vector3f &Y_Vector,Eigen::Vector3f Z_Vector,Eigen::Matrix3f &Rotation_matrix)
{
X_Vector[0]=Point_2.x-Point_1.x;
X_Vector[1]=Point_2.y-Point_1.y;
X_Vector[2]=Point_2.z-Point_1.z;
X_Vector=X_Vector.normalized();
Y_Vector=Z_Vector.cross(X_Vector);	
	
Rotation_matrix.block<3,1>(0,0)=X_Vector;
Rotation_matrix.block<3,1>(0,1)=Y_Vector;
Rotation_matrix.block<3,1>(0,2)=Z_Vector;
std::cout << "Rotation_matrix="<<std::endl<<Rotation_matrix<<std::endl;

}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{

//~ m.lock();
//ros::NodeHandle nh2;
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_partitioned;

//  const std::string & cloudname = "cloud"; 

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt (new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PCLPointCloud2* cloud_p2 = new pcl::PCLPointCloud2; 


//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl_conversions::toPCL(*cloud_msg, *cloud_p2);
//cout<<*cloud_p2;

pcl::fromPCLPointCloud2(*cloud_p2, *cloud_pt);
std::vector<int> indices;
pcl::removeNaNFromPointCloud(*cloud_pt, *cloud_out, indices);

//cout<<*cloud_pt;
//  sensor_msgs::PointCloud2 output;
// output= *cloud_msg;
//pcl::fromROSMsg(*cloud_msg, *cloud_pt);
//cout<<*cloud_pt;
//~ float resolution = 128.0f;
//~ pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
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
map_to_3d_pointcloud(cloud_pt,object2_grasp_point);
map_to_3d_pointcloud(cloud_pt,object3_grasp_point);
map_to_3d_pointcloud(cloud_pt,object4_grasp_point);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_1);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_2);
map_to_3d_pointcloud(cloud_pt,object1_rect_coordinates_3);
map_to_3d_pointcloud(cloud_pt,object2_rect_coordinates_1);
map_to_3d_pointcloud(cloud_pt,object2_rect_coordinates_2);
map_to_3d_pointcloud(cloud_pt,object2_rect_coordinates_3);
map_to_3d_pointcloud(cloud_pt,object3_rect_coordinates_1);
map_to_3d_pointcloud(cloud_pt,object3_rect_coordinates_2);
map_to_3d_pointcloud(cloud_pt,object3_rect_coordinates_3);
map_to_3d_pointcloud(cloud_pt,object4_rect_coordinates_1);
map_to_3d_pointcloud(cloud_pt,object4_rect_coordinates_2);
map_to_3d_pointcloud(cloud_pt,object4_rect_coordinates_3);

cout<<"\n"<<object1_grasp_point;
cout<<"\n"<<object2_grasp_point;
cout<<"\n"<<object3_grasp_point;
cout<<"\n"<<object4_grasp_point;

cout<<"\n"<<object1_rect_coordinates_1;
cout<<"\n"<<object1_rect_coordinates_2;
cout<<"\n"<<object1_rect_coordinates_3;


std_msgs::Float32MultiArray array;
Point_bat_box.x=object1_grasp_point.x;
Point_bat_box.y=object1_grasp_point.y;
Point_bat_box.z=object1_grasp_point.z;
Point_staple.x=object2_grasp_point.x;
Point_staple.y=object2_grasp_point.y;
Point_staple.z=object2_grasp_point.z;
Point_staple.x=object3_grasp_point.x;
Point_staple.y=object3_grasp_point.y;
Point_staple.z=object3_grasp_point.z;
Point_test.x=object3_grasp_point.x;
Point_test.y=object3_grasp_point.y;
Point_test.z=object3_grasp_point.z;
Point_bombe.x=object4_grasp_point.x;
Point_bombe.y=object4_grasp_point.y;
Point_bombe.z=object4_grasp_point.z;
pub_batbox_points.publish(Point_bat_box);
pub_staple_points.publish(Point_staple);
pub_test_points.publish(Point_test);
pub_bombe_points.publish(Point_bombe);


if (kdtree.radiusSearch (object1_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{            
cout<<"YES!";
//~ for(size_t i=0;i<radiusIdx.size();++i)
//~ {
//~ cout<<" "<< cloud_pt->points[radiusIdx[i]].x << "  " <<	cloud_pt->points[radiusIdx[i]].y << "  " << cloud_pt->points[radiusIdx[i]].z <<  "squared distance: " << radiusIdx[i] << ")"<<endl;
//~ }
}                                     
if (kdtree1.radiusSearch (object2_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{
cout<<"YES!";
}       
if (kdtree2.radiusSearch (object3_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{
cout<<"YES!";
}      
if (kdtree3.radiusSearch (object4_grasp_point, radius, radiusIdx, radiusSQDist) > 0)
{
cout<<"YES!";
} 
computePointNormal(*cloud_pt,radiusIdx,plane_parameters,curvature); 
computePointNormal(*cloud_pt,radiusIdx,plane_parameters1,curvature1); 
computePointNormal(*cloud_pt,radiusIdx,plane_parameters2,curvature2); 
computePointNormal(*cloud_pt,radiusIdx,plane_parameters3,curvature3); 
//~ flipNormalTowardsViewpoint (object1_grasp_point, 0, 0, 0, plane_parameters);
//~ flipNormalTowardsViewpoint (object2_grasp_point, 0, 0, 0, plane_parameters1);
//~ flipNormalTowardsViewpoint (object3_grasp_point, 0, 0, 0, plane_parameters2);
//~ flipNormalTowardsViewpoint (object4_grasp_point, 0, 0, 0, plane_parameters3);

cout<<"param: "<<plane_parameters<<endl;
cout<<"param: "<<plane_parameters1<<endl;
cout<<"param: "<<plane_parameters2<<endl;
cout<<"param: "<<plane_parameters3<<endl;
cout<<"curvature: "<<curvature<<endl;
cout<<"curvature: "<<curvature1<<endl;
cout<<"curvature: "<<curvature2<<endl;
cout<<"curvature: "<<curvature3<<endl;



Orientation_batbox.x=(double)plane_parameters[0];
Orientation_batbox.y=(double)plane_parameters[1];
Orientation_batbox.z=(double)plane_parameters[2];
Orientation_batbox.w=(double)plane_parameters[3];

Eigen::Quaternionf quat(plane_parameters);
Eigen::Vector3f position(object1_grasp_point.x,object1_grasp_point.y,object1_grasp_point.z);
Eigen::Vector3f vector;
Eigen::Vector3f x_vector_object1;
Eigen::Vector3f y_vector_object1;
Eigen::Matrix3f Rotation_object1;
vector(0)=(double)plane_parameters[0];
vector(1)=(double)plane_parameters[1];
vector(2)=(double)plane_parameters[2];
form_3_point_Rotation_Matrix(object1_grasp_point,object1_rect_coordinates_3,object1_rect_coordinates_3,x_vector_object1,y_vector_object1,vector,Rotation_object1);
//~ x_vector_object1(0)=object1_rect_coordinates_3.x-object1_grasp_point.x;
//~ x_vector_object1(1)=object1_rect_coordinates_3.y-object1_grasp_point.y;
//~ x_vector_object1(2)=object1_rect_coordinates_3.z-object1_grasp_point.z;
//~ x_vector_object1=x_vector_object1.normalized();
//~ y_vector_object1=vector.cross(x_vector_object1);
//~ cout<<vector<<"\t"<<x_vector_object1<<"\t"<<y_vector_object1;
//~ Eigen::Matrix3f Rotation_object1;
//~ Rotation_object1.block<3,1>(0,0)=x_vector_object1;
//~ Rotation_object1.block<3,1>(0,1)=y_vector_object1;
//~ Rotation_object1.block<3,1>(0,2)=vector;


std::cout << "Rotation_object1="<<std::endl<<Rotation_object1<<std::endl;
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object1,Rotation_object1,position);
cout<<"\n"<<"Homgenous_Matrix_Sensor_Object1="<<Homgenous_Matrix_Sensor_Object1;
Eigen::Matrix4f Transformed_Object_to_Robot=Homgenous_Matrix_Sensor_Object1;
transformation_to_Tool_Center_Point(Transformed_Object_to_Robot);
cout<<"\n"<<Transformed_Object_to_Robot;
Eigen::Matrix3f Transformed_Rotation_object1=Transformed_Object_to_Robot.block<3,3>(0,0);
Eigen::Vector3f Trasformed_Position_object1=Transformed_Object_to_Robot.block<3,1>(0,3);
Eigen::Vector3f orientation_in_rpy=Transformed_Rotation_object1.eulerAngles(0,1,2);
cout<<"\n"<<"Rotation_object1_in radians"<<"\n"<<orientation_in_rpy<<endl;
orientation_in_rpy(0)=(orientation_in_rpy(0)*180)/3.14;
orientation_in_rpy(1)=(orientation_in_rpy(1)*180)/3.14;
orientation_in_rpy(2)=(orientation_in_rpy(2)*180)/3.14;
cout<<"\n"<<"Rotation_object1"<<"\n"<<orientation_in_rpy<<endl;
cout<<"\n"<<"Position_object1"<<Trasformed_Position_object1<<endl;


//~ Eigen::Matrix3f Check_Rotation;
//~ Check_Rotation=Eigen::AngleAxisf(orientation_in_rpy[0],Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(orientation_in_rpy[1],Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(orientation_in_rpy[2],Eigen::Vector3f::UnitZ());
//~ Eigen::Vector3f orientaion_check=Check_Rotation.eulerAngles(0,1,2);
//~ Check_Rotation=Check_Rotation-Transformed_Rotation_object1;
//~ cout<<"Check_Rotation="<<endl<<Check_Rotation<<endl;
//~ cout<<"orientaion_check="<<endl<<orientaion_check<<endl;





Eigen::Quaternionf quat1(plane_parameters1);
Eigen::Vector3f position1(object2_grasp_point.x,object2_grasp_point.y,object2_grasp_point.z);
Eigen::Vector3f vector1;
Eigen::Vector3f x_vector_object2;
Eigen::Vector3f y_vector_object2;
vector1(0)=(double)plane_parameters1[0];
vector1(1)=(double)plane_parameters1[1];
vector1(2)=(double)plane_parameters1[2];
Orientation_staple.x=(double)plane_parameters1[0];
Orientation_staple.y=(double)plane_parameters1[1];
Orientation_staple.z=(double)plane_parameters1[2];
Orientation_staple.w=(double)plane_parameters1[3];
Eigen::Matrix3f Rotation_object2;
form_3_point_Rotation_Matrix(object2_grasp_point,object2_rect_coordinates_3,object2_rect_coordinates_3,x_vector_object2,y_vector_object2,vector1,Rotation_object2);
std::cout << "Rotation_object2="<<std::endl<<Rotation_object2<<std::endl;
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object2,Rotation_object2,position1);
cout<<"\n"<<"Homgenous_Matrix_Sensor_Object2="<<Homgenous_Matrix_Sensor_Object2;
Eigen::Matrix4f Transformed_Object2_to_Robot=Homgenous_Matrix_Sensor_Object2;
transformation_to_Tool_Center_Point(Transformed_Object_to_Robot);
Eigen::Matrix3f Transformed_Rotation_object2=Transformed_Object2_to_Robot.block<3,3>(0,0);
Eigen::Vector3f Trasformed_Position_object2=Transformed_Object2_to_Robot.block<3,1>(0,3);
Eigen::Vector3f orientation2_in_rpy=Transformed_Rotation_object2.eulerAngles(0,1,2);
cout<<"\n"<<"Rotation_object2_in radians"<<"\n"<<orientation2_in_rpy<<endl;
orientation2_in_rpy(0)=(orientation2_in_rpy(0)*180)/3.14;
orientation2_in_rpy(1)=(orientation2_in_rpy(1)*180)/3.14;
orientation2_in_rpy(2)=(orientation2_in_rpy(2)*180)/3.14;
cout<<"\n"<<"Rotation_object2"<<"\n"<<orientation2_in_rpy<<endl;
cout<<"\n"<<"Position_object2"<<Trasformed_Position_object2<<endl;




Eigen::Quaternionf quat2(plane_parameters2);
Eigen::Vector3f position2(object3_grasp_point.x,object3_grasp_point.y,object3_grasp_point.z);
Eigen::Vector3f vector2;
Eigen::Vector3f x_vector_object3;
Eigen::Vector3f y_vector_object3;
vector2(0)=(double)plane_parameters2[0];
vector2(1)=(double)plane_parameters2[1];
vector2(2)=(double)plane_parameters2[2];
Orientation_test.x=(double)plane_parameters2[0];
Orientation_test.y=(double)plane_parameters2[1];
Orientation_test.z=(double)plane_parameters2[2];
Orientation_test.w=(double)plane_parameters2[3];
Eigen::Matrix3f Rotation_object3;
form_3_point_Rotation_Matrix(object3_grasp_point,object3_rect_coordinates_3,object3_rect_coordinates_3,x_vector_object2,y_vector_object2,vector2,Rotation_object2);
std::cout << "Rotation_object3="<<std::endl<<Rotation_object3<<std::endl;
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object3,Rotation_object3,position2);
cout<<"\n"<<"Homgenous_Matrix_Sensor_Object3="<<Homgenous_Matrix_Sensor_Object3;
Eigen::Matrix4f Transformed_Object3_to_Robot=Homgenous_Matrix_Sensor_Object3;
transformation_to_Tool_Center_Point(Transformed_Object3_to_Robot);
Eigen::Matrix3f Transformed_Rotation_object3=Transformed_Object3_to_Robot.block<3,3>(0,0);
Eigen::Vector3f Trasformed_Position_object3=Transformed_Object3_to_Robot.block<3,1>(0,3);
Eigen::Vector3f orientation3_in_rpy=Transformed_Rotation_object3.eulerAngles(0,1,2);
cout<<"\n"<<"Rotation_object3_in radians"<<"\n"<<orientation3_in_rpy<<endl;
orientation3_in_rpy(0)=(orientation3_in_rpy(0)*180)/3.14;
orientation3_in_rpy(1)=(orientation3_in_rpy(1)*180)/3.14;
orientation3_in_rpy(2)=(orientation3_in_rpy(2)*180)/3.14;
cout<<"\n"<<"Rotation_object3"<<"\n"<<orientation3_in_rpy<<endl;
cout<<"\n"<<"Position_object3"<<Trasformed_Position_object3<<endl;







Eigen::Quaternionf quat3(plane_parameters3);
Eigen::Vector3f position3(object4_grasp_point.x,object4_grasp_point.y,object4_grasp_point.z);
Eigen::Vector3f vector3;
Eigen::Vector3f x_vector_object4;
Eigen::Vector3f y_vector_object4;
vector3(0)=(double)plane_parameters3[0];
vector3(1)=(double)plane_parameters3[1];
vector3(2)=(double)plane_parameters3[2];     
Orientation_bombe.x=(double)plane_parameters3[0];
Orientation_bombe.y=(double)plane_parameters3[1];
Orientation_bombe.z=(double)plane_parameters3[2];
Orientation_bombe.w=(double)plane_parameters3[3]; 
Eigen::Matrix3f Rotation_object4;
std::cout << "Rotation_object4="<<std::endl<<Rotation_object4<<std::endl;
form_3_point_Rotation_Matrix(object3_grasp_point,object3_rect_coordinates_3,object3_rect_coordinates_3,x_vector_object2,y_vector_object2,vector3,Rotation_object2);
Convert_to_Homgenous_Matrix(Homgenous_Matrix_Sensor_Object4,Rotation_object4,position2);
cout<<"\n"<<"Homgenous_Matrix_Sensor_Object4="<<Homgenous_Matrix_Sensor_Object4;
Eigen::Matrix4f Transformed_Object4_to_Robot=Homgenous_Matrix_Sensor_Object4;
transformation_to_Tool_Center_Point(Transformed_Object4_to_Robot);
Eigen::Matrix3f Transformed_Rotation_object4=Transformed_Object4_to_Robot.block<3,3>(0,0);
Eigen::Vector3f Trasformed_Position_object4=Transformed_Object4_to_Robot.block<3,1>(0,3);
Eigen::Vector3f orientation4_in_rpy=Transformed_Rotation_object4.eulerAngles(0,1,2);
cout<<"\n"<<"Rotation_object4_in radians"<<"\n"<<orientation4_in_rpy<<endl;
orientation4_in_rpy(0)=(orientation4_in_rpy(0)*180)/3.14;
orientation4_in_rpy(1)=(orientation4_in_rpy(1)*180)/3.14;
orientation4_in_rpy(2)=(orientation4_in_rpy(2)*180)/3.14;
cout<<"\n"<<"Rotation_object4"<<"\n"<<orientation4_in_rpy<<endl;
cout<<"\n"<<"Position_object4"<<Trasformed_Position_object4<<endl;

pose_staple.position=Point_staple;
pose_staple.orientation=Orientation_staple;
pose_test.orientation=Orientation_test;
pose_test.position=Point_test;
pose_bombe.position=Point_bombe;
pose_bombe.orientation=Orientation_bombe;

//calculate final pose by multiplying with the transformation matrix according to the Robot tool and publish it#



pub_batbox_pose.publish(pose_bat_box);
pub_staple_pose.publish(pose_staple);
pub_test_pose.publish(pose_test);
pub_bombe_pose.publish(pose_bombe);
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
//~ cout<<"\n"<<indicesptr->size();
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
normalsVis(cloud_pt,normals,object1_grasp_point,object2_grasp_point,object3_grasp_point,object4_grasp_point,Rotation_object1,Rotation_object2,Rotation_object3,Rotation_object4);
}

/*
 * This method is to get Grasping Points of Object 1
 * @parameter[IN] coordinate point(X,Y,Z) of object 1
 * */
void  normal_estimate_cb(const geometry_msgs::Point::ConstPtr& msg)
{
object1_grasp_point.z = (msg->z);
object1_grasp_point.x =(msg->x) ;
object1_grasp_point.y =(msg->y);
}

/*
 * This method is to get Grasping Points of Object 2
 * @parameter[IN] coordinate point(X,Y,Z) of object 2
 * */
void  normal_estimate_cb1(const geometry_msgs::Point::ConstPtr& msg)
{

object2_grasp_point.z = (msg->z);
object2_grasp_point.x =(msg->x) ;
object2_grasp_point.y =(msg->y);
}
/*
 * This method is to get Grasping Points of Object 3
 * @parameter[IN] coordinate point(X,Y,Z) of object 3
 * */

void  normal_estimate_cb2(const geometry_msgs::Point::ConstPtr& msg)
{
object3_grasp_point.z = (msg->z);
object3_grasp_point.x =(msg->x) ;
object3_grasp_point.y =(msg->y);
}
/*
 * This method is to get Grasping Points of Object 4
 * @parameter[IN] coordinate point(X,Y,Z) of object 4
 * */


void  normal_estimate_cb3(const geometry_msgs::Point::ConstPtr& msg)
{
object4_grasp_point.z = (msg->z);
object4_grasp_point.x =(msg->x) ;
object4_grasp_point.y =(msg->y);
}

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

object1_rect_coordinates_3.x=(object1_rect_coordinates_1.x+object1_rect_coordinates_2.x)/2;
object1_rect_coordinates_3.y=(object1_rect_coordinates_1.y+object1_rect_coordinates_2.y)/2;
}
void  grasp_rectange_2(const std_msgs::Float32MultiArray::ConstPtr & msg)
{


//~ for(int i=0;i<8;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }

object2_rect_coordinates_1.x=msg->data[0];
object2_rect_coordinates_1.y=msg->data[1];
object2_rect_coordinates_2.x=msg->data[2];
object2_rect_coordinates_2.y=msg->data[3];

object2_rect_coordinates_3.x=(object2_rect_coordinates_1.x+object2_rect_coordinates_2.x)/2;
object2_rect_coordinates_3.y=(object2_rect_coordinates_1.y+object2_rect_coordinates_2.y)/2;
}
void  grasp_rectange_3(const std_msgs::Float32MultiArray::ConstPtr & msg)
{


//~ for(int i=0;i<8;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }

object3_rect_coordinates_1.x=msg->data[0];
object3_rect_coordinates_1.y=msg->data[1];
object3_rect_coordinates_2.x=msg->data[2];
object3_rect_coordinates_2.y=msg->data[3];

object3_rect_coordinates_3.x=(object3_rect_coordinates_1.x+object3_rect_coordinates_2.x)/2;
object3_rect_coordinates_3.y=(object3_rect_coordinates_1.y+object3_rect_coordinates_2.y)/2;
}
void  grasp_rectange_4(const std_msgs::Float32MultiArray::ConstPtr & msg)
{


//~ for(int i=0;i<8;i++)
//~ {
//~ cout<<"\n"<<msg->data[i];	
//~ }

object4_rect_coordinates_1.x=msg->data[0];
object4_rect_coordinates_1.y=msg->data[1];
object4_rect_coordinates_2.x=msg->data[2];
object4_rect_coordinates_2.y=msg->data[3];

object4_rect_coordinates_3.x=(object4_rect_coordinates_1.x+object4_rect_coordinates_2.x)/2;
object4_rect_coordinates_3.y=(object4_rect_coordinates_1.y+object4_rect_coordinates_2.y)/2;
}

int main (int argc, char** argv)
{
// Initialize ROS
ros::init (argc, argv, "my_pcl");
ros::NodeHandle nh;
geometry_msgs::Point search;

pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
pub2 = nh.advertise<geometry_msgs::Point> ("output2", 1);
pub_batbox_points = nh.advertise<geometry_msgs::Point> ("Cameraspace_point_batbox", 1);
pub_staple_points = nh.advertise<geometry_msgs::Point> ("Cameraspace_point_staple", 1);
pub_test_points = nh.advertise<geometry_msgs::Point> ("Cameraspace_point_test", 1);
pub_bombe_points = nh.advertise<geometry_msgs::Point> ("Cameraspace_point_bombe", 1);
pub_batbox_pose=nh.advertise<geometry_msgs::Pose>("pose_bat_box", 1);
pub_staple_pose=nh.advertise<geometry_msgs::Pose>("pose_staple", 1);
pub_test_pose=nh.advertise<geometry_msgs::Pose>("pose_test", 1);
pub_bombe_pose=nh.advertise<geometry_msgs::Pose>("pose_bombe", 1);

//~ pub7 = nh.advertise<geometry_msgs::Point> ("output7", 1);
pub3 = nh.advertise< sensor_msgs::PointCloud2> ("output3", 1);
pub5 = nh.advertise< sensor_msgs::PointCloud2> ("output5", 1);
pub4 = nh.advertise<geometry_msgs::Quaternion> ("output4", 1);
pub6 = nh.advertise<geometry_msgs::Point> ("output6", 1);

// Create a ROS subscriber for the input point cloud
ros::Subscriber pt = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_bat_box", 1, normal_estimate_cb);
ros::Subscriber pt_1 = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_staple", 1, normal_estimate_cb1);
ros::Subscriber pt_2 = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_test", 1, normal_estimate_cb2);
ros::Subscriber pt_3 = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_bombe", 1, normal_estimate_cb3);
ros::Subscriber pt_4 = nh.subscribe<std_msgs::Float32MultiArray> ("/sim_grasp/grasp_rect_points_bat_box", 1, grasp_rectange_1);
ros::Subscriber pt_5 = nh.subscribe<std_msgs::Float32MultiArray> ("/sim_grasp/grasp_rect_points_staple", 1, grasp_rectange_2);
ros::Subscriber pt_6 = nh.subscribe<std_msgs::Float32MultiArray> ("/sim_grasp/grasp_rect_points_test", 1, grasp_rectange_3);
ros::Subscriber pt_7 = nh.subscribe<std_msgs::Float32MultiArray> ("/sim_grasp/grasp_rect_points_bombe", 1, grasp_rectange_4);
ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1, cloud_cb);
ros::spin();
}
