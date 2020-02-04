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
#include <vector>

using namespace message_filters;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;

sensor_msgs::PointCloud2 output;
sensor_msgs::PointCloud2 output2;
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pt ( new pcl::PointCloud<pcl::PointXYZ> );
pcl::PCLPointCloud2* cloud_p2 = new pcl::PCLPointCloud2; 
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_p2);
geometry_msgs::Point Pointer;
pcl::PointXYZ searchPoint;
pcl::PointXYZ searchPoint1;
pcl::PointXYZ searchPoint2;
geometry_msgs::Quaternion Orientation;
int count=0;

double cx = 9.6762715580208510e+02;
double cy = 5.5317956136661428e+02;
double fx = 1.0628881742777685e+03;
double fy = 1.0620718706590644e+03;

//~ static float cx = 254.878f;
//~ static float cy = 205.395f;
//~ static float fx = 365.456f;
//~ static float fy = 365.456f;
//~ static float k1 = 0.0905474;
//~ static float k2 = -0.26819;
//~ static float k3 = 0.0950862;

boost::mutex m;


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::PointXYZ center,pcl::PointXYZ center1,pcl::PointXYZ center2, Eigen::Vector3f vector,Eigen::Vector3f vector1,Eigen::Vector3f vector2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
      //~ m.lock();
 //~ pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
       
   //~ viewer->spinOnce (10);
        
//~ cout<< ("PointCloud with %d data points (%s)  and frame %s.",
            //~ cloud->width * cloud->height,
            //~ pcl::getFieldsList (*cloud).c_str (),
            //~ cloud->header.frame_id.c_str ());   
  //~ color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(cloud, "rgb" ));            
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud ,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //~ viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  
   pcl::PointXYZ x_axis (vector(0) + center.x, vector(1) + center.y, vector(2) + center.z);
   pcl::PointXYZ x_axis1 (vector1(0) + center1.x, vector1(1) + center1.y, vector1(2) + center1.z);
   pcl::PointXYZ x_axis2 (vector2(0) + center2.x, vector2(1) + center2.y, vector2(2) + center2.z);
  pcl::PointXYZ y_axis (vector(0) + center.x, vector(1) + center.y, vector(2) + center.z);
  pcl::PointXYZ y_axis1 (vector1(0) + center1.x, vector1(1) + center1.y, vector1(2) + center1.z);
  pcl::PointXYZ y_axis2 (vector2(0) + center2.x, vector2(1) + center2.y, vector2(2) + center2.z);
  pcl::PointXYZ z_axis (vector(0) + center.x, vector(1) + center.y, vector(2) + center.z);
  pcl::PointXYZ z_axis1 (vector1(0) + center1.x, vector1(1) + center1.y, vector1(2) + center1.z);
  pcl::PointXYZ z_axis2 (vector2(0) + center2.x, vector2(1) + center2.y, vector2(2) + center2.z);
  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"+1);
  viewer->addLine (center1, x_axis1, 1.0f, 0.0f, 0.0f, "major eigen vector1"+2);
  viewer->addLine (center2, x_axis2, 1.0f, 0.0f, 0.0f, "major eigen vector2"+3);
  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector3"+4);
  viewer->addLine (center1, y_axis1, 0.0f, 1.0f, 0.0f, "middle eigen vector4"+5);
  viewer->addLine (center2, y_axis2, 0.0f, 1.0f, 0.0f, "middle eigen vector5"+6);
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector6"+7);
  viewer->addLine (center1, z_axis1, 0.0f, 0.0f, 1.0f, "minor eigen vector7"+8);
  viewer->addLine (center2, z_axis2, 0.0f, 0.0f, 1.0f, "minor eigen vector8"+9);
   viewer->addCoordinateSystem (1.0);
   //viewer->initCameraParameters ();
     while(!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
  }
   return(viewer);
  //~ m.unlock();
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

        //~ octree.setInputCloud (cloud.makeShared());
        kdtree.setInputCloud(cloud_out);
         kdtree1.setInputCloud(cloud_out);
           kdtree2.setInputCloud(cloud_out);
        //kdtree.addPointsFromInputCloud ();

        //pcl::PointXYZ center_point;
        //center_point.x = 0 ;
        //center_point.y = 0.4;
        //center_point.z = -1.4;

        float radius = 1;//search in sphere of 1 cm           
        std::vector<int> radiusIdx;
        std::vector<float> radiusSQDist;
       
          Eigen::Vector4f plane_parameters; 
                   float curvature; 
        
          Eigen::Vector4f plane_parameters1; 
                   float curvature1; 
        
          Eigen::Vector4f plane_parameters2; 
                   float curvature2; 
        
     if (kdtree.radiusSearch (searchPoint, radius, radiusIdx, radiusSQDist) > 0)
   {            
             cout<<kdtree;
             cout<<"YES!";
     }                                     
              //~ if (kdtree1.radiusSearch (searchPoint1, radius, radiusIdx, radiusSQDist) > 0)
   //~ {
             //~ cout<<"YES!";
     //~ }       
         //~ if (kdtree2.radiusSearch (searchPoint2, radius, radiusIdx, radiusSQDist) > 0)
   //~ {
             //~ cout<<"YES!";
     //~ }      
     computePointNormal(*cloud_out,radiusIdx,plane_parameters,curvature); 
     //~ computePointNormal(*cloud_out,radiusIdx,plane_parameters1,curvature1); 
     //~ computePointNormal(*cloud_out,radiusIdx,plane_parameters2,curvature2); 
          cout<<"param: "<<plane_parameters<<endl;
         cout<<"param: "<<plane_parameters1<<endl;
        cout<<"param: "<<plane_parameters2<<endl;
         Orientation.x=(double)plane_parameters[0];
         Orientation.y=(double)plane_parameters[1];
         Orientation.z=(double)plane_parameters[2];
         Orientation.w=(double)plane_parameters[3];
         pub4.publish(Orientation);
      Eigen::Quaternionf quat(plane_parameters);
      Eigen::Vector3f position(searchPoint.x,searchPoint.y,searchPoint.z);
      Eigen::Vector3f vector;
      vector(0)=(double)plane_parameters[0];
      vector(1)=(double)plane_parameters[1];
      vector(2)=(double)plane_parameters[2];
      //~ Eigen::Quaternionf quat1(plane_parameters1);
      //~ Eigen::Vector3f position1(searchPoint1.x,searchPoint1.y,searchPoint1.z);
      //~ Eigen::Vector3f vector1;
      //~ vector1(0)=(double)plane_parameters1[0];
      //~ vector1(1)=(double)plane_parameters1[1];
      //~ vector1(2)=(double)plane_parameters1[2];
      //~ Eigen::Quaternionf quat2(plane_parameters2);
      //~ Eigen::Vector3f position2(searchPoint2.x,searchPoint2.y,searchPoint2.z);
      //~ Eigen::Vector3f vector2;
      //~ vector2(0)=(double)plane_parameters2[0];
      //~ vector2(1)=(double)plane_parameters2[1];
      //~ vector2(2)=(double)plane_parameters2[2];
            
        
         //pub3.publish(*cloud_out);
          // m.unlock();
         
         //cout<<searchPoint;
         //cout<<radiusIdx[0];
       pcl::toROSMsg(*cloud_out, output);
          pub.publish(output);
        //output.header.frame_id = "odom";
        //pcl_pub.publish(output);
      
       //  m.lock();

   
     //~ m.unlock();

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  
       pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); 
         ne.setSearchMethod (tree);

  ne.setRadiusSearch (500);
  ne.setKSearch(0);
  //~ ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  //~ ne.setMaxDepthChangeFactor(0.02f);
  //~ ne.setNormalSmoothingSize(10.0f);
  pcl::fromPCLPointCloud2(*cloud_p2,*temp_cloud);
  ne.setInputCloud(temp_cloud);
  ne.compute(*normals);


      pcl::toROSMsg(*normals,output2);
      pub3.publish(output2);  
       
 //}
   

    // normalsVis(cloud_out,cloud_normals1);
      pcl::PCLPointCloud2 cloud_filtered;
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
  

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output5;
  pcl_conversions::fromPCL(cloud_filtered, output5);

  // Publish the data
  pub5.publish (output5);
  //~ normalsVis(*cloud_filtered,normals);
  //~ normalsVis(temp_cloud,normals,searchPoint,searchPoint1,searchPoint2,vector,vector1,vector2);
   //~ m.unlock();
}


void  normal_estimate_cb(const geometry_msgs::Point::ConstPtr& msg)
{

  searchPoint.z = (double)(msg->z)/1000.;
  searchPoint.x =(double)(((msg->x) - cx) * searchPoint.z / fx);
  searchPoint.y =(double)(((msg->y) - cy) * searchPoint.z / fy);
  Pointer.x=searchPoint.x;
  Pointer.y=searchPoint.y;
  Pointer.z=searchPoint.z;
   //cout<<searchPoint;
   pub2.publish(Pointer);
 //~ count++;
   //cout<<Pointer;
  
 }
void  normal_estimate_cb1(const geometry_msgs::Point::ConstPtr& msg)
{

  searchPoint1.z = (double)(msg->z)/1000.;
  searchPoint1.x =(double)(((msg->x) - cx) * searchPoint1.z / fx);
  searchPoint1.y =(double)(((msg->y) - cy) * searchPoint1.z / fy);
  //~ Pointer1.x=searchPoint1.x;
  //~ Pointer1.y=searchPoint1.y;
  //~ Pointer1.z=searchPoint1.z;
   //~ cout<<searchPoint1;
   //~ pub2.publish(Pointer);
 //~ count++;
   //cout<<Pointer;
  
 }
void  normal_estimate_cb2(const geometry_msgs::Point::ConstPtr& msg)
{

  searchPoint2.z = (double)(msg->z)/1000.;
  searchPoint2.x =(double)(((msg->x) - cx) * searchPoint2.z / fx);
  searchPoint2.y =(double)(((msg->y) - cy) * searchPoint2.z / fy);
  //~ Pointer.x=searchPoint.x;
  //~ Pointer.y=searchPoint.y;
  //~ Pointer.z=searchPoint.z;
   //~ cout<<searchPoint;
   //~ pub2.publish(Pointer);
 //~ count++;
   //cout<<Pointer;
  
 }






int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl");
  ros::NodeHandle nh;
  geometry_msgs::Point search;

  //ros::Publisher pub2;
 /* 
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud(nh,"/kinect2/qhd/points", 1);
  message_filters::Subscriber<geometry_msgs::Point> sub_point(nh,"/sim_grasp/point", 1);
  sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::Point> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_point);
  sync.registerCallback(boost::bind(&callback, _1, _2)*/
  //~ m.lock();
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub2 = nh.advertise<geometry_msgs::Point> ("output2", 1);
  //~ pub6 = nh.advertise<geometry_msgs::Point> ("output6", 1);
  //~ pub7 = nh.advertise<geometry_msgs::Point> ("output7", 1);
   pub3 = nh.advertise< sensor_msgs::PointCloud2> ("output3", 1);
      pub5 = nh.advertise< sensor_msgs::PointCloud2> ("output5", 1);

  pub4 = nh.advertise<geometry_msgs::Quaternion> ("output4", 1);
  // Create a ROS subscriber for the input point cloud
 ros::Subscriber pt = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_bat_box", 1, normal_estimate_cb);
 //~ ros::Subscriber pt_1 = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_staple", 1, normal_estimate_cb1);
 //~ ros::Subscriber pt_2 = nh.subscribe<geometry_msgs::Point> ("/sim_grasp/points_test", 1, normal_estimate_cb2);
 ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 1, cloud_cb);
 cout<<"Called Cloud_cb";
  //ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2> ("/output", 1, normalsVis);
//m.unlock();
  //ros::Subscriber pub_2 = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect2/qhd/points", 10, cloud_cb);
    //~ m.unlock();
    



   ros::spin();
}
