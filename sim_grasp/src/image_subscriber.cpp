#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>    // OpenCV
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <fstream>
#include <sstream> 

#include "geometry_msgs/Point.h"
#include "sim_grasp/sim_grasp.h" 

#define APPROXIMATE
#define SPACE_KEY (32)

#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

class myfile1;
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
//using namespace sensor_msgs;
using namespace message_filters;


// Contador para la numeración de los archivos.
// Counter for filenames.
unsigned int cnt = 1;



// Handler / callback
void callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth, const sensor_msgs::ImageConstPtr& msg_bb )
{
    cv_bridge::CvImagePtr img_ptr_rgb;
    cv_bridge::CvImagePtr img_ptr_depth;
    
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }
    
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    //cv::Mat& mat_depth = img_ptr_depth->image;
    //cv::Mat& mat_rgb = img_ptr_rgb->image;
 
    /*char file_rgb[100];
    char file_depth[100];

    sprintf( file_rgb, "%04d_rgb.png", cnt );
    sprintf( file_depth, "%04d_depth.png", cnt );

    vector<int> png_parameters;
    png_parameters.push_back( CV_IMWRITE_PNG_COMPRESSION );
        /* We save with no compression for faster processing.
         * Guardamos PNG sin compresión para menor retardo. 
    png_parameters.push_back( 9 ); */


    cv::imshow(OPENCV_WINDOW, img_ptr_rgb->image);
    int key = cv::waitKey(50);
    static int image_count = 0;  
    if ( key == SPACE_KEY ) {
    ofstream myfile1,myfile2;
    //myfile1.open ("/home/tncong/Untitled Folder/rgb.txt");
    //myfile2.open ("/home/tncong/Untitled Folder/d.txt");
    ROS_INFO("Key is pressed!");                         // added this
    std::stringstream sstream_rgb, sstream_d;                               // added this
    sstream_rgb << "/home/tncong/Untitled Folder/my_rgb" << image_count << ".png" ;   
    sstream_d << "/home/tncong/Untitled Folder/my_d" << image_count << ".png" ;                // added this
    //imwrite( sstream.str(),  cv_ptr->image );      // added this
    cv::imwrite( sstream_rgb.str()  , img_ptr_rgb->image );
    cv::imwrite( sstream_d.str() , img_ptr_depth->image );
    //myfile1 << (img_ptr_rgb->image);
    //myfile2 << (img_ptr_depth->image);
    //int foo [][] = cv_ptr->image;
    //cout << *bb.size()<<endl;
    image_count++; 
    //myfile1.close();   
    //myfile2.close();                                    // added this
    }
}


int main(int argc, char** argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "img_listener");
    ros::NodeHandle nh;


    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( nh , "/kinect2/qhd/image_depth_rect" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( nh , "/kinect2/qhd/image_color_rect" , 1 );
    message_filters::Subscriber<sensor_msgs::Image> bat_box_bb( nh , "/kinect2/qhd/image_color_rect" , 1 );;
    
    cv::namedWindow(OPENCV_WINDOW);

    #ifdef EXACT
    typedef sync_policies::ExactTime<sensor_msgs::Image, sim_grasp::sim_grasp::ConstPtr> MySyncPolicy;
    #endif
    #ifdef APPROXIMATE
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    //typedef sync_policies::ApproximateTime<geometry_msgs::Point> MySyncPolicy;
    #endif


    // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth, bat_box_bb);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    while( ros::ok() ){
    ros::spin();
    }

    return 0;
}
