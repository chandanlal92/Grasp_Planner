#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#define SPACE_KEY (32)
#include <sstream>                            
                     // Added this
namespace enc = sensor_msgs::image_encodings;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_img_sub_;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &ImageConverter::imageCb, this);
    depth_img_sub_ = it_.subscribe("/kinect2/qhd/image_depth_rect", 1, &ImageConverter::depthCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   /* // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    int key = cv::waitKey(50);
    static int image_count = 0;  
    if ( key == SPACE_KEY ) {
    ROS_INFO("Key is pressed!");                         // added this
    std::stringstream sstream;                               // added this
    sstream << "/home/tncong/Untitled Folder/my_image" << image_count << ".png" ;                  // added this
    imwrite( sstream.str(),  cv_ptr->image );      // added this
    image_count++;                                      // added this
    }
   */
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
 
    
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /*// Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/

    // Update GUI Window

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    int key = cv::waitKey(50);
    static int image_count = 0;  
    if ( key == SPACE_KEY ) {
    ofstream myfile;
    myfile.open ("/home/tncong/Untitled Folder/example.txt");
    ROS_INFO("Key is pressed!");                         // added this
    std::stringstream sstream;                               // added this
    sstream << "/home/tncong/Untitled Folder/my_image" << image_count << ".png" ;                  // added this
    imwrite( sstream.str(),  cv_ptr->image );      // added this
    //myfile << (cv_ptr->image);
    //int foo [][] = cv_ptr->image;
    myfile << (cv_ptr->image).size()<<endl;
    image_count++; 
    myfile.close();                                     // added this
    }
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_listener");
  ImageConverter ic;
  ros::spin();
  return 0;
}