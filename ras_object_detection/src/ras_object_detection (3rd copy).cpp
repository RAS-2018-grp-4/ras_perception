#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>
using namespace cv;

// class maze_object
// {
//   public: // DECLARATIONS

//   int loop_frequency = 20;  // in Hz
//   image_transport::Publisher rgb_pub;
//   image_transport::Subscriber rgb_sub, depth_sub;
//   Mat rgb_input, depth_input, rgb_output;

//   // constructor
//   maze_object();

//   // callback for RGB image from Realsense   
//   void callback_inputRGB(const sensor_msgs::ImageConstPtr&);

//   // callback for depth image from Realsense   
//   void callback_inputDepth(const sensor_msgs::ImageConstPtr&);
// }

/*  FUNCTION DEFINITIONS  */
void callback_inputRGB(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  try
  {
    //cv_bridge::toCvShare(rgb_msg, "bgr8")->rgb_input;
    cv::imshow("view", cv_bridge::toCvShare(rgb_msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert RGB from '%s' to 'bgr8'.", rgb_msg->encoding.c_str());
  }
}

void callback_inputDepth(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    //cv_bridge::toCvShare(depth_msg, "bgr8")->depth_input;
    cv::imshow("view", cv_bridge::toCvShare(depth_msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert DEPTH from '%s' to 'bgr8'.", depth_msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
  int loop_frequency = 20;
  //Mat rgb_input, depth_input, rgb_output;

  ros::init(argc, argv, "ras_object_detection");
  ros::NodeHandle n;
  ros::Rate loop_rate(loop_frequency);
  image_transport::ImageTransport it(n);
  
  // Subscribers
  image_transport::Subscriber rgb_sub = it.subscribe('/camera/rgb/image_rect_color', 1, callback_inputRGB);
  image_transport::Subscriber depth_sub = it.subscribe('/camera/depth/image_raw', 1, callback_inputDepth);
  
  // Publishers
  //image_transport::Publisher rgb_pub = it.advertise("/maze_object/image", 1);

  //show images for reference
  // imshow("RGB image", rgb_input);
  // waitKey(20);
  // imshow("Depth image", depth_input);
  // waitKey(20);

  ros::spin();
  cv::destroyWindow("view");
  
  return 0;
}