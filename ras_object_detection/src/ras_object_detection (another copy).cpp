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

class maze_object
{
  public: // DECLARATIONS

  int loop_frequency = 20;  // in Hz
  image_transport::Publisher rgb_pub;
  image_transport::Subscriber rgb_sub, depth_sub;
  Mat rgb_input, depth_input, rgb_output;

  // constructor
  maze_object();

  // callback for RGB image from Realsense   
  void callback_inputRGB(const sensor_msgs::ImageConstPtr&);

  // callback for depth image from Realsense   
  void callback_inputDepth(const sensor_msgs::ImageConstPtr&);
}

/*  FUNCTION DEFINITIONS  */
void maze_object::callback_inputRGB(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  try
  {
    cv_bridge::toCvShare(rgb_msg, "bgr8")->rgb_input;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert RGB from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void maze_object::callback_inputDepth(const sensor_msgs::ImageConstPtr& depth_msg)
{
  try
  {
    cv_bridge::toCvShare(depth_msg, "bgr8")->depth_input;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert DEPTH from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void main()
{
  // create an object of type maze_object
  maze_object object;

  ros::init(argc, argv, "ras_object_detection");
  ros::NodeHandle n;
  ros::Rate loop_rate(object.loop_frequency);
  image_transport::ImageTransport it(n);
  
  // Subscribers
  object.rgb_sub = it.subscribe('/camera/rgb/image_rect_color', 1, &maze_object::callback_inputRGB, &object);
  //object.depth_sub = it.subscribe('/camera/depth/image_raw', 1, &maze_object::callback_inputDepth, &object);
  
  // Publishers
  object.rgb_pub = it.advertise("/maze_object/image", 1);

  //show images for reference
  imshow("RGB image", object.rgb_input);
  waitKey(20);
  //imshow("Depth image", object.depth_input);
  waitKey(20);
}