#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include "std_msgs/String.h"
// for sending an array as message
#include "std_msgs/Float32MultiArray.h"
// for visualizing marker on ros
#include <visualization_msgs/Marker.h>
// point stamped point for target position of object
#include <geometry_msgs/PointStamped.h>
// #include <tf/transform_listener.h>


#define DEBUG 0

using namespace cv;
using namespace std;

class maze_object
{
  public:
  // image containers
  Mat rgb_input, depth_input;

  /*
    Realsense camera intrinsic parameters. To obtain type 

          rostopic echo /camera/rgb/camera_info
      
    Source: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  */
  float fx = 616.344;
  float fy = 616.344;
  float cx = 314.855;
  float cy = 223.877;

  // Object Position
  // float x_world = 0;
  // float y_world = 0;
  // float z_world = 0;

  //geometry_msgs::PointStamped base_point;
  geometry_msgs::PointStamped object_point;

  // Max and Min Depth in meters (0 and 255 respectively on depth image)
  float max_depth = 2.0;  
  float min_depth = 0.05; 
  
  //public:

  Rect bBoxes[6];
  int max_bBox_ind = 0;

  // Subscriber callbacks
  void callback_inputDepth(const sensor_msgs::ImageConstPtr& );
  void callback_inputRGB(const sensor_msgs::ImageConstPtr& msg);

  // detecting regions of interest based on color 
  Rect detectColor(Scalar, Scalar);

  // Extract depth of detected object: DELETED
  //void extract_depth(Rect *, int);

  // Display Bounding box which has max area, i.e. dominant color
  void display_bBox(Rect *, int);

  // Transform object to map
  //void transformPoint(const tf::TransformListener&);
  //void transformPoint();
};


/*********************
  Constructor
**********************/
// void maze_object::maze_object()
// {

// }

/*********************
  callback_inputDepth
**********************/
void maze_object::callback_inputDepth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // depth_input = cv_bridge::toCvShare(msg, "32FC1")->image;
    //For camera/depth/ topic, the encoding is : TYPE_16UC1 
    depth_input = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    //imshow("Depth", depth_input);
    //imshow("Depth Image", cv_bridge::toCvShare(msg, "32FC1")->image);
    //waitKey(30);

    //cout<<"Depth at 320,240 is: \t"<<depth_input.at<float>(240,320)<<"\n";
    //cout<< depth_input;
    //cout<<"\n\n\n\n\n\n\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n\n\n\n\n\n\n";
    


    // Extract depth of largest object
    int x_pos = int(bBoxes[max_bBox_ind].x + bBoxes[max_bBox_ind].width/2);
    int y_pos = int(bBoxes[max_bBox_ind].y + bBoxes[max_bBox_ind].height/2);
    float z_world_temp = depth_input.at<float>(y_pos, x_pos);
    
    if(!isnan(z_world_temp))
    {
      object_point.point.z = z_world_temp;

    //cout<<"Depth at "<<x_pos<<" and "<<y_pos<<" is :\t"<<z_world<<"\n";
      // Calculate world coordinates
      object_point.header.frame_id = "objects";

      //we'll just use the most recent transform available for our simple example
      object_point.header.stamp = ros::Time();
      object_point.point.x = (x_pos - cx) / fx * z_world_temp;//(y_pos - cy) / fy * z_world;
      object_point.point.y = -(y_pos - cy) / fy * z_world_temp;//(x_pos - cx) / fx * z_world;
      object_point.point.y = - object_point.point.y;    // based on observation
    }
    else
    {
      object_point.point.x = z_world_temp;
      object_point.point.y = z_world_temp;
      object_point.point.z = z_world_temp;
    }

    //cout<<"World coordinates of object, w.r.t. camera are: "<<x_world<<" ,"<<y_world<<" ,"<<z_world<<"\n"; 
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
  }
}

/*********************
  callback_inputRGB
**********************/
void maze_object::callback_inputRGB(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    rgb_input = cv_bridge::toCvShare(msg, "bgr8")->image;
    //imshow("RGB", rgb_input);
    //imshow("RGB Image", cv_bridge::toCvShare(msg, "bgr8")->image);
    //waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/*********************
  detectColor
**********************/
// int hmin, int smin, int vmin, int hmax, int smax, int vmax,
Rect maze_object::detectColor(Scalar min_val, Scalar max_val)
{
  Rect bBox;
  //std::cout<<"Entered detectColor \n";
  
  if(!rgb_input.empty())
  {
    //std::cout<<"rgb_imput is non empty \n";
    Mat img_clone = rgb_input.clone();
    Mat img_hsv, thresh_hsv, morph_opening;
  
    cvtColor(img_clone, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, min_val, max_val, thresh_hsv);



    // Apply the specified morphology operation
	  int morph_operator = 2;		//OPENING
    int morph_elem = 2;			//ELLIPSE
    int morph_size = 3	;		//SIZE of Strel

    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );

    // Find Contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( morph_opening, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if (contours.size() != 0)
  	{
      // find index of largest contour
      int ind_max = 0;
      double ctr_area = 0;
      double ctr_maxarea = 0;

      for ( int i = 0; i < contours.size(); i++)
		  {
        ctr_area = contourArea(contours[i]);
        if(ctr_area > ctr_maxarea)
        {
          ctr_maxarea = ctr_area;
          ind_max = i;
        }
      }

      // find Bounding Box of largest contour
      bBox = boundingRect(Mat(contours[ind_max]));

#if(DEBUG == 1)
      imshow("Thresholded image", thresh_hsv);

      imshow("Morphed image", morph_opening);
      waitKey(20);
#endif
    }
  }
  else
  {
    //std::cout<<"RGB is EMPTY \n";
  }

  return bBox;
}

/*********************
  display_bBox
**********************/
void maze_object::display_bBox(Rect *bBox, int ind)
{
  if(!rgb_input.empty())
  {
    Mat img_clone = rgb_input.clone();
    //rectangle( img_clone, bBox->tl(), bBox->br(), Scalar(0,255,0), 2, 8, 0 );
    rectangle( img_clone, bBox[ind].tl(), bBox[ind].br(), Scalar(0,255,0), 2, 8, 0 );

    imshow("Detected Object", img_clone);
    waitKey(20);
  }
}


/*********************
  Transform objects
**********************/

// void maze_object::transformPoint(const tf::TransformListener& listener)
// //void maze_object::transformPoint()
// {
//   //tf::TransformListener listener(ros::Duration(20));
//   try
//   {
//     listener.transformPoint("base_link", object_point, base_point);

//     ROS_INFO("base_object: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
//         object_point.point.x, object_point.point.y, object_point.point.z,
//         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//   }
//   catch(tf::TransformException& ex)
//   {
//     ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
//   }
// }

// //&maze_object::transformPoint
// void expt_fcn(const tf::TransformListener& listener)
// {
//   cout<<"It works!";
// }


/*********************
  MAIN
**********************/
int main(int argc, char **argv)
{
  maze_object object;
  int loop_frequency = 20;
  //Mat rgb_input, depth_input, rgb_output;
  //Rect bBoxes[6];
  //int max_bBox_ind = 0;

  // create an object of std::msg::String type for state mc message
  std_msgs::String sm_msg;
  std::stringstream ss;
  ros::init(argc, argv, "ras_object_detection");
  
  ros::NodeHandle n;
  ros::Rate loop_rate(loop_frequency);
  image_transport::ImageTransport it(n);
  
  // namedWindow("view");
  // startWindowThread();
  
  // Subscribers to RGB and Depth image
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, &maze_object::callback_inputDepth, &object);
  image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_rect_color", 1, &maze_object::callback_inputRGB, &object);
  
  // Publisher for message to state machine
  ros::Publisher state_machine_pub = n.advertise<std_msgs::String>("/flag_done", 1); 
  
  // Publisher for publishing object coordinates
  //ros::Publisher obj_pose_pub = n.advertise<std_msgs::Float32MultiArray>("/object_position", 100);
  ros::Publisher obj_pose_pub = n.advertise<geometry_msgs::PointStamped>("/object_position_cam_link", 100);
  
  // Publisher for rviz marker
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/object_marker", 1);

  // tf::TransformListener listener(ros::Duration(20));

  // //we'll transform a point once every second
  // ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(expt_fcn, boost::ref(listener)));
  // //    ros::Timer timer = n.createTimer(ros::Duration(0.1), &maze_object::transformPoint);
  
  while(ros::ok())
  {
    int bBox_area = 0;
    int max_bBox_area = 0;
    //std::cout<<"Entered While loop \n";
    // Call Color detection for all colors
    #if(DEBUG == 0)
    object.bBoxes[0] = object.detectColor(Scalar(40,160,90), Scalar(50,255,200));  // Green 
    object.bBoxes[1] = object.detectColor(Scalar(1,210,90), Scalar(6,255,160));    // Red
    object.bBoxes[2] = object.detectColor(Scalar(15,210,110), Scalar(22,255,190)); // Yellow
    object.bBoxes[3] = object.detectColor(Scalar(7,220,110), Scalar(13,255,205));  // Orange
    object.bBoxes[4] = object.detectColor(Scalar(142,45,80), Scalar(179,132,150)); // purple 
    object.bBoxes[5] = object.detectColor(Scalar(90,70,45), Scalar(101,255,150));  // blue 
    #else
    object.bBoxes[5] = object.detectColor(Scalar(90,70,45), Scalar(101,255,150));  // blue 
    #endif

    // find largest bounding box
    for( int i = 0; i < 6; i++)
		  {
        bBox_area = int(object.bBoxes[i].width * object.bBoxes[i].height);

        if(bBox_area > max_bBox_area)
        {
           max_bBox_area = bBox_area;
           object.max_bBox_ind = i;
        }
      }

    //cout<<max_bBox_area<<"\n";
    if(max_bBox_area > 1000)
    {
      // Publish message to state machine: object found
      //std::cout<<"object found!"<<std::endl;
      string flag_string = "detect_object_done";
      //ss << "detect_object_done";
      //sm_msg.data = ss.str();
      sm_msg.data = flag_string;
      state_machine_pub.publish(sm_msg);

      // Publish object coordinates
      // create a float array message
      // std_msgs::Float32MultiArray object_position;
      // //Clear array
      // object_position.data.clear();

      // object_position.data.push_back(object.x_world);
      // object_position.data.push_back(object.y_world);
      // object_position.data.push_back(object.z_world);

      // tf::TransformListener listener(ros::Duration(20));

      //we'll transform a point once every second
      //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&maze_object::transformPoint, boost::ref(listener)));
      //ros::Timer timer = n.createTimer(ros::Duration(0.1), &maze_object::transformPoint);
      //object.transformPoint();

      obj_pose_pub.publish(object.object_point);
      //obj_pose_pub.publish(object.base_point);


      // Publish a marker for detected object
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/camera_link";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "detected object";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = visualization_msgs::Marker::CUBE;;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = object.object_point.point.x;
      marker.pose.position.y = object.object_point.point.y;
      marker.pose.position.z = object.object_point.point.z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration(0.25);

      marker_pub.publish(marker);
      loop_rate.sleep();
    }
    
    //Display object with largest bBox 
    object.display_bBox(object.bBoxes, object.max_bBox_ind);

    ros::spinOnce();
  }

  //ros::spin();
  destroyWindow("view");
}