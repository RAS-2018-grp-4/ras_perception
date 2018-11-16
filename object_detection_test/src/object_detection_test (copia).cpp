#include <iostream>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
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

//Global images to use
Mat rgb_input, depth_input;


void callback_inputRGB(const sensor_msgs::ImageConstPtr& msg)
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
  callback_inputDepth
**********************/
void callback_inputDepth(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    depth_input = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    //cout<<"Depth at 320,240 is: \t"<<depth_input.at<float>(240,320)<<"\n";
    //cout<< depth_input;
    //cout<<"\n\n\n\n\n\n\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n\n\n\n\n\n\n";
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
  }
}

class maze_object
{
  public:
  // image containers
  Mat color_threshold;
  int number_same_color = 0;

  float fx = 616.344;
  float fy = 616.344;
  float cx = 314.855;
  float cy = 223.877;

  geometry_msgs::PointStamped object_point[];

  // Max and Min Depth in meters (0 and 255 respectively on depth image)
  float max_depth = 2.0;  
  float min_depth = 0.05; 

  Rect bBoxes[]; //Array of bounding boxes of objects of each color

  // Subscriber callbacks
  void obtainDepth();
  int findBoundingBoxes();  // detecting regions of interest based on color 
  void display_bBox(Mat); // Display Bounding box of each object

};

/*********************
  Constructor
**********************/
// void maze_object::maze_object()
// {
// }

/*********************
  findBoundingBoxes
**********************/
int maze_object::findBoundingBoxes() //This function returns bounding boxes of each possible object of one color
{
  // Find Contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours( color_threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  if (contours.size() != 0)
  {
    // find index of largest contour
    double ctr_area = 0;
    number_same_color = 0;

    for ( int i = 0; i < contours.size(); i++)
    {
      ctr_area = contourArea(contours[i]);
      if(ctr_area > 1500.0)
      {
        bBoxes[number_same_color] = boundingRect(Mat(contours[i]));
        number_same_color+=1;
      }
    }

    #if(DEBUG == 1)
      imshow("Thresholded image", thresh_hsv);

      imshow("Morphed image", morph_opening);
      waitKey(20);
    #endif

    return 0;
  }
  
  else
  {
    //std::cout<<"No contours detected \n";
    return -1;
  }
}

/*********************
  obtainDepth
**********************/
void maze_object::obtainDepth(){
  int x_pos[number_same_color];
  int y_pos[number_same_color];
  int z_world_temp[number_same_color];
  for(int i=0; i<number_same_color;i++){
    x_pos[i] = int(bBoxes[i].x + bBoxes[i].width/2);
    y_pos[i] = int(bBoxes[i].y + bBoxes[i].height/2);
    z_world_temp[i] = depth_input.at<float>(y_pos[i], x_pos[i]);
    z_world_temp[i] = - z_world_temp[i];     // observation suggests so

    if(!isnan(z_world_temp[i]))
    {
      object_point[i].point.z = z_world_temp[i];

    //cout<<"Depth at "<<x_pos<<" and "<<y_pos<<" is :\t"<<z_world<<"\n";
      // Calculate world coordinates
      object_point[i].header.frame_id = "/camera_link";

      //we'll just use the most recent transform available for our simple example
      object_point[i].header.stamp = ros::Time();
      object_point[i].point.x = (x_pos[i] - cx) / fx * z_world_temp[i];//(y_pos - cy) / fy * z_world;
      object_point[i].point.y = -(y_pos[i] - cy) / fy * z_world_temp[i];//(x_pos - cx) / fx * z_world;
      object_point[i].point.y = - object_point[i].point.y;    // based on observation
    }
    else
    {
      object_point[i].point.x = z_world_temp[i];
      object_point[i].point.y = z_world_temp[i];
      object_point[i].point.z = z_world_temp[i];
    }
  }
}

/*********************
  display_bBox
**********************/
void maze_object::display_bBox(Mat detected_image)
{
  if(!rgb_input.empty())
  {
    for(int i=0;i<number_same_color;i++){
      rectangle( detected_image, bBoxes[i].tl(), bBoxes[i].br(), Scalar(0,255,0), 2, 8, 0 );
    }
  }
}


/*********************
  Main loop
**********************/
int main(int argc, char **argv)
{
  int loop_frequency = 20;

  // create an object of std::msg::String type for state mc message
  std_msgs::String sm_msg;
  std::stringstream ss;
  ros::init(argc, argv, "object_detection_test");
  
  ros::NodeHandle n;
  ros::Rate loop_rate(loop_frequency);
  image_transport::ImageTransport it(n);
  
  // VideoCapture cap(0);
  // if(!cap.isOpened()){
	//   std::cout<<"Error opening the webcam" <<std::endl;
	//   return -1;
	// }

  // Subscribers to RGB and Depth image
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, callback_inputDepth);
  image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_rect_color", 1, callback_inputRGB);
  
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
  
  Mat frame;

  while(ros::ok())
  {
    maze_object object[6];
    // bool bSuccess = cap.read(frame);
	  // if(!bSuccess){
    //   std::cout<<"Cannot read a frame from webcam"<<std::endl;
		//   break;
    // }

    int bBox_area = 0;

    Mat img_clone = frame.clone();
    Mat detected_image = frame.clone();
    Mat img_hsv, thresh_hsv, morph_opening;
  
    cvtColor(img_clone, img_hsv, COLOR_BGR2HSV);

    // Apply the specified morphology operation
    //Apply Opening (erosion + dilation to each image)
	  int morph_operator = 2;		//OPENING
    int morph_elem = 2;			//ELLIPSE
    int morph_size = 3	;		//SIZE of Strel
    Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

    //Green
    inRange(img_hsv, Scalar(40,160,90), Scalar(50,255,200), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[0].color_threshold = thresh_hsv;
    //Red
    inRange(img_hsv, Scalar(1,210,90), Scalar(6,255,160), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[1].color_threshold = thresh_hsv;
    //Yellow
    inRange(img_hsv, Scalar(15,210,110), Scalar(22,255,190), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[2].color_threshold = thresh_hsv;
    //Orange
    inRange(img_hsv, Scalar(7,220,110), Scalar(13,255,205), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[3].color_threshold = thresh_hsv;
    //Purple
    inRange(img_hsv, Scalar(142,45,80), Scalar(179,132,150), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[4].color_threshold = thresh_hsv;
    //Blue
    inRange(img_hsv, Scalar(90,70,45), Scalar(101,255,150), thresh_hsv);
    morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
    object[5].color_threshold = thresh_hsv;


    // find largest bounding box
    // for( int i = 0; i < 6; i++)
  	// 	  {
    //     bBox_area = int(object.bBoxes[i].width * object.bBoxes[i].height);

    //     if(bBox_area > max_bBox_area)
    //     {
    //        max_bBox_area = bBox_area;
    //        object.max_bBox_ind = i;
    //     }
    //   }

    //Obtaun bounding boxes of each object of each color
    for(int i = 0; i<6;i++){
      object[i].findBoundingBoxes();
    }

    //Display number of objects detected in each frame
    int number_objects_detected = 0;
    for(int i=0;i<6;i++){
      number_objects_detected += object[i].number_same_color;
    }
    cout<<"Objects deteted are:"<<number_objects_detected<<endl<<endl;

    //Obtain the depth and world position of each object
    //Saved in object.object_point[]
    for(int i = 0; i<6; i++){
      if(object[i].number_same_color != 0){
        object[i].obtainDepth();
      }
    }
    
    
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

    /////obj_pose_pub.publish(object[0].object_point); //!!!!!!!!!!!!!!!
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
    ///marker.pose.position.x = object[0].object_point.point.x;
    ///marker.pose.position.y = object[0].object_point.point.y; //!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!
    //marker.pose.position.z = object[0].object_point.point.z;
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
    
    
    //Display image of detected objects
    for(int i=0;i<6;i++){
      if(object[i].number_same_color!=0){
        object[i].display_bBox(detected_image);
      }
    }
    imshow("Detected objects", detected_image);
    if(waitKey(30)==27){
      std::cout<<"esc key pressed"<<std::endl;
      break;
	  }
    ros::spinOnce();
  }

  //ros::spin();
  //destroyWindow("view");
}
