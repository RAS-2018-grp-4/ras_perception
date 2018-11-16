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
#include <object_detection_test/objects_found.h>

using namespace cv;
using namespace std;

//Global images to use
Mat rgb_input, depth_input;
Mat depth_input_backup;
//vector<vector<float>> depth_input_backup;


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
    depth_input_backup = depth_input.clone();
    // for(int indexx = 0;indexx<depth_input.rows;indexx++){
    //   for(int indexy = 0;indexy<depth_input.cols;indexy++){
    //     depth_input_backup[].push_back()
    //   }
    // }
    //depth_input_backup;
    //cout<<"Depth at 320,240 is: \t"<<depth_input.at<float>(240,320)<<"\n";
    //cout<< depth_input;
    //cout<<"\n\n\n\n\n\n\n§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§\n\n\n\n\n\n\n";
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
  }
}

class object_detected{
  public:
  geometry_msgs::PointStamped position;
  int color; //0: green, 1: red, 2: yellow, 3: orange, 4: purple, 5:blue
  object_detected(geometry_msgs::PointStamped,int);
};

//Overloaded constructor if known position and color
object_detected::object_detected(geometry_msgs::PointStamped obj_position, int obj_color){
  position = obj_position;
  color = obj_color;
}

class maze_object
{
  public:
  // image containers
  Mat color_threshold;
  int number_same_color;

  float fx = 616.344;
  float fy = 616.344;
  float cx = 314.855;
  float cy = 223.877;

  vector<geometry_msgs::PointStamped> vector_position;

  // Max and Min Depth in meters (0 and 255 respectively on depth image)
  float max_depth = 2.0;  
  float min_depth = 0.05; 

  vector<Rect> bBoxes; //Array of bounding boxes of objects of each color

  maze_object();
  // Subscriber callbacks
  void obtainDepth();
  void findBoundingBoxes();  // detecting regions of interest based on color 
  void display_bBox(Mat,int); // Display Bounding box of each object

};

/*********************
  Constructor
**********************/
maze_object::maze_object(){
  number_same_color = 0;
}

/*********************
  findBoundingBoxes
**********************/
void maze_object::findBoundingBoxes() //This function returns bounding boxes of each possible object of one color
{
  // Find Contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(this->color_threshold, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  if (contours.size() != 0)
  {
    // find large objects of one color
    double ctr_area = 0;
    this->number_same_color = 0;
    for ( int ind = 0; ind < contours.size(); ind++)
    {
      ctr_area = contourArea(contours[ind]);
      if(ctr_area > 2000.0)
      {
        bBoxes.push_back(boundingRect(Mat(contours[ind])));
        this->number_same_color+=1;
      }
    }
    
  }
  else
  {
    //std::cout<<"No contours detected \n";
  }
}

/*********************
  obtainDepth
**********************/
void maze_object::obtainDepth(){
  int x_pos[number_same_color];
  int y_pos[number_same_color];
  float z_world_temp[number_same_color];
  vector<int> defectuous_index;
  if(!depth_input_backup.empty()){  
    for(int i=0; i<number_same_color;i++){
      x_pos[i] = int(bBoxes[i].x + bBoxes[i].width/2);
      y_pos[i] = int(bBoxes[i].y + bBoxes[i].height/2);
      float temporary_z = 0.0;
      int temporary_counter = 0;
      //We suppose it won't go out of boundaries of the image
      //We obtain the mean of the depth around the center of the object detected
      for(int j = 0;j<3;j++){
        for(int k = 0;k<3;k++){
          if(!isnan(depth_input_backup.at<float>(y_pos[i]-1+j, x_pos[i]-1+k))){
            //cout<<depth_input_backup.at<float>(y_pos[i]-1+j, x_pos[i]-1+k)<<endl;
            temporary_counter += 1;
            temporary_z += depth_input_backup.at<float>(y_pos[i]-1+j, x_pos[i]-1+k);
          }  
        }
      }
      if(temporary_counter != 0){
        z_world_temp[i] = temporary_z / temporary_counter;
        z_world_temp[i] = - z_world_temp[i];     // observation suggests so
        geometry_msgs::PointStamped temp_position;
        temp_position.header.frame_id = "/camera_link";
        temp_position.header.stamp = ros::Time();
        temp_position.point.z = z_world_temp[i];
        temp_position.point.x = ((float)x_pos[i] - cx) / fx * z_world_temp[i];//(y_pos - cy) / fy * z_world;
        temp_position.point.y = -((float)y_pos[i] - cy) / fy * z_world_temp[i];//(x_pos - cx) / fx * z_world;
        temp_position.point.y = - temp_position.point.y;    // based on observation
        cout<<temp_position<<endl;
        vector_position.push_back(temp_position);
      }
      else{
        cout<<"All depths are nan. Therefore the object detected is useless"<<endl;
        defectuous_index.push_back(i);
      }
    }

    if(defectuous_index.size() != 0){
      //There are useless objects detected
      for(int q = 0;q<defectuous_index.size();q++){
        number_same_color -= 1; //we eliminate the object from the list
        bBoxes.erase(bBoxes.begin()+defectuous_index[q]-q);
      }
    }
  }
}

/*********************
  display_bBox
**********************/
void maze_object::display_bBox(Mat detected_image, int index)
{
  for(int i=0;i<this->number_same_color;i++){
    rectangle( detected_image, this->bBoxes[i].tl(), this->bBoxes[i].br(), Scalar(0,255,0), 2, 8, 0 );
  }
}


/*********************
  Main loop
**********************/
int main(int argc, char **argv)
{
  int loop_frequency = 20;

  std_msgs::String sm_msg; // create an object of std::msg::String type for state mc message
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
  //image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image", 1, &callback_inputDepth);
  image_transport::Subscriber depth_sub = it.subscribe("/camera/depth_registered/sw_registered/image_rect", 1, &callback_inputDepth);
  image_transport::Subscriber rgb_sub = it.subscribe("/camera/rgb/image_rect_color", 1, &callback_inputRGB);
 
  //ros::Publisher obj_pose_pub = n.advertise<geometry_msgs::PointStamped>("/object_position_cam_link", 100);
  ros::Publisher obj_pose_pub = n.advertise<object_detection_test::objects_found>("/object_position_cam_link", 1);
  // Publisher for rviz marker
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/object_marker", 1);
  
  //----------------MAIN LOOP----------------------

  while(ros::ok())
  {
    //vector<maze_object> object;

    maze_object green_objects; 
    maze_object red_objects; 
    maze_object yellow_objects; 
    maze_object orange_objects; 
    maze_object purple_objects; 
    maze_object blue_objects; 
    vector<object_detected> vector_objects_detected; //vector that we will publish
    // bool bSuccess = cap.read(frame);
	  // if(!bSuccess){
    //   std::cout<<"Cannot read a frame from webcam"<<std::endl;
		//   break;
    // }
    if(!rgb_input.empty()){
      Mat img_clone = rgb_input.clone();
      Mat detected_image = rgb_input.clone();
      Mat img_hsv, thresh_hsv, morph_opening;
    
      cvtColor(img_clone, img_hsv, COLOR_BGR2HSV);

      // Apply the specified morphology operation
      //Apply Opening (erosion + dilation to each image)
      int morph_operator = 2;		//OPENING
      int morph_elem = 2;			//ELLIPSE
      int morph_size = 3	;		//SIZE of Strel
      Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
      
      int number_objects_detected = 0;
      
      int number_detected = 0;


      //---------------START OF PROCESSING FOR EACH COLOR--------------------

      //Green : 0
      //First, threshold the image with HSV channel
      inRange(img_hsv, Scalar(40,160,90), Scalar(50,255,200), thresh_hsv);
      //Then, we apply opening to the thresholded image
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      green_objects.color_threshold = morph_opening;
      //We find the bounding boxes using the "opened" image
      green_objects.findBoundingBoxes();
      number_objects_detected += green_objects.number_same_color;
      if(green_objects.number_same_color != 0){
        //We have found at least an object
        //We obtain the depth from depth input of RGBD camera
        green_objects.obtainDepth();
        //We eliminate the object if depth = nan
        if(green_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < green_objects.number_same_color; j++){
            //We add the detected objects to a common vector, to be published
            object_detected temp_object(green_objects.vector_position[j],0);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          green_objects.display_bBox(detected_image,0);
        }
      }

      //Red : 1
      inRange(img_hsv, Scalar(1,210,90), Scalar(6,255,160), thresh_hsv);
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      red_objects.color_threshold = morph_opening;
      red_objects.findBoundingBoxes();
      number_objects_detected += red_objects.number_same_color;
      
      if(red_objects.number_same_color != 0){

        red_objects.obtainDepth();
        if(red_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < red_objects.number_same_color; j++){
            object_detected temp_object(red_objects.vector_position[j],1);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          red_objects.display_bBox(detected_image,1);
        }
      }

      //Yellow : 2
      inRange(img_hsv, Scalar(15,210,110), Scalar(22,255,190), thresh_hsv);
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      yellow_objects.color_threshold = morph_opening;
      yellow_objects.findBoundingBoxes();
      number_objects_detected += yellow_objects.number_same_color;
      if(yellow_objects.number_same_color != 0){
        yellow_objects.obtainDepth();
        if(yellow_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < yellow_objects.number_same_color; j++){
            object_detected temp_object(yellow_objects.vector_position[j],2);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          yellow_objects.display_bBox(detected_image,2);
        }
      }

      //Orange : 3
      inRange(img_hsv, Scalar(7,220,110), Scalar(13,255,205), thresh_hsv);
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      orange_objects.color_threshold = morph_opening;
      orange_objects.findBoundingBoxes();
      number_objects_detected += orange_objects.number_same_color;
      if(orange_objects.number_same_color != 0){
        orange_objects.obtainDepth();
        if(orange_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < orange_objects.number_same_color; j++){
            object_detected temp_object(orange_objects.vector_position[j],3);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          orange_objects.display_bBox(detected_image,3);
        }
      }

      //Purple : 4
      inRange(img_hsv, Scalar(142,45,80), Scalar(179,132,150), thresh_hsv);
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      purple_objects.color_threshold = morph_opening;
      purple_objects.findBoundingBoxes();
      number_objects_detected += purple_objects.number_same_color;
      if(purple_objects.number_same_color != 0){
        purple_objects.obtainDepth();
        if(purple_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < purple_objects.number_same_color; j++){
            object_detected temp_object(purple_objects.vector_position[j],4);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          purple_objects.display_bBox(detected_image,4);
        }
      }

      //Blue : 5
      inRange(img_hsv, Scalar(90,70,45), Scalar(101,255,150), thresh_hsv);
      morphologyEx( thresh_hsv, morph_opening, morph_operator, element );
      blue_objects.color_threshold = morph_opening;
      blue_objects.findBoundingBoxes();
      number_objects_detected += blue_objects.number_same_color;
      if(blue_objects.number_same_color != 0){
        blue_objects.obtainDepth();
        if(blue_objects.number_same_color != 0){ //It may have changed
          for(int j = 0; j < blue_objects.number_same_color; j++){
            object_detected temp_object(blue_objects.vector_position[j],5);
            vector_objects_detected.push_back(temp_object);
            number_detected += 1;
          }
          blue_objects.display_bBox(detected_image,5);
        }
      }

      //-----------------FINISHED PROCESSING--------------------


      //-----------------START OF PUBLISHING--------------------

      //Display number of objects detected in each frame
      //cout<<"Objects detected are:"<<number_objects_detected<<endl;
      //cout<<"Objects detected with depth not nan are:"<<number_detected<<endl;


      if(vector_objects_detected.size() != 0){
        //We have good objects to publish
        //We use the created msg type

        object_detection_test::objects_found temp_objects;
        temp_objects.number_of_objects = vector_objects_detected.size();
        for(int index_object = 0; index_object<temp_objects.number_of_objects;index_object++){
          temp_objects.array_objects_found.push_back(vector_objects_detected[index_object].position);
          temp_objects.array_colors.push_back(vector_objects_detected[index_object].color);
        }
        obj_pose_pub.publish(temp_objects);
      }

      imshow("Detected objects", detected_image);
    }
    else{
      cout<<"NO INPUT IMAGE RECEIVED!"<<endl;
    }
    if(waitKey(30)==27){
      std::cout<<"esc key pressed"<<std::endl;
      break;
	  }
    //cout<<endl<<endl<<endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}