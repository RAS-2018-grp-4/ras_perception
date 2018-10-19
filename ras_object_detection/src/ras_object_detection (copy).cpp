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

class maze_object{
  public:
    char color;
    //COLORS REFERENCE
    //n: not defined, p: purple, b: blue, y: yellow, o: orange, g: green, r: red
    char shape;
    //SHAPES REFERENCE
    //n: not defined, c: cube, s: star, t: triangle, o: sphere, u: cilinder, w: weird shape
    maze_object(char col = 'n', char shp = 'n');
    char get_color();
    void set_color(char col);
    char get_shape();
    void set_shape(char shp);
};

maze_object::maze_object(char col, char shp){
  color = col;
  shape = shp;
}
char maze_object::get_color(){return color;}
char maze_object::get_shape(){return shape;}
void maze_object::set_color(char col){color = col;}
void maze_object::set_shape(char shp){shape = shp;}


bool detectObject(Mat objects[]){
  return 0;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ras_object_detection");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  image_transport::ImageTransport it(n);
  image_transport::Publisher img_pub = it.advertise("camera_bgr_image", 1);
  ros::Publisher done_pub = n.advertise<std_msgs::String>("flag_done", 1);

  VideoCapture cap(0);
  if(!cap.isOpened()){
    std::cout<<"Error opening the webcam"<<std::endl;
    return -1;
  }

  Mat cam_image;
  Mat image_clone;
  Mat hsv_image;
  //Mat channelHSV[3];
  Mat green_image;
  Mat red_image;
  Mat yellow_image;
  Mat orange_image;
  Mat purple_image;
  Mat blue_image;
  Mat objects[6];

  while(ros::ok()){

    bool frame_success = cap.read(cam_image);
    if(!frame_success){
      std::cout<<"Cannot read a frame from webcam"<<std::endl;
      break;
    }

    //Image operations
    cvtColor(cam_image, hsv_image, CV_BGR2HSV);
    image_clone = cam_image.clone();

    //split(hsv_image, channelHSV);

    //Separate the HSV using Hue as threshold for geting colors in image
    inRange(hsv_image, Scalar(40,160,90), Scalar(50,255,200), green_image);
    objects[0] = green_image;
    inRange(hsv_image, Scalar(1,210,90), Scalar(6,255,160), red_image);
    objects[1] = red_image;
    inRange(hsv_image, Scalar(15,210,110), Scalar(22,255,190), yellow_image);
    objects[2] = yellow_image;
    inRange(hsv_image, Scalar(7,220,110), Scalar(13,255,205), orange_image);
    objects[3] = orange_image;
    inRange(hsv_image, Scalar(142,45,80), Scalar(179,132,150), purple_image);
    objects[4] = purple_image;
    inRange(hsv_image, Scalar(90,70,45), Scalar(101,255,150), blue_image);
    objects[5] = blue_image;

    for(int i=0;i<6;i++){
      if(countNonZero(objects[i]) > 600){ 
        //We suppose there is an object when bigger than 18x18 pixels in binary image
        std::cout<<countNonZero(objects[i])<<std::endl;
        std::cout<<"Found an object!"<<std::endl;
        erode(objects[i], objects[i],Mat());
        dilate(objects[i], objects[i],Mat());

        std_msgs::String msg;
        std::stringstream ss;
        ss << "detect_object_done";
        msg.data = ss.str();

        done_pub.publish(msg);
        ros::spinOnce();

      }

    }




    //Results shown
    imshow("Green objects", green_image);
    //imshow("Camera image", cam_image);
    //imshow("HSV Output", hsv_image);

    if(waitKey(30)==27){
      std::cout<<"esc key pressed"<<std::endl;
      break;
    }

    //Convert the opencv image to Ros image and 
    //publish the ros image to RViz for visualization
    sensor_msgs::ImagePtr img_msg;
    img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_image).toImageMsg();
		img_pub.publish(img_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
