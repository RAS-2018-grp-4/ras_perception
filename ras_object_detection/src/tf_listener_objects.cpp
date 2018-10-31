#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PointStamped temp_point; 

void callback_pos(const geometry_msgs::PointStamped &msg)
{
  geometry_msgs::PointStamped transformed_point;
  tf::transformPoint("map", msg, transformed_point);
  // //cout<<"REACHED";
  // ::temp_point.header.frame_id = "object";

  // //we'll just use the most recent transform available for our simple example
  // ::temp_point.header.stamp = ros::Time();

  // ::object_point.point.x = msg.point.x;
  // ::object_point.point.y = msg.point.y;
  // ::object_point.point.z = msg.point.z;
  ::temp_point = msg;
}

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped object_point;
  object_point.header.frame_id = "object";

  //we'll just use the most recent transform available for our simple example
  object_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  object_point.point.x = ::temp_point.point.x;
  object_point.point.y = ::temp_point.point.y;
  object_point.point.z = ::temp_point.point.z;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", object_point, base_point);

    ROS_INFO("base_object: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        object_point.point.x, object_point.point.y, object_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  ros::Subscriber pos_sub = n.subscribe("/object_position_cam_link", 100, callback_pos);

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
