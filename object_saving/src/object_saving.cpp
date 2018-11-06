#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/MarkerArray.h>
#include "object_saving/objects.h"


class map_object{

    public:

    //Members: shape, position, its value and if it has been already picked or not
    std::string shape;
    geometry_msgs::PointStamped map_position;
    int value;
    bool picked;

    map_object();
    map_object(geometry_msgs::PointStamped);

    //void publish_position_best_object();
};

//Default constructor
map_object::map_object(void){
    shape = "unknown";
    value = 1;
    picked = false;
}

//Constructor overloaded with known position
map_object::map_object(geometry_msgs::PointStamped position){
    shape = "unknown";
    value = 1;
    picked = false;
    map_position = position;
}

//Global variables needed for this node
std::vector<map_object> objects;
int number_objects;

void position_callBack(const geometry_msgs::PointStamped position){

    if(objects.size()!= 0){
        //Compare if it's the same object, not taking into account the closest ones
        if((!std::isnan(position.point.x)) && (!std::isnan(position.point.y)) && (!std::isnan(position.point.z))){
            int temp_counter = 0;
            for(int i= 0; i < objects.size(); i++){
                if(sqrt(pow(position.point.x-objects[i].map_position.point.x,2)+ pow(position.point.y-objects[i].map_position.point.y,2))> 0.15){
                    temp_counter += 1;
                }
            }
            if(temp_counter == objects.size()){
                map_object temp_object(position);
                objects.push_back(temp_object);
                number_objects += 1;
            }
        }
    }

    else{
        //There are no objects yet, so the first object is created in the vector
        if((!std::isnan(position.point.x)) && (!std::isnan(position.point.y)) && (!std::isnan(position.point.z))){
            map_object temp_object(position);
            objects.push_back(temp_object);
            number_objects += 1;
        }
    }
}

void smach_callBack(const bool flag_picked){
    if(flag_picked){
        //-----------HERE CODE FOR CHANGING FLAG OF OBJECT PICKED-------------
        //objects[i].picked = true;
    }

}

void identification_callBack(const std::string shape_of_object_detected){

    //-------- EDIT THIS VALUES FOR THE VALUE OF EACH OBJECT IN THE MAZE
    int value_of_shape = 1;
    if(shape_of_object_detected == "Red Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Red Hollow Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Blue Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Green Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Yellow Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Yellow Ball") value_of_shape = 10;
    else if(shape_of_object_detected == "Red Ball") value_of_shape = 10;
    else if(shape_of_object_detected == "Red Cylinder") value_of_shape = 10;
    else if(shape_of_object_detected == "Green Cylinder") value_of_shape = 10;
    else if(shape_of_object_detected == " Green Hollow Cube") value_of_shape = 10;
    else if(shape_of_object_detected == "Blue Triangle") value_of_shape = 10;
    else if(shape_of_object_detected == "Purple Cross") value_of_shape = 10;
    else if(shape_of_object_detected == "Purple Star") value_of_shape = 10;
    else if(shape_of_object_detected == "Orange Cross") value_of_shape = 10;
    else if(shape_of_object_detected == "Patric") value_of_shape = 10;

    //-----------HERE THE CODE FOR CHOOSING WHICH OBJECT IS BEING IDENTIFIED----------
    ///objects[i].shape = shape_of_object_detected;
    //objects[i].value = value_of_shape;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "object_saving");
    ros::NodeHandle n;
    ros::Rate r(10);



    //------HERE ROS SUBSCRIBER TO IDENTIFICATION BY DARKNET_ROS
    //------HERE ROS SUBSCRIBER TO STATE MACHINE FLAG FOR OBJECT PICKED
    ros::Subscriber sub_new_position = n.subscribe("/object_position_map", 1, position_callBack);
    ros::Publisher objects_pub = n.advertise<object_saving::objects>("objects_detected", 1);
    ros::Publisher best_object_pub =  n.advertise<geometry_msgs::PointStamped>("/best_object", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/all_objects_marker", 1);

    while(ros::ok()){

        if((objects.size() != 0) && (number_objects != 0)){
            visualization_msgs::MarkerArray markers;
            markers.markers.resize(objects.size());
            object_saving::objects temp_objects;
            temp_objects.number_of_objects = number_objects;
            int best_value = 0;
            int best_index;
            for(int i= 0; i< objects.size(); i++){
                temp_objects.objects_detected.push_back(objects[i].map_position);
                if(objects[i].value > best_value){
                    best_index = i;
                    best_value = objects[i].value;
                }

                
                markers.markers[i].header.frame_id = "/map";
                markers.markers[i].header.stamp = ros::Time::now();

                // Set the namespace and id for this markers[i].  This serves to create a unique ID
                // Any markers[i] sent with the same namespace and id will overwrite the old one
                markers.markers[i].ns = "detected objects";
                markers.markers[i].id = i;

                // Set the markers[i] type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                markers.markers[i].type = visualization_msgs::Marker::CUBE;

                // Set the markers[i] action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                markers.markers[i].action = visualization_msgs::Marker::ADD;

                // Set the pose of the markers[i].  This is a full 6DOF pose relative to the frame/time specified in the header
                markers.markers[i].pose.position.x = objects[i].map_position.point.x;
                markers.markers[i].pose.position.y = objects[i].map_position.point.y;
                markers.markers[i].pose.position.z = objects[i].map_position.point.z;
                markers.markers[i].pose.orientation.x = 0.0;
                markers.markers[i].pose.orientation.y = 0.0;
                markers.markers[i].pose.orientation.z = 0.0;
                markers.markers[i].pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                markers.markers[i].scale.x = 0.05;
                markers.markers[i].scale.y = 0.05;
                markers.markers[i].scale.z = 0.05;

                // Set the color -- be sure to set alpha to something non-zero!
                markers.markers[i].color.r = 0.0f;
                markers.markers[i].color.g = 1.0f;
                markers.markers[i].color.b = 0.0f;
                markers.markers[i].color.a = 1.0;

                markers.markers[i].lifetime = ros::Duration(0.25);
                
            }
            //------HERE CODE TO PUBLISH ARRAY OF OBJECTS INTO RVIZ-------
            marker_pub.publish(markers);
            objects_pub.publish(temp_objects);
            best_object_pub.publish(objects[best_index].map_position);
        } 

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
