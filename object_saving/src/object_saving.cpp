#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/MarkerArray.h>
#include "object_saving/objects.h"
#include "object_saving/objects_found.h"
#include <sstream>
#include <limits>
#include <iostream>
#include <fstream>

using namespace std;

class map_object{

    public:

    //Members: shape, position, its value and if it has been already picked or not
    vector<int> shape; // 0: Ball, 1: Cube, 2: Cylinder, 3: Hollow cube, 4: Cross
                       // 5: Triangle, 6: Star, 7: Nothing, 8: Obstacle!!!
    geometry_msgs::PointStamped map_position;
    int color;
    int value;
    float real_priority_value;
    int probability;
    bool picked;

    map_object();
    map_object(geometry_msgs::PointStamped, int, int,float);

    //void publish_position_best_object();
};

//Default constructor
map_object::map_object(void){
    vector<int> temp_shape(9,0);
    temp_shape[7] = 100;
    shape = temp_shape;
    color = 0;
    probability = 50;
    value = 1;
    picked = false;
}

//Constructor overloaded with known position, color and shape
map_object::map_object(geometry_msgs::PointStamped position, int this_color, int this_shape, float this_distance){
    vector<int> temp_shape(9,0);
    color = this_color;
    switch(this_color){
        case 0: //the object is yellow
            if((this_shape != 0) || (this_shape != 1)){
                temp_shape[7] = 100; //The identified shape is not possible
                value = 1;
            }
            else if(this_shape == 0){ //It is a yellow ball
                temp_shape[this_shape] = 100;
                value = 5;
            }
            else{ //It is a yellow cube
                temp_shape[this_shape] = 100;
                value = 10;
            }
            shape = temp_shape;
            break;
        case 1: //the object is green
            switch(this_shape){
                case 1: //It is a green cube
                    temp_shape[this_shape] = 100;
                    value = 30;
                    break;
                case 2: //It is a green cylinder
                    temp_shape[this_shape] = 100;
                    value = 40;
                    break;
                case 3: //It is a green hollow cube
                    temp_shape[this_shape] = 100;
                    value = 50;
                    break;
                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
        case 2: //the object is orange
            switch(this_shape){
                case 4: //It is a orange cross
                    temp_shape[this_shape] = 100;
                    value = 100;
                    break;
                case 6: //It is Patric!!
                    temp_shape[this_shape] = 100;
                    value = 1000;
                    break;
                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
        case 3: //the object is red
            switch(this_shape){
                case 2: //It is a red cylinder
                    temp_shape[this_shape] = 100;
                    value = 30;
                    break;
                case 3: //It is a red hollow cube
                    temp_shape[this_shape] = 100;
                    value = 130;
                    break;
                case 0: //It is a red ball
                    temp_shape[this_shape] = 100;
                    value = 15;
                    break;
                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
        case 4: //the object is blue
            switch(this_shape){
                case 1: //It is a blue cube
                    temp_shape[this_shape] = 100;
                    value = 60;
                    break;
                case 5: //It is a blue triangle
                    temp_shape[this_shape] = 100;
                    value = 70;
                    break;

                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
        case 5: //the object is purple
            switch(this_shape){
                case 4: //It is a purple cross
                    temp_shape[this_shape] = 100;
                    value = 120;
                    break;
                case 6: //It is a purple star
                    temp_shape[this_shape] = 100;
                    value = 200;
                    break;

                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
  
    }
    shape = temp_shape;
    real_priority_value = (float)value / (2*this_distance);
    picked = false;
    probability = 50;
    map_position = position;
}

//Global variables needed for this node
std::vector<map_object> objects;
int number_objects;
geometry_msgs::Twist current_robot_vel;

void analyze_objects(object_saving::objects_found objects_found){
    vector<vector<int> > index_conc;
    object_saving::objects_found objects_found_temp = objects_found;

    if(abs(current_robot_vel.angular.z) < 0.2){

        //Robustness against same object detected in two bounding boxes
        for(int j = 0; j < objects_found.number_of_objects;j++){
            vector<int> temp_coincidence;
            temp_coincidence.push_back(j);
            for(int k = 0; k < objects_found.number_of_objects;k++){
                if(k != j){
                    if(sqrt(pow(objects_found.array_objects_found[j].point.x-objects_found.array_objects_found[k].point.x,2)+ pow(objects_found.array_objects_found[j].point.y-objects_found.array_objects_found[k].point.y,2)) < 0.05){
                        if(objects_found.array_colors[j] == objects_found.array_colors[k]){
                            //There is an object detected twice or more
                            temp_coincidence.push_back(k);
                        }
                    }
                }
            }
            if(temp_coincidence.size() > 1){
                index_conc.push_back(temp_coincidence);
                //cout<<"Detected a duplicate"<<endl;
            }
        }

        int finish_for = index_conc.size();
        if(finish_for != 0){
            for(int j = 0; j < finish_for; j++){
                for(int k = 0; k < finish_for; k++){
                    int equal = 0;
                    if((j != k) && (index_conc[j].size() == index_conc[k].size())){
                        for(int q = 0; q < index_conc[k].size(); q++){
                            if(index_conc[j][0] == index_conc[k][q]){
                                //j = k; k is eliminated from the vector
                                equal = 1;
                            }
                        }
                    }
                    if(equal != 0){
                        finish_for -= 1;
                        index_conc.erase(index_conc.begin()+1);
                    }
                }
            }

            for(int j = 0; j < finish_for; j++){
                float avg_x = 0.0;
                float avg_y = 0.0;
                for(int k = 0; k < index_conc[j].size();k++){
                    avg_x += objects_found_temp.array_objects_found[index_conc[j][k]].point.x;
                    avg_y += objects_found_temp.array_objects_found[index_conc[j][k]].point.y;
                }
                avg_x /= index_conc[j].size();
                avg_y /= index_conc[j].size();
                objects_found_temp.array_objects_found[index_conc[j][0]].point.x = avg_x;
                objects_found_temp.array_objects_found[index_conc[j][0]].point.y = avg_y;
                //Now we eliminate the same objects
                for(int k = 1; k < index_conc[j].size();k++){
                    objects_found_temp.array_objects_found.erase(objects_found_temp.array_objects_found.begin()+index_conc[j][k]-1);
                    objects_found_temp.array_colors.erase(objects_found_temp.array_colors.begin()+index_conc[j][k]-1);
                    objects_found_temp.number_of_objects -= 1;
                    //Now we gotta change the vector of indexes of duplicate objects
                    for(int q = 0;q<index_conc.size();q++){
                        for(int w = 0; w<index_conc[q].size();w++){
                            if(index_conc[q][w] >= index_conc[j][k]) index_conc[q][w] -= 1;
                        }
                    }
                }
            }
        }
        //else cout<<"No duplicates detected"<<endl;

        //Once having eliminated duplicated detected objects,
        //they are marked in the map if they are not close to another existing object in the map 
        //If there is detecting the same object, the probability of it gets higher by 10 (until 1000)
        if(objects.size()!= 0){
            int previous_size = objects.size();
            vector<int> objects_same(previous_size,0);
            vector<int> objects_same_detected(previous_size,0);
            for(int q = 0; q < objects_found_temp.number_of_objects;q++){
            //Compare if it's the same object, not taking into account the closest ones
                if((!std::isnan(objects_found_temp.array_objects_found[q].point.x)) && (!std::isnan(objects_found_temp.array_objects_found[q].point.y)) && (!std::isnan(objects_found_temp.array_objects_found[q].point.z))){
                    int temp_counter = 0;
                    for(int i= 0; i < objects.size(); i++){
                        if(sqrt(pow(objects_found_temp.array_objects_found[q].point.x-objects[i].map_position.point.x,2)+ pow(objects_found_temp.array_objects_found[q].point.y-objects[i].map_position.point.y,2))> 0.10){
                            temp_counter += 1;
                        }
                        else if(objects_found_temp.array_colors[q] == objects[i].color){
                            //We have found the same object as one previously found
                            objects_same[i] = 1;
                            objects_same_detected[i] = q; 

                            //objects_same.push_back(i); //??
                        }
                    }
                    if(temp_counter == objects.size()){
                        float distance_to_beginning = sqrt(pow(objects_found_temp.array_objects_found[q].point.x-0.2,2) + pow(objects_found_temp.array_objects_found[q].point.y-0.2,2));
                        map_object temp_object(objects_found_temp.array_objects_found[q],objects_found_temp.array_colors[q],objects_found_temp.array_shape[q],distance_to_beginning);
                        objects.push_back(temp_object);
                        number_objects += 1;
                        cout<<"Found a new object"<<endl;
                    }
                    else{
                        cout<<"Detected but too close to an object"<<endl;
                        //objects[same_index].probability += 10; //We have found the same object, so probability gets higher
                    } 
                }
            }
            for(int q = 0; q < previous_size; q++){
                if(objects_same[q] == 1){
                    //ADD HERE CODE FOR CHANGING PROBABILITY OF SHAPE------------
                    //WHEN SHAPE IS DETECTED AND NOT UNKNOWN---------------------
                    if(objects[q].probability < 991) objects[q].probability += 5; //We have found the same object, so probability gets higher
                    else objects[q].probability = 1000;
                    if(objects[q].shape[7] == 100){
                        objects[q].shape[objects_found_temp.array_shape[objects_same_detected[q]]] = 100;
                        objects[q].shape[7] = 0;
                    } 
                    else if(objects[q].shape[8] != 100){
                        for(int w = 0; w < 9; w++){
                            if(w != objects_found_temp.array_shape[objects_same_detected[q]]){
                                if(objects[q].shape[w] > 0) objects[q].shape[w] -= 1;
                            }
                            else if(objects[q].shape[w] < 100) objects[q].shape[objects_found_temp.array_shape[objects_same_detected[q]]] += 1;
                        }
                    }
                } 
                else if(objects_same[q] == 0){
                    if(objects[q].probability > 50) objects[q].probability -= 1; //we have not found this object, so prob. gets reduced
                } 
            }
            // if(objects_found_temp.number_of_objects == 0){
            //     for(int q = 0; q < objects.size(); q++){
            //         objects[q].probability -= 1;
            //     }
            // }
        }

        else{
            //There are no objects yet, so the first object is created in the vector
            for(int q = 0;q<objects_found_temp.number_of_objects;q++){
                if((!std::isnan(objects_found_temp.array_objects_found[q].point.x)) && (!std::isnan(objects_found_temp.array_objects_found[q].point.y)) && (!std::isnan(objects_found_temp.array_objects_found[q].point.z))){
                    float distance_to_beginning = sqrt(pow(objects_found_temp.array_objects_found[q].point.x-0.2,2) + pow(objects_found_temp.array_objects_found[q].point.y-0.2,2));
                    map_object temp_object(objects_found_temp.array_objects_found[q],objects_found_temp.array_colors[q],objects_found_temp.array_shape[q],distance_to_beginning);
                    objects.push_back(temp_object);
                    number_objects += 1;
                }
            }
        }
    }
}

void position_callBack(const object_saving::objects_found objects_found){
    analyze_objects(objects_found);
}
void position_classical_callBack(const object_saving::objects_found objects_found){
    analyze_objects(objects_found);
}

void robot_vel_callBack(const geometry_msgs::Twist robot_vel){
    current_robot_vel = robot_vel;
}

void smach_callBack(const bool flag_picked){
    if(flag_picked){
        //-----------HERE CODE FOR CHANGING FLAG OF OBJECT PICKED-------------(
            int best_value = 0;
            int best_index;
        for(int i = 0; i < objects.size();i++){
            if(objects[i].value > best_value){
                best_value = objects[i].value;
                best_index = i;
            }
        }
        if(!std::isnan(best_index)) objects[best_index].picked = true;
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

void objects_to_file(vector<map_object> objects_to_write)
{
    ofstream file;
    file.open("/home/ras14/catkin_ws/src/round1objects.txt");
    
    string output = "";  
    for (int i = 0; i < objects_to_write.size(); i++)
    {     
        output += to_string(number_objects) + "\n";
        //output += "shape#";
        for(int j = 0; j < objects_to_write[i].shape.size(); j++){
            output += std::to_string(objects_to_write[i].shape[j]) + ",";
        }
        output += "\n";
        //output += "map_position#";
        output += std::to_string(objects_to_write[i].map_position.point.x) + ",";
        output += std::to_string(objects_to_write[i].map_position.point.y) + ",";
        output += std::to_string(objects_to_write[i].map_position.point.z) + "," + "\n";
        //output += "color#";
        output += std::to_string(objects_to_write[i].color) + "\n";
        //output += "value#";
        output += std::to_string(objects_to_write[i].value) + "\n";
        //output += "real_priority_value#";
        output += std::to_string(objects_to_write[i].real_priority_value) + "\n";
        //output += "probability#";
        output += std::to_string(objects_to_write[i].probability) + "\n";
        //output += "picked#";
        if(objects_to_write[i].picked){
            output += std::to_string(1) + "\n";
        } 
        else output += std::to_string(0) + "\n";

    }
    file << output;
    file.close();

    //cout << output <<endl<<endl<<endl<<endl;
}

void refresh_objects(){

    ifstream myfile;
    myfile.open("/home/ras14/catkin_ws/src/round1objects.txt");

    if (myfile.is_open()){
        //-------------ADD HERE CODE FOR DECODING INFORMATION OF TXT FILE--------------------
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "object_saving");
    ros::NodeHandle n;
    ros::Rate r(5);


    //------HERE ROS SUBSCRIBER TO STATE MACHINE FLAG FOR OBJECT PICKED
    ros::Subscriber sub_new_position = n.subscribe("/object_position_map", 1, position_callBack);
    ros::Subscriber sub_new_position_classical = n.subscribe("/object_position_map_classical", 1, position_classical_callBack);
    ros::Subscriber sub_robot_velocity = n.subscribe("/vel", 1, robot_vel_callBack);
    
    ros::Publisher objects_pub = n.advertise<object_saving::objects>("objects_detected", 1);
    ros::Publisher best_object_pub =  n.advertise<geometry_msgs::PointStamped>("/best_object", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/all_objects_marker", 1);
    ros::Publisher current_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/current_objects_marker", 1);

    std::ifstream ifile;
    ifile.open("/home/ras14/catkin_ws/src/round1objects.txt");
    if ((bool)ifile)
    {
        refresh_objects();
        cout<< "FOUND"<<endl;
    }
    else
    {
        cout<< "NOT FOUND"<<endl;
    }


    while(ros::ok()){

        if((objects.size() != 0) && (number_objects != 0)){

            objects_to_file(objects);

            visualization_msgs::MarkerArray markers;
            markers.markers.resize(objects.size());
            visualization_msgs::MarkerArray markers_current;
            object_saving::objects temp_objects;
            temp_objects.number_of_objects = number_objects;
            int best_value = 0;
            int best_index;
            for(int i= 0; i< objects.size(); i++){
                if(objects[i].probability > 50) objects[i].probability -= 1;
                temp_objects.objects_detected_position.push_back(objects[i].map_position);
                temp_objects.array_probability.push_back(objects[i].probability);
                temp_objects.array_real_priority_value.push_back(objects[i].real_priority_value);
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
                markers.markers[i].scale.x = 0.03;
                markers.markers[i].scale.y = 0.03;
                markers.markers[i].scale.z = 0.03;

                // Set the color -- be sure to set alpha to something non-zero!
                markers.markers[i].color.a = 1.0;
                switch(objects[i].color){
                            case 0: //the object is yellow
                                markers.markers[i].color.r = 1.0f;
                                markers.markers[i].color.g = 1.0f;
                                markers.markers[i].color.b = 0.0f;
                                break;
                            case 1: //the object is green
                                markers.markers[i].color.r = 0.0f;
                                markers.markers[i].color.g = 1.0f;
                                markers.markers[i].color.b = 0.0f;
                                break;
                            case 2: //the object is orange
                                markers.markers[i].color.r = 1.0f;
                                markers.markers[i].color.g = 0.7f;
                                markers.markers[i].color.b = 0.0f;
                                break;
                            case 3: //the object is red
                                markers.markers[i].color.r = 1.0f;
                                markers.markers[i].color.g = 0.0f;
                                markers.markers[i].color.b = 0.0f;
                                break;
                            case 4: //the object is blue
                                markers.markers[i].color.r = 0.0f;
                                markers.markers[i].color.g = 0.0f;
                                markers.markers[i].color.b = 1.0f;
                                break;
                            case 5: //the object is purple
                                markers.markers[i].color.r = 0.5f;
                                markers.markers[i].color.g = 0.0f;
                                markers.markers[i].color.b = 0.5f;
                                break;
                }             
                markers.markers[i].lifetime = ros::Duration(0.25);
                
            }

            vector<map_object> objects_current;
            for(int i = 0; i < objects.size();i++){
                if(objects[i].probability > 50){
                    objects_current.push_back(objects[i]);
                }
            }
            //cout<<"I'm here"<< endl;
            markers_current.markers.resize(objects_current.size());
            for(int i = 0; i < objects_current.size(); i++){
                markers_current.markers[i].header.frame_id = "/map";
                markers_current.markers[i].header.stamp = ros::Time::now();

                // Set the namespace and id for this markers[i].  This serves to create a unique ID
                // Any markers[i] sent with the same namespace and id will overwrite the old one
                markers_current.markers[i].ns = "detected current objects";
                markers_current.markers[i].id = i + objects.size();

                // Set the markers[i] type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                markers_current.markers[i].type = visualization_msgs::Marker::CUBE;

                // Set the markers[i] action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                markers_current.markers[i].action = visualization_msgs::Marker::ADD;

                // Set the pose of the markers[i].  This is a full 6DOF pose relative to the frame/time specified in the header
                markers_current.markers[i].pose.position.x = objects_current[i].map_position.point.x;
                markers_current.markers[i].pose.position.y = objects_current[i].map_position.point.y;
                markers_current.markers[i].pose.position.z = objects_current[i].map_position.point.z;
                markers_current.markers[i].pose.orientation.x = 0.0;
                markers_current.markers[i].pose.orientation.y = 0.0;
                markers_current.markers[i].pose.orientation.z = 0.0;
                markers_current.markers[i].pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                markers_current.markers[i].scale.x = 0.07;
                markers_current.markers[i].scale.y = 0.07;
                markers_current.markers[i].scale.z = 0.07;

                // Set the color -- be sure to set alpha to something non-zero!
                markers_current.markers[i].color.a = 1.0;
                switch(objects_current[i].color){
                            case 0: //the object is yellow
                                markers_current.markers[i].color.r = 1.0f;
                                markers_current.markers[i].color.g = 1.0f;
                                markers_current.markers[i].color.b = 0.0f;
                                break;
                            case 1: //the object is green
                                markers_current.markers[i].color.r = 0.0f;
                                markers_current.markers[i].color.g = 1.0f;
                                markers_current.markers[i].color.b = 0.0f;
                                break;
                            case 2: //the object is orange
                                markers_current.markers[i].color.r = 1.0f;
                                markers_current.markers[i].color.g = 0.7f;
                                markers_current.markers[i].color.b = 0.0f;
                                break;
                            case 3: //the object is red
                                markers_current.markers[i].color.r = 1.0f;
                                markers_current.markers[i].color.g = 0.0f;
                                markers_current.markers[i].color.b = 0.0f;
                                break;
                            case 4: //the object is blue
                                markers_current.markers[i].color.r = 0.0f;
                                markers_current.markers[i].color.g = 0.0f;
                                markers_current.markers[i].color.b = 1.0f;
                                break;
                            case 5: //the object is purple
                                markers_current.markers[i].color.r = 0.5f;
                                markers_current.markers[i].color.g = 0.0f;
                                markers_current.markers[i].color.b = 0.5f;
                                break;
                }             
                markers_current.markers[i].lifetime = ros::Duration(0.25);
            }
            //cout<<"I'm here"<<endl<<endl;
            //------HERE CODE TO PUBLISH ARRAY OF OBJECTS INTO RVIZ-------
            marker_pub.publish(markers);
            current_marker_pub.publish(markers_current);
            objects_pub.publish(temp_objects);
            best_object_pub.publish(objects[best_index].map_position);
        } 

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
