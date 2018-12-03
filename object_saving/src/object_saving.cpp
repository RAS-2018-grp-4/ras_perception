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

//Global variables needed for this node
int number_objects;
geometry_msgs::Twist current_robot_vel;

//Global int parameters of the values of the objects
int ncolors = 6;
vector< vector<int> > values_objects(ncolors);

//Global flags for speaking
bool speak_objects_round1 = false;
bool speak_object_detected = false;
std_msgs::String what_to_speak;

class map_object{

    public:

    //Members: shape, position, its value and if it has been already picked or not
    vector<int> shape; // 0: Ball, 1: Cube, 2: Cylinder, 3: Hollow cube, 4: Cross
                       // 5: Triangle, 6: Star, 7: Nothing, 8: Obstacle!!!
    geometry_msgs::PointStamped map_position;
    int color; //0: yellow, 1: 
    int value;
    float real_priority_value;
    float distance;
    int probability;
    bool picked;
    int times_detected;
    bool object_spoken;

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
    distance = 100.0;
    picked = false;
    times_detected = 0;
    object_spoken = false;
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
                temp_shape[this_shape] = 1;
                value = values_objects[0][0];
            }
            else{ //It is a yellow cube
                temp_shape[this_shape] = 1;
                value = values_objects[0][1];
            }
            shape = temp_shape;
            break;
        case 1: //the object is green
            switch(this_shape){
                case 1: //It is a green cube
                    temp_shape[this_shape] = 1;
                    value = values_objects[1][1];
                    break;
                case 2: //It is a green cylinder
                    temp_shape[this_shape] = 1;
                    value = values_objects[1][2];
                    break;
                case 3: //It is a green hollow cube
                    temp_shape[this_shape] = 1;
                    value = values_objects[1][3];
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
                    temp_shape[this_shape] = 1;
                    value = values_objects[2][4];
                    break;
                case 6: //It is Patric!!
                    temp_shape[this_shape] = 1;
                    value = values_objects[2][6];
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
                    temp_shape[this_shape] = 1;
                    value = values_objects[3][2];
                    break;
                case 3: //It is a red hollow cube
                    temp_shape[this_shape] = 1;
                    value = values_objects[3][3];
                    break;
                case 0: //It is a red ball
                    temp_shape[this_shape] = 1;
                    value = values_objects[3][0];
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
                    temp_shape[this_shape] = 1;
                    value = values_objects[4][1];
                    break;
                case 5: //It is a blue triangle
                    temp_shape[this_shape] = 1;
                    value = values_objects[4][5];
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
                    temp_shape[this_shape] = 1;
                    value = values_objects[5][4];
                    break;
                case 6: //It is a purple star
                    temp_shape[this_shape] = 1;
                    value = values_objects[5][6];
                    break;

                default: //The identified shape is not possible
                    temp_shape[7] = 100;
                    value = 1;
                    break;
            }
            break;
  
    }
    shape = temp_shape;
    distance = this_distance;
    real_priority_value = (float)value / (2*this_distance);
    picked = false;
    times_detected = 1;
    probability = 50;
    map_position = position;
    object_spoken = false;
}

std::vector<map_object> objects;
vector<geometry_msgs::PointStamped> pos_best_objects_gripped; 

void get_object_to_speak(int color, int shape){
    switch(color){
        case 0:
        //It's yellow
            switch(shape){
                case 0:
                    what_to_speak.data = "Found a yellow ball";
                    break;
                case 1:
                    what_to_speak.data = "Found a yellow cube";
                    break;
                default:
                    break;
            }
            break;
        case 1:
        //It's green
            switch(shape){
                case 1:
                    what_to_speak.data = "Found a green cube";
                    break;
                case 2:
                    what_to_speak.data = "Found a green cylinder";
                    break;
                case 3:
                    what_to_speak.data = "Found a green hollow cube";
                    break;
                default:
                    break;
            }
            break;
        case 2:
        //It's orange
            switch(shape){
                case 4:
                    what_to_speak.data = "Found a orange cross";
                    break;
                case 6:
                    what_to_speak.data = "Found Patric";
                    break;
                default:
                    break;
            }
            break;
        case 3:
        //It's red
            switch(shape){
                case 2:
                    what_to_speak.data = "Found a red cylinder";
                    break;
                case 3:
                    what_to_speak.data = "Found a red hollow cube";
                    break;
                case 0:
                    what_to_speak.data = "Found a red ball";
                    break;
                default:
                    break;
            }
            break;
        case 4:
        //It's blue
            switch(shape){
                case 1:
                    what_to_speak.data = "Found a blue cube";
                    break;
                case 5:
                    what_to_speak.data = "Found a blue triangle";
                    break;
                default:
                    break;
            }
            break;
        case 5:
        //It's purple
            switch(shape){
                case 4:
                    what_to_speak.data = "Found a purple cross";
                    break;
                case 6:
                    what_to_speak.data = "Found a purple star";
                    break;
                default:
                    break;
            }
            break;

    }

}

void analyze_objects(object_saving::objects_found objects_found){
    vector<vector<int> > index_conc;
    object_saving::objects_found objects_found_temp = objects_found;

    if(abs(current_robot_vel.angular.z) < 0.1){

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
                        //New object has been identified. It gets updated in the objects vector
                        float distance_to_beginning = sqrt(pow(objects_found_temp.array_objects_found[q].point.x-0.2,2) + pow(objects_found_temp.array_objects_found[q].point.y-0.2,2));
                        map_object temp_object(objects_found_temp.array_objects_found[q],objects_found_temp.array_colors[q],objects_found_temp.array_shape[q],distance_to_beginning);
                        //get_object_to_speak(temp_object.color, objects_found_temp.array_shape[q]);
                        //speak_object_detected = true;
                        objects.push_back(temp_object);
                        number_objects += 1;
                        cout<<"Found a new object"<<endl;

                        //Speak having found a new object
                        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        //espeak_pub.publish();
                    }
                    else{
                        cout<<"Detected but too close to an object"<<endl;
                        //objects[same_index].probability += 10; //We have found the same object, so probability gets higher
                    } 
                }
            }
            for(int q = 0; q < previous_size; q++){
                if(objects_same[q] == 1){

                    if(objects[q].probability < 991) objects[q].probability += 5; //We have found the same object, so probability gets higher
                    else objects[q].probability = 1000;
                    if(objects[q].shape[7] == 100){
                        objects[q].shape[objects_found_temp.array_shape[objects_same_detected[q]]] = 1;
                        objects[q].shape[7] = 0;
                        objects[q].times_detected = 1;
                        objects[q].value = values_objects[objects[q].color][objects_found_temp.array_shape[objects_same_detected[q]]];
                        objects[q].real_priority_value = (objects[q].value / objects[q].distance)*pow(objects[q].times_detected, 0.33);

                        
                    } 
                    else if(objects[q].shape[8] != 100){
                        int best_shape = 0;
                        int best_shape_index = 7;
                        int number_shape_identified = 0;
                        for(int w = 0; w < 9; w++){
                            // if(w != objects_found_temp.array_shape[objects_same_detected[q]]){
                            //     if(objects[q].shape[w] > 0) objects[q].shape[w] -= 1;
                            // }
                            if(w == objects_found_temp.array_shape[objects_same_detected[q]]) objects[q].shape[objects_found_temp.array_shape[objects_same_detected[q]]] += 1;
                            
                            if(objects[q].shape[w] > best_shape){
                                best_shape = objects[q].shape[w];
                                best_shape_index = w;
                            }
                            number_shape_identified += objects[q].shape[w];
                        }
                        //Update value (& real priority value) if shape changes
                        objects[q].value = values_objects[objects[q].color][best_shape_index];
                        objects[q].real_priority_value = (objects[q].value / objects[q].distance)*pow(objects[q].times_detected, 0.33);
                        objects[q].times_detected += 1;
                        if(number_shape_identified >= 5){
                            //Object has been identified 5 times already. The speaker says the object
                            if(!objects[q].object_spoken){
                                get_object_to_speak(objects[q].color, best_shape_index);
                                objects[q].object_spoken = true;
                                speak_object_detected = true;
                            }
                            
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
                    //get_object_to_speak(temp_object.color, objects_found_temp.array_shape[q]);
                    //speak_object_detected = true;
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

void gripped_callBack(const std_msgs::String flag_gripped){
    
    //-----------HERE CODE FOR CHANGING FLAG OF OBJECT PICKED-------------(
    float best_value = 0.0;
    int best_index;
    for(int i = 0; i < objects.size();i++){
        if(!objects[i].picked){
            if(objects[i].real_priority_value > best_value){
                best_value = objects[i].real_priority_value;
                best_index = i;
            }
        }
    }
    if(!std::isnan(best_index)){
        objects[best_index].picked = true;
        geometry_msgs::PointStamped pos_best_temp;
        pos_best_temp = objects[best_index].map_position;
        pos_best_objects_gripped.push_back(pos_best_temp);
    } 

}

// void identification_callBack(const std::string shape_of_object_detected){

//     //-------- EDIT THIS VALUES FOR THE VALUE OF EACH OBJECT IN THE MAZE
//     int value_of_shape = 1;
//     if(shape_of_object_detected == "Red Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Red Hollow Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Blue Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Green Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Yellow Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Yellow Ball") value_of_shape = 10;
//     else if(shape_of_object_detected == "Red Ball") value_of_shape = 10;
//     else if(shape_of_object_detected == "Red Cylinder") value_of_shape = 10;
//     else if(shape_of_object_detected == "Green Cylinder") value_of_shape = 10;
//     else if(shape_of_object_detected == " Green Hollow Cube") value_of_shape = 10;
//     else if(shape_of_object_detected == "Blue Triangle") value_of_shape = 10;
//     else if(shape_of_object_detected == "Purple Cross") value_of_shape = 10;
//     else if(shape_of_object_detected == "Purple Star") value_of_shape = 10;
//     else if(shape_of_object_detected == "Orange Cross") value_of_shape = 10;
//     else if(shape_of_object_detected == "Patric") value_of_shape = 10;

//     //-----------HERE THE CODE FOR CHOOSING WHICH OBJECT IS BEING IDENTIFIED----------
//     ///objects[i].shape = shape_of_object_detected;
//     //objects[i].value = value_of_shape;
// }

void objects_to_file(vector<map_object> objects_to_write)
{
    ofstream file;
    file.open("/home/ras14/catkin_ws/src/round1objects.txt");
    
    string output = "";  
    output += to_string(objects.size()) + "\n";
    for (int i = 0; i < objects_to_write.size(); i++)
    {     
        //output += "shape#";
        for(int j = 0; j < objects_to_write[i].shape.size(); j++){
            if(j != objects_to_write[i].shape.size() - 1) output += std::to_string(objects_to_write[i].shape[j]) + ",";
            else output += std::to_string(objects_to_write[i].shape[j]);
        }
        output += "\n";
        //output += "map_position#";
        output += std::to_string(objects_to_write[i].map_position.point.x) + ",";
        output += std::to_string(objects_to_write[i].map_position.point.y) + ",";
        output += std::to_string(objects_to_write[i].map_position.point.z) + "\n";
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

    string line;
    int numline = 0;
    int num_obj_round1;
    int counter_obj = 0;
    map_object objects_round1;
    ifstream myfile;
    myfile.open("/home/ras14/catkin_ws/src/round1objects.txt");
    if (myfile.is_open()){
        cout<<endl<<"FILE OF OBJECTS DETECTED. WE ARE IN ROUND 2"<<endl<<endl;
        while(getline(myfile,line))
        {  
            int remainder_numline = numline % 7;
            if(numline == 0) {
                num_obj_round1 = stoi(line);
                //cout<<line<<endl;
                //objects.reserve(num_obj_round1);
            }
            else if (remainder_numline == 1){
                //cout<<line<<endl;
                //Array of shape
                string delimiter = ",";
                size_t pos = 0;
                int str_pos = 0;
                std::string token;
                while ((pos = line.find(delimiter)) != std::string::npos) {
                    token = line.substr(0, pos);
                    objects_round1.shape[str_pos] = stoi(token);
                    str_pos ++;
                    line.erase(0, pos + delimiter.length());
                }
            }
            else if (remainder_numline == 2){
                //cout<<line<<endl;
                //Array of position
                objects_round1.map_position.header.frame_id = "/camera_link";
                objects_round1.map_position.header.stamp = ros::Time();

                string delimiter = ",";
                size_t pos = 0;
                int str_pos = 0;
                std::string token;
                while ((pos = line.find(delimiter)) != std::string::npos) {
                    token = line.substr(0, pos);
                    if(str_pos == 0) objects_round1.map_position.point.x = stof(token);
                    else if(str_pos == 1) objects_round1.map_position.point.y = stof(token);
                    else if(str_pos == 2) objects_round1.map_position.point.z = 0.01;
                    str_pos ++;
                    line.erase(0, pos + delimiter.length());
                }
            }
            else if (remainder_numline == 3){
                //cout<<line<<endl;
                objects_round1.color = stoi(line);
            }
            else if (remainder_numline == 4){
                //cout<<line<<endl;
                objects_round1.value = stoi(line);
            }
            else if (remainder_numline == 5){
                //cout<<line<<endl;
                objects_round1.real_priority_value = stof(line);
            }
            else if (remainder_numline == 6){
                //cout<<line<<endl;
                objects_round1.probability = stoi(line);
            }
            else if (remainder_numline == 0){
                //cout<<line<<endl<<endl;
                objects_round1.picked = false;
                speak_objects_round1 = true;
                objects.push_back(objects_round1);
                counter_obj ++;
            }
            numline ++;
        }
    }
    //else cout<<"FILE NOT FOUND. WE ARE IN ROUND 1"<<endl;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "object_saving");
    ros::NodeHandle n;
    ros::Rate r(5);

    for( int i = 0 ; i < ncolors ; i++ ){
        values_objects[i].resize(9);
    }

    ros::Subscriber sub_gripped = n.subscribe("/flag_gripped", 1, gripped_callBack);
    ros::Subscriber sub_new_position = n.subscribe("/object_position_map", 1, position_callBack);
    ros::Subscriber sub_new_position_classical = n.subscribe("/object_position_map_classical", 1, position_classical_callBack);
    ros::Subscriber sub_robot_velocity = n.subscribe("/vel", 1, robot_vel_callBack);
    
    ros::Publisher espeak_pub = n.advertise<std_msgs::String>("/espeak/string", 1);
    ros::Publisher objects_pub = n.advertise<object_saving::objects>("objects_detected", 1);
    ros::Publisher best_object_pub =  n.advertise<geometry_msgs::PointStamped>("/best_object", 1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/all_objects_marker", 1);
    ros::Publisher best_marker_pub = n.advertise<visualization_msgs::Marker>("/best_objects_marker", 1);
    ros::Publisher current_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/current_objects_marker", 1);

    int counter_write_file = 0;
    int counter_read_file = 1;
    bool tried_to_read_file = false;
    bool tried_to_get_values = false;


    while(ros::ok()){

        //Obtain the parameters of the values of the objects
        if(!tried_to_get_values) {
            if(n.hasParam("/value_object_0_0")){
                //If it detects one parameter, all of them should be OK
                string str_param;
                for(int i = 0; i < ncolors; i++){
                    for(int j = 0;j < 9; j++){
                        str_param = "/value_object_" + to_string(i) + "_" + to_string(j);
                        if(n.hasParam(str_param)) n.getParam(str_param, values_objects[i][j]);
                    }
                }
                tried_to_get_values = true;
            }
        }

        if (counter_read_file >= 60){
            std::ifstream ifile;
            ifile.open("/home/ras14/catkin_ws/src/round1objects.txt");
            if ((bool)ifile)
            {
                refresh_objects();
                //cout<<objects.size()<<endl;
            }
            else
            {
                cout<< "FILE OF OBJECTS NOT FOUND. WE ARE IN ROUND 1"<<endl;
            }
            tried_to_read_file = true;
            counter_read_file = 0;
        }
        if(counter_read_file != 0) counter_read_file++;

        //Speak if there were objects from Round 1
        if(speak_objects_round1){
            speak_objects_round1 = false;
            std_msgs::String str_espeak;
            str_espeak.data = "Found objects from round 1";
            espeak_pub.publish(str_espeak);
        }

        if(speak_object_detected){
            espeak_pub.publish(what_to_speak);
            speak_object_detected = false;
        }
        if(tried_to_read_file == true){
            //if((objects.size() != 0) && (number_objects != 0)){
            if(objects.size() != 0){
                
                if(counter_write_file >= 19){
                    objects_to_file(objects);
                    counter_write_file = 0;
                }
                counter_write_file ++;
                
                

                visualization_msgs::MarkerArray markers;
                markers.markers.resize(objects.size());
                visualization_msgs::MarkerArray markers_current;
                object_saving::objects temp_objects;
                temp_objects.number_of_objects = number_objects;
                float best_value = 0.0;
                int best_index;
                for(int i= 0; i< objects.size(); i++){
                    if(objects[i].probability > 50) objects[i].probability -= 1;
                    temp_objects.objects_detected_position.push_back(objects[i].map_position);
                    temp_objects.array_probability.push_back(objects[i].probability);
                    temp_objects.array_real_priority_value.push_back(objects[i].real_priority_value);
                    temp_objects.array_color.push_back(objects[i].color);
                    if(!objects[i].picked){
                        bool okay_dist = true;
                        for(int j = 0; j < pos_best_objects_gripped.size(); j++){
                            if(sqrt(pow(pos_best_objects_gripped[j].point.x - objects[i].map_position.point.x,2) + pow(pos_best_objects_gripped[j].point.y - objects[i].map_position.point.y,2)) < 0.2){
                                okay_dist = false;
                            }
                        }

                        if(okay_dist){
                            if(objects[i].real_priority_value > best_value){
                                best_index = i;
                                best_value = objects[i].real_priority_value;
                            }
                        }
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
                    markers.markers[i].pose.position.z = 0.01;
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

                //Publish best object in RViz
                visualization_msgs::Marker best_marker;
                best_marker.header.frame_id = "/map";
                best_marker.header.stamp = ros::Time::now();

                // Set the namespace and id for this markers[i].  This serves to create a unique ID
                // Any markers[i] sent with the same namespace and id will overwrite the old one
                best_marker.ns = "best_object";
                best_marker.id = objects.size();

                // Set the markers[i] type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                best_marker.type = visualization_msgs::Marker::SPHERE;

                // Set the markers[i] action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                best_marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the markers[i].  This is a full 6DOF pose relative to the frame/time specified in the header
                best_marker.pose.position.x = objects[best_index].map_position.point.x;
                best_marker.pose.position.y = objects[best_index].map_position.point.y;
                best_marker.pose.position.z = 0.01;
                best_marker.pose.orientation.x = 0.0;
                best_marker.pose.orientation.y = 0.0;
                best_marker.pose.orientation.z = 0.0;
                best_marker.pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                best_marker.scale.x = 0.05;
                best_marker.scale.y = 0.05;
                best_marker.scale.z = 0.05;

                // Set the color -- be sure to set alpha to something non-zero!
                best_marker.color.a = 1.0;
                best_marker.color.r = 1.0f;
                best_marker.color.g = 0.78f;
                best_marker.color.b = 0.82f;
                best_marker.lifetime = ros::Duration(0.25);

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
                    markers_current.markers[i].id = i + objects.size() + 1;

                    // Set the markers[i] type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                    markers_current.markers[i].type = visualization_msgs::Marker::CUBE;

                    // Set the markers[i] action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                    markers_current.markers[i].action = visualization_msgs::Marker::ADD;

                    // Set the pose of the markers[i].  This is a full 6DOF pose relative to the frame/time specified in the header
                    markers_current.markers[i].pose.position.x = objects_current[i].map_position.point.x;
                    markers_current.markers[i].pose.position.y = objects_current[i].map_position.point.y;
                    markers_current.markers[i].pose.position.z = 0.01;
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
                best_marker_pub.publish(best_marker);
                objects_pub.publish(temp_objects);
                best_object_pub.publish(objects[best_index].map_position);
            } 
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
