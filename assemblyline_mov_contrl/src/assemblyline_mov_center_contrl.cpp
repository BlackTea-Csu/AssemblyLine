#include <sstream>
#include "assemblyline_mov_contrl/assemblyline_mov_contrl.h"
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <tf/tf.h>

#define Pi 3.1415926536

void set_aim_orientation(std::map<std::string, double> *aim_position, tf::Quaternion q){
    double orientation_x, orientation_y, orientation_z, orientation_w;
    orientation_x = q.getX();
    orientation_y = q.getY();
    orientation_z = q.getZ();
    orientation_w = q.getW();
    aim_position->insert(std::map < std::string, double >::value_type("orientation_x", orientation_x));
    aim_position->insert(std::map < std::string, double >::value_type("orientation_y", orientation_y));
    aim_position->insert(std::map < std::string, double >::value_type("orientation_z", orientation_z));
    aim_position->insert(std::map < std::string, double >::value_type("orientation_w", orientation_w));
}

void set_aim_position(std::map<std::string, double> *aim_position, double *position){
    double position_x, position_y, position_z;
    position_x = position[0];
    position_y = position[1];
    position_z = position[2];
    aim_position->insert(std::map < std::string, double >::value_type("position_x", position_x));
    aim_position->insert(std::map < std::string, double >::value_type("position_y", position_y));
    aim_position->insert(std::map < std::string, double >::value_type("position_z", position_z));
}

void moving_to_object(ros::NodeHandle node_handle){
    std::map<std::string, double> aim_position;  // 目标坐标系位置及旋转
    tf::Quaternion orientation;  // 目标坐标系旋转角_四元数
    double position[3];  // 目标坐标系位置

    orientation = tf::createQuaternionFromRPY(0, 0, 0);  // 采用弧度值,以Pi为基准
    position[0] = 0.158;
    position[1] = 0.00;
    position[2] = 0.100;

    set_aim_orientation(&aim_position,orientation);
    set_aim_position(&aim_position, position);

    node_handle.setParam("/arm/aim_position", aim_position);
    node_handle.setParam("/arm/moving", "moving");
    node_handle.setParam("/arm/task", "Move to target");

    ROS_INFO("orientation: x: %f, y: %f, z: %f, w: %f", aim_position["orientation_x"], aim_position["orientation_y"], aim_position["orientation_z"], aim_position["orientation_w"]);
    ROS_INFO("Position: x: %f, y: %f, z: %f", aim_position["position_x"], aim_position["position_y"], aim_position["position_z"]);
}

void moving_to_space(ros::NodeHandle node_handle){
    std::map<std::string, double> aim_position;  // 目标坐标系位置及旋转
    tf::Quaternion orientation;  // 目标坐标系旋转角_四元数
    double position[3];  // 目标坐标系位置

    orientation = tf::createQuaternionFromRPY(0, 0, 1.57);  // 采用弧度值,以Pi为基准
    position[0] = 0.012;
    position[1] = 0.186;
    position[2] = 0.15;

    set_aim_orientation(&aim_position,orientation);
    set_aim_position(&aim_position, position);

    node_handle.setParam("/arm/aim_position", aim_position);
    node_handle.setParam("/arm/moving", "moving");
    node_handle.setParam("/arm/task", "Move to space");

    ROS_INFO("orientation: x: %f, y: %f, z: %f, w: %f", aim_position["orientation_x"], aim_position["orientation_y"], aim_position["orientation_z"], aim_position["orientation_w"]);
    ROS_INFO("Position: x: %f, y: %f, z: %f", aim_position["position_x"], aim_position["position_y"], aim_position["position_z"]);
}


void placing_object(ros::NodeHandle node_handle){
    std::map<std::string, double> aim_position;  // 目标坐标系位置及旋转
    tf::Quaternion orientation;  // 目标坐标系旋转角_四元数
    double position[3];  // 目标坐标系位置

    orientation = tf::createQuaternionFromRPY(0, 0, 1.57);  // 采用弧度值,以Pi为基准
    position[0] = 0.012;
    position[1] = 0.186;
    position[2] = 0.05;

    set_aim_orientation(&aim_position,orientation);
    set_aim_position(&aim_position, position);

    node_handle.setParam("/arm/aim_position", aim_position);
    node_handle.setParam("/arm/moving", "moving");
    node_handle.setParam("/arm/task", "Place object");

    ROS_INFO("orientation: x: %f, y: %f, z: %f, w: %f", aim_position["orientation_x"], aim_position["orientation_y"], aim_position["orientation_z"], aim_position["orientation_w"]);
    ROS_INFO("Position: x: %f, y: %f, z: %f", aim_position["position_x"], aim_position["position_y"], aim_position["position_z"]);
}

void init_station(ros::NodeHandle node_handle){
    std::map<std::string, double> aim_position;  // 目标坐标系位置及旋转
    tf::Quaternion orientation;  // 目标坐标系旋转角_四元数
    double position[3];  // 目标坐标系位置

    orientation = tf::createQuaternionFromRPY(0, 0, 0);  // 采用弧度值,以Pi为基准
    position[0] = 0.158;
    position[1] = 0.00;
    position[2] = 0.198;

    set_aim_orientation(&aim_position,orientation);
    set_aim_position(&aim_position, position);

    node_handle.setParam("/arm/aim_position", aim_position);
    node_handle.setParam("/arm/moving", "moving");
    node_handle.setParam("/arm/task", "initialization");

    ROS_INFO("orientation: x: %f, y: %f, z: %f, w: %f", aim_position["orientation_x"], aim_position["orientation_y"], aim_position["orientation_z"], aim_position["orientation_w"]);
    ROS_INFO("Position: x: %f, y: %f, z: %f", aim_position["position_x"], aim_position["position_y"], aim_position["position_z"]);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "assembly_line_center_control");
    ROS_INFO("start assembly_line_center_control");
    ros::NodeHandle node_handle;
    std::string moving, task;  // 暂存rosparam变量
    bool reach;  // 暂存rosparam变量,判断arm是否到达了目标

    int k = 1;

    ros::Publisher gripper_state = node_handle.advertise<std_msgs::String>("/robotis/open_manipulator/gripper", 100);

    node_handle.setParam("/arm/moving", "stop");
    node_handle.setParam("/arm/reach", "no");
    node_handle.setParam("/arm/task", "no");

    while(ros::ok()){
        if(k>0){
            moving_to_object(node_handle);
            k = 0;}
        if(node_handle.getParam("/arm/moving",moving) && moving == "Move to target")
            if(node_handle.getParam("/arm/reach", reach) && reach){
                std_msgs::String gripper_command;
                std::stringstream ss;
                ss << "grip_on";
                gripper_command.data=ss.str();
//                gripper_state.publish(gripper_command);
                ROS_INFO("Now gripper is on");
                ros::Duration(2.0).sleep();
                moving_to_space(node_handle);}
        if(node_handle.getParam("/arm/moving",moving) && moving == "Move to space")
            if(node_handle.getParam("/arm/reach", reach) && reach)
                placing_object(node_handle);
        if(node_handle.getParam("/arm/moving",moving) && moving == "Place object")
            if(node_handle.getParam("/arm/reach",reach) && reach){
                init_station(node_handle);
                ros::Duration(1.0).sleep();
//                gripper_state.publish("grip_off");
            }


    }





    return 0;
}
