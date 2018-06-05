#include <sstream>
#include "assemblyline_mov_contrl/assemblyline_mov_contrl.h"
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <tf/tf.h>

#define Pi 3.1415926536

int main(int argc, char **argv){
    ros::init(argc, argv, "assembly_line_center_control");
    ROS_INFO("start assembly_line_center_control");
    ros::NodeHandle node_handle;

    double position_x, position_y, position_z;
    std::string moving;
    std::map<std::string, double> aim_position;

    tf::Quaternion q;
    double orientation_x, orientation_y, orientation_z, orientation_w;
    q = tf::createQuaternionFromRPY(0, 0, Pi);  // 采用弧度值,以Pi为基准
    aim_position["orientation_x"] = q.getX();
    aim_position["orientation_y"] = q.getY();
    aim_position["orientation_z"] = q.getZ();
    aim_position["orientation_w"] = q.getW();

    aim_position["position_x"] = -0.158;
    aim_position["position_y"] = 0.00;
    aim_position["position_z"] = 0.198;


    node_handle.setParam("/arm/aim_position", aim_position);
    node_handle.setParam("/arm/moving", "moving");

    ROS_INFO("x: %f, y: %f, z: %f, w: %f", aim_position["orientation_x"], aim_position["orientation_y"], aim_position["orientation_z"], aim_position["orientation_w"]);
    return 0;
}
