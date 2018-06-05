#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/String.h>
#include <sstream>
#include "assemblyline_mov_contrl/assemblyline_mov_contrl.h"

using namespace assemblyline_mov_contrl;

// 实例化构造函数
ArmMove::ArmMove(std::string plan_name = "arm"):
    node_handle_("arm_move"),move_group_(plan_name),visual_tools_("world_frame")
{

    joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(plan_name);
    text_pose_ = Eigen::Affine3d::Identity();  // 总题位置
    text_pose_.translation().z() = 0.5; // above head
    sub_text_pose_ = Eigen::Affine3d::Identity(); // 副标位置
    sub_text_pose_.translation().z() = 0.35;
}

void ArmMove::arm_marking_initi_(std::string text_name = "Demo"){
    namespace rvt = rviz_visual_tools;
    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose_, text_name, rvt::WHITE, rvt::XLARGE, false);
    visual_tools_.trigger();
}

// 空参数
void ArmMove::arm_moving()
{
    namespace rvt = rviz_visual_tools;
    visual_tools_.publishText(sub_text_pose_,"Arm Grasp",rvt::WHITE, rvt::XLARGE, false); // 字体颜色及大小

    std::string aim_reach;

    geometry_msgs::Pose target_pose;
    target_pose.orientation.w=1;
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.00;
    target_pose.position.z = 0.15;
    move_group_.setPlanningTime(100);
    move_group_.setPoseTarget(target_pose);
    ROS_INFO("arm positon for now: x= %f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group_.setPlannerId("RRTkConfigDefault");
    // 产生规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success_plan = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // move_group.execute(my_plan);
    move_group_.move();  // put move() above visual_tool.pulish*,不然直接导致Gazebo运动失败
    // 判断是否已经到达了目标位置
    bool success = (move_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success){
        aim_reach = "reach";
        node_handle_.setParam("/arm/reach", true);
        node_handle_.setParam("/arm/moving", "reach");
    }
    else{
        aim_reach = "reach";
        node_handle_.setParam("/arm/reach", false);
    }

    std::string name;
    name = "pose";

    visual_tools_.publishAxisLabeled(target_pose,name);
    visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
    visual_tools_.trigger(); // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    ros::Duration(1.0).sleep();  //暂停1s
}


// 填充了aim_position的map对象参数
void ArmMove::arm_moving(std::map<std::string, double> aim_position, std::string task="Nothing"){

    namespace rvt = rviz_visual_tools;
    visual_tools_.publishText(sub_text_pose_,task,rvt::WHITE, rvt::XLARGE, true); // 字体颜色及大小

    geometry_msgs::Pose target_pose;

    target_pose.orientation.x = aim_position["orientation_x"];
    target_pose.orientation.y = aim_position["orientation_y"];
    target_pose.orientation.z = aim_position["orientation_z"];
    target_pose.orientation.w = aim_position["orientation_w"];
    target_pose.position.x = aim_position["position_x"];
    target_pose.position.y = aim_position["position_y"];
    target_pose.position.z = aim_position["position_z"];

    move_group_.setPlanningTime(1);
//    move_group_.setStartState(*move_group_.getCurrentState());
    move_group_.setPoseTarget(target_pose);
    ROS_INFO("arm positon for now: x= %f, y=%f, z=%f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
    ROS_INFO("arm orientation for now: x= %f, y=%f, z=%f, w=%f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
    move_group_.setPlannerId("RRTConnectkConfigDefault");
//     产生规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success_plan = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success_plan){
    std::string name;
    name = "pose";
    visual_tools_.publishAxisLabeled(target_pose,name);
    visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
    visual_tools_.trigger();} // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations

    move_group_.move();
    bool success_move = (move_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //    判断是否已经到达了目标位置
    if(success_move){
        node_handle_.setParam("/arm/reach", true);
        node_handle_.setParam("/arm/moving",task);
        }
        else{
        node_handle_.setParam("/arm/reach", false);
        node_handle_.setParam("/arm/moving", "Failed");
        }
    ros::Duration(1.0).sleep();  //暂停1s
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo");
//  创建一个异步的自旋线程（spinning thread）
  ROS_INFO("Start move_group_interface_demo");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ArmMove arm_move("arm");

  std::string moving;  // 决定move_group是否发布运动轨迹
  std::string task;  // 当前运动任务
  std::map<std::string,double> aim_position;  // map容器类,包含了运动目标的位置及四元数信息
  arm_move.arm_marking_initi_("MoveGroupInterface Demo");  // 初始化rviz的MarkerArray

  while(ros::ok()){
      if(arm_move.node_handle_.getParam("/arm/moving",moving)&&(moving=="moving")){
          if(arm_move.node_handle_.getParam("/arm/aim_position",aim_position) && arm_move.node_handle_.getParam("/arm/task",task)){
             arm_move.arm_moving(aim_position, task);}
      }}

  ros::waitForShutdown();
  return 0;

}

