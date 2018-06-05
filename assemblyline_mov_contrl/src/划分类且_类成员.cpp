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
}


void ArmMove::arm_moving()
{
    namespace rvt = rviz_visual_tools;
    visual_tools_.deleteAllMarkers();
    visual_tools_.publishText(text_pose_, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools_.trigger();
//    double i = 0;
//    for(i=0; i<0.3 ;)
//    {
//     位姿设定
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w=1;
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.05;
    move_group_.setPlanningTime(100);
    move_group_.setPoseTarget(target_pose);
//    i += 0.05;
//    ROS_INFO("i is %f", i);

    move_group_.setPlannerId("RRTConnectkConfigDefault");
//     产生规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    move_group.execute(my_plan);
    move_group_.move();  // put move() above visual_tool.pulish*,不然直接导致Gazebo运动失败
    std::string name;
    name = "mygod";

    visual_tools_.publishAxisLabeled(target_pose,name);
    visual_tools_.publishText(text_pose_,"Arm Grasp",rvt::WHITE, rvt::XLARGE); // 字体颜色及大小
    visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);

    visual_tools_.trigger(); // Batch publishing is used to reduce the number of messages
                            // being sent to RViz for large visualizations
//    ros::Duration(1.0).sleep();
//    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo");
//  创建一个异步的自旋线程（spinning thread）

  ROS_INFO("Start move_group_interface_demo");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ArmMove arm_move("arm");
  int k = 1;
  while(ros::ok()){
      if(k>0){
          arm_move.arm_moving();
          k =0;
      }
  }
  ros::waitForShutdown();
  return 0;
}


