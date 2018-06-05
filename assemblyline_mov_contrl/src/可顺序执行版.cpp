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




int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo");
//   创建一个异步的自旋线程（spinning thread）
  ros::NodeHandle node_handle;
  ROS_INFO("tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("tutorial3");
  static const std::string PLANNING_GROUP = "arm";
  // 连接move_group节点中的机械臂实例组
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 使用常指针指向规划组
  const robot_state::JointModelGroup *joint_model_group =
          move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;

  moveit_visual_tools::MoveItVisualTools visual_tools("world_frame");

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 0.5; // above head
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  double i;

  for(i=0; i<0.3 ;)
  {


  // 位姿设定
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w=1;
  target_pose.position.x = 0.1;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.05+i;
  move_group.setPlanningTime(100);
  move_group.setPoseTarget(target_pose);

  i += 0.05;
  std::cout<<i;
  ROS_INFO("i is %f", i);
//  visual_tools.deleteAllMarkers();

  // 关节移动
  moveit::core::RobotStateConstPtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group,joint_group_positions );


  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(10);



  // 产生规划
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan);
  move_group.move();  // put move() above visual_tool.pulish*,不然直接导致Gazebo运动失败

  visual_tools.publishAxisLabeled(target_pose,"pose1");
  visual_tools.publishText(text_pose,"Arm Grasp",rvt::WHITE, rvt::XLARGE); // 字体颜色及大小
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

  visual_tools.trigger(); // Batch publishing is used to reduce the number of messages
                          // being sent to RViz for large visualizations
  ros::Duration(1.0).sleep();
  }

//  visual_tools.prompt("next step"); //若在此句后,ros::param::set 无效


//    ros::NodeHandle gripper;
//    ros::Publisher gripper_close = gripper.advertise<std_msgs::String>("robotis/open_manipulator/gripper",100);
//    std_msgs::String close;
//    close.data="grip_off";
//    int k = 2;
//    ros::Rate loop_rate(1);
//    while (ros::ok())
//    {
//     if(k>0)
//     {
//     gripper_close.publish(close);
//     ROS_INFO("%s", close.data.c_str());
//     k -= 1;
//     }
//     ros::spinOnce();
//     loop_rate.sleep();
//    }




//  ros::waitForShutdown();
  return 0;
}

