#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/String.h>
#include <sstream>

namespace assemblyline_mov_contrl
{
class ArmMove
{
private:

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;  // moveit!可视化接口
    Eigen::Affine3d text_pose_;  // 主标题
    Eigen::Affine3d sub_text_pose_;  // 副标题
    const robot_state::JointModelGroup *joint_model_group_;
public:
    ros::NodeHandle node_handle_;
    ArmMove(std::string plan_name);  // 以plan_name作为规划组规划对象
    void arm_moving(std::map<std::string, double> aim_position, std::string moving_name);  // 移动到目标位置
    void arm_moving(void);  // 移动到目标位置
    void arm_marking_initi_(std::string text_name);
};

class GripMove
{
private:

    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;  // moveit!可视化接口
    Eigen::Affine3d text_pose_;  // 夹持器状态显示
    const robot_state::JointModelGroup *joint_model_group_;
public:
    ros::NodeHandle node_handle_;
    GripMove(std::string plan_name);  // 以plan_name作为规划组规划对象
    void grip_moving(std::map<std::string, double> grip_position, std::string moving_name);  // 移动到目标位置
    void grip_moving(void);  // 移动到目标位置
    void grip_marking_initi_(std::string text_name);
};
}
