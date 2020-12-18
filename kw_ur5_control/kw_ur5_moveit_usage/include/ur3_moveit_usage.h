//ros libary
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//third party
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

//msgs & srvs
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "kuawei_msgs/SetArmPose.h"
#include "kuawei_msgs/SetArmStates.h"

class ur3_moveit_usage
{
public:
    ur3_moveit_usage(const ros::NodeHandle &nh,
                     const std::string &PLANNING_GROUP,
                     const std::string &BASE_FRAME_ID);

    bool set_Armpose_srvCB(kuawei_msgs::SetArmPoseRequest &req,
                           kuawei_msgs::SetArmPoseResponse &res);

    bool goto_JointStateCB(kuawei_msgs::SetArmStates::Request &req,
                           kuawei_msgs::SetArmStates::Response &res);

private:
    ros::NodeHandle nh_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;

    const robot_state::JointModelGroup *joint_model_group_;
    std::string base_frame_id_, tool_frame_id_, tcp_frame_id_;
    double max_v_factor_, max_acc_factor_;
};