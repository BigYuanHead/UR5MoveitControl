#include <ros/ros.h>
#include "ur3_moveit_usage.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_usage_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string base_frame_id, planning_group;
    ros::param::get("/kuawei_param/robot/base_frame_id", base_frame_id);
    ros::param::get("/kuawei_param/robot/move_group_name", planning_group);

    ur3_moveit_usage moveit_usage(node_handle, planning_group, base_frame_id);

    ros::ServiceServer pose_service = node_handle.advertiseService("set_pose", &ur3_moveit_usage::set_Armpose_srvCB, &moveit_usage);
    ros::ServiceServer state_service = node_handle.advertiseService("set_state", &ur3_moveit_usage::goto_JointStateCB, &moveit_usage);
    ROS_INFO_NAMED("kw_ur3moveit", "ur3_MoveServer online");

    ros::waitForShutdown();
}