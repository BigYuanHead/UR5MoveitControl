#include "ur3_moveit_usage.h"
/**
 * param:
 * ros::NodeHandle& nh
 * std_msgs::String& PLANNING_GROUP_
 * std_msgs::String& BASE_FRAME_ID_
 * @@ Use ros::param::get("/kuawei_param/robot/*****", ---)
**/

ur3_moveit_usage::ur3_moveit_usage(const ros::NodeHandle &nh,
                                   const std::string &PLANNING_GROUP,
                                   const std::string &BASE_FRAME_ID)
    : nh_(nh), move_group_(PLANNING_GROUP), visual_tools_(BASE_FRAME_ID)
{
  ros::param::get("/kuawei_param/robot/tool_frame_id", tool_frame_id_);
  ros::param::get("/kuawei_param/robot/tcp_frame_id", tcp_frame_id_);
  ros::param::get("/kuawei_param/robot/max_v_factor", max_v_factor_);
  ros::param::get("/kuawei_param/robot/max_acc_factor", max_acc_factor_);
  base_frame_id_ = BASE_FRAME_ID;

  move_group_.setGoalTolerance(0.0005);
  move_group_.setPlanningTime(5);
  move_group_.setEndEffectorLink(tool_frame_id_);
  move_group_.setMaxVelocityScalingFactor(max_v_factor_);
  move_group_.setMaxAccelerationScalingFactor(max_acc_factor_);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  ROS_INFO_NAMED("kw_ur3moveit", "Reference frame: %s", move_group_.getPlanningFrame().c_str());
  ROS_INFO_NAMED("kw_ur3moveit", "End effector link: %s", move_group_.getEndEffectorLink().c_str());

  //ros::ServiceServer service = nh_.advertiseService("set_pose", &ur3_moveit_usage::set_Armpose_srvCB, this);
  //ROS_INFO_NAMED("kw_ur3moveit", "ur3_MoveServer online");
}

bool ur3_moveit_usage::set_Armpose_srvCB(kuawei_msgs::SetArmPose::Request &req,
                                         kuawei_msgs::SetArmPose::Response &res)
{
  // Planning to a Pose goal, matrix_4x4_to_pose_msg
  // ^^^^^^^^^^^^^^^^^^^^^^^

  /** T_matrix
     * 
        |xx yx zx px|
        |xy yy zy py|
        |xz yz zz pz|
        |0  0  0  1 |
    **/

  Eigen::Matrix4d T_matrix;
  T_matrix.Zero(4, 4);
  double pose_16[16];

  for (size_t i = 1; i <= 4; i++)
    for (size_t j = 1; j <= 4; j++)
      //pose_16[i - 1, j - 1] = req.pose_on_camera_frame[(i * j) - 1];
      T_matrix(i - 1, j - 1) = double(req.pose_on_camera_frame[(i * j) - 1]);

  //Eigen::Matrix4d T_matrix = req.pose_on_camera_frame.c_array;
  Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>> rotation_matrix(T_matrix.data(), 3, 3, Eigen::OuterStride<>(4));
  Eigen::Quaterniond quaternion(rotation_matrix);

  move_group_.setPoseReferenceFrame(base_frame_id_);

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = quaternion.w();
  target_pose1.orientation.x = quaternion.x();
  target_pose1.orientation.y = quaternion.y();
  target_pose1.orientation.z = quaternion.z();
  target_pose1.position.x = req.pose_on_camera_frame[3];
  target_pose1.position.y = req.pose_on_camera_frame[7];
  target_pose1.position.z = req.pose_on_camera_frame[11];
  move_group_.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(my_plan);

  ROS_INFO("plan_result is :%d", plan_result);
  res.result = true;
  // if (plan_result)
  // {
  //   // Visualizing plans
  //   // ^^^^^^^^^^^^^^^^^
  //   visual_tools_.publishAxisLabeled(target_pose1, "pick_pose");
  //   visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
  //   visual_tools_.trigger();
  //   //Execute
  //   move_group_.execute(my_plan);
  //   res.success = true;
  // }
  // else
  // {
  //   ROS_ERROR("Mission FAILED");
  //   res.success = false;
  // }

  return true;
}

bool ur3_moveit_usage::goto_JointStateCB(kuawei_msgs::SetArmStates::Request &req,
                                         kuawei_msgs::SetArmStates::Response &res)
{
  std::vector<double> vector_armstate6(6);

  for (size_t i = 0; i < 6; i++)
    vector_armstate6.push_back(double(req.target_states[i]));

  move_group_.clearPoseTargets();
  move_group_.setJointValueTarget(vector_armstate6);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode plan_result = move_group_.plan(my_plan);

  ROS_INFO("plan_result is :%d", plan_result);

  move_group_.execute(my_plan);
  res.result = true;
  return true;
}

/**
{
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

}
**/
