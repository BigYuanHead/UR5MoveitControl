from __future__ import print_function

# third-party libraries
from trajectory_msgs.msg import*
from control_msgs.msg import *
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ExecuteTrajectoryAction
from moveit_msgs.msg import MoveGroupAction
from actionlib_msgs.msg import GoalStatusArray, GoalID


import moveit_commander
import moveit_msgs
import moveit_msgs.msg
import actionlib
import numpy as np
import rospy
import copy

class UR:

    def __init__(self):
        
        print("robot_arm init...", end='')

        self.base_frame_id  = rospy.get_param("/kuawei_param/robot/base_frame_id")
        self.place_frame_id = rospy.get_param("/kuawei_param/robot/place_frame_id")
        self.tool_frame_id  = rospy.get_param("/kuawei_param/robot/tool_frame_id")
        self.tcp_frame_id   = rospy.get_param("/kuawei_param/robot/tcp_frame_id")
        move_group_name     = rospy.get_param("/kuawei_param/robot/move_group_name")
        max_v_factor        = rospy.get_param("/kuawei_param/robot/max_v_factor")
        max_acc_factor      = rospy.get_param("/kuawei_param/robot/max_acc_factor")

        # self.init_joint_states = rospy.get_param("/RobotArm/init_joint_states")
        # self.place_joint_states = rospy.get_param("/RobotArm/place_joint_states")

        rospy.loginfo("waiting for moveit")
        Moveit_act =  actionlib.ActionClient("/move_group", MoveGroupAction)
        Moveit_act.wait_for_action_server_to_start( rospy.Duration(20) )
        ExeTraj_act =  actionlib.ActionClient("/execute_trajectory", ExecuteTrajectoryAction)
        ExeTraj_act.wait_for_action_server_to_start( rospy.Duration(20) )
        rospy.loginfo("moveit service online")

        self.ExeTraj_status = GoalStatusArray
        self.Status_ID = "blank_ID"
        ExeTraj_status_sub = rospy.Subscriber("/execute_trajectory" + '/status', GoalStatusArray, callback=self._ExeTraj_status_CB, queue_size=10)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(move_group_name)

        self.group.set_goal_tolerance(0.01)
        self.group.set_planning_time(5)
        self.group.set_end_effector_link(self.tcp_frame_id)
        self.group.set_max_velocity_scaling_factor(max_v_factor)
        self.group.set_max_acceleration_scaling_factor(max_acc_factor)
        self.group.set_pose_reference_frame(self.base_frame_id)

        self.display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path',
                                                      moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        print("============ Reference frame: %s" % self.group.get_planning_frame())
        print("============ pose reference frame is:{}".format(self.group.get_pose_reference_frame()))
        print("============ end effector frame: %s" % self.group.get_end_effector_link())
        print("============ Robot Groups:")
        print(self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("============")

        print("robot_arm is ready")

    # def go_to_init_joint_states(self):
    #     self.goz_to_target_joint_states(self.init_joint_states)

    # def go_to_place_joint_states(self, idx):
    #     self.go_to_target_joint_states(self.place_joint_states[idx % len(self.place_joint_states)])
    
    def _ExeTraj_status_CB(self, msg):
        self.ExeTraj_status = copy.deepcopy(msg)


    def get_EndEffector_pose(self):
        "return Pose from geometry_msgs.msg "
        Pose = self.group.get_current_pose(end_effector_link=self.tcp_frame_id)
        Pose = Pose.pose
        return Pose

    def get_Current_JointState(self):
        "a list of current joint values"
        return self.group.get_current_joint_values()

    def go_to_target_joint_states(self, joint_state_position_array_6x1):
        #self.group.clear_pose_targets()
        self.group.set_joint_value_target(joint_state_position_array_6x1)
        rospy.loginfo("Planning trajectory according to target joint state position...")
        plan = self.group.plan()

        trajectory_wrist3 = []
        for point in plan.joint_trajectory.points:
            trajectory_wrist3.append(point.positions[-1])

        #rospy.logwarn(trajectory_wrist3)
        if(len(trajectory_wrist3) == 0):
            rospy.logwarn("No plan found!")
            return False
        
        rospy.loginfo("Plan found.")
        rospy.loginfo("Executing plan...")
        if self.group.execute(plan, wait=True):
            rospy.loginfo("Execute Complete.")
            return True
        else:
            rospy.logwarn("Execute fail.")
            return False

    def move_tool_to_target_pose(self, tool_target_pose, is_wait=True):
        
        """ tool_target_pose should be a Pose message, a PoseStamped message or a list of 6 floats:"""
        """ [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats [x, y, z, qx, qy, qz, qw] """

        # TODO: More Roboust Path Planning
        self.group.set_pose_reference_frame(self.base_frame_id)
        self.group.set_pose_target(tool_target_pose)
        plan = self.group.plan()

        trajectory_wrist3 = []
        for point in plan.joint_trajectory.points:
            trajectory_wrist3.append(point.positions[-1])

        rospy.logwarn(trajectory_wrist3)
        if(len(trajectory_wrist3) == 0):
            rospy.logwarn("No plan found!")
            return False

        if(max(trajectory_wrist3) - min(trajectory_wrist3) > np.pi / 2.0):
            rospy.logwarn("\n wrist3_angle too large. re-planning \n")
            
            # re-planning
            self.group.clear_pose_targets()
            joint_target = list(plan.joint_trajectory.points[-1].positions)
            if joint_target[-1] < 0: 
                joint_target[-1] += np.pi
            else: 
                joint_target[-1] -= np.pi


            self.group.set_joint_value_target(joint_target)
            plan = self.group.plan()
            trajectory_wrist3 = []
            for point in plan.joint_trajectory.points:
                trajectory_wrist3.append(point.positions[5])

        if(len(plan.joint_trajectory.points) == 0):
            rospy.logwarn("No plan found!")
            return False

        if(len(plan.joint_trajectory.points) > 28):
            rospy.logwarn("long trajectory! refuse to execute")
            rospy.logwarn(trajectory_wrist3)
            return False
        
        rospy.loginfo("Plan found.")
        rospy.loginfo("Executing plan...")
        if self.group.execute(plan, wait=True):
            rospy.loginfo("Execute Complete.")
            return True
        else:
            rospy.logwarn("Execute fail.")
            return False

    def CartesianMove_To_TargetPose(self, tool_target_poses,reference_frame="base", is_wait=True):
        #tool_target_poses: Array 
        #reference_frame: base or place_points    
        
        if reference_frame== "place_points":
            self.group.set_pose_reference_frame(self.place_frame_id)
        else:
            self.group.set_pose_reference_frame(self.base_frame_id)
        
        waypoints = []
        for pose in tool_target_poses.poses:
            waypoints.append(pose)

        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.05,        # eef_step
                                        0.0)         # jump_threshold
        # if ( not self.Check_MoveitActionStatus(self.Moveit_status) == 3):
        #     print("\033[1;37;41m no plan find")
        #     return False
        # self.Check_MoveitActionStatus(self.Moveit_status)

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_pub.publish(display_trajectory);  

        print("\033[0;37;44m Cartesian plan fraction is:", fraction)
        if(fraction > 0.6):
            result = self.group.execute(plan, wait=True)
            print(">"*40)
            print(result)
            print("\033[0;37;44m status is:", self.ExeTraj_status.status_list[0].text)
            print(">"*40)
            return result
        else:
            return False

    def Cartesian_PreCompute(self, tool_target_poses,reference_frame="base", is_wait=True):
        #tool_target_poses: Array 
        #reference_frame: base or place_points    
        
        if reference_frame== "place_points":
            self.group.set_pose_reference_frame(self.place_frame_id)
        else:
            self.group.set_pose_reference_frame(self.base_frame_id)
        
        waypoints = []
        for pose in tool_target_poses.poses:
            waypoints.append(pose)

        _, fraction = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.05,        # eef_step
                                        0.0)         # jump_threshold
        # if ( not self.Check_MoveitActionStatus(self.Moveit_status) == 3):
        #     print("\033[1;37;41m no plan find")
        #     return False
        # self.Check_MoveitActionStatus(self.Moveit_status)

        self.group.clear_pose_targets()

        print("="*20)
        print("precompute Cartesian plan fraction is:", fraction)
        print("="*20)
        
        if(fraction < 0.6):
            return False
        else:
            return True

    def Check_ExeTrajActionStatus(self):
        """return action status code 
        return 3 is success """
        while( len(self.ExeTraj_status.status_list) == 0 and not rospy.is_shutdown() ):
            continue
        while( self.Status_ID == self.ExeTraj_status.status_list[0].goal_id.id and
                not rospy.is_shutdown() ):
            continue
        print("\033[0;37;44m status is:", self.ExeTraj_status.status_list[0].text)
        return self.ExeTraj_status.status_list[0].status
