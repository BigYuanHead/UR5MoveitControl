#!/usr/bin/env python

# python standard libraries
import os
import sys
dir_src = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "Universal_Robots_ROS_Driver/ur_robot_driver/scripts")
sys.path.append(dir_src)
dir_src = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.append(dir_src)

# os.environ["OPENBLAS_NUM_THREADS"] = "1"
# os.environ["OMP_NUM_THREADS"] = "1"

# third-party libraries
import numpy as np
import rospy

# self-defined packages
import io_interface
from RobotArm import UR
from Gripper import RobotiqGripperCModelRtu

from util import transform_msg_to_matrix_4x4, matrix_4x4_to_pose_msg
from kuawei_msgs.srv import SetArmPose, SetArmPoseResponse
from kuawei_msgs.srv import SetArmStates, SetArmStatesResponse
from kuawei_msgs.srv import SetGripperState, SetGripperStateResponse
from kuawei_msgs.srv import GetEndEffectorPose, GetEndEffectorPoseResponse
from kuawei_msgs.srv import SetPlacePoseArray, SetPlacePoseArrayResponse
from kuawei_msgs.srv import GetJointState, GetJointStateResponse
from ur_dashboard_msgs.srv import Load, GetProgramState
from std_srvs. srv import Trigger

class KwRobot:

    def __init__(self):

        # rospy.wait_for_service("/ur_hardware_interface/dashboard/load_program", 20)
        # ask_ROSurcap_Load = rospy.ServiceProxy("/ur_hardware_interface/dashboard/load_program", Load)
        # ask_ROSurcap_Play = rospy.ServiceProxy("/ur_hardware_interface/dashboard/play", Trigger)
        # ask_ROSurcap_Stop = rospy.ServiceProxy("/ur_hardware_interface/dashboard/stop", Trigger)

        # Load_resp = ask_ROSurcap_Load("/programs/ROS_drive.urp")
        # if(not Load_resp.success):
        #     rospy.logwarn("Load URcap fail")
       
        # Play_resp = ask_ROSurcap_Play()
        # while(not Play_resp.success):
        #     rospy.logwarn("Play URcap fail, restart")
        #     Stop_resp = ask_ROSurcap_Stop()
        #     rospy.sleep(0.5)
        #     Play_resp = ask_ROSurcap_Play()

        # rospy.set_param("/kuawei_param/ur3_hardware_ready", True)

        self.tool_mode = rospy.get_param("/kuawei_param/robot/tool_model")

        rospy.wait_for_service("/ur_hardware_interface/dashboard/program_state", 20)
        ask_ROSurcap_state = rospy.ServiceProxy("/ur_hardware_interface/dashboard/program_state", GetProgramState)

        state_resp = ask_ROSurcap_state()

        while( not state_resp.state.state == "PLAYING" or not rospy.is_shutdown):
            state_resp = ask_ROSurcap_state()
            rospy.logwarn("start the program")
            rospy.sleep(0.5)

        self.robot_arm = UR()
        if(self.tool_mode == "TwoFinger"):
            self.gripper = RobotiqGripperCModelRtu()   
        elif (self.tool_mode == "Cylinder"):
            self.gripper = io_interface.ur_IOsetup()
        else:
            rospy.logerr("unknow gripper type")

        
        self.set_io = io_interface.ur_IOsetup()
        self.DIGI_PIN_NUM = 0 #use DO0 in CB3

        #self.set_arm_pose_srv = rospy.Service("/kuawei_service/robot/SetArmPose", SetArmPose, self.CartesianSet_MultPose_Matrix)
        self.set_place_pose_srv = rospy.Service("/kuawei_service/robot/SetPlacePose", SetPlacePoseArray ,self.CartesianSet_MultPose_Quaternion)
        self.precompute_place_pose_srv = rospy.Service("/kuawei_service/robot/PreCompute_PickPose", SetPlacePoseArray ,self.PreCompute_Cartesian_MultPose_Quaternion)

        self.set_arm_states_srv = rospy.Service("/kuawei_service/robot/SetArmStates", SetArmStates, self.Set_Armstates)

        self.set_gripper_state_srv = rospy.Service("/kuawei_service/robot/SetGripperState", SetGripperState, self.set_gripper_state)

        self.get_gripper_pose_srv = rospy.Service("/kuawei_service/robot/GetGripperPose", GetEndEffectorPose ,self.get_EndEffector_pose)
        self.get_joint_state_srv = rospy.Service("/kuawei_service/robot/GetJointState", GetJointState, self.get_Current_JointState)

        rospy.set_param("/kw_robot_ready", True)

    def Set_Pose_Matrix(self, req):
        "result is True or False"
        "True means path work, otherwise False"
        
        pose_tool = np.array(req.pose_on_camera_frame).reshape(4,4)

        result = self.robot_arm.move_tool_to_target_pose(matrix_4x4_to_pose_msg(pose_tool))
        return SetArmPoseResponse(result)

    def CartesianSet_MultPose_Matrix(self, req):
        pose_T = np.array(req.pose_on_camera_frame).reshape(4,4)
        pose = matrix_4x4_to_pose_msg(pose_T)
        result = self.robot_arm.CartesianMove_To_TargetPose(pose)
        return result

    def Set_Pose_Quaternion(self, req):
        "overload function, this one use Quaternion as input"

        result = self.robot_arm.move_tool_to_target_pose(req.EndEffector_poses)
        return result

    def CartesianSet_MultPose_Quaternion(self, req):
        
        result = self.robot_arm.CartesianMove_To_TargetPose(req.EndEffector_poses)
        return result

    def PreCompute_Cartesian_MultPose_Quaternion(self, req):
        result = self.robot_arm.Cartesian_PreCompute(req.EndEffector_poses)
        return result

    def Set_Armstates(self, req):
        "result is True or False"
        "True means path work, otherwise False"

        result = self.robot_arm.go_to_target_joint_states(req.target_states)
        return SetArmStatesResponse(result)

    def get_EndEffector_pose(self, req):
        "get EndEffector pose"

        pose = self.robot_arm.get_EndEffector_pose()
        return GetEndEffectorPoseResponse(pose)

    def get_Current_JointState(self, req):
        JointStates_list = self.robot_arm.get_Current_JointState()
        return GetJointStateResponse(JointStates_list)
    
    def set_gripper_state(self, req):
        if(self.tool_mode == "TwoFinger"):
            if req.open:
                self.gripper.jaw_open()
            else:
                self.gripper.jaw_close()
            return SetGripperStateResponse(True)
        elif(self.tool_mode == "Cylinder"):
            if req.open:
                self.set_io.set_digital_out(self.DIGI_PIN_NUM, False)
            else:
                self.set_io.set_digital_out(self.DIGI_PIN_NUM, True)
            return SetGripperStateResponse(True)
        else:
            rospy.logerr("unknow gripper type")
            return SetGripperStateResponse(False)




if __name__ == "__main__":
    
    rospy.init_node("kw_robot")
    kw_robot = KwRobot()
    rospy.spin()