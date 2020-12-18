# python standard libraries
import pickle
import time as sys_time
import threading
import os, sys

# third-party libraries
import roslib
roslib.load_manifest('robotiq_c_model_control')
roslib.load_manifest('robotiq_modbus_rtu')

dir_src = os.path.join(os.path.dirname(os.path.dirname(__file__)), "robotiq")
dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(dir_src)

from robotiq_c_model_control.baseCModel import robotiqBaseCModel
from robotiq_modbus_rtu import comModbusRtu
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg


class RobotiqGripperCModelRtu:
    full_mouth_size_in_meter = 0.085

    def __init__(self, device="/dev/ttyUSB0"):
        print("gripper init...")

        GripperConfigFile_dir = os.path.join(os.path.dirname(__file__), "data/robotiq_gripper_open_close_rPR.pkl")
        istream = open(GripperConfigFile_dir, 'r')
        open_close_rPR_dict = pickle.load(istream)
        istream.close()
        
        self.close_rPR = open_close_rPR_dict["close_rPR"]
        # self.close_rPR = 255 The reason 255 fails because the gripper cannot reach this value
        self.open_rPR = open_close_rPR_dict["open_rPR"]
        print("file loading complete")
        # self.open_rPR = 0 The same reason as 255
        self.gripper = robotiqBaseCModel()
        print("robotiqBaseCModel")

        self.gripper.client = comModbusRtu.communication()
        self.gripper.client.connectToDevice(device)
        print("connectToDevice")
        self.__init_gripper()
        print("__init_gripper")
        self.test_way_points_thread = threading.Thread(target=self.__inspect_gripper_position)
        print("gripper is ready")

    def jaw_open(self, is_wait=True):
        '''
            Open parallel jaw as wide as possible
            Args:
                is_wait (boolen): Wait until the gripper finish execution.
        '''
        self.__move_A(rPR=self.open_rPR, is_wait=is_wait)

    def jaw_close(self, is_wait=True):
        '''
            close parallel jaw (called to grip object)
            Args:
                is_wait (boolen): Wait until the gripper finish execution.
        '''
        self.__move_A(rPR=self.close_rPR, is_wait=is_wait)

    def open_jaw_in_meter(self, target_mouth_size_in_meter):
        if RobotiqGripperCModelRtu.full_mouth_size_in_meter >= target_mouth_size_in_meter >= 0:
            target_rPR = int(round((self.open_rPR - self.close_rPR) * target_mouth_size_in_meter / RobotiqGripperCModelRtu.full_mouth_size_in_meter + self.close_rPR))
            self.__move_A(target_rPR)

    def __inspect_gripper_position(self):
        while True:
            sys_time.sleep(0.01)
            status = self.gripper.getStatus()
            print(status.gPO)   
    
    def __init_gripper(self):
        command = outputMsg.CModel_robot_output(rACT=1, rSP=255, rFR=150)
        while True:
            self.gripper.refreshCommand(command)
            self.gripper.sendCommand()
            sys_time.sleep(0.01)
            status = self.gripper.getStatus()
            # if status.gIMC == 3:
            if status.gSTA == 3:
                break
            sys_time.sleep(1)

    def __move_A(self, rPR, is_wait=True):
        command = outputMsg.CModel_robot_output(rACT=1, rGTO=1, rPR=rPR, rSP=128, rFR=50)
        self.gripper.refreshCommand(command)
        self.gripper.sendCommand()
        pre_gPO = None
        while is_wait:
            sys_time.sleep(0.4)
            status = self.gripper.getStatus()
            print(status.gPR, status.gPO)
            if status.gPO == pre_gPO and pre_gPO is not None:
                break
            else:
                pre_gPO = status.gPO

            if status.gPO == status.gPR == rPR:
                break
    
    def __del__(self):
        self.gripper.client.disconnectFromDevice()
