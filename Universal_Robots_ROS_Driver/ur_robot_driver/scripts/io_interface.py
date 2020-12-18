#!/usr/bin/env python
import sys
import rospy
import ur_msgs.srv
import ur_msgs.msg



class ur_IOsetup():
    
    def __init__(self):
        self.set_io = None
        self.Digital_Out_States = [0,0,0,0,0,0,0,0,0,0]  #8(controller)+2(tool)
        self.Digital_In_States = [0,0,0,0,0,0,0,0,0,0]   #8(controller)+2(tool)
        self.Analog_Out_States = [0,0]  #2(controller)
        self.Analog_In_States = [0,0]   #2(controller)+0(tool)
        self.ANALOG_TOLERANCE_VALUE = 0.01

        self.FUN_SET_DIGITAL_OUT = 1
        self.FUN_SET_FLAG = 2
        self.FUN_SET_ANALOG_OUT = 3
        self.FUN_SET_TOOL_VOLTAGE = 4

        
        rospy.Subscriber("/ur_hardware_interface/io_states", ur_msgs.msg.IOStates, self.callback)

        rospy.wait_for_service('/ur_hardware_interface/set_io')
        self.set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO)


    def set_io_val(self, fun, pin, val):
        try:
            self.set_io(fun, pin, val)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

    def set_tool_voltage(self, volts):
        try:
            self.set_io(self.FUN_SET_TOOL_VOLTAGE, volts, 0)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

    def set_digital_out(self, pin, val):
        "@param"
        "1.pin: 1, 2, ...., 10 "
        "2.val: False = low; True = High "

        try:
            self.set_io(self.FUN_SET_DIGITAL_OUT, pin, val)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e

    def set_analog_out(self, pin, val):
        try:
            self.set_io(self.FUN_SET_ANALOG_OUT, pin, val)
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
    

    def callback(self, data):
        for i in range(0,10):
            del self.Digital_Out_States[i]
            self.Digital_Out_States.insert(i, data.digital_out_states[i].state)
        for i in range(0,10):
            del self.Digital_In_States[i]
            self.Digital_In_States.insert(i, data.digital_in_states[i].state)
        for i in range(0,2):
            del self.Analog_Out_States[i]
            self.Analog_Out_States.insert(i, data.analog_out_states[i].state)
        #ToolInput analog_in[2] & analog_in[3] currently not supported
        for i in range(0,2):
            del self.Analog_In_States[i]
            self.Analog_In_States.insert(i, data.analog_in_states[i].state)


    



