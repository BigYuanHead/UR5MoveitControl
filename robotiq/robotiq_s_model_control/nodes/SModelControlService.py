#!/usr/bin/env python


# ROS service for controlling the open and close of the RobotiQ Gripper

import roslib
import rospy
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.srv import GripperCommand
from time import sleep

pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)


def genCommand(req):
    """Update the command according to the character entered by the user."""

    command = outputMsg.SModel_robot_output();
    # print command

    if req.comd == 'a':
        # command = outputMsg.SModel_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if req.comd == 'r':
        # command = outputMsg.SModel_robot_output();
        command.rACT = 0

    if req.comd == 'c':
        # command = outputMsg.SModel_robot_output();
        # command.rACT = 1
        # command.rGTO = 1
        # command.rSPA = 255
        # command.rFRA = 150
        command.rPRA = 255

    if req.comd == 'o':
        # command = outputMsg.SModel_robot_output();
        # command.rACT = 1
        # command.rGTO = 1
        # command.rFRA = 150
        command.rPRA = 0

    if req.comd == 'b':
        # command = outputMsg.SModel_robot_output();
        command.rMOD = 0

    if req.comd == 'p':
        # command = outputMsg.SModel_robot_output();
        command.rMOD = 1

    if req.comd == 'w':
        # command = outputMsg.SModel_robot_output();
        command.rMOD = 2

    if req.comd == 's':
        # command = outputMsg.SModel_robot_output();
        command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
        # command = outputMsg.SModel_robot_output();
        command.rPRA = int(req.comd)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if req.comd == 'f':
        # command = outputMsg.SModel_robot_output();
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if req.comd == 'l':
        # command = outputMsg.SModel_robot_output();
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

    if req.comd == 'i':
        # command = outputMsg.SModel_robot_output();
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if req.comd == 'd':
        # command = outputMsg.SModel_robot_output();
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    # pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)
    pub.publish(command)
    rospy.sleep(0.1)



def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = 'Simple S-Model Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = ' + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
    ##    currentCommand += ', rGLV = ' + str(command.rGLV)
    ##    currentCommand += ', rICF = ' + str(command.rICF)
    ##    currentCommand += ', rICS = ' + str(command.rICS)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)

    # We only show the simple control mode
    ##    currentCommand += ', rPRB = ' + str(command.rPRB)
    ##    currentCommand += ', rSPB = ' + str(command.rSPB)
    ##    currentCommand += ', rFRB = ' + str(command.rFRB)
    ##    currentCommand += ', rPRC = ' + str(command.rPRC)
    ##    currentCommand += ', rSPC = ' + str(command.rSPC)
    ##    currentCommand += ', rFRC = ' + str(command.rFRC)
    ##    currentCommand += ', rPRS = ' + str(command.rPRS)
    ##    currentCommand += ', rSPS = ' + str(command.rSPS)
    ##    currentCommand += ', rFRS = ' + str(command.rFRS)

    print currentCommand

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return raw_input(strAskForCommand)


def publisher():
    """Main loop which requests new commands and publish them on the SModelRobotOutput topic."""

    rospy.init_node('SModelSimpleController')

    # pub = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)

    # command = outputMsg.SModel_robot_output();

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)

        pub.publish(command)

        rospy.sleep(0.1)


def main():
    rospy.init_node('SModelControlService')
    # command = outputMsg.SModel_robot_output();
    # print "currnet command is %s" % command
    rospy.Service('gripper/move', GripperCommand, genCommand)
    rospy.spin()


if __name__ == '__main__':
    # publisher()
    main()
