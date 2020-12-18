# python standard libraries
import sys

# third-party libraries
import moveit_commander
import rospy
import yaml

# self-defined packages
from KwRobot import KwRobot


def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kw_robot', anonymous=True)
    
    kw_robot = KwRobot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
