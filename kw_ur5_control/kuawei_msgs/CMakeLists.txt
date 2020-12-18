cmake_minimum_required(VERSION 2.8.3)
project(kuawei_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)


## Generate messages in the 'msg' folder
# add_message_files(
# )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SetArmPose.srv
   SetArmStates.srv
   SetGripperState.srv
   GetEndEffectorPose.srv
   SetPlacePoseArray.srv
   GetJointState.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_msgs
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############
