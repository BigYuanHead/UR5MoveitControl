
# 1. git clone
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src   
git clone http://192.168.0.26/zhangjc/kw_robot.git

# 2. (install dependances) and then catkin_make
catkin_make -DCMAKE_BUILD_TYPE=Release  

# 3. initialize gripper:
sudo chmod 777 /dev/ttyUSB0  
cd ~/catkin_ws  
source devel/setup.bash  
roslaunch robotiq_85_bringup robotiq_85.launch  

# 4. check paramters in kw_robot_usage/config/config.yaml  

# 5. start the moveit control  
cd ~/catkin_ws/  
source devel/setup.bash  
roslaunch kw_robot_usage kw_robot_real_robot.launch  

# 6. Handeye calibration
roslaunch "your_camera"  
rosed kw_robot_usage hand_eye_calibration.launch  # modify "image_topic" and camera_info_topic  
roslaunch kw_robot_usage hand_eye_calibration.launch # more than 15 groups of robot pose and image data  
