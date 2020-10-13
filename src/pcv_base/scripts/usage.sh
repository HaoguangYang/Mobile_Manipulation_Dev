cd ~/Dev/Mobile_Manipulation_Dev		# ROS workspace
source ./devel/setup.sh

roslaunch pcv_base pcv_node.launch		# starts robot base + odometry only
# in a new terminal window
cd ~/Dev/Mobile_Manipulation_Dev		# ROS workspace
source ./devel/setup.sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py	# keyboard teleop script to move the robot

roslaunch pcv_base build_map.launch		# SLAM mode, with keyboard teleop
# when done, save map in a new terminal
rosrun map_server map_saver -f ~/Dev/Mobile_Manipulation_Dev/src/pcv_base/resources/map/IMI_corner

roslaunch pcv_base pcv_node.launch		# starts robot base + odometry only
# in a new terminal window
roslaunch wall_follower gbaseUI.launch	# path following mode, by specifying waypoints in rviz

# utilities
rosnode list
rostopic list
rostopic echo /mobile_base_controller/cmd_vel	# this is the velocity command topic
