# source the setup script
source /opt/ros/kinetic/setup.bash
echo "----------- sourced setup script ----------"
# make a directory if one does not already exist
mkdir -p ~/catkin_ws/src
echo "----------- made catkin_ws/src directories --------"
# move to that directory
cd ~/catkin_ws
echo "----------- moved to the new directory -----------"
# create and compile the workspace
catkin_make
echo "----------- compiled the workspace -------------"
# source the development folder setup file
source devel/setup.bash
echo "---------- sourced the workspace dev file -------"
# check that the make was successful
echo $ROS_PACKAGE_PATH
# move back to user's last directory
cd -
echo "DONE."
