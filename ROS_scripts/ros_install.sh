# run this script to download ros-kinetic on Ubuntu 16.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
echo "---------- downloaded the packages --------------"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
echo "---------- got the keys -------------------------"
sudo apt-get update
echo "---------- updated -----------------------------"
var="$(sudo apt-get -y install ros-kinetic-desktop-full | tail -n -1)"
echo "---------- downloaded ros-kinetic-desktop-full --------------"
echo "var is"
echo $var
# possibly won't fetch every package so use a loop
while [[ $var == "E:"* ]]
do
   sudo apt-get update --fix-missing
   echo "---------- fixed missing packages -------------------"
   var="$(sudo apt-get -y install ros-kinetic-desktop-full | tail -n -1)"
   echo "---------- downloaded ros-kinetic-desktop-full --------------"
done

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "------------- sourced ros -----------------"
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
echo "------------- installed important packages ----------"
sudo apt install python-rosdep
echo "------------- install rosdep -------------------"
sudo rosdep init
echo "------------- initialized rosdep ---------------"
rosdep update
echo "------------- updated rosdep -------------------"
echo "DONE."
