# SAE_autodrive
Code for the 12th Unmanned Team at Texas A&amp;M University
</br>

# Problem #

</br>

> Part 1: 
> Set up ROS kinetic on a Ubuntu 16.04 virtualbox environment
> 
> Part 2: 
> Create a user interface for can_bus - the Tamu software for automated vehicle - use rqt plugins for ROS
> clone the repository @ https://github.tamu.edu/autodrive-common/can_bus
> 
> Objectives for GUI:
> * can_bus commands (startus, indicators to check its status)
> * two modes: manual and automatic
> * tracking error state: give an indicator something's gone wrong
> 
</br>

# Solution #

</br>

**Part 1:**

</br>
Start an ubuntu server with virtualbox using Ubuntu 16.04.7 desktop image. </br>
Go to Settings->Software & Updates and check main, resricted, universe, and multiverse are checked. </br>
Follow the instructions @ http://wiki.ros.org/Installation/Ubuntu </br>

If problems keep occuring such as Error 400, run </br>
>$ sudo apt-get update --fix-missing
followed by, </br>
>$ sudo apt-get install ros-kinetic-desktop-full
keep running until installed 100% </br>
</br>

**Part 2:**

</br>

**2.1 Creating a catkin workspace:**

</br>

>$ source /opt/ros/kinetic/setup.bash
>
>$ mkdir -p ~/catkin_ws/src
>
>$ cd ~/catkin_ws/
>
>$ catkin_make
>
>$ source devel/setup.bash
>
>$ echo $ROS_PACKAGE_PATH
>

**2.2 Creating a catkin Package**

>$ cd ~/catkin_ws/src </br>
>
>$ ### catkin_create_pkg <package_name> [depend1] [depend2] [depend3] </br>
>
>$ cd ~/catkin_ws </br>
>
>$ catkin_make </br>
>
>$ . ~/catkin_ws/devel/setup.bash


**2.3 Verify working by Checking Package Dependancies**


>$ rospack depends1 <package_name>


**2.4 package.xml file**


>$ roscd <package_name>
>$ nano package.xml

<?xml version="1.0"?> </br>
<package format="2"> </br>
  <name>my_gui_pkg</name> </br>
  <version>0.0.0</version> </br>
  <description>The my_gui_pkg package</description> </br>
  <maintainer email="gcpetri@tamu.edu">Gregory Petri</maintainer> </br>
  <license>BSD</license> </br>
  <buildtool_depend>catkin</buildtool_depend> </br>
  <build_depend>rospy</build_depend> </br>
  <build_depend>std_msgs</build_depend> </br>
  <build_depend>rqt_gui</build_depend> </br>
  <build_depend>rqt_gui_py</build_depend> </br>
  <build_export_depend>rqt_gui</build_export_depend> </br>
  <build_export_depend>rospy</build_export_depend> </br>
  <build_export_depend>std_msgs</build_export_depend> </br>
  <build_export_depend>rqt_gui_py</build_export_depend> </br>
  <exec_depend>rqt_gui</exec_depend> </br>
  <exec_depend>rospy</exec_depend> </br>
  <exec_depend>std_msgs</exec_depend> </br>
  <exec_depend>rqt_gui_py</exec_depend> </br>
  <export> </br>
    <my_gui_pkg plugin="${prefix}/plugin.xml"/> </br>
  </export> </br>
</package> </br>
'''

**2.5 plugin.xml file**


add this file to the package directory </br>

>$ touch plugin.xml
>
>$ nano plugin.xml

<library path="src"> </br>
  <class name="My Plugin" type="PACKAGE_NAME.my_module.MyPlugin" base_class_type="rqt_gui_py::Plugin"> </br>
    <description> </br>
      An example Python GUI plugin to create a great user interface. </br>
    </description> </br>
    <qtgui> </br>
      <label>My first Python Plugin</label> </br>
      <icon type="theme">system-help</icon> </br>
      <statustip>Great user interface to provide real value.</statustip> </br>
    </qtgui> </br>
  </class> </br>
</library> </br>


**2.6 CMakeList.txt**


>$ nano CMakeList.txt

cmake_minimum_required(VERSION 3.0.2) </br>
project(my_gui_pkg) </br>
find_package(catkin REQUIRED COMPONENTS </br>
  rospy </br>
  std_msgs </br>
  rqt_gui </br>
  rqt_gui_py </br>
) </br>
catkin_python_setup() </br>
catkin_package() </br>
install(FILES </br>
   plugin.xml </br>
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} </br>
) </br>
install(DIRECTORY resource </br>
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} </br>
) </br>
install(PROGRAMS scripts/my_gui_pkg </br>
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} </br>
) </br>


**2.7 setup.py file**


>$ touch setup.py
>
>$ nano setup.py

#!/usr/bin/env python </br>
</br>
from distutils.core import setup </br>
from catkin_pkg.python_setup import generate_distutils_setup </br>
</br>
d = generate_distutils_setup( </br>
    packages=['<package_name>'], </br>
    package_dir={'': 'src'}, </br>
    scripts=['scripts/<package_name>'] </br>
) </br>
</br>
setup(**d) </br>
</br>

**2.7 scripts/<executable_name>**
  
</br>
>$ chmod 755 <executable> </br>
#!/usr/bin/env python </br>
</br>
import sys </br>
</br>
from my_gui_pkg.my_module import MyPlugin </br>
from rqt_gui.main import Main </br>
</br>
plugin = 'my_gui_pkg' </br>
main = Main(filename=plugin) </br>
sys.exit(main.main(standalone=plugin))


**2.8 Build Package**


>$ source /opt/ros/kinetic/setup.bash
>
>$ cd ~/catkin_ws/
>
>$ catkin_make
>
>$ roscd <package_name>
>
>$ catkin_make
