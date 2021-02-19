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


**2.1 Creating a catkin workspace:**


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
```
<?xml version="1.0"?>
<package format="2"> 
  <name>my_gui_pkg</name> 
  <version>0.0.0</version> 
  <description>The my_gui_pkg package</description> 
  <maintainer email="gcpetri@tamu.edu">Gregory Petri</maintainer> 
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend> 
  <build_depend>rqt_gui</build_depend> 
  <build_depend>rqt_gui_py</build_depend> 
  <build_export_depend>rqt_gui</build_export_depend> 
  <build_export_depend>rospy</build_export_depend> 
  <build_export_depend>std_msgs</build_export_depend> 
  <build_export_depend>rqt_gui_py</build_export_depend> 
  <exec_depend>rqt_gui</exec_depend> 
  <exec_depend>rospy</exec_depend> 
  <exec_depend>std_msgs</exec_depend> 
  <exec_depend>rqt_gui_py</exec_depend> 
  <export> 
    <my_gui_pkg plugin="${prefix}/plugin.xml"/> 
  </export> 
</package> 
```

**2.5 plugin.xml file**


add this file to the package directory </br>

>$ touch plugin.xml

>$ nano plugin.xml
```
<library path="src">
  <class name="My Plugin" type="PACKAGE_NAME.my_module.MyPlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
      An example Python GUI plugin to create a great user interface.
    </description>
    <qtgui> 
      <label>My first Python Plugin</label>
      <icon type="theme">system-help</icon>
      <statustip>Great user interface to provide real value.</statustip>
    </qtgui>
  </class>
</library> 
```

**2.6 CMakeList.txt**


>$ nano CMakeList.txt
```
cmake_minimum_required(VERSION 3.0.2)
project(my_gui_pkg)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  rqt_gui
  rqt_gui_py
)
catkin_python_setup()
catkin_package()
install(FILES
   plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY resource
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} 
)
install(PROGRAMS scripts/my_gui_pkg 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)
```

**2.7 setup.py file**


>$ touch setup.py

>$ nano setup.py
```
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['<package_name>'],
    package_dir={'': 'src'}, 
    scripts=['scripts/<package_name>']
)

setup(**d)
```

**2.7 scripts/<executable_name>**
  
>$ chmod 755 <executable>

```
#!/usr/bin/env python

import sys

from my_gui_pkg.my_module import MyPlugin
from rqt_gui.main import Main
</br>
plugin = 'my_gui_pkg'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
```

**2.8 Build Package**

> $ source /opt/ros/kinetic/setup.bash
> $ cd ~/catkin_ws/
> $ catkin_make
> $ roscd <package_name>
> $ catkin_make
>
