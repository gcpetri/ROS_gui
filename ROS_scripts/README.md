# ROS_scripts
Scripts for the 12th Unmanned Team at Texas A&amp;M University
</br>

# Objective #

</br>

> Part 1: </br>
> Set up ROS kinetic on a Ubuntu 16.04 virtualbox environment </br>
> 
> Part 2: </br>
> Create a user interface for can_bus - the Tamu software for automated vehicle - use rqt plugins for ROS </br>
>

</br>

# Solution #

> Follow the helper instructions below -OR- </br>
> Run the scripts I created in the ROS_scripts directory </br>
> They will save hours in setup time... </br>
> (be sure to edit the global variables at the top of the files to customize your workspace and package) </br>

# Scripts #

> 1.  ros_install.sh  --  install ROS in your Linux Environment </br>
> 2.  ros_create_catkin_workspace.sh  --  create and configure a catkin workspace, required to use ROS </br>
> 3.  ros_create_catkin_package.sh  --  create a simple custom rqt package in your workspace </br>

</br>

**Requirements**

Start an ubuntu server with virtualbox using Ubuntu 16.04.7 desktop image. </br>
Go to Settings->Software & Updates and check main, resricted, universe, and multiverse are checked. </br>
Follow the instructions @ http://wiki.ros.org/Installation/Ubuntu </br>

__below is OPTIONAL__
<hr></hr>

**1. Setting up ROS**

If problems keep occuring such as Error 400, run </br>
> `$ sudo apt-get update --fix-missing`
followed by, </br>
> `$ sudo apt-get install ros-kinetic-desktop-full` </br>
keep running until installed 100% </br>
</br>

**2. Catkin Workspace**


> `$ source /opt/ros/kinetic/setup.bash`
> `$ mkdir -p ~/catkin_ws/src`
> `$ cd ~/catkin_ws/`
> `$ catkin_make`
> `$ source devel/setup.bash`
> `$ echo $ROS_PACKAGE_PATH`


**3.0 Creating a Catkin Package**

> `$ cd ~/catkin_ws/src </br>`
> `$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]`
> `$ cd ~/catkin_ws`
> `$ catkin_make`
> `$ . ~/catkin_ws/devel/setup.bash`


**3.1 Verify working by Checking Package Dependancies**


> `$ rospack depends1 <package_name>`


**3.2 package.xml file**

edit this file in package directory </br>
> `$ roscd <PACKAGE_NAME>`
> `$ nano package.xml`

```
<?xml version="1.0"?>
<package format="2"> 
  <name>PACKAGE_NAME</name> 
  <version>0.0.0</version> 
  <description>DESCRIPTION</description> 
  <maintainer email="---YOUR_EMAIL---">---YOUR_NAME---</maintainer> 
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
    <rqt_gui plugin="${prefix}/plugin.xml"/> 
  </export> 
</package> 
```

**3.3 plugin.xml file**


add this file to the package directory </br>

> `$ touch plugin.xml`
> `$ nano plugin.xml`

```
<library path="src">
  <class name="MyPlugin" type="---PACKAGE_NAME---.my_module.MyPlugin" base_class_type="rqt_gui_py::Plugin">
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

**3.4 CMakeList.txt**

edit file in package directory </br>
> `$ nano CMakeList.txt`

```
cmake_minimum_required(VERSION 3.0.2)
project(<PACKAGE_NAME>)
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

**3.5 setup.py file**

create file in package directory </br>
> `$ touch setup.py`
> `$ nano setup.py`

```
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['<PACKAGE_NAME>'],
    package_dir={'': 'src'}, 
    scripts=['scripts/<PACKAGE_NAME>']
)

setup(**d)
```

**3.6 scripts/<PACKAGE_NAME>**
  
give executable permissions </br>
> `$ chmod 755 <PACKAGE_NAME>`

```
#!/usr/bin/env python

import sys

from <PACKAGE_NAME>.my_module import MyPlugin
from rqt_gui.main import Main

plugin = '<PACKAGE_NAME>'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
```

**3.7 Build Package**

> `$ source /opt/ros/kinetic/setup.bash`
> `$ cd ~/catkin_ws/`
> `$ source devel/setup.bash`</br>
> `$ source src/can_bus/include/Virtual_CAN_Setup.sh`
> `$ catkin_make`
> `$ roscd <package_name>`

**3.8 Run Package**

> `$ rosrun <PACKAGE_NAME> <package_name>`
or </br>
> `$ rqt --standalone <PACKAGE_NAME>`

Notes: </br>

> `echo $PYTHONPATH`

/opt/ros/kinetic/lib/python2.7/dist-packages
