# global variables
# user variables
your_name="Gregory Petri"
your_email="gcpetri@tamu.edu"
# ros package variables
name="can_bus_gui"
ros_depends1="std_msgs"
ros_depends2="rqt_gui"
ros_depends3="rqt_gui_py"
ros_depends4="rospy"
plugin_name="StartPlugin"
module_filename="start_module"
ui_filename="canbusmain"

# move to the source folder
cd ~/catkin_ws/src
echo "-------- moved to the source folder ---------"
# run the create package command
catkin_create_pkg $name $ros_depends1 $ros_depends2 $ros_depends3 $ros_depends4
echo "-------- created the packages internally specified depends ---------"
# move to the workspace folder
cd ~/catkin_ws
echo "-------- moved to the workspace folder ---------"
# compile the workspace
catkin_make
echo "-------- compiled the workpace ---------"
. ~/catkin_ws/devel/setup.bash
# check functionality by the depends
rospack depends1 $name
roscd $name
echo "-------- moved to the package directory --------"
# make package.xml correct
> package.xml
echo "-------- emptied the package.xml file ---------"
cat >> package.xml <<EOL
<?xml version="1.0"?>
<package format="2">
  <name>$name</name>
  <version>0.0.0</version>
  <description>DESCRIPTION</description>
  <maintainer email="$your_email">$your_name</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>$ros_depends1</build_depend>
  <build_depend>$ros_depends2</build_depend>
  <build_depend>$ros_depends3</build_depend>
  <build_depend>$ros_depends4</build_depend>
  <build_export_depend>$ros_depends1</build_export_depend>
  <build_export_depend>$ros_depends2</build_export_depend>
  <build_export_depend>$ros_depends3</build_export_depend>
  <build_export_depend>$ros_depends4</build_export_depend>
  <exec_depend>$ros_depends1</exec_depend>
  <exec_depend>$ros_depends2</exec_depend>
  <exec_depend>$ros_depends3</exec_depend>
  <exec_depend>$ros_depends4</exec_depend>
  <export>
    <rqt_gui plugin="\${prefix}/plugin.xml"/>
  </export>
</package>
EOL
echo "------- wrote correct contents to package.xml file ----------"
# make plugin.xml file
touch plugin.xml
echo "------- create plugin.xml file -----------"
cat >> plugin.xml <<EOL
<library path="src">
  <class name="$plugin_name" type="$name.$module_filename.$plugin_name" base_class_type="rqt_gui_py::Plugin">
    <description>
      An example Python GUI plugin to create a great user interface.
    </description>
    <qtgui>
      <label> </label>
      <icon type="theme">system-run</icon>
      <statustip>Great user interface to provide real value.</statustip>
    </qtgui>
  </class>
</library>
EOL
echo "------ wrote correct contents to plugin.xml file ---------"
# edit the CMakeList.txt file
> CMakeLists.txt
echo "------- emptied the CMakeLists.txt file -------"
cat >> CMakeLists.txt <<EOL
cmake_minimum_required(VERSION 3.0.2)
project($name)
find_package(catkin REQUIRED COMPONENTS
  $ros_depends1
  $ros_depends2
  $ros_depends3
  $ros_depends4
)
catkin_python_setup()
catkin_package()
install(FILES
   plugin.xml
   DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY resource
   DESTINATION \${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/$name
   DESTINATION \${CATKIN_PACKAGE_BIN_DESTINATION}
)
EOL
echo "------- wrote correct contents to CMakeLists.txt -------"
# create the setup.py file
touch setup.py
echo "------- created the setup.py file --------"
cat >> setup.py <<EOL
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['$name'],
    package_dir={'': 'src'},
    scripts=['scripts/$name']
)
setup(**d)
EOL
echo "-------- wrote correct contents to setup.py ---------"
# add a scripts folder
mkdir -p scripts
echo "------- made the scripts folder -------"
cd scripts
echo "------- moved to the scripts folder ------"
touch $name
chmod 755 $name
echo "------- created the $name file --------"
cat >> $name <<EOL
#!/usr/bin/env python

import sys

from $name.$module_filename import $plugin_name
from rqt_gui.main import Main

plugin = '$name'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
EOL
echo "-------- wrote the correct contents to $name ---------"
cd ../
echo "-------- moved to the package folder ---------"
# make package source and package folders
mkdir -p src/$name
cd src/$name
echo "-------- made and move to the src/$name folder ---------"
# create an empty file
touch __init__.py
echo "------- created empty __init__.py file ---------"
# create a module file
touch $module_filename.py
echo "------- created $module_filename.py file ---------"
cat >> $module_filename.py <<EOL
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot
from python_qt_binding.QtWidgets import QWidget

class $plugin_name(Plugin):

    def __init__(self, context):
        super($plugin_name, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('$plugin_name')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
	#self._widget.setGeometry(700,200,600,500)
        #self._widget.setStyleSheet("background: rgb(250,250,250);")
	uifile = os.path.join(rospkg.RosPack().get_path('$name'), 'resource', '$ui_filename.ui')
	loadUi(uifile, self._widget)
	self._widget.setObjectName('$ui_filename')

        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

	# Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

   # def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        # pass

    #def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        # pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
EOL
echo "--------- wrote correct contents to the $module_filename.py file ------------"
cd ../../
echo "--------- moved back to the package folder ----------"
# make a resource folder
mkdir -p resource
echo "--------- created a resource folder ----------"
cd resource
echo "--------- moved into the resource folder --------"
touch $ui_filename.ui
echo "--------- create the $ui_filename.ui file ---------"
cat >> $ui_filename.ui <<EOL
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>$ui_filename</class>
 <widget class="QWidget" name="$ui_filename">
 </widget>
 <resources/>
 <connections/>
</ui>
EOL
echo "------ wrote a simple QWidget to the $ui_filename.ui file --------"

# compile the package
cd ~/catkin_ws
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
catkin_make
roscd $name
read -p "did the package compile: [y]? " bool_compile
if [ $bool_compile != "y" ]
then
   echo "----------- ending script -----------"
   read -p "do you want to delete the pacakge: [y]?" bool_delete
   if [ $bool_delete == "y" ]
   then
       rm -r ~/catkin_ws/src/$name
       echo "----------- cleaned package $name --------"
   fi
   exit 1;
fi
echo "---------- package creation compiled ------------"
echo "Done."
