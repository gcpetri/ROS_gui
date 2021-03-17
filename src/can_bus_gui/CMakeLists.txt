cmake_minimum_required(VERSION 3.0.2)
project(can_bus_gui)
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
#install(DIRECTORY resource
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install(PROGRAMS scripts/can_bus_gui
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
