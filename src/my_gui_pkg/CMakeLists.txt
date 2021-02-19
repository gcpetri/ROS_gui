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

install(PROGRAMS scripts/my_gui_pkg
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY resource
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
