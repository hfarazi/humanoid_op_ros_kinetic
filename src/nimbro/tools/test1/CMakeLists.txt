cmake_minimum_required(VERSION 2.8.3)
project(test1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui_py
)
catkin_python_setup()
catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES test1
	#  CATKIN_DEPENDS rospy rqt_gui_py
	#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS scripts/test1
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
