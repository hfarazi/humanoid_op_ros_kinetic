cmake_minimum_required(VERSION 2.8.3)
project(control_widget)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp config_server nimbro_op_interface rqt_gui rqt_gui_cpp message_generation robotcontrol gait_msgs)

# Decide on a Qt version
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
    set(USE_QT5 true)
else()
    set(USE_QT5 false)
endif()

# Find Qt
if(USE_QT5)
    find_package(Qt5 COMPONENTS Core Gui REQUIRED)
    find_package(Qt5Widgets REQUIRED)
    set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
else()
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
endif() 

include_directories(${CMAKE_CURRENT_BINARY_DIR})

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES walk_control
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

if(USE_QT5)
    qt5_wrap_ui(UIFILES include/control_widget/controlwidget.ui)
    qt5_wrap_cpp(MOCFILES include/control_widget/controlwidget.h include/control_widget/controlbutton.h)
else()
    qt4_wrap_ui(UIFILES include/control_widget/controlwidget.ui)
    qt4_wrap_cpp(MOCFILES include/control_widget/controlwidget.h include/control_widget/controlbutton.h)
endif()

## Declare a cpp library
add_library(control_widget
  ${UIFILES}
  ${MOCFILES}
  src/controlwidget.cpp
  src/controlbutton.cpp
  src/service.cpp
)

## Add dependencies to the executable
add_dependencies(control_widget gait_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(control_widget
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS control_widget
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES control_widget_plugin.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
