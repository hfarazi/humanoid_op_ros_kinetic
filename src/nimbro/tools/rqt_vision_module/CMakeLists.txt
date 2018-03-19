cmake_minimum_required(VERSION 2.8.3)
project(rqt_vision_module)

find_package(catkin REQUIRED COMPONENTS std_msgs rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge message_generation)

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

catkin_python_setup()
   
add_message_files(DIRECTORY msg FILES
	GuiEvent.msg
	ConsoleMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge message_runtime
)

if(USE_QT5)
    qt5_wrap_cpp(MOCFILES include/${PROJECT_NAME}/main.hpp include/${PROJECT_NAME}/layout.hpp)
    qt5_wrap_ui(UIFILES src/${PROJECT_NAME}/main.ui)
else()
    qt4_wrap_cpp(MOCFILES include/${PROJECT_NAME}/main.hpp include/${PROJECT_NAME}/layout.hpp)
    qt4_wrap_ui(UIFILES src/${PROJECT_NAME}/main.ui)
endif()

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/main.cpp
   src/${PROJECT_NAME}/layout.cpp
   ${MOCFILES}
   ${UIFILES}
)
include_directories(include 
   ${catkin_INCLUDE_DIRS} 
   ${CMAKE_CURRENT_BINARY_DIR} 
   ${CMAKE_CURRENT_SOURCE_DIR}
   )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
   
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/${PROJECT_NAME}
   DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


