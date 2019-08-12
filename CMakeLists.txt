cmake_minimum_required(VERSION 2.8.3)
project(traj_gen)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# If Debug mode, CGAL precision goes down


set(CMAKE_BUILD_TYPE Debug)## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  qt_build
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

set(qpOASES_SRC /home/jbs/lib/qpOASES-3.2.1)

file(GLOB_RECURSE qpOASES_LIBS ${qpOASES_SRC}/src/*.cpp)

##############################################################################
# Qt Environment
##############################################################################
set(CMAKE_AUTOUIC ON) 
# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in manifest.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
set(QT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/qt_ui)
set(QT_FORMS ${QT_DIR}/mainwindow.ui) # by adding this, the catkin build make ui_mainWindow.h
file(GLOB QT_MOC   ${QT_DIR}/*.h)
file(GLOB QT_SOURCES ${QT_DIR}/*.cpp)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
message("qt form: ${QT_DIR}")
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message("qt form path: ${CMAKE_CURRENT_BINARY_DIR}")


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PolyCoeff.msg
   PolySpline.msg
   PolySplineXYZ.msg
 )



## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   traj_gen
   nav_msgs
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES traj_gen
  CATKIN_DEPENDS message_generation nav_msgs roscpp
    
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/PolyTrajGen.cpp 
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_client qt_ui/main.cpp src/PolyTrajGen.cpp ${qpOASES_LIBS} ${QT_FORMS_CPP} ${QT_SOURCES} ${QT_MOC_HPP} ${qpOASES_LIBS})

 target_link_libraries(${PROJECT_NAME}_client
 ${catkin_LIBRARIES} ${QT_LIBRARIES})


## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_server
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


