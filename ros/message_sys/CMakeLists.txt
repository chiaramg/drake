cmake_minimum_required(VERSION 2.8.3)
project(message_sys)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  drake_ros_common
  drake_ros_systems)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(EIGEN3_FOUND true)

#CHIARA EDITED 04.12
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS
#)

catkin_package()


## Specifies additional locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}	  
  ${CMAKE_INSTALL_PREFIX}/include
)

link_directories("${CMAKE_INSTALL_PREFIX}/lib")

add_executable(message_sys
    message_sys.cpp
    msgs_chiara.cpp
    RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.cpp
    RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.cpp)

#target_link_libraries(message_sys message_sys.cpp)

target_link_libraries(message_sys
  ${catkin_LIBRARIES}
  drakeCommon
  drakeSystemFramework
  drakeSystemAnalysis
)

#add_library(message_sys message_sys.cpp)

##drakeCommon
 ##drakeSystemFramework


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

