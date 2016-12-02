cmake_minimum_required(VERSION 2.8.3)
project(message_sys)


set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  roscpp)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(EIGEN3_FOUND true)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

## Specifies additional locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}	  
  ${CMAKE_INSTALL_PREFIX}/include
)

add_executable(message_sys.cpp
    RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.cpp
    RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.cpp)

#target_link_libraries(message_sys message_sys.cpp)
add_library(message_sys message_sys.cpp)

##drakeCommon
 ##drakeSystemFramework


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

