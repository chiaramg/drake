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
  ${CMAKE_INSTALL_PREFIX}/include/lcmtypes
)

link_directories("${CMAKE_INSTALL_PREFIX}/lib")

add_executable(message_sys_demo
        message_sys_demo.cc
        msgs_chiara.cc
        RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Publisher.cc
        RPG_Drake_ROS_Bridge/RPG_Quad_ROS_Receiver.cc)

#target_link_libraries(message_sys message_sys.cpp)

target_link_libraries(message_sys_demo
  ${catkin_LIBRARIES}
  drakeCommon
  drakeSystemFramework
  drakeSystemAnalysis
  drakeSystemPrimitives)


add_executable(quadrotor_lqr_demo
        quadrotor_lqr_demo.cc
        quadrotor_plant.cc
        lqr_demo_publisher.cc)
target_link_libraries(quadrotor_lqr_demo
        ${catkin_LIBRARIES}
        drakeCommon
        drakeSystemFramework
        drakeSystemAnalysis
        drakeLinearControl
        drakeRigidBodyPlant
        drakeRBM
        #drakeLCMTypes
        drake_lcmtypes
        lcm
        drakeLCMSystem2
        drakeLcm
        drakeSystemPrimitives)


add_executable(RPG_Drake_ROS_demo
        RPG_Drake_ROS_demo.cpp
        quadrotor_plant.cc
        lqr_demo_publisher.cc
        lqr_demo_subscriber.cpp
        )
target_link_libraries(RPG_Drake_ROS_demo
        ${catkin_LIBRARIES}
        drakeCommon
        drakeSystemFramework
        drakeSystemAnalysis
        drakeLinearControl
        drakeRigidBodyPlant
        drakeRBM
        #drakeLCMTypes
        drake_lcmtypes
        lcm
        drakeLCMSystem2
        drakeLcm
        drakeSystemPrimitives)



#generate_messages(DEPENDENCIES std_msgs)

add_executable(message_sys_send_drake_ros send_msg_demo.cpp)
target_link_libraries(message_sys_send_drake_ros
        ${catkin_LIBRARIES}
        drakeCommon
        drakeSystemFramework
        drakeSystemAnalysis
        drakeSystemPrimitives)
add_executable(message_sys_sub_drake_ros sub_msg_demo.cpp)
target_link_libraries(message_sys_sub_drake_ros
        ${catkin_LIBRARIES}
        drakeCommon
        drakeSystemFramework
        drakeSystemAnalysis
        drakeSystemPrimitives)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

