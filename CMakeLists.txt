cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(StateMachine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dji_sdk
  std_msgs
#  message_generation
  geometry_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Position.msg
#  Attitude.msg
#  Num.msg
#  MonoMsg.msg
#)


#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)


catkin_package(
   CATKIN_DEPENDS 
#   message_runtime 
   roscpp 
   std_msgs
   rospy
   dji_sdk
   DEPENDS 
   system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
  "/home/ubuntu/Agriculture/devel/include"
)

add_executable(StateMachineNode
src/StateMachine.cpp
src/Functions.cpp
src/SimpleGPIO.cpp
)
target_link_libraries(StateMachineNode ${catkin_LIBRARIES})
add_executable(testland
src/testland.cpp
src/Functions.cpp
src/SimpleGPIO.cpp)
target_link_libraries(testland ${catkin_LIBRARIES})
