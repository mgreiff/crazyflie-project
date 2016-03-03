cmake_minimum_required(VERSION 2.8.3)
project(crazy_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   Num.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
