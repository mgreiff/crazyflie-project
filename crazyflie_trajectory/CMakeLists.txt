cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_trajectory)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   GenericLogData.msg
   SynchronizationPacket.msg
   TrajectoryPacket.msg
   PointPacket.msg
   #FullControl.msg
 )

add_service_files(
  FILES
  UpdateParams.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
