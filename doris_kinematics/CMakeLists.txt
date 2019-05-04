cmake_minimum_required(VERSION 2.8.3)
project(doris_kinematics)

find_package(catkin REQUIRED
  rospy
  std_msgs
  inno_arm_control_msgs
  message_generation
  message_runtime
)

generate_messages(
   DEPENDENCIES
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
