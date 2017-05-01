cmake_minimum_required(VERSION 2.8.3)
project(base_module_follow_human)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leg_detector
  move_base_msgs
  people_msgs
  rospy
  #tf2_geometry_msgs
  tf2_ros
  actionlib_msgs
)

add_action_files(
  FILES
  FollowHuman.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
