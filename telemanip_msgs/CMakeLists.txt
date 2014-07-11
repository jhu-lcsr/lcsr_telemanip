cmake_minimum_required(VERSION 2.8.6)

project(telemanip_msgs)

find_package(catkin REQUIRED 
  COMPONENTS std_msgs cartesian_trajectory_msgs geometry_msgs message_generation)

add_message_files(FILES
  TelemanipCommand.msg
  Inertia.msg
  InertiaStamped.msg
  AttachedInertia.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  cartesian_trajectory_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs cartesian_trajectory_msgs message_runtime)


