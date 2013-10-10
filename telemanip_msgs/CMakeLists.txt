cmake_minimum_required(VERSION 2.8.6)

project(telemanip_msgs)

find_package(catkin REQUIRED 
  COMPONENTS cartesian_trajectory_msgs message_generation)

add_message_files(FILES
  TelemanipCommand.msg
  )

generate_messages(
  DEPENDENCIES
  cartesian_trajectory_msgs
  )


catkin_package(
  CATKIN_DEPENDS cartesian_trajectory_msgs message_runtime)


